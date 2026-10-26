/**
 * @file    traj.c
 * @brief   录制/回放：边录边写 W25Q64，回放按扇区缓存读
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "traj.h"
#include "motor_mgr.h"
#include "rs_protocol.h"
#include "bsp_w25q.h"
#include "mit_ctrl.h"

#define CSP_SPEED       2.0
#define CSP_ACC         20.0
/* CSP 每轴安全扭矩（N·m），取自 lite.ros2_control.xacro 的 max_safe_torque：
 *   11/21 rs-02 7.0 | 12..14/22..24 rs-00 7.0/2.5/4.0
 *   15..17/25..27 rs-05 1.5/1.8/1.8 | 18/28 夹爪 rs-05 0.25
 * 下发：limit_cur[Apk] = max_safe_torque / kt * √2 */
static const double kCspSafeTorque[TRAJ_AXES] = {
    7.0, 7.0, 2.5, 4.0, 1.5, 1.8, 1.8, 0.25,
    7.0, 7.0, 2.5, 4.0, 1.5, 1.8, 1.8, 0.25,
};
static const char *const kCspModel[TRAJ_AXES] = {
    "rs-02", "rs-00", "rs-00", "rs-00", "rs-05", "rs-05", "rs-05", "rs-05",
    "rs-02", "rs-00", "rs-00", "rs-00", "rs-05", "rs-05", "rs-05", "rs-05",
};

#define FLASH_BASE          0x000000U
#define FLASH_SLOT_SIZE     0x40000U        /* 每槽 256KB */
#define FLASH_DATA_OFF      0x1000U         /* 头占一个扇区 */
#define FLASH_MAGIC         0x4152544CU     /* "LTRA" */

typedef struct {
    uint32_t magic;
    uint16_t frames;
    uint16_t reserved;
    uint32_t crc;
    uint32_t duration_ms;
} traj_hdr_t;

static traj_meta_t s_meta[TRAJ_SLOTS];

/* 录制状态 */
static uint8_t  s_recording;
static uint8_t  s_rec_slot;
static uint32_t s_rec_last_ms;
static uint16_t s_rec_frames;
static uint32_t s_rec_crc;
static uint8_t  s_page[W25Q_PAGE_SIZE];
static uint16_t s_page_len;
static uint32_t s_page_addr;

/* 回放状态 */
static uint8_t  s_playing;
static uint8_t  s_play_slot;
static uint8_t  s_play_loops;
static uint8_t  s_play_loop_done;
static uint16_t s_play_gap_ms = 0;      /* 每轮之间的停留时间 */
static uint8_t  s_play_waiting;         /* 1 = 正在间隔停留 */
static uint32_t s_play_wait_until;
static double   s_play_pos;
static uint32_t s_play_last_ms;

/* 读缓存（一个扇区） */
static uint8_t  s_cache[W25Q_SECTOR_SIZE];
static uint32_t s_cache_base;
static uint8_t  s_cache_valid;

/* 回放平滑：一阶低通（时间常数 s_smooth_tau_ms，0 = 关闭） */
static uint16_t s_smooth_tau_ms = 40;

/* 回放方式：0 = CSP（loc_ref），1 = MIT（kp/kd 位置跟踪） */
static uint8_t  s_play_mit;
static double   s_filt[TRAJ_AXES];
static uint8_t  s_filt_init;

static uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t len)
{
    while (len--) {
        crc ^= *p++;
        for (uint8_t k = 0; k < 8U; k++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
        }
    }
    return crc;
}

static uint32_t slot_addr(uint8_t slot)
{
    return FLASH_BASE + (uint32_t)slot * FLASH_SLOT_SIZE;
}

int Traj_FlashReady(void)
{
    return W25Q_IsReady();
}

/* ---------------- 读帧（带扇区缓存） ---------------- */
static int read_frame(uint8_t slot, uint16_t frame, int16_t out[TRAJ_AXES])
{
    uint32_t off = slot_addr(slot) + FLASH_DATA_OFF + (uint32_t)frame * TRAJ_AXES * 2U;
    uint32_t base = off - (off % W25Q_SECTOR_SIZE);
    uint32_t rel  = off - base;

    if (!s_cache_valid || s_cache_base != base) {
        if (!W25Q_Read(base, s_cache, W25Q_SECTOR_SIZE)) {
            return 0;
        }
        s_cache_base  = base;
        s_cache_valid = 1;
    }
    if ((rel + TRAJ_AXES * 2U) > W25Q_SECTOR_SIZE) {
        return 0;                       /* 跨扇区（帧 32B 对齐，正常不会发生） */
    }
    memcpy(out, &s_cache[rel], TRAJ_AXES * 2U);
    return 1;
}

void Traj_Init(void)
{
    memset(s_meta, 0, sizeof(s_meta));
    s_recording = 0;
    s_playing   = 0;
    s_cache_valid = 0;
}

const traj_meta_t *Traj_GetMeta(uint8_t slot)
{
    return (slot < TRAJ_SLOTS) ? &s_meta[slot] : NULL;
}

int16_t Traj_GetSample(uint8_t slot, uint16_t frame, uint8_t axis)
{
    int16_t buf[TRAJ_AXES];

    if (axis >= TRAJ_AXES || !read_frame(slot, frame, buf)) {
        return 0;
    }
    return buf[axis];
}

void Traj_Clear(uint8_t slot)
{
    if (slot < TRAJ_SLOTS) {
        memset(&s_meta[slot], 0, sizeof(s_meta[slot]));
    }
}

/* ---------------- 录制（边录边写 Flash） ---------------- */
static void flush_page(void)
{
    if (s_page_len == 0U) {
        return;
    }
    if (!W25Q_Write(s_page_addr, s_page, s_page_len)) {
        printf("!! Flash 写入失败 0x%06lX\r\n", (unsigned long)s_page_addr);
    }
    s_page_addr += s_page_len;
    s_page_len = 0;
}

int Traj_StartRecord(uint8_t slot)
{
    uint32_t base;

    if (slot >= TRAJ_SLOTS) {
        return -1;
    }
    if (s_playing) {
        printf("回放中，不能录制\r\n");
        return -2;
    }
    /* 录制 = 零力矩示教：切 ZERO（零力矩 + 逐轴阻尼），不带重力补偿 */
    if (MitCtrl_Mode() != MIT_MODE_ZERO) {
        printf("进入零力矩模式（ZERO，零力矩 + 阻尼）...\r\n");
        MitCtrl_SetMode(MIT_MODE_ZERO);
        HAL_Delay(300);
    }
    MotorMgr_Scan();
    if (MotorMgr_OnlineCount() == 0U) {
        printf("录制拒绝：没有在线电机（先 motor status 确认 16/16）\r\n");
        return -5;
    }
    {
        const motor_t *m0 = MotorMgr_Get(0);
        printf("当前姿态确认：#1=%.1f° #2=%.1f° #3=%.1f°\r\n",
               (m0 != NULL) ? m0->pos * 57.29578 : 0.0,
               (MotorMgr_Get(1) != NULL) ? MotorMgr_Get(1)->pos * 57.29578 : 0.0,
               (MotorMgr_Get(2) != NULL) ? MotorMgr_Get(2)->pos * 57.29578 : 0.0);
    }

    base = slot_addr(slot);
    printf("擦除槽位 %u（256KB，约 2s）...\r\n", (unsigned)(slot + 1U));
    for (uint32_t off = 0; off < FLASH_DATA_OFF + TRAJ_MAX_FRAMES * TRAJ_AXES * 2U;
         off += W25Q_SECTOR_SIZE) {
        if (!W25Q_EraseSector(base + off)) {
            printf("!! 擦除失败 0x%06lX\r\n", (unsigned long)(base + off));
            return -6;
        }
    }

    memset(&s_meta[slot], 0, sizeof(s_meta[slot]));
    s_cache_valid = 0;

    s_rec_slot    = slot;
    s_rec_frames  = 0;
    s_rec_crc     = 0xFFFFFFFFU;
    s_page_len    = 0;
    s_page_addr   = base + FLASH_DATA_OFF;
    s_rec_last_ms = HAL_GetTick();
    s_recording   = 1;

    printf("开始录制 → 槽位 %u（最长 %u 帧 = %u ms），边录边写 Flash\r\n",
           (unsigned)(slot + 1U), (unsigned)TRAJ_MAX_FRAMES,
           (unsigned)(TRAJ_MAX_FRAMES * 10U));
    return 0;
}

int Traj_RecordTick(uint32_t now_ms)
{
    int16_t frame[TRAJ_AXES];

    if (!s_recording) {
        return 0;
    }
    if ((now_ms - s_rec_last_ms) < 10U) {
        return 1;
    }
    s_rec_last_ms = now_ms;

    if (s_rec_frames >= TRAJ_MAX_FRAMES) {
        Traj_StopRecord();
        printf("录制结束：已满 %u 帧\r\n", (unsigned)TRAJ_MAX_FRAMES);
        return 0;
    }

    /* 状态帧只在轮询时回来：录制期间每拍主动请求一次位置 */
    MotorMgr_RequestPos();

    for (uint8_t i = 0; i < TRAJ_AXES; i++) {
        const motor_t *m = MotorMgr_Get(i);
        double q = (m != NULL) ? m->pos : 0.0;
        double v = q * TRAJ_SCALE;

        if (v > 32767.0)  { v = 32767.0; }
        if (v < -32768.0) { v = -32768.0; }
        frame[i] = (int16_t)lround(v);
    }

    s_rec_crc = crc32_update(s_rec_crc, (const uint8_t *)frame, sizeof(frame));
    memcpy(&s_page[s_page_len], frame, sizeof(frame));
    s_page_len = (uint16_t)(s_page_len + sizeof(frame));
    if (s_page_len >= W25Q_PAGE_SIZE) {
        flush_page();
    }

    s_rec_frames++;
    s_meta[s_rec_slot].frames      = s_rec_frames;
    s_meta[s_rec_slot].duration_ms = (uint32_t)s_rec_frames * 10U;
    return 1;
}

void Traj_StopRecord(void)
{
    traj_hdr_t hdr;

    if (!s_recording) {
        return;
    }
    s_recording = 0;
    flush_page();

    hdr.magic       = FLASH_MAGIC;
    hdr.frames      = s_rec_frames;
    hdr.reserved    = 0;
    hdr.crc         = ~s_rec_crc;
    hdr.duration_ms = (uint32_t)s_rec_frames * 10U;
    (void)W25Q_EraseSector(slot_addr(s_rec_slot));
    (void)W25Q_Write(slot_addr(s_rec_slot), (const uint8_t *)&hdr, sizeof(hdr));

    s_meta[s_rec_slot].valid       = (s_rec_frames > 1U) ? 1U : 0U;
    s_meta[s_rec_slot].frames      = s_rec_frames;
    s_meta[s_rec_slot].duration_ms = hdr.duration_ms;
    s_cache_valid = 0;

    printf("停止录制：槽位 %u，%u 帧，%u ms，已写入 Flash\r\n",
           (unsigned)(s_rec_slot + 1U), (unsigned)s_rec_frames,
           (unsigned)hdr.duration_ms);
    printf("当前模式：%s（可继续 rec / ctrl hold / play %u）\r\n",
           MitCtrl_ModeName(MitCtrl_Mode()), (unsigned)(s_rec_slot + 1U));
}

uint8_t Traj_Recording(void)
{
    return s_recording;
}

/* ---------------- 回放（CSP） ---------------- */
static void csp_setup(void)
{
    for (uint8_t i = 0; i < MotorMgr_Count(); i++) {
        MotorMgr_Enable(i, 0);
        HAL_Delay(1);
    }
    HAL_Delay(30);
    for (uint8_t i = 0; i < MotorMgr_Count(); i++) {
        MotorMgr_SendParamU8(i, RS_PARAM_RUN_MODE, RS_RUN_MODE_CSP);
        HAL_Delay(2);
        MotorMgr_SendParamF32(i, RS_PARAM_LIMIT_SPD, (float)CSP_SPEED);
        HAL_Delay(2);
        MotorMgr_SendParamF32(i, RS_PARAM_ACC_RAD, (float)CSP_ACC);
        HAL_Delay(2);
        {
            double kt  = rs_torque_constant_for_model(kCspModel[i]);
            double lim = rs_csp_limit_current_from_torque(kCspSafeTorque[i], kt);

            MotorMgr_SendParamF32(i, RS_PARAM_CURRENT_LIMIT, (float)lim);
            (void)kt;
        }
        HAL_Delay(2);
    }
    HAL_Delay(30);
    for (uint8_t i = 0; i < MotorMgr_Count(); i++) {
        MotorMgr_Enable(i, 1);
        HAL_Delay(2);
    }
    HAL_Delay(100);
    printf("CSP 已下发（先失能 → run_mode=5 → 限幅 → 使能）\r\n");
}

int Traj_StartPlay(uint8_t slot, uint8_t loops)
{
    if (slot >= TRAJ_SLOTS || !s_meta[slot].valid) {
        printf("槽位 %u 没有有效轨迹\r\n", (unsigned)(slot + 1U));
        return -1;
    }
    if (s_recording) {
        printf("录制中，不能回放\r\n");
        return -2;
    }
    if (s_playing) {
        printf("已经在回放中（stop 先停）\r\n");
        return -4;
    }

    MotorMgr_SetTimeoutEnabled(0);
    if (s_play_mit) {
        /* MIT 位置跟踪：切回 MIT 并重新使能（先失能才能写 run_mode） */
        MotorMgr_SetRunModeAll(RS_RUN_MODE_MIT, 1);
        printf("回放方式：MIT（kp=%.0f kd=%.2f）\r\n",
               MitCtrl_Kp(), MitCtrl_Kd());
    } else {
        csp_setup();
        printf("回放方式：CSP（loc_ref + 速度/加速度限幅）\r\n");
    }

    s_play_slot      = slot;
    s_play_loops     = loops;      /* 0 = 无限次，直到 stop */
    s_play_loop_done = 0;
    s_play_waiting   = 0;
    s_play_pos       = 0.0;
    s_play_last_ms   = HAL_GetTick();
    s_filt_init      = 0;
    s_playing        = 1;
    printf("开始回放：槽位 %u，%u 帧，%s，间隔 %ums，平滑 τ=%ums\r\n",
           (unsigned)(slot + 1U), (unsigned)s_meta[slot].frames,
           (s_play_loops == 0U) ? "无限次" : "有限次",
           (unsigned)s_play_gap_ms, (unsigned)s_smooth_tau_ms);
    if (s_play_loops != 0U) {
        printf("  轮数 %u，间隔 %u ms，单轮 %u ms\r\n",
               (unsigned)s_play_loops, (unsigned)s_play_gap_ms,
               (unsigned)s_meta[slot].duration_ms);
    }
    return 0;
}

void Traj_PlayTick(uint32_t now_ms)
{
    uint16_t f0, f1;
    double   a;
    int16_t  s0[TRAJ_AXES], s1[TRAJ_AXES];

    if (!s_playing) {
        return;
    }
    if ((now_ms - s_play_last_ms) < 10U) {
        return;
    }
    s_play_last_ms = now_ms;

    f0 = (uint16_t)s_play_pos;
    f1 = (uint16_t)(f0 + 1U);
    if (s_play_waiting) {
        if ((int32_t)(now_ms - s_play_wait_until) < 0) {
            return;                     /* 间隔中：保持最后一条指令 */
        }
        s_play_waiting = 0;
    }

    if (f0 >= s_meta[s_play_slot].frames) {
        s_play_loop_done++;

        if (s_play_loops != 0U && s_play_loop_done >= s_play_loops) {
            Traj_StopPlay();
            printf("回放完成（%u 轮）\r\n", (unsigned)s_play_loop_done);
            return;
        }
        if (s_play_gap_ms > 0U) {
            s_play_waiting    = 1;
            s_play_wait_until = now_ms + s_play_gap_ms;
        }
        s_play_pos = 0.0;
        f0 = 0;
        f1 = 1;
        if (s_play_waiting) {
            printf("第 %u 轮结束，间隔 %ums\r\n",
                   (unsigned)s_play_loop_done, (unsigned)s_play_gap_ms);
            return;
        }
    }
    if (f1 >= s_meta[s_play_slot].frames) {
        f1 = (uint16_t)(s_meta[s_play_slot].frames - 1U);
        a  = 0.0;
    } else {
        a = s_play_pos - (double)f0;
    }

    if (!read_frame(s_play_slot, f0, s0)) {
        Traj_StopPlay();
        printf("回放中止：Flash 读失败\r\n");
        return;
    }
    if (!read_frame(s_play_slot, f1, s1)) {
        memcpy(s1, s0, sizeof(s1));
    }

    for (uint8_t i = 0; i < TRAJ_AXES; i++) {
        double v0 = (double)s0[i] / TRAJ_SCALE;
        double v1 = (double)s1[i] / TRAJ_SCALE;
        double q  = v0 + a * (v1 - v0);

        if (s_smooth_tau_ms > 0U) {
            /* 一阶低通：alpha = dt / (tau + dt)，dt = 10ms */
            double alpha = 10.0 / ((double)s_smooth_tau_ms + 10.0);

            if (!s_filt_init) {
                s_filt[i] = q;
            }
            s_filt[i] += alpha * (q - s_filt[i]);
            q = s_filt[i];
        }
        if (s_play_mit) {
            MotorMgr_SendMit(i, q, 0.0, MitCtrl_Kp(), MitCtrl_Kd(), 0.0);
        } else {
            MotorMgr_SendLocRef(i, (float)q);
        }
    }
    s_filt_init = 1;
    s_play_pos += 1.0;
}

void Traj_SetPlayGap(uint16_t gap_ms)
{
    s_play_gap_ms = gap_ms;
}

uint16_t Traj_GetPlayGap(void)
{
    return s_play_gap_ms;
}

void Traj_SetPlayMode(uint8_t mit)
{
    s_play_mit = (uint8_t)(mit ? 1 : 0);
}

uint8_t Traj_GetPlayMode(void)
{
    return s_play_mit;
}

void Traj_SetSmooth(uint16_t tau_ms)
{
    s_smooth_tau_ms = tau_ms;
}

uint16_t Traj_GetSmooth(void)
{
    return s_smooth_tau_ms;
}

void Traj_StopPlay(void)
{
    s_playing = 0;
    MotorMgr_SetTimeoutEnabled(1);

    /* 回放结束 → 切到 ZERO（零力矩 + 逐轴阻尼），电机保持使能但松劲。
     * MitCtrl_SetMode 内部会先失能再写 run_mode 回到 MIT，最后重新使能。 */
    MitCtrl_SetMode(MIT_MODE_ZERO);
    printf("回放结束 → ZERO 模式（零力矩 + 阻尼）\r\n");
}

uint8_t Traj_Playing(void)
{
    return s_playing;
}

/* ---------------- Flash 兼容接口 ---------------- */
int Traj_SaveToFlash(uint8_t slot)
{
    if (slot >= TRAJ_SLOTS || !s_meta[slot].valid) {
        printf("槽位 %u 为空\r\n", (unsigned)(slot + 1U));
        return -2;
    }
    printf("录制时已直接写入 Flash，无需 save（槽 %u）\r\n", (unsigned)(slot + 1U));
    return 0;
}

int Traj_LoadFromFlash(uint8_t slot)
{
    traj_hdr_t hdr;

    if (slot >= TRAJ_SLOTS) {
        return -1;
    }
    if (!W25Q_Read(slot_addr(slot), (uint8_t *)&hdr, sizeof(hdr))) {
        return -2;
    }
    if (hdr.magic != FLASH_MAGIC || hdr.frames == 0U || hdr.frames > TRAJ_MAX_FRAMES) {
        return -3;
    }
    s_meta[slot].valid       = 1;
    s_meta[slot].frames      = hdr.frames;
    s_meta[slot].duration_ms = hdr.duration_ms;
    return 0;
}

int Traj_LoadAll(void)
{
    int n = 0;

    s_cache_valid = 0;
    for (uint8_t i = 0; i < TRAJ_SLOTS; i++) {
        if (Traj_LoadFromFlash(i) == 0) {
            n++;
        }
    }
    return n;
}
