/**
 * @file    motor_mgr.c
 * @brief   16 轴电机管理（虚拟总线上已经跑通，换真实 CAN 只改 can_bus 后端）
 */
#include <string.h>

#include "main.h"
#include "motor_mgr.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "can_bus.h"

static motor_t           s_motors[MOTOR_COUNT];
static motor_disc_t      s_disc[32];
static uint8_t           s_disc_count;
static uint8_t           s_count = MOTOR_COUNT;
static uint8_t           s_sniff;
static uint8_t           s_timeout_en = 1;
static uint8_t           s_warned[128];     /* 每个未知 ID 只报一次 */
static uint32_t          s_unknown_frames;
#define SNIFF_Q_LEN      16U
static rs_can_frame_t    s_sniff_q[SNIFF_Q_LEN];
static volatile uint8_t  s_sniff_head;
static volatile uint8_t  s_sniff_tail;   /* 当前表里在用的电机数量 */
static const rs_limits_t *s_lim;
static uint32_t           s_last_ctrl_ms;

static motor_t *by_index(uint8_t idx)
{
    return (idx < MOTOR_COUNT) ? &s_motors[idx] : NULL;
}

static motor_t *by_id(uint8_t id)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (s_motors[i].id == id) {
            return &s_motors[i];
        }
    }
    return NULL;
}

void MotorMgr_Init(void)
{
    memset(s_motors, 0, sizeof(s_motors));
    s_lim = rs_limits_for_model("rs-02");
    if (s_lim == NULL) {
        s_lim = rs_limits_for_model("rs-00");
    }

    /* 固定电机表（实测）：11..18 在 CAN2，21..28 在 CAN1 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (i < 8U) {
            s_motors[i].id  = (uint8_t)(11U + i);   /* 11..18 */
            s_motors[i].bus = 1U;                   /* CAN2 */
        } else {
            s_motors[i].id  = (uint8_t)(21U + (i - 8U));  /* 21..28 */
            s_motors[i].bus = 0U;                         /* CAN1 */
        }
        s_motors[i].lim = rs_limits_for_can_id(s_motors[i].id);
        if (s_motors[i].lim == NULL) {
            s_motors[i].lim = s_lim;
        }
    }
    s_last_ctrl_ms = HAL_GetTick();
    CAN_Bus_Init(MotorMgr_OnRx, NULL);
}

void MotorMgr_OnRx(const rs_can_frame_t *frame, void *user)
{
    rs_parsed_frame_t parsed;
    rs_status_t       st;
    motor_t          *m;

    (void)user;

    if (s_sniff) {                      /* 只入队，打印放到主循环 */
        uint8_t next = (uint8_t)((s_sniff_head + 1U) % SNIFF_Q_LEN);
        if (next != s_sniff_tail) {
            s_sniff_q[s_sniff_head] = *frame;
            s_sniff_head = next;
        }
    }

    if (!rs_parse_frame(frame, &parsed)) {
        return;
    }
    if (rs_is_status_like(parsed.comm_type)) {
        const uint8_t id = (uint8_t)(parsed.extra_data & 0xFFU);

        m = by_id(id);
        if (m == NULL) {
            s_unknown_frames++;
            if (id < 128U && !s_warned[id]) {
                s_warned[id] = 1U;
                printf("[WARN] 收到表外电机 id=%u 的状态帧（表内只有 11-18 / 21-28）\r\n",
                       (unsigned)id);
            }
            return;
        }
        if (!rs_decode_status(&parsed, (m->lim != NULL) ? m->lim : s_lim, &st)) {
            return;
        }
        m->pos        = st.position;
        m->vel        = st.velocity;
        m->torque     = st.torque;
        m->temp       = st.temperature;
        m->fault_bits = st.fault_bits;
        m->online     = 1;
        m->last_rx_ms = HAL_GetTick();
        /* 只有硬故障（欠压/栅驱/过温/编码器/堵转）才自动失能；
         * bit5 = 未标定，只提示不阻止控制（新电机常见）。 */
        if ((st.fault_bits & 0x1FU) != 0U) {
            m->enabled = 0;
        }
        return;
    }

    if (parsed.comm_type == RS_COMM_GET_ID) {
        uint8_t id = (uint8_t)(parsed.extra_data & 0xFFU);

        for (uint8_t i = 0; i < s_disc_count; i++) {
            if (s_disc[i].id == id) {
                return;                       /* 重复应答 */
            }
        }
        if (s_disc_count < (uint8_t)(sizeof(s_disc) / sizeof(s_disc[0]))) {
            s_disc[s_disc_count].id  = id;
            s_disc[s_disc_count].bus = frame->bus;
            for (uint8_t k = 0; k < 8; k++) {
                s_disc[s_disc_count].uid[k] = frame->data[k];
            }
            s_disc_count++;
        }
        return;
    }

    if (parsed.comm_type == RS_COMM_READ_PARAM) {
        rs_read_reply_t reply;

        /* 实测回包：id = 0x11 <电机ID> <主机号 0xFD>
         * 即电机 ID 在 extra_data 低字节，device 字节是主机号 */
        m = by_id((uint8_t)(parsed.extra_data & 0xFFU));
        if (m == NULL) {
            m = by_id(parsed.device_id);
        }
        if (m == NULL) {
            uint8_t uid = (uint8_t)(parsed.extra_data & 0xFFU);
            s_unknown_frames++;
            if (uid < 128U && !s_warned[uid]) {
                s_warned[uid] = 1U;
                printf("[WARN] 收到表外电机 id=%u 的读参数回包（表内只有 11-18 / 21-28）\r\n",
                       (unsigned)uid);
            }
            return;
        }
        if (!rs_decode_read_param(&parsed, &reply)) {
            return;
        }
        if (reply.param_id == RS_PARAM_MECH_POS) {
            m->pos = (double)rs_read_param_f32(&reply);
        } else if (reply.param_id == RS_PARAM_RUN_MODE) {
            m->enabled = (rs_read_param_u8(&reply) != 0U) ? 1U : 0U;
            printf("  id=%u run_mode=%u (0=MIT 5=CSP)\r\n",
                   (unsigned)m->id, (unsigned)rs_read_param_u8(&reply));
        } else if (reply.param_id == RS_PARAM_LOC_REF) {
            printf("  id=%u loc_ref=%.3f\r\n",
                   (unsigned)m->id, (double)rs_read_param_f32(&reply));
        }
        m->online     = 1;
        m->last_rx_ms = HAL_GetTick();
    }
}

static void sniff_flush(void)
{
    while (s_sniff_tail != s_sniff_head) {
        const rs_can_frame_t *f = &s_sniff_q[s_sniff_tail];
        printf("[RX C%u] id=0x%08lX dlc=%u  %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               (unsigned)(f->bus + 1U), (unsigned long)f->id, (unsigned)f->dlc,
               f->data[0], f->data[1], f->data[2], f->data[3],
               f->data[4], f->data[5], f->data[6], f->data[7]);
        s_sniff_tail = (uint8_t)((s_sniff_tail + 1U) % SNIFF_Q_LEN);
    }
}

void MotorMgr_Tick(uint32_t now_ms)
{
    rs_can_frame_t f;

    if (s_sniff) {
        sniff_flush();
    }

    if ((now_ms - s_last_ctrl_ms) < 10U) {
        return;
    }
    s_last_ctrl_ms = now_ms;

    for (uint8_t i = 0; i < s_count; i++) {
        motor_t *m = &s_motors[i];

        /* 超时：只有在"被使能、正在被轮询"时才算；失能状态本来就不发帧 */
        if (s_timeout_en && m->enabled && m->online &&
            (now_ms - m->last_rx_ms) > MOTOR_TIMEOUT_MS) {
            m->online  = 0;
            m->enabled = 0;
            m->vel     = 0.0;
            m->torque  = 0.0;
        }
    }
}

void MotorMgr_SendMit(uint8_t idx, double pos, double vel,
                      double kp, double kd, double trq)
{
    motor_t       *m = by_index(idx);
    rs_can_frame_t f;

    if (m == NULL || !m->online || !m->enabled) {
        return;
    }
    rs_encode_mit(&f, m->id, (m->lim != NULL) ? m->lim : s_lim,
                  pos, vel, kp, kd, trq);
    CAN_Bus_Send(m->bus, &f);
}

void MotorMgr_RequestPos(void)
{
    rs_can_frame_t f;

    for (uint8_t i = 0; i < s_count; i++) {
        rs_encode_read_param(&f, s_motors[i].id, RS_HOST_ID_DEFAULT,
                             RS_PARAM_MECH_POS);
        CAN_Bus_Send(s_motors[i].bus, &f);
    }
}

void MotorMgr_RequestParamAll(uint16_t pid)
{
    rs_can_frame_t f;

    for (uint8_t i = 0; i < s_count; i++) {
        rs_encode_read_param(&f, s_motors[i].id, RS_HOST_ID_DEFAULT, pid);
        CAN_Bus_Send(s_motors[i].bus, &f);
    }
}

void MotorMgr_Scan(void)
{
    rs_can_frame_t f;

    for (uint8_t i = 0; i < s_count; i++) {
        rs_encode_read_param(&f, s_motors[i].id, RS_HOST_ID_DEFAULT,
                             RS_PARAM_MECH_POS);
        CAN_Bus_Send(s_motors[i].bus, &f);
    }
    HAL_Delay(30);              /* 等应答进来（调用方在命令行，可以阻塞） */
}

void MotorMgr_SetTimeoutEnabled(uint8_t on)
{
    s_timeout_en = (uint8_t)(on ? 1 : 0);
}

void MotorMgr_SetSniff(uint8_t on)
{
    s_sniff = (uint8_t)(on ? 1 : 0);
}

void MotorMgr_DiscClear(void)
{
    s_disc_count = 0;
}

uint8_t MotorMgr_DiscCount(void)
{
    return s_disc_count;
}

const motor_disc_t *MotorMgr_DiscAt(uint8_t i)
{
    return (i < s_disc_count) ? &s_disc[i] : NULL;
}

void MotorMgr_Discover(uint8_t from, uint8_t to)
{
    rs_can_frame_t f;

    MotorMgr_DiscClear();
    if (from < 1U) { from = 1U; }
    if (to > 127U) { to = 127U; }

    for (uint8_t id = from; id <= to; id++) {
        /* 两路都发一遍（电机可能挂在任一总线） */
        rs_encode_control(&f, RS_COMM_GET_ID, id, RS_HOST_ID_DEFAULT, NULL, 0);
        f.dlc = 0;
        CAN_Bus_Send(0, &f);
        CAN_Bus_Send(1, &f);
        HAL_Delay(3);
    }
    HAL_Delay(60);            /* 收尾：等最后几个应答 */

    /* 按 ID 升序排一下，adopt 的顺序才稳定 */
    for (uint8_t i = 1; i < s_disc_count; i++) {
        motor_disc_t tmp = s_disc[i];
        int8_t j = (int8_t)(i - 1U);
        while (j >= 0 && s_disc[(uint8_t)j].id > tmp.id) {
            s_disc[(uint8_t)(j + 1)] = s_disc[(uint8_t)j];
            j--;
        }
        s_disc[(uint8_t)(j + 1)] = tmp;
    }
}

uint8_t MotorMgr_Adopt(void)
{
    uint8_t n = s_disc_count;

    if (n == 0U) {
        return 0;
    }
    if (n > MOTOR_COUNT) {
        n = MOTOR_COUNT;
    }

    memset(s_motors, 0, sizeof(s_motors));
    for (uint8_t i = 0; i < n; i++) {
        s_motors[i].id  = s_disc[i].id;
        s_motors[i].bus = s_disc[i].bus;
        s_motors[i].cmd_pos = 0.0;
        s_motors[i].lim = rs_limits_for_can_id(s_motors[i].id);
        if (s_motors[i].lim == NULL) {
            s_motors[i].lim = s_lim;
        }
    }
    s_count = n;
    return n;
}

void MotorMgr_PollZeroMit(void)
{
    rs_can_frame_t f;

    for (uint8_t i = 0; i < s_count; i++) {
        const rs_limits_t *lim = (s_motors[i].lim != NULL) ? s_motors[i].lim : s_lim;
        rs_encode_mit(&f, s_motors[i].id, lim, 0.0, 0.0, 0.0, 0.0, 0.0);
        CAN_Bus_Send(s_motors[i].bus, &f);
        HAL_Delay(1);
    }
    HAL_Delay(30);
}

void MotorMgr_SendParamU8(uint8_t idx, uint16_t pid, uint8_t val)
{
    motor_t       *m = by_index(idx);
    rs_can_frame_t f;

    if (m == NULL) {
        return;
    }
    rs_encode_write_param_u8(&f, m->id, RS_HOST_ID_DEFAULT, pid, val);
    CAN_Bus_Send(m->bus, &f);
}

void MotorMgr_SendParamF32(uint8_t idx, uint16_t pid, float val)
{
    motor_t       *m = by_index(idx);
    rs_can_frame_t f;

    if (m == NULL) {
        return;
    }
    rs_encode_write_param_float(&f, m->id, RS_HOST_ID_DEFAULT, pid, val);
    CAN_Bus_Send(m->bus, &f);
}

void MotorMgr_SendLocRef(uint8_t idx, float rad)
{
    MotorMgr_SendParamF32(idx, RS_PARAM_LOC_REF, rad);
}

void MotorMgr_SetRunModeAll(uint8_t run_mode, uint8_t reenable)
{
    for (uint8_t i = 0; i < s_count; i++) {
        MotorMgr_Enable(i, 0);
    }
    HAL_Delay(30);
    for (uint8_t i = 0; i < s_count; i++) {
        rs_can_frame_t f;

        rs_encode_write_param_u8(&f, s_motors[i].id, RS_HOST_ID_DEFAULT,
                                 RS_PARAM_RUN_MODE, run_mode);
        CAN_Bus_Send(s_motors[i].bus, &f);
        HAL_Delay(2);
    }
    HAL_Delay(30);
    if (reenable) {
        for (uint8_t i = 0; i < s_count; i++) {
            MotorMgr_Enable(i, 1);
            HAL_Delay(2);
        }
        HAL_Delay(20);
    }
}

/* 类型 6 必须带 1 字节负载 0x01（参考 lite_driver 的 set_zero 工具），
 * dlc=0 的空帧会被电机忽略 —— 这就是之前"归零没反应"的原因。 */
static void send_set_zero(uint8_t idx)
{
    static const uint8_t one = 1;
    rs_can_frame_t f;

    rs_encode_control(&f, RS_COMM_SET_ZERO, s_motors[idx].id,
                      RS_HOST_ID_DEFAULT, &one, 1);
    CAN_Bus_Send(s_motors[idx].bus, &f);
    s_motors[idx].cmd_pos = 0.0;
}

int MotorMgr_ZeroAllVerify(void)
{
    int bad = 0;

    /* 1) 必须失能后才能写零 */
    for (uint8_t i = 0; i < s_count; i++) {
        MotorMgr_Enable(i, 0);
    }
    HAL_Delay(80);

    printf("写零中（类型 6，payload=0x01）...\r\n");
    for (uint8_t i = 0; i < s_count; i++) {
        send_set_zero(i);          /* 连发两次，防偶发丢帧 */
        HAL_Delay(3);
        send_set_zero(i);
        HAL_Delay(3);
    }
    HAL_Delay(300);

    /* 2) 回读校验 */
    MotorMgr_Scan();
    printf("归零结果（|pos| <= 1° 视为成功）：\r\n");
    for (uint8_t i = 0; i < s_count; i++) {
        const motor_t *m = MotorMgr_Get(i);
        double deg = (m != NULL) ? m->pos * 57.29578 : 0.0;
        int ok = (fabs(deg) <= 1.0) && (m != NULL) && m->online;

        if (!ok) {
            bad++;
        }
        printf("  #%2u id=%2u %7.2f°  %s%s", (unsigned)(i + 1U),
               (unsigned)((m != NULL) ? m->id : 0U), deg,
               ok ? "OK" : "!! 超差",
               ((i % 2U) == 1U) ? "\r\n" : "    ");
    }
    if ((s_count % 2U) != 0U) {
        printf("\r\n");
    }
    printf("归零完成：%u/%u 台成功\r\n",
           (unsigned)(s_count - (uint8_t)bad), (unsigned)s_count);
    return bad;
}

void MotorMgr_Enable(uint8_t idx, uint8_t on)
{
    motor_t       *m = by_index(idx);
    rs_can_frame_t f;

    if (m == NULL) {
        return;
    }
    if (on) {
        m->cmd_pos = m->pos;
        m->enabled = 1;                 /* 先置位；下一帧状态确认 */
        m->last_rx_ms = HAL_GetTick();
        rs_encode_control(&f, RS_COMM_ENABLE, m->id, RS_HOST_ID_DEFAULT, NULL, 0);
    } else {
        rs_encode_control(&f, RS_COMM_DISABLE, m->id, RS_HOST_ID_DEFAULT, NULL, 0);
        m->enabled = 0;
    }
    CAN_Bus_Send(m->bus, &f);
}

void MotorMgr_EnableAll(uint8_t on)
{
    for (uint8_t i = 0; i < s_count; i++) {
        MotorMgr_Enable(i, on);
    }
}

void MotorMgr_ZeroAll(void)
{
    for (uint8_t i = 0; i < s_count; i++) {
        send_set_zero(i);
    }
}

void MotorMgr_Hold(uint8_t idx)
{
    motor_t *m = by_index(idx);

    if (m != NULL) {
        m->cmd_pos = m->pos;
    }
}

void MotorMgr_MoveTo(uint8_t idx, double pos_rad)
{
    motor_t *m = by_index(idx);

    if (m != NULL) {
        m->cmd_pos = pos_rad;
    }
}

const motor_t *MotorMgr_Get(uint8_t idx)
{
    return (idx < MOTOR_COUNT) ? &s_motors[idx] : NULL;
}

uint32_t MotorMgr_UnknownCount(void)
{
    return s_unknown_frames;
}

uint8_t MotorMgr_Count(void)
{
    return s_count;
}

uint8_t MotorMgr_FirstId(void)
{
    return (s_count > 0U) ? s_motors[0].id : 0U;
}

uint8_t MotorMgr_LastId(void)
{
    return (s_count > 0U) ? s_motors[s_count - 1U].id : 0U;
}

uint8_t MotorMgr_OnlineCount(void)
{
    uint8_t n = 0;

    for (uint8_t i = 0; i < s_count; i++) {
        if (s_motors[i].online) {
            n++;
        }
    }
    return n;
}

uint8_t MotorMgr_EnabledCount(void)
{
    uint8_t n = 0;

    for (uint8_t i = 0; i < s_count; i++) {
        if (s_motors[i].enabled) {
            n++;
        }
    }
    return n;
}
