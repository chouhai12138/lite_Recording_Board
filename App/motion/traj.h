/**
 * @file    traj.h
 * @brief   录制/回放（A6b：边录边存 W25Q64，最长 60 s）
 *
 * 存储（W25Q64，每槽 256KB）：
 *   槽内 0x0000: 头（magic/帧数/CRC32/时长）
 *   槽内 0x1000: 数据，6000 帧 × 16 轴 × int16（0.0005 rad/LSB）= 187.5KB
 */
#ifndef __TRAJ_H__
#define __TRAJ_H__

#include <stdint.h>

#define TRAJ_SLOTS      5
#define TRAJ_MAX_FRAMES 6000        /* 60 s @100Hz */
#define TRAJ_AXES       16
#define TRAJ_SCALE      2000.0

typedef struct {
    uint8_t  valid;
    uint16_t frames;
    uint32_t duration_ms;
} traj_meta_t;

void Traj_Init(void);

int  Traj_StartRecord(uint8_t slot);
int  Traj_RecordTick(uint32_t now_ms);
void Traj_StopRecord(void);
uint8_t Traj_Recording(void);

/** loops: 0 = 无限次（stop 停止） */
int  Traj_StartPlay(uint8_t slot, uint8_t loops);
/** 回放平滑时间常数（ms），0 = 关闭；默认 40ms */
/** 回放方式：0 = CSP（默认），1 = MIT */
/** 轮次之间的停留（ms），0 = 连续播放 */
void     Traj_SetPlayGap(uint16_t gap_ms);
uint16_t Traj_GetPlayGap(void);
void     Traj_SetPlayMode(uint8_t mit);
uint8_t  Traj_GetPlayMode(void);
void     Traj_SetSmooth(uint16_t tau_ms);
uint16_t Traj_GetSmooth(void);
void Traj_PlayTick(uint32_t now_ms);
void Traj_StopPlay(void);
uint8_t Traj_Playing(void);

const traj_meta_t *Traj_GetMeta(uint8_t slot);
int16_t Traj_GetSample(uint8_t slot, uint16_t frame, uint8_t axis);
void Traj_Clear(uint8_t slot);

/* 头部已随录制写入 Flash，这里只为兼容旧命令 */
int  Traj_SaveToFlash(uint8_t slot);
int  Traj_LoadFromFlash(uint8_t slot);
int  Traj_LoadAll(void);
int  Traj_FlashReady(void);

#endif /* __TRAJ_H__ */
