/**
 * @file    bsp_can.h
 * @brief   双路 CAN（F407 片上 bxCAN）：CAN1 = PB8/PB9，CAN2 = PB12/PB6，AF9
 *
 * 位时序：APB1 42MHz，预分频 3 → 14MHz tq；BS1=11、BS2=2 → 14 tq/bit = 1 Mbit/s
 * 采样点 85.7%。
 */
#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include <stdint.h>

typedef enum {
    BSP_CAN_1 = 0,
    BSP_CAN_2,
    BSP_CAN_COUNT
} bsp_can_id_t;

typedef struct {
    uint8_t  ready;
    uint8_t  loopback;
    uint8_t  state;        /* 由 MSR 推导：0=OK 1=WARN 2=PASSIVE 3=BUSOFF 0xFF=未初始化 */
    uint32_t msr;
    uint32_t esr;
    uint8_t  last_error;   /* HAL_CAN_ErrorTypeDef */
    uint16_t tec;          /* 发送错误计数 */
    uint16_t rec;          /* 接收错误计数 */
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t rx_lost;
} bsp_can_stats_t;

typedef void (*bsp_can_rx_cb_t)(bsp_can_id_t bus, uint32_t id,
                                uint8_t dlc, const uint8_t *data);

/** 初始化两路 CAN（1 Mbit/s，接收全部 ID，FIFO0 中断接收） */
int  BSP_CAN_Init(bsp_can_rx_cb_t cb);
/** mode: 0 = 正常，1 = 内部回环（自测用，不需要总线上有第二个节点） */
int  BSP_CAN_SetMode(uint8_t loopback);
int  BSP_CAN_Send(bsp_can_id_t bus, uint32_t id, const uint8_t *data, uint8_t dlc);
void BSP_CAN_GetStats(bsp_can_id_t bus, bsp_can_stats_t *out);
void BSP_CAN_IrqHandler(bsp_can_id_t bus);

#endif /* __BSP_CAN_H__ */
