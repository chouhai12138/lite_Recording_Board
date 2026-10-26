/**
 * @file    bsp_can.c
 * @brief   双路 bxCAN 驱动（CAN1: PB8/PB9，CAN2: PB12/PB6）
 */
#include <string.h>

#include "main.h"
#include "bsp_can.h"

#define CAN_BAUD_PRESCALER  3U      /* 42MHz / 3 = 14MHz */
#define CAN_BS1             CAN_BS1_11TQ
#define CAN_BS2             CAN_BS2_2TQ

static CAN_HandleTypeDef s_hcan[BSP_CAN_COUNT];
static bsp_can_rx_cb_t   s_cb;
static bsp_can_stats_t   s_stats[BSP_CAN_COUNT];
static uint8_t           s_loopback;

static void can_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;          /* CAN1_RX / CAN1_TX */
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Alternate = GPIO_AF9_CAN2;
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_6;         /* CAN2_RX=PB12 / CAN2_TX=PB6 */
    HAL_GPIO_Init(GPIOB, &gpio);
}

static int can_apply_mode(bsp_can_id_t bus, uint32_t mode)
{
    CAN_HandleTypeDef *h = &s_hcan[bus];

    h->Init.Mode = mode;
    if (HAL_CAN_Init(h) != HAL_OK) {
        return 0;
    }
    return 1;
}

static int can_setup(bsp_can_id_t bus)
{
    CAN_HandleTypeDef *h = &s_hcan[bus];
    CAN_FilterTypeDef f = {0};

    h->Instance = (bus == BSP_CAN_1) ? CAN1 : CAN2;
    h->Init.Prescaler         = CAN_BAUD_PRESCALER;
    h->Init.Mode              = s_loopback ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL;
    h->Init.SyncJumpWidth     = CAN_SJW_1TQ;
    h->Init.TimeSeg1          = CAN_BS1;
    h->Init.TimeSeg2          = CAN_BS2;
    h->Init.TimeTriggeredMode = DISABLE;
    h->Init.AutoBusOff        = ENABLE;
    h->Init.AutoWakeUp        = DISABLE;
    h->Init.AutoRetransmission = ENABLE;
    h->Init.ReceiveFifoLocked = DISABLE;
    h->Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(h) != HAL_OK) {
        return 0;
    }

    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;      /* 全接收 */
    f.FilterMaskIdLow      = 0x0000;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14;
    f.FilterBank           = (bus == BSP_CAN_1) ? 0 : 14;

    if (HAL_CAN_ConfigFilter(h, &f) != HAL_OK) {
        return 0;
    }
    if (HAL_CAN_Start(h) != HAL_OK) {
        return 0;
    }
    if (HAL_CAN_ActivateNotification(h,
            CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
        return 0;
    }
    return 1;
}

int BSP_CAN_Init(bsp_can_rx_cb_t cb)
{
    s_cb = cb;
    memset(s_stats, 0, sizeof(s_stats));
    s_loopback = 0;

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();

    can_gpio_init();

    for (uint8_t i = 0; i < BSP_CAN_COUNT; i++) {
        if (!can_setup((bsp_can_id_t)i)) {
            return 0;
        }
        s_stats[i].ready = 1;
        s_stats[i].loopback = s_loopback;
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    return 1;
}

int BSP_CAN_SetMode(uint8_t loopback)
{
    s_loopback = (uint8_t)(loopback ? 1 : 0);

    for (uint8_t i = 0; i < BSP_CAN_COUNT; i++) {
        if (s_stats[i].ready) {
            (void)HAL_CAN_Stop(&s_hcan[i]);
        }
    }
    for (uint8_t i = 0; i < BSP_CAN_COUNT; i++) {
        if (!can_setup((bsp_can_id_t)i)) {
            return 0;
        }
        s_stats[i].loopback = s_loopback;
    }
    return 1;
}

int BSP_CAN_Send(bsp_can_id_t bus, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx;
    uint32_t mailbox = 0;
    uint32_t timeout = 2000;
    uint8_t  buf[8];

    if (bus >= BSP_CAN_COUNT || dlc > 8U) {
        return 0;
    }
    if (!s_stats[bus].ready) {
        return 0;
    }

    memset(buf, 0, sizeof(buf));
    if (data != NULL) {
        memcpy(buf, data, dlc);
    }

    tx.StdId   = 0;
    tx.ExtId   = id & 0x1FFFFFFFU;
    tx.IDE     = CAN_ID_EXT;
    tx.RTR     = CAN_RTR_DATA;
    tx.DLC     = dlc;
    tx.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&s_hcan[bus]) == 0U && timeout-- > 0U) {
        /* 等邮箱 */
    }
    if (HAL_CAN_AddTxMessage(&s_hcan[bus], &tx, buf, &mailbox) != HAL_OK) {
        return 0;
    }
    s_stats[bus].tx_frames++;
    return 1;
}

void BSP_CAN_GetStats(bsp_can_id_t bus, bsp_can_stats_t *out)
{
    if (bus >= BSP_CAN_COUNT || out == NULL) {
        return;
    }
    uint32_t msr = s_hcan[bus].Instance->MSR;
    uint32_t esr = s_hcan[bus].Instance->ESR;

    s_stats[bus].msr        = msr;
    s_stats[bus].esr        = esr;
    s_stats[bus].last_error = (uint8_t)HAL_CAN_GetError(&s_hcan[bus]);
    s_stats[bus].tec        = (uint16_t)((esr >> 16) & 0xFFU);
    s_stats[bus].rec        = (uint16_t)((esr >> 24) & 0xFFU);

    /* F4 bxCAN：错误标志在 ESR —— bit2 BOFF / bit1 EPVF / bit0 EWGF */
    if ((msr & CAN_MSR_INAK) != 0U) {
        s_stats[bus].state = 0xFFU;          /* 还在初始化 */
    } else if ((esr & CAN_ESR_BOFF) != 0U) {
        s_stats[bus].state = 3U;
    } else if ((esr & CAN_ESR_EPVF) != 0U) {
        s_stats[bus].state = 2U;
    } else if ((esr & CAN_ESR_EWGF) != 0U) {
        s_stats[bus].state = 1U;
    } else {
        s_stats[bus].state = 0U;
    }
    *out = s_stats[bus];
}

void BSP_CAN_IrqHandler(bsp_can_id_t bus)
{
    HAL_CAN_IRQHandler(&s_hcan[bus]);
}

/* ---------------- HAL 回调 ---------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    bsp_can_id_t bus = (hcan->Instance == CAN1) ? BSP_CAN_1 : BSP_CAN_2;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK) {
            s_stats[bus].rx_lost++;
            break;
        }
        s_stats[bus].rx_frames++;
        if (s_cb != NULL) {
            s_cb(bus, (rx.IDE == CAN_ID_EXT) ? rx.ExtId : rx.StdId,
                 (uint8_t)rx.DLC, data);
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    bsp_can_id_t bus = (hcan->Instance == CAN1) ? BSP_CAN_1 : BSP_CAN_2;
    s_stats[bus].last_error = (uint8_t)HAL_CAN_GetError(hcan);
}
