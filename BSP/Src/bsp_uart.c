/**
 * @file    bsp_uart.c
 * @brief   调试串口：寄存器级收发（不经过 HAL 的 UART 状态机）
 *
 * 为什么不用 HAL_UART_Transmit / HAL_UART_Receive_IT：
 *   HAL 的发送会持有 huart->Lock，而接收回调里再调 HAL_UART_Receive_IT
 *   也要拿同一把锁。主循环打印（整屏 UI 重绘）期间来一帧 RX，回调里的
 *   重新挂接收就会因为锁冲突失败 —— 于是接收悄悄停掉，表现是"串口突然
 *   不响应任何命令，但板子还在跑"。改成寄存器级收发后没有这个锁。
 */
#include "bsp_uart.h"

#define UART_RX_BUF_SIZE    256U

static UART_HandleTypeDef s_uart;
static USART_TypeDef     *s_reg;

/* 接收环形缓冲区（中断写入，主循环读取） */
static volatile uint8_t  s_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

static inline void rx_push(uint8_t b)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % UART_RX_BUF_SIZE);
    if (next != s_rx_tail) {            /* 满了丢新字节 */
        s_rx_buf[s_rx_head] = b;
        s_rx_head = next;
    }
}

void BSP_UART_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};

    BOARD_UART_CLK_ENABLE();
    BOARD_UART_GPIO_CLK_ENABLE();

    /* PA9 = TX, PA10 = RX */
    gpio.Pin       = BOARD_UART_TX_PIN | BOARD_UART_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = BOARD_UART_GPIO_AF;
    HAL_GPIO_Init(BOARD_UART_GPIO_PORT, &gpio);

    s_uart.Instance          = BOARD_UART;
    s_uart.Init.BaudRate     = baudrate;
    s_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits     = UART_STOPBITS_1;
    s_uart.Init.Parity       = UART_PARITY_NONE;
    s_uart.Init.Mode         = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&s_uart) != HAL_OK) {
        Error_Handler();
    }

    s_reg = BOARD_UART;

    /* 只用 RXNE 中断；不用 HAL 的接收状态机 */
    __HAL_UART_ENABLE_IT(&s_uart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(BOARD_UART_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(BOARD_UART_IRQn);
}

static void tx_byte(uint8_t b)
{
    while ((s_reg->SR & USART_SR_TXE) == 0U) {
        /* 等发送寄存器空 */
    }
    s_reg->DR = b;
}

void BSP_UART_Write(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        tx_byte(data[i]);
    }
}

int BSP_UART_PutChar(int ch)
{
    tx_byte((uint8_t)ch);
    return ch;
}

uint8_t BSP_UART_ReadByte(uint8_t *ch)
{
    if (s_rx_tail == s_rx_head) {
        return 0;
    }
    *ch = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % UART_RX_BUF_SIZE);
    return 1;
}

uint16_t BSP_UART_RxCount(void)
{
    return (uint16_t)((s_rx_head + UART_RX_BUF_SIZE - s_rx_tail) % UART_RX_BUF_SIZE);
}

void BSP_UART_IRQHandler(void)
{
    uint32_t sr = s_reg->SR;

    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) != 0U) {
        uint8_t b = (uint8_t)(s_reg->DR & 0xFFU);   /* 读 SR+DR：清 RXNE/ORE */
        if ((sr & USART_SR_RXNE) != 0U) {
            rx_push(b);
        }
    }
    if ((sr & (USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U) {
        (void)s_reg->DR;                            /* 清错误标志 */
    }
}
