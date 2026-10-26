/**
 * @file    stm32f4xx_it.c
 * @brief   中断服务程序
 *
 * 只保留最小集合；用到哪个外设就在这里加对应的 IRQHandler。
 */
#include "main.h"
#include "stm32f4xx_it.h"
#include "bsp_uart.h"
#include "bsp_can.h"

/* ---------------- Cortex-M4 core exceptions ---------------- */
void NMI_Handler(void)
{
    while (1) {
    }
}

void HardFault_Handler(void)
{
    /* 调试时停在这里，用 GDB 的 bt 看调用栈 */
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void CAN1_RX0_IRQHandler(void)
{
    BSP_CAN_IrqHandler(BSP_CAN_1);
}

void CAN1_TX_IRQHandler(void)
{
    BSP_CAN_IrqHandler(BSP_CAN_1);
}

void CAN2_TX_IRQHandler(void)
{
    BSP_CAN_IrqHandler(BSP_CAN_2);
}

void CAN2_RX0_IRQHandler(void)
{
    BSP_CAN_IrqHandler(BSP_CAN_2);
}

/* 调试串口接收中断（USART1 = PA9/PA10）。BSP 里用寄存器级收发，
 * 不再经过 HAL 的 UART 状态机，避免发送持锁导致接收挂死。 */
void USART1_IRQHandler(void)
{
    BSP_UART_IRQHandler();
}
