/**
 * @file    bsp_uart.h
 * @brief   调试串口（USART1，PA9=TX / PA10=RX），支持收发
 */
#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "main.h"

void    BSP_UART_Init(uint32_t baudrate);
void    BSP_UART_Write(const uint8_t *data, uint16_t len);
int     BSP_UART_PutChar(int ch);

/* 非阻塞读：缓冲区里有数据返回 1 并取走一个字节 */
uint8_t BSP_UART_ReadByte(uint8_t *ch);
/* 接收缓冲区里还有多少字节 */
uint16_t BSP_UART_RxCount(void);

/* 给中断向量用 */
void    BSP_UART_IRQHandler(void);

#endif /* __BSP_UART_H__ */
