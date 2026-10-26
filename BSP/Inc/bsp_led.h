/**
 * @file    bsp_led.h
 * @brief   板载 LED：CAN(PD15) / FAULT(PD13)（PA15 归 W25Q64 CS，PD12 归 LCD DC）
 */
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "main.h"

typedef enum {
    BSP_LED_CAN = 0,
    BSP_LED_FAULT,
    BSP_LED_COUNT
} bsp_led_t;

void BSP_LED_Init(void);
void BSP_LED_On(bsp_led_t led);
void BSP_LED_Off(bsp_led_t led);
void BSP_LED_Toggle(bsp_led_t led);
void BSP_LED_Write(bsp_led_t led, uint8_t on);

#endif /* __BSP_LED_H__ */
