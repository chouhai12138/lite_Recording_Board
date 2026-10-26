/**
 * @file    bsp_led.c
 * @brief   4 路 LED 驱动（引脚见 board_config.h）
 */
#include "bsp_led.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} led_pin_t;

static const led_pin_t s_leds[BSP_LED_COUNT] = {
    [BSP_LED_CAN]   = { BOARD_LED_CAN_PORT,   BOARD_LED_CAN_PIN   },
    [BSP_LED_FAULT] = { BOARD_LED_FAULT_PORT, BOARD_LED_FAULT_PIN },
};

static GPIO_PinState led_level(uint8_t on)
{
    return on ? BOARD_LED_ACTIVE_LEVEL
              : (GPIO_PinState)!BOARD_LED_ACTIVE_LEVEL;
}

void BSP_LED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    BOARD_LED_CAN_CLK_ENABLE();
    BOARD_LED_FAULT_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < (uint8_t)BSP_LED_COUNT; i++) {
        gpio.Pin = s_leds[i].pin;
        HAL_GPIO_WritePin(s_leds[i].port, s_leds[i].pin, led_level(0));
        HAL_GPIO_Init(s_leds[i].port, &gpio);
    }
}

void BSP_LED_Write(bsp_led_t led, uint8_t on)
{
    if (led >= BSP_LED_COUNT) {
        return;
    }
    HAL_GPIO_WritePin(s_leds[led].port, s_leds[led].pin, led_level(on));
}

void BSP_LED_On(bsp_led_t led)
{
    BSP_LED_Write(led, 1);
}

void BSP_LED_Off(bsp_led_t led)
{
    BSP_LED_Write(led, 0);
}

void BSP_LED_Toggle(bsp_led_t led)
{
    if (led >= BSP_LED_COUNT) {
        return;
    }
    HAL_GPIO_TogglePin(s_leds[led].port, s_leds[led].pin);
}
