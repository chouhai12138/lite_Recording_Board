/**
 * @file    board_config.h
 * @brief   板级引脚/时钟配置（换板子只改这里）
 *
 * 说明：本工程是"录音板"的移植骨架，默认引脚按通用接法给出，
 *       请对照原理图核对后再烧写。
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "stm32f4xx_hal.h"

/* ------------------------- 外部晶振 ------------------------- */
/* 板载 HSE 晶振频率：本板实测 25 MHz（HSE 起振、PLL 锁定 168MHz）。
 * 改这里的同时 Makefile 里的 HSE 变量也要一致。 */
#ifndef HSE_VALUE
#define HSE_VALUE               25000000U
#endif

/* ------------------------- LED（3 路） -------------------------
 * 来源：SCH_STM32F4x_DAP_CORE_CAN485 原理图 —— KT-0603R 经 1k 接 3V3，IO 拉低点亮。
 *   CAN   PD15  （原理图 LED3 / 网络 PD15_LCD）
 *   FAULT PD13  （扩展 IO）
 * 注：PA15 在板上同时接了 LED2 和 W25Q64 的 /CS，现在归 W25Q64 使用
 *     （访问 Flash 时那颗 LED 会闪，属正常）。
 * ⚠️ PD12 现在是 LCD 的 DC，不能再当 LED。PA15 复位后是 JTAG 的 JTDI，
 *    配成 GPIO 后 JTAG 不可用（SWD 不受影响）。*/
#define BOARD_LED_ACTIVE_LEVEL  GPIO_PIN_RESET

#define BOARD_LED_CAN_PORT          GPIOD
#define BOARD_LED_CAN_PIN           GPIO_PIN_15
#define BOARD_LED_CAN_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()

#define BOARD_LED_FAULT_PORT        GPIOD
#define BOARD_LED_FAULT_PIN         GPIO_PIN_13
#define BOARD_LED_FAULT_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

/* ------------------------- LCD：1.54" ST7789 240×240，SPI2 -------------------------
 * 来源：DOCS/SCH_STM32F4x_DAP_CORE_CAN485_原理图_2026-08-08.pdf 的 U22（10P FPC）：
 *   pin1 PD11_BL     pin2 PD12_DC     pin3 PB15_SPI2_MOSI
 *   pin4 PB13_SPI2_SCK  pin5 PB14_SPI2_MISO
 *   pin6 GND         pin7/8 3.3V      pin9/10 GND
 * 该连接器**没有引出 CS 和 RST**（模块内部固定），所以代码不驱动 CS/RST。 */
/* ------------------------- W25Q64（SPI1） -------------------------
 * 原理图：CS=PA15 / MISO=PB4 / SCK=PA5 / MOSI=PB5（全部 AF5） */
#define BOARD_FLASH_SPI             SPI1
#define BOARD_FLASH_SPI_CLK_ENABLE() __HAL_RCC_SPI1_CLK_ENABLE()
#define BOARD_FLASH_GPIO_CLK_ENABLE() \
    do { __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); } while (0)
#define BOARD_FLASH_CS_PORT         GPIOA
#define BOARD_FLASH_CS_PIN          GPIO_PIN_15
#define BOARD_FLASH_SCK_PORT        GPIOA
#define BOARD_FLASH_SCK_PIN         GPIO_PIN_5
#define BOARD_FLASH_MISO_PORT       GPIOB
#define BOARD_FLASH_MISO_PIN        GPIO_PIN_4
#define BOARD_FLASH_MOSI_PORT       GPIOB
#define BOARD_FLASH_MOSI_PIN        GPIO_PIN_5
#define BOARD_FLASH_GPIO_AF         GPIO_AF5_SPI1

/* 面板原生 240×240；翻转方向的坐标偏移见 BSP/bsp_lcd.c（来自原厂 1.54" 例程） */
#define BOARD_LCD_PANEL_W           240
#define BOARD_LCD_PANEL_H           240

#define BOARD_LCD_SPI               SPI2
#define BOARD_LCD_SPI_CLK_ENABLE()  __HAL_RCC_SPI2_CLK_ENABLE()
#define BOARD_LCD_GPIO_CLK_ENABLE() \
    do { __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE(); } while (0)

#define BOARD_LCD_SCK_PORT          GPIOB
#define BOARD_LCD_SCK_PIN           GPIO_PIN_13
#define BOARD_LCD_MOSI_PORT         GPIOB
#define BOARD_LCD_MOSI_PIN          GPIO_PIN_15
#define BOARD_LCD_GPIO_AF           GPIO_AF5_SPI2

/* ⚠️ 模块 FPC 是"反序"的：板子 pin5 接到模块的 CS（模块 1~10 = G,G,3,3,G,CS,SCK,SDA,WR,PWR）。
 * 所以 PB14 不是 MISO，而是 CS —— 必须由 MCU 拉低，否则屏幕收不到任何命令（纯黑）。 */
#define BOARD_LCD_CS_PORT           GPIOB
#define BOARD_LCD_CS_PIN            GPIO_PIN_14

#define BOARD_LCD_DC_PORT           GPIOD
#define BOARD_LCD_DC_PIN            GPIO_PIN_12
#define BOARD_LCD_BL_PORT           GPIOD
#define BOARD_LCD_BL_PIN            GPIO_PIN_11

/* ------------------------- 按键（底板 4 键）-------------------------
 * 底板已装好，4 个按键全部接到 GPIOE，按下接 GND（低有效，内部上拉）：
 *   PE14 = 上(UP)     PE12 = 右/确认(RIGHT)
 *   PE10 = 下(DOWN)   PE8  = 左/返回(LEFT)
 * ⚠️ 这 4 个脚在核心板丝印上是 *_LCD，底板借用为按键。 */
#define BOARD_KEY_LONG_MS           1000U

#define BOARD_KEY_UP_PORT           GPIOE
#define BOARD_KEY_UP_PIN            GPIO_PIN_14
#define BOARD_KEY_UP_ACTIVE         GPIO_PIN_RESET
#define BOARD_KEY_UP_ENABLED        1

#define BOARD_KEY_DOWN_PORT         GPIOE
#define BOARD_KEY_DOWN_PIN          GPIO_PIN_10
#define BOARD_KEY_DOWN_ACTIVE       GPIO_PIN_RESET
#define BOARD_KEY_DOWN_ENABLED      1

#define BOARD_KEY_LEFT_PORT         GPIOE
#define BOARD_KEY_LEFT_PIN          GPIO_PIN_8
#define BOARD_KEY_LEFT_ACTIVE       GPIO_PIN_RESET
#define BOARD_KEY_LEFT_ENABLED      1

#define BOARD_KEY_RIGHT_PORT        GPIOE
#define BOARD_KEY_RIGHT_PIN         GPIO_PIN_12
#define BOARD_KEY_RIGHT_ACTIVE      GPIO_PIN_RESET
#define BOARD_KEY_RIGHT_ENABLED     1

#define BOARD_KEYS_GPIO_CLK_ENABLE() \
    do { __HAL_RCC_GPIOE_CLK_ENABLE(); } while (0)

/* ------------------------- 调试串口 -------------------------- */
/* 波特率：WCH-Link 的串口桥在 115200 满速连续输出时会丢字节（USB CDC 侧限制），
 * 57600 更稳。用 make UART_BAUD=57600 编译可切换，终端也要改成同样的值。 */
#ifndef UART_BAUD
#define UART_BAUD               115200U
#endif

/* USART1: PA9 = TX, PA10 = RX, AF7。接 USB-TTL 即可看到 printf。 */
#define BOARD_UART              USART1
#define BOARD_UART_IRQn         USART1_IRQn
#define BOARD_UART_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()
#define BOARD_UART_GPIO_PORT    GPIOA
#define BOARD_UART_TX_PIN       GPIO_PIN_9
#define BOARD_UART_RX_PIN       GPIO_PIN_10
#define BOARD_UART_GPIO_AF      GPIO_AF7_USART1
#define BOARD_UART_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

/* ------------------------- 目标主频 -------------------------- */
#define BOARD_SYSCLK_HZ         168000000U

#endif /* __BOARD_CONFIG_H__ */
