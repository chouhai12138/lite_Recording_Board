/**
 * @file    bsp_lcd.h
 * @brief   1.54" ST7789（240×240）显示驱动，SPI2
 *
 * 硬件（SCH_STM32F4x_DAP_CORE_CAN485 原理图 U22 FPC 座）：
 *   PB13 = SCK, PB15 = MOSI, PB14 = MISO（不用）
 *   PD12 = DC,  PD11 = BL
 *   连接器没有引出 CS / RST（模块内部固定），本驱动不驱动它们。
 *
 * 面板参数来自原厂例程（配套测试程序/彩色液晶屏驱动-基于1.54寸240x240分辨率）：
 *   竖屏      MADCTL=0x00, offset 0,0
 *   竖屏 180° MADCTL=0xC0, offset X=0  Y=80   ← 默认（按实装方向）
 *   横屏      MADCTL=0x70, offset 0,0
 *   横屏 180° MADCTL=0xA0, offset X=80 Y=0
 */
#ifndef __BSP_LCD_H__
#define __BSP_LCD_H__

#include "main.h"

#define LCD_RGB565(r, g, b)  ((uint16_t)((((r) & 0xF8U) << 8) | (((g) & 0xFCU) << 3) | ((b) >> 3)))

/* 注意：原厂驱动头文件里也定义了 LCD_RED/…（24bit RGB888），
 * 这里不再定义同名宏，避免冲突；BSP 接口统一用 RGB565，用 LCD_RGB565() 构造。 */

typedef enum {
    BSP_LCD_DIR_PORTRAIT = 0,      /* 240×240 竖屏 */
    BSP_LCD_DIR_PORTRAIT_180,      /* 240×240 竖屏旋转 180°（默认，按实装方向） */
    BSP_LCD_DIR_LANDSCAPE,         /* 240×240 横屏 */
    BSP_LCD_DIR_LANDSCAPE_180,     /* 240×240 横屏旋转 180° */
} bsp_lcd_dir_t;

void BSP_LCD_Init(void);
/** 面板初始化是否成功（SPI 卡死/超时返回 0） */
int  BSP_LCD_IsReady(void);
void BSP_LCD_SetDirection(bsp_lcd_dir_t dir);
uint16_t BSP_LCD_Width(void);
uint16_t BSP_LCD_Height(void);

void BSP_LCD_Backlight(uint8_t on);

void BSP_LCD_Fill(uint16_t color);
void BSP_LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void BSP_LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

/** 上板自检图案：红/绿/蓝三条 */
void BSP_LCD_TestPattern(void);

#endif /* __BSP_LCD_H__ */
