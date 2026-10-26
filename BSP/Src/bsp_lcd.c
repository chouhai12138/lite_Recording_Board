/**
 * @file    bsp_lcd.c
 * @brief   屏幕适配层：直接调用原厂驱动（BSP/lcd154/lcd_spi_154.c）
 *
 * 原厂例程 = 模组科技 SPI154M1-240*240（ST7789）+ MZ407M3-VET6 核心板，
 * 引脚和我们板子完全一致：SCK=PB13 / SDA=PB15 / CS=PB14 / DC=PD12 / BL=PD11。
 * 这里只做 RGB565 → RGB888 的转换和方向枚举映射，初始化/画点/清屏全用原厂代码。
 */
#include "bsp_lcd.h"
#include "lcd_spi_154.h"

static uint8_t s_ready;

static uint32_t to888(uint16_t c)
{
    uint32_t r = (uint32_t)((c >> 11) & 0x1FU);
    uint32_t g = (uint32_t)((c >> 5) & 0x3FU);
    uint32_t b = (uint32_t)(c & 0x1FU);

    r = (r * 255U + 15U) / 31U;
    g = (g * 255U + 31U) / 63U;
    b = (b * 255U + 15U) / 31U;
    return (r << 16) | (g << 8) | b;
}

void BSP_LCD_Init(void)
{
    SPI_LCD_Init();          /* 原厂驱动：SPI2 + CS/DC/BL + ST7789 初始化 + 清屏 + 开背光 */

    /* 实装方向 ≈ 原厂竖屏转 180°（实机确认：原厂 V 方向下画面上下颠倒）。
     * 用原厂 Direction_V_Flip（MADCTL 0xC0 + Y 偏移 80）摆正。 */
    LCD_SetDirection(Direction_V_Flip);
    LCD_SetBackColor(0x000000);
    LCD_Clear();

    s_ready = 1;
}

int BSP_LCD_IsReady(void)
{
    return s_ready;
}

void BSP_LCD_Backlight(uint8_t on)
{
    if (on) {
        LCD_Backlight_ON;
    } else {
        LCD_Backlight_OFF;
    }
}

void BSP_LCD_SetDirection(bsp_lcd_dir_t dir)
{
    switch (dir) {
        case BSP_LCD_DIR_LANDSCAPE:      LCD_SetDirection(Direction_H);      break;
        case BSP_LCD_DIR_LANDSCAPE_180:  LCD_SetDirection(Direction_H_Flip); break;
        case BSP_LCD_DIR_PORTRAIT:       LCD_SetDirection(Direction_V);      break;
        case BSP_LCD_DIR_PORTRAIT_180:
        default:                         LCD_SetDirection(Direction_V_Flip); break;
    }
}

uint16_t BSP_LCD_Width(void)
{
    return (uint16_t)LCD_Width;
}

uint16_t BSP_LCD_Height(void)
{
    return (uint16_t)LCD_Height;
}

void BSP_LCD_Fill(uint16_t color)
{
    LCD_SetBackColor(to888(color));
    LCD_Clear();
}

void BSP_LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0U || h == 0U) {
        return;
    }
    LCD_SetBackColor(to888(color));
    LCD_ClearRect(x, y, w, h);
}

void BSP_LCD_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_DrawPoint(x, y, to888(color));
}

void BSP_LCD_TestPattern(void)
{
    uint16_t h = (uint16_t)(BSP_LCD_Height() / 3U);

    BSP_LCD_FillRect(0, 0, BSP_LCD_Width(), h, LCD_RGB565(255, 0, 0));
    BSP_LCD_FillRect(0, h, BSP_LCD_Width(), h, LCD_RGB565(0, 255, 0));
    BSP_LCD_FillRect(0, (uint16_t)(2U * h), BSP_LCD_Width(),
                     (uint16_t)(BSP_LCD_Height() - 2U * h),
                     LCD_RGB565(0, 0, 255));
    BSP_LCD_Backlight(1);
}
