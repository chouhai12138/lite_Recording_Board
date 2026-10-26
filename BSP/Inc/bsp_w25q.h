/**
 * @file    bsp_w25q.h
 * @brief   W25Q64（8 MB SPI NOR）驱动：SPI1，CS=PA15 / SCK=PA5 / MISO=PB4 / MOSI=PB5
 *
 * 只实现录制/回放需要的：读 ID、按页写、按扇区擦、连续读。
 */
#ifndef __BSP_W25Q_H__
#define __BSP_W25Q_H__

#include <stdint.h>

#define W25Q_PAGE_SIZE      256U
#define W25Q_SECTOR_SIZE    4096U
#define W25Q_BLOCK_SIZE     65536U

/** 初始化 SPI1 + CS；返回 1 表示读到的 JEDEC ID 是 W25Q64 */
int  W25Q_Init(void);
uint32_t W25Q_JedecId(void);

int  W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len);
int  W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len);   /* 自动按页 */
int  W25Q_EraseSector(uint32_t addr);
int  W25Q_EraseBlock(uint32_t addr);
int  W25Q_IsReady(void);

#endif /* __BSP_W25Q_H__ */
