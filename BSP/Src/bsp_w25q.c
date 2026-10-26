/**
 * @file    bsp_w25q.c
 * @brief   W25Q64 驱动（SPI1 阻塞式，21MHz）
 */
#include "main.h"
#include "bsp_w25q.h"

#define CMD_WRITE_ENABLE    0x06U
#define CMD_READ_STATUS1    0x05U
#define CMD_READ_DATA       0x03U
#define CMD_PAGE_PROGRAM    0x02U
#define CMD_SECTOR_ERASE    0x20U
#define CMD_BLOCK_ERASE     0xD8U
#define CMD_JEDEC_ID        0x9FU

static SPI_HandleTypeDef s_spi;
static uint8_t           s_ok;
static uint32_t          s_jedec;

static void cs_low(void)
{
    HAL_GPIO_WritePin(BOARD_FLASH_CS_PORT, BOARD_FLASH_CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(BOARD_FLASH_CS_PORT, BOARD_FLASH_CS_PIN, GPIO_PIN_SET);
}

static void spi_tx(const uint8_t *buf, uint16_t len)
{
    HAL_SPI_Transmit(&s_spi, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

static void spi_rx(uint8_t *buf, uint16_t len)
{
    HAL_SPI_Receive(&s_spi, buf, len, HAL_MAX_DELAY);
}

static uint8_t read_status(void)
{
    uint8_t cmd = CMD_READ_STATUS1;
    uint8_t st  = 0;

    cs_low();
    spi_tx(&cmd, 1);
    spi_rx(&st, 1);
    cs_high();
    return st;
}

int W25Q_IsReady(void)
{
    return ((read_status() & 0x01U) == 0U) ? 1 : 0;
}

static void write_enable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    cs_low();
    spi_tx(&cmd, 1);
    cs_high();
}

static int wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (!W25Q_IsReady()) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return 0;
        }
    }
    return 1;
}

int W25Q_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    BOARD_FLASH_GPIO_CLK_ENABLE();
    BOARD_FLASH_SPI_CLK_ENABLE();

    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = BOARD_FLASH_GPIO_AF;
    gpio.Pin       = BOARD_FLASH_SCK_PIN;
    HAL_GPIO_Init(BOARD_FLASH_SCK_PORT, &gpio);
    gpio.Pin       = BOARD_FLASH_MOSI_PIN | BOARD_FLASH_MISO_PIN;
    HAL_GPIO_Init(BOARD_FLASH_MOSI_PORT, &gpio);

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = BOARD_FLASH_CS_PIN;
    HAL_GPIO_WritePin(BOARD_FLASH_CS_PORT, BOARD_FLASH_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(BOARD_FLASH_CS_PORT, &gpio);
    cs_high();

    s_spi.Instance               = BOARD_FLASH_SPI;
    s_spi.Init.Mode              = SPI_MODE_MASTER;
    s_spi.Init.Direction         = SPI_DIRECTION_2LINES;
    s_spi.Init.DataSize          = SPI_DATASIZE_8BIT;
    s_spi.Init.CLKPolarity       = SPI_POLARITY_LOW;
    s_spi.Init.CLKPhase          = SPI_PHASE_1EDGE;
    s_spi.Init.NSS               = SPI_NSS_SOFT;
    s_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;   /* 84MHz/4 = 21MHz */
    s_spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    s_spi.Init.TIMode            = SPI_TIMODE_DISABLE;
    s_spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    s_spi.Init.CRCPolynomial     = 10;

    if (HAL_SPI_Init(&s_spi) != HAL_OK) {
        return 0;
    }
    __HAL_SPI_ENABLE(&s_spi);

    /* JEDEC ID */
    {
        uint8_t cmd = CMD_JEDEC_ID;
        uint8_t id[3] = {0, 0, 0};

        cs_low();
        spi_tx(&cmd, 1);
        spi_rx(id, 3);
        cs_high();
        s_jedec = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    }
    s_ok = (s_jedec == 0xEF4017U) ? 1U : 0U;   /* W25Q64 */
    return s_ok;
}

uint32_t W25Q_JedecId(void)
{
    return s_jedec;
}

int W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];

    if (!s_ok || buf == NULL || len == 0U) {
        return 0;
    }
    cmd[0] = CMD_READ_DATA;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)addr;

    cs_low();
    spi_tx(cmd, 4);
    spi_rx(buf, (uint16_t)len);
    cs_high();
    return 1;
}

int W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t done = 0;

    if (!s_ok || buf == NULL) {
        return 0;
    }
    while (done < len) {
        uint32_t room  = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);
        uint32_t chunk = (len - done < room) ? (len - done) : room;
        uint8_t  cmd[4];

        write_enable();
        cmd[0] = CMD_PAGE_PROGRAM;
        cmd[1] = (uint8_t)(addr >> 16);
        cmd[2] = (uint8_t)(addr >> 8);
        cmd[3] = (uint8_t)addr;

        cs_low();
        spi_tx(cmd, 4);
        spi_tx(buf + done, (uint16_t)chunk);
        cs_high();

        if (!wait_ready(100)) {
            return 0;
        }
        addr += chunk;
        done += chunk;
    }
    return 1;
}

int W25Q_EraseSector(uint32_t addr)
{
    uint8_t cmd[4];

    if (!s_ok) {
        return 0;
    }
    write_enable();
    cmd[0] = CMD_SECTOR_ERASE;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)addr;

    cs_low();
    spi_tx(cmd, 4);
    cs_high();
    return wait_ready(2000);
}

int W25Q_EraseBlock(uint32_t addr)
{
    uint8_t cmd[4];

    if (!s_ok) {
        return 0;
    }
    write_enable();
    cmd[0] = CMD_BLOCK_ERASE;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)addr;

    cs_low();
    spi_tx(cmd, 4);
    cs_high();
    return wait_ready(5000);
}
