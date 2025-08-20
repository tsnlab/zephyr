#ifndef SPI_H
#define SPI_H

#include <stdint.h>

#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#define SPI_BASE 0xA0100000

#define SPI_DATA(base)       (base + 0x00)
#define SPI_STAT(base)       (base + 0x04)
#define SPI_INTEN(base)      (base + 0x08)
#define SPI_MODE(base)       (base + 0x0c)
#define SPI_CTRL(base)       (base + 0x10)
#define SPI_EVTCTRL(base)    (base + 0x14)
#define SPI_DMA_CTRL(base)   (base + 0x2c)
#define SPI_DMA_STATUS(base) (base + 0x30)
#define SPI_TXBASE(base)     (base + 0x20)
#define SPI_RXBASE(base)     (base + 0x24)
#define SPI_PACKET(base)     (base + 0x28)
#define SPI_DMA_ICR(base)    (base + 0x34)

#define SPI_INTEN_DR BIT(30)
#define SPI_INTEN_DW BIT(31)

#define SPI_DMA_CTR_EN   BIT(0)
#define SPI_DMA_CTR_PCLR BIT(2)
#define SPI_DMA_CTR_DRE  BIT(30)
#define SPI_DMA_CTR_DTE  BIT(31)

#define SPI_DMA_ICR_IED BIT(17)
#define SPI_DMA_ICR_ISD BIT(29)

#define SPI_WORD_BYTES 4

int transceive(uint8_t* tx_buf, uint8_t* rx_buf, uint32_t length);
void setup_dma();

#endif /* SPI_H */