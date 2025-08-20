#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/arm64/misc.h>

#include "spi.h"

static inline void get_ths(uint32_t* wth, uint32_t* rth) {
    uint32_t status = sys_read32(SPI_STAT(SPI_BASE));
    *wth = (status & (0x1 << 1)) >> 1;
    *rth = (status & (0x1 << 0)) >> 0;
}

int transceive(uint8_t* tx_buf, uint8_t* rx_buf, uint32_t length) {
    if (tx_buf == NULL || rx_buf == NULL || length == 0) {
        return -1;
    }

    uint32_t tx_offset = 0, rx_offset = 0;
    uint32_t wth, rth;
    uint32_t spi_data;
    get_ths(&wth, &rth);
    while (tx_offset < length || rx_offset < length) {
        if (tx_offset < length && wth != 0) {
            spi_data = sys_get_be32(&tx_buf[tx_offset]);
            sys_write32(spi_data, SPI_DATA(SPI_BASE));
            tx_offset += SPI_WORD_BYTES;
        }
        if (rx_offset < length && rth != 0) {
            spi_data = sys_read32(SPI_DATA(SPI_BASE));
            sys_put_be32(spi_data, &rx_buf[rx_offset]);
            rx_offset += SPI_WORD_BYTES;
        }

        get_ths(&wth, &rth);
    }

    return 0;
}

void setup_dma() {
	/* Configure DMA settings */
	sys_write32(0, SPI_DMA_CTRL(SPI_BASE));  /* Clear everything */
	sys_write32(SPI_INTEN_DW | SPI_INTEN_DR, SPI_INTEN(SPI_BASE));  /* TODO: This might be unnecessary */
	sys_write32(SPI_DMA_CTR_PCLR, SPI_DMA_CTRL(SPI_BASE));  /* Clear state bits */

	uint32_t dma_icr = sys_read32(SPI_DMA_ICR(SPI_BASE));
	dma_icr |= SPI_DMA_ICR_IED;
	sys_write32(dma_icr, SPI_DMA_ICR(SPI_BASE));
}
