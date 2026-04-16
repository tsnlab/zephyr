#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_DW_BURST_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_DW_BURST_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int spi_dw_write_burst(const struct device *dev,
			 const struct spi_config *config,
			 const uint8_t *tx_buf,
			 size_t len);

int spi_dw_write_burst_dt(const struct spi_dt_spec *spec,
			    const uint8_t *tx_buf,
			    size_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_SPI_DW_BURST_H_ */