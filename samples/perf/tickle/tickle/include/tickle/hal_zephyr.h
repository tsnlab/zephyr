#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>

struct tt_hal {
    /* FIXME: This is used since network feature does not work on VCP-G yet */
    struct spi_dt_spec spi;
};
