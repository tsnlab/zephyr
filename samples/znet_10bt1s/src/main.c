#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/kernel/mm.h>

/* RP1 base */
#define RP1_GPIO_PHYS   0x1f000d0000ULL
#define RP1_SPI0_PHYS   0x1f00050000ULL

#define MAP_SIZE        0x1000

/* SPI registers */
#define DW_CTRLR0   0x00
#define DW_SSIENR   0x08
#define DW_SER      0x10
#define DW_BAUDR    0x14
#define DW_SR       0x28
#define DW_DR       0x60

/* SR bits */
#define SR_TFNF (1 << 1)
#define SR_RFNE (1 << 3)

/* GPIO (RP1 simplified offsets) */
#define GPIO_DIR   0x04
#define GPIO_OUT   0x08

static inline void gpio_set_output(uint8_t *gpio, int pin)
{
    uint32_t v = sys_read32((uintptr_t)gpio + GPIO_DIR);
    v |= (1 << pin);
    sys_write32(v, (uintptr_t)gpio + GPIO_DIR);
}

static inline void gpio_set(uint8_t *gpio, int pin)
{
    uint32_t v = sys_read32((uintptr_t)gpio + GPIO_OUT);
    v |= (1 << pin);
    sys_write32(v, (uintptr_t)gpio + GPIO_OUT);
}

static inline void gpio_clear(uint8_t *gpio, int pin)
{
    uint32_t v = sys_read32((uintptr_t)gpio + GPIO_OUT);
    v &= ~(1 << pin);
    sys_write32(v, (uintptr_t)gpio + GPIO_OUT);
}

void main(void)
{
    uint8_t *gpio;
    uint8_t *spi;

    printk("\n=== LAN865x Raw SPI Test ===\n");

    /* MMIO map */
    k_mem_map_phys_bare(&gpio, RP1_GPIO_PHYS, MAP_SIZE,
                        K_MEM_PERM_RW | K_MEM_CACHE_NONE);

    k_mem_map_phys_bare(&spi, RP1_SPI0_PHYS, MAP_SIZE,
                        K_MEM_PERM_RW | K_MEM_CACHE_NONE);

    printk("GPIO VA = %p\n", gpio);
    printk("SPI VA  = %p\n", spi);

    /* GPIO8 = CS */
    gpio_set_output(gpio, 8);

    /* GPIO22 = Reset */
    gpio_set_output(gpio, 22);

    /* Reset sequence */
    printk("Reset LOW\n");
    gpio_clear(gpio, 22);
    k_msleep(200);

    printk("Reset HIGH\n");
    gpio_set(gpio, 22);
    k_msleep(1000);   // 중요: 충분히 대기

    /* SPI Disable */
    sys_write32(0x0, (uintptr_t)spi + DW_SSIENR);

    /* Mode 0, 8-bit */
    sys_write32(0x00070000, (uintptr_t)spi + DW_CTRLR0);

    /* 4MHz (divider=50 가정) */
    sys_write32(50, (uintptr_t)spi + DW_BAUDR);

    /* Enable SPI */
    sys_write32(0x1, (uintptr_t)spi + DW_SSIENR);

    printk("SPI configured\n");

    /* CS LOW */
    printk("CS LOW\n");
    gpio_clear(gpio, 8);
    k_msleep(10);

    for (int i = 0; i < 10; i++) {

        /* TX FIFO empty wait */
        while (!(sys_read32((uintptr_t)spi + DW_SR) & SR_TFNF));

        /* Send dummy */
        sys_write32(0xFF, (uintptr_t)spi + DW_DR);

        /* RX available wait */
        while (!(sys_read32((uintptr_t)spi + DW_SR) & SR_RFNE));

        uint32_t rx = sys_read32((uintptr_t)spi + DW_DR);

        printk("RX[%d] = 0x%02x\n", i, rx & 0xFF);
    }

    /* CS HIGH */
    printk("CS HIGH\n");
    gpio_set(gpio, 8);

    printk("=== Test Done ===\n");
}
