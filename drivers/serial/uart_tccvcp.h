/*
 * Copyright (c) 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_TCCVCP_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_TCCVCP_H_
#include <zephyr/types.h>

#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif

#ifndef NULL
#define NULL (0)
#endif

#define MCU_BSP_UART_BASE (0xA0200000UL)
#define MCU_BSP_GDMA_BASE (0xA0800000UL)

#ifndef FALSE
#define FALSE (0U)
#endif

#ifndef TRUE
#define TRUE (1U)
#endif

#ifndef ON
#define ON  (TRUE)
#define OFF (FALSE)
#endif

#define SALDisabled (FALSE)
#define SALEnabled  (TRUE)

#define GIC_PRIORITY_NO_MEAN (16UL)

#define UART_POLLING_MODE (0U)
#define UART_INTR_MODE    (1U)
#define UART_DMA_MODE     (2U)

#define UART_CTSRTS_ON  (1U)
#define UART_CTSRTS_OFF (0U)

#define ENABLE_FIFO  (1U)
#define DISABLE_FIFO (0U)

#define TWO_STOP_BIT_ON  (1U)
#define TWO_STOP_BIT_OFF (0U)

#define UART_DEBUG_CH  (UART_CH1)
#define UART_DEBUG_CLK (48000000UL) /* 48MHz */

/* UART Channels */
#define UART_CH0    (0U)
#define UART_CH1    (1U)
#define UART_CH2    (2U)
#define UART_CH3    (3U)
#define UART_CH4    (4U)
#define UART_CH5    (5U)
#define UART_CH_MAX (6U)

/* UART Base address */
#define UART_GET_BASE(n) (MCU_BSP_UART_BASE + (0x10000UL * (n)))

/* UART Register (BASE Address + Offset) */
#define UART_REG_DR    (0x00U) /* Data register */
#define UART_REG_RSR   (0x04U) /* Receive Status register */
#define UART_REG_ECR   (0x04U) /* Error Clear register */
#define UART_REG_FR    (0x18U) /* Flag register */
#define UART_REG_IBRD  (0x24U) /* Integer Baud rate register */
#define UART_REG_FBRD  (0x28U) /* Fractional Baud rate register */
#define UART_REG_LCRH  (0x2cU) /* Line Control register */
#define UART_REG_CR    (0x30U) /* Control register */
#define UART_REG_IFLS  (0x34U) /* Interrupt FIFO Level status register */
#define UART_REG_IMSC  (0x38U) /* Interrupt Mask Set/Clear register */
#define UART_REG_RIS   (0x3cU) /* Raw Interrupt Status register */
#define UART_REG_MIS   (0x40U) /* Masked Interrupt Status register */
#define UART_REG_ICR   (0x44U) /* Interrupt Clear register */
#define UART_REG_DMACR (0x48U) /* DMA Control register */

/* UART Flag Register(FR) Fields */
#define UART_FR_TXFE (1UL << 7U) /* Transmit FIFO empty */
#define UART_FR_RXFF (1UL << 6U) /* Receive FIFO full */
#define UART_FR_TXFF (1UL << 5U) /* Transmit FIFO full */
#define UART_FR_RXFE (1UL << 4U) /* Receive FIFO empty */
#define UART_FR_BUSY (1UL << 3U) /* UART busy */
#define UART_FR_CTS  (1UL << 0U) /* Clear to send */

/* UART Line Control Register (LCR_H) Fields */
#define UART_LCRH_SPS     (1UL << 7U) /* Stick parity select */
#define UART_LCRH_WLEN(x) ((x) << 5U) /* Word length */
#define UART_LCRH_FEN     (1UL << 4U) /* Enable FIFOs */
#define UART_LCRH_STP2    (1UL << 3U) /* Two stop bits select */
#define UART_LCRH_EPS     (1UL << 2U) /* Even parity select */
#define UART_LCRH_PEN     (1UL << 1U) /* Parity enable */
#define UART_LCRH_BRK     (1UL << 0U) /* Send break */

/* UART Control Register (CR) Fields */
#define UART_CR_CTSEN (1UL << 15U) /* CTS hardware flow control enable */
#define UART_CR_RTSEN (1UL << 14U) /* RTS hardware flow control enable */
#define UART_CR_RTS   (1UL << 11U) /* Request to send */
#define UART_CR_RXE   (1UL << 9U)  /* Receive enable */
#define UART_CR_TXE   (1UL << 8U)  /* Transmit enable */
#define UART_CR_LBE   (1UL << 7U)  /* Loopback enable */
#define UART_CR_EN    (1UL << 0U)  /* UART enable */

#define UART_TX_FIFO_SIZE (8UL)
#define UART_RX_FIFO_SIZE (12UL)

#define UART_INT_OEIS (1UL << 10U) /* Overrun error interrupt */
#define UART_INT_BEIS (1UL << 9U)  /* Break error interrupt */
#define UART_INT_PEIS (1UL << 8U)  /* Parity error interrupt */
#define UART_INT_FEIS (1UL << 7U)  /* Framing error interrupt */
#define UART_INT_RTIS (1UL << 6U)  /* Receive timeout interrupt */
#define UART_INT_TXIS (1UL << 5U)  /* Transmit interrupt */
#define UART_INT_RXIS (1UL << 4U)  /* Receive interrupt */

/* UART Settings */
#define UART_BUFF_SIZE (0x100UL) /* 256 */

#define UART_MODE_TX (0UL)
#define UART_MODE_RX (1UL)

#define UART_PORT_CFG_MAX  (16U)
#define UART_PORT_TBL_SIZE (UART_PORT_CFG_MAX)

/* DMA Control Register (DMACR) Fields */
#define UART_DMACR_DMAONERR (1UL << 2U) /* DMA on error */
#define UART_DMACR_TXDMAE   (1UL << 1U) /* Transmit DMA enable */
#define UART_DMACR_RXDMAE   (1UL << 0U) /* Receive DMA enable */

#define TCC_GPNONE   (0xFFFFUL)
#define TCC_GPSD0(x) (TCC_GPNONE)
#define TCC_GPSD1(x) (TCC_GPNONE)
#define TCC_GPSD2(x) (TCC_GPNONE)

#define GIC_INT_TYPE_LEVEL_HIGH   (0x1U)
#define GIC_INT_TYPE_LEVEL_LOW    (0x2U)
#define GIC_INT_TYPE_EDGE_RISING  (0x4U)
#define GIC_INT_TYPE_EDGE_FALLING (0x8U)
#define GIC_INT_TYPE_EDGE_BOTH    (GIC_INT_TYPE_EDGE_RISING | GIC_INT_TYPE_EDGE_FALLING)

#define GIC_PPI_START (16UL)
#define GIC_SPI_START (32UL)

#define GIC_INT_TYPE_LEVEL_HIGH   (0x1U)
#define GIC_INT_TYPE_LEVEL_LOW    (0x2U)
#define GIC_INT_TYPE_EDGE_RISING  (0x4U)
#define GIC_INT_TYPE_EDGE_FALLING (0x8U)
#define GIC_INT_TYPE_EDGE_BOTH    (GIC_INT_TYPE_EDGE_RISING | GIC_INT_TYPE_EDGE_FALLING)

#define GIC_EINT_START_INT (GIC_EXT0)
#define GIC_EINT_END_INT   (GIC_EXT9)
#define GIC_EINT_NUM       ((uint32_t)GIC_EXTn0 - (uint32_t)GIC_EXT0)

#define GDMA_CON_MAX (8UL)
#define GDMA_CH_MAX  (2UL)

/* UART DMA Register offsets */
#define GDMA_INTSR  (0x00UL) /* Interrupt Status Register */
#define GDMA_ITCSR  (0x04UL) /* Interrupt Terminal Count Status Register */
#define GDMA_ITCCR  (0x08UL) /* Interrupt Terminal Count Clear Register */
#define GDMA_IESR   (0x0CUL) /* Interrupt Error Status Register */
#define GDMA_IECR   (0x10UL) /* Interrupt Error Clear Register */
#define GDMA_RITCSR (0x14UL) /* Raw Interrupt Terminal Count Status Register */
#define GDMA_REISR  (0x18UL) /* Raw Error Interrupt Status Register */
#define GDMA_ECR    (0x1CUL) /* Enabled Channel Register */
#define GDMA_SBRR   (0x20UL) /* Software Burst Request Register */
#define GDMA_SSRR   (0x24UL) /* Software Single Request Register */
#define GDMA_SLBRR  (0x28UL) /* Software Last Burst Request Register */
#define GDMA_SLSRR  (0x2CUL) /* Software Last Single Request Register */
#define GDMA_CR     (0x30UL) /* Configuration Register */
#define GDMA_SR     (0x34UL) /* Reserved */

#define GDMA_CON_BASE(x) ((uint32_t)MCU_BSP_GDMA_BASE + ((x)*0x10000UL))

#define GDMA_CH_SRC_ADDR(x) (((uint32_t)0x100UL + ((x)*0x20UL)))
#define GDMA_CH_DST_ADDR(x) (((uint32_t)0x104UL + ((x)*0x20UL)))
#define GDMA_CH_LLI(x)      (((uint32_t)0x108UL + ((x)*0x20UL)))
#define GDMA_CH_CON(x)      (((uint32_t)0x10CUL + ((x)*0x20UL)))
#define GDMA_CH_CONFIG(x)   (((uint32_t)0x110UL + ((x)*0x20UL)))

#define GDMA_BUFF_SIZE   (0x3ffUL) /* 1023. */
#define GDMA_BUFF_MARGIN (0x0UL)

#define GDMA_CH0 (0UL)
#define GDMA_CH1 (1UL)

#define GDMA_INC    (1U)
#define GDMA_NO_INC (0U)

#define GDMA_TRANSFER_SIZE_BYTE (0U)
#define GDMA_TRANSFER_SIZE_HALF (1U)
#define GDMA_TRANSFER_SIZE_WORD (2U)

#define GDMA_BURST_SIZE_1   (0U)
#define GDMA_BURST_SIZE_4   (1U)
#define GDMA_BURST_SIZE_8   (2U)
#define GDMA_BURST_SIZE_16  (3U)
#define GDMA_BURST_SIZE_32  (4U)
#define GDMA_BURST_SIZE_64  (5U)
#define GDMA_BURST_SIZE_128 (6U)
#define GDMA_BURST_SIZE_256 (7U)

/* GDMA Address */
#define GDMA_ADDRESS_OFFSET (0UL)
#define GDMA_ADDRESS_UNIT   (GDMA_BUFF_SIZE + GDMA_BUFF_MARGIN)
#define GDMA_ADDRESS_UNIT_CH_RX(ch)                                                                \
	((uint32_t)GDMA_ADDRESS_OFFSET + ((GDMA_ADDRESS_UNIT * 2UL) * (ch)))
#define GDMA_ADDRESS_UNIT_CH_TX(ch)                                                                \
	(((uint32_t)GDMA_ADDRESS_OFFSET + ((GDMA_ADDRESS_UNIT * 2UL) * (ch))) + GDMA_ADDRESS_UNIT)

#define GDMA_FLOW_TYPE_M2M       (0UL)
#define GDMA_FLOW_TYPE_M2P       (1UL)
#define GDMA_FLOW_TYPE_P2M       (2UL)
#define GDMA_FLOW_TYPE_P2P       (3UL)
#define GDMA_FLOW_TYPE_P2P_BY_DP (4UL)
#define GDMA_FLOW_TYPE_M2P_BY_P  (5UL)
#define GDMA_FLOW_TYPE_P2M_BY_P  (6UL)
#define GDMA_FLOW_TYPE_P2P_BY_SP (7UL)

#define GDMA_PERI_REQ_PORT_UART0_RX (0UL)
#define GDMA_PERI_REQ_PORT_UART0_TX (1UL)

#define GDMA_PERI_REQ_PORT_UART1_RX (0UL)
#define GDMA_PERI_REQ_PORT_UART1_TX (1UL)

#define GDMA_PERI_REQ_PORT_UART2_RX (0UL)
#define GDMA_PERI_REQ_PORT_UART2_TX (1UL)

#define GDMA_PERI_REQ_PORT_UART3_RX (0UL)
#define GDMA_PERI_REQ_PORT_UART3_TX (1UL)

#define GDMA_PERI_REQ_PORT_UART4_RX (0UL)
#define GDMA_PERI_REQ_PORT_UART4_TX (1UL)

#define GDMA_PERI_REQ_PORT_UART5_RX (0UL)
#define GDMA_PERI_REQ_PORT_UART5_TX (1UL)
/* MICOM Subsystem Register Offsets */
#define CLOCK_MCKC_HCLK0            (0x000)
#define CLOCK_MCKC_HCLK1            (0x004)
#define CLOCK_MCKC_HCLK2            (0x008)
#define CLOCK_MCKC_HCLKSWR0         (0x00C)
#define CLOCK_MCKC_HCLKSWR1         (0x010)
#define CLOCK_MCKC_HCLKSWR2         (0x014)

#define GPIO_PERICH_CH0 (0)
#define GPIO_PERICH_CH1 (1)
#define GPIO_PERICH_CH2 (2)
#define GPIO_PERICH_CH3 (3)

#define GPIO_OUTPUT_SHIFT (9)
#define GPIO_OUTPUT       (1UL << (uint32_t)GPIO_OUTPUT_SHIFT)
#define GPIO_INPUT        (0UL << (uint32_t)GPIO_OUTPUT_SHIFT)

#define GPIO_MFIO_CFG_CH_SEL0     (0)
#define GPIO_MFIO_CFG_PERI_SEL0   (4)
#define GPIO_MFIO_CFG_CH_SEL1     (8)
#define GPIO_MFIO_CFG_PERI_SEL1   (12)
#define GPIO_MFIO_CFG_CH_SEL2     (16)
#define GPIO_MFIO_CFG_PERI_SEL2   (20)
#define GPIO_MFIO_DISABLE         (0)
#define GPIO_MFIO_SPI2            (1)
#define GPIO_MFIO_UART3           (2)
#define GPIO_MFIO_I2C3            (3)
#define GPIO_MFIO_SPI3            (1)
#define GPIO_MFIO_UART4           (2)
#define GPIO_MFIO_I2C4            (3)
#define GPIO_MFIO_SPI4            (1)
#define GPIO_MFIO_UART5           (2)
#define GPIO_MFIO_I2C5            (3)
#define GPIO_MFIO_CH0             (0)
#define GPIO_MFIO_CH1             (1)
#define GPIO_MFIO_CH2             (2)
#define GPIO_MFIO_CH3             (3)
#define GPIO_PERICH_SEL_UARTSEL_0 (0)
#define GPIO_PERICH_SEL_UARTSEL_1 (1)
#define GPIO_PERICH_SEL_UARTSEL_2 (2)
#define GPIO_PERICH_SEL_I2CSEL_0  (3)
#define GPIO_PERICH_SEL_I2CSEL_1  (4)
#define GPIO_PERICH_SEL_I2CSEL_2  (5)
#define GPIO_PERICH_SEL_SPISEL_0  (6)
#define GPIO_PERICH_SEL_SPISEL_1  (7)
#define GPIO_PERICH_SEL_I2SSEL_0  (8)
#define GPIO_PERICH_SEL_PWMSEL_0  (10)
#define GPIO_PERICH_SEL_PWMSEL_1  (12)
#define GPIO_PERICH_SEL_PWMSEL_2  (14)
#define GPIO_PERICH_SEL_PWMSEL_3  (16)
#define GPIO_PERICH_SEL_PWMSEL_4  (18)
#define GPIO_PERICH_SEL_PWMSEL_5  (20)
#define GPIO_PERICH_SEL_PWMSEL_6  (22)
#define GPIO_PERICH_SEL_PWMSEL_7  (24)
#define GPIO_PERICH_SEL_PWMSEL_8  (26)
#define GPIO_PERICH_CH0           (0)
#define GPIO_PERICH_CH1           (1)
#define GPIO_PERICH_CH2           (2)
#define GPIO_PERICH_CH3           (3)

typedef enum uart_word_len {
	WORD_LEN_5 = 0,
	WORD_LEN_6,
	WORD_LEN_7,
	WORD_LEN_8
} uart_word_len_t;

typedef enum uart_parity {
	PARITY_SPACE = 0,
	PARITY_EVEN,
	PARITY_ODD,
	PARITY_MARK
} uart_parity_t;

typedef struct uart_board_port {
	uint32_t bPortCfg; /* Config port ID */
	uint32_t bPortTx;  /* UT_TXD GPIO */
	uint32_t bPortRx;  /* UT_RXD GPIO */
	uint32_t bPortRts; /* UT_RTS GPIO */
	uint32_t bPortCts; /* UT_CTS GPIO */
	uint32_t bPortFs;  /* UART function select */
	uint32_t bPortCH;  /* Channel */
} uart_board_port_t;

typedef struct uart_interrupt_data {
	int8_t *iXmitBuf;
	int32_t iHead;
	int32_t iTail;
	int32_t iSize;
} uart_interrupt_data_t;

typedef void (*gic_isr_fn)(void *pArg);

typedef struct uart_param {
	uint8_t channel;
	uint32_t priority;           /* Interrupt priority */
	uint32_t baud_rate;          /* Baudrate */
	uint8_t mode;                /* polling or interrupt or dma */
	uint8_t cts_rts;             /* on/off */
	uint8_t port_cfg;            /* port selection */
	uint8_t fifo;                /* on/off */
	uint8_t stop_bit;            /* on/off */
	uart_word_len_t word_length; /* 5~8 bits */
	uart_parity_t parity;        /* space, even, odd, mark */
	gic_isr_fn callback_fn;      /* callback function */
} uart_param_t;

typedef void (*gdma_isr_callback_fn)(void *pArg);

typedef struct gdma_information {
	uint32_t iCon;
	uint32_t iCh;
	uint8_t *iSrcAddr;
	uint8_t *iDestAddr;
	uint32_t iBufSize;
	uint32_t iTransSize;
	gdma_isr_callback_fn fpIsrCallbackForComplete;
} gdma_information_t;

typedef struct GDMA_CONTROLLER {
	uint32_t cController;
	gdma_information_t *cCh[GDMA_CH_MAX];
} GDMAController_t;

typedef struct uart_status {
	unsigned char sIsProbed;
	uint32_t sBase;                /* UART Controller base address */
	uint8_t sCh;                   /* UART Channel */
	uint8_t sOpMode;               /* Operation Mode */
	uint8_t sCtsRts;               /* CTS and RTS */
	uint8_t s2StopBit;             /* 1: two stop bits are transmitted */
	uint32_t baudrate;             /* Baudrate setting in bps */
	uart_parity_t sParity;         /* 0:disable, 1:enable */
	uart_word_len_t sWordLength;   /* Word Length */
	uart_board_port_t sPort;       /* GPIO Port Information */
	gdma_information_t sRxDma;     /* Rx DMA */
	gdma_information_t sTxDma;     /* Tx DMA */
	uart_interrupt_data_t sRxIntr; /* Rx Interrupt */
	uart_interrupt_data_t sTxIntr; /* Tx Interrupt */
} uart_status_t;

/** Device configuration structure */
struct uart_tccvcp_dev_config {
	DEVICE_MMIO_ROM;
	uint8_t channel;
	uint32_t sys_clk_freq;
	uart_param_t uart_pars;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif
	uint32_t baud_rate;
};

/** Device data structure */
struct uart_tccvcp_dev_data_t {
	DEVICE_MMIO_RAM;
	uint32_t parity;
	uint32_t stopbits;
	uint32_t databits;
	uint32_t flowctrl;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
};

typedef void (*GICIsrFunc)(void *pArg);

typedef struct GICIntFuncPtr {
	GICIsrFunc ifpFunc;
	uint8_t ifpIsBothEdge;
	void *ifpArg;
} GICIntFuncPtr_t;

/* I/O Bus pwdn/swreset */
typedef enum CLOCK_IO_BUS {
	CLOCK_IOBUS_SFMC = 0,
	CLOCK_IOBUS_IMC = 1,
	CLOCK_IOBUS_PFLASH = 2,
	CLOCK_IOBUS_DFLASH = 3,
	CLOCK_IOBUS_GIC = 4,
	CLOCK_IOBUS_SOC400 = 5,
	CLOCK_IOBUS_DMA_CON0 = 6,
	CLOCK_IOBUS_DMA_CON1 = 7,
	CLOCK_IOBUS_DMA_CON2 = 8,
	CLOCK_IOBUS_DMA_CON3 = 9,
	CLOCK_IOBUS_DMA_CON4 = 10,
	CLOCK_IOBUS_DMA_CON5 = 11,
	CLOCK_IOBUS_DMA_CON6 = 12,
	CLOCK_IOBUS_DMA_CON7 = 13,
	CLOCK_IOBUS_CAN0 = 14,
	CLOCK_IOBUS_CAN1 = 15,
	CLOCK_IOBUS_CAN2 = 16,
	CLOCK_IOBUS_CAN_CONF = 17,
	CLOCK_IOBUS_UART0 = 18,
	CLOCK_IOBUS_UART1 = 19,
	CLOCK_IOBUS_UART2 = 20,
	CLOCK_IOBUS_UART3 = 21,
	CLOCK_IOBUS_UART4 = 22,
	CLOCK_IOBUS_UART5 = 23,
	CLOCK_IOBUS_CONF = 24,
	CLOCK_IOBUS_I2C0 = 25,
	CLOCK_IOBUS_I2C1 = 26,
	CLOCK_IOBUS_I2C2 = 27,
	CLOCK_IOBUS_I2C3 = 28,
	CLOCK_IOBUS_I2C4 = 29,
	CLOCK_IOBUS_I2C5 = 30,
	CLOCK_IOBUS_I2C_M_PORTCFG = 31,
	CLOCK_IOBUS_PWM0 = 32,      /* HCLK_MASK1[0] */
	CLOCK_IOBUS_PWM1 = 33,      /* HCLK_MASK1[1] */
	CLOCK_IOBUS_PWM2 = 34,      /* HCLK_MASK1[2] */
	CLOCK_IOBUS_PWM_CONF = 35,  /* HCLK_MASK1[3] */
	CLOCK_IOBUS_ICTC0 = 36,     /* HCLK_MASK1[4] */
	CLOCK_IOBUS_ICTC1 = 37,     /* HCLK_MASK1[5] */
	CLOCK_IOBUS_ICTC2 = 38,     /* HCLK_MASK1[6] */
	CLOCK_IOBUS_ICTC3 = 39,     /* HCLK_MASK1[7] */
	CLOCK_IOBUS_ICTC4 = 40,     /* HCLK_MASK1[8] */
	CLOCK_IOBUS_ICTC5 = 41,     /* HCLK_MASK1[9] */
	CLOCK_IOBUS_ADC0 = 42,      /* HCLK_MASK1[10] */
	CLOCK_IOBUS_ADC1 = 43,      /* HCLK_MASK1[11] */
	CLOCK_IOBUS_TIMER0 = 44,    /* HCLK_MASK1[12] */
	CLOCK_IOBUS_TIMER1 = 45,    /* HCLK_MASK1[13] */
	CLOCK_IOBUS_TIMER2 = 46,    /* HCLK_MASK1[14] */
	CLOCK_IOBUS_TIMER3 = 47,    /* HCLK_MASK1[15] */
	CLOCK_IOBUS_TIMER4 = 48,    /* HCLK_MASK1[16] */
	CLOCK_IOBUS_TIMER5 = 49,    /* HCLK_MASK1[17] */
	CLOCK_IOBUS_TIMER6 = 50,    /* HCLK_MASK1[18] */
	CLOCK_IOBUS_TIMER7 = 51,    /* HCLK_MASK1[19] */
	CLOCK_IOBUS_TIMER8 = 52,    /* HCLK_MASK1[20] */
	CLOCK_IOBUS_TIMER9 = 53,    /* HCLK_MASK1[21] */
	CLOCK_IOBUS_GPSB0 = 54,     /* HCLK_MASK1[22] */
	CLOCK_IOBUS_GPSB1 = 55,     /* HCLK_MASK1[23] */
	CLOCK_IOBUS_GPSB2 = 56,     /* HCLK_MASK1[24] */
	CLOCK_IOBUS_GPSB3 = 57,     /* HCLK_MASK1[25] */
	CLOCK_IOBUS_GPSB4 = 58,     /* HCLK_MASK1[26] */
	CLOCK_IOBUS_GPSB_CONF = 59, /* HCLK_MASK1[27] */
	CLOCK_IOBUS_GPSB_SM = 60,   /* HCLK_MASK1[28] */
	CLOCK_IOBUS_I2S = 61,       /* HCLK_MASK1[29] */
	CLOCK_IOBUS_GMAC = 62,      /* HCLK_MASK1[30] */
	CLOCK_IOBUS_RESERVED = 63,  /* HCLK_MASK1[31] */
	CLOCK_IOBUS_WDT = 64,       /* HCLK_MASK2[0] */
	CLOCK_IOBUS_GPIO = 65,      /* HCLK_MASK2[1] */
	CLOCK_IOBUS_CMU = 66,       /* HCLK_MASK2[2] */
	CLOCK_IOBUS_SYSSM = 67,     /* HCLK_MASK2[3] */
	CLOCK_IOBUS_MAX = 68
} CLOCKIobus_t;

typedef enum CLOCK_PERI { /* Peri. Name */
	/* MICOM Peri */
	CLOCK_PERI_SFMC = 0,
	CLOCK_PERI_CAN0 = 1,
	CLOCK_PERI_CAN1 = 2,
	CLOCK_PERI_CAN2 = 3,
	CLOCK_PERI_GPSB0 = 4,
	CLOCK_PERI_GPSB1 = 5,
	CLOCK_PERI_GPSB2 = 6,
	CLOCK_PERI_GPSB3 = 7,
	CLOCK_PERI_GPSB4 = 8,
	CLOCK_PERI_UART0 = 9,
	CLOCK_PERI_UART1 = 10,
	CLOCK_PERI_UART2 = 11,
	CLOCK_PERI_UART3 = 12,
	CLOCK_PERI_UART4 = 13,
	CLOCK_PERI_UART5 = 14,
	CLOCK_PERI_I2C0 = 15,
	CLOCK_PERI_I2C1 = 16,
	CLOCK_PERI_I2C2 = 17,
	CLOCK_PERI_I2C3 = 18,
	CLOCK_PERI_I2C4 = 19,
	CLOCK_PERI_I2C5 = 20,
	CLOCK_PERI_PWM0 = 21,
	CLOCK_PERI_PWM1 = 22,
	CLOCK_PERI_PWM2 = 23,
	CLOCK_PERI_ICTC0 = 24,
	CLOCK_PERI_ICTC1 = 25,
	CLOCK_PERI_ICTC2 = 26,
	CLOCK_PERI_ICTC3 = 27,
	CLOCK_PERI_ICTC4 = 28,
	CLOCK_PERI_ICTC5 = 29,
	CLOCK_PERI_ADC0 = 30,
	CLOCK_PERI_ADC1 = 31,
	CLOCK_PERI_TIMER0 = 32,
	CLOCK_PERI_TIMER1 = 33,
	CLOCK_PERI_TIMER2 = 34,
	CLOCK_PERI_TIMER3 = 35,
	CLOCK_PERI_TIMER4 = 36,
	CLOCK_PERI_TIMER5 = 37,
	CLOCK_PERI_TIMER6 = 38,
	CLOCK_PERI_TIMER7 = 39,
	CLOCK_PERI_TIMER8 = 40,
	CLOCK_PERI_TIMER9 = 41,
	CLOCK_PERI_I2S0 = 42,
	CLOCK_PERI_I2S1 = 43,
	CLOCK_PERI_GMAC0 = 44,
	CLOCK_PERI_GMAC1 = 45,
	CLOCK_PERI_MAX = 46
} CLOCKPeri_t;

enum {
	GDMA_PERI_RX = 0,
	GDMA_PERI_TX = 1
};

extern uint32_t __nc_dmastart;

SALRetCode_t FR_CoreMB(void);

int32_t CLOCK_SetIobusPwdn(int32_t iId, unsigned char bEn);

SALRetCode_t GPIO_Config(uint32_t uiPort, uint32_t uiConfig);

int32_t CLOCK_SetSwReset(int32_t iId, unsigned char bReset);
uint32_t CLOCK_GetPeriRate(int32_t iId);
int32_t CLOCK_SetPeriRate(int32_t iId, uint32_t uiRate);
int32_t CLOCK_EnablePeri(int32_t iId);
int32_t CLOCK_EnableIobus(int32_t iId, unsigned char bEn);
#endif /* ZEPHYR_DRIVERS_SERIAL_UART_TCCVCP_H_ */
