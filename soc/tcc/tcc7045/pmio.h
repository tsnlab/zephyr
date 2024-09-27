#ifndef MCU_BSP_PMIO_HEADER
#define MCU_BSP_PMIO_HEADER

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/

/* CONFIGURATION for PMIO FUNCTION ===================================================*/
/*
    PMIO_CONF_DEBUG_ALONE   : [Def] Print always about errors and debugs log.
					(No use a LOG_D/E console.)

    PMIO_CONF_PREVENT_LEAK  : [Def] Prevent power current leakage
					when sys enter power down.
					The settings are tailored to TC EVM environment only.

    PMIO_CONF_DET_BU        : [Def] Detect BU(GPK4) signal.
					refer to sources/dev.drivers/pmio/tcc70xx/pmio_dev.c
					"#define PMIO_VA_BU_DET  (4UL)"

    PMIO_CONF_DET_ACC       : [Def] Detect ACC(GPK5) signal.
					refer to sources/dev.drivers/pmio/tcc70xx/pmio_dev.c
					"#define PMIO_VA_ACC_DET  (5UL)"

    PMIO_CONF_DET_CAN       : [Def] Detect CAN(GPK1) signal.
					refer to sources/dev.drivers/pmio/tcc70xx/pmio_dev.c
					"#define PMIO_VA_CAN_DET  (1UL)"

    PMIO_CONF_RTC_WAKE_UP   : [Def] Run RTC Wake-up when retention mode lasts for a certain time.
				    After RTC Wake-up, system enters Deep power down immediately.
					Need to support S/W RTC driver.

    PMIO_CONF_LDO_TRIM_CTL  : [Def] Use LDO_TRIM_DATA. When system power down , efuse is power-off.
				    This config maintains the LDO TRIM behalf of efuse, when system
   power down.

    PMIO_CONF_MISC_DEV_FUNC : [Def] Supports miscellaneous test functions for further development.
*/

#define PMIO_CONF_DEBUG_ALONE
#define PMIO_CONF_PREVENT_LEAK
#define PMIO_CONF_LDO_TRIM_CTL
#define PMIO_CONF_DET_BU
#define PMIO_CONF_DET_ACC
//#define PMIO_CONF_DET_CAN
//#define PMIO_CONF_RTC_WAKE_UP
//#define PMIO_CONF_MISC_DEV_FUNC

/* =================================================== CONFIGURATION for PMIO FUNCTION*/

/*PMIO interrupt*/
#define PMIO_VA_INTERRUPT_SRC_BU  ((unsigned long)(GIC_EXT0))
#define PMIO_VA_INTERRUPT_SRC_ACC ((unsigned long)(GIC_EXT1))
#define PMIO_VA_INTERRUPT_SRC_CAN ((unsigned long)(GIC_EXT2))
#define PMIO_VA_INTERRUPT_SRC_ALL ((unsigned long)(0UL))

#define PMIO_GPK(x)               ((unsigned long)1 << (unsigned long)x)
#define PMIO_FUNC_MSEC_TO_USEC(x) ((unsigned long)(x) * (unsigned long)(1000UL))
#define PMIO_FUNC_SEC_TO_USEC(x)                                                                   \
	((unsigned long)(x) * (unsigned long)(1000UL) * (unsigned long)(1000UL))

/*PMIO enum*/
typedef enum {
	PMIO_MODE_NOT_SET = 0,
	PMIO_MODE_POWER_ON = 1,
	PMIO_MODE_POWER_DOWN = 2,
	PMIO_MODE_POWER_DEEP_DOWN = 3,
	PMIO_MODE_MAX = 4
} PMIOMode_t;

typedef enum {
	PMIO_REASON_NOT_SET = 0,
	PMIO_REASON_BU_ON = 1,
	PMIO_REASON_ACC_ON = 2,
	PMIO_REASON_CAN_ON = 3,
	PMIO_REASON_RTC_ON = 4,
	PMIO_REASON_MAX = 5
} PMIOReason_t;

typedef enum {
	PMIO_PORT_SEL_RELEASE = 0,

	PMIO_PORT_SEL_GPIO = 1,

	PMIO_PORT_SEL_PMGPIO_IN = 2,
	PMIO_PORT_SEL_PMGPIO_OUT_L = 3,
	PMIO_PORT_SEL_PMGPIO_OUT_H = 4,
	PMIO_PORT_SEL_PMGPIO_PU = 5,
	PMIO_PORT_SEL_PMGPIO_PD = 6
} PMIOPortSel_t;

typedef void (*POWERAppHandler)(void);
/*
***************************************************************************************************
*                                             LOCAL VARIABLES
***************************************************************************************************
*/

/**************************************************************************************************/
/*                                         FUNCTION PROTOTYPES                                    */
/**************************************************************************************************/

void PMIO_EarlyWakeUp(void);

PMIOReason_t PMIO_GetBootReason(void);

#endif // MCU_BSP_PMIO_HEADER
