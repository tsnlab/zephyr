
#include <pmio.h>
#include <pmio_dev.h>
/*************************************************************************************************
 *                                             DEFINITIONS
 *************************************************************************************************/
#ifdef PMIO_CONF_DET_BU
#define PMIO_VA_BU_DET (4UL)

#define PMIO_VA_BU_DET_PORT (PMIO_GPK(PMIO_VA_BU_DET))
#define PMIO_VA_BU_DET_GPK  (GPIO_GPK(PMIO_VA_BU_DET))
#else
#define PMIO_VA_BU_DET_PORT (PMIO_VA_BITCLEAR)
#define PMIO_VA_BU_DET_GPK  (0UL)
#endif

#ifdef PMIO_CONF_DET_ACC
//#define PMIO_FEATURE_ACC_TIMER

#define PMIO_VA_ACC_DET (5UL)

#define PMIO_VA_ACC_DET_PORT (PMIO_GPK(PMIO_VA_ACC_DET))
#define PMIO_VA_ACC_DET_GPK  (GPIO_GPK(PMIO_VA_ACC_DET))
#else
#define PMIO_VA_ACC_DET_PORT (PMIO_VA_BITCLEAR)
#define PMIO_VA_ACC_DET_GPK  (0UL)
#endif

#ifdef PMIO_CONF_DET_CAN
#define PMIO_VA_CAN_DET (1UL)

#define PMIO_VA_CAN_DET_PORT (PMIO_GPK(PMIO_VA_CAN_DET))
#define PMIO_VA_CAN_DET_GPK  (GPIO_GPK(PMIO_VA_CAN_DET))
#else
#define PMIO_VA_CAN_DET_PORT (PMIO_VA_BITCLEAR)
#define PMIO_VA_CAN_DET_GPK  (0UL)
#endif

#if defined(DEBUG_ENABLE) || defined(PMIO_FEATURE_USE_DEBUG_ALONE)
#define PMIO_D(args...)                                                                            \
	{                                                                                          \
		mcu_printf("[PMIO][%s:%d] ", __func__, __LINE__);                                  \
		mcu_printf(args);                                                                  \
	}
#else
#define PMIO_D(args...)
#endif

static void PMIO_EnTrimData(unsigned long uiSwEn, unsigned long uiAwoEn);

static void PMIO_SetPower(PMIOMode_t tMode);

static PMIOMode_t PMIO_GetBootMode(void);

// for debugging power status
static unsigned long gModeForDbg = 0UL;
static unsigned long gReasonForDbg = 0UL;

static void PMIO_EnTrimData(unsigned long uiSwEn, unsigned long uiAwoEn)
{
	if ((uiSwEn == 1UL) || (uiAwoEn == 1UL)) {
		PMIO_REG_PMGPIO_CTL |= PMIO_VA_BIT28;
	} else {
		PMIO_REG_PMGPIO_CTL &= ~(PMIO_VA_BIT28);
	}

	if (uiSwEn == 1UL) {
		PMIO_REG_PMGPIO_CTL |= (PMIO_VA_BIT27 | PMIO_VA_BIT25);
	} else {
		PMIO_REG_PMGPIO_CTL &= ~(PMIO_VA_BIT27 | PMIO_VA_BIT25);
	}

	if (uiAwoEn == 1UL) {
		PMIO_REG_PMGPIO_CTL |= (PMIO_VA_BIT26 | PMIO_VA_BIT24);
	} else {
		PMIO_REG_PMGPIO_CTL &= ~(PMIO_VA_BIT26 | PMIO_VA_BIT24);
	}
}

/* **********************************************************************************************/
/*                                             EXTERN FUNCTION                                  */
/* **********************************************************************************************/

void PMIO_EarlyWakeUp(void)
{
	PMIOReason_t tReason;
	PMIOMode_t tMode;

	tReason = PMIO_GetBootReason();
	tMode = PMIO_GetBootMode();

	gReasonForDbg = (unsigned long)tReason;
	gModeForDbg = (unsigned long)tMode;

	PMIO_SetPower(PMIO_MODE_POWER_ON);
}

static void PMIO_SetPower(PMIOMode_t tMode)
{
	switch (tMode) {
	case PMIO_MODE_NOT_SET: {
		PMIO_D("power mode not set\n");
		break;
	}
	case PMIO_MODE_POWER_ON: {
		PMIO_EnTrimData(1UL, 1UL);

		// retention off, init off, power on
		PMIO_REG_SW_LDO10 = (0UL);
		break;
	}
	case PMIO_MODE_POWER_DOWN:
	case PMIO_MODE_POWER_DEEP_DOWN: {
		PMIO_EnTrimData(1UL, 1UL);

		// retention off, init on, power off
		PMIO_REG_SW_LDO10 = (1UL);
		break;
	}
	default: {
		PMIO_D("Unknown Power Mode %d\n", tMode);
		break;
	}
	}
}

static PMIOMode_t PMIO_GetBootMode(void)
{
	PMIOMode_t tMode;

	tMode = PMIO_MODE_POWER_ON;

	return tMode;
}

PMIOReason_t PMIO_GetBootReason(void)
{
	PMIOReason_t tBoot;

	tBoot = PMIO_REASON_NOT_SET;

	if ((PMIO_REG_LVSE_EN & PMIO_VA_BIT8) != 0UL) {
		tBoot = PMIO_REASON_RTC_ON;
	} else if ((PMIO_REG_PMGPIO_STR & (unsigned long)PMIO_VA_BU_DET_PORT) == 0UL) {
		if ((PMIO_REG_PMGPIO_STF & (unsigned long)PMIO_VA_BU_DET_PORT) == 0UL) {
			if ((PMIO_REG_PMGPIO_STR & PMIO_VA_ACC_DET_PORT) != 0UL) {
				tBoot = PMIO_REASON_ACC_ON;
			} else if ((PMIO_REG_PMGPIO_STF & PMIO_VA_ACC_DET_PORT) != 0UL) {
				tBoot = PMIO_REASON_ACC_ON;
			} else {
				if ((PMIO_REG_PMGPIO_STR & PMIO_VA_CAN_DET_PORT) != 0UL) {
					tBoot = PMIO_REASON_CAN_ON;
				}

				if ((PMIO_REG_PMGPIO_STF & PMIO_VA_CAN_DET_PORT) != 0UL) {
					tBoot = PMIO_REASON_CAN_ON;
				}
			}
		}
	} else {
		tBoot = PMIO_REASON_BU_ON;
	}

	if (tBoot == PMIO_REASON_NOT_SET) {
		tBoot = PMIO_REASON_BU_ON;
	}

	return tBoot;
}
