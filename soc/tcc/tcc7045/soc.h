/*
 * Copyright 2024 Hounjoung Rim <hounjoung@tsnlab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _BOARD_SOC__H_
#define _BOARD_SOC__H_

/* Define CMSIS configurations */
#define __CR_REV 1U

#ifndef FALSE
#define FALSE (0U)
#endif

#ifndef TRUE
#define TRUE (1U)
#endif

#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif

#ifndef NULL
#define NULL (0)
#endif

#ifndef ON
#define ON  (TRUE)
#define OFF (FALSE)
#endif

/* sal_com start */
#define SALDisabled (FALSE)
#define SALEnabled  (TRUE)

/*
 * Error Codes
 */
enum vcp_error_code {
	VCP_ERR_INIT = 100,              /**< Initialization error for each APIs             */
	VCP_ERR_NO_SPACE = 101,          /**< No more space (memory, channel, etc)           */
	VCP_ERR_INVALID_PARAMETER = 102, /**< Invalid parameter is passed                    */
	VCP_ERR_NOT_SUPPORT = 103,       /**< Not supported operation or resources           */
	VCP_ERR_TIMEOUT = 104,           /**< Timed out while processing                     */
	VCP_ERR_INVALID_HANDLE = 105,    /**< Invalid handle is detected                     */
	VCP_ERR_NO_DATA = 106,           /**< No data                                        */
	VCP_ERR_UNDEF_STATE = 107,       /**< Not defined state                              */
	VCP_ERR_FAIL_CREATE = 108,       /**< Fail to create a component(Task, event, etc)   */
	VCP_ERR_FAIL_GET_DATA = 109,     /**< Fail to get data from a component              */
	VCP_ERR_FAIL_SEND_DATA = 110,    /**< Fail to send data to a component               */
	VCP_ERR_FAIL_START = 111,        /**< Fail to start a component                      */
	VCP_ERR_FAIL_DELETE = 112,       /**< Fail to delete a job of a component            */
	VCP_ERR_FAIL_RELEASE = 113,      /**< Fail to release a job of a component           */
	VCP_ERR_UNINIT_ITEM = 114,       /**< Uninitialized item, variable, or contents      */
	VCP_ERR_OUTOF_SIZE = 115,        /**< Size overflow or is lower than threshold       */
	VCP_ERR_FAIL_GET_KEY = 116,      /**< Fail to get key or ownership                   */
	VCP_ERR_FAIL_SET_CONFIG = 117,   /**< Fail to set configuration or status            */
	VCP_ERR_NOT_USEFUL = 118         /**< The status is not available                    */
};

typedef uint32_t (*SALCoreMB)(void);

uint32_t soc_cpu_dsb(void);
uint32_t soc_div64_to_32(unsigned long long *pullDividend, uint32_t uiDivisor, uint32_t *puiRem);

typedef unsigned char boolean; /* for use with TRUE/FALSE        */

#define MCU_BSP_UART_BASE  (0xA0200000UL)
#define MCU_BSP_GDMA_BASE  (0xA0800000UL)
#define MCU_BSP_TIC_BASE   (0xA0F10000UL)
#define MCU_BSP_GPIO_BASE  (0xA0F22000UL)
#define CLOCK_BASE_ADDR    (0xA0F24000UL)
#define MCU_BSP_PMIO_BASE  (0xA0F28800UL)
#define MCU_BSP_TIMER_BASE (0xA0F2A000UL)

#define SYS_PWR_EN (GPIO_GPC(2UL))

#define TEST_LED_BLINK (GPIO_GPB(4UL))

#endif /* _BOARD_SOC__H_ */
