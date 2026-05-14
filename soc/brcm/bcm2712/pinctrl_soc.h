/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RP1 Hardware Register Definitions
 *
 * This header contains:
 *  - Register offsets
 *  - Bitfields
 *  - Override values
 *  - RIO registers
 */
#include <zephyr/types.h>
#include <zephyr/devicetree.h>

#ifndef ZEPHYR_SOC_BCM2712_PINCTRL_SOC_H_
#define ZEPHYR_SOC_BCM2712_PINCTRL_SOC_H_

/* Pinmux structure definition for RP1 */
typedef uint32_t pinctrl_soc_pin_t; /* 32-bit value representing a pinmux setting */

/* ============================================================
 * General HW Layout
 * ============================================================ */

/* Number of GPIO banks in RP1 */
#define RP1_NUM_BANKS 3

/* ============================================================
 * Register Offsets
 * ============================================================ */

#define RP1_GPIO_STATUS 0x0000
#define RP1_GPIO_CTRL   0x0004

#define RP1_RIO_OUT 0x00
#define RP1_RIO_OE  0x04
#define RP1_RIO_IN  0x08

/* ============================================================
 * GPIO_CTRL Bitfields
 * ============================================================ */

/* Function Select: Determines the function of the GPIO pin */
#define RP1_GPIO_CTRL_FUNCSEL_LSB  0
#define RP1_GPIO_CTRL_FUNCSEL_MASK 0x0000001f

/* Output Override: Configures output override values */
#define RP1_GPIO_CTRL_OUTOVER_LSB  12
#define RP1_GPIO_CTRL_OUTOVER_MASK 0x00003000

/* Output Enable Override: Configures output enable override */
#define RP1_GPIO_CTRL_OEOVER_LSB  14
#define RP1_GPIO_CTRL_OEOVER_MASK 0x0000c000

/* Input Override: Configures input override */
#define RP1_GPIO_CTRL_INOVER_LSB  16
#define RP1_GPIO_CTRL_INOVER_MASK 0x00030000

/* ============================================================
 * PAD Control Bitfields
 * ============================================================ */

/* Input enable and output disable masks for pad control */
#define RP1_PAD_IN_ENABLE_MASK   0x00000040
#define RP1_PAD_OUT_DISABLE_MASK 0x00000080

/* ============================================================
 * Pinmux Encoding
 * ============================================================ */

/* Pinmux encoding layout for Devicetree pinmuxing */
#define RP1_PIN_SHIFT 16
#define RP1_PIN_MASK  0xFFFF

#define RP1_FUNC_SHIFT 0
#define RP1_FUNC_MASK  0xFF

/* ============================================================
 * Override Modes
 * ============================================================ */

/* Output override options */
#define RP1_OUTOVER_PERI    0
#define RP1_OUTOVER_INVPERI 1
#define RP1_OUTOVER_LOW     2
#define RP1_OUTOVER_HIGH    3

/* Output Enable override options */
#define RP1_OEOVER_PERI    0
#define RP1_OEOVER_INVPERI 1
#define RP1_OEOVER_DISABLE 2
#define RP1_OEOVER_ENABLE  3

/* ============================================================
 * Utility Macros
 * ============================================================ */

/* Field get and set macros for register manipulation */
#define RP1_FLD_GET(r, mask, lsb)    (((r) & (mask)) >> (lsb))
#define RP1_FLD_SET(r, mask, lsb, v) (((r) & ~(mask)) | ((v) << (lsb)))

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) \
	DT_PROP(DT_PHANDLE(node_id, prop), pinmux)

#define Z_PINCTRL_STATE_PINS_LEN(node_id, prop)  \
	DT_PROP_LEN(DT_PHANDLE(node_id, prop), pinmux)

#endif /* ZEPHYR_SOC_BCM2712_PINCTRL_SOC_H_ */