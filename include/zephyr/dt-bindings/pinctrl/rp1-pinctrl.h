/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * RP1 Devicetree Pinmux Definitions
 *
 * This header contains ONLY definitions required for
 * Devicetree pinmux encoding.
 */

#ifndef ZEPHYR_DT_BINDINGS_PINCTRL_RP1_PINCTRL_H_
#define ZEPHYR_DT_BINDINGS_PINCTRL_RP1_PINCTRL_H_

#ifndef RP1_PIN_SHIFT
#define RP1_PIN_SHIFT 16
#endif

#ifndef RP1_FUNC_SHIFT
#define RP1_FUNC_SHIFT 0
#endif


/* ============================================================
 * General SoC Characteristics
 * ============================================================ */

/* Total number of GPIO pins for RP1 */
#define RP1_NUM_GPIOS 54
/* Number of GPIO banks */
#define RP1_NUM_BANKS 3

/* ============================================================
 * Function (ALT) Numbers for Pinmuxing
 * ============================================================ */

/* Alternate function numbers (ALT0, ALT1, etc.) */
#define RP1_FSEL_ALT0 0x00
#define RP1_FSEL_ALT1 0x01
#define RP1_FSEL_ALT2 0x02
#define RP1_FSEL_ALT3 0x03
#define RP1_FSEL_ALT4 0x04
#define RP1_FSEL_GPIO 0x05
#define RP1_FSEL_ALT6 0x06
#define RP1_FSEL_ALT7 0x07
#define RP1_FSEL_ALT8 0x08

/* None function (no pinmux function set) */
#define RP1_FSEL_NONE 0x09

/* ============================================================
 * Override Values for GPIO Configurations
 * ============================================================ */

/* Override configurations for GPIO states */
#define RP1_OUTOVER_PERI   0
#define RP1_OEOVER_PERI    0
#define RP1_OEOVER_DISABLE 2

/* ============================================================
 * GPIO_CTRL Register Bitfields
 * ============================================================ */

/* GPIO control register bit fields for configuring pin functions */
#define RP1_GPIO_CTRL_FUNCSEL_LSB  0
#define RP1_GPIO_CTRL_FUNCSEL_MASK 0x0000001f

#define RP1_GPIO_CTRL_OUTOVER_LSB  12
#define RP1_GPIO_CTRL_OUTOVER_MASK 0x00003000

#define RP1_GPIO_CTRL_OEOVER_LSB  14
#define RP1_GPIO_CTRL_OEOVER_MASK 0x0000c000

#define RP1_GPIO_CTRL_INOVER_LSB  16
#define RP1_GPIO_CTRL_INOVER_MASK 0x00030000

#define RP1_GPIO_CTRL_IRQOVER_LSB  30
#define RP1_GPIO_CTRL_IRQOVER_MASK 0xc0000000

/* ============================================================
 * PAD Control Bitfields
 * ============================================================ */

/* PAD control configurations */
#define RP1_PAD_IN_ENABLE_MASK   0x00000040
#define RP1_PAD_OUT_DISABLE_MASK 0x00000080

/* ============================================================
 * Devicetree Pin Configuration (Packed into the 2nd argument)
 * ============================================================ */

/*
 * NOTE:
 * The 2nd argument of RP1_PINMUX(pin, cfg) is a packed value:
 *   - lower bits: function select (ALT)
 *   - upper bits: pin configuration flags (PAD/override options)
 *
 * This keeps Devicetree usage to 2 arguments while allowing extra configuration.
 *
 * Example:
 *   RP1_PINMUX(0, RP1_FSEL_ALT0 | RP1_PINCFG_INPUT_EN)
 */
#ifndef RP1_CFG_FUNC_BITS
#define RP1_CFG_FUNC_BITS 8U
#endif

#ifndef RP1_CFG_FUNC_MASK
#define RP1_CFG_FUNC_MASK ((1U << RP1_CFG_FUNC_BITS) - 1U) /* 0xFF */
#endif

#ifndef RP1_CFG_FLAGS_SHIFT
#define RP1_CFG_FLAGS_SHIFT RP1_CFG_FUNC_BITS
#endif

/* Pin configuration flags (packed into cfg upper bits). */
#define RP1_PINCFG_INPUT_EN   (1U << (RP1_CFG_FLAGS_SHIFT + 0))
#define RP1_PINCFG_OUTPUT_DIS (1U << (RP1_CFG_FLAGS_SHIFT + 1))

/* Placeholders for future expansion (driver may ignore if unimplemented). */
#define RP1_PINCFG_PULL_UP    (1U << (RP1_CFG_FLAGS_SHIFT + 2))
#define RP1_PINCFG_PULL_DOWN  (1U << (RP1_CFG_FLAGS_SHIFT + 3))
#define RP1_PINCFG_DRIVE_2MA  (1U << (RP1_CFG_FLAGS_SHIFT + 4))
#define RP1_PINCFG_DRIVE_4MA  (1U << (RP1_CFG_FLAGS_SHIFT + 5))

/* Helpers to extract func/flags from the packed cfg */
#define RP1_CFG_GET_FUNC(cfg)  ((uint32_t)(cfg) & (uint32_t)RP1_CFG_FUNC_MASK)
#define RP1_CFG_GET_FLAGS(cfg) ((uint32_t)(cfg) & ~(uint32_t)RP1_CFG_FUNC_MASK)

/* ============================================================
 * Utility Macros
 * ============================================================ */

/*
 * Pinmux encoding helper macro to generate pinmux settings.
 *
 * Encoding:
 *   - upper 16 bits: pin number
 *   - lower 16 bits: cfg (func + flags)
 *
 * Backward compatible:
 *   - Passing a plain function value (e.g., RP1_FSEL_ALT0) still works.
 *   - Flags can be OR'ed into the cfg value.
 */
#ifndef RP1_PINMUX
#define RP1_PINMUX(pin, cfg) (((pin) << RP1_PIN_SHIFT) | ((cfg) << RP1_FUNC_SHIFT))
#endif

#endif /* ZEPHYR_DT_BINDINGS_PINCTRL_RP1_PINCTRL_H_ */