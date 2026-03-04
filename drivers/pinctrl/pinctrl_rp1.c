#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/drivers/pinctrl_rp1.h>
#include <zephyr/dt-bindings/pinctrl/rp1-pinctrl.h>

#include <zephyr/sys/__assert.h>   /* __ASSERT */

LOG_MODULE_REGISTER(pinctrl_rp1, CONFIG_PINCTRL_RP1_LOG_LEVEL);

#define DT_DRV_COMPAT raspberrypi_rp1_pinctrl

/* -------------------------------------------------------------------------- */
/* Driver configuration structure (Physical addresses from Devicetree)       */
/* -------------------------------------------------------------------------- */

struct rp1_pinctrl_config {
	uintptr_t gpio_phys;
	uintptr_t rio_phys;
	uintptr_t pads_phys;
	size_t gpio_size;
	size_t rio_size;
	size_t pads_size;
};

/* -------------------------------------------------------------------------- */
/* Runtime data structure (Mapped virtual addresses)                          */
/* -------------------------------------------------------------------------- */

struct rp1_pinctrl_data {
	uint8_t *gpio;
	uint8_t *rio;
	uint8_t *pads;
};

/* -------------------------------------------------------------------------- */
/* I/O bank layout (Same layout as Linux driver)                               */
/* -------------------------------------------------------------------------- */

struct rp1_iobank_desc {
	uint8_t min_gpio;
	uint8_t num_gpios;
	uint32_t gpio_offset;
	uint32_t rio_offset;
	uint32_t pads_offset;
};

static const struct rp1_iobank_desc rp1_iobanks[RP1_NUM_BANKS] = {
	{  0, 28, 0x0000, 0x0000, 0x0004 },
	{ 28,  6, 0x4000, 0x4000, 0x4004 },
	{ 34, 20, 0x8000, 0x8000, 0x8004 },
};

/* -------------------------------------------------------------------------- */
/* MMIO helpers (Virtual address access only)                                 */
/* -------------------------------------------------------------------------- */

static inline uint32_t rp1_read(uintptr_t addr)
{
	return sys_read32(addr);
}

static inline void rp1_write(uint32_t value, uintptr_t addr)
{
	sys_write32(value, addr);
}

/* -------------------------------------------------------------------------- */
/* PAD update helper                                                           */
/* -------------------------------------------------------------------------- */

static void rp1_pad_update(uintptr_t pad_addr,
			   uint32_t clear_mask,
			   uint32_t set_mask)
{
	uint32_t val = rp1_read(pad_addr);
	val &= ~clear_mask;
	val |= set_mask;
	rp1_write(val, pad_addr);
}

/* -------------------------------------------------------------------------- */
/* Configure FSEL (Function Select)                                            */
/* -------------------------------------------------------------------------- */

static void rp1_set_fsel(uintptr_t gpio_base,
			 uintptr_t pads_base,
			 const struct rp1_iobank_desc *bank,
			 uint32_t pin,
			 uint32_t fsel)
{
	uint32_t offset = pin - bank->min_gpio;

	uintptr_t gpio_addr =
		gpio_base +
		bank->gpio_offset +
		offset * sizeof(uint32_t) * 2;

	uintptr_t pad_addr =
		pads_base +
		bank->pads_offset +
		offset * sizeof(uint32_t);

	uint32_t ctrl = rp1_read(gpio_addr + RP1_GPIO_CTRL);

	/* Enable input buffer */
	rp1_pad_update(pad_addr,
		       RP1_PAD_IN_ENABLE_MASK,
		       RP1_PAD_IN_ENABLE_MASK);

	/* Ensure output is not disabled */
	rp1_pad_update(pad_addr,
		       RP1_PAD_OUT_DISABLE_MASK,
		       0);

	if (fsel == RP1_FSEL_NONE) {
		ctrl = RP1_FLD_SET(ctrl,
				   RP1_GPIO_CTRL_OEOVER_MASK,
				   RP1_GPIO_CTRL_OEOVER_LSB,
				   RP1_OEOVER_DISABLE);
	} else {
		ctrl = RP1_FLD_SET(ctrl,
				   RP1_GPIO_CTRL_OUTOVER_MASK,
				   RP1_GPIO_CTRL_OUTOVER_LSB,
				   RP1_OUTOVER_PERI);

		ctrl = RP1_FLD_SET(ctrl,
				   RP1_GPIO_CTRL_OEOVER_MASK,
				   RP1_GPIO_CTRL_OEOVER_LSB,
				   RP1_OEOVER_PERI);
	}

	ctrl = RP1_FLD_SET(ctrl,
			   RP1_GPIO_CTRL_FUNCSEL_MASK,
			   RP1_GPIO_CTRL_FUNCSEL_LSB,
			   fsel);

	rp1_write(ctrl, gpio_addr + RP1_GPIO_CTRL);
}

static void rp1_set_alt(const struct device *dev, uint32_t pin, uint32_t func)
{
    struct rp1_pinctrl_data *data = dev->data;

    uintptr_t ctrl = (uintptr_t)data->gpio + 0x4 + (pin * 8);

    uint32_t val = sys_read32(ctrl);
    val &= ~0x1F;          // func field clear (예: 하위 5비트)
    val |= func & 0x1F;    // 새 func
    sys_write32(val, ctrl);
}

static uint32_t rp1_get_ctrl(const struct device *dev,
                             uint32_t pin)
{
    struct rp1_pinctrl_data *data = dev->data;

    uintptr_t ctrl =
        (uintptr_t)data->gpio + 0x4 + (pin * 8);

    return sys_read32(ctrl);
}

/* -------------------------------------------------------------------------- */
/* Zephyr pinctrl entry                                                       */
/* reg argument now carries pointer to rp1_pinctrl_data (mapped VA)           */
/* -------------------------------------------------------------------------- */
#define RP1_PINCTRL_NODE DT_NODELABEL(rp1_pinctrl)

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins,
			   uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);

	const struct device *pctl = DEVICE_DT_GET(RP1_PINCTRL_NODE);
	const struct rp1_pinctrl_data *data;

	/* Ensure the pinctrl provider is ready before touching HW registers */
	if (!device_is_ready(pctl)) {
		return -ENODEV;
	}

	data = (const struct rp1_pinctrl_data *)pctl->data;

	/* Bases must be mapped in rp1_pinctrl_init() */
	if (data == NULL || data->gpio == NULL || data->pads == NULL) {
		return -EAGAIN;
	}

	/* Nothing to configure */
	if (pins == NULL || pin_cnt == 0U) {
		return 0;
	}

	uintptr_t gpio_base = (uintptr_t)data->gpio;
	uintptr_t pads_base = (uintptr_t)data->pads;

	/* Debug: print mapped base addresses once per call */
	LOG_ERR("RP1 pinctrl: gpio_base=%p pads_base=%p (pin_cnt=%u)",
		(void *)gpio_base, (void *)pads_base, pin_cnt);

	for (uint8_t i = 0; i < pin_cnt; i++) {

		uint32_t pinmux = pins[i];

		uint32_t pin = (pinmux >> RP1_PIN_SHIFT) & RP1_PIN_MASK;
		uint32_t fsel = (pinmux >> RP1_FUNC_SHIFT) & RP1_FUNC_MASK;

		if (pin >= RP1_NUM_GPIOS) {
			LOG_ERR("RP1 pinctrl: invalid pin %u (pinmux=0x%08x)", pin, pinmux);
			return -EINVAL;
		}

		const struct rp1_iobank_desc *bank = NULL;

		for (int b = 0; b < RP1_NUM_BANKS; b++) {
			if (pin >= rp1_iobanks[b].min_gpio &&
			    pin < (rp1_iobanks[b].min_gpio + rp1_iobanks[b].num_gpios)) {
				bank = &rp1_iobanks[b];
				break;
			}
		}

		if (!bank) {
			LOG_ERR("RP1 pinctrl: no bank for pin %u", pin);
			return -EINVAL;
		}

		/* ---- Debug: compute addresses exactly like rp1_set_fsel() does ----
		 * This is used to verify we are writing/reading the expected registers.
		 */
		uint32_t offset = pin - bank->min_gpio;

		/* GPIO bank layout: each pin has 2 x 32-bit registers (STATUS, CTRL) => 8 bytes stride
		 * The CTRL register offset within the pair is RP1_GPIO_CTRL (0x4).
		 */
		uintptr_t gpio_addr =
			gpio_base +
			bank->gpio_offset +
			offset * sizeof(uint32_t) * 2;

		/* PAD register layout: one 32-bit word per pin => 4 bytes stride */
		uintptr_t pad_addr =
			pads_base +
			bank->pads_offset +
			offset * sizeof(uint32_t);

		uint32_t ctrl_before = rp1_read(gpio_addr + RP1_GPIO_CTRL);
		uint32_t pad_before  = rp1_read(pad_addr);

		LOG_ERR("RP1 pinctrl: pin=%u fsel=%u bank(min=%u n=%u goff=0x%x poff=0x%x) pinmux=0x%08x",
			pin, fsel,
			bank->min_gpio, bank->num_gpios,
			bank->gpio_offset, bank->pads_offset,
			pinmux);

		LOG_ERR("RP1 pinctrl: pin=%u gpio_addr=%p ctrl_addr=%p pad_addr=%p",
			pin,
			(void *)gpio_addr,
			(void *)(gpio_addr + RP1_GPIO_CTRL),
			(void *)pad_addr);

		LOG_ERR("RP1 pinctrl: pin=%u BEFORE: CTRL=0x%08x PAD=0x%08x",
			pin, ctrl_before, pad_before);

		/* Apply function select (pinmux) for this GPIO */
		rp1_set_fsel(gpio_base, pads_base, bank, pin, fsel);

		uint32_t ctrl_after = rp1_read(gpio_addr + RP1_GPIO_CTRL);
		uint32_t pad_after  = rp1_read(pad_addr);

		LOG_ERR("RP1 pinctrl: pin=%u AFTER : CTRL=0x%08x PAD=0x%08x",
			pin, ctrl_after, pad_after);

		/* Extra hint: specifically track PAD OUT_DISABLE and IN_ENABLE bits */
		LOG_ERR("RP1 pinctrl: pin=%u PAD bits: IN_EN %u->%u OUT_DIS %u->%u",
			pin,
			!!(pad_before & RP1_PAD_IN_ENABLE_MASK),
			!!(pad_after  & RP1_PAD_IN_ENABLE_MASK),
			!!(pad_before & RP1_PAD_OUT_DISABLE_MASK),
			!!(pad_after  & RP1_PAD_OUT_DISABLE_MASK));
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
/* Driver initialization                                                       */
/* Maps physical addresses to virtual addresses using MMU                     */
/* -------------------------------------------------------------------------- */

static int rp1_pinctrl_init(const struct device *dev)
{
	const struct rp1_pinctrl_config *cfg = dev->config;
	struct rp1_pinctrl_data *data = dev->data;

	LOG_ERR("RP1 pinctrl cfg: dev=%p cfg=%p data=%p",
		    dev, dev->config, dev->data);

	k_mem_map_phys_bare(&data->gpio,
			    cfg->gpio_phys,
			    cfg->gpio_size,
			    K_MEM_PERM_RW | K_MEM_CACHE_NONE);

	k_mem_map_phys_bare(&data->rio,
			    cfg->rio_phys,
			    cfg->rio_size,
			    K_MEM_PERM_RW | K_MEM_CACHE_NONE);

	k_mem_map_phys_bare(&data->pads,
			    cfg->pads_phys,
			    cfg->pads_size,
			    K_MEM_PERM_RW | K_MEM_CACHE_NONE);

	LOG_INF("RP1 pinctrl mapped: gpio=%p pads=%p",
		data->gpio, data->pads);

	return 0;
}

static const struct rp1_pinctrl_api rp1_api = {
    .set_alt = rp1_set_alt,
    .get_ctrl = rp1_get_ctrl,
};

/* -------------------------------------------------------------------------- */
/* Device instantiation                                                        */
/* -------------------------------------------------------------------------- */

#define RP1_INIT(inst)                                                   \
	static const struct rp1_pinctrl_config rp1_config_##inst = {         \
		.gpio_phys = DT_INST_REG_ADDR_BY_IDX(inst, 0),                    \
		.rio_phys  = DT_INST_REG_ADDR_BY_IDX(inst, 1),                    \
		.pads_phys = DT_INST_REG_ADDR_BY_IDX(inst, 2),                    \
		.gpio_size = DT_INST_REG_SIZE_BY_IDX(inst, 0),                    \
		.rio_size  = DT_INST_REG_SIZE_BY_IDX(inst, 1),                    \
		.pads_size = DT_INST_REG_SIZE_BY_IDX(inst, 2),                    \
	};                                                                   \
	static struct rp1_pinctrl_data rp1_data_##inst;                      \
	DEVICE_DT_INST_DEFINE(inst,                                           \
			      rp1_pinctrl_init,                                \
			      NULL,                                            \
			      &rp1_data_##inst,                                \
			      &rp1_config_##inst,                              \
			      PRE_KERNEL_1,                                    \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,             \
			      &rp1_api);

DT_INST_FOREACH_STATUS_OKAY(RP1_INIT)
