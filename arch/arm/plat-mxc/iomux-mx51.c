/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*
 * i.MX51 GPIO and IOMUXC Setup
 *
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/iomux-mx51.h>

/*
 * IOMUX register (base) addresses
 */
#define IOMUXGPR0		MX5_IO_ADDRESS(IOMUXC_BASE_ADDR)
#define IOMUXGPR1		(MX5_IO_ADDRESS(IOMUXC_BASE_ADDR + 0x004))
#define IOMUXSW_MUX_CTL		MX5_IO_ADDRESS(IOMUXC_BASE_ADDR)
#define IOMUXSW_MUX_END		(MX5_IO_ADDRESS(IOMUXC_BASE_ADDR + MUX_I_END))
#define IOMUXSW_PAD_CTL		(MX5_IO_ADDRESS(IOMUXC_BASE_ADDR + PAD_I_START))
#define IOMUXSW_INPUT_CTL	MX5_IO_ADDRESS(IOMUXC_BASE_ADDR)

#define MUX_PIN_NUM_MAX	       ((MUX_I_END >> 2) + 1)

static u8 iomux_pin_res_table[MUX_PIN_NUM_MAX];
static DEFINE_SPINLOCK(gpio_mux_lock);

static inline void __iomem *_get_mux_reg(iomux_pin_name_t pin)
{
	u32 mux_reg = PIN_TO_IOMUX_MUX(pin);

	if (mx51_revision() < MX51_CHIP_REV_2_0) {
		if ((pin == MX51_PIN_NANDF_RB5) ||
			(pin == MX51_PIN_NANDF_RB6) ||
			(pin == MX51_PIN_NANDF_RB7))
			; /* Do nothing */
		else if (mux_reg >= 0x2FC)
			mux_reg += 8;
		else if (mux_reg >= 0x130)
			mux_reg += 0xC;
	}
	return IOMUXSW_MUX_CTL + mux_reg;
}

static inline void __iomem *_get_pad_reg(iomux_pin_name_t pin)
{
	u32 pad_reg = PIN_TO_IOMUX_PAD(pin);

	if (mx51_revision() < MX51_CHIP_REV_2_0) {
		if ((pin == MX51_PIN_NANDF_RB5) ||
			(pin == MX51_PIN_NANDF_RB6) ||
			(pin == MX51_PIN_NANDF_RB7))
			; /* Do nothing */
		else if (pad_reg == 0x4D0 - PAD_I_START)
			pad_reg += 0x4C;
		else if (pad_reg == 0x860 - PAD_I_START)
			pad_reg += 0x9C;
		else if (pad_reg >= 0x804 - PAD_I_START)
			pad_reg += 0xB0;
		else if (pad_reg >= 0x7FC - PAD_I_START)
			pad_reg += 0xB4;
		else if (pad_reg >= 0x4E4 - PAD_I_START)
			pad_reg += 0xCC;
		else
			pad_reg += 8;
	}
	return IOMUXSW_PAD_CTL + pad_reg;
}

static inline void __iomem *_get_mux_end(void)
{
	if (mx51_revision() >= MX51_CHIP_REV_2_0)
		return MX5_IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x3F0 - 4;
	else
		return MX5_IO_ADDRESS(IOMUXC_BASE_ADDR) + 0x3F8 - 4;
}

/*!
 * This function is used to configure a pin through the IOMUX module.
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
static int iomux_config_mux(iomux_pin_name_t pin, unsigned int config)
{
	u32 ret = 0;
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	void __iomem *mux_reg = _get_mux_reg(pin);
	u32 mux_data = 0;
	u8 *rp;

	if (pin_index >= ARRAY_SIZE(iomux_pin_res_table))
		return -EINVAL;

	if (WARN_ON((mux_reg > _get_mux_end()) || (mux_reg < IOMUXSW_MUX_CTL)))
		return -EINVAL;

	spin_lock(&gpio_mux_lock);

	if (config & IOMUX_CONFIG_GPIO)
		mux_data = PIN_TO_ALT_GPIO(pin) | (config & IOMUX_CONFIG_SION);
	else
		mux_data = config;
	/*
	 * Log a warning if a pin changes ownership
	 */
	rp = &iomux_pin_res_table[pin_index];
	if ((mux_data & *rp) && (*rp != mux_data)) {
		/*
		 * Don't call printk if we're tweaking the console uart or
		 * we'll deadlock.
		 */
		printk(KERN_WARNING "iomux %p already in use as %02x\n",
			mux_reg, *rp);
		ret = -EBUSY;
		goto out;
	}
	*rp = mux_data;
	__raw_writel(mux_data, mux_reg);
out:
	spin_unlock(&gpio_mux_lock);
	return ret;
}

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	a configuration as defined in \b #iomux_pin_cfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mx51_request_iomux(iomux_pin_name_t pin, unsigned int config)
{
	int ret;
	int gpio_port = GPIO_TO_PORT(IOMUX_TO_GPIO(pin));
	int gpio = IOMUX_TO_GPIO(pin);
	unsigned int conf = config & ~IOMUX_CONFIG_SION;

	if ((gpio_port != NON_GPIO_PORT) &&
		((conf == IOMUX_CONFIG_GPIO) ||
			(conf == PIN_TO_ALT_GPIO(pin)))) {
		ret = gpio_request(gpio, NULL);
		if (ret) {
			printk(KERN_ERR "%s: Failed to request GPIO %d: %d\n",
				__FUNCTION__, gpio, ret);
			return ret;
		}
	}
	ret = iomux_config_mux(pin, config);
	if (ret)
		gpio_free(gpio);

	return ret;
}
EXPORT_SYMBOL(mx51_request_iomux);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  config	config as defined in \b #iomux_pin_ocfg_t
 */
void mx51_free_iomux(iomux_pin_name_t pin, unsigned int config)
{
	u32 pin_index = PIN_TO_IOMUX_INDEX(pin);
	u8 *rp;
	int gpio_port = GPIO_TO_PORT(IOMUX_TO_GPIO(pin));

	if (pin_index >= ARRAY_SIZE(iomux_pin_res_table))
		return;
	rp = &iomux_pin_res_table[pin_index];
	*rp = 0;
	config &= ~IOMUX_CONFIG_SION;
	if ((gpio_port != NON_GPIO_PORT) &&
		((config == IOMUX_CONFIG_GPIO) ||
			(config == PIN_TO_ALT_GPIO(pin)))) {
		gpio_free(IOMUX_TO_GPIO(pin));
	}
}
EXPORT_SYMBOL(mx51_free_iomux);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mx51_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
	void __iomem *pad_reg = _get_pad_reg(pin);

	BUG_ON(pad_reg < IOMUXSW_PAD_CTL);
	__raw_writel(config, pad_reg);
}
EXPORT_SYMBOL(mx51_iomux_set_pad);

unsigned int mx51_iomux_get_pad(iomux_pin_name_t pin)
{
	void __iomem *pad_reg = _get_pad_reg(pin);

	return __raw_readl(pad_reg);
}
EXPORT_SYMBOL(mx51_iomux_get_pad);

/*!
 * This function configures input path.
 *
 * @param  input        index of input select register as defined in \b #iomux_input_select_t
 * @param  config       the binary value of elements defined in \b #iomux_input_config_t
 *      */
void mx51_iomux_set_input(iomux_input_select_t input, u32 config)
{
	u32 reg;

	if (mx51_revision() >= MX51_CHIP_REV_2_0) {
		reg = (input << 2) + INPUT_CTL_START;
	} else {
		if (input == MUX_IN_IPU_IPP_DI_0_IND_DISPB_SD_D_SELECT_INPUT)
			input -= 4;
		else if (input == MUX_IN_IPU_IPP_DI_1_IND_DISPB_SD_D_SELECT_INPUT)
			input -= 3;
		else if (input >= MUX_IN_KPP_IPP_IND_COL_6_SELECT_INPUT)
			input -= 2;
		else if (input >= MUX_IN_HSC_MIPI_MIX_PAR_SISG_TRIG_SELECT_INPUT)
			input -= 5;
		else if (input >= MUX_IN_HSC_MIPI_MIX_IPP_IND_SENS1_DATA_EN_SELECT_INPUT)
			input -= 3;
		else if (input >= MUX_IN_ECSPI2_IPP_IND_SS_B_3_SELECT_INPUT)
			input -= 2;
		else if (input >= MUX_IN_CCM_PLL1_BYPASS_CLK_SELECT_INPUT)
			input -= 1;

		reg = (input << 2) + INPUT_CTL_START_TO1;
	}

	BUG_ON(input >= MUX_INPUT_NUM_MUX);
	__raw_writel(config, IOMUXSW_INPUT_CTL + reg);
}
EXPORT_SYMBOL(mx51_iomux_set_input);

int mx51_iomux_request_pads(struct mx51_pad_desc *desc, size_t count)
{
	int ret = 0;
	int i;

	for (i = 0; i < count; i++) {
		ret = mx51_request_iomux(desc[i].pad, desc[i].config);
		if (ret) {
			while (--i >= 0) {
				mx51_free_iomux(desc[i].pad, desc[i].config);
			}
			return ret;
		}
	}
	return ret;
}
EXPORT_SYMBOL(mx51_iomux_request_pads);

void mx51_iomux_set_pads(struct mx51_pad_desc *desc, size_t count)
{
	int i;

	for (i = 0; i < count; i++) {
		mx51_iomux_set_pad(desc[i].pad, desc[i].config);
	}
}
EXPORT_SYMBOL(mx51_iomux_set_pads);

void mx51_iomux_set_inputs(struct mx51_pad_desc *desc, size_t count)
{
	int i;

	for (i = 0; i < count; i++) {
		mx51_iomux_set_input(desc[i].pad, desc[i].config);
	}
}
EXPORT_SYMBOL(mx51_iomux_set_inputs);

void mx51_iomux_release_pads(struct mx51_pad_desc *desc, size_t count)
{
	int i;

	for (i = 0; i < count; i++) {
		mx51_free_iomux(desc[i].pad, desc[i].config);
	}
}
EXPORT_SYMBOL(mx51_iomux_release_pads);
