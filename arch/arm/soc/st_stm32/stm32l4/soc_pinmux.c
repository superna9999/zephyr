/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include "soc.h"
#include "soc_pinmux.h"
#include <device.h>
#include <misc/util.h>
#include <pinmux/stm32/pinmux_stm32.h>
#include <drivers/clock_control/stm32_clock_control.h>

static const stm32_pin_func_t pin_pa9_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PA9_USART1_TX - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pa10_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PA10_USART1_RX - 1] =
		STM32L4X_PIN_CONFIG_BIAS_HIGH_IMPEDANCE,
};

static const stm32_pin_func_t pin_pa2_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PA2_USART2_TX - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pa3_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PA3_USART2_RX - 1] =
		STM32L4X_PIN_CONFIG_BIAS_HIGH_IMPEDANCE,
};

static const stm32_pin_func_t pin_pb6_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PB6_I2C1_SCL - 1] =
		STM32L4X_PIN_CONFIG_OPEN_DRAIN_PULL_UP,
};

static const stm32_pin_func_t pin_pb7_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PB7_I2C1_SDA - 1] =
		STM32L4X_PIN_CONFIG_OPEN_DRAIN_PULL_UP,
};

static const stm32_pin_func_t pin_pb10_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PB10_USART3_TX - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pb11_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PB11_USART3_RX - 1] =
		STM32L4X_PIN_CONFIG_BIAS_HIGH_IMPEDANCE,
};

static const stm32_pin_func_t pin_pa0_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PA0_PWM2_CH1 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd0_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD0_FMC_D2 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd1_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD1_FMC_D3 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd8_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD8_FMC_D13 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd9_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD9_FMC_D14 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd10_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD10_FMC_D15 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd11_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD11_FMC_A16 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd12_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD12_FMC_A17 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd13_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD13_FMC_A18 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe7_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE7_FMC_D4 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe8_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE8_FMC_D5 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe9_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE9_FMC_D6 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe10_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE10_FMC_D7 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe11_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE11_FMC_D8 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe12_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE12_FMC_D9 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe13_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE13_FMC_D10 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe14_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE14_FMC_D11 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pe15_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE15_FMC_D12 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf0_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF0_FMC_A0 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf1_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF1_FMC_A1 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf2_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF2_FMC_A2 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf3_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF3_FMC_A3 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf4_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF4_FMC_A4 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf5_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF5_FMC_A5 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf12_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF12_FMC_A6 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf13_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF13_FMC_A7 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf14_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF14_FMC_A8 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pf15_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PF15_FMC_A9 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pg0_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PG0_FMC_A10 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pg1_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PG1_FMC_A11 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pg2_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PG2_FMC_A12 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pg3_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PG3_FMC_A14 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pg4_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PG4_FMC_A14 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pg5_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PG5_FMC_A15 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL,
};

static const stm32_pin_func_t pin_pd4_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD4_FMC_NOE - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL_PULL_UP,
};

static const stm32_pin_func_t pin_pd5_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PD5_FMC_NWE - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL_PULL_UP,
};

static const stm32_pin_func_t pin_pe0_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE0_FMC_NBL0 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL_PULL_UP,
};

static const stm32_pin_func_t pin_pe1_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PE1_FMC_NBL1 - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL_PULL_UP,
};

static const stm32_pin_func_t pin_pg9_funcs[] = {
	[STM32L4X_PINMUX_FUNC_PG9_FMC_NCE - 1] =
		STM32L4X_PIN_CONFIG_PUSH_PULL_PULL_UP,
};

/**
 * @brief pin configuration
 */
static const struct stm32_pinmux_conf pins[] = {
	STM32_PIN_CONF(STM32_PIN_PA0, pin_pa0_funcs),
	STM32_PIN_CONF(STM32_PIN_PA2, pin_pa2_funcs),
	STM32_PIN_CONF(STM32_PIN_PA3, pin_pa3_funcs),
	STM32_PIN_CONF(STM32_PIN_PA9, pin_pa9_funcs),
	STM32_PIN_CONF(STM32_PIN_PA10, pin_pa10_funcs),
	STM32_PIN_CONF(STM32_PIN_PB6, pin_pb6_funcs),
	STM32_PIN_CONF(STM32_PIN_PB7, pin_pb7_funcs),
	STM32_PIN_CONF(STM32_PIN_PB10, pin_pb10_funcs),
	STM32_PIN_CONF(STM32_PIN_PB11, pin_pb11_funcs),
	STM32_PIN_CONF(STM32_PIN_PD0, pin_pd0_funcs),
	STM32_PIN_CONF(STM32_PIN_PD1, pin_pd1_funcs),
	STM32_PIN_CONF(STM32_PIN_PD4, pin_pd4_funcs),
	STM32_PIN_CONF(STM32_PIN_PD5, pin_pd5_funcs),
	STM32_PIN_CONF(STM32_PIN_PD8, pin_pd8_funcs),
	STM32_PIN_CONF(STM32_PIN_PD9, pin_pd9_funcs),
	STM32_PIN_CONF(STM32_PIN_PD10, pin_pd10_funcs),
	STM32_PIN_CONF(STM32_PIN_PD11, pin_pd11_funcs),
	STM32_PIN_CONF(STM32_PIN_PD12, pin_pd12_funcs),
	STM32_PIN_CONF(STM32_PIN_PD13, pin_pd13_funcs),
	STM32_PIN_CONF(STM32_PIN_PE0, pin_pe0_funcs),
	STM32_PIN_CONF(STM32_PIN_PE1, pin_pe1_funcs),
	STM32_PIN_CONF(STM32_PIN_PE7, pin_pe7_funcs),
	STM32_PIN_CONF(STM32_PIN_PE8, pin_pe8_funcs),
	STM32_PIN_CONF(STM32_PIN_PE9, pin_pe9_funcs),
	STM32_PIN_CONF(STM32_PIN_PE10, pin_pe10_funcs),
	STM32_PIN_CONF(STM32_PIN_PE11, pin_pe11_funcs),
	STM32_PIN_CONF(STM32_PIN_PE12, pin_pe12_funcs),
	STM32_PIN_CONF(STM32_PIN_PE13, pin_pe13_funcs),
	STM32_PIN_CONF(STM32_PIN_PE14, pin_pe14_funcs),
	STM32_PIN_CONF(STM32_PIN_PE15, pin_pe15_funcs),
	STM32_PIN_CONF(STM32_PIN_PF0, pin_pf0_funcs),
	STM32_PIN_CONF(STM32_PIN_PF1, pin_pf1_funcs),
	STM32_PIN_CONF(STM32_PIN_PF2, pin_pf2_funcs),
	STM32_PIN_CONF(STM32_PIN_PF3, pin_pf3_funcs),
	STM32_PIN_CONF(STM32_PIN_PF4, pin_pf4_funcs),
	STM32_PIN_CONF(STM32_PIN_PF5, pin_pf5_funcs),
	STM32_PIN_CONF(STM32_PIN_PF12, pin_pf12_funcs),
	STM32_PIN_CONF(STM32_PIN_PF13, pin_pf13_funcs),
	STM32_PIN_CONF(STM32_PIN_PF14, pin_pf14_funcs),
	STM32_PIN_CONF(STM32_PIN_PF15, pin_pf15_funcs),
	STM32_PIN_CONF(STM32_PIN_PG0, pin_pg0_funcs),
	STM32_PIN_CONF(STM32_PIN_PG1, pin_pg1_funcs),
	STM32_PIN_CONF(STM32_PIN_PG2, pin_pg2_funcs),
	STM32_PIN_CONF(STM32_PIN_PG3, pin_pg3_funcs),
	STM32_PIN_CONF(STM32_PIN_PG4, pin_pg4_funcs),
	STM32_PIN_CONF(STM32_PIN_PG5, pin_pg5_funcs),
	STM32_PIN_CONF(STM32_PIN_PG9, pin_pg9_funcs),
};

int stm32_get_pin_config(int pin, int func)
{
	/* GPIO function is always available, to save space it is not
	 * listed in alternate functions array
	 */
	if (func == STM32_PINMUX_FUNC_GPIO) {
		return STM32L4X_PIN_CONFIG_BIAS_HIGH_IMPEDANCE;
	}

	/* analog function is another 'known' setting */
	if (func == STM32_PINMUX_FUNC_ANALOG) {
		return STM32L4X_PIN_CONFIG_ANALOG;
	}

	for (int i = 0; i < ARRAY_SIZE(pins); i++) {
		if (pins[i].pin == pin) {
			if ((func - 1) >= pins[i].nfuncs) {
				return -EINVAL;
			}

			return pins[i].funcs[func - 1];
		}
	}
	return -EINVAL;
}
