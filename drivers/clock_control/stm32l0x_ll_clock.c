/*
 *
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <soc.h>
#include <soc_registers.h>
#include <clock_control.h>
#include <misc/util.h>
#include <clock_control/stm32_clock_control.h>
#include "stm32_ll_clock.h"


#ifdef CONFIG_CLOCK_STM32_SYSCLK_SRC_PLL

/* Macros to fill up division factors values */
#define _pllm(v) LL_RCC_PLL_MUL_ ## v
#define pllm(v) _pllm(v)

#define _plld(v) LL_RCC_PLL_DIV_ ## v
#define plld(v) _plld(v)

/**
 * @brief fill in pll configuration structure
 */
void config_pll_init(LL_UTILS_PLLInitTypeDef *pllinit)
{
	pllinit->PLLDiv = plld(CONFIG_CLOCK_STM32_PLL_DIVISOR);
	pllinit->PLLMul = pllm(CONFIG_CLOCK_STM32_PLL_MULTIPLIER);
}
#endif /* CONFIG_CLOCK_STM32_SYSCLK_SRC_PLL */

/**
 * @brief Activate default clocks
 */
void config_enable_default_clocks(void)
{
	/* Nothing for now */
}
