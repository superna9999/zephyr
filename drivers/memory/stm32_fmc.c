/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <string.h>

#include <soc.h>
#include <board.h>
#include <clock_control.h>

#include <misc/util.h>

#include <clock_control/stm32_clock_control.h>

#include "stm32_fmc.h"

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct stm32_fmc_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)							\
	((struct stm32_fmc_data * const)(dev)->driver_data)

static int stm32_fmc_init(struct device *dev)
{
	struct stm32_fmc_data *data = DEV_DATA(dev);
	const struct stm32_fmc_config *cfg = DEV_CFG(dev);
	struct device *clk =
		device_get_binding(STM32_CLOCK_CONTROL_NAME);
	FMC_NORSRAM_InitTypeDef init;
	FMC_NORSRAM_TimingTypeDef timings;

	__ASSERT_NO_MSG(clk);

	data->clock = clk;

	/* enable clock */
	clock_control_on(data->clock,
		(clock_control_subsys_t *)&cfg->pclken);

	memcpy(&init, &cfg->config, sizeof(FMC_NORSRAM_InitTypeDef));
	memcpy(&timings, &cfg->timings, sizeof(FMC_NORSRAM_TimingTypeDef));

	if (FMC_NORSRAM_Init(FMC_NORSRAM_DEVICE, &init) != HAL_OK) {
		return -EIO;
	}

	if (FMC_NORSRAM_Timing_Init(FMC_NORSRAM_DEVICE,
				    &timings, init.NSBank) != HAL_OK) {
		return -EIO;
	}

	__FMC_NORSRAM_ENABLE(FMC_NORSRAM_DEVICE, init.NSBank);

	return 0;
}

#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1

static const struct stm32_fmc_config stm32_fmc_cfg_1 = {
	.pclken = { .bus = STM32_CLOCK_BUS_APB1,
		    .enr = LL_APB1_GRP1_PERIPH_I2C1 },
	.config = {
		.NSBank             = FMC_NORSRAM_BANK1,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1_ADDRESS_MUX_ENABLE
		.DataAddressMux     = FMC_DATA_ADDRESS_MUX_ENABLE,
#endif
		.MemoryType         = FMC_NOR_SRAM_BANK_1_TYPE,
		.MemoryDataWidth    = FMC_NOR_SRAM_BANK_1_WIDTH,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1_BURST_ACCESS_MODE_ENABLE
		.BurstAccessMode    = FMC_BURST_ACCESS_MODE_ENABLE,
#endif
		.WaitSignalPolarity = FMC_NOR_SRAM_BANK_1_SIGNAL_POLARITY,
		.WaitSignalActive   = FMC_NOR_SRAM_BANK_1_TIMING_WS,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1_WRITE_ENABLE
		.WriteOperation     = FMC_WRITE_OPERATION_ENABLE,
#endif
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1_WAIT_SIGNAL_ENABLE
		.WaitSignal	    = FMC_WAIT_SIGNAL_ENABLE,
#endif
		.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1_ASYNCHRONOUS_WAIT_ENABLE
		.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_ENABLE,
#endif
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1_WRITE_BURST_ENABLE
		.WriteBurst         = FMC_WRITE_BURST_ENABLE,
#endif
		.ContinuousClock    = FMC_NOR_SRAM_BANK_1_CLOCK_SYNC,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_1_ADDRESS_MUX_ENABLE
		.WriteFifo          = FMC_WRITE_FIFO_ENABLE,
#endif
		.PageSize 	    = FMC_NOR_SRAM_BANK_1_PAGE_SIZE,
	},
	.timings = {
		.AddressSetupTime =
			CONFIG_STM32_FMC_NOR_SRAM_BANK_1_ADDR_SETUP_TIME,
		.AddressHoldTime =
			CONFIG_STM32_FMC_NOR_SRAM_BANK_1_ADDR_HOLD_TIME,
		.DataSetupTime =
			CONFIG_STM32_FMC_NOR_SRAM_BANK_1_DATA_SETUP_TIME,
		.BusTurnAroundDuration = 
			CONFIG_STM32_FMC_NOR_SRAM_BANK_1_BUS_TURNAROUND_DURATION,
		.CLKDivision = CONFIG_STM32_FMC_NOR_SRAM_BANK_1_CLK_DIVISION,
		.DataLatency = CONFIG_STM32_FMC_NOR_SRAM_BANK_1_DATA_LATENCY,
		.AccessMode = FMC_NOR_SRAM_BANK_1_ACCESS_MODE,
	},
};

static struct stm32_fmc_data stm32_fmc_dev_data_1 = {
};

DEVICE_AND_API_INIT(stm32_fmc_bank_1, CONFIG_STM32_FMC_NOR_SRAM_BANK_1_NAME,
		    &stm32_fmc_init, &stm32_fmc_dev_data_1,
		    &stm32_fmc_cfg_1, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    NULL);

#endif /* STM32_FMC_NOR_SRAM_BANK_1 */

#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2

static const struct stm32_fmc_config stm32_fmc_cfg_1 = {
	.pclken = { .bus = STM32_CLOCK_BUS_APB1,
		    .enr = LL_APB1_GRP1_PERIPH_I2C1 },
	.config = {
		.NSBank             = FMC_NORSRAM_BANK1,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2_ADDRESS_MUX_ENABLE
		.DataAddressMux     = FMC_DATA_ADDRESS_MUX_ENABLE,
#endif
		.MemoryType         = FMC_NOR_SRAM_BANK_2_TYPE,
		.MemoryDataWidth    = FMC_NOR_SRAM_BANK_2_WIDTH,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2_BURST_ACCESS_MODE_ENABLE
		.BurstAccessMode    = FMC_BURST_ACCESS_MODE_ENABLE,
#endif
		.WaitSignalPolarity = FMC_NOR_SRAM_BANK_2_SIGNAL_POLARITY,
		.WaitSignalActive   = FMC_NOR_SRAM_BANK_2_TIMING_WS,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2_WRITE_ENABLE
		.WriteOperation     = FMC_WRITE_OPERATION_ENABLE,
#endif
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2_WAIT_SIGNAL_ENABLE
		.WaitSignal	    = FMC_WAIT_SIGNAL_ENABLE,
#endif
		.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2_ASYNCHRONOUS_WAIT_ENABLE
		.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_ENABLE,
#endif
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2_WRITE_BURST_ENABLE
		.WriteBurst         = FMC_WRITE_BURST_ENABLE,
#endif
		.ContinuousClock    = FMC_NOR_SRAM_BANK_2_CLOCK_SYNC,
#ifdef CONFIG_STM32_FMC_NOR_SRAM_BANK_2_ADDRESS_MUX_ENABLE
		.WriteFifo          = FMC_WRITE_FIFO_ENABLE,
#endif
		.PageSize 	    = FMC_NOR_SRAM_BANK_2_PAGE_SIZE,
	},
	.timings = {
		.AddressSetupTime =
			CONFIG_STM32_FMC_NOR_SRAM_BANK_2_ADDR_SETUP_TIME,
		.AddressHoldTime =
			CONFIG_STM32_FMC_NOR_SRAM_BANK_2_ADDR_HOLD_TIME,
		.DataSetupTime =
			CONFIG_STM32_FMC_NOR_SRAM_BANK_2_DATA_SETUP_TIME,
		.BusTurnAroundDuration = 
			CONFIG_STM32_FMC_NOR_SRAM_BANK_2_BUS_TURNAROUND_DURATION,
		.CLKDivision = CONFIG_STM32_FMC_NOR_SRAM_BANK_2_CLK_DIVISION,
		.DataLatency = CONFIG_STM32_FMC_NOR_SRAM_BANK_2_DATA_LATENCY,
		.AccessMode = FMC_NOR_SRAM_BANK_2_ACCESS_MODE,
	},
};

static struct stm32_fmc_data stm32_fmc_dev_data_1 = {
};

DEVICE_AND_API_INIT(stm32_fmc_bank_1, CONFIG_STM32_FMC_NOR_SRAM_BANK_2_NAME,
		    &stm32_fmc_init, &stm32_fmc_dev_data_1,
		    &stm32_fmc_cfg_1, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    NULL);

#endif /* STM32_FMC_NOR_SRAM_BANK_2 */
