/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_I2C_H_
#define _STM32_I2C_H_

typedef void (*irq_config_func_t)(struct device *port);

/* device config */
struct i2c_stm32_config {
	void *base;
	irq_config_func_t irq_config_func;
	/* clock subsystem driving this peripheral */
	struct stm32_pclken pclken;
};

/* driver data */
struct i2c_stm32_data {
	/* I2C peripheral handler */
	I2C_HandleTypeDef hi2c;
	/* clock device */
	struct device *clock;
	/* Device config */
	union dev_config dev_config;
	/* ISR Sync */
	struct k_sem device_sync_sem;
	/* Complete status */
	bool is_err;
};

#endif	/* _STM32_UART_H_ */
