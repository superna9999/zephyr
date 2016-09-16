/*
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_SPI_H_
#define _STM32_SPI_H_

#ifdef CONFIG_SOC_SERIES_STM32F4X
#include <stm32f4xx_ll_spi.h>
#endif

#ifdef CONFIG_SOC_SERIES_STM32L4X
#include <stm32l4xx_ll_spi.h>
#endif

#ifdef CONFIG_SOC_SERIES_STM32F3X
#include <stm32f3xx_ll_spi.h>
#endif

typedef void (*irq_config_func_t)(struct device *port);

struct spi_stm32_config {
	struct stm32_pclken pclken;
	SPI_TypeDef *spi;
#ifdef CONFIG_SPI_STM32_INTERRUPT
	irq_config_func_t irq_config;
#endif
};

struct spi_stm32_buffer_rx;
struct spi_stm32_buffer_tx;

typedef void (*rx_func_t)(SPI_TypeDef *spi, struct spi_stm32_buffer_rx *p);
typedef void (*tx_func_t)(SPI_TypeDef *spi, struct spi_stm32_buffer_tx *p);

struct spi_stm32_buffer_rx {
	rx_func_t process;
	unsigned int len;
	u8_t *buf;
};

struct spi_stm32_buffer_tx {
	tx_func_t process;
	unsigned int len;
	const u8_t *buf;
};

struct spi_stm32_data {
	struct spi_stm32_buffer_tx tx;
	struct spi_stm32_buffer_rx rx;
#ifdef CONFIG_SPI_STM32_INTERRUPT
	struct k_sem sync;
#endif
};

#endif	/* _STM32_SPI_H_ */
