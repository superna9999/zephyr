/*
 * Copyright (c) 2016 BayLibre, SAS
 * Copyright (c) 2017 Linaro Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef _STM32_I2C_H_
#define _STM32_I2C_H_

typedef void (*irq_config_func_t)(struct device *port);

struct i2c_stm32_config {
#ifdef CONFIG_I2C_STM32_INTERRUPT
	irq_config_func_t irq_config_func;
#endif
	struct stm32_pclken pclken;
	I2C_TypeDef *i2c;
	u32_t bitrate;
};

struct i2c_stm32_data {
#ifdef CONFIG_I2C_STM32_INTERRUPT
	struct k_sem device_sync_sem;
#endif
	u32_t dev_config;
#ifdef CONFIG_I2C_STM32_V1
	u16_t slave_address;
#endif
	struct {
#ifdef CONFIG_I2C_STM32_V1
		unsigned int is_restart;
		unsigned int flags;
#endif
		unsigned int is_write;
		unsigned int is_nack;
		unsigned int is_err;
		struct i2c_msg *msg;
		unsigned int len;
		u8_t *buf;
	} current;
#ifdef CONFIG_I2C_SLAVE
	const struct i2c_slave_api *slave_funcs;
	void *slave_priv;
	bool slave_attached;
#endif 
};

s32_t stm32_i2c_msg_write(struct device *dev, struct i2c_msg *msg, u8_t *flg,
			  u16_t sadr);
s32_t stm32_i2c_msg_read(struct device *dev, struct i2c_msg *msg, u8_t *flg,
			 u16_t sadr);
s32_t stm32_i2c_configure_timing(struct device *dev, u32_t clk);
int i2c_stm32_runtime_configure(struct device *dev, u32_t config);

void stm32_i2c_event_isr(void *arg);
void stm32_i2c_error_isr(void *arg);

#if defined(CONFIG_I2C_SLAVE) && defined(CONFIG_I2C_STM32_V2)
bool i2c_stm32_slave_is_supported(struct device *dev);
int i2c_stm32_slave_attach(struct device *dev, u8_t address,
			   const struct i2c_slave_api *funcs,
			   void *priv);
int i2c_stm32_slave_detach(struct device *dev, u8_t address,
			   void *priv);
#endif

#define DEV_DATA(dev) ((struct i2c_stm32_data * const)(dev)->driver_data)
#define DEV_CFG(dev)	\
((const struct i2c_stm32_config * const)(dev)->config->config_info)

#endif	/* _STM32_I2C_H_ */
