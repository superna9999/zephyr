/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/util.h>
#include <kernel.h>
#include <errno.h>
#include <i2c.h>
#include <string.h>
#include <drivers/i2c/slave_eeprom.h>

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_I2C_SLAVE_LEVEL
#include <logging/sys_log.h>

#define EEPROM_SIZE (CONFIG_I2C_SLAVE_EEPROM_SIZE * 1024)

struct i2c_eeprom_slave_data
{
	struct device *i2c_controller;
	unsigned int i2c_address;
	u8_t buffer[EEPROM_SIZE];
	u8_t buffer_idx;
	bool first_write;
};

static int eeprom_slave_program(struct device *dev, u8_t *eeprom_data,
				unsigned int length)
{
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	if (length > EEPROM_SIZE)
		return -EINVAL;

	memcpy(data->buffer, eeprom_data, length);

	return 0;
}

static int eeprom_slave_read(struct device *dev, u8_t *eeprom_data,
			     unsigned int offset)
{
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	if (!data || offset >= EEPROM_SIZE)
		return -EINVAL;

	*eeprom_data = data->buffer[offset];

	return 0;
}

static const struct eeprom_slave_api eeprom_slave_funcs = {
	.program = eeprom_slave_program,
	.read = eeprom_slave_read,
};

static int eeprom_slave_write_request(void * priv)
{
	struct device *dev = priv;
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	SYS_LOG_DBG("eeprom: write req");

	data->first_write = true;

	return 0;
}

static int eeprom_slave_read_request(void * priv, u8_t *val)
{
	struct device *dev = priv;
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	SYS_LOG_DBG("eeprom: read req %d", data->buffer_idx);

	*val = data->buffer[data->buffer_idx];

	return 0;
}

static int eeprom_slave_write_done(void * priv, u8_t val)
{
	struct device *dev = priv;
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	SYS_LOG_DBG("eeprom: write done %x", val);

	if (data->first_write) {
		data->buffer_idx = val;
		data->first_write = false;
	} else {
		data->buffer[data->buffer_idx++] = val;
	}

	data->buffer_idx = data->buffer_idx % EEPROM_SIZE;

	return 0;
}

static int eeprom_slave_read_done(void * priv)
{
	struct device *dev = priv;
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	SYS_LOG_DBG("eeprom: read done");

	data->buffer_idx = (data->buffer_idx + 1) % EEPROM_SIZE;

	return 0;
}

static int eeprom_slave_stop(void * priv)
{
	struct device *dev = priv;
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	SYS_LOG_DBG("eeprom: stop");

	data->first_write = true;

	return 0;
}

static const struct i2c_slave_api i2c_slave_funcs = {
	.write_request = eeprom_slave_write_request,
	.read_request = eeprom_slave_read_request,
	.write_done = eeprom_slave_write_done,
	.read_done = eeprom_slave_read_done,
	.stop = eeprom_slave_stop,
};

static int eeprom_slave_attach(struct device *dev)
{
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	return i2c_slave_attach(data->i2c_controller, data->i2c_address,
				&i2c_slave_funcs, dev);
}

static int eeprom_slave_detach(struct device *dev)
{
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	return i2c_slave_detach(data->i2c_controller, data->i2c_address, dev);
}

static const void * eeprom_slave_get_funcs(struct device *dev)
{
	return &eeprom_slave_funcs;
}

static const struct i2c_slave_driver_api api_funcs = {
	.attach = eeprom_slave_attach,
	.detach = eeprom_slave_detach,
	.get_funcs = eeprom_slave_get_funcs,
};

static int i2c_eeprom_slave_init(struct device *dev)
{
	struct i2c_eeprom_slave_data *data = dev->driver_data;

	data->i2c_controller =
		device_get_binding(CONFIG_I2C_SLAVE_EEPROM_CONTROLLER_DEV_NAME);
	if (!data->i2c_controller) {
		SYS_LOG_ERR("i2c controller not found: %s",
			    CONFIG_I2C_SLAVE_EEPROM_CONTROLLER_DEV_NAME);
		return -EINVAL;
	}

	if (!i2c_slave_is_supported(data->i2c_controller)) {
		SYS_LOG_ERR("i2c controller does not support slave ops: %s",
			    CONFIG_I2C_SLAVE_EEPROM_CONTROLLER_DEV_NAME);
		return -EINVAL;
	}

	data->i2c_address = CONFIG_I2C_SLAVE_EEPROM_ADDRESS;

	return 0;
}

#ifdef CONFIG_I2C_SLAVE_EEPROM

static struct i2c_eeprom_slave_data i2c_eeprom_slave_dev_data;

DEVICE_AND_API_INIT(i2c_eeprom_slave, CONFIG_I2C_SLAVE_EEPROM_NAME,
		    &i2c_eeprom_slave_init,
		    &i2c_eeprom_slave_dev_data, NULL,
		    POST_KERNEL, CONFIG_I2C_SLAVE_INIT_PRIORITY,
		    &api_funcs);

#endif /* CONFIG_I2C_SLAVE_EEPROM */
