/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <i2c.h>
#include <init.h>
#include <sensor.h>
#include <misc/__assert.h>

#include "lib3dsh.h"

static void lib3dsh_convert(struct sensor_value *val, s64_t raw_val)
{
	/* val = raw_val * LIS3DSH_ACCEL_SCALE / (10^6 * (2^16 - 1)) */
	raw_val = raw_val * LIS3DSH_ACCEL_SCALE / 1000000;
	val->val1 = raw_val / 0xFFFF;
	val->val2 = (raw_val % 0xFFFF) * 1000000 / 0xFFFF;

	/* normalize val to make sure val->val2 is positive */
	if (val->val2 < 0) {
		val->val1 -= 1;
		val->val2 += 1000000;
	}
}

static int lib3dsh_channel_get(struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct lib3dsh_data *drv_data = dev->driver_data;

	if (chan == SENSOR_CHAN_ACCEL_X) {
		lib3dsh_convert(val, drv_data->x_sample);
	} else if (chan == SENSOR_CHAN_ACCEL_Y) {
		lib3dsh_convert(val, drv_data->y_sample);
	} else if (chan == SENSOR_CHAN_ACCEL_Z) {
		lib3dsh_convert(val, drv_data->z_sample);
	} else if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		lib3dsh_convert(val, drv_data->x_sample);
		lib3dsh_convert(val + 1, drv_data->y_sample);
		lib3dsh_convert(val + 2, drv_data->z_sample);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

int lib3dsh_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct lib3dsh_data *drv_data = dev->driver_data;
	u8_t buf[6];

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_ACCEL_XYZ);

	/*
	 * since all accel data register addresses are consecutive,
	 * a burst read can be used to read all the samples
	 */
	if (i2c_burst_read(drv_data->i2c, LIS3DSH_I2C_ADDRESS,
			   LIS3DSH_REG_ACCEL_X_LSB, buf, 6) < 0) {
		SYS_LOG_DBG("Could not read accel axis data");
		return -EIO;
	}

	drv_data->x_sample = (buf[1] << 8) | buf[0];
	drv_data->y_sample = (buf[3] << 8) | buf[2];
	drv_data->z_sample = (buf[5] << 8) | buf[4];

	return 0;
}

static const struct sensor_driver_api lib3dsh_driver_api = {
#if CONFIG_LIS3DSH_TRIGGER
	.trigger_set = lib3dsh_trigger_set,
#endif
	.sample_fetch = lib3dsh_sample_fetch,
	.channel_get = lib3dsh_channel_get,
};

int lib3dsh_init(struct device *dev)
{
	struct lib3dsh_data *drv_data = dev->driver_data;

	drv_data->i2c = device_get_binding(CONFIG_LIS3DSH_I2C_MASTER_DEV_NAME);
	if (drv_data->i2c == NULL) {
		SYS_LOG_DBG("Could not get pointer to %s device",
		    CONFIG_LIS3DSH_I2C_MASTER_DEV_NAME);
		return -EINVAL;
	}

	/* enable accel measurements and set power mode and data rate */
	if (i2c_reg_write_byte(drv_data->i2c, LIS3DSH_I2C_ADDRESS,
			       LIS3DSH_REG_CTRL4, LIS3DSH_ACCEL_EN_BITS |
			       LIS3DSH_ODR_BITS) < 0) {
		SYS_LOG_DBG("Failed to configure chip.");
	}

	/* set full scale range */
	if (i2c_reg_write_byte(drv_data->i2c, LIS3DSH_I2C_ADDRESS,
			       LIS3DSH_REG_CTRL5, LIS3DSH_FS_BITS) < 0) {
		SYS_LOG_DBG("Failed to set full scale range.");
		return -EIO;
	}

#ifdef CONFIG_LIS3DSH_TRIGGER
	if (lib3dsh_init_interrupt(dev) < 0) {
		SYS_LOG_DBG("Failed to initialize interrupts.");
		return -EIO;
	}
#endif

	dev->driver_api = &lib3dsh_driver_api;

	return 0;
}

struct lib3dsh_data lib3dsh_driver;

DEVICE_INIT(lib3dsh, CONFIG_LIS3DSH_NAME, lib3dsh_init, &lib3dsh_driver,
	    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);
