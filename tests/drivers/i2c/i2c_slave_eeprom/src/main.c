/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#define SYS_LOG_LEVEL SYS_LOG_LEVEL_DEBUG
#include <logging/sys_log.h>

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <misc/util.h>

#include <i2c.h>
#include <drivers/i2c/slave_eeprom.h>

#define TEST_DATA_SIZE	20

static u8_t eeprom_data[TEST_DATA_SIZE] = "0123456789abcdefghij";
static u8_t i2c_buffer[TEST_DATA_SIZE];

/*
 * We need 5x(buffer size) + 1 to print a comma-separated list of each
 * byte in hex, plus a null.
 */
u8_t buffer_print_eeprom[TEST_DATA_SIZE * 5 + 1];
u8_t buffer_print_i2c[TEST_DATA_SIZE * 5 + 1];

static void to_display_format(const u8_t *src, size_t size, char *dst)
{
	size_t i;

	for (i = 0; i < size; i++) {
		sprintf(dst + 5 * i, "0x%02x,", src[i]);
	}
}

#define I2C_MASTER_DEV_NAME	"I2C_2"

static int run_full_read(struct device *i2c)
{
	int ret;

	SYS_LOG_INF("Start full read from EEPROM");

	/* Read EEPROM from I2C Master requests, then compare */
	ret = i2c_burst_read(i2c, CONFIG_I2C_SLAVE_EEPROM_ADDRESS,
			     0, i2c_buffer, TEST_DATA_SIZE);
	if (ret) {
		SYS_LOG_ERR("Failed to read EEPROM Code %d", ret);
		return -1;
	}

	if (memcmp(i2c_buffer, eeprom_data, TEST_DATA_SIZE)) {
		to_display_format(i2c_buffer, TEST_DATA_SIZE,
				  buffer_print_i2c);
		to_display_format(eeprom_data, TEST_DATA_SIZE,
				  buffer_print_eeprom);
		SYS_LOG_ERR("Buffer contents are different: %s",
			    buffer_print_i2c);
		SYS_LOG_ERR("                           vs: %s",
			    buffer_print_eeprom);
		return -1;
	}

	return 0;
}

static int run_partial_read(struct device *i2c, unsigned int offset)
{
	int ret;

	SYS_LOG_INF("Start partial read from EEPROM (off=%d)", offset);

	ret = i2c_burst_read(i2c, CONFIG_I2C_SLAVE_EEPROM_ADDRESS,
			     offset, i2c_buffer, TEST_DATA_SIZE-offset);
	if (ret) {
		SYS_LOG_ERR("Failed to read EEPROM Code %d", ret);
		return -1;
	}

	if (memcmp(i2c_buffer, &eeprom_data[offset], TEST_DATA_SIZE-offset)) {
		to_display_format(i2c_buffer, TEST_DATA_SIZE-offset,
				  buffer_print_i2c);
		to_display_format(&eeprom_data[offset], TEST_DATA_SIZE-offset,
				  buffer_print_eeprom);
		SYS_LOG_ERR("Buffer contents are different: %s",
			    buffer_print_i2c);
		SYS_LOG_ERR("                           vs: %s",
			    buffer_print_eeprom);
		return -1;
	}

	return 0;
}

static int run_program_read(struct device *i2c, unsigned int offset)
{
	int ret, i;

	SYS_LOG_INF("Start program of EEPROM (off=%d)", offset);

	for (i = 0 ; i < TEST_DATA_SIZE-offset ; ++i)
		i2c_buffer[i] = i;

	ret = i2c_burst_write(i2c, CONFIG_I2C_SLAVE_EEPROM_ADDRESS,
			      offset, i2c_buffer, TEST_DATA_SIZE-offset);

	if (ret) {
		SYS_LOG_ERR("Failed to write EEPROM Code %d", ret);
		return -1;
	}

	memset(i2c_buffer, 0xFF, TEST_DATA_SIZE);

	/* Read back EEPROM from I2C Master requests, then compare */
	ret = i2c_burst_read(i2c, CONFIG_I2C_SLAVE_EEPROM_ADDRESS,
			     offset, i2c_buffer, TEST_DATA_SIZE-offset);
	if (ret) {
		SYS_LOG_ERR("Failed to read EEPROM Code %d", ret);
		return -1;
	}

	for (i = 0 ; i < TEST_DATA_SIZE-offset ; ++i) {
		if (i2c_buffer[i] != i) {
			to_display_format(i2c_buffer, TEST_DATA_SIZE-offset,
					  buffer_print_i2c);
			SYS_LOG_ERR("Invalid Buffer content: %s",
				    buffer_print_i2c);
			return -1;
		}
	}

	return 0;
}

void main(void)
{
	struct device *eeprom;
	struct device *i2c;
	const struct eeprom_slave_api *funcs;
	int ret, offset;

	i2c = device_get_binding(I2C_MASTER_DEV_NAME);
	if (i2c) {
		SYS_LOG_INF("Found I2C device %s", I2C_MASTER_DEV_NAME);
	} else {
		SYS_LOG_ERR("I2C device %s not found",
			    I2C_MASTER_DEV_NAME);
		return;
	}

	eeprom = device_get_binding(CONFIG_I2C_SLAVE_EEPROM_NAME);
	if (eeprom) {
		SYS_LOG_INF("Found EEPROM device %s", CONFIG_I2C_SLAVE_EEPROM_NAME);
	} else {
		SYS_LOG_ERR("EEPROM device %s not found",
			    CONFIG_I2C_SLAVE_EEPROM_NAME);
		return;
	}

	/* Get EEPROM funcs to read/write the internal memory */
	funcs = i2c_slave_driver_get_funcs(eeprom);
	if (!funcs) {
		SYS_LOG_ERR("Failed to get EEPROM funcs");
		return;
	}

	/* Program dummy bytes */
	ret = funcs->program(eeprom, eeprom_data, TEST_DATA_SIZE);
	if (ret) {
		SYS_LOG_ERR("Failed to program EEPROM");
		return;
	}

	/* Attach EEPROM */
	ret = i2c_slave_driver_attach(eeprom);
	if (ret) {
		SYS_LOG_ERR("Failed to attach EEPROM");
		return;
	}

	SYS_LOG_INF("EEPROM Attached !");

	if (run_full_read(i2c))
		goto out;

	for(offset = 0 ; offset < TEST_DATA_SIZE-1 ; ++offset)
		if (run_partial_read(i2c, offset))
			goto out;

	for(offset = 0 ; offset < TEST_DATA_SIZE-1 ; ++offset)
		if (run_program_read(i2c, offset))
			goto out;

	SYS_LOG_INF("Success !");

out:
	/* Detach EEPROM */
	ret = i2c_slave_driver_detach(eeprom);
	if (ret) {
		SYS_LOG_ERR("Failed to detach EEPROM");
	}

	SYS_LOG_INF("EEPROM Detached !");

	return;
}
