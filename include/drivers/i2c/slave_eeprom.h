/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef I2C_SLAVE_EEPROM_H
#define I2C_SLAVE_EEPROM_H

typedef int (*eeprom_slave_program_t)(struct device *dev, u8_t *data,
				      unsigned int length);
typedef int (*eeprom_slave_read_t)(struct device *dev, u8_t *data,
				   unsigned int offset);

struct eeprom_slave_api {
	eeprom_slave_program_t program;
	eeprom_slave_read_t read;
};

#endif /* I2C_SLAVE_EEPROM_H */
