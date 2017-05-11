/*
 * Copyright (c) 2016 RnDity Sp. z o.o.
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <string.h>
#include <flash.h>
#include <init.h>
#include <soc.h>

#include "flash_stm32.h"

bool flash_stm32_valid_range(off_t offset, u32_t len)
{
	u16_t no_of_pages = len / CONFIG_FLASH_PAGE_SIZE;

	/* Check offset and size alignment. */
	if (((offset % CONFIG_FLASH_PAGE_SIZE) != 0) ||
	    ((len % CONFIG_FLASH_PAGE_SIZE) != 0) ||
	    (no_of_pages == 0)) {
		return false;
	}

	return true;
}

static u8_t flash_stm32_program_halfword(struct flash_stm32_priv *p,
					 u32_t address, u16_t data)
{
	volatile struct stm32f3x_flash *reg = p->regs;
	int status;

	__ASSERT_NO_MSG(IS_FLASH_PROGRAM_ADDRESS(address));

	status = flash_stm32_wait_flash_idle(p);

	if (!status) {
		reg->cr |= FLASH_CR_PG;

		*(volatile u16_t *)address = data;

		status = flash_stm32_wait_flash_idle(p);

		reg->cr &= ~FLASH_CR_PG;
	}

	return status;
}

int flash_stm32_check_status(struct flash_stm32_priv *p)
{
	volatile struct stm32f3x_flash *reg = p->regs;
	int status = 0;

	do {
		if ((reg->sr & FLASH_SR_BSY) == FLASH_SR_BSY) {
			status = -EIO;
			break;
		}

		if ((reg->sr & FLASH_SR_WRPERR) != (u32_t)0x00) {
			status = -EIO;
			break;
		}

		if ((reg->sr & FLASH_SR_PGERR) != (u32_t)0x00) {
			status = -EIO;
			break;
		}
	} while (0);

	return status;
}

static int flash_stm32_erase_page(struct flash_stm32_priv *p,
				  u32_t page_address)
{
	volatile struct stm32f3x_flash *reg = p->regs;
	int status;

	__ASSERT_NO_MSG(IS_FLASH_PROGRAM_ADDRESS(page_address));

	status = flash_stm32_wait_flash_idle(p);

	if (!status) {
		reg->cr |= FLASH_CR_PER;
		reg->ar = page_address;
		reg->cr |= FLASH_CR_STRT;

		status = flash_stm32_wait_flash_idle(p);

		reg->cr &= ~FLASH_CR_PER;
	}

	return status;
}

int flash_stm32_erase(struct device *dev, off_t offset, size_t size)
{
	struct flash_stm32_priv *p = dev->driver_data;
	u32_t first_page_addr = 0;
	u32_t last_page_addr = 0;
	u16_t no_of_pages = size / CONFIG_FLASH_PAGE_SIZE;
	u16_t page_index = 0;
	int status;

	/* Check offset and size alignment. */
	if (flash_stm32_valid_range(offset, size)) {
		return -EINVAL;
	}

	/* Find address of the first page to be erased. */
	page_index = offset / CONFIG_FLASH_PAGE_SIZE;

	first_page_addr = CONFIG_FLASH_BASE_ADDRESS +
			  page_index * CONFIG_FLASH_PAGE_SIZE;

	__ASSERT_NO_MSG(IS_FLASH_PROGRAM_ADDRESS(first_page_addr));

	/* Find address of the last page to be erased. */
	page_index = ((offset + size) / CONFIG_FLASH_PAGE_SIZE) - 1;

	last_page_addr = CONFIG_FLASH_BASE_ADDRESS +
			 page_index * CONFIG_FLASH_PAGE_SIZE;

	__ASSERT_NO_MSG(IS_FLASH_PROGRAM_ADDRESS(last_page_addr));

	while (no_of_pages) {
		status = flash_stm32_erase_page(p, first_page_addr);
		if (status) {
			return status;
		}
		no_of_pages--;
		first_page_addr += CONFIG_FLASH_PAGE_SIZE;
	}

	return 0;
}

int flash_stm32_write(struct device *dev, off_t offset,
		      const void *data, size_t len)
{
	struct flash_stm32_priv *p = dev->driver_data;
	u16_t halfword = 0;
	int status;

	u32_t address =
		CONFIG_FLASH_BASE_ADDRESS + offset;

	u8_t remainder = 0;

	if ((len % 2) != 0) {
		remainder = 1;
	}

	len = len / 2;

	while (len--) {
		halfword = *((u8_t *)data++);
		halfword |= *((u8_t *)data++) << 8;
		status = flash_stm32_program_halfword(p, address, halfword);
		if (status) {
			return -EINVAL;
		}
		address += 2;
	}

	if (remainder) {
		halfword = (*((u16_t *)data)) & 0x00FF;
		status = flash_stm32_program_halfword(p, address, halfword);
		if (status) {
			return -EINVAL;
		}
	}

	return 0;
}
