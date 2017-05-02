/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <string.h>
#include <flash.h>
#include <init.h>
#include <soc.h>

#include <flash_stm32.h>

bool flash_stm32_valid_range(off_t offset, u32_t len)
{
	return offset >= 0 && (offset + len - 1 <= STM32F4X_FLASH_END);
}

int flash_stm32_check_status(struct flash_stm32_priv *p)
{
	volatile struct stm32f4x_flash *regs = p->regs;

	u32_t const error =
		FLASH_FLAG_WRPERR |
		FLASH_FLAG_PGAERR |
#if defined(FLASH_FLAG_RDERR)
		FLASH_FLAG_RDERR  |
#endif
		FLASH_FLAG_PGPERR |
		FLASH_FLAG_PGSERR |
		FLASH_FLAG_OPERR;

	if (regs->sr & error) {
		return -EIO;
	}

	return 0;
}

static int write_byte(off_t offset, u8_t val, struct flash_stm32_priv *p)
{
	volatile struct stm32f4x_flash *regs = p->regs;
	u32_t tmp;
	int rc;

	/* if the control register is locked, do not fail silently */
	if (regs->cr & FLASH_CR_LOCK) {
		return -EIO;
	}

	rc = flash_stm32_wait_flash_idle(p);
	if (rc < 0) {
		return rc;
	}

	regs->cr &= ~CR_PSIZE_MASK;
	regs->cr |= FLASH_PSIZE_BYTE;
	regs->cr |= FLASH_CR_PG;

	/* flush the register write */
	tmp = regs->cr;

	*((u8_t *) offset + CONFIG_FLASH_BASE_ADDRESS) = val;

	rc = flash_stm32_wait_flash_idle(p);
	regs->cr &= (~FLASH_CR_PG);

	return rc;
}

static int erase_sector(u16_t sector, struct flash_stm32_priv *p)
{
	volatile struct stm32f4x_flash *regs = p->regs;
	u32_t tmp;
	int rc;

	/* if the control register is locked, do not fail silently */
	if (regs->cr & FLASH_CR_LOCK) {
		return -EIO;
	}

	rc = flash_stm32_wait_flash_idle(p);
	if (rc < 0) {
		return rc;
	}

	regs->cr &= STM32F4X_SECTOR_MASK;
	regs->cr |= FLASH_CR_SER | (sector << 3);
	regs->cr |= FLASH_CR_STRT;

	/* flush the register write */
	tmp = regs->cr;

	rc = flash_stm32_wait_flash_idle(p);
	regs->cr &= ~(FLASH_CR_SER | FLASH_CR_SNB);

	return rc;
}

int flash_stm32_erase(struct device *dev, off_t offset, size_t len)
{
	struct flash_stm32_priv *p = dev->driver_data;
	int i, rc = 0;

	if (!flash_stm32_valid_range(offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	k_sem_take(&p->sem, K_FOREVER);

	i = stm32f4x_get_sector(offset);
	for (; i <= stm32f4x_get_sector(offset + len - 1); i++) {
		rc = erase_sector(i, p);
		if (rc < 0) {
			break;
		}
	}
	flash_stm32_flush_caches(p);

	k_sem_give(&p->sem);

	return rc;
}

int flash_stm32_write(struct device *dev, off_t offset,
		      const void *data, size_t len)
{
	struct flash_stm32_priv *p = dev->driver_data;
	int rc, i;

	if (!flash_stm32_valid_range(offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	k_sem_take(&p->sem, K_FOREVER);

	for (i = 0; i < len; i++, offset++) {
		rc = write_byte(offset, ((const u8_t *) data)[i], p);
		if (rc < 0) {
			k_sem_give(&p->sem);
			return rc;
		}
	}

	k_sem_give(&p->sem);

	return 0;
}
