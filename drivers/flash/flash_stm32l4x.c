/*
 * Copyright (c) 2017 Linaro Limited
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

#include <clock_control.h>
#include <clock_control/stm32_clock_control.h>

#include <flash_registers.h>

#define STM32L4X_BANK_SIZE_MAX	512
#define STM32L4X_PAGE_SHIFT	11

#define STM32L4X_FLASH_TIMEOUT	((u32_t) 0x000B0000)

#define STM32L4X_FLASH_END \
	((u32_t)(STM32L4X_BANK_SIZE_MAX << STM32L4X_PAGE_SHIFT) - 1)

#define FLASH_KEY1	((uint32_t)0x45670123U) /* Flash key1 */
#define FLASH_KEY2	((uint32_t)0xCDEF89ABU)	/* Flash key2 */

struct flash_priv {
	struct stm32l4x_flash *regs;
	/* clock subsystem driving this peripheral */
	struct stm32_pclken pclken;
	struct k_sem sem;
};

/* offset and len must be aligned on 8, positive and not beyond end of flash */
static bool valid_range(off_t offset, u32_t len)
{
	return offset % 8 == 0 &&
	       len % 8 == 0 &&
	       offset >= 0 &&
	       (offset + len - 1 <= STM32L4X_FLASH_END);
}

/* STM32L4xx devices can have up to 512 2K pages on two 256x2K pages banks */
static unsigned int flash_stm32l4x_get_page(off_t offset)
{
	return offset >> STM32L4X_PAGE_SHIFT;
}

static int check_status(struct flash_priv *p)
{
	volatile struct stm32l4x_flash *regs = p->regs;
	u32_t const error =
		FLASH_SR_WRPERR |
		FLASH_SR_PGAERR |
		FLASH_SR_RDERR  |
		FLASH_SR_PGSERR |
		FLASH_SR_OPERR;

	if (regs->sr & error) {
		return -EIO;
	}

	return 0;
}

static int wait_flash_idle(struct flash_priv *p)
{
	volatile struct stm32l4x_flash *regs = p->regs;
	u32_t timeout = STM32L4X_FLASH_TIMEOUT;
	int rc;

	rc = check_status(p);
	if (rc < 0) {
		return -EIO;
	}

	while ((regs->sr & FLASH_SR_BSY) && timeout) {
		timeout--;
	}

	if (!timeout) {
		return -EIO;
	}

	return 0;
}

static int write_dword(off_t offset, u64_t val, struct flash_priv *p)
{
	volatile u32_t *flash = (u32_t *)(offset + CONFIG_FLASH_BASE_ADDRESS);
	volatile struct stm32l4x_flash *regs = p->regs;
	u32_t tmp;
	int rc;

	/* if the control register is locked, do not fail silently */
	if (regs->cr & FLASH_CR_LOCK) {
		return -EIO;
	}

	/* Check that no Flash main memory operation is ongoing */
	rc = wait_flash_idle(p);
	if (rc < 0) {
		return rc;
	}

	/* Check if this double word is erased */
	if (flash[0] != 0xFFFFFFFFUL ||
	    flash[1] != 0xFFFFFFFFUL) {
		return -EIO;
	}

	/* Set the PG bit */
	regs->cr |= FLASH_CR_PG;

	/* Flush the register write */
	tmp = regs->cr;

	/* Perform the data write operation at the desired memory address */
	flash[0] = (uint32_t)val;
	flash[1] = (uint32_t)(val >> 32);

	/* Wait until the BSY bit is cleared */
	rc = wait_flash_idle(p);

	/* Clear the PG bit */
	regs->cr &= (~FLASH_CR_PG);

	return rc;
}

static int erase_page(unsigned int page, struct flash_priv *p)
{
	volatile struct stm32l4x_flash *regs = p->regs;
	u32_t tmp;
	int rc;

	/* if the control register is locked, do not fail silently */
	if (regs->cr & FLASH_CR_LOCK) {
		return -EIO;
	}

	/* Check that no Flash memory operation is ongoing */
	rc = wait_flash_idle(p);
	if (rc < 0) {
		return rc;
	}

	/* Set the PER bit and select the page you wish to erase */
	regs->cr |= FLASH_CR_PER;
#ifdef FLASH_CR_BKER
	regs->cr &= ~FLASH_CR_BKER_Msk;
	/* Select bank, only for DUALBANK devices */
	if (page >= 256)
		regs->cr |= FLASH_CR_BKER;
#endif
	regs->cr &= ~FLASH_CR_PNB_Msk;
	regs->cr |= ((page % 256) << 3);

	/* Set the STRT bit */
	regs->cr |= FLASH_CR_STRT;

	/* flush the register write */
	tmp = regs->cr;

	/* Wait for the BSY bit */
	rc = wait_flash_idle(p);

	regs->cr &= ~FLASH_CR_PER;

	return rc;
}

static void flush_caches(struct flash_priv *p)
{
	volatile struct stm32l4x_flash *regs = p->regs;

	if (regs->acr.val & FLASH_ACR_ICEN) {
		regs->acr.val &= ~FLASH_ACR_ICEN;
		regs->acr.val |= FLASH_ACR_ICRST;
		regs->acr.val &= ~FLASH_ACR_ICRST;
		regs->acr.val |= FLASH_ACR_ICEN;
	}

	if (regs->acr.val & FLASH_ACR_DCEN) {
		regs->acr.val &= ~FLASH_ACR_DCEN;
		regs->acr.val |= FLASH_ACR_DCRST;
		regs->acr.val &= ~FLASH_ACR_DCRST;
		regs->acr.val |= FLASH_ACR_DCEN;
	}
}

static int flash_stm32l4x_erase(struct device *dev, off_t offset, size_t len)
{
	struct flash_priv *p = dev->driver_data;
	int i, rc = 0;

	if (!valid_range(offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	k_sem_take(&p->sem, K_FOREVER);

	i = flash_stm32l4x_get_page(offset);
	for (; i <= flash_stm32l4x_get_page(offset + len - 1) ; ++i) {
		rc = erase_page(i, p);
		if (rc < 0) {
			break;
		}
	}
	flush_caches(p);

	k_sem_give(&p->sem);

	return rc;
}

static int flash_stm32l4x_read(struct device *dev, off_t offset, void *data,
	size_t len)
{
	if (!valid_range(offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	memcpy(data, (void *) CONFIG_FLASH_BASE_ADDRESS + offset, len);

	return 0;
}

static int flash_stm32l4x_write(struct device *dev, off_t offset,
	const void *data, size_t len)
{
	struct flash_priv *p = dev->driver_data;
	int rc, i;

	if (!valid_range(offset, len)) {
		return -EINVAL;
	}

	if (!len) {
		return 0;
	}

	k_sem_take(&p->sem, K_FOREVER);

	for (i = 0; i < len; i += 8, offset += 8) {
		rc = write_dword(offset, ((const u64_t *) data)[i], p);
		if (rc < 0) {
			k_sem_give(&p->sem);
			return rc;
		}
	}

	k_sem_give(&p->sem);

	return 0;
}

static int flash_stm32l4x_write_protection(struct device *dev, bool enable)
{
	struct flash_priv *p = dev->driver_data;
	volatile struct stm32l4x_flash *regs = p->regs;
	int rc = 0;


	k_sem_take(&p->sem, K_FOREVER);

	if (enable) {
		rc = wait_flash_idle(p);
		if (rc) {
			k_sem_give(&p->sem);
			return rc;
		}
		regs->cr |= FLASH_CR_LOCK;
	} else {
		if (regs->cr & FLASH_CR_LOCK) {
			regs->keyr = FLASH_KEY1;
			regs->keyr = FLASH_KEY2;
		}
	}

	k_sem_give(&p->sem);

	return rc;
}

static struct flash_priv flash_data = {
	.regs = (struct stm32l4x_flash *) FLASH_R_BASE,
	.pclken = { .bus = STM32_CLOCK_BUS_AHB1,
		    .enr = LL_AHB1_GRP1_PERIPH_FLASH },
};

static const struct flash_driver_api flash_stm32l4x_api = {
	.write_protection = flash_stm32l4x_write_protection,
	.erase = flash_stm32l4x_erase,
	.write = flash_stm32l4x_write,
	.read = flash_stm32l4x_read,
};

static int stm32l4x_flash_init(struct device *dev)
{
	struct flash_priv *p = dev->driver_data;
	struct device *clk = device_get_binding(STM32_CLOCK_CONTROL_NAME);

	/* enable clock */
	clock_control_on(clk, (clock_control_subsys_t *)&p->pclken);

	k_sem_init(&p->sem, 1, 1);

	return flash_stm32l4x_write_protection(dev, false);
}

DEVICE_AND_API_INIT(stm32l4x_flash,
	CONFIG_SOC_FLASH_STM32_DEV_NAME,
	stm32l4x_flash_init,
	&flash_data,
	NULL,
	POST_KERNEL,
	CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
	&flash_stm32l4x_api);

