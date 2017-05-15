/*
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <arch/arm/cortex_m/mpu/arm_mpu.h>

#include "arm_mpu_mem_cfg.h"

/* SoC Private Peripheral Bus */
#define PPB_BASE  0xE0000000

/* Flash permission */
#if defined(CONFIG_STM32_MPU_FLASH_RO)
#define REGION_FLASH_0_ATTR REGION_FLASH_PERM_ATTR(REGION_FLASH_SIZE, RO)
#elif defined(CONFIG_STM32_MPU_FLASH_P_RW_U_NA)
#define REGION_FLASH_0_ATTR REGION_FLASH_PERM_ATTR(REGION_FLASH_SIZE, P_RW_U_NA)
#elif defined(CONFIG_STM32_MPU_FLASH_P_RW_U_RO)
#define REGION_FLASH_0_ATTR REGION_FLASH_PERM_ATTR(REGION_FLASH_SIZE, P_RW_U_RO)
#elif defined(CONFIG_STM32_MPU_FLASH_FULL_ACCESS)
#define REGION_FLASH_0_ATTR REGION_FLASH_PERM_ATTR(REGION_FLASH_SIZE, P_RW_U_RW)
#elif defined(CONFIG_STM32_MPU_FLASH_P_RO_U_NA)
#define REGION_FLASH_0_ATTR REGION_FLASH_PERM_ATTR(REGION_FLASH_SIZE, P_RO_U_NA)
#else
#error Invalid Flash Permissions
#endif

static struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0",
			 CONFIG_FLASH_BASE_ADDRESS,
			 REGION_FLASH_0_ATTR),
	/* Region 1 */
	MPU_REGION_ENTRY("RAM_0",
			 CONFIG_SRAM_BASE_ADDRESS,
			 REGION_RAM_ATTR(REGION_SRAM_0_SIZE)),
	/* Region 2 */
	MPU_REGION_ENTRY("RAM_1",
			 (CONFIG_SRAM_BASE_ADDRESS + REGION_SRAM_1_START),
			 REGION_RAM_ATTR(REGION_SRAM_1_SIZE)),
	/* Region 3 */
	MPU_REGION_ENTRY("PERIPHERAL_0",
			 APB1PERIPH_BASE,
			 REGION_IO_ATTR(REGION_512M)),
	/* Region 4 */
	MPU_REGION_ENTRY("PPB_0",
			 PPB_BASE,
			 REGION_PPB_ATTR(REGION_256M)),
#if defined(CONFIG_BL_APPLICATION)
	/* Region 5 */
	/*
	 * The application booting from a bootloader has no access to the
	 * bootloader region. This behavior can be changed at runtime by
	 * the bootloader.
	 */
	MPU_REGION_ENTRY("BOOTLOADER_0",
			 CONFIG_FLASH_BASE_ADDRESS,
			 (NORMAL_OUTER_INNER_NON_CACHEABLE_NON_SHAREABLE |
			  REGION_32K | P_NA_U_NA)),
#endif
};

struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
