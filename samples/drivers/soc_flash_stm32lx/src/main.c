/*
 * Copyright (c) 2016 Linaro Limited
 *               2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <flash.h>
#include <device.h>
#include <stdio.h>

/* Offset between pages */
#define FLASH_TEST_OFFSET 0x3f000
#define FLASH_PAGE_SIZE   2048
#define TEST_DATA_WORD_0  0x11223344
#define TEST_DATA_WORD_1  0xaabbccdd
#define TEST_DATA_WORD_2  0xabcdefab
#define TEST_DATA_WORD_3  0x12344567

void main(void)
{
	struct device *flash_dev;
	u64_t buf_array_1[4] = { TEST_DATA_WORD_0, TEST_DATA_WORD_1,
				    TEST_DATA_WORD_2, TEST_DATA_WORD_3 };
	u64_t buf_array_2[4] = { TEST_DATA_WORD_3, TEST_DATA_WORD_1,
				    TEST_DATA_WORD_2, TEST_DATA_WORD_0 };
	u64_t buf_word = 0;
	u32_t i, offset;

	printf("\nSTM32L4x Flash Testing\n");
	printf("=========================\n");

	flash_dev = device_get_binding(CONFIG_SOC_FLASH_STM32_DEV_NAME);

	if (!flash_dev) {
		printf("STM32L4x flash driver was not found!\n");
		return;
	}

	flash_write_protection_set(flash_dev, false);

	printf("\nTest 1: Flash erase page at 0x%x\n", FLASH_TEST_OFFSET);
	if (flash_erase(flash_dev, FLASH_TEST_OFFSET, FLASH_PAGE_SIZE) != 0) {
		printf("   Flash erase failed!\n");
	} else {
		printf("   Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write (word array 1)\n");
	for (i = 0; i < ARRAY_SIZE(buf_array_1); i++) {
		offset = FLASH_TEST_OFFSET + (i << 3);
		printf("   Attempted to write at 0x%x : %x\n", offset,
				(u32_t)buf_array_1[i]);
		if (flash_write(flash_dev, offset, &buf_array_1[i],
					sizeof(u64_t)) != 0) {
			printf("   Flash write failed!\n");
			return;
		}
		printf("   Attempted to read at 0x%x\n", offset);
		if (flash_read(flash_dev, offset, &buf_word,
					sizeof(u64_t)) != 0) {
			printf("   Flash read failed!\n");
			return;
		}
		printf("   Data read: %x\n", (u32_t)buf_word);
		if (buf_array_1[i] == buf_word) {
			printf("   Data read matches data written. Good!\n");
		} else {
			printf("   Data read does not match data written!\n");
		}
	}

	offset = FLASH_TEST_OFFSET - FLASH_PAGE_SIZE * 2;
	printf("\nTest 3: Flash erase (4 pages at 0x%x)\n", offset);
	if (flash_erase(flash_dev, offset, FLASH_PAGE_SIZE * 4) != 0) {
		printf("   Flash erase failed!\n");
	} else {
		printf("   Flash erase succeeded!\n");
	}

	printf("\nTest 4: Flash write (word array 2)\n");
	for (i = 0; i < ARRAY_SIZE(buf_array_2); i++) {
		offset = FLASH_TEST_OFFSET + (i << 3);
		printf("   Attempted to write at 0x%x : %x\n", offset,
				(u32_t)buf_array_2[i]);
		if (flash_write(flash_dev, offset, &buf_array_2[i],
					sizeof(u64_t)) != 0) {
			printf("   Flash write failed!\n");
			return;
		}
		printf("   Attempted to read at 0x%x\n", offset);
		if (flash_read(flash_dev, offset, &buf_word,
					sizeof(u64_t)) != 0) {
			printf("   Flash read failed!\n");
			return;
		}
		printf("   Data read: %x\n", (u32_t)buf_word);
		if (buf_array_2[i] == buf_word) {
			printf("   Data read matches data written. Good!\n");
		} else {
			printf("   Data read does not match data written!\n");
		}
	}
}
