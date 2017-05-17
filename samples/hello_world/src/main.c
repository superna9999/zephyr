/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>

void main(void)
{
	volatile u8_t *p8 = (void *)0x64000000UL;
	volatile u16_t *p16 = (void *)0x64000000UL;
	volatile u32_t *p32 = (void *)0x64000000UL;
	printk("Hello World! %s\n", CONFIG_ARCH);
	*p32 = 0xaabbccddUL;
	/*printk("%x\n", *p8);
	*p16 = 0x1122;
	printk("%x\n", *p16);
	*p8 = 0xa5;*/
	printk("%x\n", *p32);
}
