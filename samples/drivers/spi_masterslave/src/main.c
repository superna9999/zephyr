/*
 * Copyright (c) 2017 jorge.ramirez-ortiz@linaro.org
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <zephyr.h>

#define SYS_LOG_LEVEL SYS_LOG_SPI_LEVEL

#include <logging/sys_log.h>
#include <misc/printk.h>
#include <sys_clock.h>
#include <string.h>
#include <spi.h>

/* CONFIGURATIONS KNOBS */
#define CONFIG_SPI_SLAVE		0
#define HW_LOOPBACK			1
/*----------------------*/


#if !defined(CONFIG_SPI_STM32)
#error test not supported
#endif

#define SET_FRAME_SIZE 			SPI_WORD(8)
#define SPI_MAX_CLK_FREQ_250KHZ 	512000
#define SPI_SLAVE 			0

#include <drivers/spi/spi_ll_stm32.h>

#define DRV_NAME 			CONFIG_SPI_1_NAME

#if CONFIG_SPI_SLAVE
	char *msg = "SLAVE ";
	#define MARK			(0x5A)
	#define MODE			STM32_SPI_MODE_SLAVE
	#define CONFIGS			0
#else
	char *msg = "MASTER";
	#define	MARK			(0xA5)
	#define MODE			STM32_SPI_MODE_MASTER
	#if HW_LOOPBACK
		#define CONFIGS		STM32_SPI_NSS_IGNORE
	#else
		#define CONFIGS		0
	#endif
#endif

#define BUF_LEN		255
unsigned char rbuf[BUF_LEN];
unsigned char wbuf[BUF_LEN];

#define COUNT "\ttransfers: "
#define COUNT_STR "\b\b\b\b\b\b\b\b\b\b\b\b"

#define MAX_LOOP	1000000
#define str1	"\b"COUNT_STR
#define str2	"\b\b"COUNT_STR
#define str3	"\b\b\b"COUNT_STR
#define str4	"\b\b\b\b"COUNT_STR
#define str5	"\b\b\b\b\b"COUNT_STR
#define str6	"\b\b\b\b\b\b"COUNT_STR

static void report_progress(int i)
{
	char *p;

	if (i < 10) {
		p = str1;
	}
	else if (i < 100) {
		p = str2;
	}
	else if (i < 1000) {
		p = str3;
	}
	else if (i < 10000) {
		p = str4;
	}
	else if (i < 100000) {
		p = str5;
	}
	else if (i < 1000000) {
		p = str6;
	}
	else {
		/* ignore */
		return;
	}

	/* todo: fix printk implmentation to accept the format %09d */
	printk("%s%d%s",COUNT, i, p);
}

static int check_transfer(unsigned char *p, unsigned char *q, int len)
{
	int i;

	for (i = 0; i < len; i++, p++, q++)
	{
#if HW_LOOPBACK == 0
		if ((*p & 0x000000FF) != (~*q & 0x000000FF) ) {
#else
		if ((*p & 0x000000FF) != (*q & 0x000000FF) ) {
#endif
			printk("expected 0x%x, received 0x%x\n"
			       "transfer size %d bytes\n"
			       "error on byte %d\n", *q, *p, len, i);
			return -1;
		}
	}

	return 0;
}

struct spi_config spi_conf = {
	.config = MODE | CONFIGS | SET_FRAME_SIZE,
	.max_sys_freq = SPI_MAX_CLK_FREQ_250KHZ,
};

void main(void)
{
	struct device *spi;
	int len = BUF_LEN;
	int i, j, k;;
	int ret;

	printk("\nSPI VALIDATOR: %s\n"
	       "\t%s, %s\n",
		msg,
		DRV_NAME, HW_LOOPBACK? "Loopback ON": "Loopback OFF");

	printk("\t%s %s\n", __DATE__, __TIME__);

	spi = device_get_binding(DRV_NAME);
	if (!spi) {
		printk("cannot find device %s\n", DRV_NAME);
		return;
	}

	ret = spi_configure(spi, &spi_conf);
	if (ret) {
		printk("configuration error %s\n", DRV_NAME);
		return;
	}
	spi_slave_select(spi, SPI_SLAVE);

	wbuf[0] = MARK;
	for (i = 1; i < BUF_LEN; i++)
		wbuf[i] = ~wbuf[i-1];

	for (j = 1, k = 0; ;j++, len--) {

		if (len == 0)
			len = BUF_LEN;

		memset(rbuf, 0xFF, sizeof(rbuf));

		ret = spi_transceive(spi, wbuf, len, rbuf, len);
		if (ret  < 0) {
			printk("spi_transcieve error (%i)", ret);
		}

		ret = check_transfer(rbuf, wbuf, len);
		if (ret) {
			printk("failed after %d transfers\n", j + k * MAX_LOOP);
			break;
		}
#ifndef CONFIG_SYS_LOG
		else {
			if (j >= MAX_LOOP) {
				printk("\n %s: ", msg);

				/* start again (j) and keep count (k) */
				j = 1;
				k++;
			}
			report_progress(j);
		}
#endif
	}

	printk("done\n");
}
