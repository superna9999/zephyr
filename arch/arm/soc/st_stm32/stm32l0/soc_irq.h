/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32L0_SOC_IRQ_H_
#define _STM32L0_SOC_IRQ_H_

/*
 * We cannot use the enum present in the ST headers for the IRQs because
 * of the IRQ_CONNECT macro. The macro exepects a number or a symbol that can
 * be processed by the preprocessor.
 */

#define STM32L0_IRQ_WWDG		0
#define STM32L0_IRQ_PVD			1
#define STM32L0_IRQ_RTC			2
#define STM32L0_IRQ_FLASH		3
#define STM32L0_IRQ_RCC			4
#define STM32L0_IRQ_EXTI_1_0		5
#define STM32L0_IRQ_EXTI_3_2		6
#define STM32L0_IRQ_EXTI_15_4		7
#define STM32L0_IRQ_TSC			8
#define STM32L0_IRQ_DMA1_CH1		9
#define STM32L0_IRQ_DMA1_CH_3_2		10
#define STM32L0_IRQ_DMA1_CH_7_4		11
#define STM32L0_IRQ_ADC			12
#define STM32L0_IRQ_LPTIM1		13
#define STM32L0_IRQ_USART4_5		14
#define STM32L0_IRQ_TIM2		15
#define STM32L0_IRQ_TIM3		16
#define STM32L0_IRQ_TIM6_DAC		17
#define STM32L0_IRQ_TIM7		18
#define STM32L0_IRQ_TIM21		20
#define STM32L0_IRQ_I2C3		21
#define STM32L0_IRQ_TIM22		22
#define STM32L0_IRQ_I2C1		23
#define STM32L0_IRQ_I2C2		24
#define STM32L0_IRQ_SPI1		25
#define STM32L0_IRQ_SPI2		26
#define STM32L0_IRQ_USART1		27
#define STM32L0_IRQ_USART2		28
#define STM32L0_IRQ_LPUART1_AES_RNG	29
#define STM32L0_IRQ_LCD			30
#define STM32L0_IRQ_USB			31

#endif	/* _STM32L0_SOC_IRQ_H_ */
