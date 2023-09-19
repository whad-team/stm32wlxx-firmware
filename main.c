/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2018 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <whad.h>

#include "lora/subghz.h"


#define RF_SW_CTRL1_PIN                          GPIO4
#define RF_SW_CTRL1_GPIO_PORT                    GPIOA
#define RF_SW_CTRL2_PIN                          GPIO5
#define RF_SW_CTRL2_GPIO_PORT                    GPIOA

#define LED_RED_PORT GPIOB
#define LED_RED_PIN GPIO11

#define HF_PA_CTRL1_PORT GPIOC
#define HF_PA_CTRL1_PIN  GPIO3
#define HF_PA_CTRL2_PORT GPIOC
#define HF_PA_CTRL2_PIN  GPIO4
#define HF_PA_CTRL3_PORT GPIOC
#define HF_PA_CTRL3_PIN  GPIO5

#define USART_CONSOLE USART1  /* PB6/7 , af7 */


/* Setup APB1 frequency to 24MHz */
static void clock_setup_bis(void)
{
  struct rcc_clock_scale pll_config = {
    .pllm = 4,
    .plln = 12,
    .pllp = 0,
    .pllq = 0,
    .pllr = RCC_PLLCFGR_PLLR_DIV2,
    .pll_source = RCC_PLLCFGR_PLLSRC_HSI16,
    .hpre = 0,
    .ppre1 = 0,
    .ppre2 = 0,
    .ahb_frequency=24e6,
    .apb1_frequency=24e6,
    .apb2_frequency=24e6
  };
  rcc_clock_setup_pll(&pll_config);

  RCC_CR |= RCC_CR_HSEBYP;
  rcc_osc_on(RCC_HSE);
  rcc_wait_for_osc_ready(RCC_HSE);

	/* Enable clocks for the ports we need */
  rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for peripherals we need */
	rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void)
{
	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF7, GPIO6);
	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Enable USART Receive interrupt. */
	USART_CR1(USART_CONSOLE) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

void usart1_isr(void)
{
  static uint8_t data = 'A';
  
  /* Check if we were called because of RXNE. */
	if (((USART_CR1(USART_CONSOLE) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART_CONSOLE) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART_CONSOLE);

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART_CONSOLE) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART_CONSOLE) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART_CONSOLE) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		usart_send(USART_CONSOLE, data);

		/* Disable the TXE interrupt as we don't need it anymore. */
		USART_CR1(USART_CONSOLE) &= ~USART_CR1_TXEIE;
	}
}

int main(void)
{
  Message msg;

  /* Setup clock & UART */
  clock_setup_bis();
  usart_setup();

  /* Initialize SUBGHZ. */
  subghz_init();

	while (1) {
    /* Process incoming messages. */

	}

	return 0;
}
