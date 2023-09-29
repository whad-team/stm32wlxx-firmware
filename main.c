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
#include <stdio.h>
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
#include "adapter.h"


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

static Message message;

void uart_send_buffer_sync(uint8_t *p_data, int size);

static whad_transport_cfg_t transport_config = {
  .max_txbuf_size = 64,
  .pfn_data_send_buffer = uart_send_buffer_sync,
  .pfn_message_cb = NULL
};

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
  rcc_periph_clock_enable(RCC_GPIOC);
}

static void usart_setup(void)
{
    /* Enable clocks for peripherals we need */
    rcc_periph_clock_enable(RCC_USART1);

    /* Enable USART1 IRQ. */
    nvic_set_priority(NVIC_USART1_IRQ, 0);
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Setup GPIO pins for USART1 transmit. */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);

    /* Setup USART1 TX pin as alternate function. */
    gpio_set_af(GPIOB, GPIO_AF7, GPIO6);

    /* Setup USART1 RX pin as alternate function. */
    gpio_set_af(GPIOB, GPIO_AF7, GPIO7);

    usart_set_baudrate(USART_CONSOLE, 115200);
    usart_set_databits(USART_CONSOLE, 8);
    usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
    usart_set_mode(USART_CONSOLE, USART_MODE_TX_RX);
    usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
    usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

    /* Enable USART Receive interrupt. */
    usart_enable_rx_interrupt(USART_CONSOLE);

    /* Finally enable the USART. */
    usart_enable(USART_CONSOLE);    
}

void usart1_isr(void)
{
  static uint8_t data = 'A';


	/* Did we receive a byte ? */
	if (((USART_CR1(USART_CONSOLE) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART_CONSOLE) & USART_ISR_RXNE) != 0)) {

        gpio_toggle(GPIOB, GPIO9);

        /* Read byte */
        data = usart_recv(USART_CONSOLE);
        
        /* Forward received byte to whad library. */
        whad_transport_data_received(&data, 1);
  }

#if 0
  /* Check if we were called because of TXE. */
	if (((USART_CR1(USART_CONSOLE) & USART_CR1_TXEIE) != 0) &&
	    ((USART_ISR(USART_CONSOLE) & USART_ISR_TXE) != 0)) {

		/* Put data into the transmit register. */
		usart_send(USART_CONSOLE, data);

		/* Disable the TXE interrupt as we don't need it anymore. */
		usart_disable_tx_interrupt(USART_CONSOLE);
	}
#endif  
}

void uart_send_buffer_sync(uint8_t *p_data, int size)
{
  int i;
  
  for (i=0; i<size; i++)
  {
    usart_send_blocking(USART_CONSOLE, p_data[i]);
  }

  /* Notify whad transport that we sent all the data. */
  whad_transport_data_sent();
}


void print_dbg(char *psz_debug)
{
  for (int i=0; i<strlen(psz_debug); i++)
    usart_send_blocking(USART_CONSOLE, psz_debug[i]);
}

void print_reg16_hex(char *prefix, uint16_t value)
{
  int i;

  char digits[] = "0123456789abcdef";
  char digit[4];
  digit[0] = digits[(value & 0xf000) >> 12];
  digit[1] = digits[(value & 0x0f00) >> 8];
  digit[2] = digits[(value & 0x00f0) >> 4];
  digit[3] = digits[(value & 0x000f)];
  for (i=0; i<strlen(prefix); i++)
    usart_send_blocking(USART_CONSOLE, prefix[i]);
  usart_send_blocking(USART_CONSOLE, '=');
  usart_send_blocking(USART_CONSOLE, digit[0]);
  usart_send_blocking(USART_CONSOLE, digit[1]);
  usart_send_blocking(USART_CONSOLE, digit[2]);
  usart_send_blocking(USART_CONSOLE, digit[3]);
  usart_send_blocking(USART_CONSOLE, '\r');
  usart_send_blocking(USART_CONSOLE, '\n');
}

int main(void)
{
  /* Setup clock & UART */
  clock_setup_bis();
  usart_setup();

  /* LEDs */
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
  gpio_clear(GPIOB, GPIO15);
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
  gpio_clear(GPIOB, GPIO9);

  /* Initialize WHAD layer. */
  whad_init(&transport_config);

  /* Initialize adapter. */
  adapter_init();

  //whad_transport_send((uint8_t *)"This is a test\r\n", 16);
  gpio_set(GPIOB, GPIO15);
  gpio_set(GPIOB, GPIO9);

  //print_dbg("OK\r\n");

  while (1) {
    /* Monitor async rx */
    if(whad_get_message(&message) == WHAD_SUCCESS)
    {
      //print_dbg("Message decoded\r\n");
      dispatch_message(&message);
    }
    /*
    else
    {
      print_dbg("Error while decoding\r\n");
    }
    */

    /* Send pending data to UART. */
    whad_transport_send_pending();
  }

	return 0;
}

