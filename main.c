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
#include "sys.h"


#define RF_SW_CTRL1_PIN                          GPIO4
#define RF_SW_CTRL1_GPIO_PORT                    GPIOA
#define RF_SW_CTRL2_PIN                          GPIO5
#define RF_SW_CTRL2_GPIO_PORT                    GPIOA

#define LED_RED_PORT GPIOB
#define LED_RED_PIN GPIO11

#ifdef NUCLEO_WL55
    #define USART_CONSOLE LPUART1  /* PA2/3 , af8 */
#endif

#ifdef LORAE5MINI
    #define USART_CONSOLE USART1  /* PB6/7 , af7 */
#endif

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
  /* Enable HSE (external osc, 32 MHz)*/
  RCC_CR |= RCC_CR_HSEBYP;
  rcc_osc_on(RCC_HSE);
  rcc_wait_for_osc_ready(RCC_HSE);
  
  struct rcc_clock_scale pll_config = {
    .pllm = 8,
    .plln = 12,
    .pllp = 0,
    .pllq = 0,
    .pllr = RCC_PLLCFGR_PLLR_DIV2,
    .pll_source = RCC_PLLCFGR_PLLSRC_HSE,
    .hpre = 0,
    .ppre1 = 0,
    .ppre2 = 0,
    .ahb_frequency=24e6,
    .apb1_frequency=24e6,
    .apb2_frequency=24e6
  };
  rcc_clock_setup_pll(&pll_config);

  /* Enable clocks for the ports we need */
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
}

static void timer_setup(void)
{
  /* And enable clock for TIMER 2. */
  rcc_periph_clock_enable(RCC_TIM2);
  /* Reset TIM2 peripheral to defaults. */
  rcc_periph_reset_pulse(RST_TIM2);

  /* Enable interrupt. */
  nvic_enable_irq(NVIC_TIM2_IRQ);
  nvic_set_priority(NVIC_TIM2_IRQ, 1);

  /* Setup timer 2 for 1ms resolution. */
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
      
  /* Reset counter value. */
  timer_set_counter(TIM2, 0);
  
  /* Set timer prescaler (Ftim2=1MHz). */
  timer_set_prescaler(TIM2, (24)-1);

  /* Timer will overflow each millisecond. */
  timer_set_period(TIM2, (1000)-1);

  /* Stop Timer2 OC1. */
  timer_disable_oc_output(TIM2, TIM_OC1);
  timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_FROZEN);
  timer_clear_flag(TIM2, TIM_SR_CC1IF);
  timer_disable_irq(TIM2, TIM_DIER_CC1IE);

  /* Enable counter. */
  timer_enable_irq(TIM2, TIM_DIER_UIE);
  timer_enable_counter(TIM2);

}

static void usart_setup(void)
{
    #ifdef LORAE5MINI

    /**
     * LoRa E5 Mini (SeeedStudio) uses USART1 for its main serial
     * communication port.
     */

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
    #endif

    #ifdef NUCLEO_WL55

    /**
     * NUCLEO WL55 board uses LPUART1 by default, exposed through
     * the main USB port.
     */

    /* Enable clocks for peripherals we need */
    rcc_periph_clock_enable(RCC_LPUART1);

    /* Enable LPUART1 IRQ. */
    nvic_set_priority(NVIC_LPUART1_IRQ, 0);
    nvic_enable_irq(NVIC_LPUART1_IRQ);

    /* Setup GPIO pins for LPUART1 transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);

    /* Setup LPUART1 TX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF8, GPIO2);

    /* Setup LPUART1 RX pin as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF8, GPIO3);
    #endif

    /* Configure serial port. */
    usart_set_baudrate(USART_CONSOLE, 115200);
    usart_set_databits(USART_CONSOLE, 8);
    usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
    usart_set_mode(USART_CONSOLE, USART_MODE_TX_RX);
    usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
    usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

    /* Enable serial port Receive interrupt. */
    usart_enable_rx_interrupt(USART_CONSOLE);

    /* Finally enable the serial port. */
    usart_enable(USART_CONSOLE);    
}


void tim2_isr(void)
{ 

  if (TIM_SR(TIM2) & TIM_SR_UIF)
  {
    //gpio_toggle(GPIOB, GPIO11);

    /* Call sys_tick() to get track of each elapsed second. */
    sys_tick();

    /**
     * Handle scheduled packets.
     * 
     * If a packet has to be scheduled for this specific timeslot (1s),
     * then setup a compare value that will trigger an IRQ in order to
     * send this packet when asked.
     **/

    //adapter_send_rdy();

    /* Ack interrupt. */
    TIM_SR(TIM2) &= ~TIM_SR_UIF;
  }

  /* If compare channel 1 is triggered */
  if ((TIM_SR(TIM2) & TIM_SR_CC1IF) && (TIM_DIER(TIM2) & TIM_DIER_CC1IE))
  {
    adapter_send_scheduled_packets();

    /* Ack interrupt. */
    TIM_SR(TIM2) &= ~TIM_SR_CC1IF;
  }
}

#ifdef NUCLEO_WL55
void lpuart1_isr(void)
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
}
#endif

#ifdef LORAE5MINI
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
}
#endif

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
  timer_setup();

  /* LEDs */
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
  gpio_clear(GPIOB, GPIO11);
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9);
  gpio_clear(GPIOB, GPIO9);

  /* Initialize WHAD layer. */
  whad_init(&transport_config);

  /* Initialize adapter. */
  adapter_init();

  //whad_transport_send((uint8_t *)"This is a test\r\n", 16);
  gpio_set(GPIOB, GPIO11);
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

