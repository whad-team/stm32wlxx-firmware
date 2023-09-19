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
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
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

int _write(int file, char *ptr, int len);

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
    //.pll_source = RCC_PLLCFGR_PLLSRC_HSE,
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

  /* Setup PCLK3 prescaler to 2 (24/2 = 12MHz) */
  //RCC_EXTCFGR = 0x08;
  //RCC_EXTCFGR |= (1<<16);

	/* Enable clocks for the ports we need */
  rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for peripherals we need */
	rcc_periph_clock_enable(RCC_USART1);
}


#if 0
static void clock_setup(void)
{
	/* FIXME - this should eventually become a clock struct helper setup */
	rcc_osc_on(RCC_HSE);

	/* 16MHz / 4 = > 4 * 12 = 48MHz VCO => 24MHz main pll  */
	rcc_set_main_pll(
    RCC_PLLCFGR_PLLSRC_HSE,
    4,
    12,
		0,
    0,
    RCC_PLLCFGR_PLLR_DIV2
  );
	
  rcc_osc_on(RCC_PLL);
	/* either rcc_wait_for_osc_ready() or do other things */

	/* Enable clocks for the ports we need */
  rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable clocks for peripherals we need */
	rcc_periph_clock_enable(RCC_USART1);

  /* TODO: figure out what's going on with our PLL ?!$@ */

	rcc_set_sysclk_source(RCC_CFGR_SW_PLL); /* careful with the param here! */
	rcc_wait_for_sysclk_status(RCC_PLL);

	/* 48MHz */
	rcc_ahb_frequency = 48e6;
	rcc_apb1_frequency = 48e6;
	rcc_apb2_frequency = 48e6;
}
#endif

static void usart_setup(void)
{
	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(GPIOB, GPIO_AF7, GPIO6);

  /*
	usart_set_baudrate(USART_CONSOLE, 4800);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_EVEN);
  */
	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

/**
 * Use USART_CONSOLE as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

void print_str(char *str, int length)
{
  int i;

  for (i=0; i<length; i++)
      usart_send_blocking(USART_CONSOLE, str[i]);  
  usart_send_blocking(USART_CONSOLE, '\r');
  usart_send_blocking(USART_CONSOLE, '\n');
}

void print_reg_hex(char *prefix, uint8_t value)
{
  int i;

  char digits[] = "0123456789abcdef";
  char digit[3];
  digit[0] = digits[(value & 0xf0)>>4];
  digit[1] = digits[(value & 0x0f)];
  digit[2] = '\0';
  for (i=0; i<strlen(prefix); i++)
    usart_send_blocking(USART_CONSOLE, prefix[i]);
  usart_send_blocking(USART_CONSOLE, '=');
  usart_send_blocking(USART_CONSOLE, digit[0]);
  usart_send_blocking(USART_CONSOLE, digit[1]);
  usart_send_blocking(USART_CONSOLE, '\r');
  usart_send_blocking(USART_CONSOLE, '\n');
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

void on_rf_switch_cb(bool tx);
void on_tx_pkt_sent(void);
void on_rx_pkt_recvd(uint8_t offset, uint8_t length);

int main(void)
{
  int res;
  uint8_t packet[255];
  uint8_t packet_size = 255;

  subghz_callbacks_t my_callbacks = {
    .pfn_on_packet_recvd = on_rx_pkt_recvd,
    .pfn_on_packet_sent = on_tx_pkt_sent,
    .pfn_on_rf_switch = on_rf_switch_cb,
    .pfn_on_timeout = NULL
  };

  subghz_lora_config_t lora_config = {
    .sf = SUBGHZ_LORA_SF7,
    .bw = SUBGHZ_LORA_BW250,
    .cr = SUBGHZ_LORA_CR_48,
    .freq = 865200000,
    .payload_length = 13,
    .preamble_length = 12,
    .header_type = SUBGHZ_PKT_FIXED_LENGTH,
    .crc_enabled = false,
    .invert_iq = false,
    .ldro = SUBGHZ_LORA_LDRO_DISABLED,
    .pa_mode = SUBGHZ_PA_MODE_HP,
    .pa_power = SUBGHZ_PA_PWR_14DBM
  };

  subghz_fsk_config_t fsk_config = {
    .freq = 865200000,
    .freq_dev = 50000,
    .bandwidth = SUBGHZ_FSK_BW373,
    .pulse_shape = SUBGHZ_FSK_GAUSSIAN_NONE,
    .bit_rate = 50000,
    .preamble_length = 8,

    .packet_type = SUBGHZ_PKT_FIXED_LENGTH,
    .sync_word_length = 0,
    .payload_length = 13,
    
    .addr_comp = SUBGHZ_ADDR_COMP_DISABLED,
    .crc = SUBGHZ_PKT_CRC_NONE,
    .whitening = false,

    .pa_mode = SUBGHZ_PA_MODE_HP,
    .pa_power = SUBGHZ_PA_PWR_22DBM
  };
  fsk_config.sync_word[0] = 0xBA;
  fsk_config.sync_word[1] = 0xDC;
  fsk_config.sync_word[2] = 0x0F;
  fsk_config.sync_word[3] = 0xFE;
	

  /* Setup clock & UART */
  clock_setup_bis();
	usart_setup();

  /* Configure our debug SPI */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO4);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO5);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO6);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
	gpio_set_af(GPIOA, GPIO_AF13, GPIO7);

  /* Configure our debug RF_BUSY */
	//gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
	//gpio_set_af(GPIOA, GPIO_AF6, GPIO12);

  /* Initialize SUBGHZ. */
  subghz_init();

	/* red led for buttons */
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_RED_PIN);
  gpio_set(LED_RED_PORT, LED_RED_PIN);

  /* Set PC3, PC4 and PC5 to high (default low-power TX mode). */
	gpio_mode_setup(HF_PA_CTRL1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL1_PIN);
  gpio_set(HF_PA_CTRL1_PORT, HF_PA_CTRL1_PIN);
	gpio_mode_setup(HF_PA_CTRL2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL2_PIN);
  gpio_set(HF_PA_CTRL2_PORT, HF_PA_CTRL2_PIN);
	gpio_mode_setup(HF_PA_CTRL3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL3_PIN);
  gpio_set(HF_PA_CTRL3_PORT, HF_PA_CTRL3_PIN);


  /* LoRa API test. */

  /* Set our callbacks. */
  subghz_set_callbacks(&my_callbacks);

  /* Set our payload. */
  subghz_set_buffer_base_address(0, 0);

  /* Enable LoRa mode. */
  printf("Enable LoRa mode\n");
  //subghz_lora_mode(&lora_config);
  if (subghz_fsk_mode(&fsk_config) == SUBGHZ_ERROR)
    printf("Config failed\n");

  /* Set TX mode (start transmission). */
  printf("Send payload\n");
  //subghz_set_payload((uint8_t *)"Hello world", 11);
  //subghz_set_tx_mode(0);
  //subghz_set_rx_mode(0xFFFFFF);
  if (subghz_send((uint8_t *)"Hello world !", 13, 0) == SUBGHZ_ERROR)
    printf("Send failed\n");

  print_reg_hex("status", subghz_get_status());
  //printf("Packet sent\r\n");
  //printf("Packet sent\r\n");
  //printf("Receiving packet\r\n");
  #if 0
  memset(packet, 0, sizeof(packet));
  res = subghz_receive(packet, &packet_size, 0xFFFFFF);
  printf("got packet\r\n");
  if (res == SUBGHZ_SUCCESS)
  {
  printf("Received packet:\r\n");
  print_str((char *)packet, packet_size);
  }
  else if (res == SUBGHZ_TIMEOUT)
  {
    printf("Reception timed out\r\n");
  }
  else
  {
    printf("An error occured\r\n");
  }
  subghz_receive_async(0xFFFFFF);
  #endif

	while (1) {
    //gpio_toggle(LED_RED_PORT, LED_RED_PIN);
        
		for (int i = 0; i < 400000; i++) { /* Wait a bit. */
			__asm__("NOP");
		}
	}

	return 0;
}

void on_tx_pkt_sent(void)
{
    printf("Packet sent\n");
}

/* handle RF switch config. */
void on_rf_switch_cb(bool tx)
{
    gpio_mode_setup(RF_SW_CTRL1_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL1_PIN);
    gpio_mode_setup(RF_SW_CTRL2_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL2_PIN);
    gpio_set(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN);

    if (tx)
    {
        /* TX mode, low power */
        printf("Enable TX (RF switch)\n");
        gpio_set(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);
    }
    else
    {
        /* RX mode, low power */
        printf("Enable RX (RF switch)\n");
        gpio_clear(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);
    }
}

void on_rx_pkt_recvd(uint8_t offset, uint8_t length)
{
    uint8_t rxbuf[256];

    printf("Packet received !\n");
    if (SUBGHZ_CMD_SUCCESS(subghz_read_buffer(offset, rxbuf, length)))
    {
        print_str((char *)rxbuf, length);
    }
}