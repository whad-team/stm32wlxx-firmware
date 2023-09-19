#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>

#ifdef STM32F103C8 // bluepill

const struct rcc_clock_scale *rccp = &rcc_hse_configs[ RCC_CLOCK_HSE8_72MHZ];

#else // blackpill

const struct rcc_clock_scale myrcc = {
  // max 
		.pllm = 25,
		.plln = 400,
		.pllp = 4,
		.pllq = 15, // no usb but sdio ok
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 100000000,
		.apb1_frequency = 50000000,
		.apb2_frequency = 100000000,
};
const struct rcc_clock_scale *rccp = &myrcc;
const struct rcc_clock_scale turbodontdothis = {
  // it works it's fun but don't
  // it can be usefull for when you *temporarily* need
  // a little extra oomph but it may crash some stuff
  // also this is not consistant in between chip, one could go faster and not the other
  // my "best of the bunch" ramps up to 138 MHz, my worst crashes if i try any overclocking
		.pllm = 12,
		.plln = 265,
		.pllp = 4,
		.pllq = 15, // no usb but sdio ok
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN |
				FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 138020000,
		.apb1_frequency = 69010000,
		.apb2_frequency = 138020000,
};
#endif
