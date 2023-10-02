#ifndef __INC_ADAPTER_H
#define __INC_ADAPTER_H

#include <whad.h>
#include "lora/subghz.h"

#define     FIRMWARE_AUTHOR "Damien Cauquil"
#define     FIRMWARE_URL "https://github.com/whad-team/stm32wlxx-firmware"

#define     RF_SW_CTRL1_PIN         GPIO4
#define     RF_SW_CTRL1_GPIO_PORT   GPIOA
#define     RF_SW_CTRL2_PIN         GPIO5
#define     RF_SW_CTRL2_GPIO_PORT   GPIOA

#define     HF_PA_CTRL1_PORT        GPIOC
#define     HF_PA_CTRL1_PIN         GPIO3
#define     HF_PA_CTRL2_PORT        GPIOC
#define     HF_PA_CTRL2_PIN         GPIO4
#define     HF_PA_CTRL3_PORT        GPIOC
#define     HF_PA_CTRL3_PIN         GPIO5

/**
 * Structure declarations.
 **/

/* Adapter state. */
typedef enum {
    STOPPED = 0,
    STARTED
} adapter_state_t;

/* Adapter mode (LoRa/FSK) */
typedef enum {
    LORA_MODE = 0,
    FSK_MODE
} adapter_mode_t;

/* Main adapter structure. */
typedef struct {
    adapter_state_t state;
    adapter_mode_t mode;
    uint16_t sync_word;
    subghz_lora_config_t lora_config;
    subghz_fsk_config_t fsk_config;
} adapter_t;

/**
 * Exported functions. 
 **/

void adapter_init(void);
void dispatch_message(Message *p_msg);

#endif /* __INC_ADAPTER_H */