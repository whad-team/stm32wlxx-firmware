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

#ifdef NUCLEO_WL55
  #define     HF_PA_CTRL1_PORT        GPIOC
  #define     HF_PA_CTRL1_PIN         GPIO3
  #define     HF_PA_CTRL2_PORT        GPIOC
  #define     HF_PA_CTRL2_PIN         GPIO4
  #define     HF_PA_CTRL3_PORT        GPIOC
  #define     HF_PA_CTRL3_PIN         GPIO5
#endif

#ifdef LORAE5MINI
  #define     HF_PA_CTRL1_PORT        GPIOA
  #define     HF_PA_CTRL1_PIN         GPIO4
  #define     HF_PA_CTRL2_PORT        GPIOA
  #define     HF_PA_CTRL2_PIN         GPIO5
#endif


#define     LORA_BW125              125000
#define     LORA_BW250              250000
#define     LORA_BW500              500000

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

    /* Prepared packets. */
    bool prepared_packet_rdy;
    uint8_t prepared_pkt[256];
    uint8_t pp_length;
    uint32_t pp_timestamp;
} adapter_t;

/**
 * Exported functions. 
 **/

void adapter_init(void);
void dispatch_message(Message *p_msg);
void adapter_send_rdy(void);

#endif /* __INC_ADAPTER_H */