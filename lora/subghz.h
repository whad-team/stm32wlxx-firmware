#ifndef __INC_SUBGHZ_H
#define __INC_SUBGHZ_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include "../sys.h"

#define SUBGHZ_DEFAULT_TIMEOUT    100U /* SUBGHZ default timeout: 100ms */
#define SUBGHZ_RFBUSY_LOOP_TIME   ((rcc_ahb_frequency*24U)>>20U)

#define SUBGHZ_RADIO_READ_REGISTER   0x1D
#define SUBGHZ_RADIO_WRITE_REGISTER  0x0D

/* Registers */
#define SUBGHZ_RAMPUPH               0x0F0
#define SUBGHZ_RAMPUPL               0x0F1
#define SUBGHZ_RAMPDNH               0x0F2
#define SUBGHZ_RAMPDNL               0x0F3
#define SUBGHZ_FRAMELIMH             0x0F4
#define SUBGHZ_FRAMELIML             0x0F5
#define SUBGHZ_GBSYNCR               0x6AC
#define SUBGHZ_GPKTCTL1AR            0x6B8
#define SUBGHZ_GWHITEINIRL           0x6B9
#define SUBGHZ_GCRCINIRH             0x6BC
#define SUBGHZ_GCRCINIRL             0x6BD
#define SUBGHZ_GCRCPOLRH             0x6BE
#define SUBGHZ_GCRCPOLRL             0x6BF
#define SUBGHZ_GSYNCR0               0x6C7
#define SUBGHZ_GSYNCR1               0x6C6
#define SUBGHZ_GSYNCR2               0x6C5
#define SUBGHZ_GSYNCR3               0x6C4
#define SUBGHZ_GSYNCR4               0x6C3
#define SUBGHZ_GSYNCR5               0x6C2
#define SUBGHZ_GSYNCR6               0x6C1
#define SUBGHZ_GSYNCR7               0x6C0
#define SUBGHZ_LSYNCRH               0x740
#define SUBGHZ_LSYNCRL               0x741
#define SUBGHZ_RNGR3                 0x819
#define SUBGHZ_RNGR2                 0x81A
#define SUBGHZ_RNGR1                 0x81B
#define SUBGHZ_RNGR0                 0x81C
#define SUBGHZ_AGCRSSIC0R            0x89B
#define SUBGHZ_RXGAINCR              0x8AC
#define SUBGHZ_PAOCPR                0x8E7
#define SUBGHZ_HSEINTRIMR            0x911
#define SUBGHZ_HSEOUTTRIMR           0x912
#define SUBGHZ_SMPSC0R               0x916
#define SUBGHZ_PCR                   0x91A
#define SUBGHZ_REGDRVTSTR            0x91F
#define SUBGHZ_SMPSC2R               0x923

/* Commands */
#define SUBGHZ_CALIBRAT_IMAGE        0x98
#define SUBGHZ_CALIBRATE             0x89
#define SUBGHZ_CFG_DIO_IRQ           0x08
#define SUBGHZ_CLR_ERROR             0x07
#define SUBGHZ_CLR_IRQ_STATUS        0x02
#define SUBGHZ_GET_ERROR             0x17
#define SUBGHZ_GET_IRQ_STATUS        0x12
#define SUBGHZ_GET_PACKET_STATUS     0x14
#define SUBGHZ_GET_PACKET_TYPE       0x11
#define SUBGHZ_GET_RSSI_INST         0x15
#define SUBGHZ_GET_RX_BUFFER_STATUS  0x13
#define SUBGHZ_GET_STATS             0x10
#define SUBGHZ_GET_STATUS            0xC0
#define SUBGHZ_READ_BUFFER           0x1E
#define SUBGHZ_READ_REGISTER         0x1D
#define SUBGHZ_RESET_STATS           0x00
#define SUBGHZ_SET_BUFFER_BA         0x8F
#define SUBGHZ_SET_CAD               0xC5
#define SUBGHZ_SET_CAD_PARAMS        0x88
#define SUBGHZ_SET_FS                0xC1
#define SUBGHZ_SET_LORA_SYMB_TO      0xA0
#define SUBGHZ_SET_MOD_PARAMS        0x8B
#define SUBGHZ_SET_PA_CONFIG         0x95
#define SUBGHZ_SET_PACKET_PARAMS     0x8C
#define SUBGHZ_SET_PACKET_TYPE       0x8A
#define SUBGHZ_SET_REGULATOR_MODE    0x96
#define SUBGHZ_SET_RF_FREQ           0x86
#define SUBGHZ_SET_RX                0x82
#define SUBGHZ_SET_RX_DUTY_CYCLE     0x94
#define SUBGHZ_SET_TXRX_FALLBACK     0x93
#define SUBGHZ_SET_SLEEP             0x84
#define SUBGHZ_SET_STANDBY           0x80
#define SUBGHZ_SET_STOP_RX_TIMER     0x9F
#define SUBGHZ_SET_TCXO_MODE         0x97
#define SUBGHZ_SET_TX                0x83
#define SUBGHZ_SET_TX_CONT_WAVE      0xD1
#define SUBGHZ_SET_TX_CONT_PREAMBLE  0xD2
#define SUBGHZ_SET_TX_PARAMS         0x8E
#define SUBGHZ_WRITE_BUFFER          0x0E
#define SUBGHZ_WRITE_REGISTER        0x0D

/* Registers masks */
#define SUBGHZ_GPKTCTL1AR_SYNCDETEN_MSK     (1 << 5)

/* Constants */
#define SUBGHZ_STATUS_MODE_MASK      (7 << 4)
#define SUBGHZ_STATUS_CMD_MASK       (7 << 1)
#define SUBGHZ_STATUS_CMD_SUCCESS    (6 << 1)
#define SUBGHZ_STATUS_MODE(x)        ((x & SUBGHZ_STATUS_MODE_MASK) >> 4)
#define SUBGHZ_STATUS_CMD(x)         ((x & SUBGHZ_STATUS_CMD_MASK) >> 1)
#define SUBGHZ_CMD_SUCCESS(x)        ((SUBGHZ_STATUS_CMD(x) & 0x03) == 2)
#define SUBGHZ_CMD_FAILED(x)         (!(SUBGHZ_CMD_SUCCESS(x)))

#define SUBGHZ_ERROR_PARAMP         (1 << 8)
#define SUBGHZ_ERROR_RFPLL_LOCK     (1 << 6)
#define SUBGHZ_ERROR_HSE32          (1 << 5)
#define SUBGHZ_ERROR_IMAGE          (1 << 4)
#define SUBGHZ_ERROR_RFADC          (1 << 3)
#define SUBGHZ_ERROR_RFPLL          (1 << 2)
#define SUBGHZ_ERROR_RC13M          (1 << 1)
#define SUBGHZ_ERROR_RC64K          (1 << 0)

#define SUBGHZ_CALIB_RC64K          (1 << 0)
#define SUBGHZ_CALIB_RC13M          (1 << 1)
#define SUBGHZ_CALIB_RFPLL          (1 << 2)
#define SUBGHZ_CALIB_RFADC_PULSE    (1 << 3)
#define SUBGHZ_CALIB_RFADCN         (1 << 4)
#define SUBGHZ_CALIB_RFADCP         (1 << 5)
#define SUBGHZ_CALIB_RFADCN         (1 << 4)
#define SUBGHZ_CALIB_IMAGE          (1 << 6)
#define SUBGHZ_CALIB_ALL            (SUBGHZ_CALIB_RC64K | SUBGHZ_CALIB_RC13M | \
                                    SUBGHZ_CALIB_RFPLL | SUBGHZ_CALIB_RFADC_PULSE | \
                                    SUBGHZ_CALIB_RFADCN | SUBGHZ_CALIB_RFADCP | \
                                    SUBGHZ_CALIB_IMAGE)

#define SUBGHZ_DEFAULT_CURMAX_LP    0x18
#define SUBGHZ_DEFAULT_CURMAX_HP    0x38

#define XTAL_FREQ                   ( 32000000UL )
#define SX_FREQ_TO_CHANNEL( channel, freq )                                  \
do                                                                           \
{                                                                            \
  channel = (uint32_t) ((((uint64_t) freq)<<25)/(XTAL_FREQ) );               \
}while( 0 )

#define SUBGHZ_SUCCESS              0
#define SUBGHZ_ERROR                -1
#define SUBGHZ_TIMEOUT              -2

/*** Enums ***/

/* Status */
typedef enum {
  SUBGHZ_STATUS_MODE_RC13M = 0x02,
  SUBGHZ_STATUS_MODE_HSE32,
  SUBGHZ_STATUS_MODE_FS,
  SUBGHZ_STATUS_MODE_RX,
  SUBGHZ_STATUS_MODE_TX
} subghz_status_mode_t;

typedef enum {
  SUBGHZ_STATUS_CMD_DATA_AVAIL = 0x02,
  SUBGHZ_STATUS_CMD_TIMEOUT,
  SUBGHZ_STATUS_CMD_ERROR,
  SUBGHZ_STATUS_CMD_EXEC_FAIL,
  SUBGHZ_STATUS_CMD_COMPLETED
} subghz_status_cmd_t;

/* Alias for subghz_status_t */
typedef uint8_t subghz_result_t;
typedef uint16_t subghz_err_t;

/* TCXO Mode */
typedef enum {
  SUBGHZ_TCXO_TRIM_1V6,
  SUBGHZ_TCXO_TRIM_1V7,
  SUBGHZ_TCXO_TRIM_1V8,
  SUBGHZ_TCXO_TRIM_2V2,
  SUBGHZ_TCXO_TRIM_2V4,
  SUBGHZ_TCXO_TRIM_2V7,
  SUBGHZ_TCXO_TRIM_3V0,
  SUBGHZ_TCXO_TRIM_3V3
} subghz_tcxo_trim_t;

/* Standby mode */
typedef enum {
  SUBGHZ_STDBY_RC13M,
  SUBGHZ_STDBY_HSE32
} subghz_standby_mode_t;

/* Regulator mode */
typedef enum {
  SUBGHZ_REGMODE_LDO,
  SUBGHZ_REGMODE_SMPS
} subghz_regulator_mode_t;

/* RX Timer Stop */
typedef enum {
  SUBGHZ_RXTIMER_STOP_DEFAULT,
  SUBGHZ_RXTIMER_STOP_PREAMBLE
} subghz_rxtimer_stop_t;

/* Packet types */
typedef enum {
  SUBGHZ_PACKET_FSK,
  SUBGHZ_PACKET_LORA,
  SUBGHZ_PACKET_BPSK,
  SUBGHZ_PACKET_MSK
} subghz_packet_type_t;


/* TX Params */
typedef enum {
  SUBGHZ_TXPARAMS_RAMPTIME_10US,
  SUBGHZ_TXPARAMS_RAMPTIME_20US,
  SUBGHZ_TXPARAMS_RAMPTIME_40US,
  SUBGHZ_TXPARAMS_RAMPTIME_80US,
  SUBGHZ_TXPARAMS_RAMPTIME_200US,
  SUBGHZ_TXPARAMS_RAMPTIME_800US,
  SUBGHZ_TXPARAMS_RAMPTIME_1700US,
  SUBGHZ_TXPARAMS_RAMPTIME_3400US
} subghz_txparams_ramptime_t;

/* Fallback mode */
typedef enum {
  SUBGHZ_FALLBACK_STDBY = 0x20,
  SUBGHZ_FALLBACK_STDBY_HSE32 = 0x30,
  SUBGHZ_FALLBACK_FS = 0x40
} subghz_fallback_mode_t;

/* CAD config */
typedef enum {
  SUBGHZ_CAD_SYMB_1,
  SUBGHZ_CAD_SYMB_2,
  SUBGHZ_CAD_SYMB_4,
  SUBGHZ_CAD_SYMB_8,
  SUBGHZ_CAD_SYMB_16,
} subghz_cad_symbols_t;

typedef enum {
  SUBGHZ_CAD_STDBY_ALWAYS,
  SUBGHZ_CAD_STDBY_NOSYMB
} subghz_cad_exit_mode_t;

/* FSK modulation parameters */
typedef enum {
  SUBGHZ_FSK_GAUSSIAN_NONE,
  SUBGHZ_FSK_GAUSSIAN_BT_03 = 0x08,
  SUBGHZ_FSK_GAUSSIAN_BT_05,
  SUBGHZ_FSK_GAUSSIAN_BT_07,
  SUBGHZ_FSK_GAUSSIAN_BT_10
} subghz_fsk_gaussian_t;

typedef enum {
  SUBGHZ_FSK_BW467 = 0x09,
  SUBGHZ_FSK_BW234, /* 0x0A */
  SUBGHZ_FSK_BW117, /* 0x0B */
  SUBGHZ_FSK_BW58,  /* 0x0C */
  SUBGHZ_FSK_BW29,  /* 0x0D */
  SUBGHZ_FSK_BW14,  /* 0x0E */
  SUBGHZ_FSK_BW7,   /* 0x0F */

  SUBGHZ_FSK_BW373 = 0x11,
  SUBGHZ_FSK_BW187, /* 0x12 */
  SUBGHZ_FSK_BW93,  /* 0x13 */
  SUBGHZ_FSK_BW46,  /* 0x14 */
  SUBGHZ_FSK_BW23,  /* 0x15 */
  SUBGHZ_FSK_BW11,  /* 0x16 */
  SUBGHZ_FSK_BW5,   /* 0x17 */

  SUBGHZ_FSK_BW312 = 0x19,
  SUBGHZ_FSK_BW156, /* 0x1A */
  SUBGHZ_FSK_BW78,  /* 0x1B */
  SUBGHZ_FSK_BW39,  /* 0x1C */
  SUBGHZ_FSK_BW19,  /* 0x1D */
  SUBGHZ_FSK_BW9,   /* 0x1E */
  SUBGHZ_FSK_BW4,   /* 0x1F */
} subghz_fsk_bandwidth_t;

/* BPSK gaussian filter. */
typedef enum {
  SUBGHZ_BPSK_GAUSSIAN_BT_05 = 0x16
} subghz_bpsk_gaussian_t;

/* LoRa modulation parameters */
typedef enum {
  SUBGHZ_LORA_SF5 = 5,
  SUBGHZ_LORA_SF6,
  SUBGHZ_LORA_SF7,
  SUBGHZ_LORA_SF8,
  SUBGHZ_LORA_SF9,
  SUBGHZ_LORA_SF10,
  SUBGHZ_LORA_SF11,
  SUBGHZ_LORA_SF12,
} subghz_lora_sf_t;

typedef enum {
  SUBGHZ_LORA_CR_44,
  SUBGHZ_LORA_CR_45,
  SUBGHZ_LORA_CR_46,
  SUBGHZ_LORA_CR_47,
  SUBGHZ_LORA_CR_48,
} subghz_lora_cr_t;

typedef enum {
  SUBGHZ_LORA_BW7,
  SUBGHZ_LORA_BW15,
  SUBGHZ_LORA_BW31,
  SUBGHZ_LORA_BW62,
  SUBGHZ_LORA_BW125,
  SUBGHZ_LORA_BW250,
  SUBGHZ_LORA_BW500,
  SUBGHZ_LORA_BW10 = 0x08,
  SUBGHZ_LORA_BW20,
  SUBGHZ_LORA_BW41
} subghz_lora_bandwidth_t;

typedef enum {
  SUBGHZ_LORA_LDRO_DISABLED,
  SUBGHZ_LORA_LDRO_ENABLED
} subghz_lora_ldro_t;

/* Generic packet parameters */
typedef enum {
  SUBGHZ_DET_PREAMBLE_DISABLED,
  SUBGHZ_DET_PREAMBLE_8BIT = 0x04,
  SUBGHZ_DET_PREAMBLE_16BIT,
  SUBGHZ_DET_PREAMBLE_24BIT,
  SUBGHZ_DET_PREAMBLE_32BIT
} subghz_det_length_t;

typedef enum {
  SUBGHZ_ADDR_COMP_DISABLED,
  SUBGHZ_ADDR_COMP_NODE,
  SUBGHZ_ADDR_COMP_NODE_BROADCAST
} subghz_addr_comp_t;

typedef enum {
  SUBGHZ_PKT_FIXED_LENGTH,
  SUBGHZ_PKT_VAR_LENGTH
} subghz_packet_length_t;

typedef enum {
  SUBGHZ_PKT_CRC_1,
  SUBGHZ_PKT_CRC_NONE,
  SUBGHZ_PKT_CRC_2,
  SUBGHZ_PKT_INV_CRC_1 = 0x04,
  SUBGHZ_PKT_INV_CRC_2 = 0x06,
} subghz_packet_crc_t;

/* LoRa packet parameters */
typedef enum {
  SUBGHZ_PKT_LORA_VAR_LENGTH = 0,
  SUBGHZ_PKT_LORA_FIXED_LENGTH = 1
} subghz_lora_packet_hdr_t;

/* Rx Buffer status. */
typedef struct {
  uint8_t payload_length;
  uint8_t buffer_offset;
} subghz_rxbuf_status_t;

/* FSK RX buffer status. */
typedef struct {
  union {
    struct {
      bool packet_sent: 1;
      bool packet_recvd: 1;
      bool abort_err: 1;
      bool length_err: 1;
      bool crc_err: 1;
      bool address_err: 1;
      bool sync_err: 1;
      bool preamble_err: 1;
    } bits;
    uint8_t rx_status;
  };
  uint8_t rssi_sync;
  uint8_t rssi_avg;
} subghz_fsk_packet_status_t;

/* LoRa Packet status. */
typedef struct {
  uint8_t rssi;
  uint8_t snr;
  uint8_t signal_rssi;
} subghz_lora_packet_status_t;

/* FSK Stats. */
typedef struct {
  uint16_t nb_packets_recvd;
  uint16_t nb_packets_crcerr;
  uint16_t nb_packets_lenerr;
} subghz_fsk_stats_t;

/* LoRa Stats. */
typedef struct {
  uint16_t nb_packets_recvd;
  uint16_t nb_packets_crcerr;
  uint16_t nb_packets_headerr;
} subghz_lora_stats_t;

/* IRQ status. */
typedef struct {
  union {
    struct {
      bool tx_done: 1;
      bool rx_done: 1;
      bool preamble: 1;
      bool sync: 1;
      bool header_valid: 1;
      bool header_err: 1;
      bool crc_err: 1;
      bool cad_done: 1;

      bool cad_detected: 1;
      bool timeout: 1;
      uint8_t _res: 6;
    } bits;
    uint16_t word;
  };
} subghz_irq_status_t;

typedef enum
{
    IRQ_RADIO_NONE                          = 0x0000,
    IRQ_TX_DONE                             = 0x0001,
    IRQ_RX_DONE                             = 0x0002,
    IRQ_PREAMBLE_DETECTED                   = 0x0004,
    IRQ_SYNCWORD_VALID                      = 0x0008,
    IRQ_HEADER_VALID                        = 0x0010,
    IRQ_HEADER_ERROR                        = 0x0020,
    IRQ_CRC_ERROR                           = 0x0040,
    IRQ_CAD_CLEAR                           = 0x0080,
    IRQ_CAD_DETECTED                        = 0x0100,
    IRQ_RX_TX_TIMEOUT                       = 0x0200,
    IRQ_RADIO_ALL                           = 0xFFFF,
} RadioIrqMasks_t;


/*******************************************
 * SubGHZ driver configuration.
 ******************************************/

typedef enum {
    SUBGHZ_PA_PWR_10DBM,
    SUBGHZ_PA_PWR_14DBM,
    SUBGHZ_PA_PWR_15DBM,
    SUBGHZ_PA_PWR_17DBM,
    SUBGHZ_PA_PWR_20DBM,
    SUBGHZ_PA_PWR_22DBM,
    SUBGHZ_PA_PWR_COUNT
} subghz_pa_pwr_t;

typedef enum {
    SUBGHZ_PA_MODE_LP,
    SUBGHZ_PA_MODE_HP,
    SUBGHZ_PA_MODE_COUNT
} subghz_pa_mode_t;

typedef enum {
  SUBGHZ_STATE_STARTUP,
  SUBGHZ_STATE_SLEEP,
  SUBGHZ_STATE_CALIB,
  SUBGHZ_STATE_STDBY,
  SUBGHZ_STATE_FS,
  SUBGHZ_STATE_TX,
  SUBGHZ_STATE_RX
} subghz_state_t;

typedef enum {
  SUBGHZ_MODE_UNDEF,
  SUBGHZ_MODE_LORA,
  SUBGHZ_MODE_FSK,
  SUBGHZ_MODE_BPSK,
  SUBGHZ_MODE_MSK
} subghz_mode_t;

typedef struct {
  /* Frequency. */
  uint32_t freq;

  /* Modulation parameters. */
  subghz_lora_bandwidth_t bw;
  subghz_lora_cr_t cr;
  subghz_lora_sf_t sf;
  subghz_lora_ldro_t ldro;

  /* Packet configuration. */
  subghz_lora_packet_hdr_t header_type;
  uint16_t preamble_length;
  uint8_t payload_length;
  bool crc_enabled;
  bool invert_iq;

  /* PA configuration. */
  subghz_pa_mode_t pa_mode;
  subghz_pa_pwr_t pa_power;
} subghz_lora_config_t;


typedef struct {
  /* Frequency. */
  uint32_t freq;

  /* Modulation parameters. */
  uint32_t bit_rate;
  uint32_t freq_dev;
  subghz_fsk_gaussian_t pulse_shape;
  subghz_fsk_bandwidth_t bandwidth;

  /* Packet configuration. */
  uint8_t sync_word_length;
  uint8_t sync_word[8];
  uint16_t preamble_length;
  subghz_det_length_t preamble_detect;
  subghz_addr_comp_t addr_comp;
  subghz_packet_length_t packet_type;
  uint8_t payload_length;
  subghz_packet_crc_t crc;
  bool whitening;

  /* PA configuration. */
  subghz_pa_mode_t pa_mode;
  subghz_pa_pwr_t pa_power;
} subghz_fsk_config_t;

/* Callback types. */
typedef void (*F_on_packet_sent)(void);
typedef void (*F_on_packet_recvd)(uint8_t offset, uint8_t length, uint32_t timestamp);
typedef void (*F_on_timeout)(void);
typedef void (*F_on_preamble)(void);
typedef void (*F_on_rf_switch)(bool tx);

/* Callback structure. */
typedef struct {
  F_on_packet_sent pfn_on_packet_sent;
  F_on_packet_recvd pfn_on_packet_recvd;
  F_on_timeout pfn_on_timeout;
  F_on_timeout pfn_on_preamble;
  F_on_rf_switch pfn_on_rf_switch;
} subghz_callbacks_t;

typedef struct {
  /* Selected mode. */
  subghz_mode_t current_mode;
  subghz_state_t current_state;

  /* Frequency setting. */
  uint32_t current_freq;
  uint32_t current_channel;

  /* Tx settings. */
  subghz_txparams_ramptime_t ramp_time;

  /* LoRa modulation parameters. */
  subghz_lora_config_t lora_params;

  /* FSK modulation parameters. */
  subghz_fsk_config_t fsk_params;

  /* Callbacks*/
  subghz_callbacks_t callbacks;
} subghz_t;

/* SUBGHZSPI HAL */
int subghz_check_device_ready(void);
int subghz_wait_on_busy(void);
subghz_result_t subghz_read_reg(uint16_t address, uint8_t *p_reg);
subghz_result_t subghz_write_reg(uint16_t address, uint8_t value);
subghz_result_t subghz_read_regs(uint16_t address, uint8_t *p_buffer, uint16_t size);
subghz_result_t subghz_write_regs(uint16_t address, uint8_t *p_buffer, uint16_t size);
subghz_result_t subghz_write_command(uint8_t command, uint8_t *p_parameters, int params_size);
subghz_result_t subghz_write_buffer(uint8_t offset, uint8_t *p_data, int length);
subghz_result_t subghz_read_buffer(uint8_t offset, uint8_t *p_data, int length);

/* SUBGHZ Commands API */
uint32_t subghz_get_errcode(void);
subghz_result_t subghz_get_status(void);
subghz_result_t subghz_get_error(subghz_err_t *error);
subghz_result_t subghz_set_tcxo_mode(subghz_tcxo_trim_t trim, uint32_t timeout);
subghz_result_t subghz_calibrate(uint8_t calib_cfg);
subghz_result_t subghz_set_standby_mode(subghz_standby_mode_t mode);
subghz_result_t subghz_set_regulator_mode(subghz_regulator_mode_t mode);
subghz_result_t subghz_set_fs_mode(void);
subghz_result_t subghz_set_tx_mode(uint32_t timeout);
subghz_result_t subghz_set_rx_mode(uint32_t timeout);
subghz_result_t subghz_set_stop_rxtimer_on_preamble(subghz_rxtimer_stop_t config);
subghz_result_t subghz_set_duty_cycle(uint32_t rx_period, uint32_t sleep_period);
subghz_result_t subghz_set_cad(void);
subghz_result_t subghz_set_tx_continuous_wave(void);
subghz_result_t subghz_set_tx_continuous_preamble(void);
subghz_result_t subghz_set_packet_type(subghz_packet_type_t packet_type);
subghz_result_t subghz_get_packet_type(subghz_packet_type_t *packet_type);
subghz_result_t subghz_set_rf_freq(uint32_t freq);
subghz_result_t subghz_set_tx_params(int8_t power, subghz_txparams_ramptime_t ramp_time);
subghz_result_t subghz_set_pa_config(uint8_t duty_cycle, uint8_t hp_max, uint8_t pa_sel);
subghz_result_t subghz_set_tx_rx_fallback_mode(subghz_fallback_mode_t mode);
subghz_result_t subghz_set_cad_params(subghz_cad_symbols_t nb_symbols, uint8_t det_peak,
                                      uint8_t det_min, subghz_cad_exit_mode_t exit_mode,
                                      uint32_t timeout);
subghz_result_t subghz_set_buffer_base_address(uint8_t tx_base_addr, uint8_t rx_base_addr);
subghz_result_t subghz_set_fsk_modulation_params(uint32_t bitrate, subghz_fsk_gaussian_t gaussian,
                                                 subghz_fsk_bandwidth_t bandwidth, uint32_t deviation);
subghz_result_t subghz_set_lora_modulation_params(subghz_lora_sf_t sf, subghz_lora_bandwidth_t bandwidth,
                                                  subghz_lora_cr_t cr, subghz_lora_ldro_t ldro);
subghz_result_t subghz_set_bpsk_modulation_params(uint32_t bitrate, subghz_bpsk_gaussian_t gaussian);
subghz_result_t subghz_set_packet_params(uint16_t preamble_length, subghz_det_length_t det_length,
                                         uint8_t syncword_length, subghz_addr_comp_t addr_comp,
                                         subghz_packet_length_t packet_length, uint8_t payload_length,
                                         subghz_packet_crc_t crc_type, bool whitening);
subghz_result_t subghz_set_lora_packet_params(uint16_t preamble_length, subghz_lora_packet_hdr_t header_type,
                                              uint8_t payload_length, bool crc_enabled, bool invert_iq);
subghz_result_t subghz_set_bpsk_packet_params(uint8_t payload_length);
subghz_result_t subghz_set_lora_symb_timeout(uint8_t symb_num);
subghz_result_t subghz_get_fsk_packet_status(subghz_fsk_packet_status_t *p_packet_status);
subghz_result_t subghz_get_lora_packet_status(subghz_lora_packet_status_t *p_packet_status);
subghz_result_t subghz_get_rssi_inst(uint8_t *p_rssi_inst);
subghz_result_t subghz_get_fsk_stats(subghz_fsk_stats_t *p_fsk_stats);
subghz_result_t subghz_get_lora_stats(subghz_lora_stats_t *p_lora_stats);
subghz_result_t subghz_reset_stats(void);
subghz_result_t subghz_config_dio_irq(uint16_t irq_mask, uint16_t irq1_mask,
                                      uint16_t irq2_mask, uint16_t irq3_mask);
subghz_result_t subghz_clear_irq(subghz_irq_status_t *p_irq_status);
subghz_result_t subghz_set_payload(uint8_t *payload, uint8_t size);

/* SubGHZ API. */
int subghz_init(void);
int subghz_lora_mode(subghz_lora_config_t *p_lora_config);
int subghz_fsk_mode(subghz_fsk_config_t *p_fsk_config);
int subghz_config_pa(subghz_pa_mode_t mode, subghz_pa_pwr_t power);
void subghz_set_callbacks(const subghz_callbacks_t *callbacks);
int subghz_send(uint8_t *p_frame, int length, uint32_t timeout);
int subghz_send_async(uint8_t *p_frame, int length, uint32_t timeout);
int subghz_receive(uint8_t *p_frame, uint8_t *p_length, uint32_t timeout);
int subghz_receive_async(uint32_t timeout);

/* Prepare send. */
int subghz_prepare_send(uint8_t *p_frame, int length);
int subghz_tx(uint32_t timeout);

/* Syncword management (FSK / LoRa). */
int subghz_set_syncword(uint8_t *p_syncword, int length);
int subghz_set_syncword16(uint16_t syncword);
int subghz_set_syncword32(uint32_t syncword);
int subghz_set_syncword64(uint32_t syncword_l, uint32_t syncword_h);
int subghz_enable_syncword(bool enable);

/* CRC management. */
int subghz_set_crc_init(uint16_t crc_init);
int subghz_set_crc_poly(uint16_t crc_poly);

/* Whitening management. */
int subghz_set_whitening_init(uint8_t white_init);

#endif /* __INC_SUBGHZ_H */