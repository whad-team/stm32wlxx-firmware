#include "adapter.h"
#include "schedpkt.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

//#define g_timestamp TIM2_CNT

/**
 * Adapter WHAD capabilities and domains
 */
#define CMD(X) (1 << (X))

static whad_domain_desc_t CAPABILITIES[] = {
    {
    DOMAIN_PHY,
    (whad_capability_t)(CAP_SNIFF | CAP_INJECT | CAP_HIJACK | CAP_SIMULATE_ROLE | CAP_NO_RAW_DATA),
    (
        CMD(phy_PhyCommand_GetSupportedFrequencies) |
        CMD(phy_PhyCommand_SetLoRaModulation) |
        CMD(phy_PhyCommand_SetFSKModulation) |
        CMD(phy_PhyCommand_SetFrequency) |
        CMD(phy_PhyCommand_SetSyncWord) |
        CMD(phy_PhyCommand_SetEndianness) |
        CMD(phy_PhyCommand_Sniff) |
        CMD(phy_PhyCommand_SetPacketSize) |
        CMD(phy_PhyCommand_SetDataRate) |
        CMD(phy_PhyCommand_Send) |
        CMD(phy_PhyCommand_Start) |
        CMD(phy_PhyCommand_Stop) |
        CMD(phy_PhyCommand_ScheduleSend)
        )
    },
    {DOMAIN_NONE, CAP_NONE, 0x00000000}
};

static whad_phy_frequency_range_t g_phy_supported_freq_ranges[] = {
    {150000000, 960000000},
    {0, 0}
};
const int g_phy_supported_ranges_nb = 2;

/**
 * RF-related callbacks.
 **/
static void adapter_on_rf_switch_cb(bool tx);
static void adapter_on_pkt_received(uint8_t offset, uint8_t length, uint32_t ts_sec, uint32_t ts_usec);
static void adapter_on_pkt_sent(void);
static void adapter_on_preamble(void);

static subghz_callbacks_t my_callbacks = {
    .pfn_on_packet_recvd = adapter_on_pkt_received,
    .pfn_on_packet_sent = adapter_on_pkt_sent,
    .pfn_on_rf_switch = adapter_on_rf_switch_cb,
    .pfn_on_timeout = adapter_on_pkt_sent,
    .pfn_on_preamble = adapter_on_preamble
};

/* Main adapter structure. */
static adapter_t g_adapter;
static uint32_t g_pkt_timestamp = 0;
/*
typedef struct {
  uint32_t timestamp;
  uint8_t packet[PACKET_MAX_SIZE];
  int length;
} planned_packet_t;
*/
static sched_packet_t g_sched_pkt;
static volatile bool g_sched_pkt_rdy = false;
static volatile bool g_sched_pkt_timer_set = false;

/**************************************
 * Subghz callbacks
 **************************************/

/**
 * @brief   Handle RF switching (TX/RX)
 * 
 * @param   tx  True if TX should be enabled, False if RX should be enabled.
 */

static void adapter_on_rf_switch_cb(bool tx)
{
    #ifdef NUCLEO_WL55
    gpio_mode_setup(RF_SW_CTRL1_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL1_PIN);
    gpio_mode_setup(RF_SW_CTRL2_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL2_PIN);
    gpio_set(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN);

    if (tx)
    {
        /* TX mode, low power */
        gpio_set(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);
    }
    else
    {
        /* RX mode, low power */
        gpio_clear(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);
    }
    #endif

    #ifdef LORAE5MINI
    gpio_mode_setup(RF_SW_CTRL1_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL1_PIN);
    gpio_mode_setup(RF_SW_CTRL2_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RF_SW_CTRL2_PIN);

    if (tx)
    {
        /* TX mode, low power */
        gpio_set(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN);
        gpio_clear(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);
    }
    else
    {
        /* RX mode, low power */
        gpio_clear(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN);
        gpio_set(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);
    }
    #endif
}

/**
 * @brief   Handle packet reception.
 * 
 * @param   offset  Offset in the reception buffer
 * @param   length  Packet length in bytes
 */

static void adapter_on_pkt_received(uint8_t offset, uint8_t length, uint32_t ts_sec, uint32_t ts_usec)
{
    Message msg;
    uint8_t rxbuf[256];
    int8_t rssi = 0;
    int8_t rssi_inst = 0;
    uint32_t freq;

    /* Read instantaneous RSSI. */
    if (SUBGHZ_CMD_SUCCESS(subghz_get_rssi_inst((uint8_t *)&rssi)))
    {
        rssi_inst = -(rssi/2);
    }
    else
    {
        rssi_inst = 0;
    }


    /* Read RX buffer. */
    if (SUBGHZ_CMD_SUCCESS(subghz_read_buffer(offset, rxbuf, length)))
    {
        /* Stop here if we are not enabled. */
        if (g_adapter.state == STOPPED)
            return;        

        if (g_adapter.mode == LORA_MODE)
        {
            freq = g_adapter.lora_config.freq;
        }
        else
        {
            freq = g_adapter.fsk_config.freq;
        }

        /* Report packet. */
        whad_phy_packet_received(
            &msg,
            freq,
            rssi_inst,
            ts_sec,
            ts_usec,
            rxbuf,
            length,
            (g_adapter.mode == LORA_MODE)?(uint8_t *)&g_adapter.sync_word:(uint8_t *)g_adapter.fsk_config.sync_word,
            (g_adapter.mode == LORA_MODE)?2:g_adapter.fsk_config.sync_word_length,
            (g_adapter.mode == LORA_MODE)?0:g_adapter.fsk_config.freq_dev,
            (g_adapter.mode == LORA_MODE)?0:g_adapter.fsk_config.bit_rate,
            phy_Endianness_LITTLE,
            (g_adapter.mode == LORA_MODE)?phy_Modulation_LORA:phy_Modulation_FSK
        );
        whad_send_message(&msg);
    }
}


/**
 * @brief   Handle packet sent.
 */

static void adapter_on_pkt_sent(void)
{
    Message cmd_result;

    /* Switch back to async rx mode. */
    if (subghz_receive_async(0xFFFFFF) == SUBGHZ_SUCCESS)
    {
        /* Success. */
        whad_generic_cmd_result(&cmd_result, WHAD_RESULT_SUCCESS);
        whad_send_message(&cmd_result);
    }
    else
    {
        /* Success. */
        whad_generic_cmd_result(&cmd_result, WHAD_RESULT_ERROR);
        whad_send_message(&cmd_result);
    }
}

static void adapter_on_preamble(void)
{
    g_pkt_timestamp = sys_get_timestamp_sec();
    whad_verbose("preamb");
}


/**************************************
 * Adapter getters/setters
 **************************************/

/**
 * @brief Set LoRa spreading factor
 * 
 * @param   spreading_factor    Spreading factor to use (SF7 -> SF12)
 */

void adapter_set_spreading_factor(phy_LoRaSpreadingFactor spreading_factor)
{
    switch (spreading_factor)
    {
        case phy_LoRaSpreadingFactor_SF8:
            g_adapter.lora_config.sf = SUBGHZ_LORA_SF8;
            break;

        case phy_LoRaSpreadingFactor_SF9:
            g_adapter.lora_config.sf = SUBGHZ_LORA_SF9;
            break;

        case phy_LoRaSpreadingFactor_SF10:
            g_adapter.lora_config.sf = SUBGHZ_LORA_SF10;
            break;

        case phy_LoRaSpreadingFactor_SF11:
            g_adapter.lora_config.sf = SUBGHZ_LORA_SF11;
            
            /* Enable LDRO if required. */
            if (g_adapter.lora_config.bw == SUBGHZ_LORA_BW125)
            {
                g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_ENABLED;
            }
            else
            {
                g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_DISABLED;
            }
            break;

        case phy_LoRaSpreadingFactor_SF12:
            g_adapter.lora_config.sf = SUBGHZ_LORA_SF12;

            /* Enable LDRO if required. */
            if ((g_adapter.lora_config.bw == SUBGHZ_LORA_BW125) || (g_adapter.lora_config.bw == SUBGHZ_LORA_BW250))
            {
                g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_ENABLED;
            }
            else
            {
                g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_DISABLED;
            }
            break;

        default:
        case phy_LoRaSpreadingFactor_SF7:
            g_adapter.lora_config.sf = SUBGHZ_LORA_SF7;
            break;
    }
}


/**
 * @brief Set LoRa coding rate
 * 
 * @param   coding_rate     Coding rate to use (4/5, 4/6, 4/7 or 4/8)
 */

void adapter_set_coding_rate(phy_LoRaCodingRate coding_rate)
{
    switch (coding_rate)
    {
        case phy_LoRaCodingRate_CR45:
            g_adapter.lora_config.cr = SUBGHZ_LORA_CR_45;
            break;

        case phy_LoRaCodingRate_CR46:
            g_adapter.lora_config.cr = SUBGHZ_LORA_CR_46;
            break;

        case phy_LoRaCodingRate_CR47:
            g_adapter.lora_config.cr = SUBGHZ_LORA_CR_47;
            break;

        case phy_LoRaCodingRate_CR48:
            g_adapter.lora_config.cr = SUBGHZ_LORA_CR_48;
            break;
    }
}

/**
 * @brief Set LoRa modulation bandwidth
 * 
 * @param   bandwidth     Bandwidth to use (125 kHz, 250 kHz or 500 kHz)
 */

void adapter_set_bandwidth(uint32_t bandwidth)
{
    /* TODO: handle FSK bandwidth as well ! */
    if (g_adapter.mode == LORA_MODE)
    {
        switch (bandwidth)
        {
            default:
            case LORA_BW125:
                g_adapter.lora_config.bw = SUBGHZ_LORA_BW125;

                /* Enable LDRO if required. */
                if ((g_adapter.lora_config.sf == SUBGHZ_LORA_SF11) || (g_adapter.lora_config.sf == SUBGHZ_LORA_SF12))
                {
                    g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_ENABLED;
                }
                else
                {
                    g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_DISABLED;
                }
                break;

            case LORA_BW250:
                g_adapter.lora_config.bw = SUBGHZ_LORA_BW250;

                /* Enable LDRO if required. */
                if (g_adapter.lora_config.sf == SUBGHZ_LORA_SF12)
                {
                    g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_ENABLED;
                }
                else
                {
                    g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_DISABLED;
                }            
                break;

            case LORA_BW500:
                g_adapter.lora_config.bw = SUBGHZ_LORA_BW500;
                break;
        }
    }
    else
    {
        /* For FSK bandwidth, we need to find the most suitable setting based
           on the requested bandwidth. Based on documentation, we defined the
           following bandwith range associated with the corresponding bandwidth
           setting:

           - below 4.5 kHz -> 4.8 kHz
           - 4.5 kHz to 5.3 kHz -> 4.8 kHz
           - 5.3 kHz to 6.6 kHz -> 5.8 kHz
           - 6.6 kHz to 8.5 kHz -> 7.3 kHz
           - 8.5 kHz to 10.7 kHz -> 9.7 kHz
           - 10.7 kHz to 13.1 kHz -> 11.7 kHz
           - 13.1 kHz to 17.0 kHz -> 14.6 kHz
           - 17.0 kHz to 21.4 kHz -> 19.5 kHz
           - 21.4 kHz to 26.3 kHz -> 23.4 kHz
           - 26.3 kHz to 34.1 kHz -> 29.3 kHz
           - 34.1 kHz to 42.9 kHz -> 39.0 kHz
           - 42.9 kHz to 52.7 kHz -> 46.9 kHz
           - 52.7 kHz to 68.4 kHz -> 58.6 kHz
           - 68.4 kHz to 86.0 kHz -> 78.2 kHz 
           - 86.0 kHz to 105.5kHz -> 93.8 kHz
           - 105.0 kHz to 136.7 kHz -> 117.3 kHz
           - 136.7 kHz to 171.7 kHz -> 156.2 kHz
           - 171.7 kHz to 210.7 kHz -> 187.2 kHz
           - 210.7 kHz to 273.1 kHz -> 234.3 kHz
           - 273.1 kHz to 342.8 kHz -> 312.0 kHz
           - 342.8 kHz to 420.3 kHz -> 373.6 kHz
           - 420.3 kHz and beyond -> 467.0 kHz
        */
        if (bandwidth < 5300)
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW4;
        }
        else if ((bandwidth >= 5300) && (bandwidth < 6600))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW5;
        }
        else if ((bandwidth >= 6600) && (bandwidth < 8500))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW7;
        }
        else if ((bandwidth >= 8500) && (bandwidth < 10700))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW9;
        }
        else if ((bandwidth >= 10700) && (bandwidth < 13100))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW11;
        }
        else if ((bandwidth >= 13100) && (bandwidth < 17000))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW14;
        }
        else if ((bandwidth >= 17000) && (bandwidth < 21400))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW19;
        }
        else if ((bandwidth >= 21400) && (bandwidth < 26300))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW23;
        }
        else if ((bandwidth >= 26300) && (bandwidth < 34100))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW29;
        }
        else if ((bandwidth >= 34100) && (bandwidth < 42900))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW39;
        }
        else if ((bandwidth >= 42900) && (bandwidth < 52700))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW46;
        }
        else if ((bandwidth >= 52700) && (bandwidth < 68400))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW58;
        }
        else if ((bandwidth >= 68400) && (bandwidth < 86000))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW78;
        }
        else if ((bandwidth >= 86000) && (bandwidth < 105500))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW93;
        }
        else if ((bandwidth >= 105500) && (bandwidth < 136700))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW117;
        }
        else if ((bandwidth >= 136700) && (bandwidth < 171700))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW156;
        }
        else if ((bandwidth >= 171700) && (bandwidth < 210700))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW187;
        }
        else if ((bandwidth >= 210700) && (bandwidth < 273100))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW234;
        }
        else if ((bandwidth >= 273100) && (bandwidth < 342800))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW312;
        }
        else if ((bandwidth >= 342800) && (bandwidth < 420300))
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW373;
        }
        else
        {
            g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW467;
        }
    }
}


/**
 * @brief Set FSK deviation
 * 
 * @param   deviation     Deviation to use.
 */

void adapter_set_deviation(uint32_t deviation)
{
    if (g_adapter.mode == FSK_MODE)
    {
        g_adapter.fsk_config.freq_dev = deviation;
    }
}


/**
 * @brief Set LoRa synchronization word
 * 
 * @param   u16_sync_word     16-bit synchronization word (specific to LoRa)
 */

void adapter_set_syncword(uint16_t u16_sync_word)
{
    g_adapter.sync_word = u16_sync_word;
}


/**
 * @brief Set adapter frequency
 * 
 * @param   freq    Frequency in Hz
 */

void adapter_set_freq(uint32_t freq)
{
    if (g_adapter.mode == LORA_MODE)
    {
        g_adapter.lora_config.freq = freq;
    }
    else
    {
        g_adapter.fsk_config.freq = freq;
    }
}


/**
 * @brief Enable CRC (LoRa and FSK)
 * 
 * @param   enabled    True to enable CRC, False to disable
 */

void adapter_enable_crc(bool enabled)
{
    if (g_adapter.mode == LORA_MODE)
    {
        g_adapter.lora_config.crc_enabled = enabled;
    }
    else
    {
        g_adapter.fsk_config.crc = enabled;
    }
}


/**
 * @brief   Set preamble length (FSK and LoRa)
 * 
 * @param   preamble_length    Preamble length in symbols
 */

void adapter_set_preamble_length(uint16_t preamble_length)
{
    if (g_adapter.mode == LORA_MODE)
    {
        g_adapter.lora_config.preamble_length = (preamble_length & 0xFFFF);
    }
    else
    {
        g_adapter.fsk_config.preamble_length = (preamble_length & 0xFFFF);
    }
}


/**
 * @brief   Enable LoRa explicit mode (variable-length packets)
 * 
 * @param   enabled     Enabled if True, disabled otherwise.
 */

void adapter_enable_explicit_mode(bool enabled)
{
    if (enabled)
    {
        g_adapter.lora_config.header_type = SUBGHZ_PKT_LORA_VAR_LENGTH;
    }
    else
    {
        g_adapter.lora_config.header_type = SUBGHZ_PKT_LORA_FIXED_LENGTH;
    }
}


void adapter_enable_invert_iq(bool enabled)
{
        g_adapter.lora_config.invert_iq = enabled;
}


void adapter_set_payload_length(uint32_t payload_length)
{
    if (g_adapter.mode == LORA_MODE)
    {
        g_adapter.lora_config.payload_length = (uint8_t)(payload_length & 0xff);
    }
    else
    {
        g_adapter.fsk_config.payload_length = (uint8_t)(payload_length & 0xff);
    }
}

/**************************************
 * Main adapter routines.
 **************************************/

/**
 * @brief   Adapter initialization
 */

void adapter_init(void)
{
    /* Initialize planned packets. */
    sched_packet_init();

    /* Initialize subghz sublayer. */
    subghz_init();

    #ifdef LORAE5MINI
    subghz_set_regulator_mode(SUBGHZ_REGMODE_SMPS);
    #endif

    /* Initialize RX/TX switch configuration (switch off). */
	gpio_mode_setup(HF_PA_CTRL1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL1_PIN);
    gpio_set(HF_PA_CTRL1_PORT, HF_PA_CTRL1_PIN);
	gpio_mode_setup(HF_PA_CTRL2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL2_PIN);
    gpio_set(HF_PA_CTRL2_PORT, HF_PA_CTRL2_PIN);

    #ifdef NUCLEO_WL55
	gpio_mode_setup(HF_PA_CTRL3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL3_PIN);
    gpio_set(HF_PA_CTRL3_PORT, HF_PA_CTRL3_PIN);
    #endif

    /* Set our callbacks. */
    subghz_set_callbacks(&my_callbacks);

    /* Set our payload base address. */
    subghz_set_buffer_base_address(0, 0);

    /* Initialize LoRa adapter. */
    memset(&g_adapter.lora_config, 0, sizeof(g_adapter.lora_config));
    g_adapter.lora_config.freq = 865200000;
    g_adapter.lora_config.sf = SUBGHZ_LORA_SF7;                      /* Default spreading factor: SF7 */
    g_adapter.lora_config.bw = SUBGHZ_LORA_BW250;                    /* Default bandwidth: 250kHz */
    g_adapter.lora_config.cr = SUBGHZ_LORA_CR_48;                    /* Default coding rate: 4/8 */
    g_adapter.lora_config.payload_length = 40,                      /* Fixed payload length: 200 bytes. */
    g_adapter.lora_config.preamble_length = 12,                      /* Default preamble length: 12 symbols. */
    g_adapter.lora_config.header_type = SUBGHZ_PKT_LORA_VAR_LENGTH,  /* Explicit mode enabled. */
    g_adapter.lora_config.crc_enabled = false,                       /* CRC disabled by default. */
    g_adapter.lora_config.invert_iq = false,                         /* No IQ invert */
    g_adapter.lora_config.ldro = SUBGHZ_LORA_LDRO_ENABLED,           /* LDRO disabled */
    #ifdef NUCLEO_WL55
    g_adapter.lora_config.pa_mode = SUBGHZ_PA_MODE_LP;               /* Power amplifier enabled */
    #endif
    #ifdef LORAE5MINI
    g_adapter.lora_config.pa_mode = SUBGHZ_PA_MODE_HP;               /* Power amplifier enabled */
    #endif
    
    g_adapter.lora_config.pa_power = SUBGHZ_PA_PWR_14DBM;            /* TX 14 dBm */

    /* Initialize FSK adapter. */
    memset(&g_adapter.fsk_config, 0, sizeof(g_adapter.fsk_config));
    g_adapter.fsk_config.freq = 868000000;
    g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW11;
    g_adapter.fsk_config.freq_dev = 50000;
    g_adapter.fsk_config.bit_rate = 1000000;
    g_adapter.fsk_config.crc = false;
    g_adapter.fsk_config.payload_length = 40;
    g_adapter.fsk_config.packet_type = SUBGHZ_PKT_FIXED_LENGTH;
    g_adapter.fsk_config.pa_mode = SUBGHZ_PA_MODE_LP;
    g_adapter.fsk_config.pa_power = SUBGHZ_PA_PWR_14DBM;            /* TX 14 dBm */
    g_adapter.fsk_config.addr_comp = SUBGHZ_ADDR_COMP_DISABLED;
    g_adapter.fsk_config.pulse_shape = SUBGHZ_FSK_GAUSSIAN_NONE;
    g_adapter.fsk_config.whitening = false;

    /* Default mode is LoRa. */
    g_adapter.mode = LORA_MODE;

    /* Adapter is stopped by default. */
    g_adapter.state = STOPPED;
    
    /* Prepared packet. */
    #if 0
    g_sched_pkt.length = 0;
    g_sched_pkt.timestamp = 0;
    #endif

    g_adapter.prepared_packet_rdy = false;
    g_adapter.pp_length = 0;
    g_adapter.pp_timestamp = 0;

    gpio_set(GPIOB, GPIO11);

    /* Put transceiver in standby mode. */
    subghz_lora_mode(&g_adapter.lora_config);
    subghz_set_standby_mode(SUBGHZ_STDBY_HSE32);
}


/**
 * @brief   Start adapter (if configured)
 */

int adapter_start(void)
{
    if (g_adapter.mode == LORA_MODE)
    {
        /* Set transceiver in LoRa mode. */
        if (subghz_lora_mode(&g_adapter.lora_config) == SUBGHZ_ERROR)
        {
            whad_verbose("cannot configure lora: error");
            return 1;
        }
    }
    else
    {
        /* Set transceiver in FSK mode. */
        if (subghz_fsk_mode(&g_adapter.fsk_config) == SUBGHZ_ERROR)
        {
            whad_verbose("cannot configure fsk: error");
            return 1;
        }
    }

    /* Start receiving (no timeout, continuous mode). */
    if (subghz_receive_async(0xFFFFFF) == SUBGHZ_ERROR)
    {
        whad_verbose("err async");
        return 1;
    }

    /* Mark adapter as started. */
    g_adapter.state = STARTED;

    whad_verbose("rf in rx");

    return 0;
}


/**
 * @brief   Stop adapter
 */

void adapter_stop(void)
{
    /* Put transceiver in standby mode. */
    subghz_set_standby_mode(SUBGHZ_STDBY_HSE32);

    /* Mark adapter as stopped. */
    g_adapter.state = STOPPED;
}

/**************************************
 * WHAD Discovery messages callbacks
 **************************************/

/**
 * @brief   Handle discovery reset request
 */

void adapter_on_reset(void)
{
    Message cmd_result;

    /* Soft reset ! */
    
    /* Adapter is ready now ! */
    whad_discovery_ready_resp(&cmd_result);
    whad_send_message(&cmd_result);
}


/**
 * @brief   Handle discovery speed change
 * 
 * @param   speed   `SetTransportSpeed` parameters
 * 
 * With this adapter, there is no need to change the default speed as it
 * cannot really go faster than 115200 bauds :/.
 */

void adapter_on_set_speed(discovery_SetTransportSpeed *speed)
{
    Message cmd_result;
    
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}


/**
 * @brief   Handle device discovery request
 * 
 * The adapter returns some information about itself, including supported protocol
 * versions and firmware-related information.
 * 
 * @param   query   `DeviceInfoQuery` parameters
 */

void adapter_on_device_info_req(discovery_DeviceInfoQuery *query)
{
    Message reply;

    memset(&reply, 0, sizeof(Message));
    whad_discovery_device_info_resp(
        &reply,
        discovery_DeviceType_BtleJack,
        (uint8_t *)"Stm32wl-LoRa",
        0x0002, /* Protocol version supported by this device (2). */
        115200, /* Max speed on UART */
        FIRMWARE_AUTHOR,
        FIRMWARE_URL,
        1,
        0,
        1,
        CAPABILITIES
    );
    
    /* Send response. */
    whad_send_message(&reply);
}


/**
 * @brief   Handle domain discovery request
 * 
 * The adapter returns some information about a supported domain.
 * 
 * @param   query   `DeviceDomainInfoQuery` parameters
 */

void adapter_on_domain_info_req(discovery_DeviceDomainInfoQuery *query)
{
    Message reply;

    switch (query->domain)
    {
        case discovery_Domain_Phy:
        {
            memset(&reply, 0, sizeof(Message));
            whad_discovery_domain_info_resp(
                &reply,
                discovery_Domain_Phy,
                CAPABILITIES
            );
            whad_send_message(&reply);
        }
        break;

        default:
        {
            whad_generic_cmd_result(
                &reply,
                generic_ResultCode_UNSUPPORTED_DOMAIN
            );
            whad_send_message(&reply);
        }
        break;

    }
}


/**
 * @brief   Unsupported message handling
 * 
 * Basically, we return an error.
 */

void adapter_on_unsupported(Message *message)
{
    Message reply;

    whad_generic_cmd_result(&reply, generic_ResultCode_ERROR);
    whad_send_message(&reply);
}


/**
 * WHAD PHY messages callbacks
 */


/**
 * @brief   Handle supported frequencies query.
 * 
 * @param   cmd     `GetSupportedFrequenciesCmd` message
 * 
 * Return the list of supported frequency ranges (min, max).
 * Ranges are statically defined above.
 */

void adapter_on_get_supported_freqs(phy_GetSupportedFrequenciesCmd *cmd)
{
    Message response;

    whad_phy_supported_frequencies(
        &response,
        g_phy_supported_freq_ranges,
        g_phy_supported_ranges_nb
    );

    /* Send back supported freq ranges. */
    whad_send_message(&response);
}


/**
 * @brief   Handle LoRa modulation parameters configuration
 * 
 * @param   cmd     `SetLoRaModulationCmd` message
 * 
 * Save the provided LoRa configuration to use it only when the
 * adapter is started.
 */

void adapter_on_lora_modulation(phy_SetLoRaModulationCmd *cmd)
{
    Message cmd_result;

    /* Switch adapter into LoRa mode. */
    g_adapter.mode = LORA_MODE;

    /* Update LoRa configuration. */
    adapter_set_spreading_factor(cmd->spreading_factor);
    adapter_set_coding_rate(cmd->coding_rate);
    adapter_set_bandwidth(cmd->bandwidth);
    adapter_set_preamble_length(cmd->preamble_length);
    adapter_enable_crc(cmd->enable_crc);
    adapter_enable_explicit_mode(cmd->explicit_mode);
    adapter_enable_invert_iq(cmd->invert_iq);

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}


/**
 * @brief   Handle FSK modulation parameters configuration
 * 
 * @param   cmd     `SetLoRaModulationCmd` message
 * 
 * Save the provided LoRa configuration to use it only when the
 * adapter is started.
 */

void adapter_on_fsk_modulation(phy_SetFSKModulationCmd *cmd)
{
    Message cmd_result;

    /* Update FSK configuration. */
    adapter_set_deviation(cmd->deviation);
    g_adapter.fsk_config.bandwidth = SUBGHZ_FSK_BW11;
    g_adapter.fsk_config.addr_comp = SUBGHZ_ADDR_COMP_DISABLED;

    /* Reset sync word, disable CRC and enabled packet fixed length. */
    g_adapter.fsk_config.sync_word_length = 0;
    g_adapter.fsk_config.crc = SUBGHZ_PKT_CRC_NONE;
    g_adapter.fsk_config.packet_type = SUBGHZ_PKT_FIXED_LENGTH;
    g_adapter.fsk_config.payload_length = 32;

    /* Switch adapter into FSK mode. */
    g_adapter.mode = FSK_MODE;

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

/**
 * @brief   Handle PHY frequency configuration
 * 
 * @param   cmd     `SetFrequencyCmd` message
 * 
 * Save the frequency to use when adapter is started (for TX or RX operations).
 */

void adapter_on_set_freq(phy_SetFrequencyCmd *cmd)
{
    Message cmd_result;

    adapter_set_freq(cmd->frequency);

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}


/**
 * @brief   Handle PHY synchronization word configuration
 * 
 * @param   cmd     `SetSyncWordCmd` message
 * 
 * Set the sub-Ghz synchronization word.
 */

void adapter_on_sync_word(phy_SetSyncWordCmd *cmd)
{
    Message cmd_result;

    /* If mode is FSK, save syncword length in packet properties. */
    if (g_adapter.mode == FSK_MODE)
    {
        /* Check that syncword size does not exceed 8 bytes. */
        if (cmd->sync_word.size > 8)
        {
            whad_generic_cmd_result(&cmd_result, WHAD_RESULT_PARAMETER_ERROR);
            whad_send_message(&cmd_result);
            return;
        }

        /* Save syncword length. */
        g_adapter.fsk_config.sync_word_length = cmd->sync_word.size;
    }
    else
    {
        /* TODO: normally LoRa syncword is on a single byte, these two bytes
           are specific to the STM32WL55... */
        if (cmd->sync_word.size != 2)
        {
            whad_generic_cmd_result(&cmd_result, WHAD_RESULT_PARAMETER_ERROR);
            whad_send_message(&cmd_result);
            return;            
        }
    }

    /* Set the transceiver registers accordingly. */
    if (subghz_set_syncword(cmd->sync_word.bytes, cmd->sync_word.size) == SUBGHZ_SUCCESS)
    {
        /* Success. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
        whad_send_message(&cmd_result);
    }
    else
    {
        /* Failure. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        whad_send_message(&cmd_result);
    }
}


/**
 * @brief   Handle adapter starting process
 * 
 * @param   cmd     `StartCmd` message
 * 
 * Start the RF PHY adapter depending on its configuration.
 */

void adapter_on_start(phy_StartCmd *cmd)
{
    Message cmd_result;
    
    if (adapter_start() != 0)
    {
        /* Failure. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        whad_send_message(&cmd_result);
    }
    else
    {
        /* Success. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
        whad_send_message(&cmd_result);
    }
}


/**
 * @brief   Handle adapter stopping process
 * 
 * @param   cmd     `StopCmd` message
 * 
 * Stop the RF PHY adapter.
 */

void adapter_on_stop(phy_StopCmd *cmd)
{
    Message cmd_result;
    
    adapter_stop();

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}


/**
 * @brief   Handle adapter sniffing mode
 * 
 * Enable sniffing (well, it is already enabled by default).
 */

void adapter_on_sniff(void)
{
    Message cmd_result;

    adapter_start();

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}


/**
 * @brief   Handle adapter packet size modification
 * 
 * @param   cmd: `SetPacketSizeCmd` message
 */

void adapter_on_packet_size(phy_SetPacketSizeCmd *cmd)
{
    Message cmd_result;

    if (cmd->packet_size > 0xFF)
    {
        /* Error. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_PARAMETER_ERROR);
        whad_send_message(&cmd_result);
    }
    else
    {
        adapter_set_payload_length(cmd->packet_size);

        /* Success. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
        whad_send_message(&cmd_result);
    }
}


/**
 * @brief   Handle adapter data rate modification
 * 
 * Well, we don't support data rate. Make the host believe that
 * everything went smooth =).
 * 
 * @param   cmd: `SetDataRateCmd` message
 */

void adapter_on_datarate(phy_SetDataRateCmd *cmd)
{
    Message cmd_result;

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

/**
 * @brief   Handle adapter endianness modification
 * 
 * Again, we don't do endianness with the STM32WLxx series, but
 * as it is required by the host for FSK modulations just let
 * it believe that everything went ok.
 * 
 * @param   endian: endianness
 */

void adapter_on_endianness(phy_Endianness endian)
{
    Message cmd_result;

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}


/**
 * @brief   Handle packet sending
 * 
 * @param   cmd     `SendCmd` message
 * 
 * Send a packet using the configured PHY layer, if adapter is started.
 */

void adapter_on_send_packet(phy_SendCmd *cmd)
{
    Message cmd_result;

    if (g_adapter.state != STARTED)
    {
        /* Error. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        whad_send_message(&cmd_result);
        return;
    }


    /* Send packet, go back to RX if timeout or when packet is successfully sent. */
    if (subghz_send_async(cmd->packet.bytes, cmd->packet.size, 0x9c400) == SUBGHZ_SUCCESS)
    {
        /* Success message will be sent when the packet will be sent. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
        whad_send_message(&cmd_result);
    }
    else
    {
        /* Failure. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        whad_send_message(&cmd_result);        
    }
}


/**
 * @brief   Handle delayed packet sending
 * 
 * @param   cmd     `ScheduleSendCmd` message
 * 
 * Send a packet using the configured PHY layer, if adapter is started.
 */

void adapter_on_sched_send_packet(phy_ScheduleSendCmd *cmd)
{
    Message cmd_result;
    int pkt_id = -1;
    uint32_t ts_sec = sys_get_timestamp_sec();
    uint32_t ts_usec = sys_get_timestamp_usec();
    uint64_t ts = ts_sec*1000000 + ts_usec;

    if (g_adapter.state != STARTED)
    {
        /* Error. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        whad_send_message(&cmd_result);
        return;
    }

    /* If packet is scheduled in the past, return an error. */
    if (cmd->timestamp < ts)
    {
        /* Error. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        whad_send_message(&cmd_result);
        return;
    }

    /* Add packet to our scheduled packets. */
    pkt_id = sched_packet_add(
        cmd->timestamp/1000000,
        cmd->timestamp%1000000,
        cmd->packet.bytes,
        cmd->packet.size
    );

    /* Return the allocated packet id. */
    if (pkt_id >= 0)
    {
        whad_phy_packet_scheduled(&cmd_result, (uint8_t)pkt_id, false);
        whad_send_message(&cmd_result);
    }
    else
    {
        whad_phy_packet_scheduled(&cmd_result, 0, true);
        whad_send_message(&cmd_result);
    }
}

/**
 * Whad message processing.
 **/

void process_phy_message(Message *message)
{
    whad_phy_msgtype_t msg_type = whad_phy_get_message_type(message);

    switch(msg_type)
    {
        /* List supported frequency ranges. */
        case WHAD_PHY_GET_SUPPORTED_FREQS:
        {
            adapter_on_get_supported_freqs(
                &message->msg.phy.msg.get_supported_freq
            );
        }
        break;

        /* Configure adapter for LoRa modulation. */
        case WHAD_PHY_SET_LORA_MOD:
        {
            adapter_on_lora_modulation(
                &message->msg.phy.msg.mod_lora
            );
        }
        break;

        /* Configure adapter for FSK modulation. */
        case WHAD_PHY_SET_FSK_MOD:
        {
            adapter_on_fsk_modulation(
                &message->msg.phy.msg.mod_fsk
            );
        }
        break;

        /* Set adapter frequency. */
        case WHAD_PHY_SET_FREQ:
        {
            adapter_on_set_freq(
                &message->msg.phy.msg.set_freq
            );
        }
        break;

        /* Set adapter synchronization word. */
        case WHAD_PHY_SET_SYNC_WORD:
        {
            adapter_on_sync_word(
                &message->msg.phy.msg.sync_word
            );
        }
        break;

        /* Set adapter endianness. */
        case WHAD_PHY_SET_ENDIANNESS:
        {
            adapter_on_endianness(
                message->msg.phy.msg.endianness.endianness
            );
        }
        break;

        /* Start adapter. */
        case WHAD_PHY_START:
        {
            adapter_on_start(
                &message->msg.phy.msg.start
            );
            
        }
        break;

        /* Stop adapter. */
        case WHAD_PHY_STOP:
        {
            adapter_on_stop(
                &message->msg.phy.msg.stop
            );
        }
        break;

        /* Enable sniffer mode. */
        case WHAD_PHY_SET_SNIFF_MODE:
        {
            adapter_on_sniff();
        }
        break;

        /* Set packet size. */
        case WHAD_PHY_SET_PACKET_SIZE:
        {
            adapter_on_packet_size(&message->msg.phy.msg.packet_size);
        }
        break;

        /* Set datarate. */
        case WHAD_PHY_SET_DATARATE:
        {
            adapter_on_datarate(&message->msg.phy.msg.datarate);
        }
        break;

        /* Send packet through configured PHY. */
        case WHAD_PHY_SEND:
        {
            adapter_on_send_packet(
                &message->msg.phy.msg.send
            );
        }
        break;

        /* Schedule packet send through configured PHY. */
        case WHAD_PHY_SEND_SCHED_PACKET:
        {
            adapter_on_sched_send_packet(
                &message->msg.phy.msg.sched_send
            );
        }
        break;

        /* Unsupported. */
        default:
            adapter_on_unsupported(message);
            break;
    }
}

void process_discovery_message(Message *message)
{
    /* Dispatch discovery message. */
    switch (whad_discovery_get_message_type(message))
    {
        /* Query device information. */
        case WHAD_DISCOVERY_DEVICE_INFO_QUERY:
        {
            adapter_on_device_info_req(
                &message->msg.discovery.msg.info_query
            );
        }
        break;

        /* Query device domain information. */
        case WHAD_DISCOVERY_DOMAIN_INFO_QUERY:
        {
            adapter_on_domain_info_req(
                &message->msg.discovery.msg.domain_query
            );
        }
        break;

        /* Request a software reset. */
        case WHAD_DISCOVERY_DEVICE_RESET:
        {
            /* Send answer and reset device. */
            adapter_on_reset();
        }

        /* Change UART speed. */
        case WHAD_DISCOVERY_SET_SPEED:
        {
            adapter_on_set_speed(
                &message->msg.discovery.msg.set_speed
            );
        }
        break;

        /* Other messages are not supported. */
        default:
            adapter_on_unsupported(message);
            break;
    }
}

/**
 * @brief   Handle WHAD message
 * 
 * @param   message     WHAD message (protobuf) to process.
 */

void dispatch_message(Message *message)
{
    whad_msgtype_t msg_type = whad_get_message_type(message);

    switch (msg_type)
    {
        case WHAD_MSGTYPE_GENERIC:
            {
                /* Not supported for now. */
                adapter_on_unsupported(message);
            }
            break;

        /* PHY domain messages */
        case WHAD_MSGTYPE_DOMAIN:
            {
                if (whad_get_message_domain(message) == DOMAIN_PHY)
                {
                    process_phy_message(message);
                }
                else
                {
                    /* Not supported for now. */
                    adapter_on_unsupported(message);
                }
            }
            break;


        /* Device discovery messages. */
        case WHAD_MSGTYPE_DISCOVERY:
            {
                process_discovery_message(message);
            }
            break;


        /* Other message types are not (yet) supported. */
        default:
            adapter_on_unsupported(message);
            break;
    }
}

void adapter_send_scheduled_packets(void)
{
    /* Send scheduled packet. */
    subghz_send_async(g_sched_pkt.packet, g_sched_pkt.length, 0x9c400);
    
    //gpio_set(GPIOB, GPIO11);
    //GPIOB_BSRR |= (1 << 11); /* set B11 */

    /* Disable timer. */
    timer_disable_oc_output(TIM2, TIM_OC1);
    timer_clear_flag(TIM2, TIM_SR_CC1IF);
    timer_disable_irq(TIM2, TIM_DIER_CC1IE);

    /* Adapter is ready for a new scheduled packet. */
    g_sched_pkt_rdy = false;
    g_sched_pkt_timer_set = false;

    whad_verbose("pkt sent");
}

void adapter_send_rdy(void)
{
    uint32_t ts_msec = sys_get_timestamp_msec();
    uint32_t ts_usec = sys_get_timestamp_usec() % 1000;

    /* Fetch next packet to schedule if required. */
    if (!g_sched_pkt_rdy)
    {
        g_sched_pkt_rdy = sched_get_next(&g_sched_pkt);
    }

    /* Do we have a packet to send ? */
    if (g_sched_pkt_rdy && (!g_sched_pkt_timer_set))
    {
        /* Is it time to setup a timer ? */
        if (g_sched_pkt.ts_msec == ts_msec)
        {
            /* If usec <= 10, send it now. */
            if (g_sched_pkt.ts_usec <= 10)
            {
                adapter_send_scheduled_packets();
            }
            else
            {
                /* Set up a timer trigger for this packet. */
                timer_enable_oc_output(TIM2, TIM_OC1);
                timer_set_oc_polarity_low(TIM2, TIM_OC1);
                timer_clear_flag(TIM2, TIM_SR_CC1IF);
                timer_set_oc_value(TIM2, TIM_OC1, g_sched_pkt.ts_usec);
                timer_enable_irq(TIM2, TIM_DIER_CC1IE);

                g_sched_pkt_timer_set = true;
            }
        }
        else
        {
            /* If packet is scheduled in the past, discard it. */
            if (
                (g_sched_pkt.ts_msec < ts_msec) ||
                ((g_sched_pkt.ts_msec == ts_msec) && (g_sched_pkt.ts_usec <= ts_usec))
            )
            {
                g_sched_pkt_rdy = false;
            }   
        }
    }
}
