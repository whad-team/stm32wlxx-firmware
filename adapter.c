#include "adapter.h"

extern void print_dbg(char *psz_debug);

typedef enum {
    STOPPED = 0,
    STARTED
} adapter_state_t;

typedef struct {
    adapter_state_t state;
    uint16_t sync_word;
    subghz_lora_config_t config;
} adapter_t;

/**
 * Adapter WHAD capabilities and domains
 */

static DeviceCapability g_adapter_cap[] = {
    {discovery_Domain_Phy, discovery_Capability_Inject | discovery_Capability_Sniff},
    {0, 0}
};
static uint64_t g_phy_supported_commands = (
    (1 << phy_PhyCommand_GetSupportedFrequencies) |
    (1 << phy_PhyCommand_SetLoRaModulation) |
    (1 << phy_PhyCommand_SetFrequency) |
    (1 << phy_PhyCommand_SetSyncWord) |
    (1 << phy_PhyCommand_Send) |
    (1 << phy_PhyCommand_Start) |
    (1 << phy_PhyCommand_Stop)
);

const phy_SupportedFrequencyRanges_FrequencyRange g_phy_supported_freq_ranges[]  = {
    {865000000, 870000000},
    {0, 0}
};
const int g_phy_supported_ranges_nb = 1;

static void adapter_on_rf_switch_cb(bool tx);
static void adapter_on_pkt_received(uint8_t offset, uint8_t length);
static void adapter_on_pkt_sent(void);

static subghz_callbacks_t my_callbacks = {
    .pfn_on_packet_recvd = adapter_on_pkt_received,
    .pfn_on_packet_sent = adapter_on_pkt_sent,
    .pfn_on_rf_switch = adapter_on_rf_switch_cb,
    .pfn_on_timeout = adapter_on_pkt_sent,
};


static adapter_t g_adapter;

/* handle RF switch config. */
static void adapter_on_rf_switch_cb(bool tx)
{
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
}

static void adapter_on_pkt_received(uint8_t offset, uint8_t length)
{
    Message msg;
    uint8_t rxbuf[256];
    int8_t rssi = 0;
    int8_t rssi_inst = 0;

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

        /* Report packet. */
        whad_phy_packet_received(
            &msg,
            g_adapter.config.freq,
            rssi_inst,
            0,
            rxbuf,
            length
        );
        whad_send_message(&msg);
    }
}

static void adapter_on_pkt_sent(void)
{
    /* Switch back to async rx mode. */
    subghz_receive_async(0xFFFFFF);
}


/**
 * Adapter routines
 */

void adapter_set_spreading_factor(phy_LoRaSpreadingFactor spreading_factor)
{
    switch (spreading_factor)
    {
        case phy_LoRaSpreadingFactor_SF8:
            g_adapter.config.sf = SUBGHZ_LORA_SF8;
            break;

        case phy_LoRaSpreadingFactor_SF9:
            g_adapter.config.sf = SUBGHZ_LORA_SF9;
            break;

        case phy_LoRaSpreadingFactor_SF10:
            g_adapter.config.sf = SUBGHZ_LORA_SF10;
            break;

        case phy_LoRaSpreadingFactor_SF11:
            g_adapter.config.sf = SUBGHZ_LORA_SF11;
            break;

        case phy_LoRaSpreadingFactor_SF12:
            g_adapter.config.sf = SUBGHZ_LORA_SF12;
            break;

        default:
        case phy_LoRaSpreadingFactor_SF7:
            g_adapter.config.sf = SUBGHZ_LORA_SF7;
            break;
    }
}

void adapter_set_coding_rate(phy_LoRaCodingRate coding_rate)
{
    switch (coding_rate)
    {
        case phy_LoRaCodingRate_CR45:
            g_adapter.config.cr = SUBGHZ_LORA_CR_45;
            break;

        case phy_LoRaCodingRate_CR46:
            g_adapter.config.cr = SUBGHZ_LORA_CR_46;
            break;

        case phy_LoRaCodingRate_CR47:
            g_adapter.config.cr = SUBGHZ_LORA_CR_47;
            break;

        case phy_LoRaCodingRate_CR48:
            g_adapter.config.cr = SUBGHZ_LORA_CR_48;
            break;
    }
}

void adapter_set_bandwidth(phy_LoRaBandwidth bandwidth)
{
    switch (bandwidth)
    {
        case phy_LoRaBandwidth_BW125:
            g_adapter.config.bw = SUBGHZ_LORA_BW125;
            break;

        case phy_LoRaBandwidth_BW250:
            g_adapter.config.bw = SUBGHZ_LORA_BW250;
            break;

        default:
        case phy_LoRaBandwidth_BW500:
            g_adapter.config.bw = SUBGHZ_LORA_BW500;
            break;
    }
}

void adapter_set_syncword(uint16_t u16_sync_word)
{
    g_adapter.sync_word = u16_sync_word;
}

void adapter_set_freq(uint32_t freq)
{
    g_adapter.config.freq = freq;
}

void adapter_enable_crc(bool enabled)
{
    g_adapter.config.crc_enabled = enabled;
}

void adapter_set_preamble_length(uint16_t preamble_length)
{
    g_adapter.config.preamble_length = (preamble_length & 0xFFFF);
}

void adapter_enable_explicit_mode(bool enabled)
{
    if (enabled)
    {
        g_adapter.config.header_type = SUBGHZ_PKT_LORA_VAR_LENGTH;
    }
    else
    {
        g_adapter.config.header_type = SUBGHZ_PKT_LORA_FIXED_LENGTH;
    }
}


void adapter_init(void)
{
    /* Initialize subghz sublayer. */
    subghz_init();

    /* Initialize RX/TX switch configuration. */
	gpio_mode_setup(HF_PA_CTRL1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL1_PIN);
    gpio_set(HF_PA_CTRL1_PORT, HF_PA_CTRL1_PIN);
	gpio_mode_setup(HF_PA_CTRL2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL2_PIN);
    gpio_set(HF_PA_CTRL2_PORT, HF_PA_CTRL2_PIN);
	gpio_mode_setup(HF_PA_CTRL3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HF_PA_CTRL3_PIN);
    gpio_set(HF_PA_CTRL3_PORT, HF_PA_CTRL3_PIN);

    /* Set our callbacks. */
    subghz_set_callbacks(&my_callbacks);

    /* Set our payload base address. */
    subghz_set_buffer_base_address(0, 0);

    /* Initialize LoRa adapter. */
    g_adapter.config.freq = 865200000;
    g_adapter.config.sf = SUBGHZ_LORA_SF7;                      /* Default spreading factor: SF7 */
    g_adapter.config.bw = SUBGHZ_LORA_BW250;                    /* Default bandwidth: 250kHz */
    g_adapter.config.cr = SUBGHZ_LORA_CR_48;                    /* Default coding rate: 4/8 */
    g_adapter.config.payload_length = 40,                      /* Fixed payload length: 200 bytes. */
    g_adapter.config.preamble_length = 12,                      /* Default preamble length: 12 symbols. */
    g_adapter.config.header_type = SUBGHZ_PKT_LORA_VAR_LENGTH,  /* Explicit mode enabled. */
    g_adapter.config.crc_enabled = false,                       /* CRC disabled by default. */
    g_adapter.config.invert_iq = false,                         /* No IQ invert */
    g_adapter.config.ldro = SUBGHZ_LORA_LDRO_DISABLED,          /* LDRO disabled */
    g_adapter.config.pa_mode = SUBGHZ_PA_MODE_HP,               /* Power amplifier enabled */
    g_adapter.config.pa_power = SUBGHZ_PA_PWR_14DBM;            /* TX 14 dBm */

    g_adapter.state = STOPPED;
}

int adapter_start(void)
{
    /* Set 16-bit LoRa sync word. */
    //subghz_set_syncword((uint8_t *)&g_adapter.sync_word, 2);
    
    /* Set transceiver in LoRa mode. */
    if (subghz_lora_mode(&g_adapter.config) == SUBGHZ_ERROR)
    {
        return 1;
    }

    /* Start receiving (no timeout, continuous mode). */
    if (subghz_receive_async(0xFFFFFF) == SUBGHZ_ERROR)
    {
        whad_verbose("err async");
        return 1;
    }

    /* Mark adapter as started. */
    g_adapter.state = STARTED;

    return 0;
}

void adapter_stop(void)
{
    /* Put transceiver in standby mode. */
    subghz_set_standby_mode(SUBGHZ_STDBY_HSE32);

    /* Mark adapter as stopped. */
    g_adapter.state = STOPPED;
}


/**
 * WHAD Discovery messages callbacks
 */

void adapter_on_reset(void)
{
    Message cmd_result;

    /* Soft reset ! */
    
    /* Adapter is ready now ! */
    whad_discovery_ready_resp(&cmd_result);
    whad_send_message(&cmd_result);
}

void adapter_on_set_speed(discovery_SetTransportSpeed *speed)
{
    Message cmd_result;
    
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);

    #if 0
    if (speed->speed <= 460800)
    {
        /* Send success message. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
        send_pb_message(&cmd_result);

        vTaskDelay(200 / portTICK_PERIOD_MS);

        /* Reconfigure UART0 */
        reconfigure_uart(speed->speed, false);
    }
    else
    {
        /* Send error message. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        send_pb_message(&cmd_result);   
    }
    #endif
}

void adapter_on_device_info_req(discovery_DeviceInfoQuery *query)
{
    Message reply;

    memset(&reply, 0, sizeof(Message));
    whad_discovery_device_info_resp(
        &reply,
        discovery_DeviceType_BtleJack,
        (uint8_t *)"Stm32wl-LoRa",
        0x0100,
        115200, /* Max speed on UART */
        FIRMWARE_AUTHOR,
        FIRMWARE_URL,
        1,
        0,
        0,
        g_adapter_cap
    );
    
    /* Send response. */
    whad_send_message(&reply);
}

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
                g_phy_supported_commands
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

void adapter_on_unsupported(Message *message)
{
    Message reply;

    whad_generic_cmd_result(&reply, generic_ResultCode_ERROR);
    whad_send_message(&reply);
}

/**
 * WHAD PHY messages callbacks
 */

void adapter_on_get_supported_freqs(phy_GetSupportedFrequenciesCmd *cmd)
{
    Message response;

    whad_phy_supported_frequencies(
        &response,
        (phy_SupportedFrequencyRanges_FrequencyRange *)g_phy_supported_freq_ranges,
        g_phy_supported_ranges_nb
    );

    /* Send back supported freq ranges. */
    whad_send_message(&response);
}

void adapter_on_lora_modulation(phy_SetLoRaModulationCmd *cmd)
{
    Message cmd_result;

    /* Update LoRa configuration. */
    adapter_set_spreading_factor(cmd->spreading_factor);
    adapter_set_coding_rate(cmd->coding_rate);
    adapter_set_bandwidth(cmd->bandwidth);
    adapter_set_preamble_length(cmd->preamble_length);
    adapter_enable_crc(cmd->enable_crc);
    adapter_enable_explicit_mode(cmd->explicit);

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

void adapter_on_set_freq(phy_SetFrequencyCmd *cmd)
{
    Message cmd_result;

    adapter_set_freq(cmd->frequency);

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

void adapter_on_sync_word(phy_SetSyncWordCmd *cmd)
{
    Message cmd_result;

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

void adapter_on_stop(phy_StopCmd *cmd)
{
    Message cmd_result;
    
    adapter_stop();

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

void adapter_on_send_packet(phy_SendCmd *cmd)
{
    Message cmd_result;

    /* Set frequency again. */
    //subghz_set_rf_freq(g_adapter.config.freq);

    /* Send packet, go back to RX if timeout or when packet is successfully sent. */
    if (subghz_send_async(cmd->packet.bytes, cmd->packet.size, 0x9c400) == SUBGHZ_SUCCESS)
    {
        /* Success. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
        whad_send_message(&cmd_result);
    }
    else
    {
        /* Success. */
        whad_generic_cmd_result(&cmd_result, generic_ResultCode_ERROR);
        whad_send_message(&cmd_result);        
    }
}


/**
 * Whad message processing.
 **/

void dispatch_message(Message *message)
{
    switch (message->which_msg)
    {
        case Message_generic_tag:
            {
                /* Not supported for now. */
                adapter_on_unsupported(message);
            }
            break;

        case Message_phy_tag:
            {
                switch(message->msg.phy.which_msg)
                {
                    case phy_Message_get_supported_freq_tag:
                    {
                        adapter_on_get_supported_freqs(
                            &message->msg.phy.msg.get_supported_freq
                        );
                    }
                    break;

                    case phy_Message_mod_lora_tag:
                    {
                        adapter_on_lora_modulation(
                            &message->msg.phy.msg.mod_lora
                        );
                    }
                    break;

                    case phy_Message_set_freq_tag:
                    {
                        adapter_on_set_freq(
                            &message->msg.phy.msg.set_freq
                        );
                    }
                    break;

                    case phy_Message_sync_word_tag:
                    {
                        adapter_on_sync_word(
                            &message->msg.phy.msg.sync_word
                        );
                    }
                    break;

                    case phy_Message_start_tag:
                    {
                        adapter_on_start(
                            &message->msg.phy.msg.start
                        );
                        
                    }
                    break;

                    case phy_Message_stop_tag:
                    {
                        adapter_on_stop(
                            &message->msg.phy.msg.stop
                        );
                    }
                    break;

                    case phy_Message_send_tag:
                    {
                        adapter_on_send_packet(
                            &message->msg.phy.msg.send
                        );
                    }
                    break;

                    default:
                        adapter_on_unsupported(message);
                        break;
                }
            }
            break;

        case Message_discovery_tag:
            {
                /* Dispatch discovery message. */
                switch (message->msg.discovery.which_msg)
                {
                    case discovery_Message_info_query_tag:
                    {
                        /* Forward DeviceInfo query to adapter. */
                        adapter_on_device_info_req(
                            &message->msg.discovery.msg.info_query
                        );
                    }
                    break;

                    case discovery_Message_domain_query_tag:
                    {
                        adapter_on_domain_info_req(
                            &message->msg.discovery.msg.domain_query
                        );
                    }
                    break;

                    case discovery_Message_reset_query_tag:
                    {
                        /* Send answer and reset device. */
                        adapter_on_reset();
                    }

                    case discovery_Message_set_speed_tag:
                    {
                        /* Change UART speed. */
                        adapter_on_set_speed(
                            &message->msg.discovery.msg.set_speed
                        );
                    }
                    break;

                    default:
                        adapter_on_unsupported(message);
                        break;
                }
            }
            break;

        default:
            adapter_on_unsupported(message);
            break;
    }
}