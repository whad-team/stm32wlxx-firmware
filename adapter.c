#include "adapter.h"

static DeviceCapability g_adapter_cap[] = {
    {discovery_Domain_Phy, discovery_Capability_Inject | discovery_Capability_Sniff},
    {0, 0}
};
static uint64_t g_phy_supported_commands = (
    (1 << phy_PhyCommand_GetSupportedFrequencies) |
    (1 << phy_PhyCommand_SetGFSKModulation) |
    (1 << phy_PhyCommand_SetFrequency) |
    (1 << phy_PhyCommand_SetSyncWord) |
    (1 << phy_PhyCommand_Start) |
    (1 << phy_PhyCommand_Stop)
);

const phy_SupportedFrequencyRanges_FrequencyRange g_phy_supported_freq_ranges[]  = {
    {868000000, 870000000},
    {0, 0}
};
const int g_phy_supported_ranges_nb = 1;

/**
 * Adapter routines
 */

void adapter_init(void)
{
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
    whad_init_error_message(&reply, generic_ResultCode_ERROR);
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
        g_phy_supported_freq_ranges,
        g_phy_supported_ranges_nb
    );

    /* Send back supported freq ranges. */
    whad_send_message(&response);
}

void adapter_on_gfsk_modulation(phy_SetGFSKModulationCmd *cmd)
{
    Message cmd_result;

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

void adapter_on_set_freq(phy_SetFrequencyCmd *cmd)
{
    Message cmd_result;

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

void adapter_on_sync_word(phy_SetSyncWordCmd *cmd)
{
    Message cmd_result;

    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

void adapter_on_start(phy_StartCmd *cmd)
{
    Message cmd_result;
    
    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
}

void adapter_on_stop(phy_StopCmd *cmd)
{
    Message cmd_result;
    
    /* Success. */
    whad_generic_cmd_result(&cmd_result, generic_ResultCode_SUCCESS);
    whad_send_message(&cmd_result);
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

                    case phy_Message_mod_gfsk_tag:
                    {
                        adapter_on_gfsk_modulation(
                            &message->msg.phy.msg.mod_gfsk
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

                    default:
                        adapter_on_unsupported(message);
                }
            }
            break;

        default:
            break;
    }
}