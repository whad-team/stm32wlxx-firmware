#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ppacket.h"

/* Our global planned packet array. */
static planned_packet_t g_planned_pkts[PLANNED_PACKET_MAX];

void planpacket_init(void)
{
    int i;

    for (i=0; i<PLANNED_PACKET_MAX; i++)
    {
        g_planned_pkts[i].timestamp = 0;
        g_planned_pkts[i].length = 0;
    }
}

bool planpacket_add(uint32_t timestamp, uint8_t *p_pkt, int length)
{
    int i;

    /* Search a free slot. */
    for (i=0; i<PLANNED_PACKET_MAX; i++)
    {
        if (g_planned_pkts[i].timestamp == 0)
        {
            /* Found a free slot, add packet. */
            g_planned_pkts[i].timestamp = timestamp;
            g_planned_pkts[i].length = length;
            memcpy(g_planned_pkts[i].packet, p_pkt, length);

            /* Found a slot. */
            return true;
        }
    }

    /* No free slot available. */
    return false;
}

void planpacket_clear(uint32_t timestamp)
{
    int i;

    /* Search a packet to send. */
    for (i=0; i<PLANNED_PACKET_MAX; i++)
    {
        if (g_planned_pkts[i].timestamp <= timestamp)
        {
            /* Found a free slot, free it and return packet. */
            g_planned_pkts[i].timestamp = 0;
        }
    }   
}

bool planpacket_find(uint32_t timestamp, uint8_t *p_pkt, int *p_length, uint32_t *p_timestamp)
{
    int i;

    /* Search a packet to send. */
    for (i=0; i<PLANNED_PACKET_MAX; i++)
    {
        if ((g_planned_pkts[i].timestamp > 0) && (g_planned_pkts[i].timestamp <= (timestamp + PACKET_PREPARE_TIME)))
        {
            /* Found a free slot, free it and return packet. */
            *p_timestamp = g_planned_pkts[i].timestamp;
            g_planned_pkts[i].timestamp = 0;
            *p_length = g_planned_pkts[i].length;
            memcpy(p_pkt, g_planned_pkts[i].packet, g_planned_pkts[i].length);

            /* Found a slot. */
            return true;
        }
    }

    /* No planned packet ready to send. */
    return false;
}