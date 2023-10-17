#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "schedpkt.h"

/* Our global planned packet array. */
static sched_packet_t g_sched_pkts[SCHED_PACKET_MAX];
static volatile int g_next_sched_pkt = -1;

void sched_packet_init(void)
{
    int i;

    /* Look for an available slot. */
    for (i=0; i<SCHED_PACKET_MAX; i++)
    {
        g_sched_pkts[i].id = i;
        g_sched_pkts[i].ts_sec = 0;
        g_sched_pkts[i].ts_usec = 0;
        g_sched_pkts[i].length = 0;
    }
}

int sched_packet_add(uint32_t ts_sec, uint32_t ts_usec, uint8_t *p_pkt, int length)
{
    int i;

    /* Look for an available slot. */
    for (i=0; i<SCHED_PACKET_MAX; i++)
    {
        if (g_sched_pkts[i].length == 0)
        {
            /* Save packet timestamp... */
            g_sched_pkts[i].ts_sec = ts_sec;
            g_sched_pkts[i].ts_usec = ts_usec;

            /* ... and content. */
            g_sched_pkts[i].length = length;
            memcpy(g_sched_pkts[i].packet, p_pkt, length);

            /* Update next packet to send. */
            if (g_next_sched_pkt >= 0)
            {
                if ((g_sched_pkts[g_next_sched_pkt].ts_sec > g_sched_pkts[i].ts_sec) ||
                    (
                        (g_sched_pkts[g_next_sched_pkt].ts_sec == g_sched_pkts[i].ts_sec) &&
                        (g_sched_pkts[g_next_sched_pkt].ts_usec > g_sched_pkts[i].ts_usec)
                    )
                )
                {
                    /* Next packet to send should be this one. */
                    g_next_sched_pkt = i;
                }
            }
            else
            {
                g_next_sched_pkt = i;
            }

            /* Success. */
            return i;
        }
    }
   
    /* No more available slot. */
    return -1;
}

void _prepare_next_pkt(void)
{
    int i,j=-1;
    uint32_t sec=0xffffffff, usec=0xffffffff;

    g_next_sched_pkt = -1;

    /* Loop on packet array and look for next packet to send. */
    for (i=0; i<SCHED_PACKET_MAX; i++)
    {
        /* If current packet timestamp is not zero and lower. */
        if ((g_sched_pkts[i].ts_sec > 0) || (g_sched_pkts[i].ts_usec > 0))
        {
            /* If packet timestamp seconds is lower or same with usec lower,
               keep track of this packet.
            */
            if (
                (g_sched_pkts[i].ts_sec < sec) ||
                ((g_sched_pkts[i].ts_sec == sec) && (g_sched_pkts[i].ts_usec < usec))
            )  
            {
                /* Save packet info. */
                sec = g_sched_pkts[i].ts_sec;
                usec = g_sched_pkts[i].ts_usec;
                j = i;
            }

            break;
        }
    }

    /* Have we found the next scheduled packet to send ? */
    if (j >= 0)
    {
        /* Save next packet slot to be sent. */
        g_next_sched_pkt = j;
    }
}

bool sched_get_next(sched_packet_t *p_packet)
{
    if (g_next_sched_pkt >= 0)
    {
        /* Copy next packet to be sent into our structure. */
        memcpy(p_packet, &g_sched_pkts[g_next_sched_pkt], sizeof(sched_packet_t));

        /* Mark packet slot as free. */
        sched_free_packet(g_next_sched_pkt);

        /* Success. */
        return true;
    }
    else
    {
        /* No packet to schedule. */
        return false;
    }
}


void sched_free_packet(uint8_t slot_id)
{
    if ((slot_id >= 0) && (slot_id < SCHED_PACKET_MAX))
    {
        /* Mark slot as free. */
        g_sched_pkts[slot_id].length = 0;
        g_sched_pkts[slot_id].ts_sec = 0;
        g_sched_pkts[slot_id].ts_usec = 0;

        /* Look for the next packet to send. */
        _prepare_next_pkt();
    }
}