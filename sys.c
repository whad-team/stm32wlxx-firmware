#include "sys.h"
#include "adapter.h"
#include <libopencm3/stm32/timer.h>

static volatile uint32_t g_timestamp_sec = 0;
static volatile uint32_t g_timestamp_msec = 0;

void sys_tick(void)
{
    /* Increment milliseconds. */
    g_timestamp_msec++;

    /* And seconds. */
    if (g_timestamp_msec >= 1000)
    {
        g_timestamp_sec++;
        g_timestamp_msec = 0;
    }


    adapter_send_rdy();
}

uint32_t sys_get_timestamp_sec(void)
{
    return g_timestamp_sec;
}

uint32_t sys_get_timestamp_msec(void)
{
    return g_timestamp_sec*1000 + g_timestamp_msec;
}


uint32_t sys_get_timestamp_usec(void)
{
    return TIM_CNT(TIM2)%1000 + g_timestamp_msec*1000;
}