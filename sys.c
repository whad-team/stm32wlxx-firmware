#include "sys.h"
#include "adapter.h"
#include <libopencm3/stm32/timer.h>

static volatile uint32_t g_timestamp_sec = 0;

void sys_tick(void)
{
    g_timestamp_sec++;
    adapter_send_rdy();
}

uint32_t sys_get_timestamp_sec(void)
{
    return g_timestamp_sec;
}

uint32_t sys_get_timestamp_usec(void)
{
    return TIM_CNT(TIM2)%1000000;
}