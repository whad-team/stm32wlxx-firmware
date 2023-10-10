#include "sys.h"

static volatile uint32_t g_timestamp = 0;

void sys_tick(void)
{
    g_timestamp++;
    adapter_send_rdy();
}

uint32_t sys_get_timestamp(void)
{
    return g_timestamp;
}