#ifndef __INC_SYS_H
#define __INC_SYS_H

#include <unistd.h>
#include <stdint.h>

void sys_tick(void);
uint32_t sys_get_timestamp_sec(void);
uint32_t sys_get_timestamp_msec(void);
uint32_t sys_get_timestamp_usec(void);

#endif /*__INC_SYS_H */