#ifndef __INC_ADAPTER_H
#define __INC_ADAPTER_H

#include <whad.h>

#define     FIRMWARE_AUTHOR "Damien Cauquil"
#define     FIRMWARE_URL "https://github.com/whad-team/stm32wlxx-firmware"

void adapter_init(void);
void dispatch_message(Message *p_msg);

#endif /* __INC_ADAPTER_H */