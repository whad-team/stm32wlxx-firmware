#ifndef __INC_PPACKET_H
#define __INC_PPACKET_H

#define PACKET_MAX_SIZE     256
#define PLANNED_PACKET_MAX  10
#define PACKET_PREPARE_TIME 500

#include <unistd.h>
#include <stdint.h>

typedef struct {
  uint32_t timestamp;
  uint8_t packet[PACKET_MAX_SIZE];
  int length;
} planned_packet_t;

void planpacket_init(void);
bool planpacket_add(uint32_t timestamp, uint8_t *p_pkt, int length);
void planpacket_clear(uint32_t timestamp);
bool planpacket_find(uint32_t timestamp, uint8_t *p_pkt, int *p_length, uint32_t *p_timestamp);

#endif /* __INC_PPACKET_H */