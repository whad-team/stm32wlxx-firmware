#ifndef __INC_SCHED_PACKET_H
#define __INC_SCHED_PACKET_H

#define PACKET_MAX_SIZE     256
#define SCHED_PACKET_MAX  10
#define PACKET_PREPARE_TIME 500

#include <unistd.h>
#include <stdint.h>

typedef struct {
  uint8_t id;
  uint32_t ts_msec;
  uint32_t ts_usec;
  uint8_t packet[PACKET_MAX_SIZE];
  int length;
} sched_packet_t;

void sched_packet_init(void);
int sched_packet_add(uint32_t ts_sec, uint32_t ts_usec, uint8_t *p_pkt, int length);
bool sched_get_next(sched_packet_t *p_packet);
void sched_free_packet(uint8_t slot_id);

#endif /* __INC_SCHED_PACKET_H */