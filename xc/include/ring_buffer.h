#ifndef GIMBAL_RING_BUFFER_H
#define GIMBAL_RING_BUFFER_H

#include "gimbal_types.h"

#define RING_BUFFER_CAPACITY 1024

typedef struct {
    uint8_t buffer[RING_BUFFER_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    uint32_t peak_occupancy;
    uint32_t drop_count;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t *rb);
bool ring_buffer_push(ring_buffer_t *rb, uint8_t byte);
bool ring_buffer_pop(ring_buffer_t *rb, uint8_t *byte_out);
uint32_t ring_buffer_available(const ring_buffer_t *rb);
uint32_t ring_buffer_space(const ring_buffer_t *rb);
bool ring_buffer_write_block(ring_buffer_t *rb, const uint8_t *data, uint32_t len);
uint32_t ring_buffer_read_block(ring_buffer_t *rb, uint8_t *dest, uint32_t max_len);
void ring_buffer_clear(ring_buffer_t *rb);

#endif
