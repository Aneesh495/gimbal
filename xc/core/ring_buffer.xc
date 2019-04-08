/*
 * Gimbal Core Lock-Free Circular Ring Buffer
 * Deterministic single-producer single-consumer circular FIFO
 * for buffering continuous CSP streaming telemetry packets.
 */

#include "ring_buffer.h"

void ring_buffer_init(ring_buffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->peak_occupancy = 0;
    rb->drop_count = 0;
    for (int i = 0; i < RING_BUFFER_CAPACITY; i++) {
        rb->buffer[i] = 0;
    }
}

uint32_t ring_buffer_available(const ring_buffer_t *rb) {
    uint32_t h = rb->head;
    uint32_t t = rb->tail;
    if (h >= t) {
        return h - t;
    }
    return RING_BUFFER_CAPACITY - (t - h);
}

uint32_t ring_buffer_space(const ring_buffer_t *rb) {
    return (RING_BUFFER_CAPACITY - 1) - ring_buffer_available(rb);
}

bool ring_buffer_push(ring_buffer_t *rb, uint8_t byte) {
    uint32_t next_head = (rb->head + 1) % RING_BUFFER_CAPACITY;
    if (next_head == rb->tail) {
        rb->drop_count++;
        return false; /* Buffer overflow */
    }
    
    rb->buffer[rb->head] = byte;
    rb->head = next_head;
    
    uint32_t occ = ring_buffer_available(rb);
    if (occ > rb->peak_occupancy) {
        rb->peak_occupancy = occ;
    }
    return true;
}

bool ring_buffer_pop(ring_buffer_t *rb, uint8_t *byte_out) {
    if (rb->head == rb->tail) {
        return false; /* Buffer empty */
    }
    
    if (byte_out) {
        *byte_out = rb->buffer[rb->tail];
    }
    rb->tail = (rb->tail + 1) % RING_BUFFER_CAPACITY;
    return true;
}

bool ring_buffer_write_block(ring_buffer_t *rb, const uint8_t *data, uint32_t len) {
    if (ring_buffer_space(rb) < len) {
        rb->drop_count += len;
        return false;
    }
    
    for (uint32_t i = 0; i < len; i++) {
        rb->buffer[rb->head] = data[i];
        rb->head = (rb->head + 1) % RING_BUFFER_CAPACITY;
    }
    
    uint32_t occ = ring_buffer_available(rb);
    if (occ > rb->peak_occupancy) {
        rb->peak_occupancy = occ;
    }
    return true;
}

uint32_t ring_buffer_read_block(ring_buffer_t *rb, uint8_t *dest, uint32_t max_len) {
    uint32_t avail = ring_buffer_available(rb);
    uint32_t to_read = (avail < max_len) ? avail : max_len;
    
    for (uint32_t i = 0; i < to_read; i++) {
        dest[i] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % RING_BUFFER_CAPACITY;
    }
    return to_read;
}

void ring_buffer_clear(ring_buffer_t *rb) {
    rb->tail = rb->head;
}
