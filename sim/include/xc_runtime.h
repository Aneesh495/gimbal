#ifndef SIM_XC_RUNTIME_H
#define SIM_XC_RUNTIME_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define XC_CHAN_BUFFER_SIZE 64

typedef struct {
    uint8_t buffer[XC_CHAN_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
    bool has_data;
    uint32_t transaction_count;
} xc_channel_t;

typedef struct {
    uint32_t current_ticks;
    uint32_t tick_step;
} xc_timer_t;

void xc_channel_init(xc_channel_t *ch);
bool xc_channel_send(xc_channel_t *ch, const void *data, size_t size);
bool xc_channel_recv(xc_channel_t *ch, void *dest, size_t size);

void xc_timer_init(xc_timer_t *tm, uint32_t tick_step);
uint32_t xc_timer_read(xc_timer_t *tm);
void xc_timer_advance(xc_timer_t *tm, uint32_t ticks);
bool xc_timer_after(uint32_t current, uint32_t target);

#endif
