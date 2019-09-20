/*
 * Native xCORE Communicating Sequential Processes (CSP) Runtime Emulator
 * Enables synchronous channel communication, timer events, and port emulation in standard C99.
 */

#include "xc_runtime.h"
#include <string.h>

void xc_channel_init(xc_channel_t *ch) {
    if (!ch) return;
    ch->head = 0;
    ch->tail = 0;
    ch->has_data = false;
    ch->transaction_count = 0;
    memset(ch->buffer, 0, sizeof(ch->buffer));
}

bool xc_channel_send(xc_channel_t *ch, const void *data, size_t size) {
    if (!ch || !data || size > XC_CHAN_BUFFER_SIZE) return false;
    memcpy(ch->buffer, data, size);
    ch->has_data = true;
    ch->transaction_count++;
    return true;
}

bool xc_channel_recv(xc_channel_t *ch, void *dest, size_t size) {
    if (!ch || !dest || !ch->has_data || size > XC_CHAN_BUFFER_SIZE) return false;
    memcpy(dest, ch->buffer, size);
    ch->has_data = false;
    return true;
}

void xc_timer_init(xc_timer_t *tm, uint32_t tick_step) {
    if (!tm) return;
    tm->current_ticks = 0;
    tm->tick_step = tick_step;
}

uint32_t xc_timer_read(xc_timer_t *tm) {
    return tm ? tm->current_ticks : 0;
}

void xc_timer_advance(xc_timer_t *tm, uint32_t ticks) {
    if (tm) {
        tm->current_ticks += ticks;
    }
}

bool xc_timer_after(uint32_t current, uint32_t target) {
    /* 32-bit unsigned modulo comparison */
    return (int32_t)(current - target) >= 0;
}
