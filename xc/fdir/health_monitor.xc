/*
 * Gimbal Real-Time Health Monitor and Multi-Tile Watchdogs
 * Monitors CSP inter-tile heartbeats and asserts fail-safe transitions upon communication loss.
 */

#include "fdir.h"

void watchdog_init(watchdog_channel_t *wd, uint32_t timeout_ticks) {
    if (!wd) return;
    wd->last_heartbeat_tick = 0;
    wd->timeout_threshold_ticks = timeout_ticks;
    wd->missed_heartbeat_count = 0;
    wd->trip_alarm = false;
}

void watchdog_feed(watchdog_channel_t *wd, uint32_t current_tick) {
    if (!wd) return;
    wd->last_heartbeat_tick = current_tick;
    wd->missed_heartbeat_count = 0;
    wd->trip_alarm = false;
}

bool watchdog_check(watchdog_channel_t *wd, uint32_t current_tick) {
    if (!wd) return true;
    
    if (current_tick >= wd->last_heartbeat_tick) {
        uint32_t elapsed = current_tick - wd->last_heartbeat_tick;
        if (elapsed > wd->timeout_threshold_ticks) {
            wd->missed_heartbeat_count++;
            wd->trip_alarm = true;
            return false; /* Watchdog timeout tripped */
        }
    }
    return true;
}
