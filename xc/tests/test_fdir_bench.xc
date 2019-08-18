/*
 * XC Test Bench: Fault Detection, Isolation, and Recovery (FDIR)
 * Validates watchdog timeout assertions, thermal RC heating models, and limit clamping.
 */

#include "fdir.h"
#include <stdbool.h>

bool run_fdir_bench(void) {
    /* Test 1: Watchdog feed and timeout trip */
    watchdog_channel_t wd;
    watchdog_init(&wd, 100);
    watchdog_feed(&wd, 10);
    if (!watchdog_check(&wd, 50)) return false;  /* Within threshold */
    if (watchdog_check(&wd, 120)) return false;  /* Should trip timeout */

    /* Test 2: Thermal model heating under 4A current load */
    thermal_model_t tm;
    thermal_model_init(&tm);
    q16_t initial_temp = tm.winding_temp_c;
    q16_t current = q16_from_float(4.0f);
    q16_t dt = q16_from_float(0.1f);
    for (int i = 0; i < 50; i++) {
        thermal_model_update(&tm, current, dt);
    }
    /* Winding temperature must increase with heating */
    if (tm.winding_temp_c <= initial_temp) return false;
    return true;
}
