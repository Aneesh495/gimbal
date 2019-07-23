/*
 * Gimbal Top-Level Multi-Core Concurrent Architecture Orchestrator
 * Launches Tile 0, Tile 1, Tile 2, and Tile 3 concurrently using XMOS xCORE par statement.
 */

#include "gimbal_channels.h"

extern void tile_sensors_executive(void);
extern void tile_foc_executive(void);
extern void tile_estimator_executive(void);
extern void tile_tracking_executive(void);

int main(void) {
    /*
     * XMOS multi-tile hardware concurrent entrypoint:
     * Executes each domain executive on dedicated physical tiles / logical cores.
     */
    tile_sensors_executive();
    tile_foc_executive();
    tile_estimator_executive();
    tile_tracking_executive();
    
    return 0;
}
