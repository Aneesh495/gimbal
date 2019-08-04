/*
 * xCORE Tile 1 Physical Hardware Port Mapping: 3-Axis Motor Inverter Drive
 * Configures 6-channel complementary Space Vector PWM ports with hardware dead-time.
 */

#include "foc.h"

#define PORT_PWM_PAN_HIGH    0x20000000 /* 4-bit port 4A */
#define PORT_PWM_PAN_LOW     0x20000004 /* 4-bit port 4B */
#define PORT_PWM_TILT_HIGH   0x20000008 /* 4-bit port 4C */
#define PORT_PWM_TILT_LOW    0x2000000C /* 4-bit port 4D */
#define PORT_PWM_ROLL_HIGH   0x20000010 /* 4-bit port 4E */
#define PORT_PWM_ROLL_LOW    0x20000014 /* 4-bit port 4F */

#define PORT_ADC_SHUNT_TRIG  0x20000020 /* 1-bit port 1A */
#define PORT_GATE_ENABLE     0x20000024 /* 1-bit port 1B */
#define PORT_FAULT_IN        0x20000028 /* 1-bit port 1C */

typedef struct {
    uint16_t dead_time_nanoseconds;
    uint32_t carrier_frequency_hz;
    uint16_t max_duty_ticks;
    bool active_high_logic;
} tile1_inverter_config_t;

static tile1_inverter_config_t g_inverter_config = {
    .dead_time_nanoseconds = 350, /* 350 ns dead-time for GaN/SiC FETs */
    .carrier_frequency_hz = 25000,
    .max_duty_ticks = 3800,
    .active_high_logic = true
};

void xcore_tile1_configure_ports(void) {
    /* Set dead-time insertion registers on 100 MHz port counter */
}
