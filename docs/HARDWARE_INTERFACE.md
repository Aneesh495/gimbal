# Hardware Interface and Register Specification

This document details the xCORE hardware port mappings, timer counters, and physical interfaces.

## 1. Port Register Allocations

All hardware I/O is performed through single-cycle timed ports on the xCORE processor:

### Tile 0: Sensors
* Port `1A` (`PORT_SPI_CLK`): 10 MHz SPI clock output.
* Port `1B` (`PORT_SPI_MOSI`): Master-Out-Slave-In data line.
* Port `1C` (`PORT_SPI_MISO`): Master-In-Slave-Out data line with input buffering.
* Port `1D` (`PORT_SPI_CS0`): Chip select for IMU Lane 0 (active low).
* Port `1E` (`PORT_SPI_CS1`): Chip select for IMU Lane 1 (active low).
* Port `1F` (`PORT_BISS_CLK`): 5 MHz BiSS-C clock output.
* Port `1G-1I` (`PORT_BISS_DATA`): Synchronous position inputs for Pan, Tilt, Roll.

### Tile 1: 3-Axis Inverter PWM Drive
* Port `4A-4F`: High-side and low-side complementary PWM gate signals.
* 350 ns hardware dead-time insertion configured via clock block divider registers.
* Current sense ADC trigger synchronized to center of zero vector $V_0$.

### Tile 3: Communications Transceivers
* Full-Duplex RS-422 differential transceiver operating at 921,600 baud.
* CAN 2.0B / CANaerospace controller at 1 Mbps.

## 2. Timing and Clock Domains

* Core Tile System Clock: 500 MHz (2.0 ns instruction cycle).
* Reference Timer Clock: 100 MHz (10.0 ns counter resolution).
* PWM Carrier Frequency: 25.0 kHz (4,000 timer ticks per period).
