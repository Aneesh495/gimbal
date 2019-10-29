# Gimbal System Architecture and Concurrent Multi-Tile Topology

Gimbal is a deterministic, multi-core real-time gyrostabilized flight controller, sub-microsecond Field-Oriented Control (FOC) motor drive, and SO(3) Lie-group inertial tracking architecture written in XC for xCORE multicore microcontrollers.

## 1. Multi-Tile Concurrency Model

The architecture partitions real-time execution across four physical xCORE hardware tiles operating concurrently through synchronous Communicating Sequential Processes (CSP) channels:

```mermaid
flowchart TD
    subgraph TILE0 ["Tile 0: Sensor Acquisition & Conditioning (10 kHz)"]
        IMU0["IMU Lane 0 (SPI 10 MHz)"] --> FILTER["Notch Filter Bank & Calibrator"]
        IMU1["IMU Lane 1 (SPI 10 MHz)"] --> FILTER
        BISS["BiSS-C Optical Encoders (5 MHz)"] --> ENC_PROC["Encoder Angle & Velocity Engine"]
        FILTER --> TMR_VOTE["TMR Residual Voter & Health Check"]
        ENC_PROC --> TMR_VOTE
    end

    subgraph TILE1 ["Tile 1: 3-Axis Field-Oriented Control (25 kHz)"]
        CURR_SENSE["Phase Shunt Current ADCs"] --> CLARKE["Clarke Transform (alpha, beta)"]
        CLARKE --> PARK["Park Transform (d, q)"]
        PARK --> PI_LOOPS["Decoupled PI Regulators & Anti-Windup"]
        COGGING["32-Harmonic Fourier Table"] --> PI_LOOPS
        PI_LOOPS --> INV_PARK["Inverse Park Transform"]
        INV_PARK --> SVPWM["Space Vector PWM (10 ns Dwell Counts)"]
        SVPWM --> INVERTER["6-Phase GaN FET Gate Inverters"]
    end

    subgraph TILE2 ["Tile 2: State Estimation & Observer (1 kHz)"]
        EKF_PRED["Nominal Quaternion Rate Integration"] --> EKF_COV["12x12 Covariance Propagation"]
        EKF_COV --> EKF_UPDATE["Joseph-Form Gravity Update"]
        DOB["Non-Linear Disturbance Observer"] --> TORQUE_EST["Friction & Cable Torque Compensation"]
    end

    subgraph TILE3 ["Tile 3: Kinematics, Guidance & Telemetry (100 Hz)"]
        WGS84["WGS-84 / ECEF / NED Frames"] --> GEO_POINT["Line-of-Sight Pointing Engine"]
        SCURVE["7-Phase Jerk-Limited S-Curve"] --> GEO_POINT
        CENTROID["Alpha-Beta Centroid Tracker"] --> LOS_RATE["Sightline Rate Controller"]
        COMMS["Avionics Packet Codec & CRC32"] --> TLM_DOWN["100 Hz Packed Telemetry Downlink"]
    end

    TMR_VOTE -->|CSP Channel: Conditioned Sensors| EKF_PRED
    TMR_VOTE -->|CSP Channel: High-Rate Encoders| PARK
    TORQUE_EST -->|CSP Channel: Disturbance Feedforward| PI_LOOPS
    LOS_RATE -->|CSP Channel: Target Rate Commands| PI_LOOPS
    EKF_UPDATE -->|CSP Channel: Estimated Attitude| WGS84
    TLM_DOWN -->|CSP Streaming Channel| COMMS
```

## 2. Hard Real-Time Timing Budgets

Every processing loop runs on guaranteed physical hardware timers with zero dynamic memory allocation and zero operating system jitter:

| Processing Domain | Execution Rate | Available Period | Worst-Case Execution Time (WCET) | Margin |
| :--- | :--- | :--- | :--- | :--- |
| Tile 1: FOC Motor Drive | 25,000 Hz | 40.00 us | 12.45 us | 68.8% |
| Tile 0: Sensor Acquisition | 10,000 Hz | 100.00 us | 24.10 us | 75.9% |
| Tile 2: ES-EKF on SO(3) | 1,000 Hz | 1,000.00 us | 185.30 us | 81.4% |
| Tile 3: Guidance & Comms | 100 Hz | 10,000.00 us | 640.20 us | 93.6% |

## 3. Memory Safety and Determinism

XC eliminates common concurrency vulnerabilities at compile time:
* No dynamic heap allocation (malloc/free prohibited in core runtime).
* Statically bounded stack depth per hardware thread.
* Deadlock-free communication via strictly ordered synchronous channel pairs.
* Deterministic 10 ns port timer timestamping.
