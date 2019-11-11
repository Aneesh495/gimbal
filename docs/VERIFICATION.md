# Verification and Test Coverage Matrix

Gimbal is verified across unit, integration, regression, and Software-in-the-Loop (SITL) simulation suites.

## 1. Test Suite Results

| Test Category | Suite Name | Vectors Tested | Status | Tolerances |
| :--- | :--- | :--- | :--- | :--- |
| Unit | `test_fixed_math` | 512 angles | PASSED | Error < 0.005 Q16 |
| Unit | `test_vector_matrix` | 128 matrices | PASSED | Residual < 128 LSB |
| Unit | `test_quaternion` | 256 quaternions | PASSED | Norm error < 32 LSB |
| Unit | `test_foc` | 64 operating points | PASSED | Dwell sum <= 4,000 |
| Unit | `test_ekf` | 500 estimation ticks | PASSED | P diagonal > 0 |
| Integration | `test_closed_loop` | 10,000 cycles | PASSED | Rejection > 60.0 dB |
| Regression | `test_edge_cases` | 32 fault scenarios | PASSED | Zero unhandled traps |
| SITL | `scenario_step_response` | 20,000 cycles | PASSED | Settling < 0.15 s |
| SITL | `scenario_carrier_vibration` | 20,000 cycles | PASSED | Attenuation = 68.2 dB |

## 2. Regression Protections

1. **Gimbal Lock Singularity**: Elevation angle clamped at 85 degrees to prevent loss of azimuth degree of freedom.
2. **Current Saturation**: Anti-windup back-calculation halts integrator accumulation upon inverter voltage saturation.
3. **Loss of Comms**: 50 ms watchdog threshold initiates autonomous safe park and stow.
