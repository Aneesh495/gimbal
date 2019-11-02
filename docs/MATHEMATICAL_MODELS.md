# Mathematical Foundations and Derivations

This document provides the formal mathematical derivations implemented across the Gimbal core modules.

## 1. Field-Oriented Control (FOC) Transformations

### Forward Clarke Transformation
Assuming a balanced 3-phase system with $i_a + i_b + i_c = 0$, the forward Clarke transformation projects from 3-phase coordinates to stationary orthogonal 2-phase coordinates $(\alpha, \beta)$:

$$i_\alpha = i_a$$
$$i_\beta = \frac{1}{\sqrt{3}} (i_a + 2 i_b) = \frac{i_b - i_c}{\sqrt{3}}$$

### Park Transformation
The rotating reference frame transformation projects stationary currents $(\alpha, \beta)$ into direct (flux) and quadrature (torque) axes $(d, q)$ using electrical rotor angle $\theta_e$:

$$i_d =  i_\alpha \cos(\theta_e) + i_\beta \sin(\theta_e)$$
$$i_q = -i_\alpha \sin(\theta_e) + i_\beta \cos(\theta_e)$$

### Space Vector Pulse Width Modulation (SVPWM)
The reference voltage vector $V_{ref} = V_\alpha + j V_\beta$ is synthesized in the complex plane using adjacent active vectors $V_1, V_2$ and zero vectors $V_0, V_7$:

$$T_1 = \frac{\sqrt{3} T_s}{V_{dc}} \left( V_\beta \cos\left(\frac{(n-1)\pi}{3}\right) - V_\alpha \sin\left(\frac{(n-1)\pi}{3}\right) \right)$$
$$T_2 = \frac{\sqrt{3} T_s}{V_{dc}} \left( -V_\beta \cos\left(\frac{n\pi}{3}\right) + V_\alpha \sin\left(\frac{n\pi}{3}\right) \right)$$
$$T_0 = T_s - T_1 - T_2$$

## 2. Error-State Extended Kalman Filter (ES-EKF) on SO(3)

The attitude state is parameterized on the Lie group $SO(3)$ using unit quaternions. The true state $x$ is decomposed into a nominal state $\hat{x}$ and an error state $\delta x$:

$$x = \hat{x} \oplus \delta x$$

### Nominal State Kinematics
$$\dot{\hat{q}} = \frac{1}{2} \hat{q} \otimes \begin{bmatrix} 0 \\ \omega - \hat{b}_\omega \end{bmatrix}$$
$$\dot{\hat{b}}_\omega = 0$$

### Error State Vector
$$\delta x = \begin{bmatrix} \delta \theta \\ \delta b_\omega \end{bmatrix} \in \mathbb{R}^6$$

### Error Covariance Propagation
$$P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k$$

where the state transition matrix $F_k$ is:
$$F_k = \begin{bmatrix} I - [\hat{\omega}_k \times] \Delta t & -I \Delta t \\ 0 & I \end{bmatrix}$$

### Joseph-Form Stabilized Covariance Update
To guarantee numerical symmetry and strict positive-definiteness on fixed-point processors:

$$P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T$$

## 3. Stribeck Non-Linear Friction Dynamics

Friction torque opposing joint angular velocity $\omega$ follows the generalized Stribeck curve:

$$\tau_{friction}(\omega) = \left[ \tau_c + (\tau_s - \tau_c) \exp\left( -\left(\frac{\omega}{\omega_s}\right)^2 \right) \right] \operatorname{sgn}(\omega) + \sigma_v \omega$$

where:
* $\tau_c$ is Coulomb dynamic friction torque.
* $\tau_s$ is static breakaway friction torque.
* $\omega_s$ is the Stribeck characteristic transition velocity.
* $\sigma_v$ is the viscous damping coefficient.
