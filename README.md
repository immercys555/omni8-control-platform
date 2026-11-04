# omni8-control-platform

Control & identification architecture for 8-tilt-rotor omnidirectional platform.

## 1. System Architecture & Kinematics
* **Configuration**: Rank-6 controllable 8-tilt-rotor architecture with tangential deflection ($\delta_i = \pm 10^\circ$) providing full $M_z$ yaw authority.
* **Control Hierarchy**:
  * **Slow Layer (SLSQP)**: Optimizes tilt configuration $\theta_i \in [10^\circ, 80^\circ]$ to minimize total power consumption and avoid saturation.
  * **Fast Layer (WLS)**: Regularized Weighted Least Squares fast-allocation ($1\text{ kHz}$) computing motor thrust commands $T_i \in [0, 25\text{ N}]$.
* **Actuator Dynamics**: Integrated first-order lag ($\tau_T = 30\text{ ms}, \tau_\theta = 50\text{ ms}$), dead time ($L = 8\text{ ms}$), and physical rate limits.

## 2. Identification & Closed-Loop Control Pipeline
* **FOPDT V3 Identification Engine**:
  * Uses a 40-point $L$-grid scan combined with Brent local refinement to eliminate gradient discontinuities and negative step degeneration.
* **Dynamic 2D Gain Scheduler**:
  * Seamlessly interpolates $K_c, T_i, K_i$ across operating points ($1200\text{--}1600\,\mu\text{s}$) with $T_i = \tau$ consistency.
  * Features hysteresis margin ($\pm 0.05\text{ N}$) to prevent high-frequency direction chattering.
  * Integrates precision **Bumpless Transfer** recalculating $e_{\text{prev}}$ upon gain/direction switching.
* **6-Level Real-Time ESTOP Safety Net**:
  * Enforces hardware bounds on PWM bounds ($1050\text{--}1950\,\mu\text{s}$), total thrust ($< 20\text{ N}$), temperature limits, run timeouts, propeller dislodgement, and sensor drift.

## 3. Verification & Software-in-the-Loop (SIL) Status
* **Virtual DAQ Simulation (SIL)**:
  * Simulated closed-loop execution incorporating OS thread scheduling jitter, noise, and 1ms pacing.
  * **5-Point Verification Results (Mock/Virtual Env)**: 
    * Sample Counts: $270\text{--}320$ samples per phase ($\text{PASS}$).
    * Timeline Monotonicity: Single-digit jitter, $\Delta t_{\text{mean}} \approx 1.001\text{ ms}$ ($\text{PASS}$).
    * PWM Convergence: Filtered PWM settles cleanly within $\pm 15\,\mu\text{s}$ threshold ($\text{PASS}$).
    * Direction Mapping: Correctly tag POS/NEG scheduling regimes ($\text{PASS}$).
    * Handshake Dynamic: Signed delta $\Delta u \in [-90, +20]\,\mu\text{s}$ across transitions ($\text{PASS}$).
* **C++ Translation Status (Desktop Replay)**:
  * `scheduler.hpp` internal states upgraded to `double` to match Python `float64`.
  * Dual-buffer ring buffer architecture designed for Teensy 4.0 IntervalTimer ISR.
  * **Pending**: Final $1\,\mu\text{s}$ floating-point boundary alignment between C++ replay and Python execution; hardware validation on physical ESC.