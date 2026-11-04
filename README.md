# omni8-control-platform

Control & identification architecture for 8-tilt-rotor omnidirectional platform.

## 1. System Architecture & Kinematics

**Configuration**: Rank-6 controllable 8-tilt-rotor architecture with tangential deflection (δi = ±10°) providing full Mz yaw authority.

**Control Hierarchy**:
- **Slow Layer (SLSQP)**: Optimizes tilt configuration θi ∈ [10°, 80°] to minimize total power consumption and avoid saturation.
- **Fast Layer (WLS)**: Regularized Weighted Least Squares fast-allocation (1 kHz) computing motor thrust commands Ti ∈ [0, 25 N].

**Actuator Dynamics**: Integrated first-order lag (τT = 30 ms, τθ = 50 ms), dead time (L = 8 ms), and physical rate limits.

## 2. Identification & Closed-Loop Control Pipeline

**FOPDT V3 Identification Engine**:
- Uses a 40-point L-grid scan combined with Brent local refinement to eliminate gradient discontinuities and negative step degeneration.

**Dynamic 2D Gain Scheduler**:
- Seamlessly interpolates Kc, Ti, Ki across operating points (1200–1600 μs) with Ti = τ consistency.
- Features hysteresis margin (±0.05 N) to prevent high-frequency direction chattering.
- Integrates precision Bumpless Transfer recalculating eprev upon gain/direction switching.

**6-Level Real-Time ESTOP Safety Net**:
- Enforces hardware bounds on PWM bounds (1050–1950 μs), total thrust (< 20 N), temperature limits, run timeouts, propeller dislodgement, and sensor drift.

## 3. Verification & Software-in-the-Loop (SIL) Status

**Virtual DAQ Simulation (SIL)**:
- Simulated closed-loop execution incorporating OS thread scheduling jitter, noise, and 1ms pacing.

**5-Point Verification Results (Mock/Virtual Env)**:
- Sample Counts: 270–320 samples per phase (PASS).
- Timeline Monotonicity: Single-digit jitter, Δtmean ≈ 1.001 ms (PASS).
- PWM Convergence: Filtered PWM settles cleanly within ±15 μs threshold (PASS).
- Direction Mapping: Correctly tag POS/NEG scheduling regimes (PASS).
- Handshake Dynamic: Signed delta Δu ∈ [−90, +20] μs across transitions (PASS).

**C++ Translation Status (Desktop Replay)**:
- `scheduler.hpp` internal states upgraded to `double` to match Python `float64`.
- Dual-buffer ring buffer architecture designed for Teensy 4.0 IntervalTimer ISR.
- **Pending**: Final 1 μs floating-point boundary alignment between C++ replay and Python execution; hardware validation on physical ESC.
