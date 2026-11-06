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

---

## Architectural Feasibility: Sparsity Constraints & Dynamic Synaptic Plasticity

To guarantee **O(N)** linear efficiency while preserving backpropagation dynamics during localized feature absorption, the M3 architecture incorporates a dynamic synaptic topology matrix **S_ij(t)** governed by a strict zero-norm bound:

`||S_ij(t)||_0 <= k * N` *(where k << N)*

### Differentiability via Straight-Through Estimator (STE)

Discrete index selection (e.g., Top-k pruning) inherently induces zero-gradient propagation for inactive pathways. To bypass discrete gradient blockages and enable continuous synaptic re-growth, M3 integrates a custom Straight-Through Estimator (STE) proxy:

`S_t = STE_TopK(W, k)`

*   **Forward Pass**: Enforces sparse tensor execution by applying a binary Top-K mask.
*   **Backward Pass**: Bypasses discrete mask gradients, passing gradients directly back to the dense parameter matrix **W**. This allows dormant connections to reactivate based on dynamic sensory loss.

### Hardware Compatibility & PyTorch Limitations

This hardware-aware formulation targets native execution on neuromorphic architectures (e.g., Intel Loihi, BrainChip) and edge NPU accelerators.

> **Engineering Caveat:** In this PyTorch implementation, the sparse mask generation uses `torch.zeros_like(W)`, which still allocates a dense VRAM footprint (O(N²) memory). This is a high-level executable proxy. True physical O(N) memory savings require a custom Triton or CUDA kernel with block-sparse memory allocation.

