---
description: >
  Reference for the quaternion-based cascade PID control architecture. Use when
  modifying the attitude loop, rate loop, error computation, derivative modes,
  target quaternion construction, or integral enable logic.
when_to_use: >
  Trigger on: PID, attitude loop, rate loop, angle loop, quaternion error,
  axis-angle, canonical, q and -q, antipodal, DerivativeMode, integral windup,
  pidAttLoop, pidRateLoop, angleControlLoop, rateControlLoop, setPoint,
  targetQuaternion, fromEuler, error extraction, control cascade.
---

# Quaternion-Based Cascade PID Control

## Architecture Overview

Three cascade loops run at different frequencies:

```
Radio sticks (50 Hz)
    │
    ▼
FSM / mainFSM_task (50 Hz)          stateMachine.cpp — FlyingState::update()
    │  builds m_setPoint.m_targetQuaternion (STAB)
    │  or sets m_setPoint.m_targetRate* directly (ACRO)
    │
    ▼
Attitude loop / pidAtt_task (1 kHz)  flightCore.cpp — pidAttLoop()
    │  quaternion error → axis-angle → degrees per body axis
    │  output: m_angleLoop[i].m_output → m_setPoint.m_targetRate* (STAB)
    │
    ▼
Rate loop / pidRate_task (2 kHz)     flightCore.cpp — pidRateLoop()
    │  error = targetRate - gyro (deg/s)
    │  output: m_torqueX/Y/Z
    │
    ▼
Motor mixer / ESCs_task (500 Hz)
```

## Target Quaternion Construction (FSM — stateMachine.cpp:136)

```cpp
m_setPoint.m_targetQuaternion = Quaternion<float>::fromEuler(
    m_radio.m_targetRoll  * DEG_TO_RAD,   // deg → rad
    m_radio.m_targetPitch * DEG_TO_RAD,
    m_radio.m_targetYaw   * DEG_TO_RAD
);
```

- **Rotation order**: XYZ intrinsic (roll first, then pitch, then yaw).
- Roll and pitch are **direct angle targets** (stick = angle, dead zone 1°).
- Yaw is **integrated**: `m_targetYaw += stick_rate * 400 deg/s * dt`, clamped to [-180, +180].
  Yaw is never a direct angle command — it accumulates while flying.
- `fromEuler` and `toEuler` exist but are **only for display and target construction**.
  They are never used inside the PID control loop itself.

## Attitude Loop — pidAttLoop() (flightCore.cpp:666)

### Step 1 — Canonical form (mandatory)

```cpp
Quaternion<float> qEst    = Quaternion<float>::canonical(m_madgwickFilter.m_qEst);
Quaternion<float> qTarget = Quaternion<float>::canonical(m_setPoint.m_targetQuaternion);
```

`q` and `-q` represent the same rotation, but `q1 * q2 ≠ (-q1) * q2`.
Without canonicalization, when `q.w` crosses zero (antipodal point), the error
quaternion sign flips, causing a sudden 360° reversal in all control outputs.
`canonical(q)` enforces `q.w >= 0` by negating the whole quaternion if needed.
**Apply to BOTH qEst and qTarget before any error computation.**

### Step 2 — Error quaternion

```cpp
Quaternion<float> qError = PID::getError(qEst, qTarget);
// = qTarget * qEst.inverse()   (normalized)
```

Semantics: the rotation needed, expressed in the current body frame, to go from
the current orientation to the target. Multiplication order matters: `target * current.inverse()`,
not the other way around.

### Step 3 — Axis-angle extraction (NOT Euler angles)

```cpp
Vector3<float> rotAxis;
float angleRad = 0.0f;
qError.toAxisAngle(rotAxis, angleRad);
// angleRad = 2 * acos(qError.w)     (w clamped to [-1, 1])
// rotAxis  = (qError.xyz) / sin(angleRad/2)
// near identity (s < 1e-6): axis = qError.xyz directly
```

**Never extract Euler angles from qError for control.** Euler angles introduce
gimbal lock at ±90° pitch. The axis-angle representation is singularity-free.

### Step 4 — Project onto body axes

```cpp
error[0] = rotAxis.m_x * angleRad * RAD_TO_DEG;  // roll error  (degrees)
error[1] = rotAxis.m_y * angleRad * RAD_TO_DEG;  // pitch error (degrees)
error[2] = rotAxis.m_z * angleRad * RAD_TO_DEG;  // yaw error   (degrees)
```

This is a projection of the rotation vector `(axis × angle)` onto each body axis.
Units are degrees. The PID gains are tuned for this unit.

### Step 5 — Angle PID + output filter

```cpp
// In angleControlLoop() (controlStrategy.cpp:49):
float pidOut = m_angleLoop[i].computePID(error[i], 0.0f, 0.0f, dt, integrate);
m_angleLoop[i].m_output = m_angleLoop[i].m_filteredOutput.apply(pidOut);
```

- `measure = 0` and `target = 0` are passed because **derivative mode is OnError**
  for the angle loop — the derivative is computed on the error signal, not on gyro.
- The output is filtered by a **Butterworth 2nd-order LPF at 15 Hz** (`pidAngleOutputCutOffFreq`
  in `mainSetup()`). This is a command filter, not a D-term filter.
- The filtered output feeds `m_setPoint.m_targetRate*` in STAB mode (set by FSM).

## Rate Loop — pidRateLoop() (flightCore.cpp:642)

```cpp
// In rateControlLoop() (controlStrategy.cpp:29):
const float error = m_rateLoop[i].m_target - m_rateLoop[i].m_measure;
m_rateLoop[i].m_output = m_rateLoop[i].computePID(
    error, m_rateLoop[i].m_measure, m_rateLoop[i].m_target, dt, true);
```

- **Measure** = `m_imu.m_gyroFilterRates` (deg/s, a different filter path from AHRS gyro).
- **Derivative mode** = `OnMeasurement`: `d = -(measure - prevMeasure) * freq * kd`.
  The minus sign is intentional — it prevents derivative kick on sudden target changes.
- **D-term filtered** by two cascaded first-order LPFs at 10 Hz each (`m_dTermLpf` then
  `m_dTermLpf2`), initialized at `rateLoopFreq = 2000 Hz`. This is aggressive noise rejection.
- **Integral** always active (`integrate = true` hardcoded), anti-windup clamp to `±m_saturation`.

## Derivative Mode Summary

| Loop | DerivativeMode | `measure` passed | `target` passed | Effect |
|---|---|---|---|---|
| Rate | `OnMeasurement` | gyro (deg/s) | target rate | d on -gyro; no kick on target jump |
| Angle | `OnError` | `0.0f` (ignored) | `0.0f` (ignored) | d on error difference; gyro not used |
| Pos | `OnError` | `0.0f` | `0.0f` | stub (TODO) |

Swapping these modes silently changes stability behavior without a compile error.

## Integral Enable Logic

| Loop | `integrate` value | Where set |
|---|---|---|
| Rate | `true` always | `rateControlLoop()` line 30 |
| Angle | `m_isFlying` | `pidAttLoop()` line 702 |
| Pos | N/A (stub) | — |

`m_isFlying` is driven by hysteretic thrust thresholds:
- `m_targetThrust > 340` → flying = true
- `m_targetThrust < 250` → flying = false

The angle loop integral is **frozen on the ground** to prevent windup at idle.
The rate loop integral runs unconditionally, including on the ground at 0% throttle.

## Mode Switching (STAB vs ACRO)

In `FlyingState::update()` (stateMachine.cpp:154):

| Mode | `m_setPoint.m_targetRate*` source |
|---|---|
| ACRO | `m_radio.m_targetRate*` directly |
| STAB | `m_ctrlStrat.m_angleLoop[i].m_output` (angle loop result) |

`m_angleLoopEnable` is only `true` in STAB and POSHOLD (`mainSetup()` line 213).
When disabled, `pidAttLoop()` returns immediately — the quaternion error is never computed.

## Anti-Patterns

- **Never compute `qError = qEst * qTarget.inverse()`** — multiplication order is
  `target * current.inverse()`, not the reverse. Reversing it inverts all control signs.
- **Never skip `canonical()` on either quaternion** — the antipodal flip will cause
  instantaneous 360° torque spikes at random orientations.
- **Never extract Euler angles from `qError` for control** — gimbal lock at ±90° pitch.
  `toEuler()` is only safe for display.
- **Never pass gyro as `measure` to the angle loop** — it uses `OnError` mode; passing
  gyro there would incorrectly use gyro as the derivative signal instead of error delta.
- **Never add a new filter after `m_filteredOutput`** in the angle chain — the 15 Hz
  Butterworth is already the final stage before rate loop input. Adding another filter
  increases phase lag and can destabilize the cascade.
