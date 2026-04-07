# PID Controller — C Implementation

A lightweight, production-ready discrete-time PID controller written in C. Forked from [pms67/PID](https://github.com/pms67/PID) and adapted for embedded systems use.

---

## Features

- Trapezoidal integration for improved accuracy
- Anti-windup via integrator clamping
- Derivative on measurement (no derivative kick on setpoint change)
- Band-limited differentiator (first-order low-pass filter on derivative term)
- Configurable output and integrator limits
- Minimal dependencies — pure C, no stdlib required

---

## Files

| File | Description |
|---|---|
| `PID.h` | Controller struct definition and function declarations |
| `PID.c` | Controller logic — init and update functions |

---

## Usage

### 1. Configure the controller

```c
PIDController pid = {
    .Kp  = 2.0f,
    .Ki  = 0.5f,
    .Kd  = 0.1f,

    .tau = 0.02f,        /* Derivative low-pass filter time constant */

    .T   = 0.01f,        /* Sample period (seconds) */

    .limMin    = -100.0f,   /* Output limits */
    .limMax    =  100.0f,

    .limMinInt = -50.0f,    /* Integrator limits (anti-windup) */
    .limMaxInt =  50.0f,
};
```

### 2. Initialise

```c
PIDController_Init(&pid);
```

### 3. Call in your control loop

```c
// Call at a fixed rate matching pid.T
float output = PIDController_Update(&pid, setpoint, measurement);
```

---

## Parameters

| Parameter | Description |
|---|---|
| `Kp` | Proportional gain |
| `Ki` | Integral gain |
| `Kd` | Derivative gain |
| `tau` | Derivative filter time constant — higher = smoother, more lag |
| `T` | Sample period in seconds — must match your loop rate |
| `limMin / limMax` | Output clamp (e.g. PWM range, voltage range) |
| `limMinInt / limMaxInt` | Integrator clamp to prevent windup |

---

## How It Works

The controller runs a standard PID algorithm adapted for discrete-time execution on a microcontroller.

**Proportional** — scales the raw error directly.

**Integral** — accumulates error over time using the trapezoidal rule, which is more accurate than simple rectangular summation. Clamped independently to prevent integrator windup when the output is saturated.

**Derivative** — computed on the measurement rather than the error to avoid large output spikes on setpoint changes. A first-order low-pass filter (controlled by `tau`) suppresses high-frequency noise amplification.

---

## Compatibility

Designed for embedded targets — tested on STM32 (ARM Cortex-M). Should work on any platform with a C99 compiler and floating-point support (hardware or software).

---

## Credits

Original implementation by [pms67](https://github.com/pms67/PID).  
This fork removes documentation files and adapts the code for direct embedded integration.

---

## License

Refer to the [original repository](https://github.com/pms67/PID) for licence information.
