# Algorithm Notes

This file records the conventions used by the code. It is not intended to be a
complete introduction to FOC; the purpose is to make the implementation
choices easy to follow while reading the source.

## Numeric Formats

I used signed Q1.15 for normalized current, voltage, sine and cosine values:

```text
q15_value = per_unit_value * 32768
```

Q1.15 is convenient for a small embedded implementation, but intermediate
results must be wider than 16 bits. Multiplications are therefore performed in
32 or 64 bits before the result is scaled and saturated back to Q1.15.

PI gains use signed Q16.16 so that gains greater than one can be represented.
The configured integral gain is already discretized:

```text
ki_config = Ki * Ts
```

## Clarke Transformation

For balanced phase currents, only two components are independent. The selected
amplitude-invariant form aligns the alpha axis with phase A:

```text
i_alpha = i_a
i_beta  = (i_a + 2 * i_b) / sqrt(3)
```

`Foc_Step()` accepts alpha-beta current directly because current reconstruction
depends on the hardware. `Foc_Clarke()` remains available when the application
already has suitable phase-current samples.

## Park Transformation

The Park transform rotates the measured stationary current into the electrical
rotor frame:

```text
i_d =  i_alpha * cos(theta) + i_beta * sin(theta)
i_q = -i_alpha * sin(theta) + i_beta * cos(theta)
```

With the d axis aligned to rotor flux, the sinusoidal alpha-beta current becomes
approximately constant in steady state. This allows the d and q currents to be
controlled with ordinary PI controllers.

The sign convention is important. The inverse Park transform uses the matching
matrix, and the tests check that a Park/inverse-Park round trip reconstructs the
original vector within fixed-point quantization error.

## PI Current Controllers

Each axis uses the parallel PI form:

```text
error       = reference - feedback
integral[k] = integral[k-1] + Ki * Ts * error
output      = Kp * error + integral + feed_forward
```

The feed-forward input is optional. It can later be used for cross-coupling or
back-EMF compensation without putting a motor model inside this small core.

The axis PI controllers use conditional integration. When an axis is saturated
and the error would drive it farther into saturation, that integral update is
not accepted.

The inverter limit is shared by the d and q axes, so limiting them independently
could change the requested voltage-vector angle. Instead, the combined request
is checked against a circle:

```text
magnitude = sqrt(v_d^2 + v_q^2)
scale     = voltage_limit / magnitude
```

When necessary, the same scale is applied to both axes. The implementation also
restores both PI integral states for that iteration. This is a deliberately
simple anti-windup choice; a complete drive could instead use back-calculation
tied to the applied voltage.

## Inverse Park Transformation

After limiting, the requested voltage is rotated back into the stationary frame
used by the modulator:

```text
v_alpha = v_d * cos(theta) - v_q * sin(theta)
v_beta  = v_d * sin(theta) + v_q * cos(theta)
```

## Space-Vector PWM

The SVPWM implementation uses the min-max form. First, inverse Clarke equations
produce three phase references:

```text
v_a = v_alpha
v_b = -v_alpha / 2 + sqrt(3) * v_beta / 2
v_c = -v_alpha / 2 - sqrt(3) * v_beta / 2
```

Then a common-mode component is added:

```text
v_offset = -(max(v_a, v_b, v_c) + min(v_a, v_b, v_c)) / 2
duty_x   = 0.5 + v_x + v_offset
```

Adding the same offset to all three phases does not change the line-to-line
voltages. It centers the phase commands in the available DC-link range and, in
the linear region, gives the same switching times as the conventional
sector/T1/T2 construction.

The sector is also reported for tests and diagnostics, although min-max duty
generation does not need it. A target with single-shunt current reconstruction
or sector-dependent sampling could use this information.

The alpha-beta voltage is expected to be normalized to the DC-link voltage.
With this convention, `1 / sqrt(3)` is used as the normal linear-modulation
limit (`18919` in Q1.15). The returned duties are normalized; timer period,
dead time and minimum-pulse handling remain outside this project.
