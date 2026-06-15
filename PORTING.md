# MCU Porting Notes

This project stops at normalized current, angle and duty values. To run it on a
real target, the application has to provide the following pieces around
`Foc_Step()`.

## PWM-Synchronous Current Sampling

Phase currents should be sampled at a repeatable point in the PWM cycle and
away from switching edges. The ADC result must be offset-corrected, scaled to
per-unit Q1.15 and converted to alpha-beta coordinates when needed.

The alpha and beta currents passed to `Foc_Step()` must represent the same
sampling instant.

## Electrical Angle

The caller supplies sine and cosine of the electrical rotor angle in Q1.15:

```text
theta_electrical =
    pole_pairs * theta_mechanical + calibrated_angle_offset
```

The angle may come from an encoder, resolver, interpolated Hall signals or a
sensorless observer. Angle estimation and sine/cosine generation are target
application responsibilities.

## PWM Duty Conversion

`Foc_Step()` returns normalized duty values:

```text
0       -> 0 percent
65535   -> 100 percent
```

Convert each value to the target timer range:

```text
compare = duty * pwm_period / 65535
```

Use a wide intermediate for this multiplication. The MCU PWM layer remains
responsible for center-aligned operation, shadow-register updates, dead time,
minimum pulse width and protection inputs.
