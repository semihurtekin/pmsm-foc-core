# Fixed-Point PMSM FOC Core

This is a small personal project to develop the inner current-control
path of PMSM field-oriented control in embedded C. It focuses on the
controller mathematics and fixed-point implementation rather than a specific
motor-control board.

The project includes:

- Clarke, Park and inverse Park transforms
- d-axis and q-axis PI current control
- conditional integration and circular voltage limiting
- min-max space-vector PWM (SVPWM)
- normalized three-phase duty-cycle outputs

There is no external motor-control library used for these calculations.

The code is intentionally independent of a microcontroller. Current sensing,
electrical-angle generation, PWM timer setup, dead time and protection belong
to the target application and are not implemented here.

Signals use signed Q1.15, PI gains use signed Q16.16, and the generated duties
use the full unsigned 16-bit range. The code follows MISRA-C:2012-oriented
practices where practical, but it has not been checked with a formal MISRA
tool (HelixQAC etc.) and is not presented as production-ready software.

## Build

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

Applications may include the complete public API through:

```c
#include "foc.h"
```

See [ALGORITHM.md](ALGORITHM.md) for the implementation notes and
[PORTING.md](PORTING.md) for MCU integration notes.
