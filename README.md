# gar-servo-pet

Application source repository for **GarServoPet**, a four-axis desk robot built
around an NXP FRDM-IMX91S, a PCA9685 PWM controller, and four SG90 servos.

This repository owns application behavior and motion logic. The parent Product
repository, `GarServoPet`, owns hardware mappings, deployment configuration, and
the pinned source set used to build a reproducible GAR artifact.

## Planned responsibilities

- Named poses and keyframe-based motion sequences
- Safe servo limits, easing, and staggered movement
- PCA9685 output through the Linux I2C interface
- A local HTTP/WebSocket control API
- Event-driven behaviors such as status and CI notifications

The initial mechanical role of each of the four servos is intentionally left
open until the chassis design is selected.

## Repository boundary

```text
gar-servo-pet              application code and behavior
GarServoPet/hardware       Product-specific wiring and servo calibration
gar-tools/targets/*        reusable Target Pack support
GaplessAgentRuntime        build, artifact, simulation, and deployment contracts
```

GarServoPet includes this repository as `sources/gar-servo-pet`.
