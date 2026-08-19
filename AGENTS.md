# gar-servo-pet Agent Rules

This repository owns GarServoPet application behavior and reusable motion
logic. Keep board provisioning and GAR Target Pack logic out of this repository.

## Ownership

- Application code, motion sequencing, and control APIs live here.
- Product-specific servo channels, directions, pulse limits, and mechanical
  calibration live in the parent `GarServoPet/hardware` directory.
- FRDM-IMX91S provisioning and reusable target lifecycle recipes belong in
  `gar-tools`.
- Generated GAR artifacts are build output and must not be committed here.

Changes consumed by the parent repository must be committed and pushed here
before updating the parent's `sources/gar-servo-pet` submodule pointer.
