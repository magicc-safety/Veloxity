# Board Bring-Up Guide

This page identifies the hardware included in the current release and directs you to the detailed
instructions for building, flashing, and validating it.

## Supported Hardware

| Board | Crate | Target | Status |
| --- | --- | --- | --- |
| Nucleo-H753ZI | `boards/nucleo` | `thumbv7em-none-eabihf` | Retained and compile-current; renewed sensor and hardware validation is still needed. |
| Pixracer Pro / STM32H7 | `boards/pixracerpro` | `thumbv7em-none-eabihf` | Active hardware-validation target; fixed 400 Hz control timing and high-rate MAVLink telemetry have been validated on hardware. |

## Choose Your Workflow

- For tool installation and explanations of the repository's check, build, and flash commands, use
  [Build and Tool Commands](../build-and-tools.md).
- For STM32 source locations, probe setup, flashing details, peripheral drivers, timing results,
  and the ordered hardware procedure, use [STM32 Boards](stm32.md).
- For how a board supplies `BoardIo`, `PwmDriver`, and the concrete `World` types, use
  [Veloxity Core Architecture](../architecture-guide.md).
- For the purpose of each top-level package and source directory, use the
  [Repository Map](../repository-map.md).