# AGENTS.md

## Repository Scope

These instructions apply to the Veloxity repository.

## ROSflight Workspace Boundary

The local ROSflight ROS 2 workspace is a runtime and reference dependency only. Its location is
provided by the caller's already-sourced shell environment.

Do not source ROSflight helper scripts from outside this repository in Veloxity scripts. Assume ROS 2
and the ROSflight workspace are already sourced by the caller before building or running Veloxity
ROS 2 integration scripts.

```bash
source scripts/build_and_source_ros2_shim.zsh
```

Do not modify `rosflight_io` source, generated files, install files, or package configuration.
`rosflight_io` must remain completely unmodified. Integration work must adapt Veloxity to the
existing `rosflight_io` behavior.

ROSflight nodes may be sourced and run for testing.

## Current Sim Integration Target

Use `rosflight_sim` standalone multirotor as the first integration target. The immediate branch
goal is to keep the Veloxity sim firmware endpoint interchangeable with the upstream ROSflight C
SIL firmware endpoint while using the existing ROSflight sim stack and unmodified `rosflight_io`.
