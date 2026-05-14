# AGENTS.md

## Repository Scope

These instructions apply to the Voloxide repository.

## ROSflight Workspace Boundary

The local ROSflight ROS 2 workspace under `/home/skink/projects/rustflight_setup/workspace` is a
runtime and reference dependency only.

Do not modify `rosflight_io` source, generated files, install files, or package configuration.
`rosflight_io` must remain completely unmodified. Integration work must adapt RustFlight/Voloxide
to the existing `rosflight_io` behavior.

ROSflight nodes may be sourced and run for testing.

## Current Sim Integration Target

Use `rosflight_sim` standalone multirotor as the first integration target. The immediate branch
goal is to prove that the RustFlight sim firmware endpoint can connect to the existing ROSflight
sim stack and exchange data with unmodified `rosflight_io`.
