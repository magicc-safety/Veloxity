# AGENTS.md

## Repository Scope

These instructions apply to the Voloxide repository.

## ROSflight Workspace Boundary

The local ROSflight ROS 2 workspace under
`/run/host/home/skink/projects/voloxide_proj/workspace` is a runtime and reference dependency only.

Source the ROSflight environment with the top-level helper:

```bash
source scripts/source_rosflight_env.zsh
source install/setup.zsh
```

Do not modify `rosflight_io` source, generated files, install files, or package configuration.
`rosflight_io` must remain completely unmodified. Integration work must adapt Voloxide/Voloxide
to the existing `rosflight_io` behavior.

ROSflight nodes may be sourced and run for testing.

## Current Sim Integration Target

Use `rosflight_sim` standalone multirotor as the first integration target. The immediate branch
goal is to keep the Voloxide sim firmware endpoint interchangeable with the upstream ROSflight C
SIL firmware endpoint while using the existing ROSflight sim stack and unmodified `rosflight_io`.
