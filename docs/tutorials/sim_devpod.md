# Veloxity with the ROSflight Simulator

Veloxity is compatible with the [ROSflight sim](https://docs.rosflight.org/latest/user-guide/rosflight-sim/).

We use a Rust/C Foreign-Function Interface (FFI) compiled as a static library to bridge between the two without performance penalties introduced by traditional bridges.

Because of the build steps required to properly build and link Veloxity and ROSflight, the recommended path to flying with the Veloxity firmware in sim is to clone
and follow the instructions on our [Devpod](https://github.com/Derekbenj/rosflight_devpod/tree/TRLarsen/devpod_test_updates), which ensure reproducibility.

Once the Devpod has built successfully, simply substitute `veloxity_sil_board_shim` for `rosflight_sim` in any of the commands in the [ROSflight tutorials](https://docs.rosflight.org/latest/user-guide/tutorials/)
to fly in sim with the Veloxity firmware!
