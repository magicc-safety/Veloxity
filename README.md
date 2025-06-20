# RustFlight

**RustFlight** is a Rust-based port of the **ROSFlight** project, bringing real-time, high-performance flight control to the Rust ecosystem. This project aims to leverage the safety and concurrency features of Rust to create a reliable and maintainable flight control software.

To get started:

1. Fork the repository.
2. Create a new branch following the naming convention.
3. Make your changes and ensure they are well-tested.
4. Open a pull request with a detailed description of the changes.

## Branching Strategy

When creating new branches, please adhere to the following naming convention:

[username]/[feature]

- **[username]**: Your GitHub username or identifier.
- **[feature]**: A concise description of the feature or issue you're working on.

### Example:

For a user named `johndoe` working on a parameter server feature, the branch name should be:

johndoe/param_server

## License

This project is licensed under the [NO IDEA](LICENSE).

## Acknowledgments

This project is a port of [ROSFlight](https://github.com/rosflight/rosflight), originally written in C++.

## How to Build the Project:

1. running "cargo build" builds all the Rustflight specific features
2. running "cargo b_nucleo" builds for the nucleo board (stm32 architecture target: see .cargo/config.toml for details). This build includes all the embedded code, peripherals, and embassy specific code.

## How to Run The Project:
Tests have been created so far for the nucleo board. These are inside the src/bin directory. Shortcuts have been generated for running a test that only includes sensors, and a test for sending/receiving heartbeats:

1. running "cargo r_nucleo_sensors" will start the nucleo board, spinning up tasks for each sensor, and processing them with the sensors module
2. running "cargo r_nucleo_mavlink" will start the nucleo board, spin up the tasks for each sensor, and use the comm_manager in the highest level loop to process incoming serial stream data and match on mavlink messages.

Debug statements throughout the code can be uncommented for debugging/visualization.
