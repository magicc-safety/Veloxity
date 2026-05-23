# ism330dhcx-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/ism330dhcx-rs
[crates-url]: https://crates.io/crates/ism330dhcx-rs
[bsd-badge]: https://img.shields.io/crates/l/ism330dhcx-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

Provides a platform-agnostic, no_std-compatible driver for the ST ISM330DHCX sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview

The ISM330DHCX is a system-in-package featuring a high-accuracy and high-performance 3D digital accelerometer and 3D digital gyroscope tailored for Industry 4.0 applications.

All the design aspects and the testing and calibration of the ISM330DHCX have been optimized to reach superior accuracy, stability, extremely low noise and full data synchronization.

The ISM330DHCX has a 3D accelerometer capable of wide bandwidth, ultra-low noise and a selectable full-scale range of ±2/±4/±8/±16 g. The 3D gyroscope has an angular rate range of ±125/±250/±500/±1000/±2000/±4000 dps and offers superior stability over temperature and time along with ultra-low noise.

The unique set of embedded features (Machine Learning Core, programmable FSM, 9 kbytes smart FIFO, sensor hub, event decoding and interrupts) facilitate the implementation of smart and complex sensor nodes which deliver high performance at very low power.

The ISM330DHCX offers specific support, both for the gyroscope and the accelerometer, to applications requiring closed control loop (like OIS and other stabilization applications). The device, through a dedicated auxiliary SPI interface and a configurable signal processing path, can provide data for the control loop while, at the same time, a second fully independent path can output data for other applications.

Like the entire portfolio of MEMS sensor modules, the ISM330DHCX leverages the robust and mature in-house manufacturing processes already used for the production of micromachined accelerometers and gyroscopes. The various sensing elements are manufactured using specialized micromachining processes, while the IC interfaces are developed using CMOS technology that allows the design of a dedicated circuit which is trimmed to better match the characteristics of the sensing element.

In the ISM330DHCX, the sensing elements of the accelerometer and of the gyroscope are implemented on the same silicon die, thus guaranteeing superior stability and robustness.

The ISM330DHCX is available in a small plastic land grid array (LGA) package of 2.5 x 3.0 x 0.83 mm.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/ism330dhcx.html](https://www.st.com/en/mems-and-sensors/ism330dhcx.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
ism330dhcx-rs = "0.1.0"
```

Or, add it directly from the terminal:

```sh
cargo add ism330dhcx-rs
```

## Usage

By default, the create exposes the **asynchronous** API, and it could be included using:
```rust
use ism330dhcx_rs::asynchronous as ism330dhcx;
use ism330dhcx::*;
use ism330dhcx::prelude::*;
```

### Blocking API (optional feature)

To use the **blocking** API instead of the asynchronous one, disable default features and enable the `blocking` feature in your Cargo.toml
```toml
[dependencies]
ism330dhcx-rs = { version = "2.0.0", default-features = false, features = ["blocking"] }
```
or from the terminal:
```sh
cargo add ism330dhcx-rs --no-default-features --features blocking
```

Then import the blocking API:
```rust
use ism330dhcx_rs::blocking as ism330dhcx;
use ism330dhcx::*;
use ism330dhcx::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance, along with a timing peripheral.

An example with I2C:

```rust
let mut sensor = Ism330dhcx::new_i2c(i2c, I2CAddress::I2cAddH, delay);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.device_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Restore default configuration
sensor.reset_set(PROPERTY_ENABLE).unwrap();

loop {
    let rst = sensor.reset_get().unwrap();
    if rst == 0 {
        break;
    }
}

// Start device configuration
sensor.device_conf_set(PROPERTY_ENABLE).unwrap();
// Enable Block Data Update
sensor.block_data_update_set(PROPERTY_ENABLE).unwrap();

// Set Output Data Rate
sensor.xl_data_rate_set(OdrXl::_12_5hz).unwrap();
sensor.gy_data_rate_set(OdrGy::_12_5hz).unwrap();
// Set Full Scale
sensor.xl_full_scale_set(FsXl::_2g).unwrap();
sensor.gy_full_scale_set(FsGy::_2000dps).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**