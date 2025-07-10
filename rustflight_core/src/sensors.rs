// /**
// ******************************************************************************
// * File     : sensors.rs
// * Date     : May 8, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/
use micro_algebra::stack::quaternion::Quaternion;

//#[cfg(feature = "nucleo")]
// pub use {
// //defmt::{trace, Format},
//embassy_time::{Duration, Instant},
//};

// ------------------------------------------------------------------------ Mock Timing on Host Computer -----------------------------------------------------------------------

//#[cfg(feature = "default")]
//#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
//pub struct Duration {
//    micros: u64, // Use u64 for the duration in microseconds
//}

//#[cfg(feature = "default")]
//impl Duration {
//    // Constructor that creates a Duration from micros (u64)
//    pub const fn from_micros(micros: u64) -> Self {
//        Duration { micros: micros }
//    }

//    // Convert the custom Duration to core::time::Duration
//    pub fn to_core_duration(&self) -> core::time::Duration {
//        core::time::Duration::from_micros(self.micros)
//    }

//    // Convert from core::time::Duration to custom Duration
//    pub fn from_core_duration(duration: core::time::Duration) -> Self {
//        Duration {
//            micros: duration.as_secs() * 1_000_000 + duration.subsec_micros() as u64,
//        }
//    }

//    // Get the duration in microseconds (u64)
//    pub fn as_micros(&self) -> u64 {
//        self.micros
//    }
//}

//#[cfg(feature = "default")]
//pub use {
//    log::info,
//    mock_instant::global::Instant, // <-- relies on "duration"
//};

//#[cfg(feature = "default")]
//pub trait InstantExt {
//    fn as_micros(&self) -> u64;
//    fn from_micros(micros: u64) -> Self;
//}

//#[cfg(feature = "default")]
//impl InstantExt for Instant {
//    fn as_micros(&self) -> u64 {
//        let duration = self.elapsed(); // Get elapsed time
//        duration.as_secs() as u64 * 1_000_000 + duration.subsec_micros() as u64
//    }

//    fn from_micros(micros: u64) -> Instant {
//        let now = Instant::now();
//        now + Duration::from_micros(micros).to_core_duration()
//    }
//}

// ---------------------------------------------------------------------- End Mock Timing on Host Computer -----------------------------------------------------------------

// ------------------------------------------------------------------------ Logging on Host Computer -----------------------------------------------------------------------

//#[cfg(feature = "default")]
//pub mod host_rtt {
//    use core::fmt::{self, Write}; // Added explicit Write import
//    use libc::{c_char, close, mkfifo, open, write, O_WRONLY};

//    const FIFO_PATH: &[u8] = b"/tmp/rustflight_rtt\0";
//    const BUF_SIZE: usize = 128;

//    pub struct RttWriter {
//        fd: i32,
//        buffer: [u8; BUF_SIZE],
//        position: usize,
//    }

//    impl RttWriter {
//        pub fn new() -> Self {
//            unsafe {
//                mkfifo(FIFO_PATH.as_ptr() as *const c_char, 0o666);
//                let fd = open(FIFO_PATH.as_ptr() as *const c_char, O_WRONLY);
//                Self {
//                    fd,
//                    buffer: [0u8; BUF_SIZE],
//                    position: 0,
//                }
//            }
//        }

//        fn write_bytes(&mut self, data: &[u8]) -> fmt::Result {
//            let mut bytes = data;
//            while !bytes.is_empty() {
//                let remaining = self.buffer.len() - self.position;
//                let copy_len = core::cmp::min(bytes.len(), remaining);
//
//                self.buffer[self.position..self.position + copy_len]
//                    .copy_from_slice(&bytes[..copy_len]);
//                self.position += copy_len;
//                bytes = &bytes[copy_len..];
//
//                if self.position == self.buffer.len() {
//                    self.flush()?; // Propagate errors
//                }
//            }
//            Ok(())
//        }

//        pub fn flush(&mut self) -> fmt::Result {
//            if self.position > 0 {
//                let written =
//                    unsafe { write(self.fd, self.buffer.as_ptr() as *const _, self.position) };
//
//                if written < 0 {
//                    return Err(fmt::Error);
//                }
//
//                self.position = 0;
//            }
//            Ok(())
//        }
//    }
//
//    impl Write for RttWriter {
//        fn write_str(&mut self, s: &str) -> fmt::Result {
//            self.write_bytes(s.as_bytes())
//        }
//    }
//}

// ---------------------------------------------------------------------- End Logging on Host Computer ---------------------------------------------------------------------

use crate::board::BoardTrait;
use crate::errors;
use crate::packets;
use crate::params::Params;

//#[cfg(feature = "nucleo")]
//pub fn synch_at(slot_rate: Duration) -> Instant {
//let dt = slot_rate.as_micros();
//let now = Instant::now().as_micros();
//Instant::from_micros((now / dt + 1u64) * dt)
//}

//#[cfg(feature = "nucleo")]
//pub fn synch_at_slot(slot_rate: Duration) -> Instant {
//let dt = slot_rate.as_micros();
//let now = Instant::now().as_micros();
//Instant::from_micros((now / dt + 1u64) * dt)
//}

//#[cfg(feature = "nucleo")]
//pub fn current_slot(timestamp: Instant, sample_period: Duration, slot_period: Duration) -> u64 {
//(timestamp.as_micros() % sample_period.as_micros()) / slot_period.as_micros()
//}

pub struct Sensors {
    pub rosflight_packet_header: Option<packets::RosflightPacketHeader>,
    pub serial_tx_packet: Option<packets::SerialTxPacket>, // TODO where is this used?
    pub serial_rx_packet: Option<packets::SerialRxPacket>,
    pub adc_packet: Option<packets::AdcPacket>,
    pub battery_packet: Option<packets::BatteryPacket>,
    pub imu_packet: Option<packets::ImuPacket>,
    pub baro_packet: Option<packets::BaroPacket>,
    pub pitot_packet: Option<packets::PitotPacket>,
    pub mag_packet: Option<packets::MagPacket>,
    pub rc_packet: Option<packets::RcPacket>,
    pub range_packet: Option<packets::RangePacket>,
    pub gnss_packet: Option<packets::GNSSPacket>,
    pub attitude_packet: Option<packets::AttitudePacket>,
    pub fcu_orientation: Option<Quaternion<f32>>,
}

impl Sensors {
    pub fn new() -> Self {
        Self {
            rosflight_packet_header: None, // <-- when we're done pulling this value out via ownership, the estimator can replace this with "none" so that everyone else knows it's been processed... I could even have a check when we read in from the board to see: if we're not "None" here, it means the previous value wasn't processed and we can decide what to do.
            serial_tx_packet: None,
            serial_rx_packet: None,
            adc_packet: None,
            battery_packet: None,
            imu_packet: None,
            baro_packet: None,
            pitot_packet: None,
            mag_packet: None,
            rc_packet: None,
            range_packet: None,
            gnss_packet: None,
            attitude_packet: None,
            fcu_orientation: None,
        }
    }

    pub fn init_imu(&mut self, params: &Params) {
        let roll = params.get_fc_roll();
        let pitch = params.get_fc_pitch();
        let yaw = params.get_fc_yaw();
        // self.fcu_orientation = Some(Quaternion::from_(roll, pitch, yaw));
    }

    pub fn start_imu_calibration() {}

    pub fn start_gyro_calibration() {}

    pub fn start_baro_calibration() {}

    pub fn start_diff_pressure_calibration() {}

    pub fn gyro_calibration_complete() -> bool {
        false
    }

    pub fn update_imu() -> bool {
        false
    }

    pub fn get_filtered_imu() {}

    pub fn update_battery_monitor() {}

    pub fn calibrate_gyro() {}

    pub fn vector_max() {} // move to math library

    pub fn vector_min() {} // move to math library

    pub fn calibrate_accel() {}

    pub fn calibrate_baro() {}

    pub fn calibrate_diff_pressure() {}

    pub fn correct_imu() {}

    pub fn correct_mag() {}

    pub fn correct_baro() {}

    pub fn correct_diff_pressure() {}

    // pub fn update_battery_monitor_multipliers() {}

    pub fn run<B: BoardTrait>(&mut self, board: &mut B) {
        // ------------------------ IMU ------------------------
        match board.imu_read() {
            Some(Ok(imu_data)) => {
                self.imu_packet = Some(imu_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: Accel: ({},{},{}) uT, Gyro: ({}, {}, {}) , Temp: {} C, Seq: {}\n",
                //    self.imu_packet.as_ref().unwrap().accel[0],
                //    self.imu_packet.as_ref().unwrap().accel[1],
                //    self.imu_packet.as_ref().unwrap().accel[2],
                //    self.imu_packet.as_ref().unwrap().gyro[0],
                //    self.imu_packet.as_ref().unwrap().gyro[0],
                //    self.imu_packet.as_ref().unwrap().gyro[0],
                //    self.imu_packet.as_ref().unwrap().temperature,
                //    self.imu_packet.as_ref().unwrap().seq
                //);
                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Accel: ({},{},{}) uT, Gyro: ({}, {}, {}) , Temp: {} C, Seq: {}\n",
                //    self.imu_packet.as_ref().unwrap().accel[0],
                //    self.imu_packet.as_ref().unwrap().accel[1],
                //    self.imu_packet.as_ref().unwrap().accel[2],
                //    self.imu_packet.as_ref().unwrap().gyro[0],
                //    self.imu_packet.as_ref().unwrap().gyro[0],
                //    self.imu_packet.as_ref().unwrap().gyro[0],
                //    self.imu_packet.as_ref().unwrap().temperature,
                //    self.imu_packet.as_ref().unwrap().seq
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: IMU error: {:?}\n", e);
                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: IMU error: {:?}\n", e).unwrap();

                self.imu_packet = None;
            }
            None => {
                self.imu_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: IMU not present.\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: IMU not present.\n")
            }
        }

        // ------------------------ Mag ------------------------
        match board.mag_read() {
            Some(Ok(mag_data)) => {
                self.mag_packet = Some(mag_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: Mag: ({},{},{}) uT, Temp: {} C\n",
                //    self.mag_packet.as_ref().unwrap().flux[0],
                //    self.mag_packet.as_ref().unwrap().flux[1],
                //    self.mag_packet.as_ref().unwrap().flux[2],
                //    self.mag_packet.as_ref().unwrap().temperature
                //);
                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: Mag: ({},{},{}) uT, Temp: {} C\n",
                //    self.mag_packet.as_ref().unwrap().flux[0],
                //    self.mag_packet.as_ref().unwrap().flux[1],
                //    self.mag_packet.as_ref().unwrap().flux[2],
                //    self.mag_packet.as_ref().unwrap().temperature
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Mag error: {:?}\n", e);
                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Mag error: {:?}\n", e).unwrap();

                self.mag_packet = None;
            }
            None => {
                // <-- if we don't have a mag, we can set the packet to None, and then we don't even need extra functions for checking if the mag is present or not...
                self.mag_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Mag not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Mag not present.\n")
            }
        }

        // ------------------------ Baro ------------------------
        match board.baro_read() {
            Some(Ok(baro_data)) => {
                self.baro_packet = Some(baro_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: Baro: {} C, ({}) kPa\n",
                //    self.baro_packet.as_ref().unwrap().pressure,
                //    self.baro_packet.as_ref().unwrap().temperature
                //);
                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: Baro: {} C, ({}) kPa\n",
                //    self.baro_packet.as_ref().unwrap().pressure,
                //    self.baro_packet.as_ref().unwrap().temperature
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Baro error: {:?}\n", e);
                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Baro error: {:?}\n", e).unwrap();

                self.baro_packet = None;
            }
            None => {
                self.baro_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Baro not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Baro not present.\n");
            }
        }

        // ------------------------ Pitot ------------------------
        match board.diff_pressure_read() {
            Some(Ok(pitot_data)) => {
                self.pitot_packet = Some(pitot_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: Pitot: {} C, ({}) kPa\n",
                //    self.pitot_packet.as_ref().unwrap().pressure,
                //    self.pitot_packet.as_ref().unwrap().temperature
                //);

                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: Pitot: {} C, ({}) kPa\n",
                //    self.pitot_packet.as_ref().unwrap().pressure,
                //    self.pitot_packet.as_ref().unwrap().temperature
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Pitot error: {:?}\n", e);

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Pitot error: {:?}\n", e).unwrap();

                self.pitot_packet = None;
            }
            None => {
                self.pitot_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Pitot not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Pitot not present.\n");
            }
        }

        // ------------------------ GNSS ------------------------
        match board.gnss_read() {
            Some(Ok(gnss_data)) => {
                println!("GNSS: {}", gnss_data.header.timestamp);

                self.gnss_packet = Some(gnss_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: GNSS: {} lat, {} lon\n",
                //    self.gnss_packet.as_ref().unwrap().lat,
                //    self.gnss_packet.as_ref().unwrap().lon
                //);

                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: GNSS: {} lat, {} lon\n",
                //    self.gnss_packet.as_ref().unwrap().lat,
                //    self.gnss_packet.as_ref().unwrap().lon
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: GNSS error: {:?}\n", e);

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: GNSS error: {:?}\n", e).unwrap();
            }
            None => {
                self.pitot_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: GNSS not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: GNSS not present.\n");
            }
        }

        // ------------------------ Sonar ------------------------
        match board.sonar_read() {
            Some(Ok(range_data)) => {
                self.range_packet = Some(range_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: Range: {} units, Min Range: {} units, Max Range: {}, Range Type: {}\n",
                //    self.range_packet.as_ref().unwrap().range,
                //    self.range_packet.as_ref().unwrap().min_range,
                //    self.range_packet.as_ref().unwrap().max_range,
                //    match self.range_packet.as_ref().unwrap().range_type {
                //        packets::RangeType::Sonar => 0,
                //        packets::RangeType::Lidar => 1,
                //    }
                //);

                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: Range: {} units, In Range: {} units, Max Range: {}, Range Type: {}\n",
                //    self.range_packet.as_ref().unwrap().range,
                //    self.range_packet.as_ref().unwrap().min_range,
                //    self.range_packet.as_ref().unwrap().max_range,
                //    match self.range_packet.as_ref().unwrap().range_type {
                //        packets::RangeType::Sonar => 0,
                //        packets::RangeType::Lidar => 1,
                //    }
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Range error: {:?}\n", e);

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Range error: {:?}\n", e).unwrap();

                self.range_packet = None;
            }
            None => {
                self.range_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Range not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Range not present.\n");
            }
        }

        // ------------------------ Battery ------------------------
        match board.battery_read() {
            Some(Ok(battery_data)) => {
                self.battery_packet = Some(battery_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: Battery: {} Volts, ({}) Amps\n",
                //    self.battery_packet.as_ref().unwrap().voltage,
                //    self.battery_packet.as_ref().unwrap().current
                //);

                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: Battery: {} Volts, ({}) Amps\n",
                //    self.battery_packet.as_ref().unwrap().voltage,
                //    self.battery_packet.as_ref().unwrap().current
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Battery error: {:?}\n", e);

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Battery error: {:?}\n", e).unwrap();

                self.battery_packet = None;
            }
            None => {
                self.battery_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Battery not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Battery not present.\n");
            }
        }

        // ------------------------ RC ------------------------
        match board.rc_read() {
            Some(Ok(rc_data)) => {
                self.rc_packet = Some(rc_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: RC: {} nchan, Frames/Packet lost({}) \n",
                //    self.rc_packet.as_ref().unwrap().n_chan,
                //    self.rc_packet.as_ref().unwrap().lol,
                //);

                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: RC: {} nchan, Frames/Packet lost({}) \n",
                //    self.rc_packet.as_ref().unwrap().n_chan,
                //    self.rc_packet.as_ref().unwrap().lol,
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: RC error: {:?}\n", e);

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: RC error: {:?}\n", e).unwrap();

                self.rc_packet = None;
            }
            None => {
                self.rc_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Battery not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: RC not present.\n");
            }
        }

        // ------------------------ Attitude ------------------------
        match board.attitude_read() {
            Some(Ok(attitude_data)) => {
                self.attitude_packet = Some(attitude_data);

                //#[cfg(feature = "nucleo")]
                //trace!(
                //    "Sensor: Attitude: ({}, {}, {})\n",
                //    self.attitude_packet.as_ref().unwrap().rate[0],
                //    self.attitude_packet.as_ref().unwrap().rate[1],
                //    self.attitude_packet.as_ref().unwrap().rate[2]
                //);

                //#[cfg(feature = "default")]
                //write!(
                //    &mut writer,
                //    "Sensor: Attitude: ({}, {}, {})\n",
                //    self.attitude_packet.as_ref().unwrap().rate[0],
                //    self.attitude_packet.as_ref().unwrap().rate[1],
                //    self.attitude_packet.as_ref().unwrap().rate[2]
                //)
                //.unwrap();
            }
            Some(Err(e)) => {
                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Attitude error: {:?}\n", e);

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Attitude error: {:?}\n", e).unwrap();

                self.battery_packet = None;
            }
            None => {
                self.battery_packet = None;

                //#[cfg(feature = "default")]
                //write!(&mut writer, "Sensor: Attitude not present\n").unwrap();

                //#[cfg(feature = "nucleo")]
                //trace!("Sensor: Attitude not present.\n");
            }
        }

        // ------------------------ Serial_rx ------------------------
        //match board.serial_rx_read() {
        //    Some(Ok(serial_data)) => {
        //        self.serial_rx_packet = Some(serial_data);

        //#[cfg(feature = "nucleo")]
        //trace!(
        //    "Sensor: Serial_rx: qos {}, ({}) len\n",
        //    match self.serial_rx_packet.as_ref().unwrap().qos {
        //        packets::Qos::High => 0,
        //        packets::Qos::Medium => 1,
        //        packets::Qos::Low => 2,
        //    },
        //    self.serial_rx_packet.as_ref().unwrap().len
        //);

        //#[cfg(feature = "default")]
        //write!(
        //    &mut writer,
        //    "Sensor: Serial_rx: qos {}, ({}) len\n\n",
        //    match self.serial_rx_packet.as_ref().unwrap().qos {
        //        packets::Qos::High => 0,
        //        packets::Qos::Medium => 1,
        //        packets::Qos::Low => 2,
        //    },
        //    self.serial_rx_packet.as_ref().unwrap().len
        //)
        //.unwrap();
        //        }
        //        Some(Err(e)) => {
        //#[cfg(feature = "nucleo")]
        //trace!("Sensor: Serial_rx error: {:?}\n", e);

        //#[cfg(feature = "default")]
        //write!(&mut writer, "Sensor: Serial_rx error: {:?}\n\n", e).unwrap();

        //            self.serial_rx_packet = None;
        //        }
        //        None => {
        //            self.pitot_packet = None;

        //#[cfg(feature = "default")]
        //write!(&mut writer, "Sensor: Serial_rx not present\n").unwrap();

        //#[cfg(feature = "nucleo")]
        //trace!("Sensor: Serial_rx not present.\n");
        //}
        //}

        //#[cfg(feature = "default")]
        //writer.flush().unwrap();
    }
}
