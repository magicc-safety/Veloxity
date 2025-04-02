
use crate::params::Params;
use crate::board::Board;
use crate::sensors;

pub struct DummyBoard;

impl Board for DummyBoard {
    fn init_board(&mut self) {
        // println!("Dummy board initialized.");
    }

    fn board_reset(&mut self, bootloader: bool) {
        // println!("Board reset.");
    }

    fn clock_millis(&self) -> u32 {
        0
    }

    fn clock_micros(&self) -> u64 {
        0
    }

    fn clock_delay(&self, milliseconds: u32) {
        // println!("Delaying for {} milliseconds.", milliseconds);
    }

    fn serial_init(&self, baud_rate: u32, dev: u32) {
        // println!("Serial initialized with baud rate {} and device {}.", baud_rate, dev);
    }

    fn serial_write(&self, src: &[u8], qos: u8) {
        // println!("Serial write called with qos {}.", qos);
    }

    fn serial_bytes_available(&self) -> u16 {
        0
    }

    fn serial_read(&self) -> u8 {
        0
    }

    fn serial_flush(&mut self) {
        // println!("Serial flushed.");
    }

    fn sensors_init(&mut self) {
        // println!("Sensors initialized.");
    }

    fn num_sensor_errors(&self) -> u16 {
        0
    }

    fn imu_has_new_data(&self) -> bool {
        false
    }

    fn imu_read(&self, accel: &mut [f32; 3], temperature: &mut f32, gyro: &mut [f32; 3], time: &mut u64) -> bool {
        false
    }

    fn imu_not_responding_error(&mut self) {
        // println!("IMU not responding error.");
    }

    // fn mag_present(&self) -> bool {
    //     false
    // }

    // fn mag_has_new_data(&mut self) -> bool {
    //     false
    // }

    // fn mag_read(&self, flux: &mut [f32; 3], temperature: &mut f32) -> bool {
    //     false
    // }

    fn mag_read(&self) -> Option<Result<sensors::MagPacket, sensors::SensorError>> {
        None 
    }
    
    // fn baro_present(&self) -> bool {
    //     false
    // }

    // fn baro_has_new_data(&mut self) -> bool {
    //     false
    // }

    // fn baro_read(&self, pressure: &mut f32, temperature: &mut f32) -> bool {
    //     false
    // }
    
    fn baro_read(&self) -> Option<Result<sensors::BaroPacket, sensors::SensorError>> {
        None
    }

    fn diff_pressure_present(&self) -> bool {
        false
    }

    fn diff_pressure_has_new_data(&self) -> bool {
        false
    }

    fn diff_pressure_read(&self, diff_pressure: &mut f32, temperature: &mut f32) -> bool {
        false
    }

    fn sonar_present(&self) -> bool {
        false
    }

    fn sonar_has_new_data(&self) -> bool {
        false
    }

    fn sonar_read(&self, range: &mut f32) -> bool {
        false
    }

    fn gnss_present(&self) -> bool {
        false
    }

    fn gnss_has_new_data(&self) -> bool {
        false
    }

    fn battery_present(&self) -> bool {
        false
    }

    fn battery_has_new_data(&self) -> bool {
        false
    }

    fn battery_read(&self, voltage: &mut f32, current: &mut f32) -> bool {
        false
    }

    fn battery_voltage_set_multiplier(&mut self, multiplier: f64) {
        // println!("Battery voltage set multiplier called with {}.", multiplier);
    }

    fn battery_current_set_multiplier(&mut self, multiplier: f64) {
        // println!("Battery current set multiplier called with {}.", multiplier);
    }

    fn rc_lost(&self) -> bool {
        false
    }

    fn rc_has_new_data(&self) -> bool {
        false
    }

    fn rc_read(&self, chan: u8) -> f32 {
        0.0
    }

    fn pwm_init(&mut self, refresh_rate: u32, idle_pwm: u16) {
        // println!(
            // "PWM initialized with refresh rate {} and idle PWM {}.",
            // refresh_rate, idle_pwm
        // );
    }

    fn pwm_init_multi(&mut self, rate: &[f32], channels: u32) {
        // println!(
            // "PWM initialized with refresh rate {} and channels {}.",
            // rate[0], channels
        // );
    }

    fn pwm_disable(&mut self) {
        // println!("PWM disabled.");
    }

    fn pwm_write(&mut self, channel: u8, value: f32) {
        // println!("PWM write called with channel {} and value {}.", channel, value);
    }

    fn pwm_write_multi(&mut self, value: &[f32], channels: u32) {
        // println!(
            // "PWM write multi called with value {} and channels {}.",
            // value[0], channels
        // );
    }

    fn memory_init(&mut self) {
        // println!("Memory initialized.");
    }

    fn memory_read(&self, dest: &mut Params) -> bool {
        false
    }

    fn memory_write(&mut self, src: &Params) -> bool {
        false
    }

    fn led0_on(&mut self) {
        // println!("LED 0 on.");
    }

    fn led0_off(&mut self) {
        // println!("LED 0 off.");
    }

    fn led0_toggle(&mut self) {
        // println!("LED 0 toggled.");
    }

    fn led1_on(&mut self) {
        // println!("LED 1 on.");
    }

    fn led1_off(&mut self) {
        // println!("LED 1 off.");
    }

    fn led1_toggle(&mut self) {
        // println!("LED 1 toggled.");
    }

    fn backup_memory_init(&mut self) {
        // println!("Backup memory initialized.");
    }

    fn backup_memory_read(&self, dest: &mut [u8]) -> bool {
        false
    }

    fn backup_memory_write(&mut self, src: &[u8]) {
        // println!("Backup memory write called.");
    }

    fn backup_memory_clear(&mut self, len: usize) {
        // println!("Backup memory cleared with length {}.", len);
    }
}
