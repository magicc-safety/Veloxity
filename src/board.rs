

enum RCType {
    RC_TYPE_PPM,
    RC_TYPE_SBUS
}

/*
TODO: Should encode the priority of the packet, with 0 being highest.
 */
enum QOS {
}

pub trait Board {

    /*
    TODO:
        * Check which functions actually need `&mut self` vs just passing &self
        * Check input types. Can we encode anything in Enums?
        * Check return types. Can we encode anything in Enums? For example, change booleans to enums
     */

    // Setup
    fn init_board(&mut self);
    fn board_reset(&mut self, bootloader: bool);

    // Clock
    fn clock_millis(&self) -> u32;
    fn clock_micros(&self) -> u64;
    fn clock_delay(&self, milliseconds: u32);

    // Serial
    fn serial_init(&self, baud_rate: u32, dev: u32);
    fn serial_write(&self, src: &[u8], qos: u8);
    fn serial_bytes_available(&self) -> u16;
    fn serial_read(&self) -> u8;
    fn serial_flush(&mut self);

    // Sensors
    fn sensors_init(&mut self);
    fn num_sensor_errors(&self) -> u16;

    // IMU
    fn imu_has_new_data(&self) -> bool;
    fn imu_read(&self, accel: &mut [f32; 3], temperature: &mut f32, gyro: &mut [f32; 3], time: &mut u64) -> bool;
    fn imu_not_responding_error(&mut self);

    // Mag
    fn mag_present(&self) -> bool;
    fn mag_has_new_data(&self) -> bool;
    fn mag_read(&self, mag: &mut [f32; 3]) -> bool;

    // Baro
    fn baro_present(&self) -> bool;
    fn baro_has_new_data(&self) -> bool;
    fn baro_read(&self, pressure: &mut f32, temperature: &mut f32) -> bool;

    // Pitot
    fn diff_pressure_present(&self) -> bool;
    fn diff_pressure_has_new_data(&self) -> bool;
    fn diff_pressure_read(&self, diff_pressure: &mut f32, temperature: &mut f32) -> bool;

    // Sonar
    fn sonar_present(&self) -> bool;
    fn sonar_has_new_data(&self) -> bool;
    fn sonar_read(&self, range: &mut f32) -> bool;

    // GPS
    fn gnss_present(&self) -> bool;
    fn gnss_has_new_data(&self) -> bool;
    // fn gnss_read(&self, gnss: &mut GNSSData, gnss_full: &mut GNSSFull) -> bool;

    // Battery
    fn battery_present(&self) -> bool;
    fn battery_has_new_data(&self) -> bool;
    fn battery_read(&self, voltage: &mut f32, current: &mut f32) -> bool;
    fn battery_voltage_set_multiplier(&mut self, multiplier: f64);
    fn battery_current_set_multiplier(&mut self, multiplier: f64);

    // RC
    // fn rc_init(&mut self, rc_type: RcType);
    fn rc_lost(&self) -> bool;
    fn rc_has_new_data(&self) -> bool;
    fn rc_read(&self, chan: u8) -> f32;

    // PWM
    fn pwm_init(&mut self, refresh_rate: u32, idle_pwm: u16);
    fn pwm_init_multi(&mut self, rate: &[f32], channels: u32);
    fn pwm_disable(&mut self);
    fn pwm_write(&mut self, channel: u8, value: f32);
    fn pwm_write_multi(&mut self, value: &[f32], channels: u32);

    // Non-volatile memory
    fn memory_init(&mut self);
    fn memory_read(&self, dest: &mut [u8]) -> bool;
    fn memory_write(&mut self, src: &[u8]) -> bool;

    // LEDs
    fn led0_on(&mut self);
    fn led0_off(&mut self);
    fn led0_toggle(&mut self);

    fn led1_on(&mut self);
    fn led1_off(&mut self);
    fn led1_toggle(&mut self);

    // Backup memory
    fn backup_memory_init(&mut self);
    fn backup_memory_read(&self, dest: &mut [u8]) -> bool;
    fn backup_memory_write(&mut self, src: &[u8]);
    fn backup_memory_clear(&mut self, len: usize);

}