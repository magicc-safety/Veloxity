

enum RCType {
    RC_TYPE_PPM,
    RC_TYPE_SBUS
}

/*
TODO: Should encode the priority of the packet, with 0 being highest.
 */
enum QOS {
}

trait Board {

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
    // TODO: The `src` arg is a `const uint8_t *` type. Not sure yet what type this should be in Rust.
    fn serial_write(&self, src: &u8, len: usize, qos: QOS);
    fn serial_bytes_available(&self) -> u16;
    fn serial_read(&self) -> u8;
    fn serial_flush(&mut self);

    // Sensors
    fn sensors_init(&mut self);
    fn num_sensors_errors(&self) -> u16;

    // IMU
    fn imu_has_new_data(&self) -> bool;
    fn imu_read(&self, accel: &[f32; 3], temperature: &f32, gyro: &[f32; 3], time: &u64) -> bool;
    fn imu_not_responding_error(&mut self);

    // Mag
    fn mag_present(&self) -> bool;
    fn mag_has_new_data(&self) -> bool;
    fn mag_read(&self, mag: &[f32; 3]) -> bool;

    // Baro
    fn baro_present(&self) -> bool;
    fn baro_has_new_data(&self) -> bool;
    fn baro_read(&self, pressure: &f32, temperature: &f32) -> bool;

    // Pitot
    fn diff_pressure_present(&self) -> bool;
    fn diff_pressure_has_new_data(&self) -> bool;
    fn diff_pressure_read(diff_pressure: &f32, temperature: &f32) -> bool;

    // Sonar
    fn sonar_present(&self) -> bool;
    fn sonar_has_new_data(&self) -> bool;
    fn sonar_read(range: &f32) -> bool;

    // GPS
    fn gnss_present(&self) -> bool;
    fn gnss_has_new_data(&self) -> bool;
    // fn gnss_read(&self, gnss: GNSSData, gnss_full: GNSSFull) -> bool; // TODO: Implement structs

    // Battery
    fn battery_present(&self) -> bool;
    fn battery_has_new_data(&self) -> bool;
    fn battery_read(&self, voltage: f32, current: f32) -> bool;
    fn battery_voltage_set_multiplier(&self, multiplier: f64) -> bool;
    fn battery_current_set_multiplier(&self, multiplier: f64) -> bool;

    // RC
    virtual void rc_init(rc_type_t rc_type) = 0;
    virtual bool rc_lost() = 0;
    virtual bool rc_has_new_data() = 0;
    virtual float rc_read(uint8_t chan) = 0;

    // PWM
    virtual void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) = 0;
    virtual void pwm_init_multi(const float * rate, uint32_t channels) = 0;
    virtual void pwm_disable() = 0;
    virtual void pwm_write(uint8_t channel, float value) = 0;
    virtual void pwm_write_multi(float * value, uint32_t channels) = 0;

    // non-volatile memory
    virtual void memory_init() = 0;
    virtual bool memory_read(void * dest, size_t len) = 0;
    virtual bool memory_write(const void * src, size_t len) = 0;

    // LEDs
    virtual void led0_on() = 0;
    virtual void led0_off() = 0;
    virtual void led0_toggle() = 0;

    virtual void led1_on() = 0;
    virtual void led1_off() = 0;
    virtual void led1_toggle() = 0;

    // Backup memory
    virtual void backup_memory_init() = 0;
    virtual bool backup_memory_read(void * dest, size_t len) = 0;
    virtual void backup_memory_write(const void * src, size_t len) = 0;
    virtual void backup_memory_clear(size_t len) = 0;
}