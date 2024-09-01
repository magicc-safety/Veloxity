
pub struct ROSFlight {
    loop_time_us: u32,
}

impl ROSFlight {
    pub fn init(
        _loop_time_us: u32,
    ) -> Self {
        Self {
            loop_time_us: _loop_time_us
        }
    }

    pub fn run(&self) {
        println!("Run function called.")
    }
}