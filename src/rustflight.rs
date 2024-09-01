
use crate::board::Board;

pub struct ROSFlight {
    loop_time_us: u32,
    /* TODO: Is `Box<>` the best way to do this? Ensures that we use the Board trait, but requires
    heap allocation */
    board: Box<dyn Board>,
}

impl ROSFlight {
    pub fn init(
        _loop_time_us: u32,
        _board: Box<dyn Board>,
    ) -> Self {
        Self {
            loop_time_us: _loop_time_us,
            board: _board,
        }
    }

    pub fn run(&self) {
        let start = self.board.clock_micros();
        println!("Run function called.")
    }
}