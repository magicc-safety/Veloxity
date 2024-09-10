
use crate::board::Board;

pub struct ROSFlight<B: Board> {
    loop_time_us: u32,
    /* TODO: Is `Box<>` the best way to do this? Ensures that we use the Board trait, but requires
    heap allocation */
    board: B,
}

impl<B: Board> ROSFlight<B> {
    pub fn init(
        _loop_time_us: u32,
        _board: B,
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