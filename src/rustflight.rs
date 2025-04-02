use crate::{board::Board, comm_manager, params, sensors};

// only necessary for stm32 architecure
#[cfg(feature = "nucleo")]
use defmt::*;

pub struct ROSFlight<B: Board> {
    loop_time_us: u32,
    /* TODO: Is `Box<>` the best way to do this? Ensures that we use the Board trait, but requires
    heap allocation */
    
    #[cfg(feature = "nucleo")]
    board: B, // <-- TODO remove public access to board when testing is done!!!
    
    #[cfg(feature = "default")]
    pub board: B, // <-- TODO remove public access to board when testing is done!!!
    
}

impl<B: Board> ROSFlight<B> {
    pub fn init(_loop_time_us: u32, _board: B) -> Self {
        Self {
            loop_time_us: _loop_time_us,
            board: _board
        }
    }
    
    pub fn run(&mut self) {
        let start = self.board.clock_micros();

        // High level: Create the variables we'll need
        let mut p = params::Params::new();
        let mut mavlink = crate::comm_manager::mavlink::Mavlink::new();
        let mut comm_manager = crate::comm_manager::CommManager::new(mavlink);
        let mut sensors = sensors::Sensors::new();

        // simulate sensor input and reading!!! <-- ultimately replace this loop with a sensors module!!!
        loop {
            sensors.run(&self.board);
        }
    }
}