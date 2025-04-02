use crate::{board::Board, comm_manager, params};

// only necessary for stm32 architecure
#[cfg(feature = "nucleo")]
use defmt::*;

pub struct ROSFlight<B: Board> {
    loop_time_us: u32,
    /* TODO: Is `Box<>` the best way to do this? Ensures that we use the Board trait, but requires
    heap allocation */
    board: B,
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

        // simulate sensor input and reading!!! <-- ultimately replace this loop with a sensors module!!!
        loop {
            if self.board.baro_has_new_data() {
                let mut pressure: f32 = 0.0;
                let mut temperature: f32 = 0.0;
                let baro_data = self.board.baro_read(&mut pressure, &mut temperature);

                #[cfg(feature = "nucleo")]
                defmt::trace!("Baro: {} C, ({}) kPa\n",
                    pressure,
                    temperature);

                #[cfg(feature = "nucleo")]
                defmt::trace!("Sin test: {}\n", micro_algebra::mathlib::sin(0.5));
            }

            if self.board.mag_has_new_data() {
                let mut data = [0.0; 3];
                let mut temperature: f32 = 0.0;
                let mag_data = self.board.mag_read(&mut data, &mut temperature);

                #[cfg(feature = "nucleo")]
                defmt::trace!("Mag: ({},{},{}) uT, Temp: {} C\n",
                    data[0],
                    data[1],
                    data[2],
                    temperature,
                );
            }
        }
    }
}