use fixed::traits::ToFixed;
use rp2350_platform::hal::{
    self as rp, Peri,
    clocks::clk_sys_freq,
    dma,
    gpio::Level,
    pio::{
        Config, Direction as PioDirection, FifoJoin, Instance, LoadedProgram, PioPin,
        ShiftDirection, StateMachine,
    },
};

pub struct PioUartDmaRxProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioUartDmaRxProgram<'d, PIO> {
    pub fn new(common: &mut rp::pio::Common<'d, PIO>) -> Self {
        let prg = pio::pio_asm!(
            r#"
                start:
                    wait 0 pin 0
                    set x, 7    [10]
                rx_bitloop:
                    in pins, 1
                    jmp x-- rx_bitloop [6]
                    jmp pin good_rx_stop

                    irq 4 rel
                    wait 1 pin 0
                    jmp start

                good_rx_stop:
                    in null 24
                    push
            "#
        );

        Self {
            prg: common.load_program(&prg.program),
        }
    }
}

pub struct PioUartDmaRx<'d, PIO: Instance, const SM: usize> {
    sm_rx: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> PioUartDmaRx<'d, PIO, SM> {
    pub fn new(
        baud: u32,
        common: &mut rp::pio::Common<'d, PIO>,
        mut sm_rx: StateMachine<'d, PIO, SM>,
        rx_pin: Peri<'d, impl PioPin>,
        program: &PioUartDmaRxProgram<'d, PIO>,
    ) -> Self {
        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);

        let rx_pin = common.make_pio_pin(rx_pin);
        sm_rx.set_pins(Level::High, &[&rx_pin]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_jmp_pin(&rx_pin);
        sm_rx.set_pin_dirs(PioDirection::In, &[&rx_pin]);

        cfg.clock_divider = (clk_sys_freq() / (8 * baud)).to_fixed();
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;
        sm_rx.set_config(&cfg);
        sm_rx.set_enable(true);

        Self { sm_rx }
    }

    pub async fn read_words_dma(&mut self, dma: &mut dma::Channel<'_>, words: &mut [u32]) {
        self.sm_rx.rx().dma_pull(dma, words, false).await;
    }

    pub fn stalled(&mut self) -> bool {
        self.sm_rx.rx().stalled()
    }
}
