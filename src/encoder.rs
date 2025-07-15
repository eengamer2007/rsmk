use core::sync::atomic::{AtomicI32, Ordering};

use embassy_rp::{
    Peri,
    peripherals::PIO0,
    pio::{Common, Instance, PioPin, StateMachine},
    pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram},
};

pub fn encoder_setup<'a, const SM: usize, PIO: Instance>(
    common: &mut Common<'a, PIO>,
    sm: StateMachine<'a, PIO, SM>,
    ea: Peri<'a, impl PioPin>,
    eb: Peri<'a, impl PioPin>,
) -> PioEncoder<'a, PIO, SM> {
    let prg = PioEncoderProgram::new(common);
    let enc = PioEncoder::new(common, sm, ea, eb, &prg);
    enc
}

#[embassy_executor::task]
pub async fn run_encoder(vol_counter: &'static AtomicI32, mut enc: PioEncoder<'static, PIO0, 0>) {
    loop {
        match enc.read().await {
            Direction::Clockwise => {
                let mut val = vol_counter.load(Ordering::Acquire);

                if val < 0 {
                    val = -1;
                } else {
                    val += -1;
                }

                vol_counter.store(val, Ordering::Release);
            }
            Direction::CounterClockwise => {
                let mut val = vol_counter.load(Ordering::Acquire);

                if val < 0 {
                    val = 1;
                } else {
                    val += 1;
                }

                vol_counter.store(val, Ordering::Release);
            }
        }
    }
}
