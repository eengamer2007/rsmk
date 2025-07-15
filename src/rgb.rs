use embassy_rp::{
    Peri,
    dma::Channel,
    peripherals::PIO1,
    pio::{Common, Instance, PioPin, StateMachine},
    pio_programs::ws2812::{PioWs2812, PioWs2812Program},
};
use embassy_time::{Instant, Timer};
use smart_leds::RGB8;

use crate::{LED_COUNT, RGB_STATE};

pub enum RgbState {
    Reset,
    Start,
}

pub fn rgb_setup<'a, const SM: usize, PIO: Instance>(
    pio: &mut Common<'a, PIO>,
    sm: StateMachine<'a, PIO, SM>,
    dma: Peri<'a, impl Channel>,
    pin: Peri<'a, impl PioPin>,
) -> PioWs2812<'a, PIO, SM, LED_COUNT> {
    let prg = PioWs2812Program::new(pio);
    let led: PioWs2812<'_, PIO, SM, LED_COUNT> = PioWs2812::new(pio, sm, dma, pin, &prg);
    led
}

#[embassy_executor::task]
pub async fn rgb_runner(mut led: PioWs2812<'static, PIO1, 0, LED_COUNT>) {
    let mut buf = [RGB8::new(255, 0, 0); LED_COUNT];

    let mut state = RgbState::Reset;

    // program will break to here if there is a new signal
    'signal: loop {
        if let Some(new_state) = RGB_STATE.try_take() {
            state = new_state;
        }
        match state {
            RgbState::Reset => {
                let buf = [RGB8::new(255, 255, 255); LED_COUNT];
                led.write(&buf).await;
                RGB_STATE.wait().await;
                //RGB_STATE.signal(RgbState::Start);
            }
            RgbState::Start => {}
        }

        for i in 0..LED_COUNT {
            use core::f32::consts::PI;
            use core::intrinsics::sinf32;
            unsafe {
                buf[i] = RGB8::new(
                    (sinf32(i as f32 + 0. * ((2. / 3.) * PI) + Instant::now().as_micros() as f32)
                        * 255.) as u8,
                    (sinf32(i as f32 + 1. * ((2. / 3.) * PI) + Instant::now().as_micros() as f32)
                        * 255.) as u8,
                    (sinf32(i as f32 + 2. * ((2. / 3.) * PI) + Instant::now().as_micros() as f32)
                        * 255.) as u8,
                )
            };
        }

        loop {
            // amount of seconds
            let mut time = Instant::now().as_micros() as f32 / 1000000.;
            time *= 1.5;
            time = -time;
            for i in 0..LED_COUNT {
                buf[i] = RGB8::new(color(i, 0., time), color(i, 1., time), color(i, 2., time))
            }

            //buf = buf.map(|v| RGB8::new(v.g, v.b, v.r));
            led.write(&buf).await;
            if RGB_STATE.signaled() {
                break 'signal;
            }
            Timer::after_millis(5).await;
        }
    }
}

fn color(idx: usize, phase: f32, time: f32) -> u8 {
    use core::f32::consts::PI;
    use core::intrinsics::sinf32;

    (((unsafe {
        sinf32((phase * (2. / 3.) * PI) + time + ((idx as f32 / LED_COUNT as f32) * 2. * PI))
    } + 1.)
        / 2.)
        * 255.) as u8
}
