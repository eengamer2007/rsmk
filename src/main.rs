#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::config::Config;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, gpio, pio_programs::ws2812, pio};
use embassy_time::{block_for, Timer, Duration};
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Config::new(ClockConfig::crystal(12*1000*1000)));
    let driver = Driver::new(p.USB, Irqs);
    spawner.must_spawn(logger_task(driver));
    //Gives a pause for the host to connect to the serial port and start viewing logs
    Timer::after_millis(2000).await;
    log::info!("Blinky example started");

    let mut led = Output::new(p.PIN_25, Level::Low);
    led.set_slew_rate(gpio::SlewRate::Fast);
    loop {
        log::info!("LED ON");
        //led.set_high();
        send_data(&mut led, [0xFF, 0xFF, 0xFF]).await;
        Timer::after_millis(1000).await;

        reset(&mut led).await;

        log::info!("LED OFF");
        //led.set_low();
        send_data(&mut led, [0, 0, 0]).await;
        Timer::after_millis(1000).await;

        reset(&mut led).await;
    }
}

async fn send_data<'a>(pin: &mut Output<'a>, data: [u8;3]) {
    for i in 0..3 {
        let mut d = data[i];
        for _ in 0..8 {
            pin.set_high();
            if d & 1 == 0 {
                block_for(Duration::from_nanos(220));
            } else {
                block_for(Duration::from_nanos(580));
            }
            pin.set_low();
            block_for(Duration::from_nanos(400));
            d >>= 1;
        }
    }
    pin.set_high();
}

async fn reset<'a>(pin: &mut Output<'a>) {
    pin.set_low();
    Timer::after_micros(300).await;
    pin.set_high();
}


#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Program metadata for `picotool info`.
// This isn't needed, but it's recomended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Rust Mechanical Keyboard (rsmk)"),
    embassy_rp::binary_info::rp_program_description!(
        c"Firmware for (split) mechanical keyboards written in rust"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];
