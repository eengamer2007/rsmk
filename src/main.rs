#![no_std]
#![no_main]

#[allow(unused)]
use log::{error, info, log, trace, warn};

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::join::{join, join3, join_array};

use embassy_executor::Spawner;

use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::clocks::ClockConfig;
use embassy_rp::config::Config;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInt};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler as PioInt, Pio};
use embassy_rp::pio_programs::ws2812::*;
use embassy_rp::gpio::Input;

use embassy_usb::{
    class::hid::{HidReaderWriter, ReportId, RequestHandler, State as HidState},
    class::cdc_acm::CdcAcmClass,
    control::OutResponse,
    {Builder, Config as UsbConfig, Handler},
};

use embassy_time::Timer;

use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use {defmt_rtt as _, panic_probe as _};

use smart_leds::RGB8;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // sparkfun pro micro rp2350 has a 12MHz crystal (just like the pico 2)
    let p = embassy_rp::init(Config::new(ClockConfig::crystal(12_000_000)));
    let driver = Driver::new(p.USB, IrqsUsb);

    let mut config = UsbConfig::new(0xc0de, 0xcafe);
    config.manufacturer = Some("me");
    config.product = Some("definitely a keyboard");
    config.serial_number = Some("why is this called a serial number");
    // max 100mA of power
    config.max_power = 100;
    // somehow also related to speed?
    config.max_packet_size_0 = 64;

    // buffers for building the descriptors
    let mut config_desc = [0; 256];
    let mut bos_desc = [0; 256];
    // microsoft OS descriptor?
    let mut msos_desc = [0; 256];
    let mut control_buf = [0; 64];
    let mut recv_handler = RecvHandler {};
    let mut device_handler = DevHandler::new();

    let mut state = HidState::new();

    let mut status = embassy_usb::class::cdc_acm::State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_desc,
        &mut bos_desc,
        &mut msos_desc,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };

    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);

    //let serial = CdcAcmClass::new(&mut builder, &mut status, 64);
    //spawner.must_spawn(logger_task(serial));

    let mut usb = builder.build();

    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();

    //log::info!("Blinky example started");

    let in_fut = async {

        let mut signal_pin = Input::new(p.PIN_29, embassy_rp::gpio::Pull::None);
        signal_pin.set_schmitt(true);

        loop {
            info!("Waiting for HIGH on pin 16");
            signal_pin.wait_for_high().await;
            info!("HIGH DETECTED");
            // Create a report with the A key pressed. (no shift modifier)
            let report = KeyboardReport {
                keycodes: [4, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            // Send the report.
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
            signal_pin.wait_for_low().await;
            info!("LOW DETECTED");
            let report = KeyboardReport {
                keycodes: [0, 0, 0, 0, 0, 0],
                leds: 0,
                modifier: 0,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };
        }
    };

    let out_fut = async {
        reader.run(false, &mut recv_handler).await;
    };

    /*let led = async {
        let Pio {
            mut common, sm0, ..
        } = Pio::new(p.PIO0, IrqsPio);

        let mut data = [RGB8::new(0, 0, 0); 1];

        let program = PioWs2812Program::new(&mut common);
        let mut led = PioWs2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_25, &program);

        loop {
            log::info!("LED ON");
            data = data.map(|_| RGB8::new(0x05, 0x05, 0x05));
            led.write(&data).await;
            Timer::after_millis(1000).await;

            log::info!("LED OFF");
            data = data.map(|_| RGB8::new(0x00, 0x00, 0x00));
            led.write(&data).await;
            Timer::after_millis(1000).await;
        }
    };*/

    /*let logger = async {
        embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, serial).await;
    };*/

    //join_array([usb_fut, /*logger,*/ in_fut, out_fut, /*led*/]).await;
    join3(usb_fut, in_fut, out_fut).await;
}

struct RecvHandler {}

impl RequestHandler for RecvHandler {
    fn get_report(&mut self, id: ReportId, _buf: &mut [u8]) -> Option<usize> {
        info!("Get report for {:?}", id);
        None
    }

    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        info!("Set report for {:?}: {:?}", id, data);
        OutResponse::Accepted
    }

    fn set_idle_ms(&mut self, id: Option<ReportId>, dur: u32) {
        info!("Set idle rate for {:?} to {:?}", id, dur);
    }

    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        info!("Get idle rate for {:?}", id);
        None
    }
}

struct DevHandler {
    configured: AtomicBool,
}

impl DevHandler {
    fn new() -> Self {
        DevHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for DevHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured
            .store(false, core::sync::atomic::Ordering::Relaxed);
        if enabled {
            info!("enabled");
        } else {
            info!("disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}
/*
#[embassy_executor::task]
async fn logger_task(class: CdcAcmClass<'static, dyn UsbDriver<'static, USB>>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class);
}
*/

bind_interrupts!(struct IrqsUsb {
    USBCTRL_IRQ => UsbInt<USB>;
});

bind_interrupts!(struct IrqsPio {
    PIO0_IRQ_0 => PioInt<PIO0>;
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
