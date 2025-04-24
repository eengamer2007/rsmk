#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

//use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::*;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Flex, Input, Pin, Pull};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};
use embassy_usb_logger::ReceiverHandler;
use log::{info, warn};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use {defmt_rtt as _, panic_probe as _};

#[allow(
    unused,
    unsafe_op_in_unsafe_fn,
    unexpected_cfgs,
    non_upper_case_globals
)]
mod bootrom;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

//const ROW_PINS: [u8; 5] = [19, 17, 9, 10, 11];
//const COLUMN_PINS: [_; 6] = [p.PIN_9, p.PIN_26, p.PIN_22, p.PIN_20, p.PIN_23, p.PIN21];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Me.");
    config.product = Some("most definitely a keyboard");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut request_handler = MyRequestHandler {};
    let mut device_handler = MyDeviceHandler::new();

    // needs to be made before the builder
    let mut state = State::new();

    let mut s_state = embassy_usb::class::cdc_acm::State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, &mut state, config);
    let serial = CdcAcmClass::new(&mut builder, &mut s_state, 64);
    //spawner.must_spawn(logger);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();

    // Do stuff with the class!
    let in_fut = async {
        // Set up the signal pin that will be used to trigger the keyboard.
        let mut r0 = Input::new(p.PIN_29, Pull::Up);

        // Enable the schmitt trigger to slightly debounce.
        r0.set_schmitt(true);

        let mut row = [r0];
 
        let column: &mut [Flex] = &mut [];
        let pins: &mut [ &impl Pin ] = &mut [];
        for pin in (p.PIN_9, p.PIN_26) {
            let mut c = Flex::new(pin);
            c.set_as_output();
            c.set_low();
        }

        let out = [[false; 1]; 1];

        loop {
            for pin in column.iter_mut() {
                
            }
        }

        /*
                loop {
                    info!("Waiting for LOW on pin 29 (A3)");
                    r0.wait_for_low().await;
                    info!("LOW DETECTED");
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
                    r0.wait_for_high().await;
                    info!("HIGH DETECTED");
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
        */
    };

    let out_fut = async {
        reader.run(true, &mut request_handler).await;
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join4(
        usb_fut,
        in_fut,
        out_fut,
        embassy_usb_logger::with_class!(1024, log::LevelFilter::Trace, serial, UsbReceiver),
        //key_scanner,
    )
    .await;
}

//#[embassy_executor::task]
//async fn logger(class: CdcAcmClass<>) {
//    embassy_usb_logger::with_class!(1024, log::LevelFilter::Trace, class).await;
//}

struct UsbReceiver {}

impl ReceiverHandler for UsbReceiver {
    fn new() -> Self {
        UsbReceiver {}
    }

    async fn handle_data(&self, data: &[u8]) -> () {
        info!("recevived: {:?}", data);
        if data.contains(&98) {
            info!("rebooting");
            info!(
                "returned: {}",
                bootrom::reboot(
                    0x0002, /* reboot to bootsel*/
                    1,      /* 1 ms delay */
                    0,      /* don't indicate a gpio because we don't use em */
                    0       /* don't disable anything or mess with LED's */
                )
            );
        }
    }
}

struct MyRequestHandler {}

impl RequestHandler for MyRequestHandler {
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

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
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
