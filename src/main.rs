#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

//use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::*;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Flex, Input, Pull};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
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
        //let mut r0 = Input::new(p.PIN_29, Pull::Up);

        // Enable the schmitt trigger to slightly debounce.
        //r0.set_schmitt(true);

        //let mut row = [r0];

        let mut columns = column_pins!(p.PIN_9, p.PIN_26, p.PIN_22, p.PIN_20, p.PIN_23, p.PIN_21);

        //let column: &mut [Flex] = &mut [];
        //let pins: &mut [ &impl Pin ] = &mut [];
        columns.iter_mut().for_each(|x| {
            x.set_low();
            x.set_as_input();
        });

        //let mut rows = [Input::new(p.PIN_29, Pull::Up), Input::new(p.PIN_27, Pull::Up)];
        let mut rows = row_pins!(p.PIN_29, p.PIN_27, p.PIN_6, p.PIN_7, p.PIN_8);
        rows.iter_mut().for_each(|r| r.set_schmitt(true));

        let mut out = [[false; 6]; 5];

        #[rustfmt::skip]
        let keymap: [[[Key; 6]; 5]; 2] = [
            [
                [Key::None,    Key::char('1'), Key::char('2'), Key::char('3'), Key::char('4'),               Key::char('5')],
                [Key::TAB,     Key::char('q'), Key::char('w'), Key::char('e'), Key::char('r'),               Key::char('t')],
                [Key::ESC,     Key::char('a'), Key::char('s'), Key::char('d'), Key::char('f'),               Key::char('g')],
                [Key::L_SHIFT, Key::char('z'), Key::char('x'), Key::char('c'), Key::char('v'),               Key::char('b')],
                [Key::None,    Key::None,      Key::None,      Key::None,      Key::Special(Special::Lower), Key::char(' ')],
            ],
            [
                [Key::REBOOT, Key::None, Key::None, Key::None, Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None, Key::None, Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None, Key::None, Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None, Key::None, Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None, Key::None, Key::Special(Special::Lower), Key::None],
            ]
        ];

        let specials: Option<(usize, usize, Special)> = keymap[0]
            .iter()
            .enumerate()
            .map(|(x, r)| r.iter().enumerate().map(move |(y, v)| (x, y, v)))
            .flatten()
            .filter_map(|v|  { let (x,y, key) = v;  if let Key::Special(key) = key { Some((x,y, (*key).clone())) } else { None } })
            .next();

        info!("{:?}", specials);

        loop {
            for (x, column) in columns.iter_mut().enumerate() {
                core::hint::black_box(column.set_as_output());
                for (y, row) in rows.iter().enumerate() {
                    out[y][x] = row.is_low();
                }
                //column.set_as_input();
                core::hint::black_box(column.set_as_input());
            }

            let mut special = 0;
            if let Some((x,y,ref key)) = specials {
                if out[x][y] == true {
                    match key {
                        Special::Lower => special = 1,
                        //Special::Reboot => reboot(),
                        _ => {}
                    }
                }
            }

            let mut keycodes: [u8; 6] = [0; 6];
            let mut idx = 0;
            let mut modifier = 0;
            for (x, r) in out.iter().enumerate() {
                for (y, v) in r.iter().enumerate() {
                    if *v {
                        match keymap[special][x][y] {
                            Key::Key(val) => {
                                if idx < 6 {
                                    keycodes[idx] = val;
                                    idx += 1;
                                } else {
                                    keycodes = [0x1; 6];
                                }
                            }
                            Key::Modifier(mod_key) => modifier = modifier | mod_key,
                            Key::Special(s) => match s {
                                Special::Reboot => reboot(),
                                _ => {}
                            },
                            _ => info!("unknown key {},{}", x, y),
                        }
                    }
                }
            }
            //info!("{:b}", modifier);
            let report = KeyboardReport {
                keycodes,
                leds: 0,
                modifier,
                reserved: 0,
            };
            match writer.write_serialize(&report).await {
                Ok(()) => {}
                Err(e) => warn!("Failed to send report: {:?}", e),
            };

            Timer::after_micros(10).await;
        }
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

#[allow(dead_code)]
#[derive(Clone)]
enum Key {
    Modifier(u8),
    Key(u8),
    Special(Special),
    None,
}

#[allow(dead_code)]
impl Key {
    const L_CTRL: Self = Key::Modifier(1 << 0);
    const L_SHIFT: Self = Key::Modifier(1 << 1);
    const L_ALT: Self = Key::Modifier(1 << 2);
    const L_SUPER: Self = Key::Modifier(1 << 3);
    const R_CTRL: Self = Key::Modifier(1 << 4);
    const R_SHIFT: Self = Key::Modifier(1 << 5);
    const R_ALT: Self = Key::Modifier(1 << 6);
    const R_SUPER: Self = Key::Modifier(1 << 7);

    const REBOOT: Self = Key::Special(Special::Reboot);

    const ESC: Self = Key::Key(0x29);
    const TAB: Self = Key::Key(0x2B);

    const fn char(c: char) -> Self {
        let num = c as u8;

        if num >= b'a' && num <= b'z' {
            return Self::Key((num - b'a') + 4);
        }

        // ascii starts with 0
        // but keycodes have 0 at the end
        if c == '0' {
            return Self::Key(39);
        }

        if num >= b'1' && num <= b'9' {
            return Self::Key((num - b'1') + 30);
        }

        if c == ' ' {
            return Key::Key(44);
        }

        Key::None
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Special {
    Lower,
    Upper,
    Reboot,
}

#[macro_export]
macro_rules! column_pins {
    ($($x:expr), *) => {{
        const SIZE: usize = count!($($x)*);
        let out: [Flex; SIZE] = [$(Flex::new($x)),*];

        out
    }};
}

#[macro_export]
macro_rules! row_pins {
    ($($x:expr), *) => {{
        const SIZE: usize = count!($($x)*);
        let out: [Input; SIZE] = [$(Input::new($x, Pull::Up)),*];

        out
    }};
}

#[macro_export]
macro_rules! count {
    () => (0usize);
    ( $x:tt $($xs:tt)* ) => (1usize + count!($($xs)*));
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
            reboot();
        }
    }
}

fn reboot () {
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
