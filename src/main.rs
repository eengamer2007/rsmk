#![no_std]
#![no_main]
#![allow(internal_features, static_mut_refs)]
#![feature(core_intrinsics)]

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use cortex_m::asm::delay;

use embassy_executor::Spawner;

use embassy_futures::join::*;

use embassy_rp::gpio::{Flex, Input, Pull};
use embassy_rp::i2c::{SclPin, SdaPin};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{I2C1, PIO0, PIO1, USB};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{Peri, bind_interrupts};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};

use embassy_usb_logger::ReceiverHandler;

use log::{info, warn};

use rgb::{rgb_runner, rgb_setup};

use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use defmt_rtt as _;

use panic_probe as _;

#[allow(
    unused,
    unsafe_op_in_unsafe_fn,
    unexpected_cfgs,
    non_upper_case_globals
)]
mod bootrom;

mod display;
use display::SSD1306;

mod rgb;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

bind_interrupts!(struct I2CIrqs {
    I2C1_IRQ => embassy_rp::i2c::InterruptHandler<I2C1>;
});

bind_interrupts!(struct Pio0Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

bind_interrupts!(struct Pio1Irqs {
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

const LED_COUNT: usize = 29;

static mut CORE1_STACK: Stack<4096> = Stack::new();

static RGB_STATE: Signal<CriticalSectionRawMutex, rgb::RgbState> = Signal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO1, crate::Pio1Irqs);

    RGB_STATE.signal(rgb::RgbState::Reset);

    let led = rgb_setup(&mut common, sm0, p.DMA_CH0, p.PIN_0);
    spawner.spawn(rgb_runner(led)).unwrap();

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
    static mut config_descriptor: [u8; 256] = [0; 256];
    static mut bos_descriptor: [u8; 256] = [0; 256];
    // You can also add a Microsoft OS descriptor.
    static mut msos_descriptor: [u8; 256] = [0; 256];
    static mut control_buf: [u8; 64] = [0; 64];
    let mut request_handler = MyRequestHandler {};
    static mut device_handler: MyDeviceHandler = MyDeviceHandler::new();

    // needs to be made before the builder
    static mut state: State = State::new();

    static mut s_state: embassy_usb::class::cdc_acm::State = embassy_usb::class::cdc_acm::State::new();

    let mut builder = Builder::new(
        driver,
        config,
        unsafe { &mut config_descriptor },
        unsafe { &mut bos_descriptor },
        unsafe { &mut msos_descriptor },
        unsafe { &mut control_buf },
    );

    builder.handler(unsafe { &mut device_handler } );

    // Create classes on the builder.
    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, unsafe { &mut state }, config);
    let serial = CdcAcmClass::new(&mut builder, unsafe { &mut s_state }, 64);
    spawner.spawn(logger(serial)).unwrap();

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let (reader, mut writer) = hid.split();

    let left_right = {
        // left right pin is either connected to ground or to VCC
        // so no pull up or down required
        let left_right_pin = Input::new(p.PIN_28, Pull::None);

        left_right_pin.is_low()
    };

    info!("detected: {}", left_right);

    let vol_counter = AtomicI32::new(0);

    // Do stuff with the class!
    let in_fut = async {
        let mut columns = column_pins!(p.PIN_9, p.PIN_26, p.PIN_22, p.PIN_20, p.PIN_23, p.PIN_21);

        columns.iter_mut().for_each(|x| {
            x.set_low();
            x.set_as_input();
        });

        let mut rows = row_pins!(p.PIN_29, p.PIN_27, p.PIN_6, p.PIN_7, p.PIN_8);
        rows.iter_mut().for_each(|r| r.set_schmitt(true));

        let mut out = [[false; 6]; 5];

        #[rustfmt::skip]
        let keymap: [[[Key; 6]; 5]; 4] = [
            [ // BASE
                [Key::TAB,          Key::char('1'), Key::char('2'), Key::char('3'), Key::char('4'),               Key::char('5')],
                [Key::ESC,          Key::char('q'), Key::char('w'), Key::char('e'), Key::char('r'),               Key::char('t')],
                [Key::L_SHIFT,      Key::char('a'), Key::char('s'), Key::char('d'), Key::char('f'),               Key::char('g')],
                [Key::L_CTRL,       Key::char('z'), Key::char('x'), Key::char('c'), Key::char('v'),               Key::char('b')],
                [Key::None,         Key::MUTE,      Key::L_ALT,     Key::L_SUPER,   Key::Special(Special::Lower), Key::char(' ')],
            ],
            [ // LOWER
                [Key::REBOOT, Key::f(1), Key::f(2),  Key::f(3),  Key::f(4),                    Key::None],
                [Key::None,   Key::f(5), Key::f(6),  Key::f(7),  Key::f(8),                    Key::None],
                [Key::None,   Key::f(9), Key::f(10), Key::f(11), Key::f(12),                   Key::None],
                [Key::None,   Key::None, Key::None,  Key::None,  Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None,  Key::None,  Key::Special(Special::Lower), Key::None],
            ],
            [ // UPPER
                [Key::REBOOT, Key::f(1), Key::f(2),  Key::f(3),  Key::f(4),                    Key::None],
                [Key::None,   Key::f(5), Key::f(6),  Key::f(7),  Key::f(8),                    Key::None],
                [Key::None,   Key::f(9), Key::f(10), Key::f(11), Key::f(12),                   Key::None],
                [Key::None,   Key::None, Key::None,  Key::None,  Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None,  Key::None,  Key::Special(Special::Lower), Key::None],
            ],
            [ // BOTH
                [Key::REBOOT, Key::f(1), Key::f(2),  Key::f(3),  Key::f(4),                    Key::None],
                [Key::None,   Key::f(5), Key::f(6),  Key::f(7),  Key::f(8),                    Key::None],
                [Key::None,   Key::f(9), Key::f(10), Key::f(11), Key::f(12),                   Key::None],
                [Key::None,   Key::None, Key::None,  Key::None,  Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None,  Key::None,  Key::Special(Special::Lower), Key::None],
            ]
        ];

        let specials: Option<(usize, usize, Special)> = keymap[0]
            .iter()
            .enumerate()
            .map(|(x, r)| r.iter().enumerate().map(move |(y, v)| (x, y, v)))
            .flatten()
            .filter_map(|v| {
                let (x, y, key) = v;
                if let Key::Special(key) = key {
                    Some((x, y, (*key).clone()))
                } else {
                    None
                }
            })
            .next();

        info!("{:?}", specials);

        let mut old_report = KeyboardReport::default();

        loop {
            for (x, column) in columns.iter_mut().enumerate() {
                column.set_as_output();
                delay(100);
                for (y, row) in rows.iter().enumerate() {
                    let low = row.is_low();
                    delay(100);
                    out[y][x] = low;
                    if low {
                        info!("trigger: {},{}", x, y);
                    }
                }
                //column.set_as_input();
                column.set_as_input();
                delay(50);
            }
            RGB_STATE.signal(rgb::RgbState::Start);

            let mut special = 0;
            if let Some((x, y, ref key)) = specials {
                if out[x][y] == true {
                    match key {
                        Special::Lower => special |= 1 >> 0,
                        Special::Upper => special |= 1 >> 1,
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
                                Special::Reboot => reboot().await,
                                _ => {}
                            },
                            _ => info!("unknown key {},{}", x, y),
                        }
                    }
                }
            }
            let cnt = vol_counter.load(Ordering::Acquire);
            if cnt > 0 {
                if idx < 6 {
                    keycodes[idx] = 0x80; //Key::INC_SOUND;
                    idx += 1;
                } else {
                    keycodes = [0x1; 6];
                }
                vol_counter.store(cnt - 1, Ordering::Release);
            } else if cnt < 0 {
                if idx < 6 {
                    keycodes[idx] = 0x81; //Key::DEC_SOUND;
                    idx += 1;
                } else {
                    keycodes = [0x1; 6];
                }
                vol_counter.store(cnt + 1, Ordering::Release);
            }
            let _ = idx;

            //info!("{:b}", modifier);
            let report = KeyboardReport {
                keycodes,
                leds: 0,
                modifier,
                reserved: 0,
            };
            if report != old_report {
                //writer.write()
                match writer.write_serialize(&report).await {
                    Ok(()) => {}
                    Err(e) => warn!("Failed to send report: {:?}", e),
                };
                old_report = report;
            }

            embassy_futures::yield_now().await;
        }
    };

    let out_fut = async {
        reader.run(true, &mut request_handler).await;
    };

    let encoder = async {
        let Pio {
            mut common, sm0, ..
        } = Pio::new(p.PIO0, Pio0Irqs);

        let prg = PioEncoderProgram::new(&mut common);
        let mut enc = PioEncoder::new(&mut common, sm0, p.PIN_4, p.PIN_5, &prg);

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
    };

    ////spawn_core1(
    //    p.CORE1,
    //    unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //    move || {
    //        display(p.I2C1, p.PIN_2, p.PIN_3);
    //        loop {
    //            wfi();
    //        }
    //    },
    //);
    //spawner.spawn(display(p.I2C1, p.PIN_2, p.PIN_3)).unwrap();

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join3(usb_fut, join(in_fut, out_fut), encoder).await;
    //embassy_usb_logger::with_class!(1024, log::LevelFilter::Trace, serial, UsbReceiver).await;
}

#[embassy_executor::task]
async fn logger(serial: CdcAcmClass<'static, Driver<'static, USB>>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Trace, serial, UsbReceiver).await;
}

//#[embassy_executor::task]
/*async*/
fn display(i2c1: Peri<'static, I2C1>, p2: Peri<'static, impl SdaPin<I2C1>>, p3: Peri<'static, impl SclPin<I2C1>>) {
    let mut display: SSD1306<'_, { display::required_buf_size(128, 32) }> =
        SSD1306::new(i2c1, p2, p3);
    display.begin();
    display.test();
    //display.display();
    //for x in 0..32 {
    //    //for y in 0..4 {
    //        display.toggle_pixel(x, x);
    //    //}
    //    display.display();
    //}
    //display.display();
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
    const MUTE: Self = Key::Key(0x7F);
    const INC_SOUND: Self = Key::Key(0x80);
    const DEC_SOUND: Self = Key::Key(0x81);

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

    const fn f(num: u8) -> Key {
        debug_assert!(num <= 24);
        debug_assert!(num > 0);

        if num <= 12 {
            return Key::Key(num - 1 + 58);
        }
        Key::Key(num - 13 + 104)
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

struct UsbReceiver {}

impl ReceiverHandler for UsbReceiver {
    fn new() -> Self {
        UsbReceiver {}
    }

    async fn handle_data(&self, data: &[u8]) -> () {
        info!("recevived: {:?}", data);
        if data.contains(&98) {
            reboot().await;
        }
    }
}

async fn reboot() {
    //RGB_STATE.signal(RgbState::Reset);
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
    const fn new() -> Self {
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
