#![no_std]
#![no_main]
#![feature(core_intrinsics)]

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

//use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::*;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Flex, Input, Pull};
use embassy_rp::peripherals::{I2C1, PIO0, PIO1, USB};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Instant, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler, State};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Config, Handler};
use embassy_usb_logger::ReceiverHandler;
use log::{info, warn};
use smart_leds::RGB8;
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
                core::hint::black_box(column.set_as_output());
                for (y, row) in rows.iter().enumerate() {
                    out[y][x] = row.is_low();
                    if row.is_low() {
                        info!("trigger: {},{}", x, y);
                    }
                }
                //column.set_as_input();
                core::hint::black_box(column.set_as_input());
            }

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

    let i2c = async {
        let config = embassy_rp::i2c::Config::default();
        let mut i2c =
            embassy_rp::i2c::I2c::new_blocking(p.I2C1, p.PIN_3, p.PIN_2, /*I2CIrqs,*/ config);

        //let data: &mut [u8] = &mut [0; 32];

        //info!("{:?}\n", data);
        //info!("{:?}", i2c.read_async(0x3Cu16, data).await);
        //info!("{:?}\n", data);
        //info!("{:?}", i2c.blocking_write(0x3Cu16, &[1 << 7, 0xAF]));

        /*let data: [&[u8]; 16] = [
            &[0xAE], // disable
            &[0x04], // lower column addressing
            &[0x10], // higher column addressing
            &[0x40], // display start line
            &[0x81, 0x80], // contrast
            &[0xA1], // segment remap
            &[0xA6], // normal display
            &[0xA8, 0x1F], // mux ratio
            &[0xC8], //  com direction scan output
            &[0xD3, 0x00], // display offset
            &[0xD5, 0xF0], // clock div and freq
            &[0xD8, 0x05], // ?
            &[0xD9, 0xC2], // pre charge period
            &[0xDA, 0x12], // com pin hw config
            &[0xDB, 0x08], // v_comh deselect
            &[0xAF], // enable
        ];*/

        /*for b in data {
            write_cmd(&mut i2c, b);
            Timer::after_secs(1).await;
        }*/

        //write_cmd(&mut i2c, &[0xAE, 0x04, 0x10, 0x40, 0x81, 0x80, 0xA1, 0xA6, 0xA8, 0x1F, 0xC8, 0xD3, 0x00, 0xD5, 0xF0, 0xD8, 0x05, 0xD9, 0xC2, 0xDA, 0x12, 0xDB, 0x08, 0xAF]);

        //write_cmd(&mut i2c, 0xA);
        //info!("{:?}", i2c.read_async(0x3Cu16, data).await);
        //info!("{:?}\n", data);

        //let data: &mut [u8] = &mut [0; 32];
        //Timer::after_secs(1).await;
        //info!("{:?}", i2c.write_async(0x3Cu16, [1 << 7, 0x81, 1 << 7, 0xFF]).await);
        //info!("{:?}", i2c.read_async(0x3Cu16, data).await);
        //info!("{:?}\n", data);

        //Timer::after_secs(1).await;

        write_cmd(&mut i2c, &[0xAE]);
        //let data: &mut [u8] = &mut [0; 32];

        write_cmd(&mut i2c, &[0x81, 0xFF]);

        info!("{:?}", i2c.blocking_write(0x3Cu16, &[1 << 7, 0xA5]));

        Timer::after_secs(1).await;
        //info!("{:?}", i2c.read_async(0x3Cu16, data).await);
        //info!("{:?}\n", data);

        //Timer::after_secs(1).await;

        //let data: &mut [u8] = &mut [0; 32];
        info!("{:?}", i2c.blocking_write(0x3Cu16, &[1 << 7, 0xA6]));

        Timer::after_secs(1).await;
        //info!("{:?}", i2c.read_async(0x3Cu16, data).await);
        //info!("{:?}\n", data);
        write_cmd(&mut i2c, &[0xAF]);
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

    let rgb = async {
        let mut buf = [RGB8::new(255, 0, 0); LED_COUNT];

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

        let Pio {
            mut common, sm0, ..
        } = Pio::new(p.PIO1, Pio1Irqs);

        let prg = PioWs2812Program::new(&mut common);
        let mut led = PioWs2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_0, &prg);

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
            Timer::after_millis(5).await;
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join5(
        usb_fut,
        join(in_fut, out_fut),
        embassy_usb_logger::with_class!(1024, log::LevelFilter::Trace, serial, UsbReceiver),
        i2c,
        join(encoder, rgb),
    )
    .await;
}

fn color(idx: usize, phase: f32, time: f32) -> u8 {
    use core::f32::consts::PI;
    use core::intrinsics::sinf32;

    (((unsafe { sinf32((phase * (2. / 3.) * PI) + time + ((idx as f32 / LED_COUNT as f32) * 2. * PI)) } + 1.) / 2.) * 255.) as u8
}

fn write_cmd(i2c: &mut embassy_rp::i2c::I2c<I2C1, embassy_rp::i2c::Blocking>, byte: &[u8]) {
    let out = &mut [0; 512];

    byte.iter().enumerate().for_each(|(i, val)| {
        out[i * 2] = 1 << 7;
        out[i * 2 + 1] = *val;
    });

    info!(
        "{:?}",
        i2c.blocking_write(0x3Cu8, &out[0..(byte.len() * 2)])
    );
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
