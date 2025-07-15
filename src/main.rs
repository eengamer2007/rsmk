#![no_std]
#![no_main]
#![allow(internal_features)]
#![feature(core_intrinsics)]

use core::mem::MaybeUninit;
use core::sync::atomic::AtomicI32;

use embassy_executor::{Executor, Spawner};

use embassy_futures::yield_now;
use embassy_rp::gpio::{Flex, Input, Pull};
use embassy_rp::i2c::{SclPin, SdaPin};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{I2C1, PIN_2, PIN_3, PIO0, PIO1, USB};
use embassy_rp::pio::Pio;
use embassy_rp::{Peri, bind_interrupts};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use embassy_usb_logger::ReceiverHandler;

use encoder::{encoder_setup, run_encoder};
use keyboard::{run_keyboard, run_reader};
use log::info;

use rgb::{rgb_runner, rgb_setup};

use static_cell::StaticCell;
use usb::usb_init;

use defmt_rtt as _;

//use panic_probe as _;

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

mod usb;

mod keyboard;

mod encoder;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
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
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

static RGB_STATE: Signal<CriticalSectionRawMutex, rgb::RgbState> = Signal::new();

static mut COLUMN_PINS: MaybeUninit<[Flex; 6]> = MaybeUninit::uninit();
static mut ROW_PINS: MaybeUninit<[Input; 5]> = MaybeUninit::uninit();

static VOL_COUNTER: AtomicI32 = AtomicI32::new(0);

#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    bootrom::reboot(
        0x0002, /* reboot to bootsel*/
        1,      /* 1 ms delay */
        0,      /* don't indicate a gpio because we don't use em */
        0,      /* don't disable anything or mess with LED's */
    );
    loop {}
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO1, crate::Pio1Irqs);

    RGB_STATE.signal(rgb::RgbState::Reset);

    let led = rgb_setup(&mut common, sm0, p.DMA_CH0, p.PIN_0);
    spawner.spawn(rgb_runner(led)).unwrap();

    let (hid, serial) = usb_init(&spawner, p.USB, Irqs);

    let (reader, writer) = hid.split();

    let is_left = {
        // left right pin is either connected to ground or to VCC
        // so no pull up or down required
        let left_right_pin = Input::new(p.PIN_28, Pull::None);

        left_right_pin.is_high()
    };

    info!("is_left: {}", is_left);

    unsafe {
        COLUMN_PINS = MaybeUninit::new(column_pins!(
            p.PIN_9, p.PIN_26, p.PIN_22, p.PIN_20, p.PIN_23, p.PIN_21
        ))
    }
    unsafe { ROW_PINS = MaybeUninit::new(row_pins!(p.PIN_29, p.PIN_27, p.PIN_6, p.PIN_7, p.PIN_8)) }
    #[allow(static_mut_refs)]
    spawner.must_spawn(run_keyboard(
        unsafe { COLUMN_PINS.assume_init_mut() },
        unsafe { ROW_PINS.assume_init_mut() },
        writer,
        &VOL_COUNTER,
    ));
    spawner.spawn(run_reader(reader)).unwrap();

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Pio0Irqs);

    let enc = encoder_setup(&mut common, sm0, p.PIN_4, p.PIN_5);

    spawner.spawn(run_encoder(&VOL_COUNTER, enc)).unwrap();
    //let _ = spawner.spawn(display_task(p.I2C1, p.PIN_2, p.PIN_3));

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor = CORE1_EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                let _ = spawner.spawn(display_task(p.I2C1, p.PIN_2, p.PIN_3));//.inspect_err(|e| info!("display task failed to start: {}", e) );
            });
        },
    );
    //spawner.spawn(display(p.I2C1, p.PIN_2, p.PIN_3)).unwrap();

    embassy_usb_logger::with_class!(1024, log::LevelFilter::Trace, serial, UsbReceiver).await;
}

//#[embassy_executor::task]
//async fn logger(serial: CdcAcmClass<'static, Driver<'static, USB>>) {
//    embassy_usb_logger::with_class!(1024, log::LevelFilter::Trace, serial, UsbReceiver).await;
//}

#[embassy_executor::task]
async fn display_task(i2c1: Peri<'static, I2C1>, p2: Peri<'static, PIN_2>, p3: Peri<'static, PIN_3>) {
    let _ = display(i2c1, p2, p3).await;//.inspect_err(|e| info!("display code failed because: {:?}", e));
}

//#[embassy_executor::task]
async fn display(
    i2c1: Peri<'static, I2C1>,
    p2: Peri<'static, impl SdaPin<I2C1>>,
    p3: Peri<'static, impl SclPin<I2C1>>,
) -> Result<(), embassy_rp::i2c::Error> {
    let mut display: SSD1306<'_, { display::required_buf_size(128, 32) }, I2C1> =
        SSD1306::new(i2c1, p2, p3);
    display.begin()?;
    display.test().await;
    //display.display();
    //for x in 0..32 {
    //    //for y in 0..4 {
    //        display.toggle_pixel(x, x);
    //    //}
    //    display.display();
    //}
    //display.display();
    Ok(())
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
macro_rules! column_pins_static {
    ( $name:tt, $($x:expr), *) => {
        static $name: [Flex; count!($($x)*)] = [$(Flex::new($x)),*];
        //const SIZE: usize = count!($($x)*);
        //let out: [Flex; SIZE] = [$(Flex::new($x)),*];

        //out
    };
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
        //info!("recevived: {:?}", data);
        if data.contains(&98) {
            reboot().await;
        }
    }
}

async fn reboot() {
    RGB_STATE.signal(rgb::RgbState::Reset);
    yield_now().await;
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
