use embassy_executor::Spawner;
use embassy_rp::{Peri, peripherals::USB, usb::Driver};
use embassy_usb::{
    Builder, Config, UsbDevice,
    class::{
        cdc_acm::{self, CdcAcmClass},
        hid::{self, HidReaderWriter},
    },
};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};

use crate::{Irqs, MyDeviceHandler, MyRequestHandler};

const USB_CONFIG: Config = {
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("me, myself and I");
    config.product = Some("Not a keyboard");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // might be nice to look into later how this works
    config.supports_remote_wakeup = false;
    config
};

static mut config_descriptor: [u8; 256] = [0; 256];
static mut bos_descriptor: [u8; 256] = [0; 256];
static mut msos_descriptor: [u8; 256] = [0; 256];
// need to be atleast the size of the biggest expected packet
// i'm making it bigger just to be sure
static mut control_buf: [u8; 256] = [0; 256];
//static mut request_handler: MyRequestHandler = MyRequestHandler {};
static mut device_handler: MyDeviceHandler = MyDeviceHandler::new();
static mut hid_state: hid::State = hid::State::new();
static mut s_state: cdc_acm::State = cdc_acm::State::new();

pub fn usb_init(spawner: &Spawner, usb: Peri<'static, USB>, irq: Irqs) -> (
    HidReaderWriter<'static, Driver<'static, USB>, 1, 8>, CdcAcmClass<'static, Driver<'static, USB>>) {
    let driver: Driver<'static, USB> = Driver::new(usb, irq);

    #[allow(static_mut_refs)]
    let mut builder = Builder::new(
        driver,
        USB_CONFIG,
        #[allow(static_mut_refs)]
        unsafe {
            &mut config_descriptor
        },
        unsafe { &mut bos_descriptor },
        unsafe { &mut msos_descriptor },
        unsafe { &mut control_buf },
    );
    #[allow(static_mut_refs)]
    builder.handler(unsafe { &mut device_handler });

    let config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };
    #[allow(static_mut_refs)]
    let hid = HidReaderWriter::<_, 1, 8>::new(&mut builder, unsafe { &mut hid_state }, config);
    #[allow(static_mut_refs)]
    let serial = CdcAcmClass::new(&mut builder, unsafe { &mut s_state }, 64);

    let usb = builder.build();

    spawner.spawn(usb_runner(usb)).unwrap();

    (hid, serial)
}

#[embassy_executor::task]
async fn usb_runner(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}
