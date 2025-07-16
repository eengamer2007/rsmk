use core::panic::PanicInfo;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    crate::bootrom::reboot(
        0x0002, /* reboot to bootsel*/
        1,      /* 1 ms delay */
        0,      /* don't indicate a gpio because we don't use em */
        0,      /* don't disable anything or mess with LED's */
    );
    loop {}
}
