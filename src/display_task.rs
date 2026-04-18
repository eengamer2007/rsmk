use embassy_rp::{
    Peri,
    i2c::{Instance, SclPin, SdaPin},
    peripherals::{I2C1, PIN_2, PIN_3},
};
use log::info;

use crate::display::{self, SSD1306};

#[embassy_executor::task]
pub async fn display_task(
    i2c1: Peri<'static, I2C1>,
    p2: Peri<'static, PIN_2>,
    p3: Peri<'static, PIN_3>,
) {
    let _ = display(i2c1, p2, p3)
        .await
        .inspect_err(|e| info!("display errored with: {:?}", e));
}

async fn display<I2C: Instance>(
    i2c1: Peri<'static, I2C>,
    p2: Peri<'static, impl SdaPin<I2C>>,
    p3: Peri<'static, impl SclPin<I2C>>,
) -> Result<(), embassy_rp::i2c::Error> {
    #[allow(non_upper_case_globals)]
    const i: bool = true;
    const O: bool = false;
    // https://www.reddit.com/r/rust/comments/pw54rx/media_heres_a_crate_i_just_made_for_converting/
    // (with some slight changes to make it look better on the oled)
    const FERRIS: [[bool; 32]; 24] = [
        [
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, i, i, i, i, i, i, O, i, O, O, i, O, i, i, i, i, i, i, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, i, i, i, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, i, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, i,
            i, i, i,
        ],
        [
            i, i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, i,
            i, i, i,
        ],
        [
            i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
            i, i, i,
        ],
        [
            i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
            i, i, i,
        ],
        [
            i, i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
            i, i, i,
        ],
        [
            i, i, O, O, O, O, O, O, O, O, O, O, i, i, i, O, O, i, i, i, O, O, O, O, O, O, O, O, O,
            O, i, i,
        ],
        [
            i, i, O, O, O, O, O, O, O, O, O, O, i, i, i, O, O, i, i, i, O, O, O, O, O, O, O, O, O,
            O, i, i,
        ],
        [
            i, O, O, O, O, O, O, O, O, O, O, O, i, i, i, O, O, i, i, i, O, O, O, O, O, O, O, O, O,
            O, O, i,
        ],
        [
            i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
            O, O, i,
        ],
        [
            i, i, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O, O,
            O, i, i,
        ],
        [
            i, i, i, O, O, i, i, i, O, O, O, O, i, i, i, i, i, i, i, i, O, O, O, O, i, i, i, O, O,
            i, i, i,
        ],
        [
            i, i, i, i, O, O, i, i, i, O, O, O, i, i, i, i, i, i, i, i, O, O, O, i, i, i, O, O, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, O, i, i, i, i, O, O, O, O, O, i, i, O, O, O, O, O, i, i, i, i, O, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, i, i, i, i, O, O, O, O, i, i, O, O, O, O, i, i, i, i, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i,
            i, i, i,
        ],
        [
            i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i,
            i, i, i,
        ],
    ];

    let mut display: SSD1306<'_, 128, 32, { display::required_buf_size(128, 32) }, I2C> =
        SSD1306::new(i2c1, p2, p3);
    display.begin()?;
    display.hline(128 - 5, true);
    display.display()?;
    display.line((0, 0), (31, 127));
    display.line((31, 0), (0, 127));
    display.display()?;
    display.load_bitmap(0, 128 - 24, FERRIS);
    display.display()?;

    //display.test().await?;
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
