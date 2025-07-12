use embassy_rp::{i2c::{self, Blocking, I2c, SclPin, SdaPin}, peripherals::{I2C1, PIN_2, PIN_3}, Peripherals};

pub struct SSD1306<'a> {
    i2c: I2c<'a, I2C1, Blocking>
}

impl<'a> SSD1306<'a> {
    pub fn new(i2c1: I2C1, p2: PIN_2, p3: PIN_3) -> Self {
        Self {
            i2c: I2c::new_blocking(i2c1, p3, p2, i2c::Config::default())
        }
    }
}
