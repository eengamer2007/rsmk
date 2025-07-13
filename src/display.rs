use defmt::error;
use embassy_rp::{
    i2c::{self, Blocking, I2c, SclPin, SdaPin}, peripherals::I2C1, Peri
};

pub const fn required_buf_size(width: usize, height: usize) -> usize {
    width * ((height * 7) / 8)
}

pub struct SSD1306<'a, const BUF_SIZE: usize> {
    addres: u8,
    i2c: I2c<'a, I2C1, Blocking>,
    buffer: [u8; BUF_SIZE],
}

impl<'a, const BUF_SIZE: usize> SSD1306<'a, BUF_SIZE> {
    pub fn new(i2c1: Peri<'a, I2C1>, p2: Peri<'a, impl SdaPin<I2C1>>, p3: Peri<'a, impl SclPin<I2C1>>) -> Self {
        let mut config = i2c::Config::default();
        config.frequency = 800_000;
        Self {
            addres: 0x3C,
            i2c: I2c::new_blocking(i2c1, p3, p2, config),
            buffer: [0x00; BUF_SIZE],
        }
    }

    pub fn command_single(&mut self, command: u8) {
        let _ = self
            .i2c
            .blocking_write(self.addres, &[0, command])
            .inspect_err(|e| error!("failed write because: {:?}", e));
    }

    pub fn command_list(&mut self, commands: &[u8]) {
        let mut out: [u8; 256] = [0; 256];
        // we set this to 1 so that the first byte stays zero
        let mut out_idx: usize = 1;

        // loop over the commands to send
        for i in commands.iter() {
            // if we overflow the buffer
            if out_idx >= out.len() {
                // send buffer
                let _ = self
                    .i2c
                    .blocking_write(self.addres, &out)
                    .inspect_err(|e| error!("failed write because: {:?}", e));
                // reset buffer
                out = [0; 256];
                // set index to start
                out_idx = 1;
            }
            // set value
            out[out_idx] = *i;
            // and go to the next index
            out_idx += 1;
        }
        // send the buffer
        let _ = self
            .i2c
            .blocking_write(self.addres, &out[..out_idx])
            .inspect_err(|e| error!("failed write because: {:?}", e));
    }

    pub fn clear_screen(&mut self) {
        self.buffer = [0x00; BUF_SIZE];
    }

    pub fn begin(&mut self) {
        self.clear_screen();
        self.command_list(&[
            Self::DISPLAYOFF,
            Self::SETDISPLAYCLOCKDIV,
            0x80,
            Self::SETMULTIPLEX,
        ]);
        // height - 1
        self.command_single(31);
        self.command_list(&[0xd3, 0x0, 0x40, 0x8D]);
        self.command_single(0x14);
        self.command_list(&[0x20, 0x0, 0xA0 | 0x01, 0xC8]);

        let comm_pins: u8 = 0x02;
        let contrast: u8 = 0x8F;

        self.command_single(0xda); //setcompins
        self.command_single(comm_pins);
        self.command_single(0x81); // set contrast
        self.command_single(contrast);

        self.command_single(0xd9); // set precharge
        self.command_single(0xF1);

        self.command_list(&[0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF]);
    }

    ///! runs a test sequence on the display
    pub fn test(&mut self) {
        self.clear_screen();
        self.display();
        for i in 0..128 {
            self.set_pixel(i, i % 32);
            self.display();
        }

        let mut count = 0;
        loop {
            count = count % self.buffer.len();
            self.buffer[count] = self.buffer[count] ^ 0xFF;
            count += 1;
            self.display();
        }

        /*
        self.clear_screen();
        self.display();
        self.buffer[0] = 1 << 7;
        let mut cur = 0;
        loop {
            if self.buffer[cur] & 1 != 0 {
                //self.buffer[cur] = 0;
                self.buffer[cur + 1] = 1 << 7;
                cur += 1;
            } else {
                self.buffer[cur] >>= 1;
                self.buffer[cur] |= 1 << 7;
            }
            self.display();
            if cur == self.buffer.len() {
                break;
            }
        }*/
    }

    pub fn display(&mut self) {
        self.command_list(&[Self::PAGEADDR, 0x00, 0xFF, Self::COLUMNADDR]);

        // column start
        self.command_single(0);
        // column end
        self.command_single(128 - 1);

        let mut out: [u8; 256] = [0; 256];
        // we set this to 1 so that the first byte stays zero
        let mut out_idx: usize = 1;
        out[0] = 0x40;

        // loop over the commands to send
        for i in self.buffer.iter() {
            // if we overflow the buffer
            if out_idx >= out.len() {
                // send buffer
                let _ = self
                    .i2c
                    .blocking_write(self.addres, &out)
                    .inspect_err(|e| error!("failed write because: {:?}", e));
                // reset buffer
                out = [0; 256];
                // set index to start
                out_idx = 1;
                out[0] = 0x40;
            }
            // set value
            out[out_idx] = *i;
            // and go to the next index
            out_idx += 1;
        }
        // send the buffer
        let _ = self
            .i2c
            .blocking_write(self.addres, &out[..out_idx])
            .inspect_err(|e| error!("failed write because: {:?}", e));
    }

    pub fn draw_pixel(&mut self, x: usize, y: usize, on: bool) {
        if on {
            self.set_pixel(x, y);
        } else {
            self.clear_pixel(x, y);
        }
    }

    pub fn set_pixel(&mut self, x: usize, y: usize) {
        debug_assert!(x < 32);
        debug_assert!(y < 128);
        self.buffer[y + (x / 8) * 32] |= 1 << (x & 7);
    }

    pub fn clear_pixel(&mut self, x: usize, y: usize) {
        debug_assert!(x < 32);
        debug_assert!(y < 128);
        self.buffer[x + (y / 8) * 32] &= !(1 << (y & 7));
    }

    pub fn toggle_pixel(&mut self, x: usize, y: usize) {
        debug_assert!(x < 32);
        debug_assert!(y < 128);
        self.buffer[x + (y / 8) * 32] ^= 1 << (y & 7);
    }
}

// command numbers
#[allow(unused)]
impl<'a, const BUF_SIZE: usize> SSD1306<'a, BUF_SIZE> {
    const MEMORYMODE: u8 = 0x20;
    const COLUMNADDR: u8 = 0x21;
    const PAGEADDR: u8 = 0x22;
    const SETCONTRAST: u8 = 0x81;
    const CHARGEPUMP: u8 = 0x8D;
    const SEGREMAP: u8 = 0xA0;
    const DISPLAYALLON_RESUME: u8 = 0xA4;
    const DISPLAYALLON: u8 = 0xA5;
    const NORMALDISPLAY: u8 = 0xA6;
    const INVERTDISPLAY: u8 = 0xA7;
    const SETMULTIPLEX: u8 = 0xA8;
    const DISPLAYOFF: u8 = 0xAE;
    const DISPLAYON: u8 = 0xAF;
    const COMSCANINC: u8 = 0xC0;
    const COMSCANDEC: u8 = 0xC8;
    const SETDISPLAYOFFSET: u8 = 0xD3;
    const SETDISPLAYCLOCKDIV: u8 = 0xD5;
    const SETPRECHARGE: u8 = 0xD9;
    const SETCOMPINS: u8 = 0xDA;
    const SETVCOMDETECT: u8 = 0xDB;
}
