use defmt::error;
use embassy_futures::yield_now;
use embassy_rp::{
    Peri,
    i2c::{self, Blocking, I2c, Instance, SclPin, SdaPin},
};
use embassy_time::{Duration, Ticker};

pub const fn required_buf_size(width: usize, height: usize) -> usize {
    (width / 8) * height
}

pub struct SSD1306<
    'a,
    const WIDTH: usize,
    const HEIGHT: usize,
    const BUF_SIZE: usize,
    I2C: Instance,
> {
    addres: u8,
    i2c: I2c<'a, I2C, Blocking>,
    buffer: [u8; BUF_SIZE],
}

#[allow(unused)]
impl<'a, const WIDTH: usize, const HEIGHT: usize, const BUF_SIZE: usize, I2C: Instance>
    SSD1306<'a, WIDTH, HEIGHT, BUF_SIZE, I2C>
{
    pub fn new(
        i2c1: Peri<'a, I2C>,
        p2: Peri<'a, impl SdaPin<I2C>>,
        p3: Peri<'a, impl SclPin<I2C>>,
    ) -> Self {
        debug_assert_eq!(
            BUF_SIZE,
            (WIDTH /  8) * HEIGHT,
            "wrong buffer size given"
        );
        let mut config = i2c::Config::default();
        config.frequency = 400_000;
        Self {
            addres: 0x3C,
            i2c: I2c::new_blocking(i2c1, p3, p2, config),
            buffer: [0x00; BUF_SIZE],
        }
    }

    pub fn command_single(&mut self, command: u8) -> Result<(), i2c::Error> {
        self.i2c
            .blocking_write(self.addres, &[0, command])
            .inspect_err(|e| error!("failed command single: {:?}", e))?;
        Ok(())
    }

    pub fn command_list(&mut self, commands: &[u8]) -> Result<(), i2c::Error> {
        let mut out: [u8; 256] = [0; 256];
        // we set this to 1 so that the first byte stays zero
        let mut out_idx: usize = 1;

        // loop over the commands to send
        for i in commands.iter() {
            // if we overflow the buffer
            if out_idx >= out.len() {
                // send buffer
                self.i2c
                    .blocking_write(self.addres, &out)
                    .inspect_err(|e| error!("failed intermediate command list: {:?}", e))?;
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
        self.i2c
            .blocking_write(self.addres, &out[..out_idx])
            .inspect_err(|e| error!("failed end command list: {:?}", e))?;

        Ok(())
    }

    pub fn clear_screen(&mut self) {
        self.buffer = [0x00; BUF_SIZE];
    }

    pub fn begin(&mut self) -> Result<(), i2c::Error> {
        self.clear_screen();
        self.command_list(&[
            Self::DISPLAYOFF,
            Self::SETDISPLAYCLOCKDIV,
            0x80,
            Self::SETMULTIPLEX,
            (HEIGHT - 1) as u8,
        ])?;
        self.command_list(&[
            Self::SETDISPLAYOFFSET,
            0x0,
            Self::SETSTARTLINE | 0x0,
            Self::SEGREMAP | 0x01,
            Self::COMSCANDEC,
            Self::CHARGEPUMP,
            0x14,
        ])?;
        self.command_list(&[Self::MEMORYMODE, Self::ADDRESS_MODE_HORIZONTAL])?;

        let comm_pins: u8 = 0x02;
        let contrast: u8 = 0x8F;

        self.command_single(Self::SETCOMPINS)?; //setcompins
        self.command_single(comm_pins)?;
        self.command_single(Self::SETCONTRAST)?; // set contrast
        self.command_single(contrast)?;

        self.command_single(Self::SETPRECHARGE)?; // set precharge
        self.command_single(0xFF)?;

        self.command_list(&[
            Self::SETVCOMDETECT,
            0x40,
            Self::DISPLAYALLON_RESUME,
            Self::INVERTDISPLAY,
            Self::DEACTIVATE_SCROLL,
            Self::DISPLAYON,
        ])?;
        Ok(())
    }

    ///! runs a test sequence on the display
    pub async fn test(&mut self) {
        self.clear_screen();
        self.display();
        for i in 0..128 {
            self.set_pixel(i, (i % 16) + 8);
            if (i % 32) == 0 {
                yield_now().await;
                self.display();
            }
        }

        let mut count = 0;
        let mut ticker = Ticker::every(Duration::from_hz(1));
        loop {
            //yield_now().await;
            //Timer::after(Duration::from_millis(100)).await;
            self.buffer[count] = self.buffer[count] ^ 0xFF;
            count = (count + 1) % self.buffer.len();
            //self.display();
            if count % (128 / 8) == 0 {
                self.display();
                ticker.next().await;
            }
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

    pub fn display(&mut self) -> Result<(), i2c::Error> {
        self.command_list(&[Self::PAGEADDR, 0x00, 0x03])?;
        self.command_list(&[Self::COLUMNADDR, 0x00, (WIDTH - 1) as u8]);
        self.command_list(&[Self::SETDISPLAYOFFSET, 0x0])?;

        let mut out: [u8; 256] = [0; 256];
        // we set this to 1 so that the first byte stays zero
        let mut out_idx: usize = 1;
        out[0] = 0x40;

        // loop over the commands to send
        for i in self.buffer.iter() {
            // if we overflow the buffer
            if out_idx >= out.len() {
                // send buffer
                self.i2c
                    // use blocking write instead of `self.send_command` because we aren't sending
                    // commands but frame data
                    .blocking_write(self.addres, &out)
                    .inspect_err(|e| error!("failed write because: {:?}", e))?;
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
        self.i2c
            .blocking_write(self.addres, &out[..out_idx])
            .inspect_err(|e| error!("failed write because: {:?}", e))?;
        Ok(())
    }

    pub fn draw_pixel(&mut self, x: usize, y: usize, on: bool) {
        if on {
            self.set_pixel(x, y);
        } else {
            self.clear_pixel(x, y);
        }
    }

    /// takes an x,y pair and turns it into a tuple of (index, mask)
    fn coord_to_index(x: usize, y: usize) -> (usize, u8) {
        let mask = 1 << (x & 7);
        let idx = ((x / 8) * WIDTH) + y;
        (idx, mask)
    }

    pub fn set_pixel(&mut self, x: usize, y: usize) {
        debug_assert!(y < WIDTH);
        debug_assert!(x < HEIGHT);
        self.buffer[y + (x / 8) * WIDTH] |= 1 << (x & 7);
    }

    pub fn clear_pixel(&mut self, x: usize, y: usize) {
        debug_assert!(y < WIDTH);
        debug_assert!(x < HEIGHT);
        self.buffer[y + (x / 8) * WIDTH] &= !(1 << (x & 7));
    }

    pub fn toggle_pixel(&mut self, x: usize, y: usize) {
        debug_assert!(y < WIDTH);
        debug_assert!(x < HEIGHT);
        self.buffer[y + (x / 8) * WIDTH] ^= 1 << (x & 7);
    }

    pub fn vline(&mut self, x: usize) {
        for y in 0..WIDTH {
            self.set_pixel(x, y);
        }
    }

    pub fn hline(&mut self, y: usize) {
        for x in 0..HEIGHT {
            self.set_pixel(x, y);
        }
    }

    pub fn load_bitmap<const SIZE_X: usize, const SIZE_Y: usize>(&mut self, x: usize, y: usize, bitmap: [[bool; SIZE_X]; SIZE_Y]) {
        for (i, x) in (x..(x+SIZE_X)).enumerate() {
            for (j, y) in (y..(y+SIZE_Y)).rev().enumerate() {
                if bitmap[j][i] {
                    self.set_pixel(x, y);
                } else {
                    self.clear_pixel(x, y);
                }
            }
        }
    }

    pub fn line(&mut self, start: (usize, usize), end: (usize, usize)) {
        if start == end {
            self.set_pixel(start.0, start.1);
            return;
        }
        let mut sx;
        let mut sy;
        let mut ex;
        let mut ey;
        if start.0 <= end.0 && start.1 <= end.1 {
            (sx, sy) = (start.0, start.1);
            (ex, ey) = (end.0, end.1);
        } else {
            (sx, sy) = (end.0, end.1);
            (ex, ey) = (start.0, start.1);
        }

        let dx = sx.abs_diff(ex);
        let dy = sy.abs_diff(ey);

        if dx == 0 {}
        if dy == 0 {}

        if dx > dy {
            // step size
            let m = dy / dx;
            let mut neg = false;
            if ex < sx {
                ex ^= sx;
                sx ^= ex;
                ex ^= sx;
                ey ^= sy;
                sy ^= ey;
                ey ^= sy;
                neg = true;
            }
            let mut x = sx;
            let mut y = sy;
            self.set_pixel(x, y);
            let mut count = 0;
            while x < ex {
                count += 1;
                x += 1;
                if count >= m {
                    count = 0;
                    if neg {
                        y -= 1;
                    } else {
                        y += 1;
                    }
                }
                self.set_pixel(x, y);
            }
        } else {
            // from here on just the same thing but x and y swapped
            let m = dy / dx;
            let mut neg = false;
            if ey < sy {
                ex ^= sx;
                sx ^= ex;
                ex ^= sx;
                ey ^= sy;
                sy ^= ey;
                ey ^= sy;
                neg = true;
            }
            let mut x = sx;
            let mut y = sy;
            self.set_pixel(x, y);
            let mut count = 0;
            while y < ey {
                count += 1;
                y += 1;
                if count >= m {
                    count = 0;
                    if neg {
                        x -= 1;
                    } else {
                        x += 1;
                    }
                }
                self.set_pixel(x, y);
            }
        }
    }
}

// command numbers
#[allow(unused)]
impl<'a, const WIDTH: usize, const HEIGHT: usize, const BUF_SIZE: usize, I2C: Instance>
    SSD1306<'a, WIDTH, HEIGHT, BUF_SIZE, I2C>
{
    /// sets the page address mode
    /// can be either
    /// - page (0b10)
    /// - horizontal (0b00)
    /// - vertical (0b01)
    const MEMORYMODE: u8 = 0x20;
    const COLUMNADDR: u8 = 0x21;
    const PAGEADDR: u8 = 0x22;
    const SETCONTRAST: u8 = 0x81;
    const CHARGEPUMP: u8 = 0x8D;
    const SEGREMAP: u8 = 0xA0;
    /// resumes back to normal operation
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

    /// sets the low nibble for the starting column in page addressing mode
    /// last 4 bits are the nibble
    const SETLOWCOLUMN: u8 = 0x00;
    /// sets the high nibble for the starting column in page addressing mode
    /// last 4 bits are the nibble
    const SETHIGHCOLUMN: u8 = 0x10;
    const SETSTARTLINE: u8 = 0x40;

    const RIGHT_HORIZONTAL_SCROLL: u8 = 0x26;
    const LEFT_HORIZONTAL_SCROLL: u8 = 0x27;
    const VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL: u8 = 0x29;
    const VERTICAL_AND_LEFT_HORIZONTAL_SCROLL: u8 = 0x2A;
    const DEACTIVATE_SCROLL: u8 = 0x2E;
    const ACTIVATE_SCROLL: u8 = 0x2F;
    const SET_VERTICAL_SCROLL_AREA: u8 = 0xA3;
}

/// page addressing modes
#[allow(unused)]
impl<'a, const WIDTH: usize, const HEIGHT: usize, const BUF_SIZE: usize, I2C: Instance>
    SSD1306<'a, WIDTH, HEIGHT, BUF_SIZE, I2C>
{
    const ADDRESS_MODE_PAGE: u8 = 0b10;
    const ADDRESS_MODE_VERTICAL: u8 = 0b01;
    const ADDRESS_MODE_HORIZONTAL: u8 = 0b00;
}
/// other consts
#[allow(unused)]
impl<'a, const WIDTH: usize, const HEIGHT: usize, const BUF_SIZE: usize, I2C: Instance>
    SSD1306<'a, WIDTH, HEIGHT, BUF_SIZE, I2C>
{
    /// External display voltage source
    const EXTERNALVCC: u8 = 0x01;
    /// Gen. display voltage from 3.3V
    /// this is what we should use for our I2C display
    const SWITCHCAPVCC: u8 = 0x02;
}
