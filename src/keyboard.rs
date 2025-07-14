use embassy_rp::gpio::{Flex, Input};
use log::warn;

#[embassy_executor::task]
pub async fn run_keyboard(
    column_pins: &'static [Flex<'static>],
    row_pins: &'static [Input<'static>],
) {
    let mut columns = column_pins!(p.PIN_9, p.PIN_26, p.PIN_22, p.PIN_20, p.PIN_23, p.PIN_21);
    //let mut columns = column_pins!(p.PIN_21, p.PIN_23, p.PIN_20, p.PIN_22, p.PIN_26, p.PIN_9);

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
                [Key::None,   Key::None, Key::None,  Key::REBOOT,  Key::REBOOT,                    Key::None],
                [Key::None,   Key::None, Key::None,  Key::REBOOT,  Key::Special(Special::Lower), Key::None],
            ],
            [ // UPPER
                [Key::REBOOT, Key::f(1), Key::f(2),  Key::f(3),  Key::f(4),                    Key::None],
                [Key::None,   Key::f(5), Key::f(6),  Key::f(7),  Key::f(8),                    Key::None],
                [Key::None,   Key::f(9), Key::f(10), Key::f(11), Key::f(12),                   Key::None],
                [Key::None,   Key::None, Key::None,  Key::None,  Key::None,                    Key::None],
                [Key::None,   Key::None, Key::None,  Key::REBOOT,  Key::Special(Special::Lower), Key::None],
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
            //delay(100);
            for (y, row) in rows.iter().enumerate() {
                let low = row.is_low();
                //row.
                delay(1000);
                out[y][x] = low;
                if low {
                    info!("trigger: {},{}", x, y);
                }
            }
            core::hint::black_box(column.set_as_input());
            //delay(100);
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
