use core::mem::MaybeUninit;

static mut DBG_LED: MaybeUninit<DbgLedInner> = MaybeUninit::uninit();

struct DbgLedInner{
    
}

impl DbgLedInner {
    fn new() -> Self {
        Self {}
    }
}

pub struct DbgLed();

impl DbgLed {
    pub unsafe fn init() {
        Self::init_inner();
    }

    fn init_inner() {
        #[allow(static_mut_refs)]
        unsafe { DBG_LED.write(DbgLedInner::new()); }
    }

    pub fn set_color(color: [u8; 3]) {

    }
}
