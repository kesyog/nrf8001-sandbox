/// Delay functionality exposed to Nordic C SDK
use crate::DELAY_HANDLE;
use hal::prelude::*;
use stm32f4xx_hal as hal;

#[no_mangle]
/// Delay for the given length of time
pub extern "C" fn delay(milliseconds: u16) {
    unsafe {
        DELAY_HANDLE
            .as_mut()
            .expect("delay uninitialized")
            .delay_ms(milliseconds);
    }
}
