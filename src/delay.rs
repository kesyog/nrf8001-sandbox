use crate::DELAY_HANDLE;
/// Delay functionality exposed to Nordic C SDK
use core::ops::DerefMut;
use cortex_m::interrupt;
use hal::prelude::*;
use stm32f4xx_hal as hal;

#[no_mangle]
/// Delay for the given length of time
pub extern "C" fn delay(milliseconds: u16) {
    // Not great that this blocks all interrupts, but it's fine for now as we're not using them
    interrupt::free(|cs| {
        let mut delay = DELAY_HANDLE.borrow(cs).borrow_mut();
        delay.deref_mut().as_mut().unwrap().delay_ms(milliseconds);
    });
}
