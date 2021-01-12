/// IO functionality exposed to Nordic C SDK
use crate::{RDY_PIN_HANDLE, REQ_PIN_HANDLE, RESET_PIN_HANDLE, SPI_HANDLE};
use core::{cell::RefCell, ops::DerefMut};
use cortex_m::interrupt::{self, Mutex};
use hal::prelude::*;
use nrf8001_sys::eGpioPinMode;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use stm32f4xx_hal as hal;

#[derive(Debug, Clone, Copy, FromPrimitive)]
pub enum IOPin {
    Mosi = 0,
    Miso = 1,
    Sck = 2,
    Reset = 3,
    RdyN = 4,
    ReqN = 5,
    Cs = 6,
    Unused = 255,
}

#[no_mangle]
pub extern "C" fn digitalRead(pin: u8) -> bool {
    if let Some(IOPin::RdyN) = IOPin::from_u8(pin) {
        interrupt::free(|cs| {
            return RDY_PIN_HANDLE
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .is_high()
                .unwrap();
        })
    } else {
        panic!("Tried to read an invalid pin");
    }
}

fn writePin<T>(pin_handle_locked: &Mutex<RefCell<Option<T>>>, level: bool)
where
    T: _embedded_hal_digital_v2_OutputPin,
{
    interrupt::free(|cs| {
        let mut pin_handle = pin_handle_locked.borrow(cs).borrow_mut();
        let pin_handle = pin_handle.as_mut().unwrap();
        if level {
            pin_handle.set_high().ok();
        } else {
            pin_handle.set_low().ok();
        }
    });
}

#[no_mangle]
pub extern "C" fn digitalWrite(pin: u8, level: bool) {
    match IOPin::from_u8(pin) {
        Some(IOPin::Reset) => writePin(&RESET_PIN_HANDLE, level),
        Some(IOPin::ReqN) => writePin(&REQ_PIN_HANDLE, level),
        _ => {}
    };
}

#[no_mangle]
pub extern "C" fn transmit_SPI_byte(txbuf: u8, rxbuf: &mut u8) -> bool {
    interrupt::free(|cs| {
        let mut spi = SPI_HANDLE.borrow(cs).borrow_mut();
        let spi = spi.deref_mut().as_mut().unwrap();
        spi.send(txbuf).unwrap();
        while !spi.is_txe() {}
        while !spi.is_rxne() {}
        *rxbuf = spi.read().unwrap();
    });
    true
}

#[no_mangle]
pub extern "C" fn pinMode(_pin: u8, _mode: eGpioPinMode) {}
