/// IO functionality exposed to Nordic C SDK
use crate::{ITM_HANDLE, RDY_PIN_HANDLE, REQ_PIN_HANDLE, RESET_PIN_HANDLE, SPI_HANDLE};
use core::{convert::Infallible, ops::DerefMut};
use cortex_m::{interrupt, iprintln};
use hal::prelude::*;
use nrf8001_sys::eGpioPinMode;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use stm32f4xx_hal as hal;

#[derive(Debug, Clone, Copy, FromPrimitive)]
pub enum Pin {
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
    if let Some(Pin::RdyN) = FromPrimitive::from_u8(pin) {
        unsafe {
            return RDY_PIN_HANDLE.as_mut().unwrap().is_high().unwrap();
        }
    } else {
        panic!("Tried to read an invalid pin");
    }
}

#[no_mangle]
pub extern "C" fn digitalWrite(pin: u8, level: bool) {
    let pin_handle = unsafe {
        match FromPrimitive::from_u8(pin) {
            Some(Pin::Reset) => RESET_PIN_HANDLE.as_mut().unwrap()
                as &mut dyn _embedded_hal_digital_v2_OutputPin<Error = Infallible>,
            Some(Pin::ReqN) => REQ_PIN_HANDLE.as_mut().unwrap()
                as &mut dyn _embedded_hal_digital_v2_OutputPin<Error = Infallible>,
            _ => return,
        }
    };
    if level {
        pin_handle.set_high().ok();
    } else {
        pin_handle.set_low().ok();
    }
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
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "SPI: Tx:{:x} Rx{:x}", txbuf, *rxbuf);
    true
}

#[no_mangle]
pub extern "C" fn pinMode(_pin: u8, _mode: eGpioPinMode) {}
