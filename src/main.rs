#![no_std]
#![no_main]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

mod delay;
mod io_interface;
mod nrf_ble_uart;
mod nrf_ble_uart_consts;
//mod nrf_verify_transport;

use crate::hal::{
    delay::Delay,
    gpio::{
        self,
        gpioa::{PA5, PA6, PA7, PA9},
        gpioc::{PC0, PC7},
        PullUp, PushPull, AF5,
    },
    prelude::*,
    spi, stm32,
};
use core::cell::RefCell;
use cortex_m::{
    interrupt::{self, Mutex},
    iprintln,
};
use cortex_m_rt::entry;
use panic_itm as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32f4xx_hal as hal;

type NrfSpi = spi::Spi<
    hal::stm32::SPI1,
    (
        PA5<gpio::Alternate<AF5>>,
        PA6<gpio::Alternate<AF5>>,
        PA7<gpio::Alternate<AF5>>,
    ),
>;
type ResetPin = PC7<gpio::Output<PushPull>>;
type RdyNPin = PA9<gpio::Input<PullUp>>;
type ReqNPin = PC0<gpio::Output<PushPull>>;

static SPI_HANDLE: Mutex<RefCell<Option<NrfSpi>>> = Mutex::new(RefCell::new(None));
static RESET_PIN_HANDLE: Mutex<RefCell<Option<ResetPin>>> = Mutex::new(RefCell::new(None));
static RDY_PIN_HANDLE: Mutex<RefCell<Option<RdyNPin>>> = Mutex::new(RefCell::new(None));
static REQ_PIN_HANDLE: Mutex<RefCell<Option<ReqNPin>>> = Mutex::new(RefCell::new(None));
// Unsafe global singletons. Not using these in interrupts so they're safe.
static mut DELAY_HANDLE: Option<Delay> = None;
static mut ITM_HANDLE: Option<stm32::ITM> = None;

#[entry]
fn main() -> ! {
    init_mcu();

    //let mut aci_context = nrf_verify_transport::Context::new();
    let mut aci_context = nrf_ble_uart::Context::new();

    loop {
        //nrf_verify_transport::poll(&mut aci_context);
        nrf_ble_uart::poll(&mut aci_context);
    }
}

fn init_mcu() {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();

    iprintln!(&mut itm.stim[0], "Boot");

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let sck = gpioa.pa5.into_alternate_af5();
    let mosi = gpioa.pa7.into_alternate_af5();
    let miso = gpioa.pa6.into_alternate_af5();
    let spi1 = spi::Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        spi::Mode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        },
        // Max supported SPI clock of the nRF8001 is 3MHz
        2.mhz().into(),
        clocks,
    );
    // The HAL library doesn't allow specifying the lsbfirst option, which the nRF8001 relies on
    // Set the necessary bit to enable it via the peripheral access crate
    let spi1_raw = unsafe { &mut *(stm32::SPI1::ptr() as *mut stm32::spi1::RegisterBlock) };
    spi1_raw.cr1.modify(|_, w| w.lsbfirst().lsbfirst());

    let reset = gpioc.pc7.into_push_pull_output();
    let rdy_n = gpioa.pa9.into_pull_up_input();
    let mut req_n = gpioc.pc0.into_push_pull_output();
    req_n.set_high().ok();
    interrupt::free(|cs| {
        SPI_HANDLE.borrow(cs).replace(Some(spi1));
        RESET_PIN_HANDLE.borrow(cs).replace(Some(reset));
        RDY_PIN_HANDLE.borrow(cs).replace(Some(rdy_n));
        REQ_PIN_HANDLE.borrow(cs).replace(Some(req_n));
    });

    let systick = cp.SYST;
    let delay = hal::delay::Delay::new(systick, clocks);
    unsafe {
        DELAY_HANDLE = Some(delay);
        ITM_HANDLE = Some(itm);
    }
}
