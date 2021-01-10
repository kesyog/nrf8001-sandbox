#![no_std]
#![no_main]

mod delay;
mod io_interface;

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

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
#[allow(unused_extern_crates)]
use cortex_m::{
    interrupt::{self, Mutex},
    iprintln,
};
use cortex_m_rt::entry;
use nrf8001_sys::{
    aci_device_operation_mode_t, aci_pins_t, aci_setup_info_t, aci_state_t, hal_aci_evt_t,
};
use stm32f4xx_hal as hal;

static DELAY_HANDLE: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));
static SPI_HANDLE: Mutex<
    RefCell<
        Option<
            spi::Spi<
                hal::stm32::SPI1,
                (
                    PA5<gpio::Alternate<AF5>>,
                    PA6<gpio::Alternate<AF5>>,
                    PA7<gpio::Alternate<AF5>>,
                ),
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));
// TODO: decide whether to prefer unsafe vs. Mutex vs. something else
// Global singletons are unsafe since there's only one thread and interrupts are unused
static mut RESET_PIN_HANDLE: Option<PC7<gpio::Output<PushPull>>> = None;
static mut RDY_PIN_HANDLE: Option<PA9<gpio::Input<PullUp>>> = None;
static mut REQ_PIN_HANDLE: Option<PC0<gpio::Output<PushPull>>> = None;
static mut ITM_HANDLE: Option<stm32::ITM> = None;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;
    iprintln!(&mut itm.stim[0], "Boot");

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();
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
        // Works but seems out of spec of the OLED datasheet ¯\_(ツ)_/¯
        1.mhz().into(),
        clocks,
    );
    interrupt::free(|cs| {
        SPI_HANDLE.borrow(cs).replace(Some(spi1));
    });

    let reset = gpioc.pc7.into_push_pull_output();
    let rdy_n = gpioa.pa9.into_pull_up_input();
    let mut req_n = gpioc.pc0.into_push_pull_output();
    req_n.set_high().ok();
    unsafe {
        ITM_HANDLE = Some(itm);
        RESET_PIN_HANDLE = Some(reset);
        RDY_PIN_HANDLE = Some(rdy_n);
        REQ_PIN_HANDLE = Some(req_n);
    }

    let systick = cp.SYST;
    interrupt::free(|cs| {
        DELAY_HANDLE
            .borrow(cs)
            .replace(Some(hal::delay::Delay::new(systick, clocks)));
    });

    // TODO: ensure timing of lines matches behavior in HAL init

    let mut aci_state = aci_state_t::default();
    let mut aci_data = hal_aci_evt_t::default();
    unsafe {
        nrf8001_sys::lib_aci_init(&mut aci_state, false);
    }

    loop {
        unsafe {
            // Dummy code just to link in the NRF SDK to test the linker
            // TODO: fill in real things
            nrf8001_sys::do_aci_setup(&mut aci_state);
            nrf8001_sys::lib_aci_event_get(&mut aci_state, &mut aci_data);
            nrf8001_sys::lib_aci_test(nrf8001_sys::aci_test_mode_change_t_ACI_TEST_MODE_DTM_UART);
        }
    }
}
