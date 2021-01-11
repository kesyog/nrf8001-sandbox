#![no_std]
#![no_main]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

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
use core::{cell::RefCell, default::Default};
use cortex_m::{
    interrupt::{self, Mutex},
    iprintln,
};
use cortex_m_rt::entry;
use nrf8001_sys::{aci_evt_t, aci_pins_t, aci_state_t, hal_aci_evt_t};
use stm32f4xx_hal as hal;

const ECHO_DATA: [u8; 20] = [
    0x00, 0xaa, 0x55, 0xff, 0x77, 0x55, 0x33, 0x22, 0x11, 0x44, 0x66, 0x88, 0x99, 0xbb, 0xdd, 0xcc,
    0x00, 0xaa, 0x55, 0xff,
];
const NUM_ECHO_CMDS: u8 = 3;

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

    let mut aci_state = nrf8001_init();
    let mut aci_data = hal_aci_evt_t::default();
    unsafe {
        nrf8001_sys::lib_aci_init(&mut aci_state, false);
    }
    let mut n_aci_echo_cmds: u8 = 0;

    loop {
        // Dummy code just to link in the NRF SDK to test the linker
        // TODO: fill in real things
        let has_message = unsafe { nrf8001_sys::lib_aci_event_get(&mut aci_state, &mut aci_data) };
        if has_message {
            match aci_data.evt.evt_opcode {
                nrf8001_sys::aci_evt_opcode_t_ACI_EVT_DEVICE_STARTED => {
                    aci_device_started(&mut aci_state, &mut aci_data.evt, &mut n_aci_echo_cmds)
                }
                nrf8001_sys::aci_evt_opcode_t_ACI_EVT_CMD_RSP => aci_cmd_rsp(&mut aci_data.evt),
                nrf8001_sys::aci_evt_opcode_t_ACI_EVT_ECHO => {
                    aci_echo(&mut aci_data.evt, &mut n_aci_echo_cmds)
                }
                _ => {
                    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
                    iprintln!(
                        &mut itm.stim[0],
                        "Unexpected opcode: {}",
                        aci_data.evt.evt_opcode
                    );
                }
            };
        }
    }
}

fn nrf8001_init() -> aci_state_t {
    aci_state_t {
        aci_pins: aci_pins_t {
            reqn_pin: io_interface::Pin::ReqN as u8,
            rdyn_pin: io_interface::Pin::RdyN as u8,
            mosi_pin: io_interface::Pin::Mosi as u8,
            miso_pin: io_interface::Pin::Miso as u8,
            sck_pin: io_interface::Pin::Sck as u8,
            reset_pin: io_interface::Pin::Reset as u8,
            active_pin: io_interface::Pin::Unused as u8,
            optional_chip_sel_pin: io_interface::Pin::Unused as u8,
            ..Default::default()
        },
        ..Default::default()
    }
}

fn aci_device_started(
    aci_state: &mut aci_state_t,
    aci_evt: &mut aci_evt_t,
    n_aci_echo_cmds: &mut u8,
) {
    unsafe {
        aci_state.data_credit_available = aci_evt.params.device_started.credit_available;
    }
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "Device started opcode");
    match unsafe { aci_evt.params.device_started.device_mode } {
        nrf8001_sys::aci_device_operation_mode_t_ACI_DEVICE_SETUP => {
            iprintln!(&mut itm.stim[0], "Evt Device Started: Setup");
            unsafe {
                nrf8001_sys::lib_aci_test(
                    nrf8001_sys::aci_test_mode_change_t_ACI_TEST_MODE_DTM_UART,
                )
            };
        }
        nrf8001_sys::aci_device_operation_mode_t_ACI_DEVICE_STANDBY => {
            iprintln!(&mut itm.stim[0], "Evt Device Started: Standby");
        }
        nrf8001_sys::aci_device_operation_mode_t_ACI_DEVICE_TEST => {
            iprintln!(&mut itm.stim[0], "Evt Device Started: Test");
            iprintln!(&mut itm.stim[0], "Started infinite echo test");
            iprintln!(
                &mut itm.stim[0],
                "Repeat the test with all bytes in echo_data inverted."
            );
            iprintln!(
                &mut itm.stim[0],
                "Waiting 4 seconds before the test starts....."
            );
            delay::delay(4000);
            for _ in 0..NUM_ECHO_CMDS {
                unsafe {
                    nrf8001_sys::lib_aci_echo_msg(ECHO_DATA.len() as u8, &ECHO_DATA[0]);
                }
                *n_aci_echo_cmds += 1;
            }
        }
        _ => iprintln!(&mut itm.stim[0], "Unexpected device_mode: {}", unsafe {
            aci_evt.params.device_started.device_mode
        }),
    }
}

fn aci_cmd_rsp(aci_evt: &mut aci_evt_t) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "Response opcode");
    if unsafe {
        aci_evt.params.cmd_rsp.cmd_status != nrf8001_sys::aci_status_code_t_ACI_STATUS_SUCCESS
    } {
        iprintln!(&mut itm.stim[0], "ACI Command 0x{:X}", unsafe {
            aci_evt.params.cmd_rsp.cmd_opcode
        });
        panic!("Evt cmd response: error");
    }
}

fn aci_echo(aci_evt: &mut aci_evt_t, n_aci_echo_cmds: &mut u8) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "Echo opcode");
    let returned =
        unsafe { core::slice::from_raw_parts(&aci_evt.params.echo.echo_data[0], ECHO_DATA.len()) };
    if returned != ECHO_DATA {
        iprintln!(&mut itm.stim[0], "Echo mismatch: {:?}", returned);
    } else {
        iprintln!(&mut itm.stim[0], "Echo OK");
    }
    if NUM_ECHO_CMDS == *n_aci_echo_cmds {
        *n_aci_echo_cmds = 0;
        for _ in 0..NUM_ECHO_CMDS {
            unsafe {
                nrf8001_sys::lib_aci_echo_msg(ECHO_DATA.len() as u8, &ECHO_DATA[0]);
            }
            *n_aci_echo_cmds += 1;
        }
    }
}
