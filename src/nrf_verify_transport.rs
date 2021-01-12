/// Test project to verify SPI/ACI by sending echo commands to the nRF8001
/// This is a port of the ble_aci_transport_layer_verification project from the Nordic SDK

use crate::delay;
use crate::io_interface::IOPin;
use crate::ITM_HANDLE;
use cortex_m::iprintln;
use nrf8001_sys::{
    aci_device_operation_mode_t_ACI_DEVICE_SETUP, aci_device_operation_mode_t_ACI_DEVICE_STANDBY,
    aci_device_operation_mode_t_ACI_DEVICE_TEST, aci_evt_opcode_t_ACI_EVT_CMD_RSP,
    aci_evt_opcode_t_ACI_EVT_DEVICE_STARTED, aci_evt_opcode_t_ACI_EVT_ECHO, aci_evt_t, aci_pins_t,
    aci_state_t, aci_status_code_t_ACI_STATUS_SUCCESS,
    aci_test_mode_change_t_ACI_TEST_MODE_DTM_UART, hal_aci_evt_t,
};

const ECHO_DATA: [u8; 20] = [
    0x00, 0xaa, 0x55, 0xff, 0x77, 0x55, 0x33, 0x22, 0x11, 0x44, 0x66, 0x88, 0x99, 0xbb, 0xdd, 0xcc,
    0x00, 0xaa, 0x55, 0xff,
];
const NUM_ECHO_CMDS: u8 = 3;

#[derive(Default)]
pub struct Context {
    aci_state: aci_state_t,
    aci_data: hal_aci_evt_t,
    n_aci_echo_cmds: u8,
}

impl Context {
    pub fn new() -> Self {
        let mut new_context = Self {
            aci_state: aci_state_t {
                aci_pins: aci_pins_t {
                    reqn_pin: IOPin::ReqN as u8,
                    rdyn_pin: IOPin::RdyN as u8,
                    mosi_pin: IOPin::Mosi as u8,
                    miso_pin: IOPin::Miso as u8,
                    sck_pin: IOPin::Sck as u8,
                    reset_pin: IOPin::Reset as u8,
                    active_pin: IOPin::Unused as u8,
                    optional_chip_sel_pin: IOPin::Unused as u8,
                    ..Default::default()
                },
                ..Default::default()
            },
            ..Default::default()
        };

        unsafe {
            nrf8001_sys::hal_aci_tl_init(&mut new_context.aci_state.aci_pins, false);
        }

        new_context
    }
}

pub fn poll(ctx: &mut Context) {
    let has_message =
        unsafe { nrf8001_sys::lib_aci_event_get(&mut ctx.aci_state, &mut ctx.aci_data) };
    if has_message {
        match ctx.aci_data.evt.evt_opcode {
            aci_evt_opcode_t_ACI_EVT_DEVICE_STARTED => aci_device_started(
                &mut ctx.aci_state,
                &mut ctx.aci_data.evt,
                &mut ctx.n_aci_echo_cmds,
            ),
            aci_evt_opcode_t_ACI_EVT_CMD_RSP => aci_cmd_rsp(&mut ctx.aci_data.evt),
            aci_evt_opcode_t_ACI_EVT_ECHO => {
                aci_echo(&mut ctx.aci_data.evt, &mut ctx.n_aci_echo_cmds)
            }
            _ => {
                let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
                iprintln!(
                    &mut itm.stim[0],
                    "Unexpected opcode: {}",
                    ctx.aci_data.evt.evt_opcode
                );
            }
        };
    }
}

fn send_echo_cmd(n_aci_echo_cmds: &mut u8) {
    for _ in 0..NUM_ECHO_CMDS {
        unsafe {
            nrf8001_sys::lib_aci_echo_msg(ECHO_DATA.len() as u8, &ECHO_DATA[0]);
        }
        *n_aci_echo_cmds += 1;
    }
}

fn aci_device_started(
    aci_state: &mut aci_state_t,
    aci_evt: &mut aci_evt_t,
    n_aci_echo_cmds: &mut u8,
) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    aci_state.data_credit_available = unsafe { aci_evt.params.device_started.credit_available };
    iprintln!(&mut itm.stim[0], "Device started opcode");
    match unsafe { aci_evt.params.device_started.device_mode } {
        aci_device_operation_mode_t_ACI_DEVICE_SETUP => {
            iprintln!(&mut itm.stim[0], "Evt Device Started: Setup");
            unsafe { nrf8001_sys::lib_aci_test(aci_test_mode_change_t_ACI_TEST_MODE_DTM_UART) };
        }
        aci_device_operation_mode_t_ACI_DEVICE_STANDBY => {
            iprintln!(&mut itm.stim[0], "Evt Device Started: Standby");
        }
        aci_device_operation_mode_t_ACI_DEVICE_TEST => {
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
            send_echo_cmd(n_aci_echo_cmds);
        }
        _ => iprintln!(&mut itm.stim[0], "Unexpected device_mode: {}", unsafe {
            aci_evt.params.device_started.device_mode
        }),
    }
}

fn aci_cmd_rsp(aci_evt: &mut aci_evt_t) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "Response opcode");
    if unsafe { aci_evt.params.cmd_rsp.cmd_status != aci_status_code_t_ACI_STATUS_SUCCESS } {
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
        send_echo_cmd(n_aci_echo_cmds);
    }
}
