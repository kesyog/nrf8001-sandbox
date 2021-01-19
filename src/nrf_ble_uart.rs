use crate::delay;
use crate::io_interface::IOPin;
use crate::nrf_ble_uart_consts::{
    BleUartAciPipe, UartControlCmd, SERVICES_PIPE_TYPE_MAPPING, SETUP_MESSAGES,
};
use crate::ITM_HANDLE;
use core::ptr;
use cortex_m::{iprint, iprintln};
use nrf8001_sys::{
    aci_cmd_opcode_t_ACI_CMD_GET_DEVICE_VERSION, aci_device_operation_mode_t_ACI_DEVICE_SETUP,
    aci_device_operation_mode_t_ACI_DEVICE_STANDBY, aci_evt_opcode_t_ACI_EVT_CMD_RSP,
    aci_evt_opcode_t_ACI_EVT_CONNECTED, aci_evt_opcode_t_ACI_EVT_DATA_CREDIT,
    aci_evt_opcode_t_ACI_EVT_DATA_RECEIVED, aci_evt_opcode_t_ACI_EVT_DEVICE_STARTED,
    aci_evt_opcode_t_ACI_EVT_DISCONNECTED, aci_evt_opcode_t_ACI_EVT_HW_ERROR,
    aci_evt_opcode_t_ACI_EVT_PIPE_ERROR, aci_evt_opcode_t_ACI_EVT_PIPE_STATUS,
    aci_evt_opcode_t_ACI_EVT_TIMING, aci_hw_error_t_ACI_HW_ERROR_NONE, aci_ll_conn_params_t,
    aci_pins_t, aci_setup_info_t, aci_setup_return_t_SETUP_SUCCESS, aci_state_t,
    aci_status_code_t_ACI_STATUS_ERROR_PEER_ATT_ERROR, aci_status_code_t_ACI_STATUS_SUCCESS,
    hal_aci_data_t, hal_aci_evt_t, services_pipe_type_mapping_t,
};
use num_traits::FromPrimitive;

#[derive(Default)]
pub struct Context {
    aci_state: aci_state_t,
    aci_data: hal_aci_evt_t,
    setup_required: bool,
    timing_change_done: bool,
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
                aci_setup_info: aci_setup_info_t {
                    services_pipe_type_mapping: &SERVICES_PIPE_TYPE_MAPPING
                        as *const services_pipe_type_mapping_t,
                    number_of_pipes: BleUartAciPipe::count(),
                    setup_msgs: &SETUP_MESSAGES as *const hal_aci_data_t,
                    num_setup_msgs: SETUP_MESSAGES.len() as u8,
                },
                ..Default::default()
            },
            ..Default::default()
        };

        unsafe {
            nrf8001_sys::lib_aci_init(&mut new_context.aci_state, false);
        }

        new_context
    }
}

fn uart_process_control_point_rx(aci_state: &mut aci_state_t, data: &[u8]) -> bool {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    if unsafe {
        nrf8001_sys::lib_aci_is_pipe_available(
            aci_state,
            BleUartAciPipe::PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_TX as u8,
        )
    } {
        match UartControlCmd::from_u8(data[0]) {
            // Queues a ACI Disconnect to the nRF8001 when this packet is received.
            // May cause some of the UART packets being sent to be dropped
            Some(UartControlCmd::Disconnect) => {
                iprintln!(&mut itm.stim[0], "💣 Peer requested disconnect");
                unsafe {
                    nrf8001_sys::lib_aci_disconnect(
                        aci_state,
                        nrf8001_sys::aci_disconnect_reason_t_ACI_REASON_TERMINATE,
                    )
                };
                true
            }

            // Queues an ACI Change Timing to the nRF8001
            Some(UartControlCmd::LinkTimingReq) => {
                iprintln!(&mut itm.stim[0], "Peer requested timing change");
                let conn_params_bytes = &data[1..];
                assert!(conn_params_bytes.len() >= core::mem::size_of::<aci_ll_conn_params_t>());
                let conn_params = unsafe {
                    ptr::read_unaligned(
                        &conn_params_bytes[0] as *const u8 as *const aci_ll_conn_params_t,
                    )
                };
                unsafe {
                    nrf8001_sys::lib_aci_change_timing(
                        conn_params.min_conn_interval,
                        conn_params.max_conn_interval,
                        conn_params.slave_latency,
                        conn_params.timeout_mult,
                    )
                };
                true
            }

            Some(UartControlCmd::TransmitStop) | Some(UartControlCmd::TransmitOk) => true,

            _ => false,
        }
    } else {
        false
    }
}

fn aci_device_started(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    let payload = unsafe { ctx.aci_data.evt.params.device_started };
    ctx.aci_state.data_credit_available = payload.credit_available;

    match payload.device_mode {
        aci_device_operation_mode_t_ACI_DEVICE_SETUP => {
            iprintln!(&mut itm.stim[0], "Evt Device Started: Setup");
            ctx.setup_required = true;
        }
        aci_device_operation_mode_t_ACI_DEVICE_STANDBY => {
            iprintln!(&mut itm.stim[0], "Evt Device Started: Standby");
            //Looking for an iPhone by sending radio advertisements
            //When an iPhone connects to us we will get an ACI_EVT_CONNECTED event from the nRF8001
            if payload.hw_error != aci_hw_error_t_ACI_HW_ERROR_NONE {
                iprintln!(&mut itm.stim[0], "Delaying for iPhone?");
                delay::delay(20); //Magic number used to make sure the HW error event is handled correctly.
            } else {
                // Set advertising interval to 50ms
                unsafe { nrf8001_sys::lib_aci_connect(180, 0x0050) };
                iprintln!(&mut itm.stim[0], "Advertising started");
            }
        }
        _ => iprintln!(
            &mut itm.stim[0],
            "Evt Device started: Unhandled device_mode: {}",
            payload.device_mode
        ),
    }
}

fn aci_cmd_rsp(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    let payload = unsafe { ctx.aci_data.evt.params.cmd_rsp };
    if aci_status_code_t_ACI_STATUS_SUCCESS != payload.cmd_status {
        iprintln!(&mut itm.stim[0], "ACI Command 0x{:X}", payload.cmd_opcode);
        panic!("Evt cmd response: error");
    }
    if aci_cmd_opcode_t_ACI_CMD_GET_DEVICE_VERSION == payload.cmd_opcode {
        let device_version = unsafe { payload.params.get_device_version };
        iprintln!(&mut itm.stim[0], "Get version");
        //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
        unsafe {
            nrf8001_sys::lib_aci_set_local_data(
                &ctx.aci_state,
                BleUartAciPipe::PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET as u8,
                &device_version as *const nrf8001_sys::aci_evt_cmd_rsp_params_get_device_version_t
                    as *const u8,
                core::mem::size_of_val(&device_version) as u8,
            )
        };
    }
}

fn aci_cmd_connected(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "🔌 Connected!");
    ctx.timing_change_done = false;

    unsafe { nrf8001_sys::lib_aci_device_version() };
}

fn aci_cmd_disconnected(_: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "⛔ Disconnected! Restarting advertising");

    unsafe { nrf8001_sys::lib_aci_connect(180, 0x100) };
}

fn aci_cmd_pipe_status(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    iprintln!(&mut itm.stim[0], "Pipe status evt");
    if unsafe {
        nrf8001_sys::lib_aci_is_pipe_available(
            &ctx.aci_state,
            BleUartAciPipe::PIPE_UART_OVER_BTLE_UART_TX_TX as u8,
        )
    } && !ctx.timing_change_done
    {
        iprintln!(&mut itm.stim[0], "Requesting timing change");
        // change the timing on the link as specified in the nRFgo studio -> nRF8002 conf. -> GAP.
        // Used to increase or decrease bandwidth
        unsafe { nrf8001_sys::lib_aci_change_timing_GAP_PPCP() };
        ctx.timing_change_done = true;
    }
}

fn aci_cmd_timing(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    let payload = unsafe { ctx.aci_data.evt.params.timing };
    iprintln!(&mut itm.stim[0], "Conn interval changed");
    unsafe {
        iprintln!(
            &mut itm.stim[0],
            "Interval: {} Slave latency: {} Timeout: {}",
            ptr::read_unaligned(&payload.conn_rf_interval),
            ptr::read_unaligned(&payload.conn_slave_rf_latency),
            ptr::read_unaligned(&payload.conn_rf_timeout)
        )
    };

    unsafe {
        nrf8001_sys::lib_aci_set_local_data(
            &ctx.aci_state,
            BleUartAciPipe::PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET as u8,
            &payload.conn_rf_interval as *const u16 as *const u8, // byte-aligned
            BleUartAciPipe::PIPE_UART_OVER_BTLE_UART_LINK_TIMING_CURRENT_SET.max_size(),
        )
    };
}

fn aci_cmd_data_received(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    let payload = unsafe { ctx.aci_data.evt.params.data_received };
    iprintln!(
        &mut itm.stim[0],
        "{} data bytes received on pipe {}",
        ctx.aci_data.evt.len - 2,
        payload.rx_data.pipe_number
    );
    if BleUartAciPipe::PIPE_UART_OVER_BTLE_UART_RX_RX as u8 == payload.rx_data.pipe_number {
        let rx_data = &payload.rx_data.aci_data[..(ctx.aci_data.evt.len - 2) as usize];
        iprint!(&mut itm.stim[0], "Data: ");
        for byte in rx_data {
            iprint!(&mut itm.stim[0], "{:02X}", byte);
        }
        iprint!(&mut itm.stim[0], "\r\n");
        // TODO: copy to uart buffer
    } else if BleUartAciPipe::PIPE_UART_OVER_BTLE_UART_CONTROL_POINT_RX as u8
        == payload.rx_data.pipe_number
    {
        let rx_data = &payload.rx_data.aci_data[..(ctx.aci_data.evt.len - 2) as usize];
        uart_process_control_point_rx(&mut ctx.aci_state, rx_data); //Subtract for Opcode and Pipe number
    }
}

fn aci_cmd_data_credit(ctx: &mut Context) {
    let payload = unsafe { ctx.aci_data.evt.params.data_credit };
    ctx.aci_state.data_credit_available += payload.credit;
}

fn aci_cmd_pipe_error(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    let payload = unsafe { ctx.aci_data.evt.params.pipe_error };
    iprintln!(
        &mut itm.stim[0],
        "Pipe Error evt: Pipe #:{} Code:0x{:02X}",
        payload.pipe_number,
        payload.error_code
    );

    //Increment the credit available as the data packet was not sent.
    //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
    //for the credit.
    if aci_status_code_t_ACI_STATUS_ERROR_PEER_ATT_ERROR != payload.error_code {
        ctx.aci_state.data_credit_available += 1;
    }
}

fn aci_cmd_hw_error(ctx: &mut Context) {
    let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
    let payload = unsafe { ctx.aci_data.evt.params.hw_error };
    iprintln!(&mut itm.stim[0], "💣 HW error: {}", unsafe {
        ptr::read_unaligned(&payload.line_num)
    });
    unsafe { nrf8001_sys::lib_aci_connect(180, 0x0050) };
    iprintln!(&mut itm.stim[0], "Restarting advertising");
}

pub fn poll(ctx: &mut Context) {
    let has_message =
        unsafe { nrf8001_sys::lib_aci_event_get(&mut ctx.aci_state, &mut ctx.aci_data) };
    if has_message {
        match ctx.aci_data.evt.evt_opcode {
            aci_evt_opcode_t_ACI_EVT_DEVICE_STARTED => aci_device_started(ctx),
            aci_evt_opcode_t_ACI_EVT_CMD_RSP => aci_cmd_rsp(ctx),
            aci_evt_opcode_t_ACI_EVT_CONNECTED => aci_cmd_connected(ctx),
            aci_evt_opcode_t_ACI_EVT_DISCONNECTED => aci_cmd_disconnected(ctx),
            aci_evt_opcode_t_ACI_EVT_PIPE_STATUS => aci_cmd_pipe_status(ctx),
            aci_evt_opcode_t_ACI_EVT_TIMING => aci_cmd_timing(ctx),
            aci_evt_opcode_t_ACI_EVT_DATA_RECEIVED => aci_cmd_data_received(ctx),
            aci_evt_opcode_t_ACI_EVT_DATA_CREDIT => aci_cmd_data_credit(ctx),
            aci_evt_opcode_t_ACI_EVT_PIPE_ERROR => aci_cmd_pipe_error(ctx),
            aci_evt_opcode_t_ACI_EVT_HW_ERROR => aci_cmd_hw_error(ctx),

            _ => {
                let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
                iprintln!(
                    &mut itm.stim[0],
                    "Unhandled evt opcode: {}",
                    ctx.aci_data.evt.evt_opcode
                );
            }
        };
    }

    if ctx.setup_required
        && unsafe { nrf8001_sys::do_aci_setup(&mut ctx.aci_state) }
            == aci_setup_return_t_SETUP_SUCCESS
    {
        let itm = unsafe { ITM_HANDLE.as_mut().unwrap() };
        iprintln!(&mut itm.stim[0], "Doing device setup");
        const DEVICE_NAME: [u8; 3] = [b'K', b'e', b's'];
        unsafe {
            let ret = nrf8001_sys::lib_aci_set_local_data(
                &ctx.aci_state,
                BleUartAciPipe::PIPE_GAP_DEVICE_NAME_SET as u8,
                &DEVICE_NAME as *const u8,
                DEVICE_NAME.len() as u8,
            );
            assert_eq!(true, ret);
        };
        unsafe {
            let ret = nrf8001_sys::lib_aci_set_local_data(
                &ctx.aci_state,
                BleUartAciPipe::PIPE_DEVICE_INFORMATION_MANUFACTURER_NAME_STRING_SET as u8,
                &DEVICE_NAME as *const u8,
                DEVICE_NAME.len() as u8,
            );
            assert_eq!(true, ret);
        };
        ctx.setup_required = false;
    }
}
