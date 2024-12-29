use crate::{
    command::{Command, CommandShift},
    error::FtdiError,
    request::{BitMode, Request, RequestReset},
};
use futures::{executor::block_on, future::join};
use nusb::{
    transfer::{Control, ControlType, Recipient, RequestBuffer},
    Interface,
};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub enum FtdiInterfaceEnum {
    A = 1,
    B = 2,
    C = 3,
    D = 4,
}
impl FtdiInterfaceEnum {
    const fn index(&self) -> u16 {
        *self as u16
    }
    pub const fn interface_number(&self) -> u8 {
        (self.index() - 1) as u8
    }
    const fn read_ep(&self) -> u8 {
        (self.index() + 0x80) as u8
    }
    const fn write_ep(&self) -> u8 {
        (self.index() * 2) as u8
    }
}

pub struct FtdiInterface {
    pub interface: Interface,
    index: FtdiInterfaceEnum,
    pub bitmode: BitMode,
    max_packet_size: usize,

    pub async_tx_buffer: Vec<u8>,
    pub async_response_len: usize,
}

impl FtdiInterface {
    pub fn new(
        interface: Interface,
        index: FtdiInterfaceEnum,
        bitmode: BitMode,
        max_packet_size: usize,
    ) -> Result<FtdiInterface, FtdiError> {
        let this = Self {
            interface,
            index,
            bitmode,
            max_packet_size,

            async_tx_buffer: Default::default(),
            async_response_len: 0,
        };
        this.reset()?;
        this.set_latency_timer(0xff)?;
        this.set_bitmode(BitMode::Mpsse)?;
        this.reset_rx()?;
        this.reset_tx()?;
        Ok(this)
    }

    pub fn write_request(&self, request: u8, value: u16, data: &[u8]) -> Result<usize, FtdiError> {
        println!("0x{request:x}-0x{value:x}");
        Ok(self
            .interface
            .control_out_blocking(
                Control {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: request,
                    value: value,
                    index: self.index.index() as u16,
                },
                data,
                Duration::from_millis(50),
            )
            .map_err(|e| FtdiError::TransferError(e))?)
    }
}

impl FtdiInterface {
    pub fn reset(&self) -> Result<(), FtdiError> {
        self.write_request(Request::Reset as u8, RequestReset::Reset as u16, &[])?;
        Ok(())
    }
    pub fn reset_rx(&self) -> Result<(), FtdiError> {
        self.write_request(
            Request::Reset as u8,
            RequestReset::PurgeRxBuffer as u16,
            &[],
        )?;
        Ok(())
    }
    pub fn reset_tx(&self) -> Result<(), FtdiError> {
        self.write_request(
            Request::Reset as u8,
            RequestReset::PurgeTxBuffer as u16,
            &[],
        )?;
        Ok(())
    }
    pub fn set_latency_timer(&self, time: u8) -> Result<(), FtdiError> {
        self.write_request(Request::SetLatencyTimer as u8, time as u16, &[])?;
        Ok(())
    }
    pub fn set_bitmode(&self, mode: BitMode) -> Result<(), FtdiError> {
        self.write_request(Request::SetBitMode as u8, (mode as u16) << 8, &[])?;
        Ok(())
    }
}

impl FtdiInterface {
    pub fn set_speed(&mut self, speed_hz: usize) -> Result<(), FtdiError> {
        let mut cmd = Vec::new();
        let mut base_speed = 30_000_000usize;
        if speed_hz < 6_000_000 {
            cmd.push(Command::EnableClkDivide5.into());
            base_speed = 6_000_000;
        } else {
            cmd.push(Command::DisableClkDivide5.into());
        }
        let mut div: u16 = 0;
        while speed_hz < base_speed {
            base_speed /= 2;
            div += 1;
        }
        cmd.push(Command::SetClkDivisor.into());
        cmd.push(div as u8);
        cmd.push((div >> 8) as u8);
        self.immidiate_write(&cmd, 0)?;
        Ok(())
    }
    pub fn common_setting(&mut self) -> Result<(), FtdiError> {
        // 85,97,8d
        // disable loopback
        // TurnOffnAdaptiveClocking
        // disable 3 phase
        self.immidiate_write(
            &[
                Command::DisLoopBack.into(),
                Command::DisableAdaptiveClocking.into(),
                Command::Disable3PhaseDataClock.into(),
            ],
            0,
        )?;
        Ok(())
    }
}

impl FtdiInterface {
    pub fn immidiate_write(
        &mut self,
        command: &[u8],
        response_len: usize,
    ) -> Result<Vec<u8>, FtdiError> {
        assert_eq!(self.async_tx_buffer.len(), 0);
        assert_eq!(self.async_response_len, 0);
        self.schedule_write_async(command, response_len);
        self.read_result_async()
    }
    pub fn schedule_write_async(&mut self, command: &[u8], response_len: usize) {
        // println!("cmd:{:x?}", command);
        self.async_tx_buffer.extend_from_slice(command);
        self.async_response_len += response_len
    }
    pub fn read_result_async(&mut self) -> Result<Vec<u8>, FtdiError> {
        let response_len = self.async_response_len;

        let tx_task = async {
            let time = Instant::now();
            let tx_buf_len = self.async_tx_buffer.len();
            let mut tx_finished_len = 0;
            while tx_finished_len < tx_buf_len {
                let this_package_len = if 512 < tx_buf_len - tx_finished_len {
                    512
                } else {
                    tx_buf_len - tx_finished_len
                };
                let tx_result = self
                    .interface
                    .bulk_out(
                        self.index.write_ep(),
                        self.async_tx_buffer[tx_finished_len..tx_finished_len + this_package_len]
                            .to_vec(),
                    )
                    .await
                    .into_result();
                if tx_result.is_err() {
                    return Err(FtdiError::TransferError(tx_result.unwrap_err()));
                }
                tx_finished_len += this_package_len;
                //println!("tx_finished_len:{tx_finished_len}");
                if time.elapsed().as_millis() > 1000 {
                    return Err(FtdiError::Timeout);
                }
            }
            self.async_tx_buffer.clear();
            Ok(())
        };

        let rx_task = async {
            let time = Instant::now();
            let mut response = Vec::new();
            while response.len() < response_len {
                let rx_data = self
                    .interface
                    .bulk_in(self.index.read_ep(), RequestBuffer::new(512))
                    .await
                    .into_result();
                match rx_data {
                    Ok(x) => {
                        if x[0] == 0xff {
                            return Err(FtdiError::CommandError(Command::from(x[1])));
                        }
                        response.extend_from_slice(&x[2..]);
                    }
                    Err(x) => {
                        return Err(FtdiError::TransferError(x));
                    }
                }
                //println!("response.len():{}",response.len());
                if time.elapsed().as_millis() > 1000 {
                    return Err(FtdiError::Timeout);
                }
            }
            self.async_response_len = 0;
            Ok(response)
        };
        let (tx_result, rx_result) = block_on(join(tx_task, rx_task));
        tx_result?;
        Ok(rx_result?)
    }
}
