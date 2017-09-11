extern crate byteorder;
extern crate spidev;

use std::io::Result;
use std::mem;

use spidev::{Spidev, SpidevTransfer};

pub const SPI_READ: u8 = 0x7f;
pub const SPI_WRITE: u8 = 0x80;

pub const REG_FIFO: u8 = 0x00;
pub const REG_OPMODE: u8 = 0x01;
pub const REG_DATAMODUL: u8 = 0x02;
pub const REG_BITRATE_MSB: u8 = 0x03;
pub const REG_BITRATE_LSB: u8 = 0x04;
pub const REG_FDEV_MSB: u8 = 0x05;
pub const REG_FDEV_LSB: u8 = 0x06;
pub const REG_FRF_MSB: u8 = 0x07;
pub const REG_FRF_MID: u8 = 0x08;
pub const REG_FRF_LSB: u8 = 0x09;
pub const REG_OSC1: u8 = 0x0A;
pub const REG_AFCCTRL: u8 = 0x0B;
pub const REG_LOWBAT: u8 = 0x0C;
pub const REG_LISTEN1: u8 = 0x0D;
pub const REG_LISTEN2: u8 = 0x0E;
pub const REG_LISTEN3: u8 = 0x0F;
pub const REG_VERSION: u8 = 0x10;
pub const REG_PALEVEL: u8 = 0x11;
pub const REG_PARAMP: u8 = 0x12;
pub const REG_OCP: u8 = 0x13;
pub const REG_LNA: u8 = 0x18;
pub const REG_RXBW: u8 = 0x19;
pub const REG_AFCBW: u8 = 0x1A;
pub const REG_OOKPEAK: u8 = 0x1B;
pub const REG_OOKAVG: u8 = 0x1C;
pub const REG_OOKFIX: u8 = 0x1D;
pub const REG_AFCFEI: u8 = 0x1E;
pub const REG_AFC_MSB: u8 = 0x1F;
pub const REG_AFC_LSB: u8 = 0x20;
pub const REG_FEI_MSB: u8 = 0x21;
pub const REG_FEI_LSB: u8 = 0x22;
pub const REG_RSSICONFIG: u8 = 0x23;
pub const REG_RSSIVALUE: u8 = 0x24;
pub const REG_DIOMAPPING1: u8 = 0x25;
pub const REG_DIOMAPPING2: u8 = 0x26;
pub const REG_IRQFLAGS1: u8 = 0x27;
pub const REG_IRQFLAGS2: u8 = 0x28;
pub const REG_RSSITHRESH: u8 = 0x29;
pub const REG_RXTIMEOUT1: u8 = 0x2A;
pub const REG_RXTIMEOUT2: u8 = 0x2B;
pub const REG_PREAMBLE_MSB: u8 = 0x2C;
pub const REG_PREAMBLE_LSB: u8 = 0x2D;
pub const REG_SYNCCONFIG: u8 = 0x2E;
pub const REG_SYNCVALUE1: u8 = 0x2F;
pub const REG_SYNCVALUE2: u8 = 0x30;
pub const REG_SYNCVALUE3: u8 = 0x31;
pub const REG_SYNCVALUE4: u8 = 0x32;
pub const REG_SYNCVALUE5: u8 = 0x33;
pub const REG_SYNCVALUE6: u8 = 0x34;
pub const REG_SYNCVALUE7: u8 = 0x35;
pub const REG_SYNCVALUE8: u8 = 0x36;
pub const REG_PACKETCONFIG1: u8 = 0x37;
pub const REG_PAYLOADLENGTH: u8 = 0x38;
pub const REG_NODEADRS: u8 = 0x39;
pub const REG_BROADCASTADRS: u8 = 0x3A;
pub const REG_AUTOMODES: u8 = 0x3B;
pub const REG_FIFOTHRESH: u8 = 0x3C;
pub const REG_PACKETCONFIG2: u8 = 0x3D;
pub const REG_AESKEY1: u8 = 0x3E;
pub const REG_AESKEY2: u8 = 0x3F;
pub const REG_AESKEY3: u8 = 0x40;
pub const REG_AESKEY4: u8 = 0x41;
pub const REG_AESKEY5: u8 = 0x42;
pub const REG_AESKEY6: u8 = 0x43;
pub const REG_AESKEY7: u8 = 0x44;
pub const REG_AESKEY8: u8 = 0x45;
pub const REG_AESKEY9: u8 = 0x46;
pub const REG_AESKEY10: u8 = 0x47;
pub const REG_AESKEY11: u8 = 0x48;
pub const REG_AESKEY12: u8 = 0x49;
pub const REG_AESKEY13: u8 = 0x4A;
pub const REG_AESKEY14: u8 = 0x4B;
pub const REG_AESKEY15: u8 = 0x4C;
pub const REG_AESKEY16: u8 = 0x4D;
pub const REG_TEMP1: u8 = 0x4E;
pub const REG_TEMP2: u8 = 0x4F;
pub const REG_TESTLNA: u8 = 0x58;
pub const REG_TESTTCXO: u8 = 0x59;
pub const REG_TESTLLBW: u8 = 0x5F;
pub const REG_TESTDAGC: u8 = 0x6F;
pub const REG_TESTAFC: u8 = 0x71;

#[repr(u8)]
pub enum OperatingMode {
    Sleep = 0,
    Standby = 1,
    FrequencySynthesizer = 2,
    Transmitter = 3,
    Receiver = 4,
}

pub struct OpModeConfig {
    pub sequencer: bool,
    pub listen: bool,
    pub listen_abort: bool,
    pub mode: OperatingMode,
}

impl Into<u8> for OpModeConfig {
    fn into(self) -> u8 {
        ((self.sequencer as u8) << 7) | ((self.listen as u8) << 6) |
            ((self.listen_abort as u8) << 5) | ((self.mode as u8 & 0b111) << 2)
    }
}

impl From<u8> for OpModeConfig {
    fn from(raw: u8) -> Self {
        OpModeConfig {
            sequencer: (raw >> 7) & 0b1 == 1,
            listen: (raw >> 6) & 0b1 == 1,
            listen_abort: (raw >> 5) & 0b1 == 1,
            mode: unsafe { mem::transmute((raw >> 2) & 0b111) },
        }
    }
}

impl Default for OpModeConfig {
    fn default() -> Self {
        OpModeConfig {
            sequencer: false,
            listen: false,
            listen_abort: false,
            mode: OperatingMode::Sleep,
        }
    }
}

#[repr(u8)]
pub enum DataModulationMode {
    Packet = 0,
    ContinuousWithSync = 2,
    Continuous = 3,
}

pub enum DataModulation {
    FSK(FSKShaping),
    OOK(OOKShaping),
}

#[repr(u8)]
pub enum FSKShaping {
    None = 0,
    // BT = 1.0
    GaussianFilter10 = 1,
    // BT = 0.5
    GaussianFilter05 = 2,
    // BT = 0.3
    GaussianFilter03 = 3,
}

#[repr(u8)]
pub enum OOKShaping {
    None = 0,
    // f_cutoff = BR
    BR = 1,
    // f_cutoff = 2*BR
    BR2 = 2,
}

pub struct DataModulationConfig {
    pub data_mode: DataModulationMode,
    pub modulation_type: DataModulation,
}

impl Into<u8> for DataModulationConfig {
    fn into(self) -> u8 {
        ((self.data_mode as u8) << 5) |
            match self.modulation_type {
                DataModulation::FSK(shaping) => 0b01000 | (shaping as u8),
                DataModulation::OOK(shaping) => 0b01001 | (shaping as u8),
            }
    }
}

impl From<u8> for DataModulationConfig {
    fn from(raw: u8) -> Self {
        DataModulationConfig {
            data_mode: unsafe { mem::transmute((raw >> 5) & 0b11) },
            modulation_type: match (raw >> 3) & 0b11 {
                0 => DataModulation::FSK(match raw & 0b11 {
                    0 => FSKShaping::None,
                    1 => FSKShaping::GaussianFilter03,
                    2 => FSKShaping::GaussianFilter10,
                    _ => unimplemented!(),
                }),
                1 => DataModulation::OOK(match raw & 0b11 {
                    0 => OOKShaping::None,
                    1 => OOKShaping::BR,
                    2 => OOKShaping::BR2,
                    _ => unimplemented!(),
                }),
                _ => unimplemented!(),
            },
        }
    }
}

impl Default for DataModulationConfig {
    fn default() -> Self {
        DataModulationConfig {
            data_mode: DataModulationMode::Packet,
            modulation_type: DataModulation::FSK(FSKShaping::None),
        }
    }
}

pub struct Osc1Config {
    pub cal_start: bool,
    pub cal_done: bool,
}

impl Into<u8> for Osc1Config {
    fn into(self) -> u8 {
        ((self.cal_start as u8) << 7) | ((self.cal_done as u8) << 6)
    }
}

impl From<u8> for Osc1Config {
    fn from(raw: u8) -> Self {
        Osc1Config {
            cal_start: (raw >> 7) & 0b01 == 1,
            cal_done: (raw >> 6) & 0b01 == 1,
        }
    }
}

impl Default for Osc1Config {
    fn default() -> Self {
        Osc1Config {
            cal_start: false,
            cal_done: true,
        }
    }
}

pub struct AfcCtrlConfig {
    /// Improved AFC routine for signals with modulation index
    /// lower than 2. Refer to section 3.5.16 for details.
    pub low_beta: bool,
}

impl Into<u8> for AfcCtrlConfig {
    fn into(self) -> u8 {
        (self.low_beta as u8) << 5
    }
}

impl From<u8> for AfcCtrlConfig {
    fn from(raw: u8) -> Self {
        AfcCtrlConfig { low_beta: (raw >> 5) & 0b01 == 1 }
    }
}

impl Default for AfcCtrlConfig {
    fn default() -> Self {
        AfcCtrlConfig { low_beta: false }
    }
}

#[repr(u8)]
pub enum LowBatThreshold {
    V1695 = 0,
    V1764 = 1,
    V1835 = 2,
    V1905 = 3,
    V1976 = 4,
    V2045 = 5,
    V2116 = 6,
    V2185 = 7,
}

// RegLowBat
pub struct LowBatConfig {
    pub monitor: bool,
    pub on: bool,
    pub trim: LowBatThreshold,
}

impl Into<u8> for LowBatConfig {
    fn into(self) -> u8 {
        ((self.on as u8) << 3) | ((self.trim as u8) & 0b111)
    }
}

impl From<u8> for LowBatConfig {
    fn from(raw: u8) -> Self {
        unsafe {
            LowBatConfig {
                monitor: (raw >> 4) & 0b1 == 1,
                on: (raw >> 3) & 0b1 == 1,
                trim: mem::transmute(raw & 0b111),
            }
        }
    }
}

impl Default for LowBatConfig {
    fn default() -> Self {
        LowBatConfig {
            monitor: false,
            on: false,
            trim: LowBatThreshold::V1835,
        }
    }
}

#[repr(u8)]
pub enum ListenModeTimeResolution {
    /// 64 microseconds
    Low = 1,
    /// 4.1 milliseconds
    Medium = 2,
    /// 262 milliseconds
    High = 3,
}

#[repr(u8)]
pub enum ListenEndAction {
    /// Chip stays in Rx mode. Listen mode stops and must be disabled
    Stop = 0,
    /// Chip stays in Rx mode until *PayloadReady* or *Timeout* interrupt occurs.
    /// It then goes to the mode defined by *Mode*. Listen mode stops and must be
    /// disabled.
    GoToDefined = 1,
    /// Chip stays in Rx mode until *PayloadReady* or *Timeout* interrupt occurs.
    /// Listen mode then resumes in Idle state.
    /// FIFO content is lost at next Rx wakeup.
    Idle = 2,
}

pub struct Listen1Config {
    pub idle_time: ListenModeTimeResolution,
    pub rx_time: ListenModeTimeResolution,
    /// Match SyncAddress
    pub criteria: bool,
    pub end_action: ListenEndAction,
}

impl From<u8> for Listen1Config {
    fn from(raw: u8) -> Self {
        unsafe {
            Listen1Config {
                idle_time: mem::transmute((raw >> 6) & 0b11),
                rx_time: mem::transmute((raw >> 4) & 0b11),
                criteria: (raw >> 3) & 0b1 == 1,
                end_action: mem::transmute((raw >> 1) & 0b11),
            }
        }
    }
}

impl Into<u8> for Listen1Config {
    fn into(self) -> u8 {
        ((self.idle_time as u8) << 6) | ((self.rx_time as u8) << 4) |
            ((self.criteria as u8) << 3) | ((self.end_action as u8) << 1)
    }
}

impl Default for Listen1Config {
    fn default() -> Self {
        Listen1Config {
            idle_time: ListenModeTimeResolution::Medium,
            rx_time: ListenModeTimeResolution::Low,
            criteria: false,
            end_action: ListenEndAction::GoToDefined,
        }
    }
}

// SPI Register access: Pg 31

pub struct RFM69 {
    dev: Spidev,
}

impl RFM69 {
    pub fn new(dev: Spidev) -> Result<Self> {
        Ok(RFM69 { dev })
    }
}


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {}
}
