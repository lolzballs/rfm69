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

pub struct PowerAmplifierLevel {
    pub pa0: bool,
    pub pa1: bool,
    pub pa2: bool,
    /// Output power settings, with 1 dB steps. 5-bit value.
    /// P_out = -18 + *OutputPower* [dBm], with PA0 or PA1.
    /// P_out = -14 + *OutputPower* [dBm], with PA1 or PA2.
    /// (limited to the 16 upper values of *OutputPower*)
    pub output_power: u8,
}

impl From<u8> for PowerAmplifierLevel {
    fn from(raw: u8) -> Self {
        PowerAmplifierLevel {
            pa0: (raw >> 7) & 0b1 == 1,
            pa1: (raw >> 6) & 0b1 == 1,
            pa2: (raw >> 5) & 0b1 == 1,
            output_power: raw & 0b11111,
        }
    }
}

impl Into<u8> for PowerAmplifierLevel {
    fn into(self) -> u8 {
        ((self.pa0 as u8) << 7) | ((self.pa1 as u8) << 6) | ((self.pa2 as u8) << 5) |
            (self.output_power & 0b11111)
    }
}

impl Default for PowerAmplifierLevel {
    fn default() -> Self {
        PowerAmplifierLevel {
            pa0: true,
            pa1: false,
            pa2: false,
            output_power: 0b11111,
        }
    }
}

#[repr(u8)]
pub enum PowerAmplifierRamp {
    US3400 = 0b0000,
    US2000 = 0b0001,
    US1000 = 0b0010,
    US500 = 0b0011,
    US250 = 0b0100,
    US125 = 0b0101,
    US100 = 0b0110,
    US62 = 0b0111,
    US50 = 0b1000,
    US40 = 0b1001,
    US31 = 0b1010,
    US25 = 0b1011,
    US20 = 0b1100,
    US15 = 0b1101,
    US12 = 0b1110,
    US10 = 0b1111,
}

impl From<u8> for PowerAmplifierRamp {
    fn from(raw: u8) -> Self {
        unsafe { mem::transmute(raw) }
    }
}

impl Into<u8> for PowerAmplifierRamp {
    fn into(self) -> u8 {
        self as u8
    }
}

impl Default for PowerAmplifierRamp {
    fn default() -> Self {
        PowerAmplifierRamp::US125
    }
}

pub struct OverloadCurrentProtection {
    pub on: bool,
    /// Trimming of OCP current:
    /// I_max = 45 + 5 * OcpTrim(mA)
    pub trim: u8,
}

impl From<u8> for OverloadCurrentProtection {
    fn from(raw: u8) -> Self {
        OverloadCurrentProtection {
            on: (raw >> 4) & 0b1 == 1,
            trim: raw & 0b1111,
        }
    }
}

impl Into<u8> for OverloadCurrentProtection {
    fn into(self) -> u8 {
        ((self.on as u8) << 4) | (self.trim & 0b1111)
    }
}

impl Default for OverloadCurrentProtection {
    fn default() -> Self {
        OverloadCurrentProtection {
            on: true,
            trim: 0b1010,
        }
    }
}

#[repr(u8)]
pub enum LNAImpedance {
    Ohms50 = 0,
    Ohms200 = 1,
}

#[repr(u8)]
pub enum LNAGainSelect {
    AGC = 0b000,
    G1 = 0b001,
    G2 = 0b010,
    G3 = 0b011,
    G4 = 0b100,
    G5 = 0b101,
    G6 = 0b110,
}

pub struct LowNoiseAmplifier {
    input_impedance: LNAImpedance,
    /// Current LNA gain, set either manually, or by the AGC
    current_gain: u8,
    /// LNA gain setting
    gain_select: LNAGainSelect,
}

impl From<u8> for LowNoiseAmplifier {
    fn from(raw: u8) -> Self {
        LowNoiseAmplifier {
            input_impedance: unsafe { mem::transmute(raw >> 7) },
            current_gain: (raw >> 3) & 0b111,
            gain_select: unsafe { mem::transmute(raw & 0b111) },
        }
    }
}

impl Into<u8> for LowNoiseAmplifier {
    fn into(self) -> u8 {
        ((self.input_impedance as u8) >> 7) | (self.current_gain >> 3) | (self.gain_select as u8)
    }
}

impl Default for LowNoiseAmplifier {
    fn default() -> Self {
        LowNoiseAmplifier {
            input_impedance: LNAImpedance::Ohms200,
            current_gain: 0b001,
            gain_select: LNAGainSelect::AGC,
        }
    }
}

#[repr(u8)]
pub enum ReceiveBandwidthMantissa {
    Mantissa16 = 0b00,
    Mantissa20 = 0b01,
    Mantissa24 = 0b10,
}

pub struct ReceiveBandwidth {
    /// Cut-off frequency of the DC offset canceller (DCC):
    /// f_c = \frac{4 \cross RxBw}{2\pi \cross 2^{DccFreq + 2}}
    pub dcc_freq: u8,
    pub bw_mant: ReceiveBandwidthMantissa,
    /// FSK: RxBw = \frac{FXOSC}{bw_mant \cross 2^{bw_exp + 2}}
    /// OOK: RxBw = \frac{FXOSC}{bw_mant \cross 2^{bw_exp + 3}}
    pub bw_exp: u8,
}

impl From<u8> for ReceiveBandwidth {
    fn from(raw: u8) -> Self {
        ReceiveBandwidth {
            dcc_freq: (raw >> 5) & 0b111,
            bw_mant: unsafe { mem::transmute((raw >> 3) & 0b11) },
            bw_exp: raw & 0b111,
        }
    }
}

impl Into<u8> for ReceiveBandwidth {
    fn into(self) -> u8 {
        ((self.dcc_freq & 0b111) << 5) | ((self.bw_mant as u8) >> 3) | (self.bw_exp & 0b111)
    }
}

impl Default for ReceiveBandwidth {
    fn default() -> Self {
        ReceiveBandwidth {
            dcc_freq: 0b010,
            bw_mant: ReceiveBandwidthMantissa::Mantissa24,
            bw_exp: 0b101,
        }
    }
}

pub struct AFCBandwidth {
    pub dcc_freq: u8,
    pub rx_bw_mant: u8,
    pub rx_bw_exp: u8,
}

impl From<u8> for AFCBandwidth {
    fn from(raw: u8) -> Self {
        AFCBandwidth {
            dcc_freq: (raw >> 5) & 0b111,
            rx_bw_mant: (raw >> 3) & 0b11,
            rx_bw_exp: raw & 0b111,
        }
    }
}

impl Into<u8> for AFCBandwidth {
    fn into(self) -> u8 {
        ((self.dcc_freq & 0b111) << 5) | ((self.rx_bw_mant & 0b11) << 3) | (self.rx_bw_exp & 0b111)
    }
}

impl Default for AFCBandwidth {
    fn default() -> Self {
        AFCBandwidth {
            dcc_freq: 0b100,
            rx_bw_mant: 0b01,
            rx_bw_exp: 0b011,
        }
    }
}

#[repr(u8)]
pub enum OOKThreshType {
    Fixed = 0b00,
    Peak = 0b01,
    Average = 0b10,
}

#[repr(u8)]
/// Size of each decrement of the RSSI threshold in the OOK demodulator
pub enum OOKPeakThreshStep {
    DB05 = 0b000,
    DB10 = 0b001,
    DB15 = 0b010,
    DB20 = 0b011,
    DB30 = 0b100,
    DB40 = 0b101,
    DB50 = 0b110,
    DB60 = 0b111,
}

#[repr(u8)]
/// Period of decrement of the RSSI threshold in the OOK demodulator
pub enum OOKPeakThreshDec {
    Once = 0b000,
    OnceEvery2 = 0b001,
    OnceEvery4 = 0b010,
    OnceEvery8 = 0b011,
    Twice = 0b100,
    FourTimes = 0b101,
    EightTimes = 0b110,
    SixteenTimes = 0b111,
}

pub struct OOKPeak {
    pub thresh_type: OOKThreshType,
    pub peak_thresh_step: OOKPeakThreshStep,
    pub peak_thresh_dec: OOKPeakThreshDec,
}

impl From<u8> for OOKPeak {
    fn from(raw: u8) -> Self {
        unsafe {
            OOKPeak {
                thresh_type: mem::transmute((raw >> 6) & 0b11),
                peak_thresh_step: mem::transmute((raw >> 3) & 0b111),
                peak_thresh_dec: mem::transmute(raw & 0b111),
            }
        }
    }
}

impl Into<u8> for OOKPeak {
    fn into(self) -> u8 {
        ((self.thresh_type as u8) << 6) | ((self.peak_thresh_step as u8) << 3) |
            ((self.peak_thresh_dec as u8))
    }
}

impl Default for OOKPeak {
    fn default() -> Self {
        OOKPeak {
            thresh_type: OOKThreshType::Peak,
            peak_thresh_step: OOKPeakThreshStep::DB05,
            peak_thresh_dec: OOKPeakThreshDec::Once,
        }
    }
}

#[repr(u8)]
/// Filter coefficients in average mode of the OOK demodulator.
pub enum OOKAverage {
    Pi32th = 0b00000000,
    Pi8th = 0b01000000,
    Pi4th = 0b10000000,
    Pi2th = 0b11000000,
}

impl From<u8> for OOKAverage {
    fn from(raw: u8) -> Self {
        unsafe { mem::transmute(raw) }
    }
}

impl Into<u8> for OOKAverage {
    fn into(self) -> u8 {
        self as u8
    }
}

impl Default for OOKAverage {
    fn default() -> Self {
        OOKAverage::Pi4th
    }
}

pub struct AFCFrequencyErrorIndicator {
    pub fei_done: bool,
    pub fei_start: bool,
    pub afc_done: bool,
    pub afc_auto_clear: bool,
    pub afc_auto_on: bool,
    pub afc_clear: bool,
    pub afc_start: bool,
}

impl From<u8> for AFCFrequencyErrorIndicator {
    fn from(raw: u8) -> Self {
        AFCFrequencyErrorIndicator {
            fei_done: (raw >> 6) & 0b1 == 1,
            fei_start: (raw >> 5) & 0b1 == 1,
            afc_done: (raw >> 4) & 0b1 == 1,
            afc_auto_clear: (raw >> 3) & 0b1 == 1,
            afc_auto_on: (raw >> 2) & 0b1 == 1,
            afc_clear: (raw >> 1) & 0b1 == 1,
            afc_start: raw & 0b1 == 1,
        }
    }
}

impl Into<u8> for AFCFrequencyErrorIndicator {
    fn into(self) -> u8 {
        ((self.fei_done as u8) << 6) | ((self.fei_start as u8) << 5) |
            ((self.afc_done as u8) << 4) | ((self.afc_auto_clear as u8) << 3) |
            ((self.afc_auto_on as u8) << 2) | ((self.afc_clear as u8) << 1) |
            (self.afc_start as u8)
    }
}

impl Default for AFCFrequencyErrorIndicator {
    fn default() -> Self {
        AFCFrequencyErrorIndicator {
            fei_done: false,
            fei_start: false,
            afc_done: true,
            afc_auto_clear: false,
            afc_auto_on: false,
            afc_clear: false,
            afc_start: false,
        }
    }
}

pub struct RSSIConfig {
    pub done: bool,
    pub start: bool,
}

impl From<u8> for RSSIConfig {
    fn from(raw: u8) -> Self {
        RSSIConfig {
            done: (raw >> 1) & 0b1 == 1,
            start: raw & 0b1 == 1,
        }
    }
}

impl Into<u8> for RSSIConfig {
    fn into(self) -> u8 {
        ((self.done as u8) << 1) | (self.start as u8)
    }
}

impl Default for RSSIConfig {
    fn default() -> Self {
        RSSIConfig {
            done: true,
            start: false,
        }
    }
}

pub struct DIOMapping1 {
    pub dio0: u8,
    pub dio1: u8,
    pub dio2: u8,
    pub dio3: u8,
}

impl From<u8> for DIOMapping1 {
    fn from(raw: u8) -> Self {
        DIOMapping1 {
            dio0: (raw >> 6) & 0b11,
            dio1: (raw >> 4) & 0b11,
            dio2: (raw >> 2) & 0b11,
            dio3: raw & 0b11,
        }
    }
}

impl Into<u8> for DIOMapping1 {
    fn into(self) -> u8 {
        ((self.dio0 & 0b11) << 6) | ((self.dio1 & 0b11) << 4) | ((self.dio2 & 0b11) << 2) |
            (self.dio3 & 0b11)
    }
}

impl Default for DIOMapping1 {
    fn default() -> Self {
        DIOMapping1 {
            dio0: 0,
            dio1: 0,
            dio2: 0,
            dio3: 0,
        }
    }
}

#[repr(u8)]
pub enum ClockoutFrequency {
    XOSC = 0b000,
    XOSC2 = 0b001,
    XOSC4 = 0b010,
    XOSC8 = 0b011,
    XOSC16 = 0b100,
    XOSC32 = 0b101,
    RC = 0b110,
    OFF = 0b111,
}

pub struct DIOMapping2 {
    pub dio4: u8,
    pub dio5: u8,
    pub clkout: ClockoutFrequency,
}

impl From<u8> for DIOMapping2 {
    fn from(raw: u8) -> Self {
        DIOMapping2 {
            dio4: (raw >> 6) & 0b11,
            dio5: (raw >> 4) & 0b11,
            clkout: unsafe { mem::transmute(raw & 0b111) },
        }
    }
}

impl Into<u8> for DIOMapping2 {
    fn into(self) -> u8 {
        ((self.dio4 & 0b11) << 6) | ((self.dio5 & 0b11) << 4) | (self.clkout as u8)
    }
}

impl Default for DIOMapping2 {
    fn default() -> Self {
        DIOMapping2 {
            dio4: 0,
            dio5: 0,
            clkout: ClockoutFrequency::OFF,
        }
    }
}

pub struct IRQFlags1 {
    pub mode_ready: bool,
    pub rx_ready: bool,
    pub tx_ready: bool,
    pub pll_lock: bool,
    pub rssi: bool,
    pub timeout: bool,
    pub auto_mode: bool,
    pub sync_address_match: bool,
}

impl From<u8> for IRQFlags1 {
    fn from(raw: u8) -> Self {
        IRQFlags1 {
            mode_ready: (raw >> 7) & 0b1 == 1,
            rx_ready: (raw >> 6) & 0b1 == 1,
            tx_ready: (raw >> 5) & 0b1 == 1,
            pll_lock: (raw >> 4) & 0b1 == 1,
            rssi: (raw >> 3) & 0b1 == 1,
            timeout: (raw >> 2) & 0b1 == 1,
            auto_mode: (raw >> 1) & 0b1 == 1,
            sync_address_match: raw & 0b1 == 1,
        }
    }
}

impl Into<u8> for IRQFlags1 {
    fn into(self) -> u8 {
        ((self.mode_ready as u8) << 7) | ((self.rx_ready as u8) << 6) |
            ((self.tx_ready as u8) << 5) | ((self.pll_lock as u8) << 4) |
            ((self.rssi as u8) << 3) | ((self.timeout as u8) << 2) |
            ((self.auto_mode as u8) << 1) | (self.sync_address_match as u8)
    }
}

impl Default for IRQFlags1 {
    fn default() -> Self {
        IRQFlags1 {
            mode_ready: true,
            rx_ready: false,
            tx_ready: false,
            pll_lock: false,
            rssi: false,
            timeout: false,
            auto_mode: false,
            sync_address_match: false,
        }
    }
}

pub struct IRQFlags2 {
    pub fifo_full: bool,
    pub fifo_not_empty: bool,
    pub fifo_level: bool,
    pub fifo_overrun: bool,
    pub packet_sent: bool,
    pub payload_ready: bool,
    pub crc_ok: bool,
    pub low_bat: bool,
}

impl From<u8> for IRQFlags2 {
    fn from(raw: u8) -> Self {
        IRQFlags2 {
            fifo_full: (raw >> 7) & 0b1 == 1,
            fifo_not_empty: (raw >> 6) & 0b1 == 1,
            fifo_level: (raw >> 5) & 0b1 == 1,
            fifo_overrun: (raw >> 4) & 0b1 == 1,
            packet_sent: (raw >> 3) & 0b1 == 1,
            payload_ready: (raw >> 2) & 0b1 == 1,
            crc_ok: (raw >> 1) & 0b1 == 1,
            low_bat: raw & 0b1 == 1,
        }
    }
}

impl Into<u8> for IRQFlags2 {
    fn into(self) -> u8 {
        ((self.fifo_full as u8) << 7) | ((self.fifo_not_empty as u8) << 6) |
            ((self.fifo_level as u8) << 5) | ((self.fifo_overrun as u8) << 4) |
            ((self.packet_sent as u8) << 3) | ((self.payload_ready as u8) << 2) |
            ((self.crc_ok as u8) << 1) | (self.low_bat as u8)
    }
}

impl Default for IRQFlags2 {
    fn default() -> Self {
        IRQFlags2 {
            fifo_full: false,
            fifo_not_empty: false,
            fifo_level: false,
            fifo_overrun: false,
            packet_sent: false,
            payload_ready: false,
            crc_ok: false,
            low_bat: false,
        }
    }
}

#[repr(u8)]
pub enum FIFOFillCondition {
    /// If SyncAddress interrupt occurs
    Interrupt = 0,
    Always = 1,
}

pub struct SyncConfig {
    on: bool,
    fifo_fill_condition: FIFOFillCondition,
    size: u8,
    tolerance: u8,
}

impl From<u8> for SyncConfig {
    fn from(raw: u8) -> Self {
        SyncConfig {
            on: (raw >> 7) & 0b1 == 1,
            fifo_fill_condition: unsafe { mem::transmute((raw >> 6) & 0b1) },
            size: (raw >> 3) & 0b111,
            tolerance: raw & 0b111,
        }
    }
}

impl Into<u8> for SyncConfig {
    fn into(self) -> u8 {
        ((self.on as u8) << 7) | ((self.fifo_fill_condition as u8) << 6) |
            ((self.size & 0b111) << 3) | (self.tolerance & 0b111)
    }
}

impl Default for SyncConfig {
    fn default() -> Self {
        SyncConfig {
            on: true,
            fifo_fill_condition: FIFOFillCondition::Interrupt,
            size: 0b011,
            tolerance: 0,
        }
    }
}

#[repr(u8)]
pub enum PacketEncoding {
    None = 0b00,
    Manchester = 0b01,
    Whitening = 0b10,
}

#[repr(u8)]
pub enum PacketFiltering {
    None = 0b00,
    MatchNode = 0b01,
    MatchNodeAndBroadcast = 0b10,
}

pub struct PacketConfig1 {
    packet_length_variable: bool,
    dc_free: PacketEncoding,
    crc: bool,
    crc_auto_clear_off: bool,
    address_filtering: PacketFiltering,
}

impl From<u8> for PacketConfig1 {
    fn from(raw: u8) -> Self {
        PacketConfig1 {
            packet_length_variable: (raw >> 7) & 0b1 == 1,
            dc_free: unsafe { mem::transmute((raw >> 5) & 0b11) },
            crc: (raw >> 4) & 0b1 == 1,
            crc_auto_clear_off: (raw >> 3) & 0b1 == 1,
            address_filtering: unsafe { mem::transmute(raw & 0b11) },
        }
    }
}

impl Into<u8> for PacketConfig1 {
    fn into(self) -> u8 {
        ((self.packet_length_variable as u8) >> 7) | ((self.dc_free as u8) >> 5) |
            ((self.crc as u8) >> 4) | ((self.crc_auto_clear_off as u8) >> 3) |
            ((self.address_filtering as u8) >> 1)
    }
}

impl Default for PacketConfig1 {
    fn default() -> Self {
        PacketConfig1 {
            packet_length_variable: false,
            dc_free: PacketEncoding::None,
            crc: true,
            crc_auto_clear_off: false,
            address_filtering: PacketFiltering::None,
        }
    }
}

#[repr(u8)]
pub enum AutoModeEnterCondition {
    None = 0b000,
    FifoNotEmpty = 0b001,
    FifoLevel = 0b010,
    CrcOk = 0b011,
    PayloadReady = 0b100,
    SyncAddress = 0b101,
    PacketSent = 0b110,
    FifoNotEmptyFalling = 0b111,
}

#[repr(u8)]
pub enum AutoModeExitCondition {
    None = 0b000,
    FifoNotEmpty = 0b001,
    FifoLevel = 0b010,
    CrcOk = 0b011,
    PayloadReady = 0b100,
    SyncAddress = 0b101,
    PacketSent = 0b110,
    Timeout = 0b111,
}

#[repr(u8)]
pub enum IntermediateMode {
    Sleep = 0b00,
    Standby = 0b01,
    Receiver = 0b10,
    Transmitter = 0b11,
}

pub struct AutoModes {
    pub enter: AutoModeEnterCondition,
    pub exit: AutoModeExitCondition,
    pub intermediate: IntermediateMode,
}

impl From<u8> for AutoModes {
    fn from(raw: u8) -> Self {
        unsafe {
            AutoModes {
                enter: mem::transmute((raw >> 5) & 0b111),
                exit: mem::transmute((raw >> 2) & 0b111),
                intermediate: mem::transmute(raw & 0b111),
            }
        }
    }
}

impl Into<u8> for AutoModes {
    fn into(self) -> u8 {
        ((self.enter as u8) << 5) | ((self.exit as u8) << 2) | (self.intermediate as u8)
    }
}

impl Default for AutoModes {
    fn default() -> Self {
        AutoModes {
            enter: AutoModeEnterCondition::None,
            exit: AutoModeExitCondition::None,
            intermediate: IntermediateMode::Sleep,
        }
    }
}

pub struct FifoThreshold {
    pub when_not_empty: bool,
    pub threshold: u8,
}

impl From<u8> for FifoThreshold {
    fn from(raw: u8) -> Self {
        FifoThreshold {
            when_not_empty: (raw >> 7) & 0b1 == 1,
            threshold: raw & 0b1111111,
        }
    }
}

impl Into<u8> for FifoThreshold {
    fn into(self) -> u8 {
        ((self.when_not_empty as u8) << 7) | (self.threshold & 0b1111111)
    }
}

impl Default for FifoThreshold {
    fn default() -> Self {
        FifoThreshold {
            when_not_empty: true,
            threshold: 0b0001111,
        }
    }
}

pub struct PacketConfig2 {
    pub inter_packet_rx_delay: u8,
    pub restart_rx: bool,
    pub auto_rx_restart: bool,
    pub aes: bool,
}

impl From<u8> for PacketConfig2 {
    fn from(raw: u8) -> Self {
        PacketConfig2 {
            inter_packet_rx_delay: (raw >> 4) & 0b1111,
            restart_rx: (raw >> 2) & 0b1 == 1,
            auto_rx_restart: (raw >> 1) & 0b1 == 1,
            aes: raw & 0b1 == 1,
        }
    }
}

impl Into<u8> for PacketConfig2 {
    fn into(self) -> u8 {
        ((self.inter_packet_rx_delay & 0b1111) << 4) | ((self.restart_rx as u8) << 2) |
            ((self.auto_rx_restart as u8) << 1) | (self.aes as u8)
    }
}

impl Default for PacketConfig2 {
    fn default() -> Self {
        PacketConfig2 {
            inter_packet_rx_delay: 0b0000,
            restart_rx: false,
            auto_rx_restart: true,
            aes: false,
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
