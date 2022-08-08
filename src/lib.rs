//! A platform agnostic Rust driver for the RFM69 wireless chips, which are based on the
//! SX1231 chip. This crates uses the
//! [`embedded_hal`](https://github.com/japaric/embedded_hal) traits.
//!
//! ## The Device
//!
//! There are multiple variants of the RFM69, which this crate aims to support.
//! Products with an H are high power chips and should be use the `HighPower` PA mode.
//! Products with a C have a different pinout arrangement but are otherwise the same as
//! their non-C variant.
//!
//! ### RFM69W
//! - [Product page](http://www.hoperf.com/rf_transceiver/modules/RFM69W.html)
//! - [Datasheet](http://www.hoperf.com/upload/rf/RFM69W-V1.3.pdf)
//!
//! ### RFM69CW
//! - [Product page](http://www.hoperf.com/rf_transceiver/modules/RFM69CW.html)
//! - [Datasheet](http://www.hoperf.com/upload/rf/RFM69HCW-V1.1.pdf)
//!
//! ### RFM69HW
//! - [Product page](http://www.hoperf.com/rf_transceiver/modules/RFM69HCW.html)
//! - [Datasheet](http://www.hoperf.com/upload/rf/RFM69HW-V1.3.pdf)
//!
//! ### RFM69HCW
//! - [Product page](http://www.hoperf.com/rf_transceiver/modules/RFM69HCW.html)
//! - [Datasheet](http://www.hoperf.com/upload/rf/RFM69HCW-V1.1.pdf)
//!
//! See `examples/` for examples on usage.

#![no_std]
#![feature(unsize)]
extern crate embedded_hal as hal;

mod time;
mod registers;

use core::any::{Any, TypeId};
use core::marker::{PhantomData};
use core::time::Duration;

use hal::blocking::spi;
use hal::digital::v2::OutputPin;

use registers::Register;
pub use time::Timer;

const TIMEOUT_MODE_READY: Duration = Duration::from_millis(100);
const TIMEOUT_TX: Duration = Duration::from_millis(100);
const FXOSC: f32 = 32000000.0;
const FSTEP: f32 = FXOSC / 524288.0;

pub struct Regular;
pub struct HighPower;

pub struct RFM69<SPI, NCS, T, PA> {
    spi: SPI,
    ncs: NCS,
    timer: T,
    rssi: f32,
    _pa: PhantomData<PA>,
}

fn new<SPI, NCS, T, PA, E>(spi: SPI, ncs: NCS, timer: T) -> Result<RFM69<SPI, NCS, T, PA>, E>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
    T: Timer,
    PA: Any,
{
    let mut rfm = RFM69 {
        spi,
        ncs,
        timer,
        rssi: 0.0,
        _pa: PhantomData,
    };

    rfm.op_mode(OpMode::Standby)?;
    rfm.mod_settings(ModulationSettings {
        mode: DataMode::Packet,
        ty: ModulationType::FSK,
        shaping: ModulationShaping::_00,
    })?;
    rfm.bitrate(10000.0)?; // 10 kbps
    rfm.fdev(20000.0)?; // 20 kHz
    rfm.freq(915000000.0)?; // 915 MHz
    rfm.write(Register::LNA, 0x88)?;
    rfm.write(Register::RXBW, 0x4C)?; // 25 kHz
    rfm.preamble(3)?;
    rfm.sync(&[0x41, 0x48])?;
    rfm.packet_length(PacketLength::Fixed(5))?;
    rfm.modify(Register::PALEVEL, |r| (r & 0xE0) | 31)?;
    rfm.fifo_mode(FifoMode::NotEmpty)?;

    Ok(rfm)
}

impl<SPI, NCS, T, PA, E> RFM69<SPI, NCS, T, PA>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
    T: Timer,
    PA: Any,
{
    pub fn op_mode(&mut self, mode: OpMode) -> Result<(), E> {
        self.modify(Register::OPMODE, |r| (r & !0b11100) | ((mode as u8) << 2))?;
        match mode {
            OpMode::Transmitter => self.high_power_regs(true)?,
            OpMode::Reciever => self.high_power_regs(false)?,
            _ => (),
        }
        Ok(())
    }

    pub fn mod_settings(&mut self, settings: ModulationSettings) -> Result<(), E> {
        self.write(
            Register::DATAMODUL,
            (settings.mode as u8) << 5 | (settings.ty as u8) << 3 | (settings.shaping as u8),
        )
    }

    pub fn bitrate(&mut self, rate: f32) -> Result<(), E> {
        self.op_mode(OpMode::Standby)?;
        let r = (FXOSC / rate) as u16;
        self.write(Register::BITRATE_MSB, (r >> 8) as u8)?;
        self.write(Register::BITRATE_LSB, (r & 0xFF) as u8)?;
        Ok(())
    }

    pub fn fdev(&mut self, fdev: f32) -> Result<(), E> {
        self.op_mode(OpMode::Standby)?;
        let r = (fdev / FSTEP) as u16;
        self.write(Register::FDEV_MSB, (r >> 8) as u8)?;
        self.write(Register::FDEV_LSB, (r & 0xFF) as u8)?;
        Ok(())
    }

    pub fn freq(&mut self, freq: f32) -> Result<(), E> {
        self.op_mode(OpMode::Standby)?;
        let r = (freq / FSTEP) as u32;
        self.write(Register::FRF_MSB, ((r >> 16) & 0xFF) as u8)?;
        self.write(Register::FRF_MID, ((r >> 8) & 0xFF) as u8)?;
        self.write(Register::FRF_LSB, (r & 0xFF) as u8)?;
        Ok(())
    }

    pub fn preamble(&mut self, len: u16) -> Result<(), E> {
        self.write(Register::PREAMBLE_MSB, (len >> 8) as u8)?;
        self.write(Register::PREAMBLE_LSB, (len & 0xFF) as u8)?;
        Ok(())
    }

    pub fn sync(&mut self, sync: &[u8]) -> Result<(), E> {
        if sync.len() == 0 {
            self.write(Register::SYNCCONFIG, 0)?;
        } else {
            self.write(
                Register::SYNCCONFIG,
                0b10000000 | ((sync.len() - 1) << 3) as u8,
            )?;

            for (i, b) in sync.iter().enumerate() {
                self.write(Register::SYNCVALUE1 + i as u8, *b)?;
            }
        }
        Ok(())
    }

    pub fn packet_length(&mut self, len: PacketLength) -> Result<(), E> {
        match len {
            PacketLength::Fixed(len) => {
                self.modify(Register::PACKETCONFIG1, |r| r & !0b10000000)?;
                self.write(Register::PAYLOADLENGTH, len)?;
            }
            PacketLength::Variable => {
                self.modify(Register::PACKETCONFIG1, |r| r | 0b10000000)?;
            }
        }
        Ok(())
    }

    pub fn packet_settings(&mut self, settings: PacketSettings) -> Result<(), E> {
        self.write(
            Register::PACKETCONFIG1,
            (settings.encoding as u8) << 5 | (settings.crc as u8) << 4
                | (settings.filtering as u8) << 1,
        )
    }

    pub fn node_address(&mut self, a: u8) -> Result<(), E> {
        self.write(Register::NODEADRS, a)
    }

    pub fn broadcast_address(&mut self, a: u8) -> Result<(), E> {
        self.write(Register::BROADCASTADRS, a)
    }

    pub fn fifo_mode(&mut self, mode: FifoMode) -> Result<(), E> {
        match mode {
            FifoMode::NotEmpty => self.write(Register::FIFOTHRESH, 0b10000000),
            FifoMode::Threshold(thresh) => self.write(Register::FIFOTHRESH, thresh & 0b1111),
        }
    }

    pub fn rssi(&mut self) -> f32 {
        self.rssi
    }

    pub fn receive(&mut self, buf: &mut [u8]) -> Result<(), E> {
        // TODO: Check buf length
        self.op_mode(OpMode::Reciever)?;
        self.wait_for_mode()?;

        while !self.is_packet_ready()? {}

        self.op_mode(OpMode::Standby)?;

        self.read_many(Register::FIFO, buf)?;
        self.rssi = self.read(Register::RSSIVALUE)? as f32 / -2.0;
        Ok(())
    }

    pub fn send(&mut self, buf: &[u8]) -> Result<(), E> {
        // TODO: Check buf length
        self.op_mode(OpMode::Standby)?;
        self.wait_for_mode()?;

        self.clear_fifo()?;

        self.write_many(Register::FIFO, buf)?;
        self.op_mode(OpMode::Transmitter)?;

        self.wait_for_packet_sent()?;
        self.op_mode(OpMode::Standby)?;

        Ok(())
    }

    fn clear_fifo(&mut self) -> Result<(), E> {
        self.write(Register::IRQFLAGS2, 0x10)
    }

    fn is_packet_ready(&mut self) -> Result<bool, E> {
        Ok(self.read(Register::IRQFLAGS2)? & 0b00000100 != 0)
    }

    fn wait_for_mode(&mut self) -> Result<(), E> {
        let start = self.timer.now();
        while self.read(Register::IRQFLAGS1)? & 0b10000000 == 0 {
            if self.timer.since(&start) > TIMEOUT_MODE_READY {
                panic!("Timeout"); // TODO: Turn this into an Error
            }
        }

        Ok(())
    }

    fn wait_for_packet_sent(&mut self) -> Result<(), E> {
        let start = self.timer.now();
        while self.read(Register::IRQFLAGS2)? & 0b00001000 == 0 {
            if self.timer.since(&start) > TIMEOUT_TX {
                panic!("Timeout"); // TODO: Turn this into an Error
            }
        }

        Ok(())
    }

    fn high_power_regs(&mut self, on: bool) -> Result<(), E> {
        if TypeId::of::<PA>() == TypeId::of::<HighPower>() {
            self.write(Register::TESTPA1, if on { 0x5D } else { 0x55 })?;
            self.write(Register::TESTPA2, if on { 0x7C } else { 0x70 })?;
        }
        Ok(())
    }

    fn modify<F>(&mut self, reg: Register, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read(reg)?;
        self.write(reg, f(r))?;
        Ok(())
    }

    fn read(&mut self, reg: Register) -> Result<u8, E> {
        let mut buf = [0u8; 1];
        self.read_many(reg, &mut buf)?;
        Ok(buf[0])
    }

    fn read_many(&mut self, reg: Register, data: &mut [u8]) -> Result<(), E> {
        self.ncs.set_low();
        self.spi.transfer(&mut [reg.read_address()])?;
        self.spi.transfer(data)?;
        self.ncs.set_high();
        Ok(())
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), E> {
        self.write_many(reg, &[val])
    }

    fn write_many(&mut self, reg: Register, buf: &[u8]) -> Result<(), E> {
        self.ncs.set_low();
        self.spi.write(&[reg.write_address()])?;
        self.spi.write(buf)?;
        self.ncs.set_high();
        Ok(())
    }
}

impl<SPI, NCS, T, E> RFM69<SPI, NCS, T, HighPower>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    T: Timer,
    NCS: OutputPin,
{
    pub fn new(spi: SPI, ncs: NCS, timer: T) -> Result<Self, E> {
        let mut rfm: Self = new(spi, ncs, timer)?;
        rfm.high_power()?;
        Ok(rfm)
    }

    fn high_power(&mut self) -> Result<(), E> {
        // Turn off over-current protection
        self.modify(Register::OCP, |r| (r & !0xF0) | (0x00))?;
        self.modify(Register::PALEVEL, |r| (r & !0b11100000) | (0b01100000))?;
        Ok(())
    }
}

impl<SPI, NCS, T, E> RFM69<SPI, NCS, T, Regular>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    T: Timer,
    NCS: OutputPin,
{
    pub fn new(spi: SPI, ncs: NCS, timer: T) -> Result<Self, E> {
        let mut rfm: Self = new(spi, ncs, timer)?;
        rfm.high_power()?;
        Ok(rfm)
    }

    fn high_power(&mut self) -> Result<(), E> {
        // Turn on over-current protection
        self.modify(Register::OCP, |r| (r & !0xF0) | (0x10))?;
        self.modify(Register::PALEVEL, |r| (r & !0b11100000) | (0b10000000))?;
        Ok(())
    }
}

#[derive(Copy, Clone)]
pub enum OpMode {
    Sleep = 0b000,
    Standby = 0b001,
    FreqSynth = 0b010,
    Transmitter = 0b011,
    Reciever = 0b100,
}

pub struct ModulationSettings {
    pub mode: DataMode,
    pub ty: ModulationType,
    pub shaping: ModulationShaping,
}

/// Data processing mode
pub enum DataMode {
    /// Packet mode
    Packet = 0b00,
    /// Continuous mode with bit synchronizer
    ContinuousSync = 0b10,
    /// Continuous mode without bit synchronizer
    Continuous = 0b11,
}

/// Modulation scheme
pub enum ModulationType {
    /// Frequency shift keying
    FSK = 0b00,
    /// On-off keying
    OOK = 0b01,
}

/// Data shaping
pub enum ModulationShaping {
    /// No Shaping
    _00 = 0b00,
    /// FSK: Gaussian filter: BT = 1.0
    /// OOK: Filtering with cutoff frequency = BR
    _01 = 0b01,
    /// FSK: Gaussian filter: BT = 0.5
    /// OOK: Filtering with cutoff frequency = 2 * 2 * BR
    _10 = 0b10,
    /// FSK: Gaussian filter: BT = 0.3
    /// OOK: Reserved
    _11 = 0b11,
}

/// LNA (Low noise amplifier) input impedance
pub enum LNAZin {
    /// 50 Ohms
    _0 = 0b0,
    /// 200 Ohms
    _1 = 0b1,
}

pub struct PacketSettings {
    pub crc: bool,
    pub encoding: DCEncoding,
    pub filtering: AddressMode,
}

pub enum PacketLength {
    Fixed(u8),
    Variable,
}

pub enum DCEncoding {
    None = 0b00,
    Manchester = 0b01,
    Whitening = 0b10,
}

/// Address filtering mode
pub enum AddressMode {
    /// No filtering
    None = 0b00,
    /// Only matches when NodeAdrs is equal
    Node = 0b01,
    /// Only matches when NodeAdrs is equal or BroadcastAdrs is equal
    NodeBroadcast = 0b10,
}

pub enum FifoMode {
    NotEmpty,
    Threshold(u8),
}
