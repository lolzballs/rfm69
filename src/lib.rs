#![no_std]
#![feature(unsize)]
extern crate embedded_hal as hal;

mod registers;

use core::any::{Any, TypeId};
use core::mem;
use core::marker::{PhantomData, Unsize};

use hal::blocking::spi;
use hal::digital::OutputPin;

use registers::Register;

const FXOSC: f32 = 32000000.0;
const FSTEP: f32 = FXOSC / 524288.0;

pub struct Regular;
pub struct HighPower;

pub struct RFM69<SPI, NCS, PA> {
    spi: SPI,
    ncs: NCS,
    _pa: PhantomData<PA>,
}

fn new<SPI, NCS, PA, E>(spi: SPI, ncs: NCS) -> Result<RFM69<SPI, NCS, PA>, E>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
    PA: Any,
{
    let mut rfm = RFM69 {
        spi,
        ncs,
        _pa: PhantomData,
    };

    rfm.op_mode(OpMode::Standby)?;
    rfm.data_mode(DataMode::Packet)?;
    rfm.mod_type(ModulationType::FSK)?;
    rfm.mod_shaping(ModulationShaping::_00)?;
    rfm.bitrate(10000.0)?; // 10 kbps
    rfm.fdev(20000.0)?; // 20 kHz
    rfm.freq(915000000.0)?; // 915 MHz
    rfm.lna_zin(LNAZin::_0)?;
    rfm.write(Register::RXBW, 0x4C)?; // 25 kHz
    rfm.preamble(3)?;
    rfm.sync(&[0x41, 0x48])?;
    rfm.packet_length(PacketLength::Fixed(5))?;
    rfm.fifo_mode(FifoMode::NotEmpty)?;

    Ok(rfm)
}

impl<SPI, NCS, PA, E> RFM69<SPI, NCS, PA>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
    PA: Any,
{
    pub fn op_mode(&mut self, mode: OpMode) -> Result<(), E> {
        match mode {
            OpMode::Transmitter => self.high_power_regs(true)?,
            OpMode::Reciever => self.high_power_regs(false)?,
            _ => (),
        }
        self.modify(Register::OPMODE, |r| (r & !0b111) | (mode as u8))
    }

    pub fn data_mode(&mut self, mode: DataMode) -> Result<(), E> {
        self.modify(Register::DATAMODUL, |r| (r & !0b1100000) | (mode as u8))
    }

    pub fn mod_type(&mut self, ty: ModulationType) -> Result<(), E> {
        self.modify(Register::DATAMODUL, |r| (r & !0b11000) | (ty as u8))
    }

    pub fn mod_shaping(&mut self, ty: ModulationShaping) -> Result<(), E> {
        self.modify(Register::DATAMODUL, |r| (r & !0b11) | (ty as u8))
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
        let r = (FSTEP * fdev) as u16;
        self.write(Register::FDEV_MSB, (r >> 8) as u8)?;
        self.write(Register::FDEV_LSB, (r & 0xFF) as u8)?;
        Ok(())
    }

    pub fn freq(&mut self, freq: f32) -> Result<(), E> {
        self.op_mode(OpMode::Standby)?;
        let r = (FSTEP * freq) as u32;
        self.write(Register::FRF_MSB, ((r >> 16) & 0xFF) as u8)?;
        self.write(Register::FRF_MID, ((r >> 8) & 0xFF) as u8)?;
        self.write(Register::FRF_LSB, (r & 0xFF) as u8)?;
        Ok(())
    }

    pub fn lna_zin(&mut self, zin: LNAZin) -> Result<(), E> {
        self.modify(Register::LNA, |r| (r & !0b10000000) | (zin as u8))
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
            self.write(Register::SYNCCONFIG, 0b10000000 | (sync.len() - 1) as u8)?;

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

    pub fn packet_encoding(&mut self, encoding: DCEncoding) -> Result<(), E> {
        self.modify(Register::PACKETCONFIG1, |r| {
            (r & !0b1100000) | (encoding as u8)
        })?;
        Ok(())
    }

    pub fn packet_crc(&mut self, on: bool) -> Result<(), E> {
        self.modify(Register::PACKETCONFIG1, |r| (r & !0b10000) | (on as u8))?;
        Ok(())
    }

    pub fn packet_address(&mut self, mode: AddressMode) -> Result<(), E> {
        self.modify(Register::PACKETCONFIG1, |r| (r & !0b110) | (mode as u8))?;
        Ok(())
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
        let buf: [u8; 2] = self.read_many(reg)?;
        Ok(buf[1])
    }

    fn read_many<B>(&mut self, reg: Register) -> Result<B, E>
    where
        B: Unsize<[u8]>,
    {
        let mut buffer: B = unsafe { mem::zeroed() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.read_address();
            self.ncs.set_low();
            self.spi.transfer(slice)?;
            self.ncs.set_high();
        }
        Ok(buffer)
    }

    fn write(&mut self, reg: Register, val: u8) -> Result<(), E> {
        self.ncs.set_low();
        self.spi.write(&[reg.write_address(), val])?;
        self.ncs.set_high();
        Ok(())
    }

    fn write_many(&mut self, reg: Register, buf: &[u8]) -> Result<(), E> {
        unimplemented!();
    }
}

impl<SPI, NCS, E> RFM69<SPI, NCS, HighPower>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
{
    pub fn new(spi: SPI, ncs: NCS) -> Result<Self, E> {
        let mut rfm: Self = new(spi, ncs)?;
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

impl<SPI, NCS, E> RFM69<SPI, NCS, Regular>
where
    SPI: spi::Write<u8, Error = E> + spi::Transfer<u8, Error = E>,
    NCS: OutputPin,
{
    pub fn new(spi: SPI, ncs: NCS) -> Result<Self, E> {
        let mut rfm: Self = new(spi, ncs)?;
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

pub enum OpMode {
    Sleep = 0b000,
    Standby = 0b001,
    FreqSynth = 0b010,
    Transmitter = 0b011,
    Reciever = 0b100,
}

pub enum DataMode {
    Packet = 0b00,
    ContinuousSync = 0b10,
    Continuous = 0b11,
}

pub enum ModulationType {
    FSK = 0b00,
    OOK = 0b01,
}

pub enum ModulationShaping {
    _00 = 0b00,
    _01 = 0b01,
    _10 = 0b10,
    _11 = 0b11,
}

/// LNA (Low noise amplifier) input impedance
pub enum LNAZin {
    /// 50 Ohms
    _0 = 0b0,
    /// 200 Ohms
    _1 = 0b1,
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
    None = 0b00,
    Node = 0b01,
    NodeBroadcast = 0b10,
}

pub enum FifoMode {
    NotEmpty,
    Threshold(u8),
}
