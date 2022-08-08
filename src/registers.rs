use core::ops::Add;
use core::mem;

const R: u8 = 0x7F;
const W: u8 = 0x80;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Register {
    FIFO = 0x00,
    OPMODE = 0x01,
    DATAMODUL = 0x02,
    BITRATE_MSB = 0x03,
    BITRATE_LSB = 0x04,
    FDEV_MSB = 0x05,
    FDEV_LSB = 0x06,
    FRF_MSB = 0x07,
    FRF_MID = 0x08,
    FRF_LSB = 0x09,
    OSC1 = 0x0A,
    AFCCTRL = 0x0B,
    LOWBAT = 0x0C,
    LISTEN1 = 0x0D,
    LISTEN2 = 0x0E,
    LISTEN3 = 0x0F,
    VERSION = 0x10,
    PALEVEL = 0x11,
    PARAMP = 0x12,
    OCP = 0x13,
    LNA = 0x18,
    RXBW = 0x19,
    AFCBW = 0x1A,
    OOKPEAK = 0x1B,
    OOKAVG = 0x1C,
    OOKFIX = 0x1D,
    AFCFEI = 0x1E,
    AFC_MSB = 0x1F,
    AFC_LSB = 0x20,
    FEI_MSB = 0x21,
    FEI_LSB = 0x22,
    RSSICONFIG = 0x23,
    RSSIVALUE = 0x24,
    DIOMAPPING1 = 0x25,
    DIOMAPPING2 = 0x26,
    IRQFLAGS1 = 0x27,
    IRQFLAGS2 = 0x28,
    RSSITHRESH = 0x29,
    RXTIMEOUT1 = 0x2A,
    RXTIMEOUT2 = 0x2B,
    PREAMBLE_MSB = 0x2C,
    PREAMBLE_LSB = 0x2D,
    SYNCCONFIG = 0x2E,
    SYNCVALUE1 = 0x2F,
    SYNCVALUE2 = 0x30,
    SYNCVALUE3 = 0x31,
    SYNCVALUE4 = 0x32,
    SYNCVALUE5 = 0x33,
    SYNCVALUE6 = 0x34,
    SYNCVALUE7 = 0x35,
    SYNCVALUE8 = 0x36,
    PACKETCONFIG1 = 0x37,
    PAYLOADLENGTH = 0x38,
    NODEADRS = 0x39,
    BROADCASTADRS = 0x3A,
    AUTOMODES = 0x3B,
    FIFOTHRESH = 0x3C,
    PACKETCONFIG2 = 0x3D,
    AESKEY1 = 0x3E,
    AESKEY2 = 0x3F,
    AESKEY3 = 0x40,
    AESKEY4 = 0x41,
    AESKEY5 = 0x42,
    AESKEY6 = 0x43,
    AESKEY7 = 0x44,
    AESKEY8 = 0x45,
    AESKEY9 = 0x46,
    AESKEY10 = 0x47,
    AESKEY11 = 0x48,
    AESKEY12 = 0x49,
    AESKEY13 = 0x4A,
    AESKEY14 = 0x4B,
    AESKEY15 = 0x4C,
    AESKEY16 = 0x4D,
    TEMP1 = 0x4E,
    TEMP2 = 0x4F,
    TESTLNA = 0x58,
    TESTTCXO = 0x59,
    TESTPA1 = 0x5A,
    TESTPA2 = 0x5C,
    TESTLLBW = 0x5F,
    TESTDAGC = 0x6F,
    TESTAFC = 0x71,
}

impl Register {
    #[inline]
    pub fn read_address(&self) -> u8 {
        *self as u8 & R
    }

    #[inline]
    pub fn write_address(&self) -> u8 {
        *self as u8 | W
    }
}

impl Add<u8> for Register {
    type Output = Self;

    fn add(self, rhs: u8) -> Self {
        unsafe { mem::transmute(self as u8 + rhs) }
    }
}
