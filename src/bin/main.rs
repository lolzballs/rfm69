extern crate spidev;
extern crate spi_rfm69;

use std::io::Result;

use spidev::{Spidev, SpidevTransfer, SpidevOptions};
use spi_rfm69::*;

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(10_000_000)
        .mode(spidev::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    let mut rfm = RFM69::new(spi).unwrap();
    rfm.set_modem_config(ModemConfig {
        modulation: DataModulationConfig {
            data_mode: DataModulationMode::Packet,
            modulation_type: DataModulation::FSK(FSKShaping::None),
        },
        bitrate: 250000.0,
        frequency_deviation: 250000.0,
        rxbw: ReceiveBandwidth {
            dcc_freq: 8,
            bw_mant: BandwidthMantissa::Mantissa16,
            bw_exp: 0,
        },
        afcbw: AFCBandwidth {
            dcc_freq: 8,
            bw_mant: BandwidthMantissa::Mantissa16,
            bw_exp: 0,
        },
        packet_config: PacketConfig1 {
            packet_length_variable: true,
            dc_free: PacketEncoding::Whitening,
            crc: true,
            crc_auto_clear_off: false,
            address_filtering: PacketFiltering::None,
        },
    }).unwrap();
    rfm.set_encryption_key(None).unwrap();
    rfm.set_sync_words(Some(&[0x2d, 0xd4])).unwrap();
    loop {
        if rfm.available().unwrap() {
            if let Some(packet) = rfm.recv().unwrap() {
                println!("{:?} {}", &packet.payload[..packet.len], packet.rssi);
            }
        }
    }
}
