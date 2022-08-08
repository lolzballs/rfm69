extern crate linux_embedded_hal as linux_hal;
extern crate rfm69;

use std::time::{Duration, Instant};

use linux_hal::{Pin, Spidev};
use linux_hal::spidev::{self, SpidevOptions};
use linux_hal::sysfs_gpio::Direction;

use rfm69::{HighPower, PacketLength, RFM69, Timer};

struct PiTimer;

impl Timer for PiTimer {
    type Instant = Instant;

    fn now(&self) -> Instant {
        Instant::now()
    }

    fn since(&self, past: &Instant) -> Duration {
        past.elapsed()
    }
}

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").expect("SPI open error");
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(10_000_000)
        .mode(spidev::SpiModeFlags::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    let pin = Pin::new(25);
    pin.export().unwrap();
    while !pin.is_exported() {}
    pin.set_direction(Direction::Out).unwrap();
    pin.set_value(1).unwrap();

    let mut rfm69 = RFM69::<_, _, _, HighPower>::new(spi, pin, PiTimer).unwrap();
    rfm69.packet_length(PacketLength::Fixed(2)).unwrap();

    let mut buf = [69, 69];
    rfm69.send(&mut buf).unwrap();
}
