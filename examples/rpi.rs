extern crate linux_embedded_hal as linux_hal;
extern crate rfm69;

use linux_hal::{Pin, Spidev};
use linux_hal::spidev::{self, SpidevOptions};
use linux_hal::sysfs_gpio::Direction;

use rfm69::{HighPower, RFM69};

fn main() {
    let mut spi = Spidev::open("/dev/spidev0.0").expect("SPI open error");
    let options = SpidevOptions::new()
        .max_speed_hz(1_000_000)
        .mode(spidev::SPI_MODE_0)
        .build();
    spi.configure(&options).unwrap();

    let pin = Pin::new(25);
    pin.export().unwrap();
    while !pin.is_exported() {}
    pin.set_direction(Direction::Out).unwrap();
    pin.set_value(1).unwrap();

    let mut rfm69 = RFM69::<_, _, HighPower>::new(spi, pin).unwrap();
}
