#![no_std]

use core::fmt::Debug;
use core::marker::PhantomData;

use embassy_stm32::i2c::Error;
use embassy_stm32::usart::{BasicInstance, RxDma, TxDma, Uart};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
pub use embedded_hal;
pub use embedded_hal_async::i2c::I2c as AsI2c;
pub use embedded_hal_async::spi::SpiDevice as AsSpi;
pub use legacy_hal::blocking::spi::Transfer;
pub use legacy_hal::digital::v2::OutputPin as LegacyOutputPin;

// use embedded_hal::serial::ErrorKind;

// use embedded_hal_async::serial;

pub struct SerialAsync<'a, M: RawMutex, I, TX, RX>
    where I: BasicInstance, TX: TxDma<I>, RX: RxDma<I> {
    bus: &'a Mutex<M, Uart<'a, I, TX, RX>>,
}

impl<'a, M: RawMutex, I, TX, RX> SerialAsync<'a, M, I, TX, RX>
    where
        M: RawMutex + 'static,
        I: BasicInstance + 'static, TX: TxDma<I>, RX: RxDma<I>
{
    pub fn new(bus: &'a Mutex<M, Uart<'a, I, TX, RX>>) -> Self {
        Self { bus }
    }

    pub async fn write(&mut self, words: &[u8]) {
        let mut bus = self.bus.lock().await;
        bus.write(words).await.map_err(SerialError::Serial).unwrap();
    }
}

#[derive(Debug)]
pub enum SerialError<BUS> {
    Serial(BUS)
}

// impl<BUS> serial::Error for SerialError<BUS>
//     where
//         BUS: serial::Error + Debug, {
//     fn kind(&self) -> ErrorKind {
//         match self {
//             Self::Serial(e) => e.kind(),
//         }
//     }
// }

pub struct TwinG<K1, K2, S1: AsSpi, S2: AsSpi>
    where K1: SPIDevice<S1>, K2: SPIDevice<S2> {
    pub s1: K1,
    pub s2: K2,
    i1: PhantomData<S1>,
    i2: PhantomData<S2>,
}

impl<S1: AsSpi, S2: AsSpi, K1, K2> TwinG<K1, K2, S1, S2>
    where
        K1: SPIDevice<S1>, K2: SPIDevice<S2> {
    pub fn new(g1: S1, g2: S2) -> TwinG<K1, K2, S1, S2> {
        Self {
            s1: K1::new_with_interface(g1),
            s2: K2::new_with_interface(g2),
            i1: PhantomData::<S1>,
            i2: PhantomData::<S2>,
        }
    }
}

pub trait SPIDevice<SI: AsSpi> {
    fn new_with_interface(sensor_interface: SI) -> Self;
}

#[allow(async_fn_in_trait)]
pub trait DroneI2c {
    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error>;
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Error>;
    async fn write_read(&mut self, addr: u8, data: &[u8], buffer: &mut [u8]) -> Result<(), Error>;
}