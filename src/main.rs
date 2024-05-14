// #![no_std]
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

// mod kalman;
// mod ahrs;
mod ahrs;
mod madgwick;

use crate::ahrs::Ahrs;
use core::f32::consts::PI;
use defmt::println;
use dr_icm_20948::{Init, MagEnabled};
use embassy_stm32::{
    bind_interrupts,
    i2c::I2c,
    mode::Async,
    peripherals::{self, I2C1},
};
use nalgebra::Vector3;
use {
    defmt::unwrap,
    defmt_rtt as _,
    dr_icm_20948::Icm20948,
    embassy_executor::Spawner,
    embassy_stm32::{i2c, time::hz, Config},
    panic_probe as _,
};

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());

    let i2c = embassy_stm32::i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        hz(5000),
        i2c::Config::default(),
    );
    let imu = Icm20948::new(i2c);
    let imu = imu.initialize_9dof().await.unwrap();

    let sample_period = (1.0) / (256.0);
    let filter = madgwick::Filter::new(sample_period, 1.0);

    unwrap!(spawner.spawn(driver(imu, filter)));
}

#[embassy_executor::task]
async fn driver(
    mut imu: Icm20948<I2c<'static, I2C1, Async>, MagEnabled, Init>,
    mut filter: madgwick::Filter<f32>,
) {
    loop {
        let data = imu.read_all().await.unwrap();

        let gyroscope = Vector3::new(data.gyr.x, data.gyr.y, data.gyr.z);
        let accelerometer = Vector3::new(data.acc.x, data.acc.y, data.acc.z);
        // добавить фильтр шума от магнитного поля моторов
        let magnetometer = Vector3::new(data.mag.x, data.mag.y, data.mag.z);

        let g = gyroscope * (PI / 180.0);
        let quat = filter.update(&g, &accelerometer, &magnetometer).unwrap();
        let (roll, pitch, yaw) = quat.euler_angles();

        println!("{} {} {}", pitch, roll, yaw);
    }
}
