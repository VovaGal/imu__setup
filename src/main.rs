// #![no_std]
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

// mod kalman;
// mod ahrs;
mod ahrs;
mod madgwick;

use crate::ahrs::Ahrs;
use core::f64::consts::PI;
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
        hz(3000),
        i2c::Config::default(),
    );
    let imu = Icm20948::new(i2c);
    let mut imu = imu.initialize_9dof().await.unwrap();
    imu.set_gyr_odr(0).await.unwrap();
    imu.set_acc_odr(0).await.unwrap();
    // imu.gyr_calibrate(100).await.unwrap();
    // imu.set_acc_dlp(dr_icm_20948::AccelerometerDlp::Hz6)
    //     .await
    //     .unwrap();

    // imu.set_gyr_dlp(dr_icm_20948::GyroscopeDlp::Hz6)
    //     .await
    //     .unwrap();

    let sample_period = (1.0) / (12.0);
    let filter = madgwick::Madgwick::new(sample_period, 0.1);

    unwrap!(spawner.spawn(driver(imu, filter)));
}

#[embassy_executor::task]
async fn driver(
    mut imu: Icm20948<I2c<'static, I2C1, Async>, MagEnabled, Init>,
    mut filter: madgwick::Madgwick<f64>,
) {
    // let mut x_max = 0.0;
    // let mut x_min = 0.0;
    // let mut y_max = 0.0;
    // let mut y_min = 0.0;
    // let mut z_max = 0.0;
    // let mut z_min = 0.0;

    loop {
        let data = imu.read_all().await.unwrap();

        let mut gyroscope = Vector3::new(data.gyr.x, data.gyr.y, data.gyr.z);
        gyroscope *= PI / 180.0;

        let accelerometer = Vector3::new(data.acc.x, data.acc.y, data.acc.z);
        // println!(
        //     "{} {} {}",
        //     accelerometer.x, accelerometer.y, accelerometer.z
        // );

        // println!(
        //     "{} {} {}",
        //     accelerometer.x, accelerometer.y, accelerometer.z
        // );

        // println!("{} {}", gyroscope.x, accelerometer.x);

        // добавить фильтр шума от магнитного поля моторов
        let magnetometer = Vector3::new(data.mag.x, data.mag.y, data.mag.z);

        let quat = filter
            .update(&gyroscope, &accelerometer, &magnetometer)
            .unwrap();

        let (x, y, z, w) = (
            quat.coords[0],
            quat.coords[1],
            quat.coords[2],
            quat.coords[3],
        );

        // println!("");

        // quaternion x y z w
        println!("{} {} {} {}", x, y, z, w);

        // x_max = x_max.max(x);
        // x_min = x_min.min(x);
        // y_max = y_max.max(y);
        // y_min = y_min.min(y);
        // z_max = z_max.max(z);
        // z_min = z_min.min(z);

        // println!("x_max: {} x_min: {} diff: {}", x_max, x_min, x_max - x_min);
        // println!("y_max: {} y_min: {} diff: {}", y_max, y_min, y_max - y_min);
        // println!("z_max: {} z_min: {} diff: {}", z_max, z_min, z_max - z_min);
    }
}
