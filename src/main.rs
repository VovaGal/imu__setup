// #![no_std]
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod ahrs;
mod madgwick;
mod mahony;
use lowpass::lowpass_filter;

use crate::ahrs::Ahrs;
use core::{f64::consts::PI};
use defmt::println;
use dr_icm_20948::{Init, MagEnabled};
use embassy_stm32::{
    bind_interrupts,
    i2c::{I2c, Instance},
    mode::Async,
    peripherals::{self, I2C1},
};
use embassy_time::{Duration, Timer, Instant};
use nalgebra::Vector3;
use {
    defmt::unwrap,
    defmt_rtt as _,
    dr_icm_20948::Icm20948,
    embassy_executor::Spawner,
    embassy_stm32::{i2c, time::hz, Config},
    panic_probe as _,
};

use gy91::*;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());

    let i2c = embassy_stm32::i2c::I2c::new(
        p.I2C1,
        p.PB6, //was pb8
        p.PB7, //was pb9
        Irqs,
        p.DMA1_CH6, //was DMA1_CH6
        p.DMA1_CH0, //was DMA1_CH0
        hz(8000),
        i2c::Config::default(),
    );

    let imu = Icm20948::new(i2c);


    let mut imu = imu.initialize_9dof().await.unwrap();

    imu.set_gyr_odr(0).await.unwrap();

    imu.set_acc_odr(0).await.unwrap();

    // adils calibrations
//     imu.set_mag_calibration(offset, scale);
//     imu.gyr_calibrate(180).await.unwrap(); //approx 360 readings in 10 secs
//     Timer::after_secs(5).await;


    imu.gyr_calibrate(180).await.unwrap(); //approx 360 readings in 10 secs
//    Timer::after_secs(5).await;

    // as our gyro is off by a lot, set this to 0.33
    let sample_period = (0.007);  //should be 0.0005, but thats 1/10 of the needed sens, and 0.00075 to compensate for only half rotation, but the .5 is overtune
    let mut gain = 0.0; // start witih  2.5 for 10 sec (rn it takes 3 sec) then switch to 0.033
    let mut filter = madgwick::Madgwick::new(sample_period, gain);
    unwrap!(spawner.spawn(driver(imu, filter)));
}

// WIP change gain after 10 secs
pub fn change(val: &mut madgwick::Madgwick<f64>) {
    *val = madgwick::Madgwick::new(0.007, 0.033);
}


#[embassy_executor::task]
async fn driver(
    mut imu: Icm20948<I2c<'static, I2C1, Async>, MagEnabled, Init>,
    mut filter: madgwick::Madgwick<f64>,
    ) {

    let mut some = 0;
//
//    let start = Instant::MIN;
//    let times = Duration::as_secs(&Instant::elapsed(&start));
//    println!("{}", times);

    loop {
//        while some <= 90 {
//            loop {
//                let data = imu.read_all().await.unwrap();
//
//                let mut gyroscope = Vector3::new(data.gyr.x, data.gyr.y, data.gyr.z);
//                gyroscope *= PI / 180.0;
//
//                let accelerometer = Vector3::new(data.acc.x, data.acc.y, data.acc.z);
//                // добавить фильтр шума от магнитного поля моторов
//                let magnetometer = Vector3::new(data.mag.x, data.mag.y, data.mag.z);
//
//                let quat = filter
//                    .update(&gyroscope, &accelerometer, &magnetometer)
//        //            .update_imu(&gyroscope, &accelerometer)
//                    .unwrap();
//
//                let (x, y, z, w) = (
//                    quat.coords[0],
//                    quat.coords[1],
//                    quat.coords[2],
//                    quat.coords[3],
//                );
//                println!("{} {} {} {}", x, y, z, w);
//                some += 1;
//                break;
//            }
//        }

//        if some == 91 {
//            change(&mut filter);
//        }

//        imu.set_gyr_offsets(Vector3::new(166, 0, 0)).await.unwrap(); // i16, so i8 for each high and low
        let data = imu.read_all().await.unwrap();

        let mut gyroscope = Vector3::new(data.gyr.x, data.gyr.y, data.gyr.z);
        gyroscope *= PI / 180.0;

        let accelerometer = Vector3::new(data.acc.x, data.acc.y, data.acc.z);
        // добавить фильтр шума от магнитного поля моторов
        let magnetometer = Vector3::new(data.mag.x, data.mag.y, data.mag.z);

        let quat = filter
            .update(&gyroscope, &accelerometer, &magnetometer)
//            .update_imu(&gyroscope, &accelerometer)
            .unwrap();

        let (x, y, z, w) = (
            quat.coords[0],
            quat.coords[1],
            quat.coords[2],
            quat.coords[3],
        );
        println!("{} {} {} {}", x, y, z, w);
    }
}
