#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::{dbg, println};
use dr_icm_20948::{Init, MagDisabled};
use embassy_stm32::{
    bind_interrupts,
    i2c::I2c,
    mode::Async,
    peripherals::{self, I2C1},
};
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

    println!("0");
    let i2c = embassy_stm32::i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        hz(1000),
        i2c::Config::default(),
    );

    let imu = Icm20948::new(i2c);
    println!("1");
    let imu = imu.initialize_6dof().await.unwrap();
    println!("2");

    unwrap!(spawner.spawn(driver(imu)));
}

#[embassy_executor::task]
async fn driver(mut imu: Icm20948<I2c<'static, I2C1, Async>, MagDisabled, Init>) {
    loop {
        let data = imu.read_all().await.unwrap();

        println!(
            "gyr_x: {}\tgyr_y: {}\tgyr_z: {}",
            data.gyr.x, data.gyr.y, data.gyr.z
        );
        println!(
            "acc_x: {}\tacc_y: {}\tacc_z: {}",
            data.acc.x, data.acc.y, data.acc.z
        );
        println!("");
    }
}
