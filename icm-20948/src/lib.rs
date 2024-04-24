#![feature(type_changing_struct_update)]
#![no_std]

mod reg;

use common::AsI2c as I2c;
use core::marker::PhantomData;
use embassy_time::{Duration, Timer};
use nalgebra::Vector3;
use reg::{Bank0, Bank2, Bank3, MagBank, Register, UserBank, REG_BANK_SEL};

const ICM20948_ADDR: u8 = 0x68;
const MAGNET_ADDR: u8 = 0x0C;

#[derive(Clone, Copy)]
/// Container for accelerometer and gyroscope measurements
pub struct Data6Dof {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub tmp: f32,
}

#[derive(Clone, Copy)]
/// Container for accelerometer, gyroscope and magnetometer measurements
pub struct Data9Dof {
    pub acc: Vector3<f32>,
    pub gyr: Vector3<f32>,
    pub mag: Vector3<f32>,
    pub tmp: f32,
}

pub struct MagEnabled {
    is_calibrated: bool,
    offset: Vector3<f32>,
    scale: Vector3<f32>,
}

pub struct MagDisabled;

pub struct Init;

pub struct NotInit;

pub struct Icm20948<I2C, MAG, INIT> {
    i2c: I2C,
    addr: u8,
    config: Icm20948Config,
    user_bank: UserBank,
    mag_state: MAG,
    init_state: PhantomData<INIT>,
}

impl<I2C, E> Icm20948<I2C, MagDisabled, NotInit>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    /// Creates an uninitialized IMU struct with the given config.
    #[must_use]
    pub fn new_from_cfg(i2c: I2C, cfg: Icm20948Config) -> Icm20948<I2C, MagDisabled, NotInit> {
        Self {
            i2c,
            addr: ICM20948_ADDR,
            config: cfg,
            user_bank: UserBank::Bank0,
            mag_state: MagDisabled,
            init_state: PhantomData::<NotInit>,
        }
    }

    /// Creates an uninitialized IMU struct with a default config on address 0x68
    #[must_use]
    pub fn new(i2c: I2C) -> Icm20948<I2C, MagDisabled, NotInit> {
        Self::new_from_cfg(i2c, Icm20948Config::default())
    }

    /*
        Configuration methods
    */

    /// Set I2C address of ICM module. Default is 0x68, alternative is 0x69
    #[must_use]
    pub fn set_address(self, addr: u8) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 { addr, ..self }
    }

    /// Set accelerometer measuring range, choices are 2G, 4G, 8G or 16G
    #[must_use]
    pub fn acc_range(self, acc_range: AccelerometerRange) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                acc_range,
                ..self.config
            },
            ..self
        }
    }

    /// Set accelerometer digital low-pass filter frequency
    #[must_use]
    pub fn acc_dlp(self, acc_dlp: AccelerometerDlp) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                acc_dlp,
                ..self.config
            },
            ..self
        }
    }

    /// Set returned unit of accelerometer measurement, choices are Gs or m/s^2
    #[must_use]
    pub fn acc_unit(self, acc_unit: AccelerometerUnit) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                acc_unit,
                ..self.config
            },
            ..self
        }
    }

    /// Set accelerometer output data rate
    #[must_use]
    pub fn acc_odr(self, acc_odr: u16) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                acc_odr,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope measuring range, choices are 250Dps, 500Dps, 1000Dps and 2000Dps
    #[must_use]
    pub fn gyr_range(self, gyr_range: GyroscopeRange) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                gyr_range,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope digital low pass filter frequency
    #[must_use]
    pub fn gyr_dlp(self, gyr_dlp: GyroscopeDlp) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                gyr_dlp,
                ..self.config
            },
            ..self
        }
    }

    /// Set returned unit of gyroscope measurement, choices are degrees/s or radians/s
    #[must_use]
    pub fn gyr_unit(self, gyr_unit: GyroscopeUnit) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                gyr_unit,
                ..self.config
            },
            ..self
        }
    }

    /// Set gyroscope output data rate
    #[must_use]
    pub fn gyr_odr(self, gyr_odr: u8) -> Icm20948<I2C, MagDisabled, NotInit> {
        Icm20948 {
            config: Icm20948Config {
                gyr_odr,
                ..self.config
            },
            ..self
        }
    }

    /*
        Initialization methods
    */

    /// Initializes the IMU with accelerometer and gyroscope
    pub async fn initialize_6dof(
        mut self,
    ) -> Result<Icm20948<I2C, MagDisabled, Init>, IcmError<E>> {
        self.setup_acc_gyr().await?;

        Ok(Icm20948 {
            mag_state: MagDisabled,
            init_state: PhantomData::<Init>,
            ..self
        })
    }

    /// Initializes the IMU with accelerometer, gyroscope and magnetometer
    pub async fn initialize_9dof(mut self) -> Result<Icm20948<I2C, MagEnabled, Init>, IcmError<E>> {
        self.setup_acc_gyr().await?;
        self.setup_mag().await?;

        Ok(Icm20948 {
            mag_state: MagEnabled {
                is_calibrated: false,
                offset: Vector3::from_element(1.),
                scale: Vector3::from_element(1.),
            },
            init_state: PhantomData::<Init>,
            ..self
        })
    }

    /// Setup accelerometer and gyroscope according to config
    async fn setup_acc_gyr(&mut self) -> Result<(), IcmError<E>> {
        // Ensure known-good state
        self.device_reset().await?;

        // Initially set user bank by force, and check identity
        self.set_user_bank(&Bank0::WhoAmI, true).await?;
        let [id] = self.read_from(Bank0::WhoAmI).await?;

        if id != 0xEA {
            return Err(IcmError::ImuSetupError);
        }

        // Disable sleep mode and set to auto select clock source
        self.write_to(Bank0::PwrMgmt1, 0x01).await?;

        // Set gyro and accel ranges
        self.set_acc_range(self.config.acc_range).await?;
        self.set_gyr_range(self.config.gyr_range).await?;

        // Apply digital low-pass filter settings
        self.set_gyr_dlp(self.config.gyr_dlp).await?;
        self.set_acc_dlp(self.config.acc_dlp).await?;

        // Set gyro and accel output data rate
        self.set_acc_odr(self.config.acc_odr).await?;
        self.set_gyr_odr(self.config.gyr_odr).await?;

        Ok(())
    }

    /// Setup magnetometer in continuous mode
    async fn setup_mag(&mut self) -> Result<(), IcmError<E>> {
        // Ensure known-good state
        self.mag_reset().await?;

        // Setup magnetometer (i2c slave) clock (default 400 kHz)
        self.write_to(Bank3::I2cMstCtrl, 0x07).await?;

        // Enable I2C master mode for magnetometer
        self.enable_i2c_master(true).await?;

        // Configure slave address as magnetometer
        self.write_to(Bank3::I2cSlv0Addr, MAGNET_ADDR).await?;

        // Verify magnetometer identifier
        let [wai] = self.mag_read_from(MagBank::DeviceId).await?;

        if wai != 9 {
            return Err(IcmError::MagSetupError);
        }

        // Reset magnetometer
        self.mag_reset().await?;

        // Set magnetometer to continuous mode 4 (100 Hz)
        self.mag_write_to(MagBank::Control2.reg(), 0b01000).await?;

        // Set slave register to read from
        self.write_to(Bank3::I2cSlv0Reg, MagBank::XDataLow.reg())
            .await?;

        // Set expected read size
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | 8).await?;

        Ok(())
    }
}

impl<I2C, E, MAG, INIT> Icm20948<I2C, MAG, INIT>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    /// Reset accelerometer / gyroscope module
    pub async fn device_reset(&mut self) -> Result<(), E> {
        Timer::after_millis(20).await;
        self.write_to_flag(Bank0::PwrMgmt1, 1 << 7, 1 << 7).await?;
        Timer::after_millis(50).await;
        Ok(())
    }

    /// Enables main ICM module to act as I2C master (eg. for magnetometer)
    async fn enable_i2c_master(&mut self, enable: bool) -> Result<(), E> {
        self.write_to_flag(Bank0::UserCtrl, u8::from(enable) << 5, 1 << 5)
            .await
    }

    /// Resets I2C master module
    async fn reset_i2c_master(&mut self) -> Result<(), E> {
        self.write_to_flag(Bank0::UserCtrl, 1 << 1, 1 << 1).await
    }

    /// Ensure correct user bank for given register
    async fn set_user_bank<R: Register + Copy>(&mut self, bank: &R, force: bool) -> Result<(), E> {
        if (self.user_bank != bank.bank()) || force {
            self.i2c
                .write(self.addr, &[REG_BANK_SEL, (bank.bank() as u8) << 4])
                .await?;
            self.user_bank = bank.bank();
        }
        Ok(())
    }

    /// Read a const number `N` of bytes from the requested register
    async fn read_from<const N: usize, R: Register + Copy>(
        &mut self,
        cmd: R,
    ) -> Result<[u8; N], E> {
        let mut buf = [0u8; N];
        self.set_user_bank(&cmd, false).await?;
        self.i2c
            .write_read(self.addr, &[cmd.reg()], &mut buf)
            .await?;
        Ok(buf)
    }

    /// Write a single byte to the requested register
    async fn write_to<R: Register + Copy>(&mut self, cmd: R, data: u8) -> Result<(), E> {
        self.set_user_bank(&cmd, false).await?;
        self.i2c.write(self.addr, &[cmd.reg(), data]).await
    }

    /// Write to a register, but only overwrite the parts corresponding to the flag byte
    async fn write_to_flag<R>(&mut self, cmd: R, data: u8, flag: u8) -> Result<(), E>
    where
        R: Register + Copy + Clone,
    {
        let [mut register] = self.read_from(cmd).await?;
        register = (register & !flag) | (data & flag);
        self.write_to(cmd, register).await
    }

    /// Put the magnetometer into read mode
    async fn set_mag_read(&mut self) -> Result<(), E> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b0111_1111;
        reg |= 1 << 7;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    /// Put the magnetometer into write mode
    async fn set_mag_write(&mut self) -> Result<(), E> {
        let [mut reg] = self.read_from(Bank3::I2cSlv0Addr).await?;
        reg &= 0b0111_1111;
        self.write_to(Bank3::I2cSlv0Addr, reg).await
    }

    /// Write `data` to the magnetometer module in `reg` (20 ms non-blocking delays)
    async fn mag_write_to(&mut self, reg: u8, data: u8) -> Result<(), E> {
        self.set_mag_write().await?;
        Timer::after_millis(10).await;
        self.write_to(Bank3::I2cSlv0Reg, reg).await?;
        self.write_to(Bank3::I2cSlv0Do, data).await?;
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | 1).await?;
        Timer::after_millis(10).await;
        self.set_mag_read().await
    }

    /// Read a `N` bytes from the magnetometer in `reg` (20 ms non-blocking delays)
    #[allow(clippy::cast_possible_truncation)]
    async fn mag_read_from<const N: usize>(&mut self, reg: MagBank) -> Result<[u8; N], E> {
        self.set_mag_read().await?;
        Timer::after_millis(10).await;
        self.write_to(Bank3::I2cSlv0Reg, reg.reg()).await?;
        self.write_to(Bank3::I2cSlv0Ctrl, 1 << 7 | N as u8).await?;
        Timer::after_millis(10).await;
        self.read_from(Bank0::ExtSlvSensData00).await
    }

    /// Reset magnetometer module ( 120 ms non-blocking delays)
    async fn mag_reset(&mut self) -> Result<(), E> {
        // Control 3 register bit 1 resets magnetometer unit
        self.mag_write_to(MagBank::Control3.reg(), 1).await?;
        Timer::after_millis(100).await;

        // Reset i2c master module
        self.reset_i2c_master().await
    }

    /// Configure accelerometer to measure with given range
    pub async fn set_acc_range(&mut self, range: AccelerometerRange) -> Result<(), E> {
        self.write_to_flag(Bank2::AccelConfig, (range as u8) << 1, 0b0110)
            .await?;
        self.config.acc_range = range;
        Ok(())
    }

    /// Configure gyroscope to measure with given range
    pub async fn set_gyr_range(&mut self, range: GyroscopeRange) -> Result<(), E> {
        self.write_to_flag(Bank2::GyroConfig1, (range as u8) << 1, 0b0110)
            .await?;
        self.config.gyr_range = range;
        Ok(())
    }

    /// Set returned unit of accelerometer
    pub fn set_acc_unit(&mut self, unit: AccelerometerUnit) {
        self.config.acc_unit = unit;
    }

    /// Set returned unit of gyroscope
    pub fn set_gyr_unit(&mut self, unit: GyroscopeUnit) {
        self.config.gyr_unit = unit;
    }

    /// Set (or disable) accelerometer digital low-pass filter
    pub async fn set_acc_dlp(&mut self, acc_dlp: AccelerometerDlp) -> Result<(), E> {
        if AccelerometerDlp::Disabled == acc_dlp {
            self.write_to_flag(Bank2::AccelConfig, 0u8, 0b0011_1001)
                .await
        } else {
            self.write_to_flag(Bank2::AccelConfig, (acc_dlp as u8) << 3 | 1, 0b0011_1001)
                .await
        }
    }

    /// Set (or disable) gyroscope digital low-pass filter
    pub async fn set_gyr_dlp(&mut self, gyr_dlp: GyroscopeDlp) -> Result<(), E> {
        if GyroscopeDlp::Disabled == gyr_dlp {
            self.write_to_flag(Bank2::GyroConfig1, 0u8, 0b0011_1001)
                .await
        } else {
            self.write_to_flag(Bank2::GyroConfig1, (gyr_dlp as u8) << 3 | 1, 0b0011_1001)
                .await
        }
    }

    /// Set accelerometer output data rate. Value will be clamped above 4095.
    pub async fn set_acc_odr(&mut self, acc_odr: u16) -> Result<(), E> {
        let [msb, lsb] = acc_odr.clamp(0, 0xFFF).to_be_bytes();
        self.write_to(Bank2::AccelSmplrtDiv1, msb).await?;
        self.write_to(Bank2::AccelSmplrtDiv2, lsb).await
    }

    /// Set gyroscope output data rate.
    pub async fn set_gyr_odr(&mut self, gyr_odr: u8) -> Result<(), E> {
        self.write_to(Bank2::GyroSmplrtDiv, gyr_odr).await
    }
}

impl<I2C, E> Icm20948<I2C, MagEnabled, Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    /// Apply the saved calibration offset+scale to measurement vector
    fn apply_mag_calibration(&self, mag: &mut Vector3<f32>) {
        if self.mag_state.is_calibrated {
            *mag = mag.zip_zip_map(&self.mag_state.offset, &self.mag_state.scale, |m, o, s| {
                (m - o) / s
            });
        }
    }

    /// Set magnetometer calibration data (offset,scale)
    pub fn set_mag_calibration(&mut self, offset: [f32; 3], scale: [f32; 3]) {
        self.mag_state.is_calibrated = true;
        self.mag_state.offset = offset.into();
        self.mag_state.scale = scale.into();
    }

    /// Resets (disables) magnetometer calibration data
    pub fn reset_mag_calibration(&mut self) {
        self.mag_state.is_calibrated = false;
        self.mag_state.offset = Vector3::from_element(1.);
        self.mag_state.scale = Vector3::from_element(1.);
    }

    /// Get vector of scaled magnetometer values
    #[allow(clippy::cast_lossless)]
    pub async fn read_mag(&mut self) -> Result<Vector3<f32>, E> {
        let mut mag = self.read_mag_unscaled().await?.map(|x| (x as f32)).into();
        self.apply_mag_calibration(&mut mag);

        Ok(mag)
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_mag_unscaled(&mut self) -> Result<[i16; 3], E> {
        let raw: [u8; 6] = self.read_from(Bank0::ExtSlvSensData00).await?;
        let mag = collect_3xi16_mag(raw);

        Ok(mag)
    }

    /// Get scaled measurement for accelerometer, gyroscope and magnetometer, and temperature
    #[allow(clippy::similar_names)]
    pub async fn read_all(&mut self) -> Result<Data9Dof, E> {
        let raw: [u8; 20] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl, mxl, mxh, myl, myh, mzl, mzh]: [u8; 20] =
            raw;

        let acc = self.scaled_acc_from_bytes([axh, axl, ayh, ayl, azh, azl]);
        let gyr = self.scaled_gyr_from_bytes([gxh, gxl, gyh, gyl, gzh, gzl]);
        let mag = self.scaled_mag_from_bytes([mxl, mxh, myl, myh, mzl, mzh]);

        let tmp = scaled_tmp_from_bytes([tph, tpl]);

        Ok(Data9Dof { acc, gyr, mag, tmp })
    }

    /// Takes 6 bytes converts them into a Vector3 of floats
    #[allow(clippy::cast_lossless)]
    fn scaled_mag_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let mut mag = collect_3xi16_mag(bytes).map(|x| (x as f32)).into();
        self.apply_mag_calibration(&mut mag);
        mag
    }
}

impl<I2C, E> Icm20948<I2C, MagDisabled, Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    /// Get scaled measurements for accelerometer and gyroscope, and temperature
    #[allow(clippy::similar_names)]
    pub async fn read_all(&mut self) -> Result<Data6Dof, E> {
        let raw: [u8; 14] = self.read_from(Bank0::AccelXoutH).await?;
        let [axh, axl, ayh, ayl, azh, azl, gxh, gxl, gyh, gyl, gzh, gzl, tph, tpl] = raw;

        let acc = self.scaled_acc_from_bytes([axh, axl, ayh, ayl, azh, azl]);
        let gyr = self.scaled_gyr_from_bytes([gxh, gxl, gyh, gyl, gzh, gzl]);

        let tmp = scaled_tmp_from_bytes([tph, tpl]);

        Ok(Data6Dof { acc, gyr, tmp })
    }
}

impl<I2C, E, MAG> Icm20948<I2C, MAG, Init>
where
    I2C: I2c<Error = E>,
    E: Into<IcmError<E>>,
{
    /// Takes 6 bytes converts them into a Vector3 of floats
    fn scaled_acc_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let acc = collect_3xi16(bytes).map(|x| f32::from(x) * self.acc_scalar());
        Vector3::from(acc)
    }

    /// Takes 6 bytes converts them into a Vector3 of floats
    fn scaled_gyr_from_bytes(&self, bytes: [u8; 6]) -> Vector3<f32> {
        let gyr = collect_3xi16(bytes).map(|x| f32::from(x) * self.gyr_scalar());
        Vector3::from(gyr)
    }

    /// Get array of unscaled accelerometer values
    pub async fn read_acc_unscaled(&mut self) -> Result<Vector3<i16>, E> {
        let raw = self.read_from(Bank0::AccelXoutH).await?;
        Ok(collect_3xi16(raw).into())
    }

    /// Get array of scaled accelerometer values
    pub async fn read_acc(&mut self) -> Result<Vector3<f32>, E> {
        let acc = self
            .read_acc_unscaled()
            .await?
            .map(|x| f32::from(x) * self.acc_scalar());
        Ok(acc)
    }

    /// Get array of unscaled gyroscope values
    pub async fn read_gyr_unscaled(&mut self) -> Result<Vector3<i16>, E> {
        let raw = self.read_from(Bank0::GyroXoutH).await?;
        Ok(collect_3xi16(raw).into())
    }

    /// Get array of scaled gyroscope values
    pub async fn read_gyr(&mut self) -> Result<Vector3<f32>, E> {
        let gyr = self
            .read_gyr_unscaled()
            .await?
            .map(|x| f32::from(x) * self.gyr_scalar());
        Ok(gyr)
    }

    /// Collects and averages `num` samples for gyro calibration and saves them on-chip
    #[allow(clippy::cast_lossless)]
    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::cast_possible_wrap)]
    pub async fn gyr_calibrate(&mut self, num: usize) -> Result<(), E> {
        let mut offset: Vector3<i32> = Vector3::default();
        for _ in 0..num {
            offset += self.read_gyr_unscaled().await?.map(|x| x as i32);
            Timer::after(Duration::from_hz(100)).await;
        }

        self.set_gyr_offsets(offset.map(|x| { x / num as i32 } as i16))
            .await
    }

    /// Set gyroscope calibration offsets by writing them to the IMU
    async fn set_gyr_offsets(&mut self, offsets: Vector3<i16>) -> Result<(), E> {
        let [[xh, xl], [yh, yl], [zh, zl]]: [[u8; 2]; 3] =
            offsets.map(|x| (-x).to_be_bytes()).into();

        self.set_user_bank(&Bank2::XgOffsUsrh, false).await?;
        self.i2c
            .write(self.addr, &[Bank2::XgOffsUsrh.reg(), xh, xl])
            .await?;
        self.i2c
            .write(self.addr, &[Bank2::YgOffsUsrh.reg(), yh, yl])
            .await?;
        self.i2c
            .write(self.addr, &[Bank2::ZgOffsUsrh.reg(), zh, zl])
            .await
    }

    pub async fn new_data_ready(&mut self) -> u8 {
        if let Ok([byte]) = self.read_from(Bank0::DataRdyStatus).await {
            byte
        } else {
            0
        }
    }

    /// Returns the scalar corresponding to the unit and range configured
    fn acc_scalar(&self) -> f32 {
        self.config.acc_unit.scalar() / self.config.acc_range.divisor()
    }

    /// Returns the scalar corresponding to the unit and range configured
    fn gyr_scalar(&self) -> f32 {
        self.config.gyr_unit.scalar() / self.config.gyr_range.divisor()
    }
}

/// Collects 6 bytes into a vector of i16 values (acc/gyr only)
fn collect_3xi16(values: [u8; 6]) -> [i16; 3] {
    let [xh, xl, yh, yl, zh, zl] = values;
    [
        i16::from_be_bytes([xh, xl]),
        i16::from_be_bytes([yh, yl]),
        i16::from_be_bytes([zh, zl]),
    ]
}

/// Collects 6 bytes into a vector of i16 values (mag only)
fn collect_3xi16_mag(values: [u8; 6]) -> [i16; 3] {
    let [xl, xh, yl, yh, zl, zh] = values;
    [
        i16::from_be_bytes([xh, xl]),
        -i16::from_be_bytes([yh, yl]),
        -i16::from_be_bytes([zh, zl]),
    ]
}

#[derive(Copy, Clone)]
pub struct Icm20948Config {
    pub acc_range: AccelerometerRange,
    pub gyr_range: GyroscopeRange,
    pub acc_unit: AccelerometerUnit,
    pub gyr_unit: GyroscopeUnit,
    pub acc_dlp: AccelerometerDlp,
    pub gyr_dlp: GyroscopeDlp,
    pub acc_odr: u16,
    pub gyr_odr: u8,
}

impl Default for Icm20948Config {
    fn default() -> Self {
        Self {
            acc_range: AccelerometerRange::Gs2,
            gyr_range: GyroscopeRange::Dps1000,
            acc_unit: AccelerometerUnit::Gs,
            gyr_unit: GyroscopeUnit::Dps,
            acc_dlp: AccelerometerDlp::Disabled,
            gyr_dlp: GyroscopeDlp::Disabled,
            acc_odr: Default::default(),
            gyr_odr: Default::default(),
        }
    }
}

#[derive(Copy, Clone)]
pub enum AccelerometerRange {
    Gs2 = 0b00,
    Gs4 = 0b01,
    Gs8 = 0b10,
    Gs16 = 0b11,
}

impl AccelerometerRange {
    #[must_use]
    pub fn divisor(self) -> f32 {
        match self {
            Self::Gs2 => 16384.0,
            Self::Gs4 => 8192.0,
            Self::Gs8 => 4096.0,
            Self::Gs16 => 2048.0,
        }
    }
}

#[derive(Copy, Clone)]
pub enum GyroscopeRange {
    Dps250 = 0b00,
    Dps500 = 0b01,
    Dps1000 = 0b10,
    Dps2000 = 0b11,
}

impl GyroscopeRange {
    #[must_use]
    pub fn divisor(self) -> f32 {
        match self {
            Self::Dps250 => 131.0,
            Self::Dps500 => 65.5,
            Self::Dps1000 => 32.8,
            Self::Dps2000 => 16.4,
        }
    }
}

#[derive(Copy, Clone)]
pub enum AccelerometerUnit {
    /// Meters per second squared (m/s^2)
    MpSs,
    /// Number of times of normal gravity
    Gs,
}

impl AccelerometerUnit {
    #[must_use]
    pub fn scalar(self) -> f32 {
        match self {
            Self::MpSs => 9.82,
            Self::Gs => 1.0,
        }
    }
}

#[derive(Copy, Clone)]
pub enum GyroscopeUnit {
    /// Radians per second
    Rps,
    /// Degrees per second
    Dps,
}

impl GyroscopeUnit {
    #[must_use]
    pub fn scalar(self) -> f32 {
        match self {
            Self::Rps => 0.017_453_293,
            Self::Dps => 1.0,
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
pub enum AccelerometerDlp {
    Hz473 = 7,
    Hz246 = 1,
    Hz111 = 2,
    Hz50 = 3,
    Hz24 = 4,
    Hz12 = 5,
    Hz6 = 6,
    Disabled = 8,
}

#[derive(Copy, Clone, PartialEq)]
pub enum GyroscopeDlp {
    Hz361 = 7,
    Hz196 = 0,
    Hz152 = 1,
    Hz120 = 2,
    Hz51 = 3,
    Hz24 = 4,
    Hz12 = 5,
    Hz6 = 6,
    Disabled = 8,
}

#[derive(Debug)]
pub enum IcmError<E> {
    BusError(E),
    ImuSetupError,
    MagSetupError,
}

impl<E> From<E> for IcmError<E> {
    fn from(error: E) -> Self {
        IcmError::BusError(error)
    }
}

/// Takes 2 bytes converts them into a temperature as a float
fn scaled_tmp_from_bytes(bytes: [u8; 2]) -> f32 {
    f32::from(i16::from_be_bytes(bytes)) / 333.87 + 21.
}
