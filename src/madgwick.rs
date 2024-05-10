#![allow(non_snake_case)]
#![allow(clippy::many_single_char_names)]

use core::hash;
use nalgebra::{
    Matrix4, Matrix6, Quaternion, RealField, Scalar, SimdRealField, SimdValue, UnitQuaternion,
    Vector2, Vector3, Vector4, Vector6,
};

/// Trait for implementing an AHRS filter.
pub trait Ahrs<T: Scalar + SimdValue> {
    /// Attempts to update the current state quaternion using 9dof IMU values, made up by `gyroscope`,
    /// `accelerometer`, and `magnetometer`.
    ///
    /// Returns a reference to the updated quaternion on success, or in the case of failure, an
    /// `Err(&str)` containing the reason.
    fn update(
        &mut self,
        gyroscope: &Vector3<T>,
        accelerometer: &Vector3<T>,
        magnetometer: &Vector3<T>,
    ) -> Result<&UnitQuaternion<T>, &str>;

    /// Attempts to update the current state quaternion using 6dof IMU values, made up by `gyroscope` &
    /// `accelerometer`.
    ///
    /// Returns a reference to the updated quaternion on success, or in the case of failure, an
    /// `Err(&str)` containing the reason.
    fn update_imu(
        &mut self,
        gyroscope: &Vector3<T>,
        accelerometer: &Vector3<T>,
    ) -> Result<&UnitQuaternion<T>, &str>;
}

/// Madgwick AHRS implementation.
///
/// # Example
/// ```
/// # use ahrs::Madgwick;
/// let mut ahrs = Madgwick::new(0.002390625f64, 0.1);
/// println!("madgwick filter: {:?}", ahrs);
///
/// // Can now process IMU data using `Ahrs::update_imu`, etc.
/// ```
#[derive(Debug)]
pub struct Filter<T: Scalar + SimdValue + Copy> {
    /// Expected sampling period, in seconds.
    sample_period: T,
    /// Filter gain.
    beta: T,
    /// Filter state quaternion.
    pub quat: UnitQuaternion<T>,
}

impl<T: SimdRealField + Eq + Copy> Eq for Filter<T> where T::Element: SimdRealField + Copy {}

impl<T: SimdRealField + Copy> PartialEq for Filter<T>
where
    T::Element: SimdRealField + Copy,
{
    fn eq(&self, rhs: &Self) -> bool {
        self.sample_period == rhs.sample_period && self.beta == rhs.beta && self.quat == rhs.quat
    }
}

impl<T: SimdRealField + hash::Hash + Copy> hash::Hash for Filter<T> {
    fn hash<H: hash::Hasher>(&self, state: &mut H) {
        self.sample_period.hash(state);
        self.beta.hash(state);
        self.quat.hash(state);
    }
}

impl<T: Scalar + Copy + SimdValue> Copy for Filter<T> {}

impl<T: Scalar + SimdValue + Copy> Clone for Filter<T> {
    #[inline]
    fn clone(&self) -> Self {
        let sample_period = self.sample_period;
        let beta = self.beta;
        let quat = self.quat;

        Self {
            sample_period,
            beta,
            quat,
        }
    }
}

impl Default for Filter<f32> {
    /// Creates a new `Madgwick` instance with default filter parameters.
    ///
    /// ```
    /// # use ahrs::Madgwick;
    /// # use nalgebra::{Quaternion, Vector4};
    /// dbg!(Madgwick::default());
    ///
    /// // prints (roughly):
    /// //
    /// // Madgwick {
    /// //     sample_period: 1.0f64/256.0,
    /// //     beta: 0.1f64,
    /// //     quat: Quaternion { w: 1.0f64, i: 0.0, j: 0.0, k: 0.0 }
    /// // };
    /// ```
    fn default() -> Filter<f32> {
        Self {
            sample_period: (1.0f32) / (256.0),
            beta: 0.1f32,
            quat: UnitQuaternion::new_unchecked(Quaternion::new(1.0f32, 0.0, 0.0, 0.0)),
        }
    }
}

impl<T: Scalar + SimdValue + num_traits::One + num_traits::Zero + Copy> Filter<T> {
    /// Creates a new `Madgwick` AHRS instance with identity quaternion.
    ///
    /// # Arguments
    ///
    /// * `sample_period` - The expected sensor sampling period in seconds.
    /// * `beta` - Filter gain.
    pub fn new(sample_period: T, beta: T) -> Self {
        Filter::new_with_quat(
            sample_period,
            beta,
            UnitQuaternion::new_unchecked(Quaternion::new(
                T::one(),
                T::zero(),
                T::zero(),
                T::zero(),
            )),
        )
    }

    /// Creates a new `Madgwick` AHRS instance with given quaternion.
    ///
    /// # Arguments
    ///
    /// * `sample_period` - The expected sensor sampling period in seconds.
    /// * `beta` - Filter gain.
    /// * `quat` - Existing filter state quaternion.
    pub fn new_with_quat(sample_period: T, beta: T, quat: UnitQuaternion<T>) -> Self {
        Self {
            sample_period,
            beta,
            quat,
        }
    }
}

impl<T: RealField + Copy> Ahrs<T> for Filter<T> {
    fn update(
        &mut self,
        gyroscope: &Vector3<T>,
        accelerometer: &Vector3<T>,
        magnetometer: &Vector3<T>,
    ) -> Result<&UnitQuaternion<T>, &str> {
        let q = self.quat.as_ref();

        let zero: T = nalgebra::zero();
        let two: T = nalgebra::convert(2.0);
        let four: T = nalgebra::convert(4.0);
        let half: T = nalgebra::convert(0.5);

        // Normalize accelerometer measurement
        let accel = match accelerometer.try_normalize(zero) {
            Some(n) => n,
            None => return Err("Accelerometer norm divided by zero."),
        };

        // Normalize magnetometer measurement
        let mag = match magnetometer.try_normalize(zero) {
            Some(n) => n,
            None => {
                return Err("Magnetometer norm divided by zero.");
            }
        };

        // Reference direction of Earth's magnetic field (Quaternion should still be conj of q)
        let h = q * (Quaternion::from_parts(zero, mag) * q.conjugate());
        let b = Quaternion::new(zero, Vector2::new(h[0], h[1]).norm(), zero, h[2]);

        // Gradient descent algorithm corrective step
        #[rustfmt::skip]
        let F = Vector6::new(
            two*(       q[0]*q[2] - q[3]*q[1]) - accel[0],
            two*(       q[3]*q[0] + q[1]*q[2]) - accel[1],
            two*(half - q[0]*q[0] - q[1]*q[1]) - accel[2],
            two*b[0]*(half - q[1]*q[1] - q[2]*q[2]) + two*b[2]*(q[0]*q[2] - q[3]*q[1]) - mag[0],
            two*b[0]*(q[0]*q[1] - q[3]*q[2])        + two*b[2]*(       q[3]*q[0] + q[1]*q[2]) - mag[1],
            two*b[0]*(q[3]*q[1] + q[0]*q[2])        + two*b[2]*(half - q[0]*q[0] - q[1]*q[1]) - mag[2]
        );

        #[rustfmt::skip]
        let J_t = Matrix6::new(
            -two*q[1], two*q[0],       zero,                -two*b[2]*q[1], -two*b[0]*q[2]+two*b[2]*q[0], two*b[0]*q[1],
             two*q[2], two*q[3], -four*q[0],                 two*b[2]*q[2],  two*b[0]*q[1]+two*b[2]*q[3], two*b[0]*q[2]-four*b[2]*q[0],
            -two*q[3], two*q[2], -four*q[1], -four*b[0]*q[1]-two*b[2]*q[3],  two*b[0]*q[0]+two*b[2]*q[2], two*b[0]*q[3]-four*b[2]*q[1],
             two*q[0], two*q[1],       zero, -four*b[0]*q[2]+two*b[2]*q[0], -two*b[0]*q[3]+two*b[2]*q[1], two*b[0]*q[0],
             zero, zero, zero, zero, zero, zero,
             zero, zero, zero, zero, zero, zero
        );

        let step = (J_t * F).normalize();

        // Compute rate of change for quaternion
        let qDot = q * Quaternion::from_parts(zero, *gyroscope) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        // Integrate to yield quaternion
        self.quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);

        Ok(&self.quat)
    }

    fn update_imu(
        &mut self,
        gyroscope: &Vector3<T>,
        accelerometer: &Vector3<T>,
    ) -> Result<&UnitQuaternion<T>, &str> {
        let q = self.quat.as_ref();

        let zero: T = nalgebra::zero();
        let two: T = nalgebra::convert(2.0);
        let four: T = nalgebra::convert(4.0);
        let half: T = nalgebra::convert(0.5);

        // Tormalize accelerometer measurement
        let accel = match accelerometer.try_normalize(zero) {
            Some(n) => n,
            None => {
                return Err("Accelerator norm divided by zero.");
            }
        };

        // Gradient descent algorithm corrective step
        #[rustfmt::skip]
        let F = Vector4::new(
            two*(       q[0]*q[2] - q[3]*q[1]) - accel[0],
            two*(       q[3]*q[0] + q[1]*q[2]) - accel[1],
            two*(half - q[0]*q[0] - q[1]*q[1]) - accel[2],
            zero
        );

        #[rustfmt::skip]
        let J_t = Matrix4::new(
            -two*q[1], two*q[0],       zero, zero,
             two*q[2], two*q[3], -four*q[0], zero,
            -two*q[3], two*q[2], -four*q[1], zero,
             two*q[0], two*q[1],       zero, zero
        );

        let step = (J_t * F).normalize();

        // Compute rate of change of quaternion
        let qDot = (q * Quaternion::from_parts(zero, *gyroscope)) * half
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        // Integrate to yield quaternion
        self.quat = UnitQuaternion::from_quaternion(q + qDot * self.sample_period);

        Ok(&self.quat)
    }
}
