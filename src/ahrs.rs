use nalgebra::{Scalar, SimdValue, UnitQuaternion, Vector3};

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
