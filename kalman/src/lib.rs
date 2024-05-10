#![no_std]
use nalgebra::{Matrix, Matrix2, Matrix3, Matrix3x2, Vector, Vector2, Vector3};

/// * `q`: process noise covariance
/// * `r`: measurement noise covariance
/// * `h`: observation matrix
/// * `f`: state transition matrix
/// * `x0`: initial guess for state mean at time 1
/// * `p0`: initial guess for state covariance at time 1
pub struct KalmanFilter {
    pub q: Matrix2<f32>,   // Process noise covariance
    pub r: Matrix3<f32>,   // Measurement noise covariance
    pub h: Matrix3x2<f32>, // Observation matrix
    pub f: Matrix2<f32>,   // State transition matrix
    pub x0: Vector2<f32>,  // State variable initial value
    pub p0: Matrix2<f32>,  // State covariance initial value
}

pub struct KalmanState {
    pub x: Vector2<f32>, // State vector
    pub p: Matrix2<f32>, // State covariance
}

impl KalmanFilter {
    pub fn filter(
        &self,
        data: &Vector3<Vector3<f32>>,
    ) -> (Vector3<KalmanState>, Vector3<KalmanState>) {
    }

    // pub fn filter(
    //     &self,
    //     data: &Vector3<Vector3<f32>>,
    // ) -> (Vector3<KalmanState>, Vector3<KalmanState>) {
    //     let t: usize = data.len();

    //     // Containers for predicted and filtered estimates
    //     let mut predicted: Vector3<KalmanState> = Vector4::new(0, 0, 0, 0);
    //     let mut filtered: Vector3<KalmanState> = Vector3::new(0.0, 0.0, 0.0);

    //     let kalman_state = KalmanState {
    //         x: (self.x0).clone(),
    //         p: (self.p0).clone(),
    //     };
    //     // predicted.push(KalmanState {
    //     //     x: (self.x0).clone(),
    //     //     p: (self.p0).clone(),
    //     // });

    //     for k in 0..t {
    //         filtered.push(update_step(self, &predicted[k], &data[k]));
    //         predicted.push(predict_step(self, &filtered[k]));
    //     }

    //     (filtered, predicted)
    // }

    // pub fn smooth(
    //     &self,
    //     filtered: &Vec<KalmanState>,
    //     predicted: &Vec<KalmanState>,
    // ) -> Vec<KalmanState> {
    //     let t: usize = filtered.len();
    //     let mut smoothed: Vec<KalmanState> = Vec::with_capacity(t);

    //     // Do Kalman smoothing in reverse order
    //     let mut init = (filtered[t - 1]).clone();
    //     smoothed.push((filtered[t - 1]).clone());

    //     for k in 1..t {
    //         smoothed.push(smoothing_step(
    //             self,
    //             &init,
    //             &filtered[t - k - 1],
    //             &predicted[t - k],
    //         ));
    //         init = (&smoothed[k]).clone();
    //     }

    //     smoothed.reverse();
    //     smoothed
    // }
}

pub fn predict_step(kalman_filter: &KalmanFilter, init: &KalmanState) -> KalmanState {
    // Predict state variable and covariance
    let xp: Vector2<f32> = &kalman_filter.f * &init.x;
    let pp: Matrix2<f32> =
        &kalman_filter.f * &init.p * &kalman_filter.f.transpose() + &kalman_filter.q;

    KalmanState { x: xp, p: pp }
}

pub fn update_step(
    kalman_filter: &KalmanFilter,
    pred: &KalmanState,
    measure: &Vector2<f32>,
) -> KalmanState {
    let identity = Matrix::<f32>::identity(kalman_filter.x0.size());

    // Compute Kalman gain
    let k: Matrix<f32> = &pred.p
        * &kalman_filter.h.transpose()
        * (&kalman_filter.h * &pred.p * &kalman_filter.h.transpose() + &kalman_filter.r)
            .inverse()
            .expect("Kalman gain computation failed due to failure to invert.");

    // Update state variable and covariance
    let x = &pred.x + &k * (measure - &kalman_filter.h * &pred.x);
    let p = (identity - &k * &kalman_filter.h) * &pred.p;

    KalmanState { x, p }
}

pub fn filter_step(
    kalman_filter: &KalmanFilter,
    init: &KalmanState,
    measure: &Vector<f32>,
) -> (KalmanState, KalmanState) {
    let pred = predict_step(kalman_filter, init);
    let upd = update_step(kalman_filter, &pred, measure);

    (
        KalmanState { x: upd.x, p: upd.p },
        KalmanState {
            x: pred.x,
            p: pred.p,
        },
    )
}

fn smoothing_step(
    kalman_filter: &KalmanFilter,
    init: &KalmanState,
    filtered: &KalmanState,
    predicted: &KalmanState,
) -> KalmanState {
    let a = Matrix2::new(1, 2, 3, 4).try_inverse();

    // let a = predicted
    //     .p
    //     .pseudo_inverse()
    //     .expect("Predicted state covariance matrix could not be inverted.");

    let j: Matrix2<f32> = &filtered.p * &kalman_filter.f.transpose() * a;

    let x: Vector2<f32> = &filtered.x + &j * (&init.x - &predicted.x);
    let p: Matrix2<f32> = &filtered.p + &j * (&init.p - &predicted.p) * &j.transpose();

    KalmanState { x, p }
}
