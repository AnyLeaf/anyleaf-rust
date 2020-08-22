use filter::kalman::kalman_filter::KalmanFilter;
use nalgebra::{
    dimension::{U1, U2},
    Matrix1, Matrix2, Vector2,
};
use num_traits::float::FloatCore;

/// Initialize the kalman filter. dt is the time between measurements, in seconds.
pub fn create(_dt: f32, std: f32) -> KalmanFilter<f32, U2, U1, U1> {
    let mut kf = KalmanFilter::default();

    // The initialization value doesn't matter much, since the filter
    // will correct quickly. Initialize to 7 pH, and no change in pH over time.
    kf.x = Vector2::new(7.0, 0.0); // initial state (pH, and dpH_dt)

    kf.F = Matrix2::new(
        // state transition matrix
        1.0, 1.0, 0.0, 1.0,
    );

    kf.H = Vector2::new(1.0, 0.0).transpose(); // Measurement function

    // Initialization of the covariance matrix is based on how close our
    // initialization is. Since we always initialize to 0.7, let's assume
    // most pH measurements will be within a few pH units of it.
    kf.P *= 3.0; // covariance matrix

    // High values of state uncertainty mean more smoothing. (Similar to low noise
    // variance)
    kf.R = Matrix1::new(0.01); // state uncertainty, ie measurement noise

    // Process uncertainly variance appears to have a large effect on both
    // the response time, and stability of the result. Smaller values means slower
    // response, more overshooting, and more smoothing. Dt is related to this.

    // todo: How do we match this with the py code equivalent that uses
    // todo discrete white noise, and compensates for dt?
    kf.Q = Matrix2::repeat(std.powi(2));
    kf
}
