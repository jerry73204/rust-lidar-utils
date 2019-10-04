use super::{Firing, DEFAULT_ALTITUDE_DEGREES, ENCODER_TICKS_PER_REV, LASER_PER_FIRING};
use failure::Fallible;
use ndarray::Array3;

pub struct PointCloudConverter {
    altitude_degrees: [f64; LASER_PER_FIRING],
    spherical_projection: Array3<f64>,
}

impl PointCloudConverter {
    /// Construct helper from altitude degrees for each laser beam.
    pub fn new(altitude_degrees: [f64; LASER_PER_FIRING]) -> Self {
        let num_columns = ENCODER_TICKS_PER_REV - 1;
        let num_rows = altitude_degrees.len();
        let mut spherical_projection = Array3::<f64>::zeros((num_columns, num_rows, 3));

        (0..num_columns).into_iter().for_each(|col_index| {
            let azimuth_angle = 2.0 * std::f64::consts::PI * col_index as f64 / num_columns as f64;

            altitude_degrees
                .iter()
                .enumerate()
                .for_each(|(row_index, altitude_deg)| {
                    let altitude_angle = std::f64::consts::PI * altitude_deg / 180.0;

                    // TODO: the formula is different from upstream
                    // https://github.com/PointCloudLibrary/pcl/blob/master/io/src/hdl_grabber.cpp#L396
                    let x = altitude_angle.cos() * azimuth_angle.cos();
                    let y = altitude_angle.cos() * azimuth_angle.sin();
                    let z = altitude_angle.sin();

                    spherical_projection[(col_index, row_index, 0)] = x;
                    spherical_projection[(col_index, row_index, 1)] = y;
                    spherical_projection[(col_index, row_index, 2)] = z;
                })
        });

        Self {
            altitude_degrees,
            spherical_projection,
        }
    }

    pub fn altitude_degrees(&self) -> &[f64; LASER_PER_FIRING] {
        &self.altitude_degrees
    }

    pub fn spherical_projection(&self) -> &Array3<f64> {
        &self.spherical_projection
    }

    /// Compute point locations from firing data from sensor.
    pub fn firing_to_points(&self, firing: &Firing) -> Fallible<Vec<(f64, f64, f64)>> {
        ensure!(
            (firing.encoder_ticks as usize) < ENCODER_TICKS_PER_REV,
            "encoder_ticks is out of bound"
        );
        let azimuth_angle = firing.azimuth_angle();
        let points = firing
            .laster_returns
            .iter()
            .zip(self.altitude_degrees.iter())
            .map(|(laster_return, altitude_deg)| {
                let altitude_angle = std::f64::consts::PI * altitude_deg / 180.0;
                let distance = laster_return.meter_distance();

                // TODO: the formula is different from upstream
                // https://github.com/PointCloudLibrary/pcl/blob/master/io/src/hdl_grabber.cpp#L396
                let x = distance * altitude_angle.cos() * azimuth_angle.cos();
                let y = distance * altitude_angle.cos() * azimuth_angle.sin();
                let z = distance * altitude_angle.sin();

                (x, y, z)
            })
            .collect::<Vec<_>>();
        Ok(points)
    }
}

impl Default for PointCloudConverter {
    fn default() -> Self {
        Self::new(DEFAULT_ALTITUDE_DEGREES)
    }
}

// References
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L312
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L396
