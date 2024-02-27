use measurements::Angle;
use std::{f64::consts::PI, time::Duration};

pub(crate) trait AngleExt {
    fn sin(self) -> f64;
    fn cos(self) -> f64;
    fn wrap_to_2pi(self) -> Self;
}

impl AngleExt for Angle {
    fn sin(self) -> f64 {
        self.as_radians().sin()
    }

    fn cos(self) -> f64 {
        self.as_radians().cos()
    }

    fn wrap_to_2pi(self) -> Self {
        let radians = self.as_radians() % (PI * 2.0);
        let radians = if radians >= 0.0 {
            radians
        } else {
            radians + PI * 2.0
        };
        Self::from_radians(radians)
    }
}

pub(crate) trait DurationExt {
    fn div_duration(self, rhs: Duration) -> f64;
}

impl DurationExt for Duration {
    fn div_duration(self, rhs: Duration) -> f64 {
        self.as_secs_f64() / rhs.as_secs_f64()
    }
}
