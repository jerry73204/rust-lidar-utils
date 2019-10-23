/// A Cartesian point.
#[derive(Debug, Clone, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl From<SphericalPoint> for Point {
    fn from(spherical_point: SphericalPoint) -> Self {
        let SphericalPoint {
            distance,
            polar_angle,
            azimuth_angle,
        } = spherical_point;

        let x = distance * polar_angle.sin() * azimuth_angle.sin();
        let y = distance * polar_angle.sin() * azimuth_angle.cos();
        let z = distance * polar_angle.cos();

        Self { x, y, z }
    }
}

/// A point location in spherical coordinate system.
#[derive(Debug, Clone, PartialEq)]
pub struct SphericalPoint {
    pub distance: f64,
    pub azimuth_angle: f64,
    pub polar_angle: f64,
}

impl SphericalPoint {
    pub fn from_altitude_angle(distance: f64, azimuth_angle: f64, altitude_angle: f64) -> Self {
        let polar_angle = std::f64::consts::FRAC_PI_2 - altitude_angle;
        Self {
            distance,
            azimuth_angle,
            polar_angle,
        }
    }
}

impl From<Point> for SphericalPoint {
    fn from(point: Point) -> Self {
        let Point { x, y, z } = point;

        let distance = (x.powf(2.0) + y.powf(2.0) + z.powf(2.0)).sqrt();
        let azimuth_angle = {
            let angle = y.atan2(x);
            if angle < 0.0 {
                angle + 2.0 * std::f64::consts::PI
            } else {
                angle
            }
        };
        let polar_angle = if distance == 0.0 {
            0.0
        } else {
            (z / distance).acos()
        };

        Self {
            distance,
            azimuth_angle,
            polar_angle,
        }
    }
}

/// A struct that wraps arbitrary type with additional timestampm.
#[derive(Debug, Clone, PartialEq)]
pub struct Timestamped<T> {
    pub value: T,
    pub timestamp_ns: u64,
}

/// A pair of [Point] and [SphericalPoint].
#[derive(Debug, Clone, PartialEq)]
pub struct PointPair {
    pub cartesian: Point,
    pub spherical: SphericalPoint,
}

impl From<Point> for PointPair {
    fn from(cartesian: Point) -> Self {
        Self {
            spherical: cartesian.clone().into(),
            cartesian,
        }
    }
}

impl From<SphericalPoint> for PointPair {
    fn from(spherical: SphericalPoint) -> Self {
        Self {
            cartesian: spherical.clone().into(),
            spherical,
        }
    }
}
