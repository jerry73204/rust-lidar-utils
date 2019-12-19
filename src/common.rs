use num_traits::{Float, Num};
use uom::{
    si::{angle::Angle, length::Length, Units},
    Conversion,
};

pub(crate) fn spherical_to_xyz<AngleUnit, LengthUnit, Value>(
    range: Length<LengthUnit, Value>,
    azimuth_angle: Angle<AngleUnit, Value>,
    altitude_angle: Angle<AngleUnit, Value>,
) -> [Length<LengthUnit, Value>; 3]
where
    LengthUnit: Units<Value> + ?Sized,
    AngleUnit: Units<Value> + ?Sized,
    Value: Conversion<Value> + Num + Float,
{
    let x = range * altitude_angle.sin() * azimuth_angle.cos();
    let y = range * altitude_angle.sin() * azimuth_angle.sin();
    let z = range * altitude_angle.cos();
    [x, y, z]
}
