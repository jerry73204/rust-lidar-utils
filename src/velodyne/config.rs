use super::marker::ReturnTypeMarker;
use std::{convert::TryFrom, fmt::Debug, marker::PhantomData};
use uom::si::{
    angle::radian,
    f64::{Angle as F64Angle, Length as F64Length},
    length::millimeter,
    time::microsecond,
    u32::Length as U32Length,
};

pub trait VelodyneConfig
where
    Self: Debug + Clone,
{
}

#[derive(Debug, Clone)]
pub struct Config16Channel<ReturnType>
where
    ReturnType: ReturnTypeMarker,
{
    /// Vertical angles per laser in degrees.
    pub vertical_degrees: [f64; 16],
    /// Vertical correction per laser in millimeters.
    pub vertical_corrections: [f64; 16],
    _phantom: PhantomData<ReturnType>,
}

impl<ReturnType> VelodyneConfig for Config16Channel<ReturnType> where ReturnType: ReturnTypeMarker {}

impl<ReturnType> Config16Channel<ReturnType>
where
    ReturnType: ReturnTypeMarker,
{
    pub fn vlp_16_config() -> Self {
        Self {
            vertical_degrees: [
                -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0,
                -1.0, 15.0,
            ],
            vertical_corrections: [
                11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7,
                -11.2,
            ],
            _phantom: PhantomData,
        }
    }

    pub fn puke_lite_config() -> Self {
        Self {
            vertical_degrees: [
                -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0,
                -1.0, 15.0,
            ],
            vertical_corrections: [
                11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7,
                -11.2,
            ],
            _phantom: PhantomData,
        }
    }

    pub fn puke_hi_res_config() -> Self {
        Self {
            vertical_degrees: [
                -10.00, 0.67, -8.67, 2.00, -7.33, 3.33, -6.00, 4.67, -4.67, 6.00, -3.33, 7.33,
                -2.00, 8.67, -0.67, 10.00,
            ],
            vertical_corrections: [
                7.4, -0.9, 6.5, -1.8, 5.5, -2.7, 4.6, -3.7, 3.7, -4.6, 2.7, -5.5, 1.8, -6.5, 0.9,
                -7.4,
            ],
            _phantom: PhantomData,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Config32Channel<ReturnType>
where
    ReturnType: ReturnTypeMarker,
{
    /// Vertical angles per laser in degrees.
    pub vertical_degrees: [f64; 32],
    /// Vertical correction per laser in millimeters.
    pub vertical_corrections: [f64; 32],
    _phantom: PhantomData<ReturnType>,
}

impl<ReturnType> VelodyneConfig for Config32Channel<ReturnType> where ReturnType: ReturnTypeMarker {}
