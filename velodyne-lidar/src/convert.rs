use crate::{
    config::Beam,
    consts::{CHANNEL_PERIOD, FIRING_PERIOD},
    types::{
        channel_array::ChannelArrayDRef,
        firing_block::{FiringBlockD16, FiringBlockD32, FiringBlockS16, FiringBlockS32},
        firing_xyz::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
        measurements::{Measurement, MeasurementDual},
        point::{PointD, PointS},
    },
    utils::{AngleExt as _, DurationExt as _},
    Config16, Config32,
};
use itertools::izip;
use measurements::{Angle, Length};
use std::iter;

pub fn firing_block_to_xyz_s16(firing: &FiringBlockS16, beams: &Config16) -> FiringXyzS16 {
    let Config16 {
        ref lasers,
        distance_resolution,
        ..
    } = *beams;
    let FiringBlockS16 {
        toh: firing_toh,
        ref azimuth_range,
        channels,
        ..
    } = *firing;

    let channel_tohs = iter::successors(Some(firing_toh), |&prev| Some(prev + CHANNEL_PERIOD));

    let points: Vec<_> = izip!(0.., channel_tohs, channels, lasers)
        .map(move |(laser_id, channel_toh, channel, laser)| -> PointS {
            let ratio = (channel_toh - firing_toh).div_duration(FIRING_PERIOD);
            let Beam {
                elevation,
                azimuth_offset,
                vertical_offset,
                horizontal_offset,
            } = *laser;

            // clockwise angle with origin points to front of sensor
            let azimuth = {
                let azimuth = azimuth_range.start
                    + ((azimuth_range.end - azimuth_range.start) * ratio)
                    + azimuth_offset;
                azimuth.wrap_to_2pi()
            };
            let distance = distance_resolution * channel.distance as f64;
            let xyz = spherical_to_xyz(
                distance,
                elevation,
                azimuth,
                vertical_offset,
                horizontal_offset,
            );

            PointS {
                laser_id,
                toh: channel_toh,
                azimuth,
                measurement: Measurement {
                    distance,
                    intensity: channel.intensity,
                    xyz,
                },
            }
        })
        .collect();
    let points: [_; 16] = points.try_into().unwrap_or_else(|_| unreachable!());

    FiringXyzS16 {
        toh: firing_toh,
        azimuth_range: azimuth_range.clone(),
        points,
    }
}

pub fn firing_block_to_xyz_s32(firing: &FiringBlockS32, beams: &Config32) -> FiringXyzS32 {
    let Config32 {
        ref lasers,
        distance_resolution,
        ..
    } = *beams;
    let FiringBlockS32 {
        toh: firing_toh,
        ref azimuth_range,
        channels,
        ..
    } = *firing;

    let channel_tohs = iter::successors(Some(firing_toh), |&prev| Some(prev + CHANNEL_PERIOD))
        .flat_map(|toh| [toh, toh]);

    let points: Vec<_> = izip!(0.., channel_tohs, channels, lasers)
        .map(move |(laser_id, channel_toh, channel, laser)| {
            // let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64((channel_idx / 2) as f64);

            let ratio = (channel_toh - firing_toh).div_duration(FIRING_PERIOD);
            let Beam {
                elevation,
                azimuth_offset,
                vertical_offset,
                horizontal_offset,
            } = *laser;

            // clockwise angle with origin points to front of sensor
            let azimuth = {
                let azimuth = azimuth_range.start
                    + ((azimuth_range.end - azimuth_range.start) * ratio)
                    + azimuth_offset;
                azimuth.wrap_to_2pi()
            };
            let distance = distance_resolution * channel.distance as f64;
            let xyz = spherical_to_xyz(
                distance,
                elevation,
                azimuth,
                vertical_offset,
                horizontal_offset,
            );

            PointS {
                laser_id,
                toh: channel_toh,
                azimuth,
                measurement: Measurement {
                    distance,
                    intensity: channel.intensity,
                    xyz,
                },
            }
        })
        .collect();

    let points = points.try_into().unwrap_or_else(|_| unreachable!());

    FiringXyzS32 {
        toh: firing_toh,
        azimuth_range: azimuth_range.clone(),
        points,
    }
}

pub fn firing_block_to_xyz_d16(firing: &FiringBlockD16, beams: &Config16) -> FiringXyzD16 {
    let Config16 {
        ref lasers,
        distance_resolution,
        ..
    } = *beams;
    let FiringBlockD16 {
        toh: firing_toh,
        ref azimuth_range,
        channels:
            ChannelArrayDRef {
                strongest: channels_strongest,
                last: channels_last,
            },
        ..
    }: FiringBlockD16<'_> = *firing;

    let channel_tohs = iter::successors(Some(firing_toh), |&prev| Some(prev + CHANNEL_PERIOD));

    let points: Vec<_> = izip!(0.., channel_tohs, channels_strongest, channels_last, lasers)
        .map(
            move |(laser_id, channel_toh, channel_strongest, channel_last, laser)| -> PointD {
                let ratio = (channel_toh - firing_toh).div_duration(FIRING_PERIOD);
                let Beam {
                    elevation,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = *laser;

                // clockwise angle with origin points to front of sensor
                let azimuth = {
                    let azimuth = azimuth_range.start
                        + ((azimuth_range.end - azimuth_range.start) * ratio)
                        + azimuth_offset;
                    azimuth.wrap_to_2pi()
                };
                let distance_strongest = distance_resolution * channel_strongest.distance as f64;
                let distance_last = distance_resolution * channel_last.distance as f64;

                let xyz_strongest = spherical_to_xyz(
                    distance_strongest,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );
                let xyz_last = spherical_to_xyz(
                    distance_last,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );

                PointD {
                    laser_id,
                    toh: channel_toh,
                    azimuth,
                    measurements: MeasurementDual {
                        strongest: Measurement {
                            distance: distance_strongest,
                            intensity: channel_strongest.intensity,
                            xyz: xyz_strongest,
                        },
                        last: Measurement {
                            distance: distance_last,
                            intensity: channel_last.intensity,
                            xyz: xyz_last,
                        },
                    },
                }
            },
        )
        .collect();
    let points: [_; 16] = points.try_into().unwrap_or_else(|_| unreachable!());

    FiringXyzD16 {
        toh: firing_toh,
        azimuth_range: azimuth_range.clone(),
        points,
    }
}

pub fn firing_block_to_xyz_d32(firing: &FiringBlockD32, beams: &Config32) -> FiringXyzD32 {
    let Config32 {
        ref lasers,
        distance_resolution,
        ..
    } = *beams;
    let FiringBlockD32 {
        toh: firing_toh,
        ref azimuth_range,
        channels:
            ChannelArrayDRef {
                strongest: channels_strongest,
                last: channels_last,
            },
        ..
    } = *firing;

    let channel_tohs = iter::successors(Some(firing_toh), |&prev| Some(prev + CHANNEL_PERIOD))
        .flat_map(|toh| [toh, toh]);

    let points: Vec<_> = izip!(0.., channel_tohs, channels_strongest, channels_last, lasers)
        .map(
            move |(laser_id, channel_toh, channel_strongest, channel_last, laser)| {
                // let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64((channel_idx / 2) as f64);

                let ratio = (channel_toh - firing_toh).div_duration(FIRING_PERIOD);
                let Beam {
                    elevation,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = *laser;

                // clockwise angle with origin points to front of sensor
                let azimuth = {
                    let azimuth = azimuth_range.start
                        + ((azimuth_range.end - azimuth_range.start) * ratio)
                        + azimuth_offset;
                    azimuth.wrap_to_2pi()
                };
                let distance_strongest = distance_resolution * channel_strongest.distance as f64;
                let distance_last = distance_resolution * channel_last.distance as f64;

                let xyz_strongest = spherical_to_xyz(
                    distance_strongest,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );
                let xyz_last = spherical_to_xyz(
                    distance_last,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );

                PointD {
                    laser_id,
                    toh: channel_toh,
                    azimuth,
                    measurements: MeasurementDual {
                        strongest: Measurement {
                            distance: distance_strongest,
                            intensity: channel_strongest.intensity,
                            xyz: xyz_strongest,
                        },
                        last: Measurement {
                            distance: distance_last,
                            intensity: channel_last.intensity,
                            xyz: xyz_last,
                        },
                    },
                }
            },
        )
        .collect();

    let points: [_; 32] = points.try_into().unwrap_or_else(|_| unreachable!());

    FiringXyzD32 {
        toh: firing_toh,
        azimuth_range: azimuth_range.clone(),
        points,
    }
}

pub fn spherical_to_xyz(
    distance: Length,
    elevation: Angle,
    azimuth: Angle,
    vertical_offset: Length,
    horizontal_offset: Length,
) -> [Length; 3] {
    spherical_to_xyz_generic(
        distance,
        elevation,
        azimuth,
        vertical_offset,
        horizontal_offset,
    )

    // #[cfg(any(
    //     not(feature = "fast_approx"),
    //     not(any(target_arch = "x86", target_arch = "x86_64"))
    // ))]
    // {
    //     spherical_to_xyz_generic(
    //         distance,
    //         elevation,
    //         azimuth,
    //         vertical_offset,
    //         horizontal_offset,
    //     )
    // }

    // #[cfg(all(
    //     feature = "fast_approx",
    //     any(target_arch = "x86", target_arch = "x86_64")
    // ))]
    // {
    //     spherical_to_xyz_x86(
    //         distance,
    //         elevation,
    //         azimuth,
    //         vertical_offset,
    //         horizontal_offset,
    //     )
    // }
}

pub fn spherical_to_xyz_generic(
    distance: Length,
    elevation: Angle,
    azimuth: Angle,
    vertical_offset: Length,
    horizontal_offset: Length,
) -> [Length; 3] {
    // The origin of elevaion_angle lies on xy plane.
    // The azimuth angle starts from y-axis, rotates clockwise.

    let elevation_sin = elevation.sin();
    let elevation_cos = elevation.cos();
    let azimuth_sin = azimuth.sin();
    let azimuth_cos = azimuth.cos();

    let distance_plane = distance * elevation_cos - vertical_offset * elevation_sin;
    let x = distance_plane * azimuth_sin - horizontal_offset * azimuth_cos;
    let y = distance_plane * azimuth_cos + horizontal_offset * azimuth_sin;
    let z = distance * elevation_sin + vertical_offset * elevation_cos;
    [x, y, z]
}

// #[allow(dead_code)]
// #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
// pub fn spherical_to_xyz_x86(
//     distance: Length,
//     elevation: Angle,
//     azimuth: Angle,
//     vertical_offset: Length,
//     horizontal_offset: Length,
// ) -> [Length; 3] {
//     use fastapprox::fast::{cosfull, sin, sinfull};
//     use std::f64::consts::FRAC_PI_2;

//     debug_assert!(((-FRAC_PI_2)..=FRAC_PI_2).contains(&elevation.as_radians()));

//     let elevation_sin = sin(elevation.as_radians() as f32) as f64;
//     let elevation_cos = 1.0 - elevation_sin.abs();
//     let azimuth_sin = sinfull(azimuth.as_radians() as f32) as f64;
//     let azimuth_cos = cosfull(azimuth.as_radians() as f32) as f64;

//     let distance_plane = distance * elevation_cos - vertical_offset * elevation_sin;
//     let x = distance_plane * azimuth_sin - horizontal_offset * azimuth_cos;
//     let y = distance_plane * azimuth_cos + horizontal_offset * azimuth_sin;
//     let z = distance * elevation_sin + vertical_offset * elevation_cos;
//     [x, y, z]
// }
