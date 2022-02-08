use crate::{
    common::*,
    utils::{AngleExt as _, DurationExt as _},
    velodyne::{
        config::LaserParameter,
        consts::{CHANNEL_PERIOD, FIRING_PERIOD},
        point::{FiringXyzSingle16, Measurement, PointSingle},
        DualFiring16, DualFiring32, FiringXyzDual16, FiringXyzDual32, FiringXyzSingle32, PointDual,
        SingleFiring16, SingleFiring32,
    },
};

pub fn firing_single_16_to_xyz(
    firing: &SingleFiring16,
    distance_resolution: Length,
    lasers: &[LaserParameter; 16],
) -> FiringXyzSingle16 {
    let SingleFiring16 {
        time: firing_time,
        ref azimuth_range,
        channels,
        block,
        ..
    } = *firing;

    let channel_times = iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD));

    let points: Vec<_> = izip!(0.., channel_times, channels, lasers)
        .map(move |(laser_id, channel_time, channel, laser)| {
            let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
            let LaserParameter {
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

            PointSingle {
                laser_id,
                time: channel_time,
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

    FiringXyzSingle16 {
        time: firing_time,
        azimuth_count: block.azimuth_count,
        azimuth_range: azimuth_range.clone(),
        points,
    }
}

pub fn firing_single_32_to_xyz(
    firing: &SingleFiring32,
    distance_resolution: Length,
    lasers: &[LaserParameter; 32],
) -> FiringXyzSingle32 {
    let SingleFiring32 {
        time: firing_time,
        ref azimuth_range,
        channels,
        block,
        ..
    } = *firing;

    let channel_times = iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD))
        .flat_map(|time| [time, time]);

    let points: Vec<_> = izip!(0.., channel_times, channels, lasers)
        .map(move |(laser_id, channel_time, channel, laser)| {
            // let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64((channel_idx / 2) as f64);

            let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
            let LaserParameter {
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

            PointSingle {
                laser_id,
                time: channel_time,
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

    FiringXyzSingle32 {
        time: firing_time,
        azimuth_count: block.azimuth_count,
        azimuth_range: azimuth_range.clone(),
        points,
    }
}

pub fn firing_dual_16_to_xyz(
    firing: &DualFiring16,
    distance_resolution: Length,
    lasers: &[LaserParameter; 16],
) -> FiringXyzDual16 {
    let DualFiring16 {
        time: firing_time,
        ref azimuth_range,
        channels_strongest,
        channels_last,
        block_strongest,
        ..
    } = *firing;

    let channel_times = iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD));

    let points: Vec<_> = izip!(
        0..,
        channel_times,
        channels_strongest,
        channels_last,
        lasers
    )
    .map(
        move |(laser_id, channel_time, channel_strongest, channel_last, laser)| {
            let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
            let LaserParameter {
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

            PointDual {
                laser_id,
                time: channel_time,
                azimuth,
                measurement_strongest: Measurement {
                    distance: distance_strongest,
                    intensity: channel_strongest.intensity,
                    xyz: xyz_strongest,
                },
                measurement_last: Measurement {
                    distance: distance_last,
                    intensity: channel_last.intensity,
                    xyz: xyz_last,
                },
            }
        },
    )
    .collect();
    let points: [_; 16] = points.try_into().unwrap_or_else(|_| unreachable!());

    FiringXyzDual16 {
        time: firing_time,
        azimuth_count: block_strongest.azimuth_count,
        azimuth_range: azimuth_range.clone(),
        points,
    }
}

pub fn firing_dual_32_to_xyz(
    firing: &DualFiring32,
    distance_resolution: Length,
    lasers: &[LaserParameter; 32],
) -> FiringXyzDual32 {
    let DualFiring32 {
        time: firing_time,
        ref azimuth_range,
        channels_strongest,
        channels_last,
        block_strongest,
        ..
    } = *firing;

    let channel_times = iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD))
        .flat_map(|time| [time, time]);

    let points: Vec<_> = izip!(
        0..,
        channel_times,
        channels_strongest,
        channels_last,
        lasers
    )
    .map(
        move |(laser_id, channel_time, channel_strongest, channel_last, laser)| {
            // let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64((channel_idx / 2) as f64);

            let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
            let LaserParameter {
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

            PointDual {
                laser_id,
                time: channel_time,
                azimuth,
                measurement_strongest: Measurement {
                    distance: distance_strongest,
                    intensity: channel_strongest.intensity,
                    xyz: xyz_strongest,
                },
                measurement_last: Measurement {
                    distance: distance_last,
                    intensity: channel_last.intensity,
                    xyz: xyz_last,
                },
            }
        },
    )
    .collect();

    let points: [_; 32] = points.try_into().unwrap_or_else(|_| unreachable!());

    FiringXyzDual32 {
        time: firing_time,
        azimuth_count: block_strongest.azimuth_count,
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
    // The origin of elevaion_angle lies on xy plane.
    // The azimuth angle starts from y-axis, rotates clockwise.

    let distance_plane = distance * elevation.cos() - vertical_offset * elevation.sin();
    let x = distance_plane * azimuth.sin() - horizontal_offset * azimuth.cos();
    let y = distance_plane * azimuth.cos() + horizontal_offset * azimuth.sin();
    let z = distance * elevation.sin() + vertical_offset * elevation.cos();
    [x, y, z]
}
