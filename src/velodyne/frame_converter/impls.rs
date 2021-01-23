use super::converter::RemainingPoints;
use crate::{
    common::*,
    velodyne::{
        marker::{ModelMarker, ReturnTypeMarker},
        packet::Packet,
        pcd_converter::{
            DualReturnPoint, DynamicReturnPoints, PointCloudConverter, SingleReturnPoint,
            VelodynePoint,
        },
    },
};

pub(crate) fn convert_single_return<PcdConverter, Model, ReturnType>(
    pcd_converter: &mut PcdConverter,
    remaining_points: &mut Vec<SingleReturnPoint>,
    packet: &Packet,
) -> Result<Vec<Vec<SingleReturnPoint>>>
where
    PcdConverter: PointCloudConverter<Model, ReturnType, Output = Vec<SingleReturnPoint>>,
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    let points = remaining_points
        .drain(..)
        .chain(pcd_converter.convert(&packet)?.into_iter());
    let (frames, new_remaining_points) = points_to_frames(points);
    let _ = mem::replace(remaining_points, new_remaining_points);
    Ok(frames)
}

pub(crate) fn convert_dual_return<PcdConverter, Model, ReturnType>(
    pcd_converter: &mut PcdConverter,
    remaining_points: &mut Vec<DualReturnPoint>,
    packet: &Packet,
) -> Result<Vec<Vec<DualReturnPoint>>>
where
    PcdConverter: PointCloudConverter<Model, ReturnType, Output = Vec<DualReturnPoint>>,
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    let points = remaining_points
        .drain(..)
        .chain(pcd_converter.convert(&packet)?.into_iter());
    let (frames, new_remaining_points) = points_to_frames(points);
    let _ = mem::replace(remaining_points, new_remaining_points);
    Ok(frames)
}

pub(crate) fn convert_dynamic_return<PcdConverter, Model, ReturnType>(
    pcd_converter: &mut PcdConverter,
    remaining_points: &mut RemainingPoints,
    packet: &Packet,
) -> Result<Vec<DynamicReturnPoints>>
where
    PcdConverter: PointCloudConverter<Model, ReturnType, Output = DynamicReturnPoints>,
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    let new_points = pcd_converter.convert(&packet)?;
    let frames = match (remaining_points, new_points) {
        (
            RemainingPoints(DynamicReturnPoints::Single(remaining_points)),
            DynamicReturnPoints::Single(new_points),
        ) => {
            let points = remaining_points.drain(..).chain(new_points.into_iter());
            let (frames, new_remaining_points) = points_to_frames(points);
            let _ = mem::replace(remaining_points, new_remaining_points);
            let frames: Vec<_> = frames
                .into_iter()
                .map(DynamicReturnPoints::Single)
                .collect();
            frames
        }
        (
            RemainingPoints(DynamicReturnPoints::Dual(remaining_points)),
            DynamicReturnPoints::Dual(new_points),
        ) => {
            let points = remaining_points.drain(..).chain(new_points.into_iter());
            let (frames, new_remaining_points) = points_to_frames(points);
            let _ = mem::replace(remaining_points, new_remaining_points);
            let frames: Vec<_> = frames.into_iter().map(DynamicReturnPoints::Dual).collect();
            frames
        }
        _ => unreachable!(),
    };
    Ok(frames)
}

fn points_to_frames<Point>(points: impl IntoIterator<Item = Point>) -> (Vec<Vec<Point>>, Vec<Point>)
where
    Point: VelodynePoint,
{
    let mut frames = vec![];
    let mut remaining_points = vec![];
    let mut prev_azimuth = None;

    points.into_iter().for_each(|point| {
        let curr_azimuth = point.original_azimuth_angle();
        let pass_zero_azimuth = prev_azimuth.map_or(false, |prev| curr_azimuth < prev);

        if pass_zero_azimuth {
            let frame = mem::replace(&mut remaining_points, vec![point]);
            frames.push(frame);
        } else {
            remaining_points.push(point);
        }
        prev_azimuth = Some(curr_azimuth);
    });

    (frames, remaining_points)
}
