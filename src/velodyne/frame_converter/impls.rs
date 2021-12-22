use super::converter::RemainingPoints;
use crate::{
    common::*,
    velodyne::{
        marker::{ModelMarker, ReturnTypeMarker},
        packet::DataPacket,
        pcd_converter::PointCloudConverter,
        point::{DualReturnPoint, DynamicReturnPoints, SingleReturnPoint, VelodynePoint},
    },
};

pub(crate) fn convert_single_return<PcdConverter, Model, ReturnType>(
    pcd_converter: &mut PcdConverter,
    remaining_points: &mut Vec<SingleReturnPoint>,
    packet: &DataPacket,
) -> Result<Vec<Vec<SingleReturnPoint>>>
where
    PcdConverter: PointCloudConverter<Model, ReturnType, Output = Vec<SingleReturnPoint>>,
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    let points = remaining_points
        .drain(..)
        .chain(pcd_converter.convert(packet)?.into_iter());

    let (frames, new_remaining_points) = points_to_frames(points);
    let _ = mem::replace(remaining_points, new_remaining_points);
    Ok(frames)
}

pub(crate) fn convert_dual_return<PcdConverter, Model, ReturnType>(
    pcd_converter: &mut PcdConverter,
    remaining_points: &mut Vec<DualReturnPoint>,
    packet: &DataPacket,
) -> Result<Vec<Vec<DualReturnPoint>>>
where
    PcdConverter: PointCloudConverter<Model, ReturnType, Output = Vec<DualReturnPoint>>,
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    let points = remaining_points
        .drain(..)
        .chain(pcd_converter.convert(packet)?.into_iter());
    let (frames, new_remaining_points) = points_to_frames(points);
    let _ = mem::replace(remaining_points, new_remaining_points);
    Ok(frames)
}

pub(crate) fn convert_dynamic_return<PcdConverter, Model, ReturnType>(
    pcd_converter: &mut PcdConverter,
    remaining_points: &mut RemainingPoints,
    packet: &DataPacket,
) -> Result<Vec<DynamicReturnPoints>>
where
    PcdConverter: PointCloudConverter<Model, ReturnType, Output = DynamicReturnPoints>,
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    let new_points = pcd_converter.convert(packet)?;
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
    let mut remaining_points: Vec<Point> = vec![];
    let mut prev_azimuth = None;
    let mut remaining_channel = vec![];

    let mut prev_lazer_id = u32::MIN;
    let mut col_idx_cnt = 0;

    let mut beam_num = 0;

    points.into_iter().for_each(|mut point| {
        let curr_azimuth = point.original_azimuth_angle();
        let pass_zero_azimuth = prev_azimuth.map_or(false, |prev| curr_azimuth < prev);

        if pass_zero_azimuth && point.col_idx() > 0 {
            let mut frame: Vec<Point> = vec![];

            // sort channel order by row_idx
            for i in 0..=(remaining_points.len() / beam_num) - 1 {
                remaining_points[i * beam_num..(i * beam_num + beam_num)]
                    .sort_by(|a, b| a.row_idx().partial_cmp(&b.row_idx()).unwrap());
            }

            frame.append(&mut remaining_points);
            frames.push(frame);

            //reset line ID for new frame
            col_idx_cnt = 0;
        } else {
            if prev_lazer_id > point.laser_id() {
                assert!(prev_lazer_id == 15 || prev_lazer_id == 31);
                assert!(point.laser_id() == 0);

                //check whether it is 32 beam or 16 beam
                beam_num = (prev_lazer_id + 1) as usize;

                //append to remaining_points when a line is collected
                remaining_points.append(&mut remaining_channel);

                //update line ID for next line
                col_idx_cnt = col_idx_cnt + 1;
            }

            //set line ID
            point.set_col_idx(col_idx_cnt);

            prev_lazer_id = point.laser_id();
            remaining_channel.push(point);
        }

        prev_azimuth = Some(curr_azimuth);
    });

    //Collect all remaining point
    let mut remain: Vec<Point> = vec![];
    remain.append(&mut remaining_points);
    remain.append(&mut remaining_channel);

    (frames, remain)
}
