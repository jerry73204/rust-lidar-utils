use super::{
    context::{ConverterContext, DynamicReturnContext, ToConverterContext},
    data::{DualReturnPoint, DynamicReturnPoint, SingleReturnPoint},
};
use crate::velodyne::{
    config::Config,
    marker::{
        DualReturn, DynamicReturn, LastReturn, ModelMarker, ReturnTypeMarker, StrongestReturn,
        Vlp16, Vlp32,
    },
    packet::{Packet, ReturnMode},
};
use anyhow::{ensure, Result};
use typenum::{U16, U32};

/// An _interface_ trait that is implemented by all variants of [PointCloudConverter]
pub trait PointCloudConverterInterface<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
    Config<Model, ReturnType>: ToConverterContext,
{
    fn from_config(config: Config<Model, ReturnType>) -> PointCloudConverter<Model, ReturnType>;
    fn convert<P>(
        &mut self,
        packet: P,
    ) -> Result<Vec<<<Config<Model, ReturnType> as ToConverterContext>::Context as ConverterContext>::OutputPoint>>
    where
        P: AsRef<Packet>;
}

/// Converts UDP packets from a Velodyne LiDAR to points.
pub struct PointCloudConverter<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
    Config<Model, ReturnType>: ToConverterContext,
{
    context: <Config<Model, ReturnType> as ToConverterContext>::Context,
}

impl PointCloudConverterInterface<Vlp16, StrongestReturn>
    for PointCloudConverter<Vlp16, StrongestReturn>
{
    fn from_config(config: Config<Vlp16, StrongestReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(
            packet.as_ref().return_mode == ReturnMode::StrongestReturn,
            "return mode does not match"
        );
        Ok(super::impls::convert_single_return_16_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Vlp16, LastReturn> for PointCloudConverter<Vlp16, LastReturn> {
    fn from_config(config: Config<Vlp16, LastReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(
            packet.as_ref().return_mode == ReturnMode::LastReturn,
            "return mode does not match"
        );
        Ok(super::impls::convert_single_return_16_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Vlp16, DualReturn> for PointCloudConverter<Vlp16, DualReturn> {
    fn from_config(config: Config<Vlp16, DualReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<DualReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(
            packet.as_ref().return_mode == ReturnMode::DualReturn,
            "return mode does not match"
        );
        Ok(super::impls::convert_dual_return_16_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Vlp16, DynamicReturn>
    for PointCloudConverter<Vlp16, DynamicReturn>
{
    fn from_config(config: Config<Vlp16, DynamicReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<DynamicReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        let points = match &mut self.context {
            DynamicReturnContext::SingleReturn(context) => {
                super::impls::convert_single_return_16_channel(context, packet)
                    .into_iter()
                    .map(|point| DynamicReturnPoint::from(point))
                    .collect::<Vec<_>>()
            }
            DynamicReturnContext::DualReturn(context) => {
                super::impls::convert_dual_return_16_channel(context, packet)
                    .into_iter()
                    .map(|point| DynamicReturnPoint::from(point))
                    .collect::<Vec<_>>()
            }
        };

        Ok(points)
    }
}

impl PointCloudConverterInterface<Vlp32, StrongestReturn>
    for PointCloudConverter<Vlp32, StrongestReturn>
{
    fn from_config(config: Config<Vlp32, StrongestReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(
            packet.as_ref().return_mode == ReturnMode::StrongestReturn,
            "return mode does not match"
        );
        Ok(super::impls::convert_single_return_32_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Vlp32, LastReturn> for PointCloudConverter<Vlp32, LastReturn> {
    fn from_config(config: Config<Vlp32, LastReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(
            packet.as_ref().return_mode == ReturnMode::LastReturn,
            "return mode does not match"
        );
        Ok(super::impls::convert_single_return_32_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Vlp32, DualReturn> for PointCloudConverter<Vlp32, DualReturn> {
    fn from_config(config: Config<Vlp32, DualReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<DualReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(
            packet.as_ref().return_mode == ReturnMode::DualReturn,
            "return mode does not match"
        );
        Ok(super::impls::convert_dual_return_32_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Vlp32, DynamicReturn>
    for PointCloudConverter<Vlp32, DynamicReturn>
{
    fn from_config(config: Config<Vlp32, DynamicReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<DynamicReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        let points = match &mut self.context {
            DynamicReturnContext::SingleReturn(context) => {
                super::impls::convert_single_return_32_channel(context, packet)
                    .into_iter()
                    .map(|point| DynamicReturnPoint::from(point))
                    .collect::<Vec<_>>()
            }
            DynamicReturnContext::DualReturn(context) => {
                super::impls::convert_dual_return_32_channel(context, packet)
                    .into_iter()
                    .map(|point| DynamicReturnPoint::from(point))
                    .collect::<Vec<_>>()
            }
        };

        Ok(points)
    }
}
