use super::{
    context::{ConverterContext, DynamicReturnContext, ToConverterContext},
    data::{DualReturnPoint, DynamicReturnPoint, SingleReturnPoint},
};
use crate::velodyne::{
    config::Config,
    marker::{DualReturn, DynamicReturn, LastReturn, StrongestReturn},
    packet::{Packet, ReturnMode},
};
use failure::{ensure, Fallible};
use typenum::{U16, U32};

/// An _interface_ trait that is implemented by all variants of [PointCloudConverter]
pub trait PointCloudConverterInterface<ConfigType>
where
    ConfigType: ToConverterContext,
{
    fn from_config(config: ConfigType) -> PointCloudConverter<ConfigType>;
    fn convert<P>(
        &mut self,
        packet: P,
    ) -> Fallible<Vec<<ConfigType::Context as ConverterContext>::OutputPoint>>
    where
        P: AsRef<Packet>;
}

/// Converts UDP packets from a Velodyne LiDAR to points.
pub struct PointCloudConverter<ConfigType>
where
    ConfigType: ToConverterContext,
{
    context: ConfigType::Context,
}

impl PointCloudConverterInterface<Config<U16, StrongestReturn>>
    for PointCloudConverter<Config<U16, StrongestReturn>>
{
    fn from_config(config: Config<U16, StrongestReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::StrongestReturn);
        Ok(super::impls::convert_single_return_16_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Config<U16, LastReturn>>
    for PointCloudConverter<Config<U16, LastReturn>>
{
    fn from_config(config: Config<U16, LastReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::LastReturn);
        Ok(super::impls::convert_single_return_16_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Config<U16, DualReturn>>
    for PointCloudConverter<Config<U16, DualReturn>>
{
    fn from_config(config: Config<U16, DualReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<DualReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::DualReturn);
        Ok(super::impls::convert_dual_return_16_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Config<U16, DynamicReturn>>
    for PointCloudConverter<Config<U16, DynamicReturn>>
{
    fn from_config(config: Config<U16, DynamicReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<DynamicReturnPoint>>
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

impl PointCloudConverterInterface<Config<U32, StrongestReturn>>
    for PointCloudConverter<Config<U32, StrongestReturn>>
{
    fn from_config(config: Config<U32, StrongestReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::StrongestReturn);
        Ok(super::impls::convert_single_return_32_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Config<U32, LastReturn>>
    for PointCloudConverter<Config<U32, LastReturn>>
{
    fn from_config(config: Config<U32, LastReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::LastReturn);
        Ok(super::impls::convert_single_return_32_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Config<U32, DualReturn>>
    for PointCloudConverter<Config<U32, DualReturn>>
{
    fn from_config(config: Config<U32, DualReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<DualReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::DualReturn);
        Ok(super::impls::convert_dual_return_32_channel(
            &mut self.context,
            packet,
        ))
    }
}

impl PointCloudConverterInterface<Config<U32, DynamicReturn>>
    for PointCloudConverter<Config<U32, DynamicReturn>>
{
    fn from_config(config: Config<U32, DynamicReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<DynamicReturnPoint>>
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
