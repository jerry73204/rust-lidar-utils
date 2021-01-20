use super::{
    context::{ConverterContext, DynamicReturnContext, ToConverterContext},
    data::{DualReturnPoint, DynamicReturnPoint, SingleReturnPoint},
};
use crate::{
    common::*,
    velodyne::{
        config::Config,
        marker::{
            DualReturn, DynamicReturn, LastReturn, ModelMarker, ReturnTypeMarker, StrongestReturn,
            Vlp16, Vlp32,
        },
        packet::{Packet, ReturnMode},
    },
};

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

/// Point cloud converter for VLP-16 device with strongest return mode.
pub type Vlp16StrongestPcdConverter = PointCloudConverter<Vlp16, StrongestReturn>;

/// Point cloud converter for VLP-16 device with last return mode.
pub type Vlp16LastPcdConverter = PointCloudConverter<Vlp16, LastReturn>;

/// Point cloud converter for VLP-16 device with dual return mode.
pub type Vlp16DualPcdConverter = PointCloudConverter<Vlp16, DualReturn>;

/// Point cloud converter for VLP-16 device with return mode configured in runtime.
pub type Vlp16DynamicPcdConverter = PointCloudConverter<Vlp16, DynamicReturn>;

/// Point cloud converter for VLP-32 device with strongest return mode.
pub type Vlp32StrongestPcdConverter = PointCloudConverter<Vlp32, StrongestReturn>;

/// Point cloud converter for VLP-32 device with last return mode.
pub type Vlp32LastPcdConverter = PointCloudConverter<Vlp32, LastReturn>;

/// Point cloud converter for VLP-32 device with dual return mode.
pub type Vlp32DualPcdConverter = PointCloudConverter<Vlp32, DualReturn>;

/// Point cloud converter for VLP-32 device with return mode configured in runtime.
pub type Vlp32DynamicPcdConverter = PointCloudConverter<Vlp32, DynamicReturn>;

impl PointCloudConverterInterface<Vlp16, StrongestReturn> for Vlp16StrongestPcdConverter {
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

impl PointCloudConverterInterface<Vlp16, LastReturn> for Vlp16LastPcdConverter {
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

impl PointCloudConverterInterface<Vlp16, DualReturn> for Vlp16DualPcdConverter {
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

impl PointCloudConverterInterface<Vlp16, DynamicReturn> for Vlp16DynamicPcdConverter {
    fn from_config(config: Config<Vlp16, DynamicReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<DynamicReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        let points: Vec<_> = match &mut self.context {
            DynamicReturnContext::SingleReturn(context) => {
                super::impls::convert_single_return_16_channel(context, packet)
                    .into_iter()
                    .map(DynamicReturnPoint::from)
                    .collect()
            }
            DynamicReturnContext::DualReturn(context) => {
                super::impls::convert_dual_return_16_channel(context, packet)
                    .into_iter()
                    .map(DynamicReturnPoint::from)
                    .collect()
            }
        };

        Ok(points)
    }
}

impl PointCloudConverterInterface<Vlp32, StrongestReturn> for Vlp32StrongestPcdConverter {
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

impl PointCloudConverterInterface<Vlp32, LastReturn> for Vlp32LastPcdConverter {
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

impl PointCloudConverterInterface<Vlp32, DualReturn> for Vlp32DualPcdConverter {
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

impl PointCloudConverterInterface<Vlp32, DynamicReturn> for Vlp32DynamicPcdConverter {
    fn from_config(config: Config<Vlp32, DynamicReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Result<Vec<DynamicReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        let points: Vec<_> = match &mut self.context {
            DynamicReturnContext::SingleReturn(context) => {
                super::impls::convert_single_return_32_channel(context, packet)
                    .into_iter()
                    .map(DynamicReturnPoint::from)
                    .collect()
            }
            DynamicReturnContext::DualReturn(context) => {
                super::impls::convert_dual_return_32_channel(context, packet)
                    .into_iter()
                    .map(DynamicReturnPoint::from)
                    .collect()
            }
        };

        Ok(points)
    }
}
