use super::{
    data::{DualReturnPoint, DynamicReturnPoints, SingleReturnPoint},
    impls,
};
use crate::{
    common::*,
    velodyne::{
        config::{
            Config, Dynamic_Config, LaserParameter, Vlp16_Dual_Config, Vlp16_Dynamic_Config,
            Vlp16_Last_Config, Vlp16_Strongest_Config, Vlp32_Dual_Config, Vlp32_Dynamic_Config,
            Vlp32_Last_Config, Vlp32_Strongest_Config,
        },
        marker::{
            DualReturn, DynamicModel, DynamicReturn, LastReturn, ModelMarker, ReturnTypeMarker,
            StrongestReturn, Vlp16, Vlp32,
        },
        packet::{Block, Packet, ReturnMode},
    },
};

pub use converter_impls::*;
pub use definition::*;

mod definition {
    use super::*;

    /// The trait is implemented by all variants of point cloud converters.
    pub trait PointCloudConverter<Model, ReturnType>
    where
        Model: ModelMarker,
        ReturnType: ReturnTypeMarker,
    {
        type Output;

        fn from_config(config: Config<Model, ReturnType>) -> Self;
        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>;
    }

    #[derive(Debug)]
    pub(crate) enum LastBlock {
        Single(Option<(Time, Block)>),
        Dual(Option<(Time, Block, Block)>),
    }

    impl LastBlock {
        pub fn new(return_type: DynamicReturn) -> Self {
            match return_type {
                DynamicReturn::LastReturn | DynamicReturn::StrongestReturn => Self::Single(None),
                DynamicReturn::DualReturn => Self::Dual(None),
            }
        }

        pub fn single(&mut self) -> &mut Option<(Time, Block)> {
            match self {
                Self::Single(last_block) => last_block,
                _ => unreachable!(),
            }
        }

        pub fn dual(&mut self) -> &mut Option<(Time, Block, Block)> {
            match self {
                Self::Dual(last_block) => last_block,
                _ => unreachable!(),
            }
        }
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Dynamic_PcdConverter {
        pub(crate) model: DynamicModel,
        pub(crate) return_type: DynamicReturn,
        pub(crate) lasers: Vec<LaserParameter>,
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: LastBlock,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Strongest_PcdConverter {
        pub(crate) lasers: [LaserParameter; 16],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: Option<(Time, Block)>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Last_PcdConverter {
        pub(crate) lasers: [LaserParameter; 16],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: Option<(Time, Block)>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Dual_PcdConverter {
        pub(crate) lasers: [LaserParameter; 16],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: Option<(Time, Block, Block)>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Dynamic_PcdConverter {
        pub(crate) return_type: DynamicReturn,
        pub(crate) lasers: [LaserParameter; 16],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: LastBlock,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Strongest_PcdConverter {
        pub(crate) lasers: [LaserParameter; 32],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: Option<(Time, Block)>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Last_PcdConverter {
        pub(crate) lasers: [LaserParameter; 32],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: Option<(Time, Block)>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Dual_PcdConverter {
        pub(crate) lasers: [LaserParameter; 32],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: Option<(Time, Block, Block)>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Dynamic_PcdConverter {
        pub(crate) return_type: DynamicReturn,
        pub(crate) lasers: [LaserParameter; 32],
        pub(crate) distance_resolution: Length,
        pub(crate) last_block: LastBlock,
    }
}

mod converter_impls {
    use super::*;

    impl PointCloudConverter<Vlp16, StrongestReturn> for Vlp16_Strongest_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp16_Strongest_Config) -> Self {
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                distance_resolution,
                last_block: None,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();
            ensure!(
                packet.return_mode == ReturnMode::StrongestReturn,
                "return mode does not match"
            );
            Ok(impls::convert_single_return_16_channel(
                lasers,
                distance_resolution,
                last_block,
                packet,
            ))
        }
    }

    impl PointCloudConverter<Vlp16, LastReturn> for Vlp16_Last_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp16_Last_Config) -> Self {
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                distance_resolution,
                last_block: None,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();
            ensure!(
                packet.return_mode == ReturnMode::LastReturn,
                "return mode does not match"
            );
            Ok(impls::convert_single_return_16_channel(
                lasers,
                distance_resolution,
                last_block,
                packet,
            ))
        }
    }

    impl PointCloudConverter<Vlp16, DualReturn> for Vlp16_Dual_PcdConverter {
        type Output = Vec<DualReturnPoint>;

        fn from_config(config: Vlp16_Dual_Config) -> Self {
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                distance_resolution,
                last_block: None,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();
            ensure!(
                packet.return_mode == ReturnMode::DualReturn,
                "return mode does not match"
            );
            Ok(impls::convert_dual_return_16_channel(
                lasers,
                distance_resolution,
                last_block,
                packet,
            ))
        }
    }

    impl PointCloudConverter<Vlp16, DynamicReturn> for Vlp16_Dynamic_PcdConverter {
        type Output = DynamicReturnPoints;

        fn from_config(config: Vlp16_Dynamic_Config) -> Self {
            let Config {
                lasers,
                return_type,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                return_type,
                distance_resolution,
                last_block: LastBlock::new(return_type),
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                return_type,
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();

            let points: DynamicReturnPoints = match return_type {
                DynamicReturn::LastReturn | DynamicReturn::StrongestReturn => {
                    impls::convert_single_return_16_channel(
                        lasers,
                        distance_resolution,
                        last_block.single(),
                        packet,
                    )
                    .into()
                }
                DynamicReturn::DualReturn => impls::convert_dual_return_16_channel(
                    lasers,
                    distance_resolution,
                    last_block.dual(),
                    packet,
                )
                .into(),
            };

            Ok(points)
        }
    }

    impl PointCloudConverter<Vlp32, StrongestReturn> for Vlp32_Strongest_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp32_Strongest_Config) -> Self {
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                distance_resolution,
                last_block: None,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();
            ensure!(
                packet.return_mode == ReturnMode::StrongestReturn,
                "return mode does not match"
            );
            Ok(impls::convert_single_return_32_channel(
                lasers,
                distance_resolution,
                last_block,
                packet,
            ))
        }
    }

    impl PointCloudConverter<Vlp32, LastReturn> for Vlp32_Last_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp32_Last_Config) -> Self {
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                distance_resolution,
                last_block: None,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();
            ensure!(
                packet.return_mode == ReturnMode::LastReturn,
                "return mode does not match"
            );
            Ok(impls::convert_single_return_32_channel(
                lasers,
                distance_resolution,
                last_block,
                packet,
            ))
        }
    }

    impl PointCloudConverter<Vlp32, DualReturn> for Vlp32_Dual_PcdConverter {
        type Output = Vec<DualReturnPoint>;

        fn from_config(config: Vlp32_Dual_Config) -> Self {
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                distance_resolution,
                last_block: None,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();
            ensure!(
                packet.return_mode == ReturnMode::DualReturn,
                "return mode does not match"
            );
            Ok(impls::convert_dual_return_32_channel(
                lasers,
                distance_resolution,
                last_block,
                packet,
            ))
        }
    }

    impl PointCloudConverter<Vlp32, DynamicReturn> for Vlp32_Dynamic_PcdConverter {
        type Output = DynamicReturnPoints;

        fn from_config(config: Vlp32_Dynamic_Config) -> Self {
            let Config {
                lasers,
                return_type,
                distance_resolution,
                ..
            } = config;

            Self {
                lasers,
                return_type,
                distance_resolution,
                last_block: LastBlock::new(return_type),
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                return_type,
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();

            let points: DynamicReturnPoints = match return_type {
                DynamicReturn::LastReturn | DynamicReturn::StrongestReturn => {
                    impls::convert_single_return_32_channel(
                        lasers,
                        distance_resolution,
                        last_block.single(),
                        packet,
                    )
                    .into()
                }
                DynamicReturn::DualReturn => impls::convert_dual_return_32_channel(
                    lasers,
                    distance_resolution,
                    last_block.dual(),
                    packet,
                )
                .into(),
            };

            Ok(points)
        }
    }

    impl PointCloudConverter<DynamicModel, DynamicReturn> for Dynamic_PcdConverter {
        type Output = DynamicReturnPoints;

        fn from_config(config: Dynamic_Config) -> Self {
            let Config {
                model,
                lasers,
                return_type,
                distance_resolution,
                ..
            } = config;

            Self {
                model,
                lasers,
                return_type,
                distance_resolution,
                last_block: LastBlock::new(return_type),
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                model,
                return_type,
                ref lasers,
                distance_resolution,
                ref mut last_block,
            } = *self;

            let packet = packet.as_ref();

            let points: DynamicReturnPoints = match (model, return_type) {
                (DynamicModel::Vlp16, DynamicReturn::LastReturn)
                | (DynamicModel::Vlp16, DynamicReturn::StrongestReturn) => {
                    let lasers: &[_; 16] = lasers.as_slice().try_into().unwrap();
                    impls::convert_single_return_16_channel(
                        lasers,
                        distance_resolution,
                        last_block.single(),
                        packet,
                    )
                    .into()
                }
                (DynamicModel::Vlp16, DynamicReturn::DualReturn) => {
                    let lasers: &[_; 16] = lasers.as_slice().try_into().unwrap();
                    impls::convert_dual_return_16_channel(
                        lasers,
                        distance_resolution,
                        last_block.dual(),
                        packet,
                    )
                    .into()
                }
                (DynamicModel::Vlp32, DynamicReturn::LastReturn)
                | (DynamicModel::Vlp32, DynamicReturn::StrongestReturn) => {
                    let lasers: &[_; 32] = lasers.as_slice().try_into().unwrap();
                    impls::convert_single_return_32_channel(
                        lasers,
                        distance_resolution,
                        last_block.single(),
                        packet,
                    )
                    .into()
                }
                (DynamicModel::Vlp32, DynamicReturn::DualReturn) => {
                    let lasers: &[_; 32] = lasers.as_slice().try_into().unwrap();
                    impls::convert_dual_return_32_channel(
                        lasers,
                        distance_resolution,
                        last_block.dual(),
                        packet,
                    )
                    .into()
                }
            };

            Ok(points)
        }
    }
}
