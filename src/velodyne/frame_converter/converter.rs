use super::impls;
use crate::{
    common::*,
    velodyne::{
        config::{
            Config, Dynamic_Config, Vlp16_Dual_Config, Vlp16_Dynamic_Config, Vlp16_Last_Config,
            Vlp16_Strongest_Config, Vlp32_Dual_Config, Vlp32_Dynamic_Config, Vlp32_Last_Config,
            Vlp32_Strongest_Config,
        },
        marker::{
            DualReturn, DynamicModel, DynamicReturn, LastReturn, ModelMarker, ReturnTypeMarker,
            StrongestReturn, Vlp16, Vlp32,
        },
        packet::Packet,
        pcd_converter::{
            DualReturnPoint, DynamicReturnPoints, Dynamic_PcdConverter, PointCloudConverter,
            SingleReturnPoint, Vlp16_Dual_PcdConverter, Vlp16_Dynamic_PcdConverter,
            Vlp16_Last_PcdConverter, Vlp16_Strongest_PcdConverter, Vlp32_Dual_PcdConverter,
            Vlp32_Dynamic_PcdConverter, Vlp32_Last_PcdConverter, Vlp32_Strongest_PcdConverter,
        },
    },
};

pub use converter_impls::*;
pub use definitions::*;

mod definitions {
    use super::*;

    /// The trait is implemented by all variants of frame converters.
    pub trait FrameConverter<Model, ReturnType>
    where
        Model: ModelMarker,
        ReturnType: ReturnTypeMarker,
    {
        type Output;

        /// Construct a frame converter from a config type.
        fn from_config(config: Config<Model, ReturnType>) -> Self;

        /// Converts a packet into a collection of frames of points.
        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>;
    }

    #[derive(Debug)]
    pub(crate) struct RemainingPoints(pub(crate) DynamicReturnPoints);

    impl RemainingPoints {
        pub fn new(return_type: DynamicReturn) -> Self {
            Self(match return_type {
                DynamicReturn::LastReturn | DynamicReturn::StrongestReturn => {
                    DynamicReturnPoints::Single(vec![])
                }
                DynamicReturn::DualReturn => DynamicReturnPoints::Dual(vec![]),
            })
        }

        // pub fn single(&mut self) -> &mut Vec<SingleReturnPoint> {
        //     match &mut self.0 {
        //         DynamicReturnPoints::Single(points) => points,
        //         _ => unreachable!(),
        //     }
        // }

        // pub fn dual(&mut self) -> &mut Vec<DualReturnPoint> {
        //     match &mut self.0 {
        //         DynamicReturnPoints::Dual(points) => points,
        //         _ => unreachable!(),
        //     }
        // }

        // pub fn dynamic(&mut self) -> &mut DynamicReturnPoints {
        //     &mut self.0
        // }
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Dynamic_FrameConverter {
        pub(crate) pcd_converter: Dynamic_PcdConverter,
        pub(crate) remaining_points: RemainingPoints,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Last_FrameConverter {
        pub(crate) pcd_converter: Vlp16_Last_PcdConverter,
        pub(crate) remaining_points: Vec<SingleReturnPoint>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Strongest_FrameConverter {
        pub(crate) pcd_converter: Vlp16_Strongest_PcdConverter,
        pub(crate) remaining_points: Vec<SingleReturnPoint>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Dual_FrameConverter {
        pub(crate) pcd_converter: Vlp16_Dual_PcdConverter,
        pub(crate) remaining_points: Vec<DualReturnPoint>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp16_Dynamic_FrameConverter {
        pub(crate) pcd_converter: Vlp16_Dynamic_PcdConverter,
        pub(crate) remaining_points: RemainingPoints,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Last_FrameConverter {
        pub(crate) pcd_converter: Vlp32_Last_PcdConverter,
        pub(crate) remaining_points: Vec<SingleReturnPoint>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Strongest_FrameConverter {
        pub(crate) pcd_converter: Vlp32_Strongest_PcdConverter,
        pub(crate) remaining_points: Vec<SingleReturnPoint>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Dual_FrameConverter {
        pub(crate) pcd_converter: Vlp32_Dual_PcdConverter,
        pub(crate) remaining_points: Vec<DualReturnPoint>,
    }

    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub struct Vlp32_Dynamic_FrameConverter {
        pub(crate) pcd_converter: Vlp32_Dynamic_PcdConverter,
        pub(crate) remaining_points: RemainingPoints,
    }
}

mod converter_impls {
    use super::*;

    impl FrameConverter<DynamicModel, DynamicReturn> for Dynamic_FrameConverter {
        type Output = Vec<DynamicReturnPoints>;

        fn from_config(config: Dynamic_Config) -> Self {
            let remaining_points = RemainingPoints::new(config.return_type);
            Self {
                pcd_converter: Dynamic_PcdConverter::from_config(config),
                remaining_points,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dynamic_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp16, LastReturn> for Vlp16_Last_FrameConverter {
        type Output = Vec<Vec<SingleReturnPoint>>;

        fn from_config(config: Vlp16_Last_Config) -> Self {
            Self {
                pcd_converter: Vlp16_Last_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp16, StrongestReturn> for Vlp16_Strongest_FrameConverter {
        type Output = Vec<Vec<SingleReturnPoint>>;

        fn from_config(config: Vlp16_Strongest_Config) -> Self {
            Self {
                pcd_converter: Vlp16_Strongest_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp16, DualReturn> for Vlp16_Dual_FrameConverter {
        type Output = Vec<Vec<DualReturnPoint>>;

        fn from_config(config: Vlp16_Dual_Config) -> Self {
            Self {
                pcd_converter: Vlp16_Dual_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dual_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp16, DynamicReturn> for Vlp16_Dynamic_FrameConverter {
        type Output = Vec<DynamicReturnPoints>;

        fn from_config(config: Vlp16_Dynamic_Config) -> Self {
            let remaining_points = RemainingPoints::new(config.return_type);
            Self {
                pcd_converter: Vlp16_Dynamic_PcdConverter::from_config(config),
                remaining_points,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dynamic_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp32, LastReturn> for Vlp32_Last_FrameConverter {
        type Output = Vec<Vec<SingleReturnPoint>>;

        fn from_config(config: Vlp32_Last_Config) -> Self {
            Self {
                pcd_converter: Vlp32_Last_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp32, StrongestReturn> for Vlp32_Strongest_FrameConverter {
        type Output = Vec<Vec<SingleReturnPoint>>;

        fn from_config(config: Vlp32_Strongest_Config) -> Self {
            Self {
                pcd_converter: Vlp32_Strongest_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp32, DualReturn> for Vlp32_Dual_FrameConverter {
        type Output = Vec<Vec<DualReturnPoint>>;

        fn from_config(config: Vlp32_Dual_Config) -> Self {
            Self {
                pcd_converter: Vlp32_Dual_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dual_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }

    impl FrameConverter<Vlp32, DynamicReturn> for Vlp32_Dynamic_FrameConverter {
        type Output = Vec<DynamicReturnPoints>;

        fn from_config(config: Vlp32_Dynamic_Config) -> Self {
            let remaining_points = RemainingPoints::new(config.return_type);
            Self {
                pcd_converter: Vlp32_Dynamic_PcdConverter::from_config(config),
                remaining_points,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: AsRef<Packet>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dynamic_return(pcd_converter, remaining_points, packet.as_ref())
        }
    }
}
