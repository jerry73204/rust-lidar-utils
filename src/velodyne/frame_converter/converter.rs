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
        packet::DataPacket,
        pcd_converter::{
            Dynamic_PcdConverter, PointCloudConverter, Vlp16_Dual_PcdConverter,
            Vlp16_Dynamic_PcdConverter, Vlp16_Last_PcdConverter, Vlp16_Strongest_PcdConverter,
            Vlp32_Dual_PcdConverter, Vlp32_Dynamic_PcdConverter, Vlp32_Last_PcdConverter,
            Vlp32_Strongest_PcdConverter,
        },
        point::{DualReturnPoint, DynamicReturnFrame, DynamicReturnPoints, SingleReturnPoint},
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
        type Frame;
        type Remain;

        /// Construct a frame converter from a config type.
        fn from_config(config: Config<Model, ReturnType>) -> Self;

        /// Converts a packet into a collection of frames of points.
        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>;

        fn pop_remaining(&mut self) -> Option<Self::Remain>;
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

        pub fn take(&mut self) -> DynamicReturnPoints {
            let empty = self.0.empty_like();
            mem::replace(&mut self.0, empty)
        }
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

    #[derive(Debug, Clone)]
    pub struct PcdFrame<P>
    where
        P: Copy + Clone,
    {
        pub height: usize,
        pub width: usize,
        pub data: Vec<P>,
    }

    impl<P> PcdFrame<P>
    where
        P: Copy,
    {
        pub fn point_at(&self, rol_idx: usize, col_idx: usize) -> Result<&P> {
            Ok(&self.data[col_idx * self.height + rol_idx])
        }

        pub(crate) fn empty() -> Self {
            Self {
                height: 0,
                width: 0,
                data: vec![],
            }
        }
    }
}

mod converter_impls {
    use super::*;

    impl FrameConverter<DynamicModel, DynamicReturn> for Dynamic_FrameConverter {
        type Frame = DynamicReturnFrame;
        type Remain = DynamicReturnPoints;

        fn from_config(config: Dynamic_Config) -> Self {
            let remaining_points = RemainingPoints::new(config.return_type);
            Self {
                pcd_converter: Dynamic_PcdConverter::from_config(config),
                remaining_points,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dynamic_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            let remaining = self.remaining_points.take();
            if remaining.is_empty() {
                None
            } else {
                Some(remaining)
            }
        }
    }

    impl FrameConverter<Vlp16, LastReturn> for Vlp16_Last_FrameConverter {
        type Frame = PcdFrame<SingleReturnPoint>;
        type Remain = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp16_Last_Config) -> Self {
            Self {
                pcd_converter: Vlp16_Last_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            if self.remaining_points.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.remaining_points))
            }
        }
    }

    impl FrameConverter<Vlp16, StrongestReturn> for Vlp16_Strongest_FrameConverter {
        type Frame = PcdFrame<SingleReturnPoint>;
        type Remain = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp16_Strongest_Config) -> Self {
            Self {
                pcd_converter: Vlp16_Strongest_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            if self.remaining_points.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.remaining_points))
            }
        }
    }

    impl FrameConverter<Vlp16, DualReturn> for Vlp16_Dual_FrameConverter {
        type Frame = PcdFrame<DualReturnPoint>;
        type Remain = Vec<DualReturnPoint>;

        fn from_config(config: Vlp16_Dual_Config) -> Self {
            Self {
                pcd_converter: Vlp16_Dual_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dual_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            if self.remaining_points.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.remaining_points))
            }
        }
    }

    impl FrameConverter<Vlp16, DynamicReturn> for Vlp16_Dynamic_FrameConverter {
        type Frame = DynamicReturnFrame;
        type Remain = DynamicReturnPoints;

        fn from_config(config: Vlp16_Dynamic_Config) -> Self {
            let remaining_points = RemainingPoints::new(config.return_type);
            Self {
                pcd_converter: Vlp16_Dynamic_PcdConverter::from_config(config),
                remaining_points,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dynamic_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            let remaining = self.remaining_points.take();
            if remaining.is_empty() {
                None
            } else {
                Some(remaining)
            }
        }
    }

    impl FrameConverter<Vlp32, LastReturn> for Vlp32_Last_FrameConverter {
        type Frame = PcdFrame<SingleReturnPoint>;
        type Remain = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp32_Last_Config) -> Self {
            Self {
                pcd_converter: Vlp32_Last_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            if self.remaining_points.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.remaining_points))
            }
        }
    }

    impl FrameConverter<Vlp32, StrongestReturn> for Vlp32_Strongest_FrameConverter {
        type Frame = PcdFrame<SingleReturnPoint>;
        type Remain = Vec<SingleReturnPoint>;

        fn from_config(config: Vlp32_Strongest_Config) -> Self {
            Self {
                pcd_converter: Vlp32_Strongest_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_single_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            if self.remaining_points.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.remaining_points))
            }
        }
    }

    impl FrameConverter<Vlp32, DualReturn> for Vlp32_Dual_FrameConverter {
        type Frame = PcdFrame<DualReturnPoint>;
        type Remain = Vec<DualReturnPoint>;

        fn from_config(config: Vlp32_Dual_Config) -> Self {
            Self {
                pcd_converter: Vlp32_Dual_PcdConverter::from_config(config),
                remaining_points: vec![],
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dual_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            if self.remaining_points.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.remaining_points))
            }
        }
    }

    impl FrameConverter<Vlp32, DynamicReturn> for Vlp32_Dynamic_FrameConverter {
        type Frame = DynamicReturnFrame;
        type Remain = DynamicReturnPoints;

        fn from_config(config: Vlp32_Dynamic_Config) -> Self {
            let remaining_points = RemainingPoints::new(config.return_type);
            Self {
                pcd_converter: Vlp32_Dynamic_PcdConverter::from_config(config),
                remaining_points,
            }
        }

        fn convert<P>(&mut self, packet: P) -> Option<Self::Frame>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                remaining_points,
            } = self;

            impls::convert_dynamic_return(pcd_converter, remaining_points, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::Remain> {
            let remaining = self.remaining_points.take();
            if remaining.is_empty() {
                None
            } else {
                Some(remaining)
            }
        }
    }
}
