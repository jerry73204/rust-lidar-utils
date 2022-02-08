use super::impls;
use crate::{
    common::*,
    velodyne::{
        config::{
            DConfig, DualReturn, LastReturn, ModelMarker, ReturnModeMarker, SConfig,
            StrongestReturn,
        },
        packet::DataPacket,
        pcd_converter::{DPcdConverter, DualPcdConverter, SinglePcdConverter},
        point::{DPoints, DualPoint, DynamicReturnFrame, SinglePoint},
        PcdConverter, ReturnMode, PUCK_HIRES, PUCK_LITE, VLP_16, VLP_32C,
    },
};

pub trait FrameConverter {
    type Output;
    type State;

    /// Converts a packet into a collection of frames of points.
    fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
    where
        P: Borrow<DataPacket>;

    fn pop_remaining(&mut self) -> Option<Self::State>;
}

pub use s_type::*;
mod s_type {
    #![allow(non_camel_case_types)]

    use super::*;

    pub struct SingleFrameConverter<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Self: FrameConverter,
        SinglePcdConverter<Model, Return>: PcdConverter,
    {
        pub(crate) pcd_converter: SinglePcdConverter<Model, Return>,
        pub(crate) state: Vec<SinglePoint>,
    }

    pub struct DualFrameConverter<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Self: FrameConverter,
        DualPcdConverter<Model, Return>: PcdConverter,
    {
        pub(crate) pcd_converter: DualPcdConverter<Model, Return>,
        pub(crate) state: Vec<DualPoint>,
    }

    // aliases

    pub type Vlp16_Strongest_FrameConverter = SingleFrameConverter<VLP_16, StrongestReturn>;
    pub type Vlp16_Last_FrameConverter = SingleFrameConverter<VLP_16, LastReturn>;
    pub type Vlp16_Dual_FrameConverter = DualFrameConverter<VLP_16, DualReturn>;

    pub type Vlp32c_Strongest_FrameConverter = SingleFrameConverter<VLP_32C, StrongestReturn>;
    pub type Vlp32c_Last_FrameConverter = SingleFrameConverter<VLP_32C, LastReturn>;
    pub type Vlp32c_Dual_FrameConverter = DualFrameConverter<VLP_32C, DualReturn>;

    pub type PuckLite_Strongest_FrameConverter = SingleFrameConverter<PUCK_LITE, StrongestReturn>;
    pub type PuckLite_Last_FrameConverter = SingleFrameConverter<PUCK_LITE, LastReturn>;
    pub type PuckLite_Dual_FrameConverter = DualFrameConverter<PUCK_LITE, DualReturn>;

    pub type PuckHires_Strongest_FrameConverter = SingleFrameConverter<PUCK_HIRES, StrongestReturn>;
    pub type PuckHires_Last_FrameConverter = SingleFrameConverter<PUCK_HIRES, LastReturn>;
    pub type PuckHires_Dual_FrameConverter = DualFrameConverter<PUCK_HIRES, DualReturn>;

    // impls

    impl<Model, Return> SingleFrameConverter<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Self: FrameConverter,
        SinglePcdConverter<Model, Return>: PcdConverter,
    {
        pub fn from_config(config: SConfig<Model, Return>) -> Self {
            Self {
                pcd_converter: SinglePcdConverter::from_config(config),
                state: vec![],
            }
        }
    }

    impl<Model> DualFrameConverter<Model, DualReturn>
    where
        Model: ModelMarker,
        Self: FrameConverter,
        DualPcdConverter<Model, DualReturn>: PcdConverter,
    {
        pub fn from_config(config: SConfig<Model, DualReturn>) -> Self {
            Self {
                pcd_converter: DualPcdConverter::from_config(config),
                state: vec![],
            }
        }
    }

    impl FrameConverter for Vlp16_Last_FrameConverter {
        type Output = PcdFrame<SinglePoint>;
        type State = Vec<SinglePoint>;

        fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                state,
            } = self;

            impls::convert_single_return(pcd_converter, state, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::State> {
            if self.state.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.state))
            }
        }
    }

    impl FrameConverter for Vlp16_Strongest_FrameConverter {
        type Output = PcdFrame<SinglePoint>;
        type State = Vec<SinglePoint>;

        fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                state,
            } = self;

            impls::convert_single_return(pcd_converter, state, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::State> {
            if self.state.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.state))
            }
        }
    }

    impl FrameConverter for Vlp16_Dual_FrameConverter {
        type Output = PcdFrame<DualPoint>;
        type State = Vec<DualPoint>;

        fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                state,
            } = self;

            impls::convert_dual_return(pcd_converter, state, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::State> {
            if self.state.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.state))
            }
        }
    }

    impl FrameConverter for Vlp32c_Last_FrameConverter {
        type Output = PcdFrame<SinglePoint>;
        type State = Vec<SinglePoint>;

        fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                state,
            } = self;

            impls::convert_single_return(pcd_converter, state, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::State> {
            if self.state.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.state))
            }
        }
    }

    impl FrameConverter for Vlp32c_Strongest_FrameConverter {
        type Output = PcdFrame<SinglePoint>;
        type State = Vec<SinglePoint>;

        fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                state,
            } = self;

            impls::convert_single_return(pcd_converter, state, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::State> {
            if self.state.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.state))
            }
        }
    }

    impl FrameConverter for Vlp32c_Dual_FrameConverter {
        type Output = PcdFrame<DualPoint>;
        type State = Vec<DualPoint>;

        fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                state,
            } = self;

            impls::convert_dual_return(pcd_converter, state, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::State> {
            if self.state.is_empty() {
                None
            } else {
                Some(mem::take(&mut self.state))
            }
        }
    }
}

pub use d_type::*;

mod d_type {
    use super::*;

    #[derive(Debug)]
    pub struct DFrameConverter
    where
        Self: FrameConverter,
    {
        pub(crate) pcd_converter: DPcdConverter,
        pub(crate) state: <Self as FrameConverter>::State,
    }

    impl DFrameConverter {
        pub fn from_config(config: DConfig) -> Self {
            let state = match config.return_mode {
                ReturnMode::Strongest | ReturnMode::Last => DPoints::Single(vec![]),
                ReturnMode::Dual => DPoints::Dual(vec![]),
            };

            Self {
                pcd_converter: DPcdConverter::from_config(config),
                state,
            }
        }
    }

    impl FrameConverter for DFrameConverter {
        type Output = DynamicReturnFrame;
        type State = DPoints;

        fn convert<P>(&mut self, packet: P) -> Option<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let Self {
                pcd_converter,
                state,
            } = self;

            impls::convert_dynamic_return(pcd_converter, state, packet.borrow())
        }

        fn pop_remaining(&mut self) -> Option<Self::State> {
            match &mut self.state {
                DPoints::Single(points) => {
                    let points = mem::take(points);

                    (!points.is_empty()).then(|| DPoints::Single(points))
                }
                DPoints::Dual(points) => {
                    let points = mem::take(points);
                    (!points.is_empty()).then(|| DPoints::Dual(points))
                }
            }
        }
    }
}

pub use definitions::*;
mod definitions {
    use super::*;

    #[derive(Debug, Clone)]
    pub struct PcdFrame<P>
    where
        P: Clone,
    {
        pub height: usize,
        pub width: usize,
        pub data: Vec<P>,
    }

    impl<P> PcdFrame<P>
    where
        P: Clone,
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
