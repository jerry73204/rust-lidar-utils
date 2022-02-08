use super::impls::{self, DualState, DynState, SingleState};
use crate::{
    common::*,
    velodyne::{
        config::{
            Config, DConfig, DualReturn, LastReturn, ModelMarker, ReturnModeMarker, SConfig,
            StrongestReturn, PUCK_HIRES, PUCK_LITE, VLP_16,
        },
        packet::{DataPacket, ReturnMode},
        point::{DualReturnPoint, DynamicReturnPoints, SingleReturnPoint},
        ProductID, VLP_32C,
    },
};

pub trait PcdConverter {
    type Output;

    /// Converts a packet into a collection points.
    fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
    where
        P: Borrow<DataPacket>;
}

pub use s_type::*;
mod s_type {
    #![allow(non_camel_case_types)]

    use super::*;

    pub struct SinglePcdConverter<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Self: PcdConverter,
    {
        config: SConfig<Model, Return>,
        last_block: Option<SingleState>,
    }

    pub struct DualPcdConverter<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Self: PcdConverter,
    {
        config: SConfig<Model, Return>,
        last_block: Option<DualState>,
    }

    // aliases

    pub type Vlp16_Strongest_PcdConverter = SinglePcdConverter<VLP_16, StrongestReturn>;
    pub type Vlp16_Last_PcdConverter = SinglePcdConverter<VLP_16, LastReturn>;
    pub type Vlp16_Dual_PcdConverter = DualPcdConverter<VLP_16, DualReturn>;

    pub type Vlp32c_Strongest_PcdConverter = SinglePcdConverter<VLP_32C, StrongestReturn>;
    pub type Vlp32c_Last_PcdConverter = SinglePcdConverter<VLP_32C, LastReturn>;
    pub type Vlp32c_Dual_PcdConverter = DualPcdConverter<VLP_32C, DualReturn>;

    pub type PuckLite_Strongest_PcdConverter = SinglePcdConverter<PUCK_LITE, StrongestReturn>;
    pub type PuckLite_Last_PcdConverter = SinglePcdConverter<PUCK_LITE, LastReturn>;
    pub type PuckLite_Dual_PcdConverter = DualPcdConverter<PUCK_LITE, DualReturn>;

    pub type PuckHires_Strongest_PcdConverter = SinglePcdConverter<PUCK_HIRES, StrongestReturn>;
    pub type PuckHires_Last_PcdConverter = SinglePcdConverter<PUCK_HIRES, LastReturn>;
    pub type PuckHires_Dual_PcdConverter = DualPcdConverter<PUCK_HIRES, DualReturn>;

    impl<Model, Return> SinglePcdConverter<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Self: PcdConverter,
    {
        /// Construct a point cloud converter from a config type.
        pub fn from_config(config: SConfig<Model, Return>) -> Self {
            Self {
                config,
                last_block: None,
            }
        }

        pub fn config(&self) -> &SConfig<Model, Return> {
            &self.config
        }
    }

    impl<Model> DualPcdConverter<Model, DualReturn>
    where
        Model: ModelMarker,
        Self: PcdConverter,
    {
        /// Construct a point cloud converter from a config type.
        pub fn from_config(config: SConfig<Model, DualReturn>) -> Self {
            Self {
                config,
                last_block: None,
            }
        }

        pub fn config(&self) -> &SConfig<Model, DualReturn> {
            &self.config
        }
    }

    // vlp 16

    impl PcdConverter for Vlp16_Strongest_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Strongest,
                "return mode does not match"
            );

            let output = impls::convert_single_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for Vlp16_Last_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Last,
                "return mode does not match"
            );

            let output = impls::convert_single_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for Vlp16_Dual_PcdConverter {
        type Output = Vec<DualReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Dual,
                "return mode does not match"
            );

            let output = impls::convert_dual_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    // vlp 32c

    impl PcdConverter for Vlp32c_Strongest_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Strongest,
                "return mode does not match"
            );

            let output = impls::convert_single_return_32_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for Vlp32c_Last_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Last,
                "return mode does not match"
            );

            let output = impls::convert_single_return_32_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for Vlp32c_Dual_PcdConverter {
        type Output = Vec<DualReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Dual,
                "return mode does not match"
            );

            let output = impls::convert_dual_return_32_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    // puck lite

    impl PcdConverter for PuckLite_Strongest_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Strongest,
                "return mode does not match"
            );

            let output = impls::convert_single_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for PuckLite_Last_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Last,
                "return mode does not match"
            );

            let output = impls::convert_single_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for PuckLite_Dual_PcdConverter {
        type Output = Vec<DualReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Dual,
                "return mode does not match"
            );

            let output = impls::convert_dual_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    // puck lite

    impl PcdConverter for PuckHires_Strongest_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Strongest,
                "return mode does not match"
            );

            let output = impls::convert_single_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for PuckHires_Last_PcdConverter {
        type Output = Vec<SingleReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Last,
                "return mode does not match"
            );

            let output = impls::convert_single_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }

    impl PcdConverter for PuckHires_Dual_PcdConverter {
        type Output = Vec<DualReturnPoint>;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            let packet = packet.borrow();
            ensure!(
                packet.return_mode == ReturnMode::Dual,
                "return mode does not match"
            );

            let output = impls::convert_dual_return_16_channel(
                &self.config.lasers,
                self.config.distance_resolution(),
                &mut self.last_block,
                packet,
            );

            Ok(output)
        }
    }
}

pub use d_type::*;
mod d_type {
    use super::*;

    #[derive(Debug)]
    pub struct DPcdConverter
    where
        Self: PcdConverter,
    {
        pub(crate) config: DConfig,
        pub(crate) last_block: DynState,
    }

    impl DPcdConverter
    where
        Self: PcdConverter,
    {
        pub fn from_config(config: DConfig) -> Self {
            use ReturnMode as R;

            let last_block = match config.return_mode {
                R::Strongest | R::Last => DynState::Single(None),
                R::Dual => DynState::Dual(None),
            };

            Self { config, last_block }
        }

        pub fn config(&self) -> &DConfig {
            &self.config
        }
    }

    impl PcdConverter for DPcdConverter {
        type Output = DynamicReturnPoints;

        fn convert<P>(&mut self, packet: P) -> Result<Self::Output>
        where
            P: Borrow<DataPacket>,
        {
            use ProductID as M;
            use ReturnMode as R;

            let packet = packet.borrow();

            let output = match (self.config.product_id, self.config.return_mode) {
                (M::VLP16 | M::PuckLite | M::PuckHiRes, R::Strongest | R::Last) => {
                    impls::convert_single_return_16_channel(
                        self.config.lasers.as_slice().try_into().unwrap(),
                        self.config.distance_resolution(),
                        self.last_block.assume_single(),
                        packet,
                    )
                    .into()
                }
                (M::VLP16 | M::PuckLite | M::PuckHiRes, R::Dual) => {
                    impls::convert_dual_return_16_channel(
                        self.config.lasers.as_slice().try_into().unwrap(),
                        self.config.distance_resolution(),
                        self.last_block.assume_dual(),
                        packet,
                    )
                    .into()
                }
                (M::VLP32C, R::Strongest | R::Last) => impls::convert_single_return_32_channel(
                    self.config.lasers.as_slice().try_into().unwrap(),
                    self.config.distance_resolution(),
                    self.last_block.assume_single(),
                    packet,
                )
                .into(),
                (M::VLP32C, R::Dual) => impls::convert_dual_return_32_channel(
                    self.config.lasers.as_slice().try_into().unwrap(),
                    self.config.distance_resolution(),
                    self.last_block.assume_dual(),
                    packet,
                )
                .into(),
                (product_id, _return_mode) => {
                    bail!("unsupported model {:?}", product_id);
                }
            };

            Ok(output)
        }
    }
}
