//! Defines a set of Velodyne LiDAR configurations.

use super::{
    consts,
    frame_converter::{DFrameConverter, DualFrameConverter, FrameConverter, SingleFrameConverter},
    packet::{ProductID, ReturnMode},
    pcd_converter::{DPcdConverter, DualPcdConverter, PcdConverter, SinglePcdConverter},
};
use crate::common::*;

pub trait Config {
    fn lasers(&self) -> &[LaserParameter];
    fn return_mode(&self) -> ReturnMode;
    fn product_id(&self) -> ProductID;
    fn distance_resolution(&self) -> Length;
}

pub use sconfig::*;
mod sconfig {
    #![allow(non_camel_case_types)]
    use super::*;

    // type

    /// Config type for Velodyne LiDARs.
    #[derive(Debug, Clone)]
    pub struct SConfig<Model, ReturnMode>
    where
        Model: ModelMarker,
        ReturnMode: ReturnModeMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        pub(crate) lasers: Model::ParamArray,
        _phantom: PhantomData<ReturnMode>,
    }

    // aliases

    pub type Vlp16_Strongest_Config = SConfig<VLP_16, StrongestReturn>;
    pub type Vlp16_Last_Config = SConfig<VLP_16, LastReturn>;
    pub type Vlp16_Dual_Config = SConfig<VLP_16, DualReturn>;
    pub type Vlp32c_Strongest_Config = SConfig<VLP_32C, StrongestReturn>;
    pub type Vlp32c_Last_Config = SConfig<VLP_32C, LastReturn>;
    pub type Vlp32c_Dual_Config = SConfig<VLP_32C, DualReturn>;
    pub type PuckLite_Strongest_Config = SConfig<PUCK_LITE, StrongestReturn>;
    pub type PuckLite_Last_Config = SConfig<PUCK_LITE, LastReturn>;
    pub type PuckLite_Dual_Config = SConfig<PUCK_LITE, DualReturn>;
    pub type PuckHires_Strongest_Config = SConfig<PUCK_HIRES, StrongestReturn>;
    pub type PuckHires_Last_Config = SConfig<PUCK_HIRES, LastReturn>;
    pub type PuckHires_Dual_Config = SConfig<PUCK_HIRES, DualReturn>;

    // impls

    impl<Model, Return> SConfig<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        pub fn new() -> Self {
            Self {
                lasers: Model::lasers(),
                _phantom: PhantomData,
            }
        }

        pub fn to_dyn(&self) -> DConfig {
            self.into()
        }

        pub fn into_dyn(self) -> DConfig {
            self.into()
        }
    }

    impl<Model> SConfig<Model, StrongestReturn>
    where
        Model: ModelMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        pub fn build_pcd_converter(self) -> SinglePcdConverter<Model, StrongestReturn>
        where
            SinglePcdConverter<Model, StrongestReturn>: PcdConverter,
        {
            SinglePcdConverter::from_config(self)
        }

        pub fn build_frame_converter(self) -> SingleFrameConverter<Model, StrongestReturn>
        where
            SingleFrameConverter<Model, StrongestReturn>: FrameConverter,
            SinglePcdConverter<Model, StrongestReturn>: PcdConverter,
        {
            SingleFrameConverter::from_config(self)
        }
    }

    impl<Model> SConfig<Model, LastReturn>
    where
        Model: ModelMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        pub fn build_pcd_converter(self) -> SinglePcdConverter<Model, LastReturn>
        where
            SinglePcdConverter<Model, LastReturn>: PcdConverter,
        {
            SinglePcdConverter::from_config(self)
        }

        pub fn build_frame_converter(self) -> SingleFrameConverter<Model, LastReturn>
        where
            SingleFrameConverter<Model, LastReturn>: FrameConverter,
            SinglePcdConverter<Model, LastReturn>: PcdConverter,
        {
            SingleFrameConverter::from_config(self)
        }
    }

    impl<Model> SConfig<Model, DualReturn>
    where
        Model: ModelMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        pub fn build_pcd_converter(self) -> DualPcdConverter<Model, DualReturn>
        where
            DualPcdConverter<Model, DualReturn>: PcdConverter,
        {
            DualPcdConverter::from_config(self)
        }

        pub fn build_frame_converter(self) -> DualFrameConverter<Model, DualReturn>
        where
            DualFrameConverter<Model, DualReturn>: FrameConverter,
            DualPcdConverter<Model, DualReturn>: PcdConverter,
        {
            DualFrameConverter::from_config(self)
        }
    }

    impl<Model, Return> Default for SConfig<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        fn default() -> Self {
            Self::new()
        }
    }

    impl<Model, Return> Config for SConfig<Model, Return>
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        fn lasers(&self) -> &[LaserParameter] {
            self.lasers.as_ref()
        }

        fn return_mode(&self) -> ReturnMode {
            Return::return_mode()
        }

        fn product_id(&self) -> ProductID {
            Model::product_id()
        }

        fn distance_resolution(&self) -> Length {
            Model::distance_resolution()
        }
    }
}

pub use dconfig::*;
mod dconfig {
    use super::*;

    // type

    /// Config type for Velodyne LiDARs.
    #[derive(Debug, Clone)]
    pub struct DConfig {
        pub lasers: Vec<LaserParameter>,
        pub return_mode: ReturnMode,
        pub product_id: ProductID,
        pub distance_resolution: Length,
    }

    // impls

    impl Config for DConfig {
        fn lasers(&self) -> &[LaserParameter] {
            &self.lasers
        }

        fn return_mode(&self) -> ReturnMode {
            self.return_mode
        }

        fn product_id(&self) -> ProductID {
            self.product_id
        }

        fn distance_resolution(&self) -> Length {
            self.distance_resolution
        }
    }

    impl DConfig {
        pub fn build_pcd_converter(self) -> DPcdConverter {
            DPcdConverter::from_config(self)
        }

        pub fn build_frame_converter(self) -> DFrameConverter {
            DFrameConverter::from_config(self)
        }
    }

    impl<Model, Return> From<&SConfig<Model, Return>> for DConfig
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        fn from(from: &SConfig<Model, Return>) -> Self {
            DConfig {
                product_id: Model::product_id(),
                lasers: from.lasers.as_ref().to_vec(),
                return_mode: Return::return_mode(),
                distance_resolution: Model::distance_resolution(),
            }
        }
    }

    impl<Model, Return> From<SConfig<Model, Return>> for DConfig
    where
        Model: ModelMarker,
        Return: ReturnModeMarker,
        Model::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        fn from(from: SConfig<Model, Return>) -> Self {
            DConfig {
                product_id: Model::product_id(),
                lasers: from.lasers.into_iter().collect(),
                return_mode: Return::return_mode(),
                distance_resolution: Model::distance_resolution(),
            }
        }
    }
}

pub use fns::*;
mod fns {
    use super::*;

    pub fn vlp_16_last_return_config() -> Vlp16_Last_Config {
        SConfig::new()
    }

    pub fn puck_hires_last_return_config() -> PuckHires_Last_Config {
        SConfig::new()
    }

    pub fn puck_lite_last_return_config() -> PuckLite_Last_Config {
        SConfig::new()
    }

    pub fn vlp_16_strongest_return_config() -> Vlp16_Strongest_Config {
        SConfig::new()
    }

    pub fn puck_hires_strongest_return_config() -> PuckHires_Strongest_Config {
        SConfig::new()
    }

    pub fn puck_lite_strongest_return_config() -> PuckLite_Strongest_Config {
        SConfig::new()
    }

    pub fn vlp_16_dual_return_config() -> Vlp16_Dual_Config {
        SConfig::new()
    }

    pub fn puck_hires_dual_return_config() -> PuckHires_Dual_Config {
        SConfig::new()
    }

    pub fn puck_lite_dual_return_config() -> PuckLite_Dual_Config {
        SConfig::new()
    }

    pub fn vlp_32c_last_return_config() -> Vlp32c_Last_Config {
        SConfig::new()
    }

    pub fn vlp_32c_strongest_return_config() -> Vlp32c_Strongest_Config {
        SConfig::new()
    }

    pub fn vlp_32c_dual_return_config() -> Vlp32c_Dual_Config {
        SConfig::new()
    }
}

pub use return_mode_marker::*;
mod return_mode_marker {
    use super::*;

    pub trait ReturnModeMarker
    where
        Self: Debug + Clone,
    {
        fn return_mode() -> ReturnMode;
    }

    #[derive(Debug, Clone, Copy)]
    pub struct StrongestReturn {
        _private: [u8; 0],
    }

    impl ReturnModeMarker for StrongestReturn {
        fn return_mode() -> ReturnMode {
            ReturnMode::Strongest
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct LastReturn {
        _private: [u8; 0],
    }

    impl ReturnModeMarker for LastReturn {
        fn return_mode() -> ReturnMode {
            ReturnMode::Last
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct DualReturn {
        _private: [u8; 0],
    }

    impl ReturnModeMarker for DualReturn {
        fn return_mode() -> ReturnMode {
            ReturnMode::Dual
        }
    }
}

pub use model_marker::*;
mod model_marker {
    pub use super::*;

    pub trait ModelMarker
    where
        Self::ParamArray: IntoIterator<Item = LaserParameter> + AsRef<[LaserParameter]>,
    {
        type ParamArray;

        fn product_id() -> ProductID;
        fn distance_resolution() -> Length;
        fn lasers() -> Self::ParamArray;
    }

    #[derive(Debug, Clone, Copy)]
    #[allow(non_camel_case_types)]
    pub struct VLP_16 {
        _private: [u8; 0],
    }

    #[derive(Debug, Clone, Copy)]
    #[allow(non_camel_case_types)]
    pub struct VLP_32C {
        _private: [u8; 0],
    }

    #[derive(Debug, Clone, Copy)]
    #[allow(non_camel_case_types)]
    pub struct PUCK_LITE {
        _private: [u8; 0],
    }

    #[derive(Debug, Clone, Copy)]
    #[allow(non_camel_case_types)]
    pub struct PUCK_HIRES {
        _private: [u8; 0],
    }

    impl ModelMarker for VLP_16 {
        type ParamArray = [LaserParameter; 16];

        fn product_id() -> ProductID {
            ProductID::VLP16
        }

        fn distance_resolution() -> Length {
            *consts::vlp_16::DISTANCE_RESOLUTION
        }

        fn lasers() -> Self::ParamArray {
            LaserParameter::vlp_16()
        }
    }

    impl ModelMarker for VLP_32C {
        type ParamArray = [LaserParameter; 32];

        fn product_id() -> ProductID {
            ProductID::VLP32C
        }

        fn distance_resolution() -> Length {
            *consts::vlp_32c::DISTANCE_RESOLUTION
        }

        fn lasers() -> Self::ParamArray {
            LaserParameter::vlp_32c()
        }
    }

    impl ModelMarker for PUCK_LITE {
        type ParamArray = [LaserParameter; 16];

        fn product_id() -> ProductID {
            ProductID::PuckLite
        }

        fn distance_resolution() -> Length {
            *consts::puck_lite::DISTANCE_RESOLUTION
        }

        fn lasers() -> Self::ParamArray {
            LaserParameter::puck_lite()
        }
    }

    impl ModelMarker for PUCK_HIRES {
        type ParamArray = [LaserParameter; 16];

        fn product_id() -> ProductID {
            ProductID::PuckHiRes
        }

        fn distance_resolution() -> Length {
            *consts::puck_hires::DISTANCE_RESOLUTION
        }

        fn lasers() -> Self::ParamArray {
            LaserParameter::puck_hires()
        }
    }
}

pub use params::*;
mod params {
    use super::*;

    #[derive(Debug, Clone)]
    pub struct LaserParameter {
        pub elevation_angle: Angle,
        pub azimuth_offset: Angle,
        pub vertical_offset: Length,
        pub horizontal_offset: Length,
    }

    impl LaserParameter {
        pub fn vlp_16() -> [LaserParameter; 16] {
            let mut params: [MaybeUninit<LaserParameter>; 16] =
                unsafe { MaybeUninit::uninit().assume_init() };
            izip!(
                params.iter_mut(),
                consts::vlp_16::ELEVAION_DEGREES,
                consts::vlp_16::VERTICAL_OFFSETS,
                consts::vlp_16::HORIZONTAL_OFFSETS,
                consts::vlp_16::AZIMUTH_OFFSETS,
            )
            .for_each(
                |(param, elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                    *param = MaybeUninit::new(LaserParameter {
                        elevation_angle: Angle::from_degrees(elevation_angle),
                        vertical_offset: Length::from_millimeters(vertical_offset),
                        horizontal_offset: Length::from_millimeters(horizontal_offset),
                        azimuth_offset: Angle::from_degrees(azimuth_offset),
                    });
                },
            );

            unsafe { mem::transmute::<_, [LaserParameter; 16]>(params) }
        }

        pub fn puck_hires() -> [LaserParameter; 16] {
            let mut params: [MaybeUninit<LaserParameter>; 16] =
                unsafe { MaybeUninit::uninit().assume_init() };
            izip!(
                params.iter_mut(),
                consts::puck_hires::ELEVAION_DEGREES,
                consts::puck_hires::VERTICAL_OFFSETS,
                consts::puck_hires::HORIZONTAL_OFFSETS,
                consts::puck_hires::AZIMUTH_OFFSETS,
            )
            .for_each(
                |(param, elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                    *param = MaybeUninit::new(LaserParameter {
                        elevation_angle: Angle::from_degrees(elevation_angle),
                        vertical_offset: Length::from_millimeters(vertical_offset),
                        horizontal_offset: Length::from_millimeters(horizontal_offset),
                        azimuth_offset: Angle::from_degrees(azimuth_offset),
                    });
                },
            );

            unsafe { mem::transmute::<_, [LaserParameter; 16]>(params) }
        }

        pub fn puck_lite() -> [LaserParameter; 16] {
            let mut params: [MaybeUninit<LaserParameter>; 16] =
                unsafe { MaybeUninit::uninit().assume_init() };
            izip!(
                params.iter_mut(),
                consts::puck_lite::ELEVAION_DEGREES,
                consts::puck_lite::VERTICAL_OFFSETS,
                consts::puck_lite::HORIZONTAL_OFFSETS,
                consts::puck_lite::AZIMUTH_OFFSETS,
            )
            .for_each(
                |(param, elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                    *param = MaybeUninit::new(LaserParameter {
                        elevation_angle: Angle::from_degrees(elevation_angle),
                        vertical_offset: Length::from_millimeters(vertical_offset),
                        horizontal_offset: Length::from_millimeters(horizontal_offset),
                        azimuth_offset: Angle::from_degrees(azimuth_offset),
                    });
                },
            );

            unsafe { mem::transmute::<_, [LaserParameter; 16]>(params) }
        }

        pub fn vlp_32c() -> [LaserParameter; 32] {
            let mut params: [MaybeUninit<LaserParameter>; 32] =
                unsafe { MaybeUninit::uninit().assume_init() };
            izip!(
                params.iter_mut(),
                consts::vlp_32c::ELEVAION_DEGREES,
                consts::vlp_32c::VERTICAL_OFFSETS,
                consts::vlp_32c::HORIZONTAL_OFFSETS,
                consts::vlp_32c::AZIMUTH_OFFSETS,
            )
            .for_each(
                |(param, elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                    *param = MaybeUninit::new(LaserParameter {
                        elevation_angle: Angle::from_degrees(elevation_angle),
                        vertical_offset: Length::from_millimeters(vertical_offset),
                        horizontal_offset: Length::from_millimeters(horizontal_offset),
                        azimuth_offset: Angle::from_degrees(azimuth_offset),
                    });
                },
            );

            unsafe { mem::transmute::<_, [LaserParameter; 32]>(params) }
        }
    }
}

pub use param_config::*;
mod param_config {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(try_from = "ParamsConfigUnchecked", into = "ParamsConfigUnchecked")]
    pub struct ParamsConfig {
        lasers: Vec<LaserConfig>,
        num_lasers: usize,
        distance_resolution: R64,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    struct ParamsConfigUnchecked {
        pub lasers: Vec<LaserConfig>,
        pub num_lasers: usize,
        pub distance_resolution: R64,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct LaserConfig {
        pub dist_correction: R64,
        pub dist_correction_x: R64,
        pub dist_correction_y: R64,
        pub focal_distance: R64,
        pub focal_slope: R64,
        pub horiz_offset_correction: Option<R64>,
        pub laser_id: usize,
        pub rot_correction: R64,
        pub vert_correction: R64,
        pub vert_offset_correction: R64,
    }

    impl ParamsConfig {
        pub fn open_yaml<P>(path: P) -> Result<Self>
        where
            P: AsRef<Path>,
        {
            let mut reader = BufReader::new(File::open(path)?);
            let config = Self::from_reader_yaml(&mut reader)?;
            Ok(config)
        }

        pub fn from_reader_yaml<R>(reader: &mut R) -> Result<Self>
        where
            R: Read,
        {
            let mut text = String::new();
            reader.read_to_string(&mut text)?;
            let config = serde_yaml::from_str(&text)?;
            Ok(config)
        }

        /// Get a reference to the params config's lasers.
        pub fn lasers(&self) -> &[LaserConfig] {
            self.lasers.as_ref()
        }

        /// Get the params config's distance resolution.
        pub fn distance_resolution(&self) -> R64 {
            self.distance_resolution
        }

        /// Get the params config's num lasers.
        pub fn num_lasers(&self) -> usize {
            self.num_lasers
        }
    }

    impl TryFrom<ParamsConfigUnchecked> for ParamsConfig {
        type Error = Error;

        fn try_from(from: ParamsConfigUnchecked) -> Result<Self, Self::Error> {
            let ParamsConfigUnchecked {
                lasers,
                num_lasers,
                distance_resolution,
            } = from;

            ensure!(
                from.distance_resolution > 0.0,
                "distance_resolution must be positive"
            );
            ensure!(
                num_lasers == lasers.len(),
                "the number of element in lasers field does not match num_layers"
            );
            ensure!(
                {
                    lasers
                        .iter()
                        .enumerate()
                        .all(|(idx, params)| idx == params.laser_id)
                },
                "the laser_id in lasers field must be consecutively counted from 1"
            );

            Ok(Self {
                lasers,
                num_lasers,
                distance_resolution,
            })
        }
    }

    impl From<ParamsConfig> for ParamsConfigUnchecked {
        fn from(from: ParamsConfig) -> Self {
            let ParamsConfig {
                lasers,
                num_lasers,
                distance_resolution,
            } = from;

            Self {
                lasers,
                num_lasers,
                distance_resolution,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Result;

    #[test]
    fn load_yaml_params_test() -> Result<()> {
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/32db.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/64e_s2.1-sztaki.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/64e_s3-xiesc.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/64e_utexas.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/VeloView-VLP-32C.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/VLP16db.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/VLP16_hires_db.yaml"
        ))?;
        Ok(())
    }
}
