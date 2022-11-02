use super::functions;
use crate::{
    common::*,
    config::{Config, LaserParameter},
    firing::types::{
        FiringDual16, FiringDual32, FiringFormat, FiringKind, FiringSingle16, FiringSingle32,
    },
    firing_xyz::{
        iter::{
            FiringXyzDual16Iter, FiringXyzDual32Iter, FiringXyzKindIter, FiringXyzSingle16Iter,
            FiringXyzSingle32Iter,
        },
        types::{
            FiringXyzDual16, FiringXyzDual32, FiringXyzKind, FiringXyzSingle16, FiringXyzSingle32,
        },
    },
    frame_xyz::{
        iter::{
            FrameXyzDual16Iter, FrameXyzDual32Iter, FrameXyzIter, FrameXyzSingle16Iter,
            FrameXyzSingle32Iter,
        },
        types::{FrameXyzDual16, FrameXyzDual32, FrameXyzSingle16, FrameXyzSingle32},
    },
    packet::DataPacket,
};

macro_rules! declare_converter {
    (
        $name:ident,
        $size:expr,
        $firing:ident,
        $firing_xyz:ident,
        $firing_xyz_iter:ident,
        $convert_fn:path,
        $firing_method:ident,
        $frame_xyz:ident,
        $frame_xyz_iter:ident $(,)?
    ) => {
        #[derive(Debug, Clone)]
        pub struct $name {
            pub(crate) lasers: [LaserParameter; $size],
            pub(crate) distance_resolution: Length,
        }

        impl $name {
            pub fn firing_to_firing_xyz<'a>(&'a self, firing: $firing<'a>) -> $firing_xyz {
                $convert_fn(&firing, self.distance_resolution, &self.lasers)
            }

            pub fn firing_iter_to_firing_xyz_iter<'a, I>(
                &'a self,
                firings: I,
            ) -> $firing_xyz_iter<impl Iterator<Item = $firing_xyz> + 'a>
            where
                I: IntoIterator<Item = $firing<'a>>,
                I::IntoIter: 'a,
            {
                let iter = firings
                    .into_iter()
                    .map(|firing| self.firing_to_firing_xyz(firing));
                $firing_xyz_iter(iter)
            }

            pub fn firing_iter_to_frame_xyz_iter<'a, I>(
                &'a self,
                firings: I,
            ) -> $frame_xyz_iter<impl Iterator<Item = $frame_xyz> + 'a>
            where
                I: IntoIterator<Item = $firing<'a>> + 'a,
            {
                self.firing_iter_to_firing_xyz_iter(firings)
                    .into_frame_iter()
            }

            pub fn packet_to_firing_xyz_iter<'a>(
                &'a self,
                packet: &'a DataPacket,
            ) -> $firing_xyz_iter<impl Iterator<Item = $firing_xyz> + 'a> {
                self.firing_iter_to_firing_xyz_iter(packet.$firing_method())
            }

            pub fn packet_iter_to_firing_xyz_iter<'a, P, I>(
                &'a self,
                packets: I,
            ) -> $firing_xyz_iter<impl Iterator<Item = $firing_xyz> + 'a>
            where
                I: IntoIterator<Item = P>,
                I::IntoIter: 'a,
                P: Borrow<DataPacket> + 'a,
            {
                let iter = packets.into_iter().flat_map(|packet| {
                    let firings: Vec<_> = self.packet_to_firing_xyz_iter(packet.borrow()).collect();
                    firings
                });
                $firing_xyz_iter(iter)
            }

            pub fn packet_iter_to_frame_xyz_iter<'a, P, I>(
                &'a self,
                packets: I,
            ) -> $frame_xyz_iter<impl Iterator<Item = $frame_xyz> + 'a>
            where
                I: IntoIterator<Item = P> + 'a,
                P: Borrow<DataPacket> + 'a,
            {
                self.packet_iter_to_firing_xyz_iter(packets)
                    .into_frame_iter()
            }
        }
    };
}

declare_converter!(
    ConverterSingle16,
    16,
    FiringSingle16,
    FiringXyzSingle16,
    FiringXyzSingle16Iter,
    functions::firing_single_16_to_xyz,
    single_16_firings,
    FrameXyzSingle16,
    FrameXyzSingle16Iter,
);

declare_converter!(
    ConverterSingle32,
    32,
    FiringSingle32,
    FiringXyzSingle32,
    FiringXyzSingle32Iter,
    functions::firing_single_32_to_xyz,
    single_32_firings,
    FrameXyzSingle32,
    FrameXyzSingle32Iter,
);

declare_converter!(
    ConverterDual16,
    16,
    FiringDual16,
    FiringXyzDual16,
    FiringXyzDual16Iter,
    functions::firing_dual_16_to_xyz,
    dual_16_firings,
    FrameXyzDual16,
    FrameXyzDual16Iter,
);

declare_converter!(
    ConverterDual32,
    32,
    FiringDual32,
    FiringXyzDual32,
    FiringXyzDual32Iter,
    functions::firing_dual_32_to_xyz,
    dual_32_firings,
    FrameXyzDual32,
    FrameXyzDual32Iter,
);

pub use kind::*;
mod kind {

    use super::*;

    #[derive(Debug, Clone)]
    pub enum ConverterKind {
        Single16(ConverterSingle16),
        Single32(ConverterSingle32),
        Dual16(ConverterDual16),
        Dual32(ConverterDual32),
    }

    impl From<ConverterDual32> for ConverterKind {
        fn from(v: ConverterDual32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<ConverterDual16> for ConverterKind {
        fn from(v: ConverterDual16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<ConverterSingle32> for ConverterKind {
        fn from(v: ConverterSingle32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<ConverterSingle16> for ConverterKind {
        fn from(v: ConverterSingle16) -> Self {
            Self::Single16(v)
        }
    }

    impl ConverterKind {
        pub fn firing_format(&self) -> FiringFormat {
            use FiringFormat as F;

            match self {
                Self::Single16(_) => F::Single16,
                Self::Single32(_) => F::Single32,
                Self::Dual16(_) => F::Dual16,
                Self::Dual32(_) => F::Dual32,
            }
        }

        pub fn firing_to_firing_xyz<'a>(
            &self,
            firing: FiringKind<'a>,
        ) -> Result<FiringXyzKind, FiringKind<'a>> {
            use FiringKind as F;

            Ok(match (self, firing) {
                (Self::Single16(conv), F::Single16(firing)) => {
                    conv.firing_to_firing_xyz(firing).into()
                }
                (Self::Single32(conv), F::Single32(firing)) => {
                    conv.firing_to_firing_xyz(firing).into()
                }
                (Self::Dual16(conv), F::Dual16(firing)) => conv.firing_to_firing_xyz(firing).into(),
                (Self::Dual32(conv), F::Dual32(firing)) => conv.firing_to_firing_xyz(firing).into(),
                (_, firing) => return Err(firing),
            })
        }

        // pub fn firing_iter_to_firing_xyz_iter<'a, I>(
        //     &'a self,
        //     firings: I,
        // ) -> impl Iterator<Item = Result<FiringXyzKind, FiringKind<'a>>> + '_
        // where
        //     I: IntoIterator<Item = FiringKind<'a>>,
        //     I::IntoIter: 'a,
        // {
        //     let firings = firings.into_iter();

        //     match self {
        //         ConverterKind::Single16(conv) => firings.map(|firing| {
        //             firing
        //                 .try_into_single16()
        //                 .map(|firing| conv.firing_to_firing_xyz(firing).into())
        //                 .boxed()
        //         }),
        //         ConverterKind::Single32(conv) => firings.map(|firing| {
        //             firing
        //                 .try_into_single32()
        //                 .map(|firing| conv.firing_to_firing_xyz(firing).into())
        //                 .boxed()
        //         }),
        //         ConverterKind::Dual16(conv) => firings.map(|firing| {
        //             firing
        //                 .try_into_dual16()
        //                 .map(|firing| conv.firing_to_firing_xyz(firing).into())
        //                 .boxed()
        //         }),
        //         ConverterKind::Dual32(conv) => firings.map(|firing| {
        //             firing
        //                 .try_into_dual32()
        //                 .map(|firing| conv.firing_to_firing_xyz(firing).into())
        //                 .boxed()
        //         }),
        //     }
        // }

        pub fn packet_to_firing_xyz_iter<'a>(
            &'a self,
            packet: &'a DataPacket,
        ) -> FiringXyzKindIter<
            impl Iterator<Item = FiringXyzSingle16> + 'a,
            impl Iterator<Item = FiringXyzSingle32> + 'a,
            impl Iterator<Item = FiringXyzDual16> + 'a,
            impl Iterator<Item = FiringXyzDual32> + 'a,
        > {
            match self {
                Self::Single16(conv) => conv.packet_to_firing_xyz_iter(packet).into(),
                Self::Single32(conv) => conv.packet_to_firing_xyz_iter(packet).into(),
                Self::Dual16(conv) => conv.packet_to_firing_xyz_iter(packet).into(),
                Self::Dual32(conv) => conv.packet_to_firing_xyz_iter(packet).into(),
            }
        }

        pub fn packet_iter_to_firing_xyz_iter<'a, P, I>(
            &'a self,
            packets: I,
        ) -> FiringXyzKindIter<
            impl Iterator<Item = FiringXyzSingle16> + 'a,
            impl Iterator<Item = FiringXyzSingle32> + 'a,
            impl Iterator<Item = FiringXyzDual16> + 'a,
            impl Iterator<Item = FiringXyzDual32> + 'a,
        >
        where
            I: IntoIterator<Item = P> + 'a,
            I::IntoIter: 'a,
            P: Borrow<DataPacket> + 'a,
        {
            match self {
                Self::Single16(conv) => conv.packet_iter_to_firing_xyz_iter(packets).into(),
                Self::Single32(conv) => conv.packet_iter_to_firing_xyz_iter(packets).into(),
                Self::Dual16(conv) => conv.packet_iter_to_firing_xyz_iter(packets).into(),
                Self::Dual32(conv) => conv.packet_iter_to_firing_xyz_iter(packets).into(),
            }
        }

        pub fn packet_iter_to_frame_xyz_iter<'a, P, I>(
            &'a self,
            packets: I,
        ) -> FrameXyzIter<
            impl Iterator<Item = FrameXyzSingle16> + 'a,
            impl Iterator<Item = FrameXyzSingle32> + 'a,
            impl Iterator<Item = FrameXyzDual16> + 'a,
            impl Iterator<Item = FrameXyzDual32> + 'a,
        >
        where
            I: IntoIterator<Item = P> + 'a,
            I::IntoIter: 'a,
            P: Borrow<DataPacket> + 'a,
        {
            match self {
                Self::Single16(conv) => conv.packet_iter_to_frame_xyz_iter(packets).into(),
                Self::Single32(conv) => conv.packet_iter_to_frame_xyz_iter(packets).into(),
                Self::Dual16(conv) => conv.packet_iter_to_frame_xyz_iter(packets).into(),
                Self::Dual32(conv) => conv.packet_iter_to_frame_xyz_iter(packets).into(),
            }
        }

        pub fn from_config(config: Config) -> Result<Self> {
            use FiringFormat as F;

            let firing_format = config
                .firing_format()
                .ok_or_else(|| format_err!("product is not supported"))?;
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            let err = || format_err!("invalid laser parameters");

            Ok(match firing_format {
                F::Single16 => ConverterSingle16 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
                F::Dual16 => ConverterDual16 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
                F::Single32 => ConverterSingle32 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
                F::Dual32 => ConverterDual32 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
            })
        }

        pub fn try_into_single16(self) -> Result<ConverterSingle16, Self> {
            if let Self::Single16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_single32(self) -> Result<ConverterSingle32, Self> {
            if let Self::Single32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual16(self) -> Result<ConverterDual16, Self> {
            if let Self::Dual16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual32(self) -> Result<ConverterDual32, Self> {
            if let Self::Dual32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn into_single16(self) -> ConverterSingle16 {
            self.try_into_single16().unwrap()
        }

        pub fn into_single32(self) -> ConverterSingle32 {
            self.try_into_single32().unwrap()
        }

        pub fn into_dual16(self) -> ConverterDual16 {
            self.try_into_dual16().unwrap()
        }

        pub fn into_dual32(self) -> ConverterDual32 {
            self.try_into_dual32().unwrap()
        }
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
//     fn spherical_to_xyz_precisoin_test() {
//         use rand::prelude::*;
//         use std::f64::consts::{FRAC_PI_2, PI};

//         let mut rng = rand::thread_rng();

//         LaserParameter::vlp_32c().into_iter().for_each(|laser| {
//             let LaserParameter {
//                 elevation,
//                 azimuth_offset,
//                 vertical_offset,
//                 horizontal_offset,
//             } = laser;

//             assert!(((-FRAC_PI_2)..=FRAC_PI_2).contains(&elevation.as_radians()));

//             (0..1000).for_each(|_| {
//                 let distance = Length::from_meters(rng.gen_range(0f64..200.0));
//                 let azimuth = (Angle::from_radians(rng.gen_range(0f64..(PI * 2.0)))
//                     + azimuth_offset)
//                     .wrap_to_2pi();

//                 let slower = spherical_to_xyz_generic(
//                     distance,
//                     elevation,
//                     azimuth,
//                     vertical_offset,
//                     horizontal_offset,
//                 );
//                 let faster = spherical_to_xyz_x86(
//                     distance,
//                     elevation,
//                     azimuth,
//                     vertical_offset,
//                     horizontal_offset,
//                 );

//                 let [x1, y1, z1] = slower;
//                 let [x2, y2, z2] = faster;

//                 let l2_m = ((x2 - x1).as_meters().powi(2)
//                     + (y2 - y1).as_meters().powi(2)
//                     + (z2 - z1).as_meters().powi(2))
//                 .sqrt();

//                 assert!(
//                     l2_m <= 14e-3,
//                     "large position error {} meters for distance={}, azimuth={}, elevation={}",
//                     l2_m,
//                     distance,
//                     azimuth,
//                     elevation,
//                 );
//             });
//         });
//     }
// }
