use crate::{
    common::*,
    firing_block::{FiringBlockD16, FiringBlockD32, FiringBlockS16, FiringBlockS32},
    firing_xyz::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
    kinds::FormatKind,
    traits::AzimuthRange,
};
#[cfg(feature = "async")]
use futures::stream::{self, Stream, StreamExt as _};

#[derive(Debug, Clone)]
pub struct Batcher<E>
where
    E: AzimuthRange,
{
    buffer: Vec<E>,
}

impl<E> Batcher<E>
where
    E: AzimuthRange,
{
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push_one(&mut self, firing: E) -> Option<Vec<E>> {
        let buffer = &mut self.buffer;
        let wrap =
            matches!(buffer.last(), Some(prev) if prev.start_azimuth() > firing.start_azimuth());

        if wrap {
            let output = mem::replace(buffer, vec![firing]);
            Some(output)
        } else {
            buffer.push(firing);
            None
        }
    }

    pub fn push_many<'a, I>(&'a mut self, firings: I) -> impl Iterator<Item = Vec<E>> + 'a
    where
        I: IntoIterator<Item = E> + 'a,
    {
        firings
            .into_iter()
            .filter_map(|firing| self.push_one(firing))
    }

    pub fn take(&mut self) -> Option<Vec<E>> {
        let firings = mem::take(&mut self.buffer);
        (!firings.is_empty()).then_some(firings)
    }

    pub fn with_iter<I>(self, firings: I) -> impl Iterator<Item = Vec<E>>
    where
        I: IntoIterator<Item = E>,
    {
        itertools::unfold(Some((firings.into_iter(), self)), |state| {
            if let Some((iter, conv)) = state {
                Some(if let Some(firing) = iter.next() {
                    conv.push_one(firing)
                } else {
                    let output = conv.take();
                    *state = None;
                    output
                })
            } else {
                None
            }
        })
        .flatten()
    }
}

impl<E> Default for Batcher<E>
where
    E: AzimuthRange,
{
    fn default() -> Self {
        Self::new()
    }
}

pub type FiringBlockBatcherS16<'a> = Batcher<FiringBlockS16<'a>>;
pub type FiringBlockBatcherS32<'a> = Batcher<FiringBlockS32<'a>>;
pub type FiringBlockBatcherD16<'a> = Batcher<FiringBlockD16<'a>>;
pub type FiringBlockBatcherD32<'a> = Batcher<FiringBlockD32<'a>>;

pub type FiringBlockBatcher<'a> = FormatKind<
    FiringBlockBatcherS16<'a>,
    FiringBlockBatcherS32<'a>,
    FiringBlockBatcherD16<'a>,
    FiringBlockBatcherD32<'a>,
>;

pub type FiringXyzBatcher =
    FormatKind<FiringXyzBatcherS16, FiringXyzBatcherS32, FiringXyzBatcherD16, FiringXyzBatcherD32>;

pub type FiringXyzBatcherS16 = Batcher<FiringXyzS16>;
pub type FiringXyzBatcherS32 = Batcher<FiringXyzS32>;
pub type FiringXyzBatcherD16 = Batcher<FiringXyzD16>;
pub type FiringXyzBatcherD32 = Batcher<FiringXyzD32>;

// macro_rules! declare_converter {
//     ($name:ident, $firing:ident, $frame:ident) => {
//         #[derive(Debug)]
//         pub struct $name {
//             buffer: Vec<$firing>,
//         }

//         impl $name {
//             pub fn new() -> Self {
//                 Self { buffer: vec![] }
//             }

//             pub fn push_one(&mut self, firing: $firing) -> Option<$frame> {
//                 let firings = push_one(&mut self.buffer, firing)?;
//                 Some($frame { firings })
//             }

//             pub fn push_many<'a, I>(&'a mut self, firings: I) -> impl Iterator<Item = $frame> + 'a
//             where
//                 I: IntoIterator<Item = $firing> + 'a,
//             {
//                 push_many(&mut self.buffer, firings).map(|firings| $frame { firings })
//             }

//             pub fn take(&mut self) -> Option<$frame> {
//                 let firings = mem::take(&mut self.buffer);
//                 (!firings.is_empty()).then(|| $frame { firings })
//             }

//             pub fn with_iter<I>(self, firings: I) -> impl Iterator<Item = $frame>
//             where
//                 I: IntoIterator<Item = $firing>,
//             {
//                 itertools::unfold(Some((firings.into_iter(), self)), |state| {
//                     if let Some((iter, conv)) = state {
//                         Some(if let Some(firing) = iter.next() {
//                             conv.push_one(firing)
//                         } else {
//                             let output = conv.take();
//                             *state = None;
//                             output
//                         })
//                     } else {
//                         None
//                     }
//                 })
//                 .flatten()
//             }
//         }

//         #[cfg(feature = "async")]
//         impl $name {
//             pub fn with_stream<I>(self, firings: I) -> impl Stream<Item = $frame>
//             where
//                 I: Stream<Item = $firing> + Send + 'static,
//             {
//                 stream::unfold(Some((firings.boxed(), self)), |mut state| async move {
//                     if let Some((iter, conv)) = &mut state {
//                         loop {
//                             if let Some(firing) = iter.next().await {
//                                 if let Some(frame) = conv.push_one(firing) {
//                                     break Some((frame, state));
//                                 }
//                             } else {
//                                 break conv.take().map(|frame| (frame, None));
//                             }
//                         }
//                     } else {
//                         None
//                     }
//                 })
//             }
//         }

//         impl Default for $name {
//             fn default() -> Self {
//                 Self::new()
//             }
//         }
//     };
// }

// declare_converter!(FrameXyzBatcherS16, FiringXyzS16, FrameXyzS16);
// declare_converter!(FrameXyzBatcherS32, FiringXyzS32, FrameXyzS32);
// declare_converter!(FrameXyzBatcherD16, FiringXyzD16, FrameXyzD16);
// declare_converter!(FrameXyzBatcherD32, FiringXyzD32, FrameXyzD32);

impl FiringXyzBatcher {
    // pub fn from_config(config: &Config) -> Result<Self> {
    //     use Format as F;
    //     let firing_format = config
    //         .firing_format()
    //         .ok_or_else(|| format_err!("product is not supported"))?;

    //     Ok(match firing_format {
    //         F::Single16 => Self::from_s16(FiringXyzBatcherS16::new()),
    //         F::Dual16 => Self::from_d16(FiringXyzBatcherD16::new().into()),
    //         F::Single32 => Self::from_s32(FiringXyzBatcherS32::new().into()),
    //         F::Dual32 => Self::from_d32(FiringXyzBatcherD32::new().into()),
    //     })
    // }

    // pub fn push_one(&mut self, firing: FiringXyz) -> Result<Option<Vec<FiringXyz>>> {
    //     use FiringXyz as F;

    //     ensure!(
    //         self.format() == firing.format(),
    //         "batcher type and firing type mismatch"
    //     );

    //     let frame: Option<_> = (move || {
    //         let frame: Vec<FiringXyz> = match (self, firing) {
    //             (Self::Single16(me), F::Single16(firing)) => me
    //                 .push_one(firing)?
    //                 .into_iter()
    //                 .map(FiringXyz::from_s16)
    //                 .collect(),
    //             (Self::Single32(me), F::Single32(firing)) => me
    //                 .push_one(firing)?
    //                 .into_iter()
    //                 .map(FiringXyz::from_s32)
    //                 .collect(),
    //             (Self::Dual16(me), F::Dual16(firing)) => me
    //                 .push_one(firing)?
    //                 .into_iter()
    //                 .map(FiringXyz::from_d16)
    //                 .collect(),
    //             (Self::Dual32(me), F::Dual32(firing)) => me
    //                 .push_one(firing)?
    //                 .into_iter()
    //                 .map(FiringXyz::from_d32)
    //                 .collect(),
    //             _ => unreachable!(),
    //         };
    //         Some(frame)
    //     })();

    //     Ok(frame)
    // }

    // pub fn push_many<'a, I>(&'a mut self, firings: I) -> impl Iterator<Item = Result<FrameXyz>> + 'a
    // where
    //     I: IntoIterator<Item = FiringXyz> + 'a,
    // {
    //     let firings = firings.into_iter();
    //     let err = || format_err!("batcher and firing type mismatch");

    //     let frame_iter: Box<dyn Iterator<Item = Result<Option<FrameXyz>>>> = match self {
    //         Self::Single16(me) => Box::new(firings.map(move |firing| {
    //             let firing = firing.try_into_single16().map_err(|_| err())?;
    //             let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
    //             Ok(frame)
    //         })),
    //         Self::Single32(me) => Box::new(firings.map(move |firing| {
    //             let firing = firing.try_into_single32().map_err(|_| err())?;
    //             let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
    //             Ok(frame)
    //         })),
    //         Self::Dual16(me) => Box::new(firings.map(move |firing| {
    //             let firing = firing.try_into_dual16().map_err(|_| err())?;
    //             let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
    //             Ok(frame)
    //         })),
    //         Self::Dual32(me) => Box::new(firings.map(move |firing| {
    //             let firing = firing.try_into_dual32().map_err(|_| err())?;
    //             let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
    //             Ok(frame)
    //         })),
    //     };
    //     frame_iter.flat_map(|frame| frame.transpose())
    // }

    // pub fn take(&mut self) -> Option<FrameXyz> {
    //     match self {
    //         Self::Single16(me) => me.take().map(Into::into),
    //         Self::Single32(me) => me.take().map(Into::into),
    //         Self::Dual16(me) => me.take().map(Into::into),
    //         Self::Dual32(me) => me.take().map(Into::into),
    //     }
    // }
}
