use crate::{
    common::*,
    config::Config,
    firing_xyz::types::{
        FiringXyz, FiringXyzD16, FiringXyzD32, FiringXyzKind, FiringXyzS16, FiringXyzS32,
    },
    frame_xyz::types::{FrameXyz, FrameXyzD16, FrameXyzD32, FrameXyzS16, FrameXyzS32},
    kinds::{Format, FormatKind},
};
#[cfg(feature = "async")]
use futures::stream::{self, Stream, StreamExt as _};

macro_rules! declare_converter {
    ($name:ident, $firing:ident, $frame:ident) => {
        #[derive(Debug)]
        pub struct $name {
            buffer: Vec<$firing>,
        }

        impl $name {
            pub fn new() -> Self {
                Self { buffer: vec![] }
            }

            pub fn push_one(&mut self, firing: $firing) -> Option<$frame> {
                let firings = push_one(&mut self.buffer, firing)?;
                Some($frame { firings })
            }

            pub fn push_many<'a, I>(&'a mut self, firings: I) -> impl Iterator<Item = $frame> + 'a
            where
                I: IntoIterator<Item = $firing> + 'a,
            {
                push_many(&mut self.buffer, firings).map(|firings| $frame { firings })
            }

            pub fn take(&mut self) -> Option<$frame> {
                let firings = mem::take(&mut self.buffer);
                (!firings.is_empty()).then(|| $frame { firings })
            }

            pub fn with_iter<I>(self, firings: I) -> impl Iterator<Item = $frame>
            where
                I: IntoIterator<Item = $firing>,
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

        #[cfg(feature = "async")]
        impl $name {
            pub fn with_stream<I>(self, firings: I) -> impl Stream<Item = $frame>
            where
                I: Stream<Item = $firing> + Send + 'static,
            {
                stream::unfold(Some((firings.boxed(), self)), |mut state| async move {
                    if let Some((iter, conv)) = &mut state {
                        loop {
                            if let Some(firing) = iter.next().await {
                                if let Some(frame) = conv.push_one(firing) {
                                    break Some((frame, state));
                                }
                            } else {
                                break conv.take().map(|frame| (frame, None));
                            }
                        }
                    } else {
                        None
                    }
                })
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }
    };
}

declare_converter!(FrameXyzBatcherS16, FiringXyzS16, FrameXyzS16);
declare_converter!(FrameXyzBatcherS32, FiringXyzS32, FrameXyzS32);
declare_converter!(FrameXyzBatcherD16, FiringXyzD16, FrameXyzD16);
declare_converter!(FrameXyzBatcherD32, FiringXyzD32, FrameXyzD32);

pub use kind::*;
mod kind {
    use super::*;

    pub type FrameXyzBatcher =
        FormatKind<FrameXyzBatcherS16, FrameXyzBatcherS32, FrameXyzBatcherD16, FrameXyzBatcherD32>;

    impl FrameXyzBatcher {
        pub fn from_config(config: &Config) -> Result<Self> {
            use Format as F;
            let firing_format = config
                .firing_format()
                .ok_or_else(|| format_err!("product is not supported"))?;

            Ok(match firing_format {
                F::Single16 => FrameXyzBatcherS16::new().into(),
                F::Dual16 => FrameXyzBatcherD16::new().into(),
                F::Single32 => FrameXyzBatcherS32::new().into(),
                F::Dual32 => FrameXyzBatcherD32::new().into(),
            })
        }

        pub fn try_into_single16(self) -> Result<FrameXyzBatcherS16, Self> {
            if let Self::Single16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_single32(self) -> Result<FrameXyzBatcherS32, Self> {
            if let Self::Single32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual16(self) -> Result<FrameXyzBatcherD16, Self> {
            if let Self::Dual16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual32(self) -> Result<FrameXyzBatcherD32, Self> {
            if let Self::Dual32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn into_single16(self) -> FrameXyzBatcherS16 {
            self.try_into_single16().unwrap()
        }

        pub fn into_single32(self) -> FrameXyzBatcherS32 {
            self.try_into_single32().unwrap()
        }

        pub fn into_dual16(self) -> FrameXyzBatcherD16 {
            self.try_into_dual16().unwrap()
        }

        pub fn into_dual32(self) -> FrameXyzBatcherD32 {
            self.try_into_dual32().unwrap()
        }

        pub fn push_one(&mut self, firing: FiringXyz) -> Result<Option<FrameXyz>> {
            use FiringXyz as F;

            let frame = match (self, firing) {
                (Self::Single16(me), F::Single16(firing)) => me.push_one(firing).map(Into::into),
                (Self::Single32(me), F::Single32(firing)) => me.push_one(firing).map(Into::into),
                (Self::Dual16(me), F::Dual16(firing)) => me.push_one(firing).map(Into::into),
                (Self::Dual32(me), F::Dual32(firing)) => me.push_one(firing).map(Into::into),
                _ => bail!("batcher type and firing type mismatch"),
            };

            Ok(frame)
        }

        pub fn push_many<'a, I>(
            &'a mut self,
            firings: I,
        ) -> impl Iterator<Item = Result<FrameXyz>> + 'a
        where
            I: IntoIterator<Item = FiringXyz> + 'a,
        {
            let firings = firings.into_iter();
            let err = || format_err!("batcher and firing type mismatch");

            let frame_iter: Box<dyn Iterator<Item = Result<Option<FrameXyz>>>> = match self {
                Self::Single16(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_single16().map_err(|_| err())?;
                    let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
                Self::Single32(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_single32().map_err(|_| err())?;
                    let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
                Self::Dual16(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_dual16().map_err(|_| err())?;
                    let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
                Self::Dual32(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_dual32().map_err(|_| err())?;
                    let frame: Option<FrameXyz> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
            };
            frame_iter.flat_map(|frame| frame.transpose())
        }

        pub fn take(&mut self) -> Option<FrameXyz> {
            match self {
                Self::Single16(me) => me.take().map(Into::into),
                Self::Single32(me) => me.take().map(Into::into),
                Self::Dual16(me) => me.take().map(Into::into),
                Self::Dual32(me) => me.take().map(Into::into),
            }
        }
    }

    impl From<FrameXyzBatcherD32> for FrameXyzBatcher {
        fn from(v: FrameXyzBatcherD32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<FrameXyzBatcherD16> for FrameXyzBatcher {
        fn from(v: FrameXyzBatcherD16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<FrameXyzBatcherS32> for FrameXyzBatcher {
        fn from(v: FrameXyzBatcherS32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<FrameXyzBatcherS16> for FrameXyzBatcher {
        fn from(v: FrameXyzBatcherS16) -> Self {
            Self::Single16(v)
        }
    }
}

fn push_one<F: FiringXyzKind>(buffer: &mut Vec<F>, curr: F) -> Option<Vec<F>> {
    let wrap = matches!(buffer.last(), Some(prev) if prev.azimuth() > curr.azimuth());

    if wrap {
        let output = mem::replace(buffer, vec![curr]);
        Some(output)
    } else {
        buffer.push(curr);
        None
    }
}

fn push_many<'a, F: FiringXyzKind, I>(
    buffer: &'a mut Vec<F>,
    iter: I,
) -> impl Iterator<Item = Vec<F>> + 'a
where
    I: IntoIterator<Item = F>,
    I::IntoIter: 'a,
{
    iter.into_iter()
        .scan(buffer, |buffer, firing| Some(push_one(buffer, firing)))
        .flatten()
}
