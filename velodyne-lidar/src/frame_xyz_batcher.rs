use crate::{
    common::*,
    config::Config,
    firing::FiringFormat,
    firing_xyz::{
        FiringXyz, FiringXyzDual16, FiringXyzDual32, FiringXyzKind, FiringXyzSingle16,
        FiringXyzSingle32,
    },
    frame_xyz::{FrameXyzDual16, FrameXyzDual32, FrameXyzKind, FrameXyzSingle16, FrameXyzSingle32},
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

declare_converter!(FrameXyzBatcherSingle16, FiringXyzSingle16, FrameXyzSingle16);
declare_converter!(FrameXyzBatcherSingle32, FiringXyzSingle32, FrameXyzSingle32);
declare_converter!(FrameXyzBatcherDual16, FiringXyzDual16, FrameXyzDual16);
declare_converter!(FrameXyzBatcherDual32, FiringXyzDual32, FrameXyzDual32);

pub use kind::*;
mod kind {

    use super::*;

    #[derive(Debug)]
    pub enum FrameXyzBatcherKind {
        Single16(FrameXyzBatcherSingle16),
        Single32(FrameXyzBatcherSingle32),
        Dual16(FrameXyzBatcherDual16),
        Dual32(FrameXyzBatcherDual32),
    }

    impl FrameXyzBatcherKind {
        pub fn from_config(config: &Config) -> Result<Self> {
            use FiringFormat as F;
            let firing_format = config
                .firing_format()
                .ok_or_else(|| format_err!("product is not supported"))?;

            Ok(match firing_format {
                F::Single16 => FrameXyzBatcherSingle16::new().into(),
                F::Dual16 => FrameXyzBatcherDual16::new().into(),
                F::Single32 => FrameXyzBatcherSingle32::new().into(),
                F::Dual32 => FrameXyzBatcherDual32::new().into(),
            })
        }

        pub fn try_into_single16(self) -> Result<FrameXyzBatcherSingle16, Self> {
            if let Self::Single16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_single32(self) -> Result<FrameXyzBatcherSingle32, Self> {
            if let Self::Single32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual16(self) -> Result<FrameXyzBatcherDual16, Self> {
            if let Self::Dual16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual32(self) -> Result<FrameXyzBatcherDual32, Self> {
            if let Self::Dual32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn into_single16(self) -> FrameXyzBatcherSingle16 {
            self.try_into_single16().unwrap()
        }

        pub fn into_single32(self) -> FrameXyzBatcherSingle32 {
            self.try_into_single32().unwrap()
        }

        pub fn into_dual16(self) -> FrameXyzBatcherDual16 {
            self.try_into_dual16().unwrap()
        }

        pub fn into_dual32(self) -> FrameXyzBatcherDual32 {
            self.try_into_dual32().unwrap()
        }

        pub fn push_one(&mut self, firing: FiringXyzKind) -> Result<Option<FrameXyzKind>> {
            use FiringXyzKind as F;

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
        ) -> impl Iterator<Item = Result<FrameXyzKind>> + 'a
        where
            I: IntoIterator<Item = FiringXyzKind> + 'a,
        {
            let firings = firings.into_iter();
            let err = || format_err!("batcher and firing type mismatch");

            let frame_iter: Box<dyn Iterator<Item = Result<Option<FrameXyzKind>>>> = match self {
                Self::Single16(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_single16().map_err(|_| err())?;
                    let frame: Option<FrameXyzKind> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
                Self::Single32(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_single32().map_err(|_| err())?;
                    let frame: Option<FrameXyzKind> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
                Self::Dual16(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_dual16().map_err(|_| err())?;
                    let frame: Option<FrameXyzKind> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
                Self::Dual32(me) => Box::new(firings.map(move |firing| {
                    let firing = firing.try_into_dual32().map_err(|_| err())?;
                    let frame: Option<FrameXyzKind> = me.push_one(firing).map(Into::into);
                    Ok(frame)
                })),
            };
            frame_iter.flat_map(|frame| frame.transpose())
        }

        pub fn take(&mut self) -> Option<FrameXyzKind> {
            match self {
                Self::Single16(me) => me.take().map(Into::into),
                Self::Single32(me) => me.take().map(Into::into),
                Self::Dual16(me) => me.take().map(Into::into),
                Self::Dual32(me) => me.take().map(Into::into),
            }
        }
    }

    impl From<FrameXyzBatcherDual32> for FrameXyzBatcherKind {
        fn from(v: FrameXyzBatcherDual32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<FrameXyzBatcherDual16> for FrameXyzBatcherKind {
        fn from(v: FrameXyzBatcherDual16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<FrameXyzBatcherSingle32> for FrameXyzBatcherKind {
        fn from(v: FrameXyzBatcherSingle32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<FrameXyzBatcherSingle16> for FrameXyzBatcherKind {
        fn from(v: FrameXyzBatcherSingle16) -> Self {
            Self::Single16(v)
        }
    }
}

fn push_one<F: FiringXyz>(buffer: &mut Vec<F>, curr: F) -> Option<Vec<F>> {
    let wrap = matches!(buffer.last(), Some(prev) if prev.azimuth() > curr.azimuth());

    if wrap {
        let output = mem::replace(buffer, vec![curr]);
        Some(output)
    } else {
        buffer.push(curr);
        None
    }
}

fn push_many<'a, F: FiringXyz, I>(
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
