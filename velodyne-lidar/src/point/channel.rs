use crate::packet::Channel;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelD {
    pub strongest: Channel,
    pub last: Channel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelRefD<'a> {
    pub strongest: &'a Channel,
    pub last: &'a Channel,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChannelKind {
    Single(Channel),
    Dual(ChannelD),
}

impl ChannelKind {
    pub fn try_into_single(self) -> Result<Channel, Self> {
        if let Self::Single(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_dual(self) -> Result<ChannelD, Self> {
        if let Self::Dual(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn as_single(&self) -> Option<&Channel> {
        if let Self::Single(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_dual(&self) -> Option<&ChannelD> {
        if let Self::Dual(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<ChannelD> for ChannelKind {
    fn from(v: ChannelD) -> Self {
        Self::Dual(v)
    }
}

impl From<Channel> for ChannelKind {
    fn from(v: Channel) -> Self {
        Self::Single(v)
    }
}
