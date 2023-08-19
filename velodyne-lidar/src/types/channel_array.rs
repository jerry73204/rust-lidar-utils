use crate::packet::Channel;

pub type ChannelArrayS<const LEN: usize> = [Channel; LEN];
pub type ChannelArraySRef<'a, const LEN: usize> = &'a [Channel; LEN];

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ChannelArrayD<const LEN: usize> {
    pub strongest: [Channel; LEN],
    pub last: [Channel; LEN],
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ChannelArrayDRef<'a, const LEN: usize> {
    pub strongest: &'a [Channel; LEN],
    pub last: &'a [Channel; LEN],
}
