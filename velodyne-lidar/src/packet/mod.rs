mod data;
pub use data::*;

mod position;
pub use position::*;

mod generic;
pub use generic::*;

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem;

    #[test]
    fn velodyne_packet_size_test() {
        assert_eq!(mem::size_of::<DataPacket>(), 1206);
        assert_eq!(mem::size_of::<PositionPacket>(), 512);
    }
}
