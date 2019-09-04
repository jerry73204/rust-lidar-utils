extern crate failure;
extern crate lidar_buffer;
// extern crate pcap;
// extern crate serde_json;
// #[macro_use]
// extern crate log;
// extern crate pretty_env_logger;

use failure::Fallible;
use lidar_buffer::ouster::CommandClient;

#[test]
fn ouster_client() -> Fallible<()> {
    let addr = "10.42.0.243";

    let mut client = CommandClient::connect((addr, 7501))?;

    client.set_udp_ip(addr.parse()?)?;
    client.set_udp_port_lidar(7777)?;

    let config_txt = client.get_config_txt()?;
    let beam_intrinsics = client.get_beam_intrinsics()?;

    dbg!(config_txt);
    dbg!(beam_intrinsics);

    Ok(())
}
