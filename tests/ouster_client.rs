extern crate failure;
extern crate lidar_utils;

use failure::Fallible;
use lidar_utils::ouster::client::CommandClient;

// Uncomment to enable test
// #[test]
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
