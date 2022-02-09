mod opts;
mod pcap_to_pcd;

use anyhow::Result;
use opts::Opts;
use structopt::StructOpt;

fn main() -> Result<()> {
    let opts = Opts::from_args();

    match opts {
        Opts::PcapToPcd(args) => pcap_to_pcd::pcap_to_pcd(args)?,
    }

    Ok(())
}
