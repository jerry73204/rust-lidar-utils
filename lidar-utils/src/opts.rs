use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt)]
pub enum Opts {
    PcapToPcd(Convert),
}

#[derive(StructOpt)]
pub struct Convert {
    pub input_file: PathBuf,
    pub output_dir: PathBuf,
    #[structopt(long)]
    pub parallel: bool,
}
