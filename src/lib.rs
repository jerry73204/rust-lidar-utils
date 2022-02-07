//! Provide data structures and utilities for LIDAR data.
//!
//! This crate supports the list of models.
//! - Ouster OS1
//! - Velodyne VLP-16
//! - Velodyne Puke Lite
//! - Velodyne Puke Hi-Res

#[cfg(feature = "pcap")]
extern crate pcap;

mod common;
pub mod ouster;
mod utils;
pub mod velodyne;
