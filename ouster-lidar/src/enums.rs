//! Useful enums for Ouster sensors.

use serde::{Deserialize, Serialize};
use std::fmt::{self, Display, Formatter};

/// The mode includes number of vertical scans in one revolution and rotation frequency (Hz).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LidarMode {
    #[serde(rename = "512x10")]
    Mode512x10,
    #[serde(rename = "512x20")]
    Mode512x20,
    #[serde(rename = "1024x10")]
    Mode1024x10,
    #[serde(rename = "1024x20")]
    Mode1024x20,
    #[serde(rename = "2048x10")]
    Mode2048x10,
}

impl LidarMode {
    pub fn columns_per_revolution(&self) -> u16 {
        use LidarMode::*;
        match self {
            Mode512x10 | Mode512x20 => 512,
            Mode1024x10 | Mode1024x20 => 1024,
            Mode2048x10 => 2048,
        }
    }
}

impl Display for LidarMode {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        use LidarMode::*;
        let text = match self {
            Mode512x10 => "512x10",
            Mode512x20 => "512x20",
            Mode1024x10 => "1024x10",
            Mode1024x20 => "1024x20",
            Mode2048x10 => "2048x10",
        };
        write!(formatter, "{}", text)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MultipurposeIoMode {
    #[serde(rename = "OUTPUT_FROM_INTERNAL_OSC")]
    OutputFromInternalOsc,
    #[serde(rename = "OUTPUT_FROM_SYNC_PULSE_IN")]
    OutputFromSyncPulseIn,
    #[serde(rename = "OUTPUT_FROM_PTP_1588")]
    OutputFromPtp1588,
    #[serde(rename = "OFF")]
    Off,
}

impl Display for MultipurposeIoMode {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        use MultipurposeIoMode::*;
        let text = match self {
            OutputFromInternalOsc => "OUTPUT_FROM_INTERNAL_OSC",
            OutputFromSyncPulseIn => "OUTPUT_FROM_SYNC_PULSE_IN",
            OutputFromPtp1588 => "OUTPUT_FROM_PTP1588",
            Off => "OFF",
        };
        write!(formatter, "{}", text)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TimestampMode {
    #[serde(rename = "TIME_FROM_INTERNAL_OSC")]
    TimeFromInternalOsc,
    #[serde(rename = "TIME_FROM_PTP_1588")]
    TimeFromPtp1588,
    #[serde(rename = "TIME_FROM_SYNC_PULSE_IN")]
    TimeFromSyncPulseIn,
}

impl Display for TimestampMode {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        use TimestampMode::*;
        let text = match self {
            TimeFromInternalOsc => "TIME_FROM_INTERNAL_OSC",
            TimeFromPtp1588 => "TIME_FROM_PTP1588",
            TimeFromSyncPulseIn => "TIME_FROM_SYNC_PULSE_IN",
        };
        write!(formatter, "{}", text)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Polarity {
    #[serde(rename = "ACTIVE_HIGH")]
    ActiveHigh,
    #[serde(rename = "ACTIVE_LOW")]
    ActiveLow,
}

impl Display for Polarity {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        use Polarity::*;
        let text = match self {
            ActiveHigh => "ACTIVE_HIGH",
            ActiveLow => "ACTIVE_LOW",
        };
        write!(formatter, "{}", text)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum OnOffMode {
    #[serde(rename = "ON")]
    On,
    #[serde(rename = "OFF")]
    Off,
}

impl Display for OnOffMode {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        use OnOffMode::*;
        let text = match self {
            On => "ON",
            Off => "OFF",
        };
        write!(formatter, "{}", text)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NmeaBaudRate {
    #[serde(rename = "BAUD_9600")]
    Baud9600,
    #[serde(rename = "BAUD_115200")]
    Baud115200,
}

impl Display for NmeaBaudRate {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        use NmeaBaudRate::*;
        let text = match self {
            Baud9600 => "BAUD_9600",
            Baud115200 => "BAUD_115200",
        };
        write!(formatter, "{}", text)
    }
}
