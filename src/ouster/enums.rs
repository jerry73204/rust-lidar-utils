//! Useful enums for Ouster sensors.

use serde::{Deserialize, Serialize};
use std::fmt::{Debug, Display, Formatter, Result as FormatResult};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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

impl Display for LidarMode {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum TimestampMode {
    #[serde(rename = "TIME_FROM_INTERNAL_OSC")]
    TimeFromInternalOsc,
    #[serde(rename = "TIME_FROM_PTP_1588")]
    TimeFromPtp1588,
    #[serde(rename = "TIME_FROM_SYNC_PULSE_IN")]
    TimeFromSyncPulseIn,
}

impl Display for TimestampMode {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        use TimestampMode::*;
        let text = match self {
            TimeFromInternalOsc => "TIME_FROM_INTERNAL_OSC",
            TimeFromPtp1588 => "TIME_FROM_PTP1588",
            TimeFromSyncPulseIn => "TIME_FROM_SYNC_PULSE_IN",
        };
        write!(formatter, "{}", text)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Polarity {
    #[serde(rename = "ACTIVE_HIGH")]
    ActiveHigh,
    #[serde(rename = "ACTIVE_LOW")]
    ActiveLow,
}

impl Display for Polarity {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> FormatResult {
        use Polarity::*;
        let text = match self {
            ActiveHigh => "ACTIVE_HIGH",
            ActiveLow => "ACTIVE_LOW",
        };
        write!(formatter, "{}", text)
    }
}
