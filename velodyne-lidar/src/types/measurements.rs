use measurements::Length;

/// Point in strongest or last return mode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Measurement {
    pub distance: Length,
    pub intensity: u8,
    pub xyz: [Length; 3],
}

/// Point in strongest or last return mode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeasurementDual {
    pub strongest: Measurement,
    pub last: Measurement,
}

/// Point in strongest or last return mode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MeasurementKind {
    Single(Measurement),
    Dual(MeasurementDual),
}

impl From<Measurement> for MeasurementKind {
    fn from(from: Measurement) -> Self {
        Self::Single(from)
    }
}

impl From<MeasurementDual> for MeasurementKind {
    fn from(from: MeasurementDual) -> Self {
        Self::Dual(from)
    }
}
