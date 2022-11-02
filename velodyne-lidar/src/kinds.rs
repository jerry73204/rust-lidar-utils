use crate::{
    common::*,
    packet::{ProductID, ReturnMode},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Format {
    Single16,
    Single32,
    Dual16,
    Dual32,
}

impl Format {
    pub fn from_model(product_id: ProductID, return_mode: ReturnMode) -> Option<Format> {
        use Format::*;
        use ProductID::*;
        use ReturnMode::*;

        Some(match (product_id, return_mode) {
            (HDL32E | VLP32C, Strongest | Last) => Single32,
            (HDL32E | VLP32C, Dual) => Dual32,
            (VLP16 | PuckLite | PuckHiRes, Strongest | Last) => Single16,
            (VLP16 | PuckLite | PuckHiRes, Dual) => Dual16,
            (Velarray, Strongest | Last) => return None,
            (Velarray, Dual) => return None,
            (VLS128, Strongest | Last) => return None,
            (VLS128, Dual) => return None,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FormatKind<S16, S32, D16, D32> {
    Single16(S16),
    Single32(S32),
    Dual16(D16),
    Dual32(D32),
}
