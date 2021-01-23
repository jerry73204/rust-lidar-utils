use crate::{
    common::*,
    velodyne::{
        config::Config,
        marker::{ModelMarker, ReturnTypeMarker},
    },
};

#[derive(Debug, Clone)]
pub struct FrameConverter<MODEL, RETURN_TYPE>
where
    MODEL: ModelMarker,
    RETURN_TYPE: ReturnTypeMarker,
{
    lasers: MODEL::ParamArray,
    return_type: RETURN_TYPE,
}

impl<MODEL, RETURN_TYPE> From<Config<MODEL, RETURN_TYPE>> for FrameConverter<MODEL, RETURN_TYPE>
where
    MODEL: ModelMarker,
    RETURN_TYPE: ReturnTypeMarker,
{
    fn from(config: Config<MODEL, RETURN_TYPE>) -> Self {
        let Config {
            lasers,
            distance_resolution,
            return_type,
            ..
        } = config;

        Self {
            lasers,
            return_type,
        }
    }
}

impl<MODEL, RETURN_TYPE> FrameConverter<MODEL, RETURN_TYPE>
where
    MODEL: ModelMarker,
    RETURN_TYPE: ReturnTypeMarker,
{
}
