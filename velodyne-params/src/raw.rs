use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawVelodyneParams {
    pub num_lasers: usize,
    pub distance_resolution: f64,
    pub lasers: Vec<RawLaser>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawLaser {
    pub dist_correction: f64,
    pub dist_correction_x: f64,
    pub dist_correction_y: f64,
    pub focal_distance: f64,
    pub focal_slope: f64,
    pub horiz_offset_correction: f64,
    pub laser_id: u32,
    pub rot_correction: f64,
    pub vert_correction: f64,
    pub vert_offset_correction: f64,
}

#[cfg(test)]
mod tests {
    use super::RawVelodyneParams;
    use std::fs;

    #[test]
    fn deserialize_test() {
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/params");

        let yaml_paths = fs::read_dir(dir).unwrap().filter_map(|entry| {
            let entry = entry.unwrap();
            if entry.file_type().unwrap().is_dir() {
                return None;
            }

            let path = entry.path();
            if path.extension()? != "yaml" {
                return None;
            }

            Some(path)
        });

        yaml_paths.for_each(|path| {
            let text = fs::read_to_string(&path).unwrap();
            let _: RawVelodyneParams = serde_yaml::from_str(&text).unwrap_or_else(|err| {
                panic!("Unable to load this file '{}':\n{}", path.display(), err)
            });
        });
    }
}
