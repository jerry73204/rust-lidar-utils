use std::io;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("I/O error: {0}")]
    IO(io::Error),
    #[error("Invalid parameters: {0}")]
    InvalidParams(String),
}

impl Error {
    pub fn invalid_params(desc: impl Into<String>) -> Self {
        Self::InvalidParams(desc.into())
    }
}
