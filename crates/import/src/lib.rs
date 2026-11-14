//! Mesh importers for the FEM workspace.
//!
//! This crate only depends on `fem-types` (plus the format-specific parser), so
//! tools that generate meshes can use it without pulling in the solver's
//! linear algebra stack. The project file format lives in `fem-types::format`.

use fem_types::MeshError;
use thiserror::Error;

pub mod stl;

#[cfg(not(target_family = "wasm"))]
pub use stl::load_stl;
pub use stl::{DropAxis, ImportReport, StlImportOptions, import_stl_mesh, load_stl_bytes};

#[derive(Debug, Error)]
pub enum ImportError {
    #[error("STL import failed: {0}")]
    Stl(String),
    #[error("mesh is empty")]
    EmptyMesh,
    #[error("mesh is not a single axis-aligned plane: {0}")]
    NonPlanar(String),
    #[error("invalid input: {0}")]
    InvalidInput(String),
    #[error(transparent)]
    Mesh(#[from] MeshError),
}
