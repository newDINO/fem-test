//! Shared, dependency-light data types for the FEM file format.
//!
//! This crate deliberately does not depend on the solver's linear algebra
//! stack (`faer`, `nalgebra`). External tools that only generate meshes or read
//! results can depend on `fem-types` alone.
//!
//! The on-disk format lives in [`format`] and is enabled by default. It uses
//! MessagePack (via `rmp-serde`) with named struct fields.

pub mod material;
pub mod mesh;
pub mod result;

#[cfg(feature = "format")]
pub mod format;

pub use material::{Material, PlaneModel};
pub use mesh::{Bounds2, ElemId, Mesh, Node, NodeId, signed_area2};
pub use result::{ElementStress, SolveResult, von_mises_stress};
