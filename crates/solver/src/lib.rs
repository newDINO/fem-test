pub mod assemble;
pub mod bc;
pub mod dof;
pub mod element;
pub mod error;
pub mod material;
pub mod mesh;
pub mod result;
pub mod solve;

pub use bc::BoundaryConditions;
pub use dof::{DofId, DofMap};
pub use error::FemError;
pub use fem_types;
pub use fem_types::model::{MaterialAssignment, MaterialAssignmentError, Model};
pub use material::{IsotropicMaterial, Material, MaterialExt, MaterialId, PlaneModel};
pub use mesh::{Bounds2, ElemId, Mesh, MeshError, Node, NodeId};
pub use result::{ElementStress, SolveResult};
pub use solve::solve;
