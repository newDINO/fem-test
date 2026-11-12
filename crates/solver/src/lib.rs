pub mod assemble;
pub mod bc;
pub mod dof;
pub mod element;
pub mod error;
pub mod io;
pub mod material;
pub mod mesh;
pub mod result;
pub mod solve;

pub use bc::BoundaryConditions;
pub use dof::{DofId, DofMap};
pub use error::FemError;
pub use io::stl::{DropAxis, ImportReport, StlImportOptions, import_stl_mesh, load_stl};
pub use material::{Material, PlaneModel};
pub use mesh::{Bounds2, ElemId, Mesh, Node, NodeId};
pub use result::{ElementStress, SolveResult};
pub use solve::solve;
