pub use fem_types::mesh::{Bounds2, ElemId, Mesh, Node, NodeId, signed_area2};

use crate::error::FemError;

/// Solver-side validation. The geometry itself lives in `fem-types` so that
/// external mesh generators can use it without pulling in the solver's
/// dependencies; validation reports `FemError`, which is a solver concern.
pub trait MeshExt {
    fn validate(&self) -> Result<(), FemError>;
}

impl MeshExt for Mesh {
    fn validate(&self) -> Result<(), FemError> {
        if self.nodes.is_empty() || self.triangles.is_empty() {
            return Err(FemError::EmptyMesh);
        }

        for node in &self.nodes {
            if !node.position.iter().all(|v| v.is_finite()) {
                return Err(FemError::InvalidInput(
                    "mesh contains a non-finite coordinate".to_owned(),
                ));
            }
        }

        let diag = self.bounds().map_or(1.0, Bounds2::diagonal).max(1e-12);
        let area_tol = 1e-12 * diag * diag;

        for (i, tri) in self.triangles.iter().enumerate() {
            if tri.iter().any(|n| n.index() >= self.nodes.len()) {
                return Err(FemError::InvalidInput(format!(
                    "triangle {i} references a node outside the mesh"
                )));
            }
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
                return Err(FemError::DegenerateElement(i));
            }
            let [a, b, c] = self.triangle_coords(*tri);
            let area2 = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
            if area2.abs() * 0.5 <= area_tol {
                return Err(FemError::DegenerateElement(i));
            }
        }

        Ok(())
    }
}
