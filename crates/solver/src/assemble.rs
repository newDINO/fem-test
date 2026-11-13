use faer::sparse::Triplet;
use fem_types::model::Model;

use crate::dof::DofMap;
use crate::element::Tri3Kernel;
use crate::error::FemError;
use crate::material::MaterialExt;
use crate::mesh::{ElemId, MeshExt};

pub struct Assembly {
    pub dof_map: DofMap,
    /// Full stiffness matrix entries. The matrix is stored as triplets rather
    /// than a sparse matrix so that constrained rows/columns can be filtered
    /// into a reduced system without mutating the matrix.
    pub triplets: Vec<Triplet<usize, usize, f64>>,
}

pub fn assemble(model: &Model) -> Result<Assembly, FemError> {
    let mesh = &model.mesh;
    mesh.validate()?;

    model
        .validate_materials()
        .map_err(|error| FemError::InvalidMaterial(error.to_string()))?;
    for material in &model.materials {
        material.validate(model.plane_model)?;
    }

    let dof_map = DofMap::new(mesh.nodes.len());
    let mut triplets = Vec::with_capacity(mesh.triangles.len() * 36);

    for (element_index, tri) in mesh.triangles.iter().enumerate() {
        let material = model
            .material_for(ElemId(element_index as u32))
            .ok_or_else(|| {
                FemError::InvalidMaterial(format!(
                    "element {element_index} has no material assignment"
                ))
            })?;
        let coords = mesh.triangle_coords(*tri);
        let kernel = Tri3Kernel::new(element_index, coords, material, model.plane_model)?;
        let k = kernel.stiffness();
        let dofs = [
            dof_map.dof(tri[0], 0),
            dof_map.dof(tri[0], 1),
            dof_map.dof(tri[1], 0),
            dof_map.dof(tri[1], 1),
            dof_map.dof(tri[2], 0),
            dof_map.dof(tri[2], 1),
        ];

        for i in 0..6 {
            for j in 0..6 {
                let value = k[(i, j)];
                if value != 0.0 {
                    triplets.push(Triplet::new(dofs[i].index(), dofs[j].index(), value));
                }
            }
        }
    }

    Ok(Assembly { dof_map, triplets })
}
