use faer::sparse::Triplet;

use crate::dof::DofMap;
use crate::element::Tri3Kernel;
use crate::error::FemError;
use crate::material::Material;
use crate::mesh::Mesh;

pub struct Assembly {
    pub dof_map: DofMap,
    /// Full stiffness matrix entries. The matrix is stored as triplets rather
    /// than a sparse matrix so that constrained rows/columns can be filtered
    /// into a reduced system without mutating the matrix.
    pub triplets: Vec<Triplet<usize, usize, f64>>,
}

pub fn assemble(mesh: &Mesh, material: &Material) -> Result<Assembly, FemError> {
    mesh.validate()?;
    material.validate()?;

    let dof_map = DofMap::new(mesh.nodes.len());
    let mut triplets = Vec::with_capacity(mesh.triangles.len() * 36);

    for (element_index, tri) in mesh.triangles.iter().enumerate() {
        let coords = mesh.triangle_coords(*tri);
        let kernel = Tri3Kernel::new(element_index, coords, material)?;
        let k = kernel.stiffness(material);
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
