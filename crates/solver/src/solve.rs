use faer::linalg::solvers::Solve;
use faer::sparse::{SparseColMat, Triplet};
use fem_types::model::Model;
use nalgebra::SVector;

use crate::assemble::assemble;
use crate::bc::BoundaryConditions;
use crate::element::Tri3Kernel;
use crate::error::FemError;
use crate::mesh::ElemId;
use crate::result::{ElementStress, SolveResult, von_mises_stress};

pub fn solve(model: &Model, bc: &BoundaryConditions) -> Result<SolveResult, FemError> {
    let mesh = &model.mesh;
    let assembly = assemble(model)?;
    let dof_map = &assembly.dof_map;
    let ndof = dof_map.ndof();
    let external_force = bc.force_vector(dof_map);

    for (&dof, &value) in &bc.forces {
        if dof.index() >= ndof {
            return Err(FemError::InvalidInput(
                "boundary condition references a DOF outside the mesh".to_owned(),
            ));
        }
        if !value.is_finite() {
            return Err(FemError::InvalidInput(
                "force contains a non-finite value".to_owned(),
            ));
        }
    }

    let mut prescribed = vec![None; ndof];
    for (&dof, &value) in &bc.prescribed {
        if dof.index() >= ndof {
            return Err(FemError::InvalidInput(
                "boundary condition references a DOF outside the mesh".to_owned(),
            ));
        }
        if !value.is_finite() {
            return Err(FemError::InvalidInput(
                "prescribed displacement contains a non-finite value".to_owned(),
            ));
        }
        prescribed[dof.index()] = Some(value);
    }

    let constrained_count = prescribed.iter().filter(|v| v.is_some()).count();
    if constrained_count < 3 {
        return Err(FemError::InsufficientConstraints {
            count: constrained_count,
        });
    }

    let free_dofs: Vec<usize> = (0..ndof).filter(|&i| prescribed[i].is_none()).collect();
    let mut free_index = vec![None; ndof];
    for (free_i, &global_i) in free_dofs.iter().enumerate() {
        free_index[global_i] = Some(free_i);
    }

    let mut reduced_rhs = vec![0.0; free_dofs.len()];
    for (free_i, &global_i) in free_dofs.iter().enumerate() {
        reduced_rhs[free_i] = external_force[global_i];
    }

    let mut reduced_triplets = Vec::with_capacity(assembly.triplets.len());
    for entry in &assembly.triplets {
        let (i, j, value) = (entry.row, entry.col, entry.val);
        match (free_index[i], free_index[j]) {
            (Some(ri), Some(rj)) => {
                reduced_triplets.push(Triplet::new(ri, rj, value));
            }
            (Some(ri), None) => {
                // K_fc * u_c moves to the right-hand side.
                reduced_rhs[ri] -= value * prescribed[j].unwrap();
            }
            _ => {}
        }
    }

    let mut displacement = vec![0.0; ndof];
    for i in 0..ndof {
        if let Some(value) = prescribed[i] {
            displacement[i] = value;
        }
    }

    if !free_dofs.is_empty() {
        let nfree = free_dofs.len();
        let reduced =
            SparseColMat::<usize, f64>::try_new_from_triplets(nfree, nfree, &reduced_triplets)
                .map_err(|e| FemError::SingularMatrix(format!("{e:?}")))?;

        let lu = reduced
            .sp_lu()
            .map_err(|e| FemError::SingularMatrix(format!("{e:?}")))?;

        let rhs = faer::Mat::<f64>::from_fn(nfree, 1, |i, _| reduced_rhs[i]);
        let solution = lu.solve(rhs);

        for (free_i, &global_i) in free_dofs.iter().enumerate() {
            let value = solution[(free_i, 0)];
            if !value.is_finite() {
                return Err(FemError::SingularMatrix(
                    "solution contains a non-finite value".to_owned(),
                ));
            }
            displacement[global_i] = value;
        }
    }

    // r = K u - f_ext. This uses the full stiffness matrix, including rows and
    // columns belonging to constrained DOFs.
    let mut reactions = vec![0.0; ndof];
    for entry in &assembly.triplets {
        reactions[entry.row] += entry.val * displacement[entry.col];
    }
    for i in 0..ndof {
        reactions[i] -= external_force[i];
    }

    let max_residual = free_dofs
        .iter()
        .map(|&i| reactions[i].abs())
        .fold(0.0_f64, f64::max);

    let mut stresses = Vec::with_capacity(mesh.triangles.len());
    let mut max_displacement = 0.0_f64;
    for (element_index, tri) in mesh.triangles.iter().enumerate() {
        let material = model
            .material_for(ElemId(element_index as u32))
            .ok_or_else(|| {
                FemError::InvalidMaterial(format!(
                    "element {element_index} has no material assignment"
                ))
            })?;
        let kernel = Tri3Kernel::new(
            element_index,
            mesh.triangle_coords(*tri),
            material,
            model.plane_model,
        )?;
        let ue = SVector::<f64, 6>::new(
            displacement[dof_map.dof(tri[0], 0).index()],
            displacement[dof_map.dof(tri[0], 1).index()],
            displacement[dof_map.dof(tri[1], 0).index()],
            displacement[dof_map.dof(tri[1], 1).index()],
            displacement[dof_map.dof(tri[2], 0).index()],
            displacement[dof_map.dof(tri[2], 1).index()],
        );
        let stress = kernel.stress(&ue);
        let (sx, sy, txy) = (stress[0], stress[1], stress[2]);
        let sz = material.sigma_z(model.plane_model, sx, sy);
        let von_mises = von_mises_stress(sx, sy, txy, sz);
        stresses.push(ElementStress {
            element: ElemId(element_index as u32),
            stress: [sx, sy, txy],
            sigma_z: sz,
            von_mises,
        });
    }

    for node_index in 0..mesh.nodes.len() {
        let ux = displacement[node_index * 2];
        let uy = displacement[node_index * 2 + 1];
        max_displacement = max_displacement.max(ux.hypot(uy));
    }

    Ok(SolveResult {
        displacements: displacement,
        reactions,
        stresses,
        max_displacement,
        max_residual,
    })
}
