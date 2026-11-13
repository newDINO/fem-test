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

    if bc.prescribed.is_empty() && bc.links.is_empty() {
        return Err(FemError::InsufficientConstraints { count: 0 });
    }

    // Prescribed displacements are links to a virtual ground DOF, so both
    // fixed and periodic DOFs go through the same reduction path.
    let relations = crate::constraints::resolve(dof_map, &bc.prescribed, &bc.links)?;
    let free_count = relations.free_count();

    let mut reduced_rhs = vec![0.0; free_count];
    for i in 0..ndof {
        if let Some(ri) = relations.reduced_index[i] {
            reduced_rhs[ri] += external_force[i];
        }
    }

    let mut reduced_triplets = Vec::with_capacity(assembly.triplets.len());
    for entry in &assembly.triplets {
        let (i, j, value) = (entry.row, entry.col, entry.val);

        // Only rows that survive the transformation contribute. The constant
        // part of u_j (prescribed value or periodic offset) moves to the RHS.
        let Some(ri) = relations.reduced_index[i] else {
            continue;
        };
        if let Some(rj) = relations.reduced_index[j] {
            reduced_triplets.push(Triplet::new(ri, rj, value));
        }
        reduced_rhs[ri] -= value * relations.constant[j];
    }

    let mut displacement = relations.constant.clone();

    if free_count > 0 {
        let reduced = SparseColMat::<usize, f64>::try_new_from_triplets(
            free_count,
            free_count,
            &reduced_triplets,
        )
        .map_err(|e| FemError::SingularMatrix(format!("{e:?}")))?;

        let lu = reduced.sp_lu().map_err(|e| {
            FemError::SingularMatrix(format!(
                "{e:?}; the model may have unconstrained rigid-body modes or a mechanism"
            ))
        })?;

        let rhs = faer::Mat::<f64>::from_fn(free_count, 1, |i, _| reduced_rhs[i]);
        let solution = lu.solve(rhs);

        for i in 0..ndof {
            if let Some(ri) = relations.reduced_index[i] {
                let value = solution[(ri, 0)];
                if !value.is_finite() {
                    return Err(FemError::SingularMatrix(
                        "solution contains a non-finite value".to_owned(),
                    ));
                }
                displacement[i] = value + relations.constant[i];
            }
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

    // Check equilibrium on the independent (reduced) DOFs. A single master row
    // is not balanced by itself: the constraint forces of its slaves are
    // transferred to it, so the reactions of the whole tied set must cancel.
    let mut reduced_reactions = vec![0.0; free_count];
    for i in 0..ndof {
        if let Some(ri) = relations.reduced_index[i] {
            reduced_reactions[ri] += reactions[i];
        }
    }
    let max_residual = reduced_reactions
        .iter()
        .map(|value| value.abs())
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
