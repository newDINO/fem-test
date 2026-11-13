use fem_solver::{
    BoundaryConditions, DofId, DofMap, Material, MaterialExt, Mesh, Model, Node, NodeId,
    PlaneModel, SolveResult, solve,
};
use fem_types::signed_area2;

fn rectangle_mesh(nx: usize, ny: usize, width: f64, height: f64) -> Mesh {
    let mut nodes = Vec::new();
    for j in 0..=ny {
        for i in 0..=nx {
            nodes.push(Node {
                position: [
                    width * i as f64 / nx as f64,
                    height * j as f64 / ny as f64,
                    0.0,
                ],
            });
        }
    }
    let node = |i: usize, j: usize| NodeId((j * (nx + 1) + i) as u32);
    let mut triangles = Vec::new();
    for j in 0..ny {
        for i in 0..nx {
            let a = node(i, j);
            let b = node(i + 1, j);
            let c = node(i + 1, j + 1);
            let d = node(i, j + 1);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { nodes, triangles }
}

fn node(nx: usize, i: usize, j: usize) -> NodeId {
    NodeId((j * (nx + 1) + i) as u32)
}

/// Periodic links for a rectangular cell. The right boundary is a slave of the
/// left boundary and the top boundary is a slave of the bottom boundary.
/// Corners appear in both sets on purpose: the redundant relations must be
/// recognized as consistent by the constraint resolver.
fn periodic_bc(
    mesh: &Mesh,
    nx: usize,
    ny: usize,
    width: f64,
    height: f64,
    strain: [f64; 3],
) -> BoundaryConditions {
    let [exx, eyy, gxy] = strain;
    let exy = 0.5 * gxy;
    let dofs = DofMap::new(mesh.nodes.len());
    let mut bc = BoundaryConditions::default();

    for j in 0..=ny {
        let slave = node(nx, nx, j);
        let master = node(nx, 0, j);
        bc.add_link(dofs.dof(slave, 0), dofs.dof(master, 0), exx * width);
        bc.add_link(dofs.dof(slave, 1), dofs.dof(master, 1), exy * width);
    }

    for i in 0..=nx {
        let slave = node(nx, i, ny);
        let master = node(nx, i, 0);
        bc.add_link(dofs.dof(slave, 0), dofs.dof(master, 0), exy * height);
        bc.add_link(dofs.dof(slave, 1), dofs.dof(master, 1), eyy * height);
    }

    // Remove the remaining translation gauge. The origin is a corner, so its
    // affine displacement is zero for every macro strain.
    let origin = node(nx, 0, 0);
    bc.set_prescribed(dofs.dof(origin, 0), 0.0);
    bc.set_prescribed(dofs.dof(origin, 1), 0.0);

    bc
}

fn average_stress(model: &Model, result: &SolveResult) -> [f64; 3] {
    let mut total_area = 0.0;
    let mut average = [0.0; 3];

    for (index, triangle) in model.mesh.triangles.iter().enumerate() {
        let [a, b, c] = model.mesh.triangle_coords(*triangle);
        let area = signed_area2(a, b, c).abs();
        total_area += area;
        for component in 0..3 {
            average[component] += area * result.stresses[index].stress[component];
        }
    }

    for value in &mut average {
        *value /= total_area;
    }
    average
}

#[test]
fn periodic_patch_test_reproduces_uniform_strain() {
    let (nx, ny) = (4, 3);
    let (width, height) = (2.0, 1.0);
    let model = Model::uniform(
        rectangle_mesh(nx, ny, width, height),
        PlaneModel::PlaneStress,
        Material::isotropic(1000.0, 0.3, 1.0),
    );
    let strain = [1e-3, -4e-4, 2e-4];
    let bc = periodic_bc(&model.mesh, nx, ny, width, height, strain);

    let result = solve(&model, &bc).unwrap();
    let d = model.materials[0].d_matrix(model.plane_model);
    let expected = d * nalgebra::Vector3::new(strain[0], strain[1], strain[2]);

    for stress in &result.stresses {
        for component in 0..3 {
            assert!(
                (stress.stress[component] - expected[component]).abs() < 1e-9,
                "element {} component {}: got {}, expected {}",
                stress.element.0,
                component,
                stress.stress[component],
                expected[component]
            );
        }
    }

    assert!(
        result.max_residual < 1e-9,
        "residual {} is too large",
        result.max_residual
    );

    for link in &bc.links {
        let actual = result.displacements[link.slave.index()]
            - result.displacements[link.master.index()];
        assert!(
            (actual - link.offset).abs() < 1e-12,
            "link {:?} -> {:?}: got jump {actual}, expected {}",
            link.slave,
            link.master,
            link.offset
        );
    }
}

#[test]
fn homogeneous_periodic_cell_reproduces_d_matrix() {
    let (nx, ny) = (3, 2);
    let (width, height) = (2.0, 1.0);
    let model = Model::uniform(
        rectangle_mesh(nx, ny, width, height),
        PlaneModel::PlaneStress,
        Material::isotropic(1000.0, 0.3, 1.0),
    );
    let d = model.materials[0].d_matrix(model.plane_model);

    let unit_strains = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut effective = [[0.0; 3]; 3];

    for (column, strain) in unit_strains.iter().enumerate() {
        let bc = periodic_bc(&model.mesh, nx, ny, width, height, *strain);
        let result = solve(&model, &bc).unwrap();
        let average = average_stress(&model, &result);
        for row in 0..3 {
            effective[row][column] = average[row];
        }
    }

    for row in 0..3 {
        for column in 0..3 {
            let expected = d[(row, column)];
            assert!(
                (effective[row][column] - expected).abs()
                    < 1e-7 * expected.abs().max(1.0),
                "C[{row}][{column}]: got {}, expected {expected}",
                effective[row][column]
            );
        }
    }
}

#[test]
fn inconsistent_corner_offsets_are_rejected() {
    let (nx, ny) = (2, 2);
    let (width, height) = (1.0, 1.0);
    let model = Model::uniform(
        rectangle_mesh(nx, ny, width, height),
        PlaneModel::PlaneStress,
        Material::isotropic(1000.0, 0.3, 1.0),
    );
    let mut bc = periodic_bc(&model.mesh, nx, ny, width, height, [1e-3, 2e-3, 3e-4]);

    // Break the cycle at the top-right corner: the x relation to the bottom
    // right corner must agree with the path through the other two corners.
    let top_right_x = DofId(node(nx, nx, ny).0 * 2);
    let bottom_right_x = DofId(node(nx, nx, 0).0 * 2);
    let link = bc
        .links
        .iter_mut()
        .find(|link| link.slave == top_right_x && link.master == bottom_right_x)
        .unwrap();
    link.offset += 1e-3;

    assert!(matches!(
        solve(&model, &bc).unwrap_err(),
        fem_solver::FemError::ConstraintConflict(_)
    ));
}

#[test]
fn chain_of_links_is_applied_by_the_solver() {
    let mesh = rectangle_mesh(2, 1, 2.0, 1.0);
    let model = Model::uniform(
        mesh,
        PlaneModel::PlaneStress,
        Material::isotropic(1000.0, 0.3, 1.0),
    );
    let dofs = DofMap::new(model.mesh.nodes.len());

    let mut bc = BoundaryConditions::default();
    // Node 1 follows node 0, node 2 follows node 1. Node 3 is the gauge.
    bc.add_link(dofs.dof(NodeId(1), 0), dofs.dof(NodeId(0), 0), 0.1);
    bc.add_link(dofs.dof(NodeId(2), 0), dofs.dof(NodeId(1), 0), 0.2);
    for node in [NodeId(0), NodeId(3), NodeId(4), NodeId(5)] {
        bc.set_prescribed(dofs.dof(node, 0), 0.0);
        bc.set_prescribed(dofs.dof(node, 1), 0.0);
    }

    let result = solve(&model, &bc).unwrap();
    let ux = |node: NodeId| result.displacements[dofs.dof(node, 0).index()];
    assert!((ux(NodeId(1)) - ux(NodeId(0)) - 0.1).abs() < 1e-12);
    assert!((ux(NodeId(2)) - ux(NodeId(1)) - 0.2).abs() < 1e-12);
}

// Keep the `Constraint` import used when the test module grows; this also
// documents that links can be built directly through the solver type.
#[allow(dead_code)]
fn direct_link() -> Constraint {
    Constraint::new(DofId(1), DofId(0), 0.0)
}
