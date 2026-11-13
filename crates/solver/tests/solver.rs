use fem_import::{DropAxis, StlImportOptions, load_stl};
use fem_solver::{
    BoundaryConditions, Material, MaterialAssignment, MaterialExt, MaterialId, Mesh, Model, Node,
    NodeId, PlaneModel, solve,
};

fn single_triangle() -> Mesh {
    Mesh {
        nodes: vec![
            Node {
                position: [0.0, 0.0, 0.0],
            },
            Node {
                position: [1.0, 0.0, 0.0],
            },
            Node {
                position: [0.0, 1.0, 0.0],
            },
        ],
        triangles: vec![[NodeId(0), NodeId(1), NodeId(2)]],
    }
}

fn uniform(mesh: Mesh, material: Material) -> Model {
    Model::uniform(mesh, PlaneModel::PlaneStress, material)
}

#[test]
fn patch_test_all_prescribed() {
    let model = uniform(single_triangle(), Material::isotropic(1000.0, 0.25, 1.0));
    let mut bc = BoundaryConditions::default();
    // u_x = 0.01*x, u_y = -0.005*y
    let displacement = [(0.0, 0.0), (0.01, 0.0), (0.0, -0.005)];
    for (i, (ux, uy)) in displacement.iter().enumerate() {
        let node = NodeId(i as u32);
        bc.prescribed.insert(fem_solver::DofId(node.0 * 2), *ux);
        bc.prescribed.insert(fem_solver::DofId(node.0 * 2 + 1), *uy);
    }

    let result = solve(&model, &bc).unwrap();
    let stress = result.stresses[0].stress;
    let d = model.materials[0].d_matrix(model.plane_model);
    let expected = d * nalgebra::Vector3::new(0.01, -0.005, 0.0);
    assert!((stress[0] - expected[0]).abs() < 1e-10);
    assert!((stress[1] - expected[1]).abs() < 1e-10);
    assert!((stress[2] - expected[2]).abs() < 1e-10);
}

#[test]
fn rigid_body_translation_has_zero_stress() {
    let model = uniform(single_triangle(), Material::default());
    let mut bc = BoundaryConditions::default();
    for i in 0..3 {
        let node = NodeId(i);
        bc.prescribed.insert(fem_solver::DofId(node.0 * 2), 1.0);
        bc.prescribed.insert(fem_solver::DofId(node.0 * 2 + 1), 2.0);
    }
    let result = solve(&model, &bc).unwrap();
    assert!(result.stresses[0].von_mises < 1e-9);
}

#[test]
fn insufficient_constraints_is_reported() {
    let model = uniform(single_triangle(), Material::default());
    let bc = BoundaryConditions::default();
    let error = solve(&model, &bc).unwrap_err();
    assert!(matches!(
        error,
        fem_solver::FemError::InsufficientConstraints { count: 0 }
    ));
}

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
    let idx = |i: usize, j: usize| NodeId((j * (nx + 1) + i) as u32);
    let mut triangles = Vec::new();
    for j in 0..ny {
        for i in 0..nx {
            let a = idx(i, j);
            let b = idx(i + 1, j);
            let c = idx(i + 1, j + 1);
            let d = idx(i, j + 1);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { nodes, triangles }
}

#[test]
fn cantilever_solves_and_balances_reactions() {
    let model = uniform(
        rectangle_mesh(8, 2, 10.0, 2.0),
        Material::isotropic(1000.0, 0.3, 1.0),
    );
    let mut bc = BoundaryConditions::default();
    for node in 0..=2 {
        let node = NodeId((node * 9) as u32); // left edge, j=0..2
        bc.prescribed.insert(fem_solver::DofId(node.0 * 2), 0.0);
        bc.prescribed.insert(fem_solver::DofId(node.0 * 2 + 1), 0.0);
    }
    let tip_nodes = [NodeId(8), NodeId(17), NodeId(26)];
    let force_per_node = -10.0 / tip_nodes.len() as f64;
    for node in tip_nodes {
        bc.forces
            .insert(fem_solver::DofId(node.0 * 2 + 1), force_per_node);
    }

    let result = solve(&model, &bc).unwrap();
    assert!(result.max_displacement > 0.0);
    assert!(result.max_residual < 1e-8);

    let total_y_reaction: f64 = (0..model.mesh.nodes.len())
        .map(|i| result.reactions[i * 2 + 1])
        .sum();
    assert!((total_y_reaction - 10.0).abs() < 1e-8);
}

#[test]
fn per_element_materials_use_their_own_d_matrix() {
    let mut model = uniform(
        rectangle_mesh(1, 1, 1.0, 1.0),
        Material::isotropic(1000.0, 0.25, 1.0),
    );
    model.materials.push(Material::isotropic(2000.0, 0.25, 1.0));
    model.element_materials = MaterialAssignment::PerElement(vec![MaterialId(0), MaterialId(1)]);

    let mut bc = BoundaryConditions::default();
    for (index, node) in model.mesh.nodes.iter().enumerate() {
        bc.prescribed
            .insert(fem_solver::DofId(index as u32 * 2), 0.01 * node.position[0]);
        bc.prescribed.insert(
            fem_solver::DofId(index as u32 * 2 + 1),
            -0.005 * node.position[1],
        );
    }

    let result = solve(&model, &bc).unwrap();
    let strain = nalgebra::Vector3::new(0.01, -0.005, 0.0);

    for stress in &result.stresses {
        let material_id = model.element_materials.get(stress.element).unwrap();
        let expected = model.materials[material_id.index()].d_matrix(model.plane_model) * strain;
        for component in 0..3 {
            assert!((stress.stress[component] - expected[component]).abs() < 1e-10);
        }
    }

    assert!(
        (result.stresses[0].stress[0] - result.stresses[1].stress[0]).abs() > 1e-6,
        "the two materials should produce different stresses"
    );
}

#[test]
fn rejects_invalid_material_assignments() {
    let bc = BoundaryConditions::default();

    let mut model = uniform(single_triangle(), Material::default());
    model.element_materials = MaterialAssignment::PerElement(Vec::new());
    assert!(matches!(
        solve(&model, &bc).unwrap_err(),
        fem_solver::FemError::InvalidMaterial(_)
    ));

    let mut model = uniform(single_triangle(), Material::default());
    model.element_materials = MaterialAssignment::PerElement(vec![MaterialId(1)]);
    assert!(matches!(
        solve(&model, &bc).unwrap_err(),
        fem_solver::FemError::InvalidMaterial(_)
    ));

    let mut model = uniform(single_triangle(), Material::default());
    model.element_materials = MaterialAssignment::Uniform(MaterialId(2));
    assert!(matches!(
        solve(&model, &bc).unwrap_err(),
        fem_solver::FemError::InvalidMaterial(_)
    ));

    let mut model = uniform(single_triangle(), Material::default());
    model.materials.clear();
    assert!(matches!(
        solve(&model, &bc).unwrap_err(),
        fem_solver::FemError::InvalidMaterial(_)
    ));
}

#[test]
fn loads_the_bundled_sample_plate() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/plate.stl");
    let (mesh, report) = load_stl(path, StlImportOptions::default()).unwrap();
    assert_eq!(report.drop_axis, DropAxis::Z);
    assert!(mesh.nodes.len() > 4);
    assert!(mesh.triangles.len() > 4);
    assert_eq!(mesh.triangles.len(), report.imported_triangles);

    // The sample should be solvable as a cantilever.
    let mut bc = BoundaryConditions::default();
    let mut left_nodes: Vec<NodeId> = mesh
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, node)| node.position[0] == 0.0)
        .map(|(index, _)| NodeId(index as u32))
        .collect();
    left_nodes.sort();
    assert!(!left_nodes.is_empty());
    for node in left_nodes {
        bc.set_prescribed(fem_solver::DofId(node.0 * 2), 0.0);
        bc.set_prescribed(fem_solver::DofId(node.0 * 2 + 1), 0.0);
    }
    let tip_right = mesh
        .nodes
        .iter()
        .map(|node| node.position[0])
        .fold(f64::NEG_INFINITY, f64::max);
    let tip_nodes: Vec<NodeId> = mesh
        .nodes
        .iter()
        .enumerate()
        .filter(|(_, node)| (node.position[0] - tip_right).abs() < 1e-9)
        .map(|(index, _)| NodeId(index as u32))
        .collect();
    for node in &tip_nodes {
        bc.set_force(fem_solver::DofId(node.0 * 2 + 1), -1.0);
    }

    let model = Model::uniform(mesh, PlaneModel::PlaneStress, Material::default());
    let result = solve(&model, &bc).unwrap();
    assert!(result.max_displacement > 0.0);
    assert!(result.max_residual < 1e-9);
}
