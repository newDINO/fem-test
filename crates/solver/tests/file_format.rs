use fem_solver::{
    BoundaryConditions, DofId, FemError, Material, MaterialAssignment, MaterialId, Model, Node,
    NodeId, PlaneModel, solve,
};
use fem_types::Mesh;
use fem_types::format::{self, Boundary, DofValue, Results};

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

fn encode(file: &format::File) -> Vec<u8> {
    let mut bytes = Vec::new();
    format::write(&mut bytes, file).unwrap();
    bytes
}

#[test]
fn mesh_file_roundtrip_solves() {
    let mesh = rectangle_mesh(2, 1, 2.0, 1.0);
    let model = Model::uniform(mesh, PlaneModel::PlaneStress, Material::default());
    let file = format::File::from_model(model);
    let loaded = format::read_bytes(&encode(&file)).unwrap();
    let loaded_model = loaded.model().unwrap().unwrap();

    loaded_model.mesh.validate().unwrap();
    loaded_model.validate_materials().unwrap();

    let mut bc = BoundaryConditions::default();
    for node in [NodeId(0), NodeId(3)] {
        bc.set_prescribed(DofId(node.0 * 2), 0.0);
        bc.set_prescribed(DofId(node.0 * 2 + 1), 0.0);
    }
    bc.set_force(DofId(5 * 2 + 1), -1.0);

    let result = solve(&loaded_model, &bc).unwrap();
    assert!(result.max_displacement > 0.0);

    let result_file = format::File::from_model(loaded_model)
        .with_boundary(bc.to_file())
        .with_results(Results::from_solve(&result));

    let decoded = format::read_bytes(&encode(&result_file)).unwrap();
    let decoded_model = decoded.model().unwrap().unwrap();
    let rebuilt = decoded
        .results
        .as_ref()
        .unwrap()
        .to_solve(&decoded_model)
        .unwrap();

    assert!((rebuilt.max_displacement - result.max_displacement).abs() < 1e-15);
    assert_eq!(rebuilt.stresses.len(), result.stresses.len());
    assert_eq!(
        decoded.boundary.map(|b| b.prescribed.len()),
        Some(bc.prescribed.len())
    );
}

#[test]
fn mesh_only_file_has_no_model() {
    let file = format::File::new(rectangle_mesh(1, 1, 1.0, 1.0));
    let decoded = format::read_bytes(&encode(&file)).unwrap();
    assert!(decoded.model().unwrap().is_none());
}

#[test]
fn per_element_assignment_roundtrip() {
    let mesh = rectangle_mesh(1, 1, 1.0, 1.0);
    let mut model = Model::uniform(
        mesh,
        PlaneModel::PlaneStrain,
        Material::isotropic(1000.0, 0.25, 1.0),
    );
    model.materials.push(Material::isotropic(2000.0, 0.3, 2.0));
    model.element_materials = MaterialAssignment::PerElement(vec![MaterialId(1), MaterialId(0)]);

    let decoded = format::read_bytes(&encode(&format::File::from_model(model.clone()))).unwrap();
    assert_eq!(decoded.model().unwrap().unwrap(), model);
}

#[test]
fn rejects_malformed_model_sections() {
    let mut file = format::File::new(rectangle_mesh(1, 1, 1.0, 1.0));
    file.plane_model = Some(PlaneModel::PlaneStress);
    assert!(matches!(
        file.model().unwrap_err(),
        format::FormatError::InvalidModel(_)
    ));

    let mut file = format::File::from_model(Model::uniform(
        rectangle_mesh(1, 1, 1.0, 1.0),
        PlaneModel::PlaneStress,
        Material::default(),
    ));
    file.element_materials = Some(MaterialAssignment::PerElement(vec![MaterialId(0)]));
    assert!(matches!(
        file.model().unwrap_err(),
        format::FormatError::InvalidModel(_)
    ));

    file.element_materials = Some(MaterialAssignment::Uniform(MaterialId(3)));
    assert!(matches!(
        file.model().unwrap_err(),
        format::FormatError::InvalidModel(_)
    ));
}

#[test]
fn boundary_conditions_roundtrip() {
    let mut bc = BoundaryConditions::default();
    bc.set_prescribed(DofId(0), 1.5);
    bc.set_prescribed(DofId(3), 0.0);
    bc.set_force(DofId(5), -2.0);

    let boundary = bc.to_file();
    assert_eq!(boundary.prescribed.len(), 2);
    assert_eq!(boundary.forces.len(), 1);

    let restored = BoundaryConditions::from_file(&boundary, 6).unwrap();
    assert_eq!(restored.prescribed, bc.prescribed);
    assert_eq!(restored.forces, bc.forces);
}

#[test]
fn rejects_out_of_range_boundary_entries() {
    let bad_component = Boundary {
        prescribed: vec![DofValue {
            node: NodeId(0),
            component: 2,
            value: 1.0,
        }],
        forces: Vec::new(),
    };
    assert!(matches!(
        BoundaryConditions::from_file(&bad_component, 6).unwrap_err(),
        FemError::InvalidInput(_)
    ));

    let bad_node = Boundary {
        prescribed: Vec::new(),
        forces: vec![DofValue {
            node: NodeId(99),
            component: 0,
            value: 1.0,
        }],
    };
    assert!(matches!(
        BoundaryConditions::from_file(&bad_node, 6).unwrap_err(),
        FemError::InvalidInput(_)
    ));
}
