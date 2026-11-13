use fem_types::format::{
    self, Boundary, DofLink, DofRef, DofValue, File, FormatError, Results,
};
use fem_types::{
    ElemId, ElementStress, Material, MaterialAssignment, MaterialId, Mesh, Model, Node, NodeId,
    PlaneModel, SolveResult, von_mises_stress,
};

fn triangle_mesh() -> Mesh {
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

fn triangle_model() -> Model {
    Model::uniform(
        triangle_mesh(),
        PlaneModel::PlaneStress,
        Material::isotropic(1000.0, 0.25, 1.0),
    )
}

fn triangle_file() -> File {
    File::from_model(triangle_model())
}

fn encode(file: &File) -> Vec<u8> {
    let mut bytes = Vec::new();
    format::write(&mut bytes, file).unwrap();
    bytes
}

#[test]
fn mesh_roundtrip_has_header() {
    let file = triangle_file();
    let bytes = encode(&file);

    assert_eq!(&bytes[0..4], b"FEMS");
    assert_eq!(
        u16::from_le_bytes([bytes[4], bytes[5]]),
        format::VERSION,
        "header stores the format version"
    );

    let decoded = format::read_bytes(&bytes).unwrap();
    assert_eq!(decoded, file);
}

#[test]
fn rejects_bad_magic() {
    let mut bytes = encode(&triangle_file());
    bytes[0] = b'X';

    assert!(matches!(
        format::read_bytes(&bytes).unwrap_err(),
        FormatError::NotFemFile
    ));
}

#[test]
fn rejects_unknown_version() {
    let mut bytes = encode(&triangle_file());
    bytes[4..6].copy_from_slice(&99u16.to_le_bytes());

    assert!(matches!(
        format::read_bytes(&bytes).unwrap_err(),
        FormatError::UnsupportedVersion {
            found: 99,
            expected: format::VERSION,
        }
    ));
}

#[test]
fn rejects_truncated_and_trailing_bytes() {
    let bytes = encode(&triangle_file());

    assert!(matches!(
        format::read_bytes(&bytes[..4]).unwrap_err(),
        FormatError::Truncated
    ));

    let mut with_trailing = bytes.clone();
    with_trailing.push(0);
    assert!(matches!(
        format::read_bytes(&with_trailing).unwrap_err(),
        FormatError::TrailingBytes
    ));
}

#[test]
fn mesh_only_file_has_no_model() {
    let file = File::new(triangle_mesh());
    let decoded = format::read_bytes(&encode(&file)).unwrap();

    assert_eq!(decoded, file);
    assert!(decoded.model().unwrap().is_none());
}

#[test]
fn model_roundtrip_keeps_boundary_and_assignment() {
    let boundary = Boundary {
        prescribed: vec![
            DofValue {
                node: NodeId(0),
                component: 0,
                value: 0.0,
            },
            DofValue {
                node: NodeId(0),
                component: 1,
                value: 0.0,
            },
        ],
        forces: vec![DofValue {
            node: NodeId(2),
            component: 1,
            value: -12.5,
        }],
        links: vec![DofLink {
            slave: DofRef {
                node: NodeId(2),
                component: 0,
            },
            master: DofRef {
                node: NodeId(1),
                component: 0,
            },
            offset: 0.25,
        }],
    };

    let mut model = triangle_model();
    model.materials.push(Material::isotropic(2000.0, 0.3, 2.0));
    model.element_materials = MaterialAssignment::PerElement(vec![MaterialId(1)]);

    let file = File::from_model(model.clone()).with_boundary(boundary.clone());
    let decoded = format::read_bytes(&encode(&file)).unwrap();

    assert_eq!(decoded.boundary, Some(boundary));
    assert_eq!(decoded.model().unwrap(), Some(model));
}

#[test]
fn rejects_unknown_material_in_assignment() {
    let mut file = triangle_file();
    file.element_materials = Some(MaterialAssignment::Uniform(MaterialId(1)));

    assert!(matches!(
        file.model().unwrap_err(),
        FormatError::InvalidModel(_)
    ));
}

#[test]
fn results_roundtrip_and_recompute_derived_values() {
    let model = triangle_model();
    let material = &model.materials[0];

    let stress = [10.0, 20.0, 30.0];
    let sigma_z = material.sigma_z(model.plane_model, stress[0], stress[1]);
    let result = SolveResult {
        displacements: vec![0.0, 0.0, 0.001, 0.0, 0.0, -0.002],
        reactions: vec![1.0, 2.0, 0.0, 0.0, 0.0, 0.0],
        stresses: vec![ElementStress {
            element: ElemId(0),
            stress,
            sigma_z,
            von_mises: von_mises_stress(stress[0], stress[1], stress[2], sigma_z),
        }],
        max_displacement: 0.0025,
        max_residual: 1e-12,
    };

    let file = triangle_file().with_results(Results::from_solve(&result));
    let decoded = format::read_bytes(&encode(&file)).unwrap();
    let loaded = decoded.results.as_ref().unwrap();

    assert_eq!(loaded.displacements.len(), model.mesh.nodes.len());
    assert_eq!(loaded.stresses, vec![stress]);

    let rebuilt = loaded.to_solve(&model).unwrap();
    assert_eq!(rebuilt.stresses, result.stresses);
    assert!((rebuilt.max_displacement - 0.002).abs() < 1e-15);
    assert!((rebuilt.max_residual - 1e-12).abs() < 1e-20);
}

#[test]
fn to_solve_rejects_size_mismatch() {
    let results = Results {
        displacements: vec![[0.0, 0.0]],
        reactions: None,
        stresses: vec![[0.0, 0.0, 0.0]],
        max_residual: None,
    };

    assert!(matches!(
        results.to_solve(&triangle_model()).unwrap_err(),
        FormatError::InvalidResult(_)
    ));
}
