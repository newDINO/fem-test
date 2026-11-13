use std::io::Cursor;

use fem_import::{DropAxis, ImportError, StlImportOptions, import_stl_mesh};

fn ascii_stl(faces: &[([f64; 3], [f64; 3], [f64; 3])], name: &str) -> Vec<u8> {
    let mut source = format!("solid {name}\n");
    for (a, b, c) in faces {
        source.push_str("facet normal 0 0 1\nouter loop\n");
        for p in [a, b, c] {
            source.push_str(&format!("vertex {} {} {}\n", p[0], p[1], p[2]));
        }
        source.push_str("endloop\nendfacet\n");
    }
    source.push_str(&format!("endsolid {name}\n"));
    source.into_bytes()
}

fn square_faces() -> [([f64; 3], [f64; 3], [f64; 3]); 2] {
    [
        ([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 5.0, 0.0]),
        ([0.0, 0.0, 0.0], [10.0, 5.0, 0.0], [0.0, 5.0, 0.0]),
    ]
}

#[test]
fn imports_planar_stl_and_welds_vertices() {
    let mut cursor = Cursor::new(ascii_stl(&square_faces(), "plate"));
    let stl = stl_io::read_stl(&mut cursor).unwrap();
    let (mesh, report) = import_stl_mesh(
        &stl,
        StlImportOptions {
            drop_axis: DropAxis::Z,
            weld_tolerance: None,
        },
    )
    .unwrap();

    assert_eq!(mesh.nodes.len(), 4);
    assert_eq!(mesh.triangles.len(), 2);
    assert_eq!(report.imported_triangles, 2);
}

#[test]
fn rejects_non_planar_stl() {
    let faces = [
        ([0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 5.0, 0.0]),
        ([0.0, 0.0, 2.0], [10.0, 5.0, 2.0], [0.0, 5.0, 2.0]),
    ];
    let mut cursor = Cursor::new(ascii_stl(&faces, "bad"));
    let stl = stl_io::read_stl(&mut cursor).unwrap();
    let error = import_stl_mesh(&stl, StlImportOptions::default()).unwrap_err();

    assert!(matches!(error, ImportError::NonPlanar(_)));
}

#[test]
fn welds_vertices_across_quantization_cells() {
    // The shared corner is perturbed by 0.26 while the weld tolerance is 0.5,
    // so the two copies land in different quantization cells. The neighbor
    // search must still merge them.
    let faces = [
        ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
        ([0.26, 0.26, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]),
    ];
    let mut cursor = Cursor::new(ascii_stl(&faces, "straddle"));
    let stl = stl_io::read_stl(&mut cursor).unwrap();
    let (mesh, report) = import_stl_mesh(
        &stl,
        StlImportOptions {
            drop_axis: DropAxis::Z,
            weld_tolerance: Some(0.5),
        },
    )
    .unwrap();

    assert_eq!(mesh.nodes.len(), 4);
    assert_eq!(mesh.triangles.len(), 2);
    assert_eq!(report.welded_vertices, 4);
}
