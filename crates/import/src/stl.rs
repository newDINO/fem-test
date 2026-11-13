use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::path::Path;

use stl_io::IndexedMesh;

use fem_types::{Mesh, Node, NodeId, signed_area2};

use crate::ImportError;

const ALIGNMENT_TOLERANCE: f64 = 0.999;
const PROJECTED_AREA_TOLERANCE: f64 = 1e-12;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum DropAxis {
    X,
    Y,
    #[default]
    Z,
}

impl DropAxis {
    pub const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::X => "X",
            Self::Y => "Y",
            Self::Z => "Z",
        }
    }

    pub const fn projected_axes(self) -> (usize, usize) {
        match self {
            Self::X => (1, 2),
            Self::Y => (0, 2),
            Self::Z => (0, 1),
        }
    }

    pub fn project(self, p: [f64; 3]) -> [f64; 2] {
        let (u, v) = self.projected_axes();
        [p[u], p[v]]
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StlImportOptions {
    pub drop_axis: DropAxis,
    /// If None, a tolerance based on the projected bounding-box diagonal is used.
    pub weld_tolerance: Option<f64>,
}

impl Default for StlImportOptions {
    fn default() -> Self {
        Self {
            drop_axis: DropAxis::Z,
            weld_tolerance: None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ImportReport {
    pub drop_axis: DropAxis,
    pub source_triangles: usize,
    pub imported_triangles: usize,
    pub source_vertices: usize,
    pub welded_vertices: usize,
    pub skipped_degenerate: usize,
    pub skipped_duplicate: usize,
    pub dropped_span: f64,
    pub in_plane_diagonal: f64,
    pub weld_tolerance: f64,
}

pub fn load_stl(
    path: impl AsRef<Path>,
    options: StlImportOptions,
) -> Result<(Mesh, ImportReport), ImportError> {
    let path = path.as_ref();
    let mut file = File::open(path)
        .map_err(|e| ImportError::Stl(format!("cannot open {}: {e}", path.display())))?;
    let stl = stl_io::read_stl(&mut file)
        .map_err(|e| ImportError::Stl(format!("cannot parse {}: {e}", path.display())))?;
    import_stl_mesh(&stl, options)
}

pub fn import_stl_mesh(
    stl: &IndexedMesh,
    options: StlImportOptions,
) -> Result<(Mesh, ImportReport), ImportError> {
    if stl.faces.is_empty() || stl.vertices.is_empty() {
        return Err(ImportError::EmptyMesh);
    }

    let axis = options.drop_axis.index();
    let mut dropped_min = f64::INFINITY;
    let mut dropped_max = f64::NEG_INFINITY;
    let mut projected_min = [f64::INFINITY; 2];
    let mut projected_max = [f64::NEG_INFINITY; 2];
    let mut usable_faces = 0usize;

    for face in &stl.faces {
        let mut points = [[0.0f64; 3]; 3];
        for (k, index) in face.vertices.iter().enumerate() {
            let vertex = stl.vertices.get(*index).ok_or_else(|| {
                ImportError::Stl("STL face references a missing vertex".to_owned())
            })?;
            points[k] = [vertex.0[0] as f64, vertex.0[1] as f64, vertex.0[2] as f64];
        }
        let [a, b, c] = points;
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let normal = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let normal_len =
            (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if normal_len <= 1e-30 {
            continue;
        }
        if (normal[axis] / normal_len).abs() < ALIGNMENT_TOLERANCE {
            return Err(ImportError::NonPlanar(format!(
                "a triangle normal is not parallel to the dropped {} axis; \
                 rotate the mesh so the plane is axis-aligned",
                options.drop_axis.label()
            )));
        }

        usable_faces += 1;
        for point in [a, b, c] {
            dropped_min = dropped_min.min(point[axis]);
            dropped_max = dropped_max.max(point[axis]);
            let projected = options.drop_axis.project(point);
            for c in 0..2 {
                projected_min[c] = projected_min[c].min(projected[c]);
                projected_max[c] = projected_max[c].max(projected[c]);
            }
        }
    }

    if usable_faces == 0 {
        return Err(ImportError::EmptyMesh);
    }

    let in_plane_size = [
        projected_max[0] - projected_min[0],
        projected_max[1] - projected_min[1],
    ];
    let in_plane_diagonal =
        (in_plane_size[0] * in_plane_size[0] + in_plane_size[1] * in_plane_size[1]).sqrt();
    if !in_plane_diagonal.is_finite() || in_plane_diagonal <= 0.0 {
        return Err(ImportError::NonPlanar(
            "the projected mesh has zero in-plane size".to_owned(),
        ));
    }

    let dropped_span = dropped_max - dropped_min;
    let planar_tolerance = (in_plane_diagonal * 1e-5).max(1e-9);
    if dropped_span > planar_tolerance {
        return Err(ImportError::NonPlanar(format!(
            "the mesh spans {dropped_span:.6e} along the dropped {} axis; \
             export a single planar surface or pick a different axis",
            options.drop_axis.label()
        )));
    }

    let weld_tolerance = options
        .weld_tolerance
        .unwrap_or_else(|| (in_plane_diagonal * 1e-6).max(1e-9));
    if !weld_tolerance.is_finite() || weld_tolerance <= 0.0 {
        return Err(ImportError::InvalidInput(
            "weld tolerance must be finite and greater than zero".to_owned(),
        ));
    }

    let mut nodes: Vec<Node> = Vec::new();
    let mut weld_map: HashMap<(i64, i64), Vec<NodeId>> = HashMap::new();
    let mut triangles = Vec::new();
    let mut seen_faces: HashSet<[u32; 3]> = HashSet::new();
    let mut skipped_duplicate = 0usize;
    let mut skipped_degenerate = 0usize;
    let area_tolerance =
        (in_plane_diagonal * in_plane_diagonal * PROJECTED_AREA_TOLERANCE).max(1e-20);

    for face in &stl.faces {
        let mut projected = [[0.0f64; 2]; 3];
        let mut valid = true;
        for (k, index) in face.vertices.iter().enumerate() {
            let Some(vertex) = stl.vertices.get(*index) else {
                return Err(ImportError::Stl(
                    "STL face references a missing vertex".to_owned(),
                ));
            };
            let p = [vertex.0[0] as f64, vertex.0[1] as f64, vertex.0[2] as f64];
            projected[k] = options.drop_axis.project(p);
            if !projected[k][0].is_finite() || !projected[k][1].is_finite() {
                valid = false;
            }
        }
        if !valid {
            skipped_degenerate += 1;
            continue;
        }

        let twice_area = 2.0 * signed_area2(projected[0], projected[1], projected[2]);
        if !twice_area.is_finite() || twice_area.abs() * 0.5 <= area_tolerance {
            skipped_degenerate += 1;
            continue;
        }

        let mut ids = [NodeId(0); 3];
        for (k, p) in projected.iter().enumerate() {
            let key = weld_key(*p, weld_tolerance)?;

            // A pure quantized-key lookup can miss duplicates that straddle a
            // cell boundary, so search the 3x3 neighborhood and pick the
            // closest node within tolerance.
            let mut chosen = None;
            let mut best_distance_sq = weld_tolerance * weld_tolerance;
            for dy in -1..=1i64 {
                for dx in -1..=1i64 {
                    let Some(candidates) =
                        weld_map.get(&(key.0.saturating_add(dx), key.1.saturating_add(dy)))
                    else {
                        continue;
                    };
                    for &candidate in candidates {
                        let q = nodes[candidate.index()].position;
                        let d2 = (q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2);
                        if d2 <= best_distance_sq {
                            best_distance_sq = d2;
                            chosen = Some(candidate);
                        }
                    }
                }
            }

            let id = match chosen {
                Some(id) => id,
                None => {
                    if nodes.len() > u32::MAX as usize {
                        return Err(ImportError::Stl(
                            "too many vertices for a 32-bit node index".to_owned(),
                        ));
                    }
                    let id = NodeId(nodes.len() as u32);
                    nodes.push(Node {
                        position: [p[0], p[1], 0.0],
                    });
                    weld_map.entry(key).or_default().push(id);
                    id
                }
            };
            ids[k] = id;
        }

        if ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0] {
            skipped_degenerate += 1;
            continue;
        }

        let mut face_key = [ids[0].0, ids[1].0, ids[2].0];
        face_key.sort_unstable();
        if !seen_faces.insert(face_key) {
            skipped_duplicate += 1;
            continue;
        }

        if twice_area < 0.0 {
            ids.swap(1, 2);
        }
        triangles.push(ids);
    }

    if triangles.is_empty() {
        return Err(ImportError::EmptyMesh);
    }

    let mesh = Mesh { nodes, triangles };
    mesh.validate()?;

    let report = ImportReport {
        drop_axis: options.drop_axis,
        source_triangles: stl.faces.len(),
        imported_triangles: mesh.triangles.len(),
        source_vertices: stl.vertices.len(),
        welded_vertices: mesh.nodes.len(),
        skipped_degenerate,
        skipped_duplicate,
        dropped_span,
        in_plane_diagonal,
        weld_tolerance,
    };

    Ok((mesh, report))
}

fn weld_key(p: [f64; 2], tolerance: f64) -> Result<(i64, i64), ImportError> {
    fn quantize(value: f64, tolerance: f64) -> Result<i64, ImportError> {
        let q = (value / tolerance).round();
        if !q.is_finite() || q < i64::MIN as f64 || q > i64::MAX as f64 {
            return Err(ImportError::InvalidInput(
                "coordinate is too large for the weld tolerance".to_owned(),
            ));
        }
        Ok(q as i64)
    }
    Ok((quantize(p[0], tolerance)?, quantize(p[1], tolerance)?))
}
