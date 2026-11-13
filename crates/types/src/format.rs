//! MessagePack container for meshes, models and results.
//!
//! Layout:
//!
//! ```text
//! offset  size  content
//! 0       4     MAGIC ("FEMS")
//! 4       2     VERSION, little endian
//! 6       ..    MessagePack payload (struct fields are encoded as maps)
//! ```
//!
//! The header is written by hand so that a reader can reject a file written by
//! an incompatible version *before* trying to deserialize it. The payload uses
//! `rmp-serde` with named fields, so adding a new optional field is a
//! backwards-compatible change as long as the version stays the same.
//!
//! The format is still under active development: it stays at version 1 even
//! while fields change, and only stable formats will get migration code.

use std::io::{Read, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::model::{MaterialAssignment, Model, PlaneModel};
use crate::result::von_mises_stress;
use crate::{ElemId, ElementStress, Material, Mesh, NodeId, SolveResult};

pub const MAGIC: [u8; 4] = *b"FEMS";
pub const VERSION: u16 = 1;
pub const HEADER_LEN: usize = 6;

/// A file. The mesh is always present; the analysis fields form one unit:
/// either all of `plane_model`, `materials` and `element_materials` are present
/// (a complete model), or none of them are (a mesh-only file).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct File {
    pub mesh: Mesh,
    #[serde(default)]
    pub plane_model: Option<PlaneModel>,
    #[serde(default)]
    pub materials: Vec<Material>,
    #[serde(default)]
    pub element_materials: Option<MaterialAssignment>,
    #[serde(default)]
    pub boundary: Option<Boundary>,
    #[serde(default)]
    pub results: Option<Results>,
}

impl File {
    /// A mesh-only file, for mesh generators that do not know about materials.
    pub fn new(mesh: Mesh) -> Self {
        Self {
            mesh,
            plane_model: None,
            materials: Vec::new(),
            element_materials: None,
            boundary: None,
            results: None,
        }
    }

    pub fn from_model(model: Model) -> Self {
        let Model {
            mesh,
            plane_model,
            materials,
            element_materials,
        } = model;
        Self {
            mesh,
            plane_model: Some(plane_model),
            materials,
            element_materials: Some(element_materials),
            boundary: None,
            results: None,
        }
    }

    pub fn with_boundary(mut self, boundary: Boundary) -> Self {
        self.boundary = Some(boundary);
        self
    }

    pub fn with_results(mut self, results: Results) -> Self {
        self.results = Some(results);
        self
    }

    /// Build the complete analysis model. Returns `Ok(None)` for a mesh-only
    /// file; a partially specified analysis section is an error.
    pub fn model(&self) -> Result<Option<Model>, FormatError> {
        let has_analysis = self.plane_model.is_some()
            || !self.materials.is_empty()
            || self.element_materials.is_some();
        if !has_analysis {
            return Ok(None);
        }

        let Some(plane_model) = self.plane_model else {
            return Err(FormatError::InvalidModel(
                "plane model is missing (materials are present)".to_owned(),
            ));
        };
        if self.materials.is_empty() {
            return Err(FormatError::InvalidModel(
                "material library is empty (plane model is present)".to_owned(),
            ));
        }
        let Some(element_materials) = &self.element_materials else {
            return Err(FormatError::InvalidModel(
                "element material assignment is missing".to_owned(),
            ));
        };

        let model = Model {
            mesh: self.mesh.clone(),
            plane_model,
            materials: self.materials.clone(),
            element_materials: element_materials.clone(),
        };
        model
            .validate_materials()
            .map_err(|error| FormatError::InvalidModel(error.to_string()))?;
        Ok(Some(model))
    }
}

/// Boundary conditions stored in node terms, not in solver DOF indices.
///
/// `component` is `0 = Ux`, `1 = Uy`, `2 = Uz`. The current 2D solver rejects
/// component 2 when the file is loaded, but keeping the component in the file
/// means a future 3D solver does not have to change the format.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DofValue {
    pub node: NodeId,
    pub component: u8,
    pub value: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Boundary {
    #[serde(default)]
    pub prescribed: Vec<DofValue>,
    #[serde(default)]
    pub forces: Vec<DofValue>,
}

/// Node-major and element-major arrays. Derived values such as von Mises
/// stress, `sigma_z` and the maximum displacement are recomputed on load.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Results {
    /// `displacements[i] = [ux, uy]` for node `i`.
    pub displacements: Vec<[f64; 2]>,
    /// `reactions[i] = [rx, ry]` for node `i`, when stored.
    #[serde(default)]
    pub reactions: Option<Vec<[f64; 2]>>,
    /// `stresses[i] = [sigma_x, sigma_y, tau_xy]` for triangle `i`.
    pub stresses: Vec<[f64; 3]>,
    /// Diagnostic only; cannot be derived from the displacement field.
    #[serde(default)]
    pub max_residual: Option<f64>,
}

impl Results {
    pub fn from_solve(result: &SolveResult) -> Self {
        let displacements = result.displacements.as_chunks::<2>().0.to_vec();
        let reactions = result.reactions.as_chunks::<2>().0.to_vec();
        let stresses = result.stresses.iter().map(|s| s.stress).collect();

        Self {
            displacements,
            reactions: Some(reactions),
            stresses,
            max_residual: Some(result.max_residual),
        }
    }

    pub fn to_solve(&self, model: &Model) -> Result<SolveResult, FormatError> {
        let mesh = &model.mesh;

        if self.displacements.len() != mesh.nodes.len() {
            return Err(FormatError::InvalidResult(format!(
                "results contain {} nodes but the mesh has {}",
                self.displacements.len(),
                mesh.nodes.len()
            )));
        }
        if self.stresses.len() != mesh.triangles.len() {
            return Err(FormatError::InvalidResult(format!(
                "results contain {} elements but the mesh has {}",
                self.stresses.len(),
                mesh.triangles.len()
            )));
        }
        model
            .validate_materials()
            .map_err(|error| FormatError::InvalidModel(error.to_string()))?;

        let displacements = self.displacements.iter().flatten().copied().collect();
        let reactions = match &self.reactions {
            Some(values) => {
                if values.len() != mesh.nodes.len() {
                    return Err(FormatError::InvalidResult(format!(
                        "results contain {} reaction nodes but the mesh has {}",
                        values.len(),
                        mesh.nodes.len()
                    )));
                }
                values.iter().flatten().copied().collect()
            }
            None => vec![0.0; mesh.nodes.len() * 2],
        };

        let mut stresses = Vec::with_capacity(self.stresses.len());
        for (index, stress) in self.stresses.iter().enumerate() {
            let element = ElemId(index as u32);
            let material = model.material_for(element).ok_or_else(|| {
                FormatError::InvalidModel(format!("element {index} has no material"))
            })?;
            let sigma_z = material.sigma_z(model.plane_model, stress[0], stress[1]);
            stresses.push(ElementStress {
                element,
                stress: *stress,
                sigma_z,
                von_mises: von_mises_stress(stress[0], stress[1], stress[2], sigma_z),
            });
        }

        let max_displacement = self
            .displacements
            .iter()
            .map(|d| d[0].hypot(d[1]))
            .fold(0.0, f64::max);

        Ok(SolveResult {
            displacements,
            reactions,
            stresses,
            max_displacement,
            max_residual: self.max_residual.unwrap_or(0.0),
        })
    }
}

#[derive(Debug, thiserror::Error)]
pub enum FormatError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    #[error("not a FEM file (bad magic)")]
    NotFemFile,
    #[error("truncated FEM header")]
    Truncated,
    #[error("unsupported FEM file version {found} (this build reads version {expected})")]
    UnsupportedVersion { found: u16, expected: u16 },
    #[error("MessagePack decode failed: {0}")]
    Decode(#[from] rmp_serde::decode::Error),
    #[error("MessagePack encode failed: {0}")]
    Encode(#[from] rmp_serde::encode::Error),
    #[error("trailing bytes after the payload")]
    TrailingBytes,
    #[error("invalid analysis model in file: {0}")]
    InvalidModel(String),
    #[error("invalid result file: {0}")]
    InvalidResult(String),
}

/// Serialize to any writer.
pub fn write<W: Write>(mut writer: W, file: &File) -> Result<(), FormatError> {
    let payload = rmp_serde::to_vec_named(file)?;
    writer.write_all(&MAGIC)?;
    writer.write_all(&VERSION.to_le_bytes())?;
    writer.write_all(&payload)?;
    Ok(())
}

/// Deserialize from any reader. The whole file is buffered so that trailing
/// bytes can be detected; mesh files are small enough that this is fine.
pub fn read<R: Read>(mut reader: R) -> Result<File, FormatError> {
    let mut bytes = Vec::new();
    reader.read_to_end(&mut bytes)?;
    read_bytes(&bytes)
}

pub fn read_bytes(bytes: &[u8]) -> Result<File, FormatError> {
    if bytes.len() < HEADER_LEN {
        return Err(FormatError::Truncated);
    }
    if bytes[0..4] != MAGIC {
        return Err(FormatError::NotFemFile);
    }

    let version = u16::from_le_bytes([bytes[4], bytes[5]]);
    if version != VERSION {
        return Err(FormatError::UnsupportedVersion {
            found: version,
            expected: VERSION,
        });
    }

    let payload = &bytes[HEADER_LEN..];
    let mut deserializer = rmp_serde::Deserializer::new(std::io::Cursor::new(payload));
    let file = File::deserialize(&mut deserializer)?;

    if deserializer.position() as usize != payload.len() {
        return Err(FormatError::TrailingBytes);
    }

    Ok(file)
}

pub fn read_from_path(path: impl AsRef<Path>) -> Result<File, FormatError> {
    read(std::fs::File::open(path)?)
}

/// Write through a temporary file so that a crash cannot leave a half-written
/// project behind.
pub fn write_to_path(path: impl AsRef<Path>, file: &File) -> Result<(), FormatError> {
    let path = path.as_ref();
    let temporary = path.with_extension("fem.tmp");

    {
        let mut handle = std::fs::File::create(&temporary)?;
        write(&mut handle, file)?;
        handle.flush()?;
    }

    std::fs::rename(&temporary, path)?;
    Ok(())
}
