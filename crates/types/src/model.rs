use std::error::Error;
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::material::{Material, MaterialId};
use crate::mesh::{ElemId, Mesh};

/// Plane stress vs plane strain is a property of the analysis, not of a single
/// material, so it is stored once per model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlaneModel {
    PlaneStress,
    PlaneStrain,
}

/// How elements map to the material library.
///
/// Both variants describe the same kind of data; `Uniform` is simply the
/// compact form for the common case. A uniform assignment stores the material
/// id explicitly instead of relying on an "empty means material 0" convention.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MaterialAssignment {
    Uniform(MaterialId),
    /// One entry per triangle. The length must match the mesh.
    PerElement(Vec<MaterialId>),
}

impl MaterialAssignment {
    pub fn uniform(material: MaterialId) -> Self {
        Self::Uniform(material)
    }

    pub fn get(&self, element: ElemId) -> Option<MaterialId> {
        match self {
            Self::Uniform(material) => Some(*material),
            Self::PerElement(materials) => materials.get(element.index()).copied(),
        }
    }

    pub fn is_uniform(&self) -> bool {
        matches!(self, Self::Uniform(_))
    }

    pub fn material_count(&self) -> usize {
        match self {
            Self::Uniform(_) => 1,
            Self::PerElement(materials) => materials.len(),
        }
    }

    /// Check that every entry points into a library with `material_count`
    /// entries. `PerElement` assignments must also cover every element.
    pub fn validate(
        &self,
        element_count: usize,
        material_count: usize,
    ) -> Result<(), MaterialAssignmentError> {
        match self {
            Self::Uniform(material) => check_material(*material, None, material_count),
            Self::PerElement(materials) => {
                if materials.len() != element_count {
                    return Err(MaterialAssignmentError::Length {
                        expected: element_count,
                        found: materials.len(),
                    });
                }
                for (element, material) in materials.iter().enumerate() {
                    check_material(*material, Some(element), material_count)?;
                }
                Ok(())
            }
        }
    }

    /// Replace the whole assignment with a single material.
    pub fn set_all(&mut self, material: MaterialId) {
        *self = Self::Uniform(material);
    }

    /// Assign one element without losing the other entries. Expands a uniform
    /// assignment to per-element data only when the new material differs.
    pub fn set(&mut self, element: ElemId, material: MaterialId, element_count: usize) {
        match self {
            Self::Uniform(current) => {
                if *current == material {
                    return;
                }
                let mut materials = vec![*current; element_count];
                if let Some(slot) = materials.get_mut(element.index()) {
                    *slot = material;
                }
                *self = Self::PerElement(materials);
            }
            Self::PerElement(materials) => {
                if let Some(slot) = materials.get_mut(element.index()) {
                    *slot = material;
                }
            }
        }
    }

    /// Collapse a per-element assignment back to `Uniform` when every entry
    /// agrees.
    pub fn compact(&mut self) {
        let Self::PerElement(materials) = self else {
            return;
        };
        if let Some(first) = materials.first().copied()
            && materials.iter().all(|material| *material == first)
        {
            *self = Self::Uniform(first);
        }
    }
}

fn check_material(
    material: MaterialId,
    element: Option<usize>,
    material_count: usize,
) -> Result<(), MaterialAssignmentError> {
    if material.index() >= material_count {
        Err(MaterialAssignmentError::UnknownMaterial { element, material })
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MaterialAssignmentError {
    EmptyLibrary,
    Length {
        expected: usize,
        found: usize,
    },
    UnknownMaterial {
        element: Option<usize>,
        material: MaterialId,
    },
}

impl fmt::Display for MaterialAssignmentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyLibrary => write!(f, "the material library is empty"),
            Self::Length { expected, found } => write!(
                f,
                "material assignment has {found} entries but the mesh has {expected} elements"
            ),
            Self::UnknownMaterial {
                element: Some(element),
                material,
            } => write!(
                f,
                "element {element} references unknown material {}",
                material.0
            ),
            Self::UnknownMaterial {
                element: None,
                material,
            } => write!(f, "assignment references unknown material {}", material.0),
        }
    }
}

impl Error for MaterialAssignmentError {}

/// A complete analysis model: geometry, the global plane assumption, the
/// material library and the element-to-material assignment.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Model {
    pub mesh: Mesh,
    pub plane_model: PlaneModel,
    pub materials: Vec<Material>,
    pub element_materials: MaterialAssignment,
}

impl Model {
    /// One material for the whole mesh.
    pub fn uniform(mesh: Mesh, plane_model: PlaneModel, material: Material) -> Self {
        Self {
            mesh,
            plane_model,
            materials: vec![material],
            element_materials: MaterialAssignment::Uniform(MaterialId(0)),
        }
    }

    pub fn material(&self, material: MaterialId) -> Option<&Material> {
        self.materials.get(material.index())
    }

    pub fn material_for(&self, element: ElemId) -> Option<&Material> {
        self.material(self.element_materials.get(element)?)
    }

    /// Structural checks that do not depend on the solver: non-empty library,
    /// one assignment entry per element and every id inside the library.
    /// Material parameter validation (E > 0, ...) lives in the solver.
    pub fn validate_materials(&self) -> Result<(), MaterialAssignmentError> {
        if self.materials.is_empty() {
            return Err(MaterialAssignmentError::EmptyLibrary);
        }
        self.element_materials
            .validate(self.mesh.triangles.len(), self.materials.len())
    }
}
