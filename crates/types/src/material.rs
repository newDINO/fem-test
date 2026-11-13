use serde::{Deserialize, Serialize};

use crate::model::PlaneModel;

/// Stable index into the material library of a [`crate::Model`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct MaterialId(pub u32);

impl MaterialId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Constitutive model. New material kinds (orthotropic, temperature dependent,
/// ...) become additional variants; the element-to-material assignment does not
/// change.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Material {
    Isotropic(IsotropicMaterial),
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct IsotropicMaterial {
    /// Young's modulus. The number is unitless; the user must keep it
    /// consistent with the geometry and loads.
    pub youngs_modulus: f64,
    pub poisson_ratio: f64,
    /// Only used for plane stress. Plane strain uses unit thickness.
    pub thickness: f64,
}

impl Default for IsotropicMaterial {
    fn default() -> Self {
        Self {
            youngs_modulus: 210_000.0,
            poisson_ratio: 0.3,
            thickness: 1.0,
        }
    }
}

impl Default for Material {
    fn default() -> Self {
        Self::Isotropic(IsotropicMaterial::default())
    }
}

impl Material {
    pub const fn isotropic(youngs_modulus: f64, poisson_ratio: f64, thickness: f64) -> Self {
        Self::Isotropic(IsotropicMaterial {
            youngs_modulus,
            poisson_ratio,
            thickness,
        })
    }

    pub fn isotropic_material(&self) -> Option<&IsotropicMaterial> {
        match self {
            Self::Isotropic(material) => Some(material),
        }
    }

    pub fn isotropic_material_mut(&mut self) -> Option<&mut IsotropicMaterial> {
        match self {
            Self::Isotropic(material) => Some(material),
        }
    }

    /// Thickness scale used by the stiffness integral for the given analysis
    /// assumption.
    pub fn thickness_factor(&self, plane_model: PlaneModel) -> f64 {
        match (self, plane_model) {
            (Self::Isotropic(material), PlaneModel::PlaneStress) => material.thickness,
            (Self::Isotropic(_), PlaneModel::PlaneStrain) => 1.0,
        }
    }

    /// Out-of-plane stress. Non-zero only for plane strain.
    pub fn sigma_z(&self, plane_model: PlaneModel, sx: f64, sy: f64) -> f64 {
        match (self, plane_model) {
            (Self::Isotropic(_), PlaneModel::PlaneStress) => 0.0,
            (Self::Isotropic(material), PlaneModel::PlaneStrain) => {
                material.poisson_ratio * (sx + sy)
            }
        }
    }
}
