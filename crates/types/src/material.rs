use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PlaneModel {
    PlaneStress,
    PlaneStrain,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Material {
    /// Young's modulus. The number is unitless; the user must keep it
    /// consistent with the geometry and loads.
    pub youngs_modulus: f64,
    pub poisson_ratio: f64,
    /// Only used for plane stress. Plane strain uses unit thickness.
    pub thickness: f64,
    pub model: PlaneModel,
}

impl Default for Material {
    fn default() -> Self {
        Self {
            youngs_modulus: 210_000.0,
            poisson_ratio: 0.3,
            thickness: 1.0,
            model: PlaneModel::PlaneStress,
        }
    }
}

impl Material {
    pub fn thickness_factor(&self) -> f64 {
        match self.model {
            PlaneModel::PlaneStress => self.thickness,
            PlaneModel::PlaneStrain => 1.0,
        }
    }

    pub fn sigma_z(&self, sx: f64, sy: f64) -> f64 {
        match self.model {
            PlaneModel::PlaneStress => 0.0,
            PlaneModel::PlaneStrain => self.poisson_ratio * (sx + sy),
        }
    }
}
