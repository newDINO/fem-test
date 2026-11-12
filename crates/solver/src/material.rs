use nalgebra::Matrix3;

use crate::error::FemError;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaneModel {
    PlaneStress,
    PlaneStrain,
}

#[derive(Clone, Copy, Debug)]
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
    pub fn validate(&self) -> Result<(), FemError> {
        if !self.youngs_modulus.is_finite() || self.youngs_modulus <= 0.0 {
            return Err(FemError::InvalidMaterial(
                "E must be finite and greater than zero".to_owned(),
            ));
        }
        if !self.poisson_ratio.is_finite()
            || self.poisson_ratio <= -1.0
            || self.poisson_ratio >= 0.5
        {
            return Err(FemError::InvalidMaterial(
                "ν must satisfy -1 < ν < 0.5".to_owned(),
            ));
        }
        if self.model == PlaneModel::PlaneStress
            && (!self.thickness.is_finite() || self.thickness <= 0.0)
        {
            return Err(FemError::InvalidMaterial(
                "plane-stress thickness must be finite and greater than zero".to_owned(),
            ));
        }
        Ok(())
    }

    pub fn thickness_factor(&self) -> f64 {
        match self.model {
            PlaneModel::PlaneStress => self.thickness,
            PlaneModel::PlaneStrain => 1.0,
        }
    }

    pub fn d_matrix(&self) -> Matrix3<f64> {
        let e = self.youngs_modulus;
        let nu = self.poisson_ratio;
        match self.model {
            PlaneModel::PlaneStress => {
                let c = e / (1.0 - nu * nu);
                Matrix3::new(
                    c,
                    c * nu,
                    0.0,
                    c * nu,
                    c,
                    0.0,
                    0.0,
                    0.0,
                    c * (1.0 - nu) * 0.5,
                )
            }
            PlaneModel::PlaneStrain => {
                let c = e / ((1.0 + nu) * (1.0 - 2.0 * nu));
                Matrix3::new(
                    c * (1.0 - nu),
                    c * nu,
                    0.0,
                    c * nu,
                    c * (1.0 - nu),
                    0.0,
                    0.0,
                    0.0,
                    c * (1.0 - 2.0 * nu) * 0.5,
                )
            }
        }
    }

    pub fn sigma_z(&self, sx: f64, sy: f64) -> f64 {
        match self.model {
            PlaneModel::PlaneStress => 0.0,
            PlaneModel::PlaneStrain => self.poisson_ratio * (sx + sy),
        }
    }
}
