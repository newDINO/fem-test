use nalgebra::Matrix3;

pub use fem_types::material::{IsotropicMaterial, Material, MaterialId};
pub use fem_types::model::PlaneModel;

use crate::error::FemError;

/// Solver-side material behavior: validation errors and the plane elasticity
/// matrix. The plain parameters live in `fem-types`.
pub trait MaterialExt {
    fn validate(&self, plane_model: PlaneModel) -> Result<(), FemError>;
    fn d_matrix(&self, plane_model: PlaneModel) -> Matrix3<f64>;
}

impl MaterialExt for Material {
    fn validate(&self, plane_model: PlaneModel) -> Result<(), FemError> {
        match self {
            Material::Isotropic(material) => {
                if !material.youngs_modulus.is_finite() || material.youngs_modulus <= 0.0 {
                    return Err(FemError::InvalidMaterial(
                        "E must be finite and greater than zero".to_owned(),
                    ));
                }
                if !material.poisson_ratio.is_finite()
                    || material.poisson_ratio <= -1.0
                    || material.poisson_ratio >= 0.5
                {
                    return Err(FemError::InvalidMaterial(
                        "ν must satisfy -1 < ν < 0.5".to_owned(),
                    ));
                }
                if plane_model == PlaneModel::PlaneStress
                    && (!material.thickness.is_finite() || material.thickness <= 0.0)
                {
                    return Err(FemError::InvalidMaterial(
                        "plane-stress thickness must be finite and greater than zero".to_owned(),
                    ));
                }
                Ok(())
            }
        }
    }

    fn d_matrix(&self, plane_model: PlaneModel) -> Matrix3<f64> {
        match self {
            Material::Isotropic(material) => {
                let e = material.youngs_modulus;
                let nu = material.poisson_ratio;
                match plane_model {
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
        }
    }
}
