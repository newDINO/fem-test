use nalgebra::{Matrix3, SMatrix, SVector};

use crate::error::FemError;
use crate::material::Material;

pub type BMatrix = SMatrix<f64, 3, 6>;
pub type KMatrix = SMatrix<f64, 6, 6>;

/// Constant-strain triangle (3-node triangle, CST).
#[derive(Clone, Copy, Debug)]
pub struct Tri3Kernel {
    pub area: f64,
    pub b: BMatrix,
    pub d: Matrix3<f64>,
}

impl Tri3Kernel {
    pub fn new(
        element_index: usize,
        coords: [[f64; 2]; 3],
        material: &Material,
    ) -> Result<Self, FemError> {
        let a = coords[0];
        let b = coords[1];
        let c = coords[2];
        let two_area = (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);

        if !two_area.is_finite() || two_area.abs() <= f64::EPSILON {
            return Err(FemError::DegenerateElement(element_index));
        }

        // Keep the node order used by the mesh. Using the signed area here
        // makes B correct for both clockwise and counter-clockwise input while
        // the stiffness B^T D B remains positive semi-definite.
        let area = 0.5 * two_area.abs();
        let inv_two_area = 1.0 / two_area;
        let (x1, y1) = (a[0], a[1]);
        let (x2, y2) = (b[0], b[1]);
        let (x3, y3) = (c[0], c[1]);

        // Strain-displacement matrix for a constant-strain triangle.
        let bmat = BMatrix::new(
            y2 - y3,
            0.0,
            y3 - y1,
            0.0,
            y1 - y2,
            0.0,
            0.0,
            x3 - x2,
            0.0,
            x1 - x3,
            0.0,
            x2 - x1,
            x3 - x2,
            y2 - y3,
            x1 - x3,
            y3 - y1,
            x2 - x1,
            y1 - y2,
        ) * inv_two_area;

        let d = material.d_matrix();
        Ok(Self { area, b: bmat, d })
    }

    pub fn stiffness(&self, material: &Material) -> KMatrix {
        let db = self.d * self.b;
        self.b.transpose() * db * (material.thickness_factor() * self.area)
    }

    pub fn stress(&self, local_displacement: &SVector<f64, 6>) -> SVector<f64, 3> {
        self.d * self.b * local_displacement
    }
}
