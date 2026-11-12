use crate::mesh::{ElemId, NodeId};

#[derive(Clone, Copy, Debug)]
pub struct ElementStress {
    pub element: ElemId,
    /// [sigma_x, sigma_y, tau_xy]
    pub stress: [f64; 3],
    /// Out-of-plane stress; non-zero for plane strain only.
    pub sigma_z: f64,
    pub von_mises: f64,
}

#[derive(Clone, Debug)]
pub struct SolveResult {
    pub displacements: Vec<f64>,
    pub reactions: Vec<f64>,
    pub stresses: Vec<ElementStress>,
    pub max_displacement: f64,
    pub max_residual: f64,
}

impl SolveResult {
    pub fn displacement(&self, node: NodeId, component: usize) -> f64 {
        self.displacements[node.index() * 2 + component]
    }

    pub fn reaction(&self, node: NodeId, component: usize) -> f64 {
        self.reactions[node.index() * 2 + component]
    }
}
