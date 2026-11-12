use std::collections::BTreeMap;

use crate::dof::{DofId, DofMap};

#[derive(Clone, Debug, Default)]
pub struct BoundaryConditions {
    /// Prescribed displacement by DOF. A fixed DOF is stored with value zero.
    pub prescribed: BTreeMap<DofId, f64>,
    /// Nodal force by DOF.
    pub forces: BTreeMap<DofId, f64>,
}

impl BoundaryConditions {
    pub fn force_vector(&self, dof_map: &DofMap) -> Vec<f64> {
        let mut f = vec![0.0; dof_map.ndof()];
        for (&dof, &value) in &self.forces {
            if dof.index() < f.len() {
                f[dof.index()] += value;
            }
        }
        f
    }

    pub fn set_prescribed(&mut self, dof: DofId, value: f64) {
        self.prescribed.insert(dof, value);
    }

    pub fn clear_prescribed(&mut self, dof: DofId) {
        self.prescribed.remove(&dof);
    }

    pub fn set_force(&mut self, dof: DofId, value: f64) {
        if value == 0.0 {
            self.forces.remove(&dof);
        } else {
            self.forces.insert(dof, value);
        }
    }

    pub fn clear_force(&mut self, dof: DofId) {
        self.forces.remove(&dof);
    }

    pub fn clear_node(&mut self, dof_map: &DofMap, node: crate::mesh::NodeId) {
        for dof in dof_map.node_dofs(node) {
            self.prescribed.remove(&dof);
            self.forces.remove(&dof);
        }
    }

    pub fn constrained_dof_count(&self) -> usize {
        self.prescribed.len()
    }

    pub fn loaded_dof_count(&self) -> usize {
        self.forces.len()
    }
}
