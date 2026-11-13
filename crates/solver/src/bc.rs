use std::collections::BTreeMap;

use fem_types::format::{Boundary, DofValue};

use crate::dof::{DOFS_PER_NODE, DofId, DofMap};

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

    /// Convert to the node-based representation used by the file format.
    pub fn to_file(&self) -> Boundary {
        Boundary {
            prescribed: self
                .prescribed
                .iter()
                .map(|(&dof, &value)| DofValue {
                    node: dof.node(),
                    component: dof.component() as u8,
                    value,
                })
                .collect(),
            forces: self
                .forces
                .iter()
                .map(|(&dof, &value)| DofValue {
                    node: dof.node(),
                    component: dof.component() as u8,
                    value,
                })
                .collect(),
        }
    }

    /// Build from the node-based file representation. Components that the
    /// current 2D solver does not know (for example `Uz`) are rejected instead
    /// of being silently dropped.
    pub fn from_file(
        boundary: &Boundary,
        node_count: usize,
    ) -> Result<Self, crate::error::FemError> {
        let mut bc = Self::default();
        for entry in &boundary.prescribed {
            bc.set_prescribed(dof_from_file(entry, node_count)?, entry.value);
        }
        for entry in &boundary.forces {
            bc.set_force(dof_from_file(entry, node_count)?, entry.value);
        }
        Ok(bc)
    }
}

fn dof_from_file(entry: &DofValue, node_count: usize) -> Result<DofId, crate::error::FemError> {
    let node = entry.node.index();
    let component = entry.component as usize;

    if node >= node_count {
        return Err(crate::error::FemError::InvalidInput(format!(
            "boundary condition references node {} outside the mesh",
            entry.node.0
        )));
    }
    if component >= DOFS_PER_NODE {
        return Err(crate::error::FemError::InvalidInput(format!(
            "boundary condition uses component {} but the 2D solver only has {DOFS_PER_NODE}",
            entry.component
        )));
    }

    Ok(DofId((node * DOFS_PER_NODE + component) as u32))
}
