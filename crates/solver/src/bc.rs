use std::collections::BTreeMap;

use fem_types::format::{Boundary, DofLink, DofRef, DofValue};

use crate::dof::{DOFS_PER_NODE, DofId, DofMap};
use crate::mesh::NodeId;

/// A linear multi-point constraint `u_slave = u_master + offset`.
///
/// The relation is applied to the assembled global system, so the two DOFs do
/// not have to belong to the same element. Periodic boundary conditions are
/// expressed as links between opposite boundary nodes; the offset is the
/// macroscopic displacement jump across the cell.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Constraint {
    pub slave: DofId,
    pub master: DofId,
    pub offset: f64,
}

impl Constraint {
    pub fn new(slave: DofId, master: DofId, offset: f64) -> Self {
        Self {
            slave,
            master,
            offset,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct BoundaryConditions {
    /// Prescribed displacement by DOF. A fixed DOF is stored with value zero.
    pub prescribed: BTreeMap<DofId, f64>,
    /// Nodal force by DOF.
    pub forces: BTreeMap<DofId, f64>,
    /// Multi-point constraints, applied after assembly.
    pub links: Vec<Constraint>,
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

    pub fn add_link(&mut self, slave: DofId, master: DofId, offset: f64) {
        self.links.push(Constraint::new(slave, master, offset));
    }

    pub fn clear_links(&mut self) {
        self.links.clear();
    }

    pub fn clear_node(&mut self, dof_map: &DofMap, node: NodeId) {
        for dof in dof_map.node_dofs(node) {
            self.prescribed.remove(&dof);
            self.forces.remove(&dof);
        }
        self.links
            .retain(|link| link.slave.node() != node && link.master.node() != node);
    }

    pub fn constrained_dof_count(&self) -> usize {
        self.prescribed.len()
    }

    pub fn link_count(&self) -> usize {
        self.links.len()
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
            links: self
                .links
                .iter()
                .map(|link| DofLink {
                    slave: dof_ref(link.slave),
                    master: dof_ref(link.master),
                    offset: link.offset,
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
        for link in &boundary.links {
            if !link.offset.is_finite() {
                return Err(crate::error::FemError::InvalidInput(
                    "linear constraint contains a non-finite offset".to_owned(),
                ));
            }
            bc.add_link(
                dof_from_parts(link.slave.node, link.slave.component, node_count)?,
                dof_from_parts(link.master.node, link.master.component, node_count)?,
                link.offset,
            );
        }
        Ok(bc)
    }
}

fn dof_ref(dof: DofId) -> DofRef {
    DofRef {
        node: dof.node(),
        component: dof.component() as u8,
    }
}

fn dof_from_file(entry: &DofValue, node_count: usize) -> Result<DofId, crate::error::FemError> {
    dof_from_parts(entry.node, entry.component, node_count)
}

fn dof_from_parts(
    node: NodeId,
    component: u8,
    node_count: usize,
) -> Result<DofId, crate::error::FemError> {
    let node_index = node.index();
    let component = component as usize;

    if node_index >= node_count {
        return Err(crate::error::FemError::InvalidInput(format!(
            "boundary condition references node {} outside the mesh",
            node.0
        )));
    }
    if component >= DOFS_PER_NODE {
        return Err(crate::error::FemError::InvalidInput(format!(
            "boundary condition uses component {} but the 2D solver only has {DOFS_PER_NODE}",
            component
        )));
    }

    Ok(DofId((node_index * DOFS_PER_NODE + component) as u32))
}
