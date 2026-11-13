//! Resolution of linear multi-point constraints into a reduced DOF set.
//!
//! Every scalar constraint has the form `u_slave = u_master + offset` (or a
//! chain of such relations). Prescribed displacements are treated as links to a
//! virtual ground DOF, so fixed and periodic DOFs go through the same path.
//!
//! The constraints are collected with a weighted union-find structure. The
//! potential on each edge is the displacement offset, which makes it possible
//! to detect inconsistent cycles (for example a corner whose x-periodic and
//! y-periodic offsets do not agree) instead of silently overwriting one of the
//! relations.

use std::collections::BTreeMap;

use crate::bc::Constraint;
use crate::dof::{DofId, DofMap};
use crate::error::FemError;

/// Resolution of every scalar DOF into a reduced unknown plus a constant part.
///
/// `u_i = q[reduced_index[i]] + constant[i]` for a free DOF, and
/// `u_i = constant[i]` when `reduced_index[i]` is `None`.
#[derive(Clone, Debug)]
pub struct DofRelations {
    pub reduced_index: Vec<Option<usize>>,
    pub constant: Vec<f64>,
    /// One global DOF index per reduced unknown, used for equilibrium checks.
    pub masters: Vec<usize>,
}

impl DofRelations {
    pub fn free_count(&self) -> usize {
        self.masters.len()
    }
}

/// Resolve prescribed displacements and linear links into a reduced DOF map.
pub fn resolve(
    dof_map: &DofMap,
    prescribed: &BTreeMap<DofId, f64>,
    links: &[Constraint],
) -> Result<DofRelations, FemError> {
    let ndof = dof_map.ndof();
    let ground = ndof;
    let mut parent: Vec<usize> = (0..=ndof).collect();
    let mut offset = vec![0.0; ndof + 1];
    let mut size = vec![1usize; ndof + 1];

    for (&dof, &value) in prescribed {
        check_dof(dof, ndof)?;
        if !value.is_finite() {
            return Err(FemError::InvalidInput(
                "prescribed displacement contains a non-finite value".to_owned(),
            ));
        }
        union(
            &mut parent,
            &mut offset,
            &mut size,
            dof.index(),
            ground,
            value,
        )?;
    }

    for link in links {
        check_dof(link.slave, ndof)?;
        check_dof(link.master, ndof)?;
        if !link.offset.is_finite() {
            return Err(FemError::InvalidInput(
                "linear constraint contains a non-finite offset".to_owned(),
            ));
        }
        union(
            &mut parent,
            &mut offset,
            &mut size,
            link.slave.index(),
            link.master.index(),
            link.offset,
        )?;
    }

    let mut reduced_index = vec![None; ndof];
    let mut constant = vec![0.0; ndof];
    let mut root_index = vec![None; ndof];
    let mut masters = Vec::new();

    for i in 0..ndof {
        let (root, off) = find(&mut parent, &mut offset, i);
        constant[i] = off;

        if root == ground {
            continue;
        }

        let reduced = match root_index[root] {
            Some(index) => index,
            None => {
                let index = masters.len();
                root_index[root] = Some(index);
                masters.push(root);
                index
            }
        };
        reduced_index[i] = Some(reduced);
    }

    Ok(DofRelations {
        reduced_index,
        constant,
        masters,
    })
}

fn check_dof(dof: DofId, ndof: usize) -> Result<(), FemError> {
    if dof.index() >= ndof {
        Err(FemError::InvalidInput(format!(
            "constraint references DOF {} outside the mesh",
            dof.index()
        )))
    } else {
        Ok(())
    }
}

/// Find the root of `index` and the total offset such that
/// `u_index = u_root + offset`.
fn find(parent: &mut [usize], offset: &mut [f64], index: usize) -> (usize, f64) {
    let mut root = index;
    let mut total = 0.0;
    while parent[root] != root {
        total += offset[root];
        root = parent[root];
    }

    // Path compression: make every visited node point directly at the root.
    let mut current = index;
    let mut accumulated = 0.0;
    while parent[current] != root {
        let next = parent[current];
        let edge = offset[current];
        parent[current] = root;
        offset[current] = total - accumulated;
        accumulated += edge;
        current = next;
    }

    (root, total)
}

/// Add the relation `u_slave = u_master + value`.
fn union(
    parent: &mut [usize],
    offset: &mut [f64],
    size: &mut [usize],
    slave: usize,
    master: usize,
    value: f64,
) -> Result<(), FemError> {
    let (slave_root, slave_offset) = find(parent, offset, slave);
    let (master_root, master_offset) = find(parent, offset, master);

    if slave_root == master_root {
        let actual = slave_offset - master_offset;
        let tolerance = 1e-9 * (1.0 + actual.abs().max(value.abs()));
        if (actual - value).abs() > tolerance {
            return Err(FemError::ConstraintConflict(format!(
                "u[{}] - u[{}] = {value} conflicts with the existing relation {actual}",
                dof_label(slave),
                dof_label(master),
            )));
        }
        return Ok(());
    }

    // `u_master_root = u_slave_root + delta`.
    let delta = slave_offset - master_offset - value;
    let ground = parent.len() - 1;

    if master_root == ground {
        // Keep the virtual ground at the root so that u_ground stays zero.
        parent[slave_root] = ground;
        offset[slave_root] = -delta;
        size[ground] += size[slave_root];
    } else if slave_root == ground {
        parent[master_root] = ground;
        offset[master_root] = delta;
        size[ground] += size[master_root];
    } else if size[slave_root] < size[master_root] {
        parent[slave_root] = master_root;
        offset[slave_root] = -delta;
        size[master_root] += size[slave_root];
    } else {
        parent[master_root] = slave_root;
        offset[master_root] = delta;
        size[slave_root] += size[master_root];
    }

    Ok(())
}

fn dof_label(index: usize) -> String {
    let dof = DofId(index as u32);
    format!("node {} component {}", dof.node().0, dof.component())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn resolve_links(node_count: usize, links: &[Constraint]) -> Result<DofRelations, FemError> {
        resolve(&DofMap::new(node_count), &BTreeMap::new(), links)
    }

    #[test]
    fn single_link_ties_slave_to_master() {
        let links = [Constraint::new(DofId(2), DofId(0), 0.5)];
        let relations = resolve_links(2, &links).unwrap();

        assert_eq!(relations.reduced_index[2], relations.reduced_index[0]);
        assert_eq!(relations.reduced_index[1], Some(1));
        assert!((relations.constant[0]).abs() < 1e-12);
        assert!((relations.constant[2] - 0.5).abs() < 1e-12);
        assert_eq!(relations.free_count(), 2);
    }

    #[test]
    fn chain_accumulates_offsets() {
        let links = [
            Constraint::new(DofId(2), DofId(1), 1.0),
            Constraint::new(DofId(1), DofId(0), 2.0),
        ];
        let relations = resolve_links(2, &links).unwrap();

        assert_eq!(relations.reduced_index[0], relations.reduced_index[2]);
        assert!((relations.constant[1] - relations.constant[0] - 2.0).abs() < 1e-12);
        assert!((relations.constant[2] - relations.constant[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn conflicting_links_are_rejected() {
        let links = [
            Constraint::new(DofId(1), DofId(0), 1.0),
            Constraint::new(DofId(1), DofId(0), 2.0),
        ];
        assert!(matches!(
            resolve_links(1, &links).unwrap_err(),
            FemError::ConstraintConflict(_)
        ));
    }

    #[test]
    fn prescribed_and_link_combine() {
        let mut prescribed = BTreeMap::new();
        prescribed.insert(DofId(0), 5.0);
        let links = [Constraint::new(DofId(1), DofId(0), 2.0)];

        let relations = resolve(&DofMap::new(1), &prescribed, &links).unwrap();

        assert_eq!(relations.reduced_index[0], None);
        assert_eq!(relations.reduced_index[1], None);
        assert!((relations.constant[0] - 5.0).abs() < 1e-12);
        assert!((relations.constant[1] - 7.0).abs() < 1e-12);
        assert_eq!(relations.free_count(), 0);
    }

    #[test]
    fn consistent_cycle_is_accepted() {
        // 3 = 0 + 3 can be reached as 3 = 1 + 2 and 1 = 0 + 1.
        let links = [
            Constraint::new(DofId(1), DofId(0), 1.0),
            Constraint::new(DofId(2), DofId(0), 2.0),
            Constraint::new(DofId(3), DofId(2), 1.0),
            Constraint::new(DofId(3), DofId(1), 2.0),
        ];
        let relations = resolve_links(2, &links).unwrap();

        assert_eq!(relations.reduced_index[0], relations.reduced_index[3]);
        assert!((relations.constant[1] - relations.constant[0] - 1.0).abs() < 1e-12);
        assert!((relations.constant[2] - relations.constant[0] - 2.0).abs() < 1e-12);
        assert!((relations.constant[3] - relations.constant[0] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn conflicting_cycle_is_rejected() {
        let links = [
            Constraint::new(DofId(1), DofId(0), 1.0),
            Constraint::new(DofId(2), DofId(0), 2.0),
            Constraint::new(DofId(3), DofId(2), 1.0),
            Constraint::new(DofId(3), DofId(1), 3.0),
        ];
        assert!(matches!(
            resolve_links(2, &links).unwrap_err(),
            FemError::ConstraintConflict(_)
        ));
    }

    #[test]
    fn prescribed_value_conflicts_with_link() {
        let mut prescribed = BTreeMap::new();
        prescribed.insert(DofId(0), 5.0);
        prescribed.insert(DofId(1), 6.0);
        let links = [Constraint::new(DofId(1), DofId(0), 2.0)];

        assert!(matches!(
            resolve(&DofMap::new(1), &prescribed, &links).unwrap_err(),
            FemError::ConstraintConflict(_)
        ));
    }
}
