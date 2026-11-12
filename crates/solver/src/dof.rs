use crate::mesh::NodeId;

pub const DOFS_PER_NODE: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct DofId(pub u32);

impl DofId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }

    pub const fn node(self) -> NodeId {
        NodeId(self.0 / DOFS_PER_NODE as u32)
    }

    pub const fn component(self) -> usize {
        (self.0 % DOFS_PER_NODE as u32) as usize
    }
}

#[derive(Clone, Debug)]
pub struct DofMap {
    node_count: usize,
}

impl DofMap {
    pub fn new(node_count: usize) -> Self {
        Self { node_count }
    }

    pub fn node_count(&self) -> usize {
        self.node_count
    }

    pub fn ndof(&self) -> usize {
        self.node_count * DOFS_PER_NODE
    }

    pub fn dof(&self, node: NodeId, component: usize) -> DofId {
        debug_assert!(component < DOFS_PER_NODE);
        DofId(node.0 * DOFS_PER_NODE as u32 + component as u32)
    }

    pub fn node_dofs(&self, node: NodeId) -> [DofId; DOFS_PER_NODE] {
        [self.dof(node, 0), self.dof(node, 1)]
    }
}
