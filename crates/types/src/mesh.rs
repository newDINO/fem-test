use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};

/// Stable index of a node inside a [`Mesh`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct NodeId(pub u32);

impl NodeId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Stable index of a triangle inside a [`Mesh`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ElemId(pub u32);

impl ElemId {
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Node {
    /// The first two components are used by the 2D solver. The third component
    /// is kept so that the mesh type can grow into a 3D solver later.
    pub position: [f64; 3],
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Mesh {
    pub nodes: Vec<Node>,
    pub triangles: Vec<[NodeId; 3]>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds2 {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl Bounds2 {
    pub fn size(self) -> [f64; 2] {
        [self.max[0] - self.min[0], self.max[1] - self.min[1]]
    }

    pub fn center(self) -> [f64; 2] {
        [
            0.5 * (self.min[0] + self.max[0]),
            0.5 * (self.min[1] + self.max[1]),
        ]
    }

    pub fn diagonal(self) -> f64 {
        let s = self.size();
        (s[0] * s[0] + s[1] * s[1]).sqrt()
    }
}

impl Mesh {
    pub fn node_position(&self, node: NodeId) -> [f64; 2] {
        let p = self.nodes[node.index()].position;
        [p[0], p[1]]
    }

    pub fn triangle_coords(&self, tri: [NodeId; 3]) -> [[f64; 2]; 3] {
        [
            self.node_position(tri[0]),
            self.node_position(tri[1]),
            self.node_position(tri[2]),
        ]
    }

    pub fn bounds(&self) -> Option<Bounds2> {
        let first = self.nodes.first()?;
        let mut min = [first.position[0], first.position[1]];
        let mut max = min;
        for node in &self.nodes[1..] {
            for c in 0..2 {
                min[c] = min[c].min(node.position[c]);
                max[c] = max[c].max(node.position[c]);
            }
        }
        Some(Bounds2 { min, max })
    }

    /// All unique triangle edges, sorted for deterministic rendering.
    pub fn edges(&self) -> Vec<[NodeId; 2]> {
        let mut set = BTreeSet::new();
        for tri in &self.triangles {
            for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                let (i, j) = (tri[a].0, tri[b].0);
                set.insert((i.min(j), i.max(j)));
            }
        }
        set.into_iter()
            .map(|(a, b)| [NodeId(a), NodeId(b)])
            .collect()
    }
}

/// Signed area of a 2D triangle; positive means counter-clockwise.
pub fn signed_area2(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
}
