use eframe::egui;
use fem_solver::{Mesh, NodeId};

use crate::camera::Camera2d;

pub fn pick_node(
    mesh: &Mesh,
    camera: &Camera2d,
    viewport: egui::Rect,
    cursor: egui::Pos2,
    radius_px: f32,
) -> Option<NodeId> {
    let radius_sq = radius_px * radius_px;
    let mut best = None;
    let mut best_dist = f32::INFINITY;

    for (index, node) in mesh.nodes.iter().enumerate() {
        let p = camera.world_to_screen([node.position[0], node.position[1]], viewport);
        let d = p - cursor;
        let d2 = d.x * d.x + d.y * d.y;
        if d2 <= radius_sq && d2 < best_dist {
            best = Some(NodeId(index as u32));
            best_dist = d2;
        }
    }
    best
}
