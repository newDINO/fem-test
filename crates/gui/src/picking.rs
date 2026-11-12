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

/// All nodes whose screen-space position falls inside `rect`.
///
/// The rectangle is clipped to the viewport so that dragging beyond the
/// viewport cannot select nodes that are not visible.
pub fn pick_nodes_in_rect(
    mesh: &Mesh,
    camera: &Camera2d,
    viewport: egui::Rect,
    rect: egui::Rect,
) -> Vec<NodeId> {
    let rect = rect.intersect(viewport);
    if !rect.is_positive() {
        return Vec::new();
    }

    mesh.nodes
        .iter()
        .enumerate()
        .filter(|(_, node)| {
            let p = camera.world_to_screen([node.position[0], node.position[1]], viewport);
            rect.contains(p)
        })
        .map(|(index, _)| NodeId(index as u32))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use fem_solver::Node;

    fn mesh() -> Mesh {
        Mesh {
            nodes: vec![
                Node {
                    position: [0.0, 0.0, 0.0],
                },
                Node {
                    position: [2.0, 2.0, 0.0],
                },
                Node {
                    position: [-3.0, 0.0, 0.0],
                },
            ],
            triangles: Vec::new(),
        }
    }

    fn viewport() -> egui::Rect {
        egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(200.0, 100.0))
    }

    fn camera() -> Camera2d {
        Camera2d {
            center: [0.0, 0.0],
            scale: 10.0,
        }
    }

    #[test]
    fn box_select_picks_nodes_inside_any_drag_direction() {
        let mesh = mesh();
        let camera = camera();
        // Node 0 is at screen (100, 50), node 1 at (120, 30).
        let rect = egui::Rect::from_two_pos(egui::pos2(95.0, 55.0), egui::pos2(125.0, 25.0));
        let hits = pick_nodes_in_rect(&mesh, &camera, viewport(), rect);
        assert_eq!(hits, vec![NodeId(0), NodeId(1)]);
    }

    #[test]
    fn box_select_ignores_nodes_outside_the_viewport() {
        let mesh = mesh();
        let camera = camera();
        // A rectangle that covers the whole viewport would include node 2 at
        // screen (70, 50), but the same rectangle shifted outside the viewport
        // must not select anything.
        let outside = egui::Rect::from_min_size(egui::pos2(300.0, 0.0), egui::vec2(50.0, 50.0));
        assert!(pick_nodes_in_rect(&mesh, &camera, viewport(), outside).is_empty());
    }
}
