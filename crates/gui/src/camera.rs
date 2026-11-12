use eframe::egui;
use fem_solver::Bounds2;

#[derive(Clone, Copy, Debug)]
pub struct Camera2d {
    pub center: [f64; 2],
    /// Screen pixels per world unit.
    pub scale: f64,
}

impl Default for Camera2d {
    fn default() -> Self {
        Self {
            center: [0.0, 0.0],
            scale: 50.0,
        }
    }
}

impl Camera2d {
    pub fn fit(&mut self, bounds: Bounds2, viewport: egui::Rect) {
        self.center = bounds.center();
        let size = bounds.size();
        let w = viewport.width().max(1.0) as f64;
        let h = viewport.height().max(1.0) as f64;
        let sx = if size[0] > 0.0 {
            w / size[0]
        } else {
            f64::INFINITY
        };
        let sy = if size[1] > 0.0 {
            h / size[1]
        } else {
            f64::INFINITY
        };
        let scale = sx.min(sy) / 1.15;
        self.scale = if scale.is_finite() && scale > 0.0 {
            scale.clamp(1e-9, 1e12)
        } else {
            50.0
        };
    }

    pub fn world_to_screen(&self, p: [f64; 2], viewport: egui::Rect) -> egui::Pos2 {
        egui::pos2(
            viewport.center().x + ((p[0] - self.center[0]) * self.scale) as f32,
            viewport.center().y - ((p[1] - self.center[1]) * self.scale) as f32,
        )
    }

    pub fn screen_to_world(&self, p: egui::Pos2, viewport: egui::Rect) -> [f64; 2] {
        [
            self.center[0] + (p.x - viewport.center().x) as f64 / self.scale,
            self.center[1] - (p.y - viewport.center().y) as f64 / self.scale,
        ]
    }

    pub fn pan(&mut self, delta: egui::Vec2) {
        self.center[0] -= delta.x as f64 / self.scale;
        self.center[1] += delta.y as f64 / self.scale;
    }

    pub fn zoom_at(&mut self, factor: f64, cursor: egui::Pos2, viewport: egui::Rect) {
        if !factor.is_finite() || factor <= 0.0 {
            return;
        }
        let before = self.screen_to_world(cursor, viewport);
        self.scale = (self.scale * factor).clamp(1e-9, 1e12);
        let after = self.screen_to_world(cursor, viewport);
        self.center[0] += before[0] - after[0];
        self.center[1] += before[1] - after[1];
    }
}
