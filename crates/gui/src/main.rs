mod app;
mod camera;
mod picking;
mod renderer;

use app::FemApp;
use eframe::egui;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 820.0])
            .with_min_inner_size([900.0, 600.0])
            .with_title("FEM 2D Prototype"),
        renderer: eframe::Renderer::Wgpu,
        // Load system fonts on demand for glyphs egui's default fonts lack
        // (e.g. Chinese labels). This is the default, kept explicit here.
        system_font_fallback: true,
        ..Default::default()
    };

    eframe::run_native(
        "FEM 2D Prototype",
        options,
        Box::new(|cc| Ok(Box::new(FemApp::new(cc)))),
    )
}
