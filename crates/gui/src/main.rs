#[cfg(target_family = "wasm")]
fn main() {
    // The web build is started from the `fem-web` crate; this binary target
    // only exists so that the GUI crate can be built for every target.
}

#[cfg(not(target_family = "wasm"))]
fn main() -> eframe::Result<()> {
    use eframe::egui;
    use fem_gui::app::FemApp;

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
