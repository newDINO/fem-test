//! Browser entry point for the FEM demo.
//!
//! Build with `crates/web/build.sh`; `index.html` imports the generated module
//! and starts a [`WebHandle`]. The desktop binary lives in `fem-gui`.

#![cfg(target_family = "wasm")]

use eframe::wasm_bindgen::prelude::*;
use fem_gui::app::FemApp;

/// Handle kept alive by JavaScript for as long as the app runs.
#[wasm_bindgen]
pub struct WebHandle {
    runner: eframe::WebRunner,
}

#[wasm_bindgen]
impl WebHandle {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        console_error_panic_hook::set_once();
        eframe::WebLogger::init(log::LevelFilter::Info).ok();
        Self {
            runner: eframe::WebRunner::new(),
        }
    }

    /// Start the app once the canvas is available.
    #[wasm_bindgen]
    pub async fn start(&self, canvas: web_sys::HtmlCanvasElement) -> Result<(), JsValue> {
        let options = eframe::WebOptions {
            renderer: eframe::Renderer::Wgpu,
            ..Default::default()
        };
        self.runner
            .start(
                canvas,
                options,
                Box::new(|cc| Ok(Box::new(FemApp::new(cc)))),
            )
            .await
    }

    #[wasm_bindgen]
    pub fn destroy(&self) {
        self.runner.destroy();
    }

    #[wasm_bindgen]
    pub fn has_panicked(&self) -> bool {
        self.runner.has_panicked()
    }
}

impl Default for WebHandle {
    fn default() -> Self {
        Self::new()
    }
}
