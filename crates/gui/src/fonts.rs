//! Locate a CJK-capable system font so the Chinese UI text renders instead of
//! showing missing-glyph boxes. egui's bundled default fonts only cover Latin.
//!
//! No font file is shipped with the project: loading a system font keeps the
//! binary small, and the feature degrades gracefully to English/ASCII or
//! tofu boxes on systems without a CJK font.

use eframe::egui;

const CANDIDATES: &[&str] = &[
    // Linux
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/google-noto-sans-cjk-fonts/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/wenquanyi/wqy-microhei/wqy-microhei.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    // Windows
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/msyh.ttf",
    "C:/Windows/Fonts/simhei.ttf",
    // macOS
    "/System/Library/Fonts/PingFang.ttc",
    "/System/Library/Fonts/STHeiti Light.ttc",
    "/Library/Fonts/Arial Unicode.ttf",
];

/// Register the first available CJK font as a fallback for both text families.
///
/// Set `FEM_CJK_FONT` to override the search with an explicit font path.
pub fn install_cjk_font(ctx: &egui::Context) {
    let env_path = std::env::var_os("FEM_CJK_FONT").map(std::path::PathBuf::from);
    let paths = env_path
        .as_deref()
        .into_iter()
        .chain(CANDIDATES.iter().map(std::path::Path::new));

    for path in paths {
        let Ok(bytes) = std::fs::read(path) else {
            continue;
        };
        let mut fonts = egui::FontDefinitions::default();
        let name = "system-cjk".to_owned();
        fonts.font_data.insert(
            name.clone(),
            std::sync::Arc::new(egui::FontData::from_owned(bytes)),
        );
        // Append rather than prepend: Latin keeps egui's default look while
        // CJK code points fall through to this font.
        for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
            fonts.families.entry(family).or_default().push(name.clone());
        }
        ctx.set_fonts(fonts);
        return;
    }

    eprintln!(
        "fem-gui: no CJK system font found; Chinese labels may render as placeholder boxes. \
         Set FEM_CJK_FONT to a .ttf/.ttc path to override."
    );
}
