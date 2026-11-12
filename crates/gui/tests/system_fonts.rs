use eframe::egui::{
    FontFamily,
    epaint::text::{FallbackRequest, FontProvider},
};
use egui_system_font_provider::SystemFontProvider;

#[test]
fn system_font_provider_finds_chinese_glyphs() {
    let provider = SystemFontProvider::new().with_locale(Some("zh-Hans"));
    let family = FontFamily::Proportional;

    let probe = "模".to_owned();
    let Some(font) = provider.font_for(&FallbackRequest {
        cluster: &probe,
        family: &family,
    }) else {
        eprintln!("no system CJK font installed; skipping font fallback test");
        return;
    };
    assert!(!font.data.bytes().is_empty());

    // If one Han glyph is available, the rest of the UI labels should be too.
    for text in ["型", "约", "束", "节", "点", "力", "求", "解"] {
        let cluster = text.to_owned();
        assert!(
            provider
                .font_for(&FallbackRequest {
                    cluster: &cluster,
                    family: &family,
                })
                .is_some(),
            "no system font found for {text}"
        );
    }
}
