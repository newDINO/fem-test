struct Uniforms {
    center: vec2<f32>,
    scale: f32,
    _pad0: f32,
    viewport: vec2<f32>,
    _pad1: vec2<f32>,
};

@group(0) @binding(0) var<uniform> uniforms: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    let relative = in.position.xy - uniforms.center;
    let screen = vec2<f32>(relative.x * uniforms.scale, -relative.y * uniforms.scale);
    let ndc = screen / (uniforms.viewport * 0.5);

    var out: VertexOutput;
    out.clip_position = vec4<f32>(ndc, 0.0, 1.0);
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
