use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use bytemuck::{Pod, Zeroable};
use eframe::egui;
use eframe::egui_wgpu::{self, wgpu};
use fem_solver::{BoundaryConditions, Mesh, NodeId, SolveResult};

use crate::camera::Camera2d;

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

#[derive(Clone, Debug, Default)]
pub struct RenderData {
    pub fill: Vec<Vertex>,
    pub lines: Vec<Vertex>,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    center: [f32; 2],
    scale: f32,
    _pad0: f32,
    viewport: [f32; 2],
    _pad1: [f32; 2],
}

impl Default for Uniforms {
    fn default() -> Self {
        Self {
            center: [0.0; 2],
            scale: 1.0,
            _pad0: 0.0,
            viewport: [1.0; 2],
            _pad1: [0.0; 2],
        }
    }
}

fn push_vertex(out: &mut Vec<Vertex>, p: [f64; 2], color: [f32; 4]) {
    out.push(Vertex {
        position: [p[0] as f32, p[1] as f32, 0.0],
        color,
    });
}

fn push_line(out: &mut Vec<Vertex>, a: [f64; 2], b: [f64; 2], color: [f32; 4]) {
    push_vertex(out, a, color);
    push_vertex(out, b, color);
}

fn stress_color(t: f32) -> [f32; 4] {
    let t = t.clamp(0.0, 1.0);
    // blue -> cyan -> green -> yellow -> red
    let (r, g, b) = if t < 0.25 {
        let s = t / 0.25;
        (0.0, s, 1.0)
    } else if t < 0.5 {
        let s = (t - 0.25) / 0.25;
        (0.0, 1.0, 1.0 - s)
    } else if t < 0.75 {
        let s = (t - 0.5) / 0.25;
        (s, 1.0, 0.0)
    } else {
        let s = (t - 0.75) / 0.25;
        (1.0, 1.0 - s, 0.0)
    };
    [r, g, b, 1.0]
}

pub fn build_render_data(
    mesh: Option<&Mesh>,
    result: Option<&SolveResult>,
    bc: &BoundaryConditions,
    selected: &BTreeSet<NodeId>,
    show_deformed: bool,
    deformation_scale: f64,
) -> RenderData {
    let Some(mesh) = mesh else {
        return RenderData::default();
    };
    let Some(bounds) = mesh.bounds() else {
        return RenderData::default();
    };

    let diag = bounds.diagonal().max(1e-12);
    let marker = diag * 0.004;
    let mut out = RenderData::default();

    let position = |node: NodeId| -> [f64; 2] {
        let base = mesh.node_position(node);
        if show_deformed {
            if let Some(result) = result {
                [
                    base[0] + deformation_scale * result.displacement(node, 0),
                    base[1] + deformation_scale * result.displacement(node, 1),
                ]
            } else {
                base
            }
        } else {
            base
        }
    };

    let stress_bounds = result.and_then(|result| {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for stress in &result.stresses {
            min = min.min(stress.von_mises);
            max = max.max(stress.von_mises);
        }
        (min.is_finite() && max.is_finite()).then_some((min, max))
    });

    for (element_index, tri) in mesh.triangles.iter().enumerate() {
        let color = match (result, stress_bounds) {
            (Some(result), Some((min, max))) => {
                let vm = result
                    .stresses
                    .get(element_index)
                    .map_or(min, |s| s.von_mises);
                let range = max - min;
                // With a uniform stress field there is no meaningful scale;
                // use the middle color instead of dumping everything to blue.
                let t = if range <= 1e-12 * max.abs().max(1.0) {
                    0.5
                } else {
                    ((vm - min) / range) as f32
                };
                stress_color(t)
            }
            _ => [0.70, 0.77, 0.84, 1.0],
        };
        for node in tri {
            push_vertex(&mut out.fill, position(*node), color);
        }
    }

    let edge_color = [0.10, 0.14, 0.18, 0.9];
    for edge in mesh.edges() {
        push_line(
            &mut out.lines,
            position(edge[0]),
            position(edge[1]),
            edge_color,
        );
    }

    // Node markers. They stay at the undeformed positions so that picking and
    // the original geometry remain stable while the filled shape deforms.
    for (index, node) in mesh.nodes.iter().enumerate() {
        let base = [node.position[0], node.position[1]];
        let node_id = NodeId(index as u32);
        let is_selected = selected.contains(&node_id);
        let color = if is_selected {
            [1.0, 0.86, 0.05, 1.0]
        } else {
            [0.20, 0.27, 0.34, 0.65]
        };
        let r = if is_selected { marker * 2.5 } else { marker };
        push_line(
            &mut out.lines,
            [base[0] - r, base[1]],
            [base[0] + r, base[1]],
            color,
        );
        push_line(
            &mut out.lines,
            [base[0], base[1] - r],
            [base[0], base[1] + r],
            color,
        );
    }

    // Constraint markers: a diamond around every constrained node.
    let mut constrained_nodes = BTreeSet::new();
    for dof in bc.prescribed.keys() {
        constrained_nodes.insert(dof.node());
    }
    for node in constrained_nodes {
        let p = mesh.node_position(node);
        let r = marker * 2.0;
        let color = [0.98, 0.48, 0.08, 1.0];
        let a = [p[0] - r, p[1]];
        let b = [p[0], p[1] + r];
        let c = [p[0] + r, p[1]];
        let d = [p[0], p[1] - r];
        push_line(&mut out.lines, a, b, color);
        push_line(&mut out.lines, b, c, color);
        push_line(&mut out.lines, c, d, color);
        push_line(&mut out.lines, d, a, color);
    }

    // Force arrows: sum the two DOF components at each node.
    let mut nodal_forces: BTreeMap<NodeId, [f64; 2]> = BTreeMap::new();
    for (&dof, &value) in &bc.forces {
        if dof.component() < 2 {
            nodal_forces.entry(dof.node()).or_insert([0.0; 2])[dof.component()] += value;
        }
    }
    let max_force = nodal_forces
        .values()
        .map(|v| (v[0] * v[0] + v[1] * v[1]).sqrt())
        .fold(0.0_f64, f64::max);
    for (node, force) in nodal_forces {
        let magnitude = (force[0] * force[0] + force[1] * force[1]).sqrt();
        if magnitude <= 1e-30 || max_force <= 0.0 {
            continue;
        }
        let p = mesh.node_position(node);
        let dir = [force[0] / magnitude, force[1] / magnitude];
        let length = (diag * 0.14 * magnitude / max_force).max(marker * 2.0);
        let tip = [p[0] + dir[0] * length, p[1] + dir[1] * length];
        let color = [0.92, 0.16, 0.10, 1.0];
        push_line(&mut out.lines, p, tip, color);

        let head = (length * 0.25).clamp(marker * 1.5, length * 0.45);
        let perp = [-dir[1], dir[0]];
        let base = [tip[0] - dir[0] * head, tip[1] - dir[1] * head];
        push_line(
            &mut out.lines,
            tip,
            [
                base[0] + perp[0] * head * 0.45,
                base[1] + perp[1] * head * 0.45,
            ],
            color,
        );
        push_line(
            &mut out.lines,
            tip,
            [
                base[0] - perp[0] * head * 0.45,
                base[1] - perp[1] * head * 0.45,
            ],
            color,
        );
    }

    out
}

pub struct ViewportGpu {
    fill_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    fill_buffer: wgpu::Buffer,
    fill_capacity: u64,
    fill_count: u32,
    line_buffer: wgpu::Buffer,
    line_capacity: u64,
    line_count: u32,
    last_revision: u64,
}

impl ViewportGpu {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("fem viewport shader"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!("render_shader.wgsl"))),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("fem viewport bind group layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("fem viewport uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("fem viewport bind group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("fem viewport pipeline layout"),
            bind_group_layouts: &[Some(&bind_group_layout)],
            immediate_size: 0,
        });

        let attributes = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4];
        let vertex_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &attributes,
        };

        let make_pipeline = |label: &str, topology: wgpu::PrimitiveTopology| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    buffers: &[Some(vertex_layout.clone())],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some("fs_main"),
                    compilation_options: wgpu::PipelineCompilationOptions::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: target_format,
                        blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                primitive: wgpu::PrimitiveState {
                    topology,
                    strip_index_format: None,
                    front_face: wgpu::FrontFace::Ccw,
                    cull_mode: None,
                    unclipped_depth: false,
                    polygon_mode: wgpu::PolygonMode::Fill,
                    conservative: false,
                },
                depth_stencil: None,
                multisample: wgpu::MultisampleState::default(),
                multiview_mask: None,
                cache: None,
            })
        };

        let fill_pipeline =
            make_pipeline("fem viewport fill", wgpu::PrimitiveTopology::TriangleList);
        let line_pipeline = make_pipeline("fem viewport lines", wgpu::PrimitiveTopology::LineList);

        let empty_buffer = |label: &str| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: 4,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };

        Self {
            fill_pipeline,
            line_pipeline,
            uniform_buffer,
            bind_group,
            fill_buffer: empty_buffer("fem viewport fill vertices"),
            fill_capacity: 4,
            fill_count: 0,
            line_buffer: empty_buffer("fem viewport line vertices"),
            line_capacity: 4,
            line_count: 0,
            last_revision: u64::MAX,
        }
    }

    fn upload_vertices(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        buffer: &mut wgpu::Buffer,
        capacity: &mut u64,
        data: &[Vertex],
    ) {
        let bytes = bytemuck::cast_slice(data);
        if bytes.len() as u64 > *capacity {
            *capacity = (bytes.len() as u64).max(4);
            *buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("fem viewport vertex buffer"),
                size: *capacity,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
        }
        if !bytes.is_empty() {
            queue.write_buffer(buffer, 0, bytes);
        }
    }

    fn upload(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, data: &RenderData) {
        Self::upload_vertices(
            device,
            queue,
            &mut self.fill_buffer,
            &mut self.fill_capacity,
            &data.fill,
        );
        self.fill_count = data.fill.len() as u32;

        Self::upload_vertices(
            device,
            queue,
            &mut self.line_buffer,
            &mut self.line_capacity,
            &data.lines,
        );
        self.line_count = data.lines.len() as u32;
    }

    fn draw<'a>(&self, pass: &mut wgpu::RenderPass<'a>) {
        pass.set_bind_group(0, &self.bind_group, &[]);

        if self.fill_count > 0 {
            pass.set_pipeline(&self.fill_pipeline);
            pass.set_vertex_buffer(0, self.fill_buffer.slice(..));
            pass.draw(0..self.fill_count, 0..1);
        }
        if self.line_count > 0 {
            pass.set_pipeline(&self.line_pipeline);
            pass.set_vertex_buffer(0, self.line_buffer.slice(..));
            pass.draw(0..self.line_count, 0..1);
        }
    }
}

#[derive(Clone)]
pub struct ViewportCallback {
    pub data: Arc<RenderData>,
    pub camera: Camera2d,
    pub viewport: [f32; 2],
    pub revision: u64,
}

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        callback_resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if let Some(gpu) = callback_resources.get_mut::<ViewportGpu>() {
            if gpu.last_revision != self.revision {
                gpu.upload(device, queue, &self.data);
                gpu.last_revision = self.revision;
            }
            let uniforms = Uniforms {
                center: [self.camera.center[0] as f32, self.camera.center[1] as f32],
                scale: self.camera.scale as f32,
                _pad0: 0.0,
                viewport: [self.viewport[0].max(1.0), self.viewport[1].max(1.0)],
                _pad1: [0.0; 2],
            };
            queue.write_buffer(&gpu.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));
        }
        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        callback_resources: &egui_wgpu::CallbackResources,
    ) {
        if let Some(gpu) = callback_resources.get::<ViewportGpu>() {
            gpu.draw(render_pass);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::future::Future;
    use std::task::{Context, Poll};

    use super::*;

    fn block_on<F: Future>(future: F) -> F::Output {
        let mut context = Context::from_waker(std::task::Waker::noop());
        let mut future = std::pin::pin!(future);
        loop {
            match future.as_mut().poll(&mut context) {
                Poll::Ready(value) => return value,
                Poll::Pending => std::thread::sleep(std::time::Duration::from_millis(2)),
            }
        }
    }

    /// Smoke test for the WGSL shader and the wgpu pipeline descriptors.
    /// Skips silently when the machine exposes no usable adapter.
    #[test]
    fn creates_pipeline_on_available_adapter() {
        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle().with_env());
        let adapter = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            compatible_surface: None,
            force_fallback_adapter: false,
            apply_limit_buckets: false,
        }));
        let Ok(adapter) = adapter else {
            eprintln!("no wgpu adapter available; skipping pipeline smoke test");
            return;
        };

        let descriptor = wgpu::DeviceDescriptor {
            label: Some("fem-gui pipeline test device"),
            ..Default::default()
        };
        let Ok((device, _queue)) = block_on(adapter.request_device(&descriptor)) else {
            eprintln!("could not create a wgpu device; skipping pipeline smoke test");
            return;
        };

        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let gpu = ViewportGpu::new(&device, wgpu::TextureFormat::Bgra8UnormSrgb);
        drop(gpu);
        let error = block_on(scope.pop());
        assert!(error.is_none(), "pipeline validation error: {error:?}");
    }

    /// Render one triangle and read it back, to pin down the world-to-clip
    /// mapping. A vertically mirrored viewport would swap the two samples.
    #[test]
    fn renders_world_positive_y_upwards() {
        let instance =
            wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle().with_env());
        let Ok(adapter) = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            compatible_surface: None,
            force_fallback_adapter: false,
            apply_limit_buckets: false,
        })) else {
            eprintln!("no wgpu adapter available; skipping orientation test");
            return;
        };
        let Ok((device, queue)) = block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("fem-gui orientation test device"),
            ..Default::default()
        })) else {
            eprintln!("could not create a wgpu device; skipping orientation test");
            return;
        };

        const SIZE: u32 = 64;
        const SCALE: f32 = 16.0;
        const COLOR: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
        let format = wgpu::TextureFormat::Rgba8UnormSrgb;

        let mut gpu = ViewportGpu::new(&device, format);
        gpu.upload(
            &device,
            &queue,
            &RenderData {
                fill: vec![
                    Vertex {
                        position: [-1.0, -1.0, 0.0],
                        color: COLOR,
                    },
                    Vertex {
                        position: [1.0, -1.0, 0.0],
                        color: COLOR,
                    },
                    Vertex {
                        position: [1.0, 1.0, 0.0],
                        color: COLOR,
                    },
                ],
                lines: Vec::new(),
            },
        );

        let uniforms = Uniforms {
            center: [0.0, 0.0],
            scale: SCALE,
            _pad0: 0.0,
            viewport: [SIZE as f32, SIZE as f32],
            _pad1: [0.0; 2],
        };
        queue.write_buffer(&gpu.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));

        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("orientation test target"),
            size: wgpu::Extent3d {
                width: SIZE,
                height: SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("orientation test encoder"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("orientation test pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            gpu.draw(&mut pass);
        }

        let bytes_per_row = SIZE * 4; // 256, already a multiple of COPY_BYTES_PER_ROW_ALIGNMENT.
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("orientation test readback"),
            size: (bytes_per_row * SIZE) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &readback,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(bytes_per_row),
                    rows_per_image: Some(SIZE),
                },
            },
            wgpu::Extent3d {
                width: SIZE,
                height: SIZE,
                depth_or_array_layers: 1,
            },
        );
        queue.submit([encoder.finish()]);

        let slice = readback.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result);
        });
        device
            .poll(wgpu::PollType::Wait {
                submission_index: None,
                timeout: None,
            })
            .expect("device poll");
        receiver.recv().expect("map callback").expect("buffer map");

        let pixels = slice.get_mapped_range().expect("mapped range");
        let pixel = |x: u32, y: u32| -> [u8; 4] {
            let i = (y * bytes_per_row + x * 4) as usize;
            [pixels[i], pixels[i + 1], pixels[i + 2], pixels[i + 3]]
        };

        // The triangle covers screen coordinates with x + y > 64, i.e. the
        // upper-right half. World +Y must land near the top of the image.
        let inside = pixel(36, 44);
        let outside = pixel(30, 20);
        assert!(
            inside[0] > 180 && inside[1] < 80,
            "expected a filled pixel at (36, 44), got {inside:?}"
        );
        assert!(
            outside[0] < 80,
            "expected background at (30, 20), got {outside:?}"
        );
    }
}
