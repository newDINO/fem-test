use std::collections::BTreeSet;
use std::path::Path;
use std::sync::Arc;

use eframe::egui;
use eframe::egui_wgpu;
use fem_solver::{
    BoundaryConditions, DropAxis, ImportReport, Material, Mesh, NodeId, PlaneModel, SolveResult,
    StlImportOptions, load_stl, solve,
};

use crate::camera::Camera2d;
use crate::picking::pick_node;
use crate::renderer::{RenderData, ViewportCallback, ViewportGpu, build_render_data};

pub struct FemApp {
    path_input: String,
    drop_axis: DropAxis,
    mesh: Option<Mesh>,
    report: Option<ImportReport>,
    material: Material,
    bc: BoundaryConditions,
    result: Option<SolveResult>,
    selection: BTreeSet<NodeId>,
    camera: Camera2d,
    needs_fit: bool,
    show_deformed: bool,
    deformation_scale: f64,
    render_data: Arc<RenderData>,
    render_revision: u64,
    cursor_world: Option<[f64; 2]>,
    status: String,
    error: Option<String>,
    edit_constrain_x: bool,
    edit_constrain_y: bool,
    edit_ux: f64,
    edit_uy: f64,
    edit_fx: f64,
    edit_fy: f64,
}

impl FemApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let render_state = cc
            .wgpu_render_state
            .as_ref()
            .expect("this application requires the wgpu renderer");
        let gpu = ViewportGpu::new(&render_state.device, render_state.target_format);
        render_state.renderer.write().callback_resources.insert(gpu);

        let mut app = Self {
            path_input: "assets/plate.stl".to_owned(),
            drop_axis: DropAxis::Z,
            mesh: None,
            report: None,
            material: Material::default(),
            bc: BoundaryConditions::default(),
            result: None,
            selection: BTreeSet::new(),
            camera: Camera2d::default(),
            needs_fit: true,
            show_deformed: true,
            deformation_scale: 1.0,
            render_data: Arc::new(RenderData::default()),
            render_revision: 0,
            cursor_world: None,
            status: "打开一个平面 STL 网格开始".to_owned(),
            error: None,
            edit_constrain_x: false,
            edit_constrain_y: false,
            edit_ux: 0.0,
            edit_uy: 0.0,
            edit_fx: 0.0,
            edit_fy: 0.0,
        };

        if Path::new(&app.path_input).exists() {
            app.load_current_path();
        }
        app
    }

    fn load_current_path(&mut self) {
        let path = self.path_input.trim().to_owned();
        if path.is_empty() {
            self.error = Some("请输入 STL 文件路径".to_owned());
            return;
        }

        let options = StlImportOptions {
            drop_axis: self.drop_axis,
            weld_tolerance: None,
        };
        match load_stl(&path, options) {
            Ok((mesh, report)) => {
                let node_count = mesh.nodes.len();
                let triangle_count = mesh.triangles.len();
                self.mesh = Some(mesh);
                self.report = Some(report);
                self.bc = BoundaryConditions::default();
                self.result = None;
                self.selection.clear();
                self.needs_fit = true;
                self.error = None;
                self.status = format!("已加载 {node_count} 个节点、{triangle_count} 个三角形");
                self.sync_editor_from_selection();
                self.mark_dirty();
            }
            Err(error) => {
                self.error = Some(error.to_string());
                self.status = "STL 加载失败".to_owned();
            }
        }
    }

    fn sync_editor_from_selection(&mut self) {
        let Some(&node) = self.selection.iter().next() else {
            self.edit_constrain_x = false;
            self.edit_constrain_y = false;
            self.edit_ux = 0.0;
            self.edit_uy = 0.0;
            self.edit_fx = 0.0;
            self.edit_fy = 0.0;
            return;
        };
        let dx = fem_solver::DofId(node.0 * 2);
        let dy = fem_solver::DofId(node.0 * 2 + 1);
        self.edit_constrain_x = self.bc.prescribed.contains_key(&dx);
        self.edit_constrain_y = self.bc.prescribed.contains_key(&dy);
        self.edit_ux = self.bc.prescribed.get(&dx).copied().unwrap_or(0.0);
        self.edit_uy = self.bc.prescribed.get(&dy).copied().unwrap_or(0.0);
        self.edit_fx = self.bc.forces.get(&dx).copied().unwrap_or(0.0);
        self.edit_fy = self.bc.forces.get(&dy).copied().unwrap_or(0.0);
    }

    fn apply_constraints(&mut self) {
        if self.selection.is_empty() {
            self.error = Some("请先选择节点".to_owned());
            return;
        }
        for &node in &self.selection {
            let dx = fem_solver::DofId(node.0 * 2);
            let dy = fem_solver::DofId(node.0 * 2 + 1);
            if self.edit_constrain_x {
                self.bc.set_prescribed(dx, self.edit_ux);
            } else {
                self.bc.clear_prescribed(dx);
            }
            if self.edit_constrain_y {
                self.bc.set_prescribed(dy, self.edit_uy);
            } else {
                self.bc.clear_prescribed(dy);
            }
        }
        self.after_bc_change("约束已应用到选中节点");
    }

    fn apply_forces(&mut self) {
        if self.selection.is_empty() {
            self.error = Some("请先选择节点".to_owned());
            return;
        }
        for &node in &self.selection {
            let dx = fem_solver::DofId(node.0 * 2);
            let dy = fem_solver::DofId(node.0 * 2 + 1);
            self.bc.set_force(dx, self.edit_fx);
            self.bc.set_force(dy, self.edit_fy);
        }
        self.after_bc_change("节点力已应用到选中节点");
    }

    fn clear_selected_constraints(&mut self) {
        for &node in &self.selection {
            self.bc.clear_prescribed(fem_solver::DofId(node.0 * 2));
            self.bc.clear_prescribed(fem_solver::DofId(node.0 * 2 + 1));
        }
        self.edit_constrain_x = false;
        self.edit_constrain_y = false;
        self.after_bc_change("已清除选中节点的约束");
    }

    fn clear_selected_forces(&mut self) {
        for &node in &self.selection {
            self.bc.clear_force(fem_solver::DofId(node.0 * 2));
            self.bc.clear_force(fem_solver::DofId(node.0 * 2 + 1));
        }
        self.edit_fx = 0.0;
        self.edit_fy = 0.0;
        self.after_bc_change("已清除选中节点的力");
    }

    fn clear_all_bc(&mut self) {
        self.bc = BoundaryConditions::default();
        self.edit_constrain_x = false;
        self.edit_constrain_y = false;
        self.edit_ux = 0.0;
        self.edit_uy = 0.0;
        self.edit_fx = 0.0;
        self.edit_fy = 0.0;
        self.after_bc_change("已清除全部边界条件");
    }

    fn after_bc_change(&mut self, message: &str) {
        self.result = None;
        self.error = None;
        self.status = message.to_owned();
        self.mark_dirty();
    }

    fn invalidate_result(&mut self) {
        self.result = None;
        self.mark_dirty();
    }

    fn solve_model(&mut self) {
        let Some(_) = self.mesh.as_ref() else {
            self.error = Some("请先加载网格".to_owned());
            return;
        };
        let diagonal = self
            .mesh
            .as_ref()
            .and_then(Mesh::bounds)
            .map_or(1.0, |b| b.diagonal().max(1e-12));

        let solve_result = {
            let mesh = self.mesh.as_ref().expect("checked above");
            solve(mesh, &self.material, &self.bc)
        };

        match solve_result {
            Ok(result) => {
                self.deformation_scale = if result.max_displacement > 1e-30 {
                    (diagonal * 0.10 / result.max_displacement).clamp(0.01, 1e6)
                } else {
                    1.0
                };
                self.status = format!(
                    "求解完成：最大位移 {:.6e}，最大残差 {:.3e}",
                    result.max_displacement, result.max_residual
                );
                self.error = None;
                self.result = Some(result);
                self.mark_dirty();
            }
            Err(error) => {
                self.result = None;
                self.error = Some(error.to_string());
                self.status = "求解失败".to_owned();
                self.mark_dirty();
            }
        }
    }

    fn mark_dirty(&mut self) {
        self.render_revision = self.render_revision.wrapping_add(1);
        self.render_data = Arc::new(build_render_data(
            self.mesh.as_ref(),
            self.result.as_ref(),
            &self.bc,
            &self.selection,
            self.show_deformed,
            self.deformation_scale,
        ));
    }

    fn fit_view(&mut self) {
        self.needs_fit = true;
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("模型");
        ui.horizontal(|ui| {
            ui.text_edit_singleline(&mut self.path_input);
            if ui.button("加载").clicked() {
                self.load_current_path();
            }
        });
        ui.label("也可以把 STL 文件拖入窗口");
        ui.horizontal(|ui| {
            ui.label("丢弃轴");
            egui::ComboBox::from_id_salt("drop_axis")
                .selected_text(self.drop_axis.label())
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.drop_axis, DropAxis::X, "X");
                    ui.selectable_value(&mut self.drop_axis, DropAxis::Y, "Y");
                    ui.selectable_value(&mut self.drop_axis, DropAxis::Z, "Z");
                });
            if ui.button("重新导入").clicked() {
                self.load_current_path();
            }
        });

        if let Some(report) = &self.report {
            ui.small(format!(
                "三角形 {} → {}，顶点 {} → {}，退化 {}，重复 {}",
                report.source_triangles,
                report.imported_triangles,
                report.source_vertices,
                report.welded_vertices,
                report.skipped_degenerate,
                report.skipped_duplicate
            ));
            ui.small(format!(
                "丢 {} 轴跨度 {:.3e}，面内对角线 {:.3e}",
                report.drop_axis.label(),
                report.dropped_span,
                report.in_plane_diagonal
            ));
        }

        ui.separator();
        ui.heading("材料");
        let mut material_changed = false;
        ui.horizontal(|ui| {
            ui.label("E");
            material_changed |= ui
                .add(egui::DragValue::new(&mut self.material.youngs_modulus).speed(100.0))
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("ν");
            material_changed |= ui
                .add(egui::DragValue::new(&mut self.material.poisson_ratio).speed(0.005))
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("厚度");
            material_changed |= ui
                .add_enabled(
                    self.material.model == PlaneModel::PlaneStress,
                    egui::DragValue::new(&mut self.material.thickness).speed(0.1),
                )
                .changed();
        });
        let mut model = self.material.model;
        egui::ComboBox::from_id_salt("plane_model")
            .selected_text(model.label())
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut model, PlaneModel::PlaneStress, "平面应力");
                ui.selectable_value(&mut model, PlaneModel::PlaneStrain, "平面应变");
            });
        if model != self.material.model {
            self.material.model = model;
            material_changed = true;
        }
        if self.material.model == PlaneModel::PlaneStrain {
            ui.small("平面应变按单位厚度计算，厚度输入被忽略。");
        }
        if material_changed {
            self.error = None;
            self.status = "材料已修改，需要重新求解".to_owned();
            self.invalidate_result();
        }
        if let Err(error) = self.material.validate() {
            ui.colored_label(egui::Color32::from_rgb(220, 80, 60), error.to_string());
        }

        ui.separator();
        ui.heading("节点边界条件");
        let selected_nodes: Vec<NodeId> = self.selection.iter().copied().collect();
        if selected_nodes.is_empty() {
            ui.label("在视口中点击节点；Shift/Ctrl 可多选。");
        } else {
            let preview = selected_nodes
                .iter()
                .take(8)
                .map(|n| n.0.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            let suffix = if selected_nodes.len() > 8 { " …" } else { "" };
            ui.label(format!(
                "选中 {} 个节点：{preview}{suffix}",
                selected_nodes.len()
            ));
            ui.small("多个节点时按下面的值批量应用。");

            ui.horizontal(|ui| {
                ui.checkbox(&mut self.edit_constrain_x, "约束 Ux");
                ui.add(egui::DragValue::new(&mut self.edit_ux).speed(0.01));
            });
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.edit_constrain_y, "约束 Uy");
                ui.add(egui::DragValue::new(&mut self.edit_uy).speed(0.01));
            });
            if ui.button("应用约束").clicked() {
                self.apply_constraints();
            }
            if ui.button("清除选中节点约束").clicked() {
                self.clear_selected_constraints();
            }

            ui.horizontal(|ui| {
                ui.label("Fx");
                ui.add(egui::DragValue::new(&mut self.edit_fx).speed(1.0));
                ui.label("Fy");
                ui.add(egui::DragValue::new(&mut self.edit_fy).speed(1.0));
            });
            if ui.button("应用节点力").clicked() {
                self.apply_forces();
            }
            if ui.button("清除选中节点力").clicked() {
                self.clear_selected_forces();
            }
        }
        ui.horizontal(|ui| {
            ui.small(format!("约束 DOF：{}", self.bc.prescribed.len()));
            ui.small(format!("载荷 DOF：{}", self.bc.forces.len()));
        });
        if ui.button("清除全部边界条件").clicked() {
            self.clear_all_bc();
        }

        ui.separator();
        ui.heading("求解");
        if ui
            .add_enabled(
                self.mesh.is_some(),
                egui::Button::new("求解").min_size(egui::vec2(120.0, 28.0)),
            )
            .clicked()
        {
            self.solve_model();
        }
        ui.small("单位：无。请保证 E、几何尺寸、厚度、力使用同一套一致单位。");

        if let Some(result) = &self.result {
            ui.separator();
            ui.heading("结果");
            let max_vm = result
                .stresses
                .iter()
                .map(|s| s.von_mises)
                .fold(0.0_f64, f64::max);
            ui.label(format!("最大位移：{:.6e}", result.max_displacement));
            ui.label(format!("最大残差：{:.3e}", result.max_residual));
            ui.label(format!("最大 von Mises：{:.6e}", max_vm));
        }

        if self.result.is_some() {
            let mut changed = false;
            changed |= ui.checkbox(&mut self.show_deformed, "显示变形").changed();
            ui.horizontal(|ui| {
                ui.label("变形放大");
                changed |= ui
                    .add(egui::DragValue::new(&mut self.deformation_scale).speed(0.1))
                    .changed();
            });
            if changed {
                self.mark_dirty();
            }
        }
    }

    fn viewport_ui(&mut self, ui: &mut egui::Ui) {
        let size = ui.available_size();
        if size.x < 2.0 || size.y < 2.0 {
            return;
        }

        let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());

        if self.mesh.is_none() {
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "加载一个平面 STL 网格，然后选择节点设置约束和力",
                egui::FontId::proportional(18.0),
                egui::Color32::from_gray(150),
            );
            return;
        }

        if self.needs_fit {
            if let Some(bounds) = self.mesh.as_ref().and_then(Mesh::bounds) {
                self.camera.fit(bounds, rect);
            }
            self.needs_fit = false;
        }

        if response.dragged_by(egui::PointerButton::Middle)
            || response.dragged_by(egui::PointerButton::Secondary)
        {
            self.camera.pan(response.drag_delta());
        }

        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.0
                && let Some(cursor) = response.hover_pos()
            {
                let factor = (scroll as f64 * 0.002).exp();
                self.camera.zoom_at(factor, cursor, rect);
            }
        }

        if response.clicked()
            && let Some(cursor) = response.interact_pointer_pos()
        {
            let toggle = ui.input(|i| i.modifiers.shift || i.modifiers.ctrl || i.modifiers.command);
            let hit = self
                .mesh
                .as_ref()
                .and_then(|mesh| pick_node(mesh, &self.camera, rect, cursor, 10.0));
            match hit {
                Some(node) => {
                    if toggle {
                        if !self.selection.remove(&node) {
                            self.selection.insert(node);
                        }
                    } else {
                        self.selection.clear();
                        self.selection.insert(node);
                    }
                    self.sync_editor_from_selection();
                    self.mark_dirty();
                }
                None => {
                    if !toggle && !self.selection.is_empty() {
                        self.selection.clear();
                        self.sync_editor_from_selection();
                        self.mark_dirty();
                    }
                }
            }
        }

        self.cursor_world = response
            .hover_pos()
            .map(|p| self.camera.screen_to_world(p, rect));

        let callback = ViewportCallback {
            data: self.render_data.clone(),
            camera: self.camera,
            viewport: [rect.width(), rect.height()],
            revision: self.render_revision,
        };
        ui.painter()
            .add(egui_wgpu::Callback::new_paint_callback(rect, callback));

        ui.painter().text(
            rect.left_top() + egui::vec2(12.0, 10.0),
            egui::Align2::LEFT_TOP,
            "左键选择节点 · 中键/右键拖拽平移 · 滚轮缩放",
            egui::FontId::proportional(12.0),
            egui::Color32::from_gray(120),
        );
    }
}

impl eframe::App for FemApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let dropped = ui.ctx().input(|i| {
            i.raw
                .dropped_files
                .first()
                .map(|file| file.path().to_path_buf())
        });
        if let Some(path) = dropped {
            self.path_input = path.display().to_string();
            self.load_current_path();
        }

        egui::Panel::top("top").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("FEM 2D Prototype");
                if ui.button("适应视图").clicked() {
                    self.fit_view();
                }
                if ui.button("清除选择").clicked() {
                    self.selection.clear();
                    self.sync_editor_from_selection();
                    self.mark_dirty();
                }
                if let Some(mesh) = &self.mesh {
                    ui.separator();
                    ui.label(format!(
                        "{} 节点 / {} 单元",
                        mesh.nodes.len(),
                        mesh.triangles.len()
                    ));
                }
            });
        });

        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal(|ui| {
                if let Some(mesh) = &self.mesh {
                    ui.label(format!(
                        "{} 节点 / {} 单元",
                        mesh.nodes.len(),
                        mesh.triangles.len()
                    ));
                }
                ui.label("单位：无（保持单位一致）");
                if let Some(p) = self.cursor_world {
                    ui.label(format!("坐标 ({:.4}, {:.4})", p[0], p[1]));
                }
                if let Some(error) = &self.error {
                    ui.colored_label(egui::Color32::from_rgb(235, 90, 70), error);
                } else {
                    ui.label(&self.status);
                }
            });
        });

        egui::Panel::left("left")
            .default_size(330.0)
            .resizable(true)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.side_panel(ui);
                });
            });

        egui::CentralPanel::default().show(ui, |ui| {
            self.viewport_ui(ui);
        });
    }
}
