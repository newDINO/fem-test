use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use eframe::egui;
use eframe::egui_wgpu;
use fem_import::{DropAxis, ImportReport, StlImportOptions, load_stl};
use fem_solver::{
    BoundaryConditions, Material, MaterialAssignment, MaterialExt, MaterialId, Mesh, Model, NodeId,
    PlaneModel, SolveResult, solve,
};
use fem_types::format::{self, File as ProjectFile, Results};
use rfd::FileDialog;

use crate::camera::Camera2d;
use crate::picking::{pick_node, pick_nodes_in_rect};
use crate::renderer::{RenderData, ViewportCallback, ViewportGpu, build_render_data};

fn plane_model_label(model: PlaneModel) -> &'static str {
    match model {
        PlaneModel::PlaneStress => "平面应力",
        PlaneModel::PlaneStrain => "平面应变",
    }
}

fn material_assignment_label(assignment: &MaterialAssignment) -> String {
    match assignment {
        MaterialAssignment::Uniform(material) => format!("均一（材料 #{}）", material.0),
        MaterialAssignment::PerElement(materials) => {
            format!("逐单元（{} 个单元）", materials.len())
        }
    }
}

/// Force a `.fem` extension without clobbering a name that already has one.
fn ensure_fem_extension(path: &Path) -> PathBuf {
    let has_fem_extension = path
        .extension()
        .and_then(|extension| extension.to_str())
        .is_some_and(|extension| extension.eq_ignore_ascii_case("fem"));
    if has_fem_extension {
        path.to_path_buf()
    } else {
        path.with_extension("fem")
    }
}

pub struct FemApp {
    drop_axis: DropAxis,
    stl_source: Option<PathBuf>,
    project_path: Option<PathBuf>,
    last_directory: Option<PathBuf>,
    mesh: Option<Mesh>,
    report: Option<ImportReport>,
    plane_model: PlaneModel,
    materials: Vec<Material>,
    material_index: usize,
    element_materials: MaterialAssignment,
    bc: BoundaryConditions,
    result: Option<SolveResult>,
    selection: BTreeSet<NodeId>,
    box_select_origin: Option<egui::Pos2>,
    box_select_current: Option<egui::Pos2>,
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
            drop_axis: DropAxis::Z,
            stl_source: None,
            project_path: None,
            last_directory: None,
            mesh: None,
            report: None,
            plane_model: PlaneModel::PlaneStress,
            materials: vec![Material::default()],
            material_index: 0,
            element_materials: MaterialAssignment::Uniform(MaterialId(0)),
            bc: BoundaryConditions::default(),
            result: None,
            selection: BTreeSet::new(),
            box_select_origin: None,
            box_select_current: None,
            camera: Camera2d::default(),
            needs_fit: true,
            show_deformed: true,
            deformation_scale: 1.0,
            render_data: Arc::new(RenderData::default()),
            render_revision: 0,
            cursor_world: None,
            status: "打开 .fem 项目或 .stl 网格开始".to_owned(),
            error: None,
            edit_constrain_x: false,
            edit_constrain_y: false,
            edit_ux: 0.0,
            edit_uy: 0.0,
            edit_fx: 0.0,
            edit_fy: 0.0,
        };

        if Path::new("assets/plate.stl").exists() {
            app.load_stl_path(Path::new("assets/plate.stl"));
        }
        app
    }

    fn open_dialog(&mut self) {
        let mut dialog = FileDialog::new()
            .add_filter("FEM 项目 / STL 网格", &["fem", "stl"])
            .add_filter("FEM 项目", &["fem"])
            .add_filter("STL 网格", &["stl"]);
        if let Some(directory) = &self.last_directory {
            dialog = dialog.set_directory(directory);
        }
        if let Some(path) = dialog.pick_file() {
            self.open_path(path);
        }
    }

    fn save(&mut self) {
        if self.mesh.is_none() {
            self.error = Some("没有可保存的网格".to_owned());
            return;
        }
        match self.project_path.clone() {
            Some(path) => self.save_to_path(&path),
            None => self.save_dialog(),
        }
    }

    fn save_dialog(&mut self) {
        if self.mesh.is_none() {
            self.error = Some("没有可保存的网格".to_owned());
            return;
        }

        let mut dialog = FileDialog::new()
            .add_filter("FEM 项目", &["fem"])
            .set_file_name(self.suggested_file_name());
        if let Some(directory) = &self.last_directory {
            dialog = dialog.set_directory(directory);
        }
        if let Some(path) = dialog.save_file() {
            self.save_to_path(&path);
        }
    }

    fn suggested_file_name(&self) -> String {
        if let Some(path) = &self.project_path
            && let Some(name) = path.file_name()
        {
            return name.to_string_lossy().into_owned();
        }
        if let Some(path) = &self.stl_source
            && let Some(stem) = path.file_stem()
        {
            return format!("{}.fem", stem.to_string_lossy());
        }
        "project.fem".to_owned()
    }

    fn open_path(&mut self, path: PathBuf) {
        let extension = path
            .extension()
            .and_then(|value| value.to_str())
            .map(|value| value.to_ascii_lowercase());

        match extension.as_deref() {
            Some("fem") => self.load_fem(&path),
            Some("stl") => self.load_stl_path(&path),
            _ => {
                self.error = Some(format!(
                    "无法识别 {}：请选择 .fem 项目或 .stl 网格",
                    path.display()
                ));
                self.status = "未加载文件".to_owned();
            }
        }
    }

    fn load_stl_path(&mut self, path: &Path) {
        let options = StlImportOptions {
            drop_axis: self.drop_axis,
            weld_tolerance: None,
        };
        match load_stl(path, options) {
            Ok((mesh, report)) => {
                let node_count = mesh.nodes.len();
                let triangle_count = mesh.triangles.len();
                self.mesh = Some(mesh);
                self.report = Some(report);
                self.ensure_material_library();
                self.element_materials = MaterialAssignment::Uniform(MaterialId(0));
                self.bc = BoundaryConditions::default();
                self.result = None;
                self.selection.clear();
                self.needs_fit = true;
                self.stl_source = Some(path.to_path_buf());
                self.project_path = None;
                self.last_directory = path.parent().map(Path::to_path_buf);
                self.error = None;
                self.status = format!(
                    "已从 {} 加载 {node_count} 个节点、{triangle_count} 个三角形",
                    path.display()
                );
                self.sync_editor_from_selection();
                self.mark_dirty();
            }
            Err(error) => {
                self.error = Some(error.to_string());
                self.status = "STL 加载失败".to_owned();
            }
        }
    }

    fn reload_stl(&mut self) {
        if let Some(path) = self.stl_source.clone() {
            // Re-importing rebuilds the mesh, but the user may have already
            // saved this model as a project; keep Ctrl+S pointing at that file.
            let project_path = self.project_path.clone();
            self.load_stl_path(&path);
            if self.error.is_none() {
                self.project_path = project_path;
            }
        }
    }

    fn load_fem(&mut self, path: &Path) {
        let file = match format::read_from_path(path) {
            Ok(file) => file,
            Err(error) => {
                self.error = Some(error.to_string());
                self.status = "项目文件读取失败".to_owned();
                return;
            }
        };

        let node_count = file.mesh.nodes.len();
        let triangle_count = file.mesh.triangles.len();

        let model = match file.model() {
            Ok(Some(model)) => model,
            Ok(None) => Model::uniform(
                file.mesh.clone(),
                PlaneModel::PlaneStress,
                Material::default(),
            ),
            Err(error) => {
                self.error = Some(error.to_string());
                self.status = "项目文件中的材料设置无效".to_owned();
                return;
            }
        };

        // A `.fem` file can come from an external generator, so validate the
        // topology before handing it to the renderer (which indexes nodes
        // directly).
        if let Err(error) = model.mesh.validate() {
            self.error = Some(error.to_string());
            self.status = "项目文件中的网格无效".to_owned();
            return;
        }

        let bc = match &file.boundary {
            Some(boundary) => match BoundaryConditions::from_file(boundary, node_count) {
                Ok(bc) => bc,
                Err(error) => {
                    self.error = Some(error.to_string());
                    self.status = "项目文件中的边界条件无效".to_owned();
                    return;
                }
            },
            None => BoundaryConditions::default(),
        };

        let result = match &file.results {
            Some(results) => match results.to_solve(&model) {
                Ok(result) => Some(result),
                Err(error) => {
                    self.error = Some(error.to_string());
                    self.status = "项目文件中的结果无效".to_owned();
                    return;
                }
            },
            None => None,
        };

        let deformation_scale = result
            .as_ref()
            .map(|result| Self::deformation_scale_for(&model.mesh, result));

        let Model {
            mesh,
            plane_model,
            materials,
            element_materials,
        } = model;
        self.mesh = Some(mesh);
        self.plane_model = plane_model;
        self.materials = materials;
        self.material_index = 0;
        self.element_materials = element_materials;
        self.bc = bc;
        self.result = result;
        self.report = None;
        self.selection.clear();
        self.needs_fit = true;
        self.stl_source = None;
        self.project_path = Some(path.to_path_buf());
        self.last_directory = path.parent().map(Path::to_path_buf);
        if let Some(scale) = deformation_scale {
            self.deformation_scale = scale;
        }

        self.error = None;
        self.status = match &self.result {
            Some(_) => format!(
                "已打开 {}：{node_count} 个节点、{triangle_count} 个三角形（含结果）",
                path.display()
            ),
            None => format!(
                "已打开 {}：{node_count} 个节点、{triangle_count} 个三角形",
                path.display()
            ),
        };
        self.sync_editor_from_selection();
        self.mark_dirty();
    }

    fn save_to_path(&mut self, path: &Path) {
        let Some(model) = self.build_model() else {
            self.error = Some("没有可保存的网格".to_owned());
            return;
        };

        let path = ensure_fem_extension(path);
        let mut file = ProjectFile::from_model(model).with_boundary(self.bc.to_file());
        if let Some(result) = &self.result {
            file = file.with_results(Results::from_solve(result));
        }

        match format::write_to_path(&path, &file) {
            Ok(()) => {
                self.project_path = Some(path.clone());
                self.last_directory = path.parent().map(Path::to_path_buf);
                self.error = None;
                self.status = format!("已保存到 {}", path.display());
            }
            Err(error) => {
                self.error = Some(error.to_string());
                self.status = "保存失败".to_owned();
            }
        }
    }

    fn build_model(&self) -> Option<Model> {
        self.mesh.clone().map(|mesh| Model {
            mesh,
            plane_model: self.plane_model,
            materials: self.materials.clone(),
            element_materials: self.element_materials.clone(),
        })
    }

    fn ensure_material_library(&mut self) {
        if self.materials.is_empty() {
            self.materials.push(Material::default());
        }
        self.material_index = self.material_index.min(self.materials.len() - 1);
    }

    fn deformation_scale_for(mesh: &Mesh, result: &SolveResult) -> f64 {
        let diagonal = mesh
            .bounds()
            .map_or(1.0, |bounds| bounds.diagonal().max(1e-12));
        if result.max_displacement > 1e-30 {
            (diagonal * 0.10 / result.max_displacement).clamp(0.01, 1e6)
        } else {
            1.0
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
        let diagonal = self
            .mesh
            .as_ref()
            .and_then(Mesh::bounds)
            .map_or(1.0, |b| b.diagonal().max(1e-12));

        let solve_result = match self.build_model() {
            Some(model) => solve(&model, &self.bc),
            None => {
                self.error = Some("请先加载网格".to_owned());
                return;
            }
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

    fn apply_box_selection(&mut self, rect: egui::Rect, viewport: egui::Rect, add: bool) {
        let Some(mesh) = self.mesh.as_ref() else {
            return;
        };
        let hits = pick_nodes_in_rect(mesh, &self.camera, viewport, rect);
        if !add {
            self.selection.clear();
        }
        self.selection.extend(hits);
        self.sync_editor_from_selection();
        self.mark_dirty();
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("文件");
        ui.horizontal(|ui| {
            if ui.button("打开…").clicked() {
                self.open_dialog();
            }
            let has_mesh = self.mesh.is_some();
            if ui
                .add_enabled(has_mesh, egui::Button::new("保存"))
                .clicked()
            {
                self.save();
            }
            if ui
                .add_enabled(has_mesh, egui::Button::new("另存为…"))
                .clicked()
            {
                self.save_dialog();
            }
        });
        let current = match (&self.project_path, &self.stl_source) {
            (Some(path), _) => format!("当前项目：{}", path.display()),
            (None, Some(path)) => format!("当前网格（未保存）：{}", path.display()),
            (None, None) => "当前没有打开文件".to_owned(),
        };
        ui.small(current);
        ui.small("支持 .fem 项目与 .stl 平面网格，也可以直接拖入窗口。");
        ui.small("快捷键：Ctrl/Cmd+O 打开，Ctrl/Cmd+S 保存。");

        if self.stl_source.is_some() {
            ui.horizontal(|ui| {
                ui.label("STL 丢弃轴");
                egui::ComboBox::from_id_salt("drop_axis")
                    .selected_text(self.drop_axis.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.drop_axis, DropAxis::X, "X");
                        ui.selectable_value(&mut self.drop_axis, DropAxis::Y, "Y");
                        ui.selectable_value(&mut self.drop_axis, DropAxis::Z, "Z");
                    });
                if ui.button("重新导入 STL").clicked() {
                    self.reload_stl();
                }
            });
            ui.small("重新导入会按当前丢弃轴重建网格，并清除边界条件与结果。");
        }

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
        self.ensure_material_library();
        if self.materials.len() > 1 {
            ui.horizontal(|ui| {
                ui.label("编辑材料");
                egui::ComboBox::from_id_salt("material_index")
                    .selected_text(format!("#{}", self.material_index))
                    .show_ui(ui, |ui| {
                        for index in 0..self.materials.len() {
                            ui.selectable_value(
                                &mut self.material_index,
                                index,
                                format!("#{index}"),
                            );
                        }
                    });
            });
        }
        ui.small(format!(
            "材料库 {} 个；单元分配：{}",
            self.materials.len(),
            material_assignment_label(&self.element_materials)
        ));

        let plane_model = self.plane_model;
        let material_index = self.material_index.min(self.materials.len() - 1);
        let mut material_changed = false;
        {
            let material = &mut self.materials[material_index];
            if let Some(isotropic) = material.isotropic_material_mut() {
                ui.horizontal(|ui| {
                    ui.label("E");
                    material_changed |= ui
                        .add(egui::DragValue::new(&mut isotropic.youngs_modulus).speed(100.0))
                        .changed();
                });
                ui.horizontal(|ui| {
                    ui.label("ν");
                    material_changed |= ui
                        .add(egui::DragValue::new(&mut isotropic.poisson_ratio).speed(0.005))
                        .changed();
                });
                ui.horizontal(|ui| {
                    ui.label("厚度");
                    material_changed |= ui
                        .add_enabled(
                            plane_model == PlaneModel::PlaneStress,
                            egui::DragValue::new(&mut isotropic.thickness).speed(0.1),
                        )
                        .changed();
                });
            }
        }

        let mut selected_plane_model = self.plane_model;
        egui::ComboBox::from_id_salt("plane_model")
            .selected_text(plane_model_label(selected_plane_model))
            .show_ui(ui, |ui| {
                for option in [PlaneModel::PlaneStress, PlaneModel::PlaneStrain] {
                    ui.selectable_value(
                        &mut selected_plane_model,
                        option,
                        plane_model_label(option),
                    );
                }
            });
        if selected_plane_model != self.plane_model {
            self.plane_model = selected_plane_model;
            material_changed = true;
        }
        ui.small("平面应力 / 平面应变是全局分析设置。");
        if self.plane_model == PlaneModel::PlaneStrain {
            ui.small("平面应变按单位厚度计算，厚度输入被忽略。");
        }
        if material_changed {
            self.error = None;
            self.status = "材料已修改，需要重新求解".to_owned();
            self.invalidate_result();
        }
        for (index, material) in self.materials.iter().enumerate() {
            if let Err(error) = material.validate(self.plane_model) {
                ui.colored_label(
                    egui::Color32::from_rgb(220, 80, 60),
                    format!("材料 #{index}：{error}"),
                );
            }
        }

        ui.separator();
        ui.heading("节点边界条件");
        let selected_nodes: Vec<NodeId> = self.selection.iter().copied().collect();
        if selected_nodes.is_empty() {
            ui.label("在视口中点击节点或拖拽框选；Shift/Ctrl 可追加选择。");
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
                "打开 .fem 项目或 .stl 平面网格，然后选择节点设置约束和力",
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

        let toggle = ui.input(|i| i.modifiers.shift || i.modifiers.ctrl || i.modifiers.command);

        if response.drag_started_by(egui::PointerButton::Primary) {
            let origin = ui.input(|i| i.pointer.press_origin());
            self.box_select_origin = origin;
            self.box_select_current = origin.or_else(|| response.interact_pointer_pos());
        }
        if response.dragged_by(egui::PointerButton::Primary)
            && let Some(cursor) = response.interact_pointer_pos()
        {
            self.box_select_current = Some(cursor);
        }
        if response.drag_stopped_by(egui::PointerButton::Primary) {
            let origin = self.box_select_origin.take();
            let end = self.box_select_current.take();
            if let (Some(origin), Some(end)) = (origin, end) {
                self.apply_box_selection(egui::Rect::from_two_pos(origin, end), rect, toggle);
            }
        }

        if response.clicked()
            && let Some(cursor) = response.interact_pointer_pos()
        {
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

        if response.dragged_by(egui::PointerButton::Primary)
            && let (Some(origin), Some(current)) = (self.box_select_origin, self.box_select_current)
        {
            let selection_rect = egui::Rect::from_two_pos(origin, current).intersect(rect);
            if selection_rect.is_positive() {
                ui.painter().rect(
                    selection_rect,
                    0.0,
                    egui::Color32::from_rgba_unmultiplied(80, 140, 255, 28),
                    egui::Stroke::new(1.0, egui::Color32::from_rgb(120, 175, 255)),
                    egui::StrokeKind::Inside,
                );
            }
        }

        ui.painter().text(
            rect.left_top() + egui::vec2(12.0, 10.0),
            egui::Align2::LEFT_TOP,
            "左键点击/拖拽框选节点 · Shift/Ctrl 追加 · 中键/右键平移 · 滚轮缩放",
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
            self.open_path(path);
        }

        let (open_shortcut, save_shortcut) = ui.ctx().input(|i| {
            (
                i.modifiers.command && i.key_pressed(egui::Key::O),
                i.modifiers.command && i.key_pressed(egui::Key::S),
            )
        });
        if open_shortcut {
            self.open_dialog();
        }
        if save_shortcut {
            self.save();
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

#[cfg(test)]
mod tests {
    use super::ensure_fem_extension;
    use std::path::Path;

    #[test]
    fn saved_projects_always_use_the_fem_extension() {
        assert_eq!(
            ensure_fem_extension(Path::new("plate")),
            Path::new("plate.fem")
        );
        assert_eq!(
            ensure_fem_extension(Path::new("plate.stl")),
            Path::new("plate.fem")
        );
        assert_eq!(
            ensure_fem_extension(Path::new("plate.FEM")),
            Path::new("plate.FEM")
        );
    }
}
