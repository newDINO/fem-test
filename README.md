# FEM Test — 2D 线弹性有限元原型

一个用于机械结构二维静力分析的实验性有限元软件原型。网格来自 Blender 等建模软件导出的 STL 平面薄片，也可以由外部程序用 `fem-types` 生成 `.fem` 项目；共享类型、外部格式导入、求解器、桌面界面与 Web 入口分为五个 crate。

当前版本只做**二维、小变形、线弹性、静力**分析，单元只有**三节点常应变三角形（CST）**。界面可以用 STL 导入网格，也可以直接打开/保存 `fem-types` 的 `.fem` 项目文件（网格 + 材料 + 边界条件 + 结果）。

## 功能

- 导入 STL 平面网格，可选择丢弃 X / Y / Z 轴，把剩下的两个坐标当作 2D 平面。
  - 自动按容差焊接重复顶点、删除重复三角形和退化三角形。
  - 要求网格是**单个、轴对齐的平面面片**；非平面或多层网格会被拒绝并给出原因。
- 打开 / 保存 `.fem` 项目文件（网格、材料、边界条件和求解结果）。桌面版使用系统原生文件选择器（`rfd`）；Web 版通过浏览器上传 / 下载文件，内容在内存中处理。
- 手动节点边界条件：
  - 每个节点、每个自由度（Ux、Uy）的位移约束，可给非零指定位移。
  - 每个节点、每个自由度的集中力。
  - 支持点击/拖拽框选多选（Shift/Ctrl 追加）后批量施加。
- 线性多点约束 `u_slave = u_master + offset`，可用于周期性边界条件（RUC）。约束在全局刚度矩阵上消元，两个节点不需要属于同一单元。目前通过 `.fem` 文件添加，GUI 只显示数量、尚未提供编辑和自动配对。
- 材料参数：杨氏模量 E、泊松比 ν、厚度 t；材料库支持多个材料，单元可以按索引引用不同材料。平面应力 / 平面应变是全局分析设置。
- 求解后在视口中显示：
  - 变形后的网格（可调放大倍数、可关闭）；
  - 按 von Mises 应力着色的三角形；
  - 约束标记（橙色菱形）和节点力箭头（红色）。
- 单位完全由用户负责：界面不做任何单位换算，只要求 E、几何、厚度、力使用同一套一致单位。

## 快速开始
```bash
# 需要 Rust 1.95+（eframe 0.37 的要求）
cargo run -p fem-gui
```
默认的 `dev profile` 已经开启了O1优化。
如果需要更多优化，可使用
```bash
cargo run -p fem-gui --release
```

首次启动会自动加载编译进程序的 `assets/plate.stl`（10×4 的矩形板，120 个三角形）。也可以：

1. 点左侧“打开…”选择 `.fem` 项目或 `.stl` 网格（`Ctrl/Cmd+O`）；
2. 直接把 `.fem` / `.stl` 文件拖进窗口。

求解后点“保存”写回当前 `.fem`（`Ctrl/Cmd+S`），或用“另存为…”选择新路径。若当前模型只来自 STL，第一次保存会弹出对话框并建议同名 `.fem`。

### Web 演示

```bash
rustup target add wasm32-unknown-unknown
cargo install wasm-pack
cargo install wasm-bindgen-cli   # 版本需与 Cargo.lock 中的 wasm-bindgen 一致
./crates/web/build.sh
python3 -m http.server 8000 --directory crates/web
# 打开 http://localhost:8000/
```

`build.sh` 使用 `wasm-pack`，并传入 `--mode no-install`，因此不会从 GitHub 下载 wasm-bindgen；`--no-opt` 同样避免下载 wasm-opt（如果本机已有 binaryen，可以删掉它来减小体积）。如果你使用的 wasm-pack 版本支持 `--no-install`，可以等价替换 `--mode no-install`。

Web 版与桌面版共用 `fem-gui` 的全部界面代码。浏览器环境没有文件系统路径，所以：

- 打开文件通过上传，读取完成后以字节形式保存在内存里；
- 保存通过浏览器下载 `.fem`，没有“覆盖当前路径”的概念，每次保存都会走保存对话框；
- 浏览器不支持重新读取已下载的项目文件，因此“重新导入 STL”只对本次会话内上传/内置的 STL 有效；
- 渲染使用 wgpu，需要支持 **WebGPU** 的浏览器（Chrome/Edge 113+，或已启用 WebGPU 的 Firefox/Safari），并且页面必须通过 HTTP(S) 打开。

示例操作（悬臂梁）：

1. 用左键点选左边界上的若干节点，勾选“约束 Ux”“约束 Uy”，点“应用约束”；
2. 点选右边界节点，设置 `Fy` 为负值（向下），点“应用节点力”；
3. 点“求解”。

视口操作：

| 操作 | 功能 |
| --- | --- |
| 左键点击节点 | 选择单个节点；Shift/Ctrl 点击可多选 |
| 左键拖拽 | 框选矩形内的节点；Shift/Ctrl 拖拽可追加选择 |
| 中键 / 右键拖拽 | 平移视图 |
| 滚轮 | 以光标为中心缩放 |
| “适应视图” | 自动缩放到网格范围 |

## 工程结构

```
Cargo.toml                 # workspace
crates/types/              # fem-types：网格/材料/结果结构体、网格校验 + 项目文件格式（无 faer/nalgebra）
crates/import/             # fem-import：STL 等外部格式导入（只依赖 fem-types）
crates/solver/             # fem-solver：材料、CST、组装、求解
crates/gui/                # fem-gui：eframe/egui 界面 + 自定义 wgpu 视口（lib + 桌面 bin）
crates/web/                # fem-web：wasm 入口、index.html、wasm-pack 构建脚本
assets/plate.stl           # 示例平面网格（编译进程序）
```

### types

| 文件 | 内容 |
| --- | --- |
| `mesh.rs` | `NodeId` / `ElemId` / `Node` / `Mesh` / `Bounds2`、纯几何辅助、`Mesh::validate` / `MeshError` |
| `material.rs` | `Material` enum、各向同性参数 `IsotropicMaterial`、`MaterialId` |
| `model.rs` | `PlaneModel`（全局平面假设）、`MaterialAssignment`、`Model` |
| `result.rs` | `SolveResult` / `ElementStress`、von Mises 计算 |
| `format.rs` | 文件头、MessagePack 读写、节点级边界条件、结果互转 |

`solver` 通过 `pub use` 重新导出这些类型，原有 `fem_solver::Mesh` 等路径保持不变；网格校验是不依赖线性代数的纯几何逻辑，放在 `fem-types` 供导入器和求解器共用；`MaterialExt::validate`、`MaterialExt::d_matrix` 这些依赖求解器错误类型或线性代数的逻辑留在 `solver`。

### import

| 文件 | 内容 |
| --- | --- |
| `stl.rs` | STL 读取、轴对齐平面校验、顶点焊接、重复面删除 |

`fem-import` 只依赖 `fem-types` 和 `stl_io`，不引入 faer/nalgebra；GUI 直接使用它导入 STL，求解器不再包含文件解析代码。

### solver

| 文件 | 内容 |
| --- | --- |
| `mesh.rs` | 重新导出 `fem-types` 的网格类型 |
| `material.rs` | 按全局平面假设生成 D 矩阵、材料校验 |
| `dof.rs` | 2 DOF/节点的自由度编号 |
| `bc.rs` | 位移约束与节点力容器 |
| `element/tri3.rs` | CST 的 B 矩阵、单元刚度、应力恢复 |
| `assemble.rs` | 全局刚度三元组组装 |
| `solve.rs` | 自由自由度缩减、约束反力、应力、残差 |
| `result.rs` | 位移、反力、单元应力、von Mises |

实现要点：

- 全局方程组用 `faer` 的稀疏 LU（`SparseColMat::sp_lu`）求解，便于以后换 Cholesky；
- 小矩阵（3×3、6×6）用 `nalgebra` 的固定尺寸类型；
- 约束处理采用**自由自由度缩减**，同时保留完整刚度矩阵用于计算约束反力 `r = K u − f`；
- 平面应力使用厚度，平面应变按单位厚度计算并输出非零 `σz`；
- 每个单元通过 `MaterialAssignment` 引用材料库；`Uniform` 分配不会逐单元展开，`PerElement` 则要求与三角形数一致。

### gui

`fem-gui` 同时提供库和桌面二进制：`src/lib.rs` 导出界面模块，`src/main.rs` 只负责创建原生窗口；`fem-web` 复用同一个库。

| 文件 | 内容 |
| --- | --- |
| `app.rs` | 应用状态、文件任务轮询、侧栏、视口交互、求解流程 |
| `camera.rs` | 2D 平移/缩放/自适应视图 |
| `picking.rs` | 屏幕空间节点拾取 |
| `platform/mod.rs` | 平台无关的 `FileRef` / `FileTask` / `TaskOutcome` |
| `platform/native.rs` | 原生同步文件对话框（`rfd::FileDialog`）与文件读写 |
| `platform/web.rs` | 浏览器异步文件对话框（`rfd::AsyncFileDialog`） |
| `renderer.rs` | 自定义 wgpu 管线、顶点缓冲、egui 回调 |
| `render_shader.wgsl` | 顶点/片元着色器 |

渲染没有使用 `egui::Mesh` 画三角形，而是通过 `egui_wgpu::CallbackTrait` 在 eframe 共享的 wgpu 设备上绘制自己的三角形/线段管线。三角形填充和线段（网格边、节点标记、约束、力箭头）各有一条管线。

文件对话框使用 `rfd`。原生版在 Linux 上默认走 `xdg-desktop-portal` 后端，不需要 GTK 开发头文件，但需要桌面环境实际提供 portal 服务。Web 版使用 `rfd::AsyncFileDialog`：浏览器只提供异步文件 API，所以 `platform/web.rs` 把选择器和读取/下载过程包装成 `FileTask`。`app.rs` 每帧轮询这个任务（最多一个），并在等待期间请求重绘；打开文件选择器的第一次轮询发生在点击处理的同一帧，以满足浏览器的 transient user activation 要求。`Ctrl/Cmd+O` / `Ctrl/Cmd+S` 与侧栏按钮等价。

### web

| 文件 | 内容 |
| --- | --- |
| `src/lib.rs` | `wasm-bindgen` 暴露的 `WebHandle`，启动 `eframe::WebRunner` |
| `index.html` | 全屏 canvas、加载 `pkg/fem_web.js`、WebGPU 失败提示 |
| `build.sh` | `wasm-pack --target web --mode no-install --no-opt` 构建脚本 |

`fem-web` 只负责装配 wasm 入口；文件、渲染和求解逻辑全部来自 `fem-gui`、`fem-solver` 与 `fem-types`。`fem-types::format` 的路径 API 在 wasm 下被 `cfg` 掉，桌面和 Web 共用的是 `to_bytes` / `read_bytes`。

中文字体说明：egui 内置字体不含中日韩字形。`eframe` 启用了 `system_fonts` feature，运行时会通过操作系统的字体枚举（fontconfig / DirectWrite / CoreText）按字符和 locale 按需加载系统中文字体作为回退；字体文件以内存映射方式读取，不会打包进程序。若系统本身没有安装中文字体（或 Linux 上缺少可用的 fontconfig 配置），中文会显示为占位方框。Web 版由 egui 在浏览器中的字体回退处理，不需要额外打包字体。

## STL 导入约束

STL 不保存拓扑，也没有统一顶点表，因此导入器会：

1. 解析 ASCII 或二进制 STL；
2. 检查每个三角形法向是否平行于被丢弃的轴，任意一个不平行就报错；
3. 检查被丢弃轴上的跨度是否远小于面内尺寸，否则判为非平面；
4. 把剩余两个坐标投影到 XY，以 `面内对角线 × 1e-6` 为默认容差焊接顶点；
5. 丢弃投影面积接近零的三角形和重复面；
6. 保证三角形绕向为逆时针。

因此输入应当是从一个**平面薄片**导出的三角网格，而不是一个封闭的 3D 实体。如果导出的是实体，请先在 Blender 里只保留要分析的那个面（或分离出该面）再导出。

## 单位约定（无单位）

界面和求解器都不假设任何单位制。例如：

- 长度用 mm、E 用 MPa、力用 N，则应力结果是 MPa；
- 长度用 m、E 用 Pa、力用 N，则应力结果是 Pa。

只要 E、几何尺寸、厚度、力保持同一套一致单位，结果就一致。平面应变忽略厚度输入。

## 项目文件格式（fem-types）

`fem-types` 是求解器和外部工具共用的数据契约，只依赖 serde / rmp-serde，不引入 faer 或 nalgebra。用 Rust 写网格生成器时只依赖它即可；需要直接读 STL 时再加 `fem-import`（同样不依赖求解器）：

```toml
[dependencies]
fem-types = { path = "../fem-test/crates/types" }
```

```rust
use fem_types::{format, Material, Model, PlaneModel};

let model = Model::uniform(mesh, PlaneModel::PlaneStress, Material::default());
format::write_to_path("plate.fem", &format::File::from_model(model))?;
```

需要多材料时，材料库和单元分配分开写：

```rust
use fem_types::{MaterialAssignment, MaterialId};

let mut model = Model::uniform(mesh, PlaneModel::PlaneStress, Material::default());
model.materials.push(Material::isotropic(70_000.0, 0.33, 1.0));
model.element_materials = MaterialAssignment::PerElement(vec![
    MaterialId(0), MaterialId(1), /* 每个三角形一项 */
]);
```

周期性边界条件通过 `boundary.links` 写入。每个 link 是一个标量关系
`u_slave = u_master + offset`；对右边界节点和它在左边界的对应节点，offset
就是宏观应变乘以节点间距：

```rust
use fem_types::format::{Boundary, DofLink, DofRef, DofValue};

let exy = 0.5 * gxy; // 工程剪应变 γxy 对应 εxy = γxy / 2
let boundary = Boundary {
    prescribed: vec![
        // 固定平移 gauge：参考点取宏观仿射场在该点的位移
        DofValue {
            node: reference,
            component: 0,
            value: exx * x_ref + exy * y_ref,
        },
        DofValue {
            node: reference,
            component: 1,
            value: exy * x_ref + eyy * y_ref,
        },
    ],
    forces: Vec::new(),
    links: vec![
        DofLink {
            slave: DofRef { node: right, component: 0 },
            master: DofRef { node: left, component: 0 },
            // u_slave - u_master = εxx * (x_right - x_left)
            offset: exx * width,
        },
        DofLink {
            slave: DofRef { node: right, component: 1 },
            master: DofRef { node: left, component: 1 },
            offset: exy * width,
        },
    ],
};
format::write_to_path("ruc.fem", &format::File::from_model(model).with_boundary(boundary))?;
```

完整 RUC 还要对上下边界做同样的处理（offset 由 `ε̄ · (0, height)` 得到）。求解器会把
link 和 prescribed 一起做自由度消元；角点上重复的 x/y 周期约束只要 offset 一致就会
自动合并，不一致会返回 `ConstraintConflict`。

文件布局：

```text
4 字节 magic "FEMS" + 2 字节小端版本号 + MessagePack 负载
```

- 负载使用 `rmp-serde` 的命名模式（struct 编码为 map），字段缺失时更容易发现；
- 一个文件可以只含 `mesh`（网格生成器不需要知道材料），也可以是一个完整模型：`plane_model`、`materials`、`element_materials` 必须同时存在；
- `PlaneModel`（平面应力 / 平面应变）是全局分析设置，不属于单个材料；
- 材料库是 `Vec<Material>`；`MaterialAssignment` 有两种等价表示：`Uniform(MaterialId)` 和 `PerElement(Vec<MaterialId>)`。uniform 变体显式保存材料 id，不使用“空数组代表材料 0”的隐式约定；
- 边界条件按**节点 + 分量**存储（`0 = Ux, 1 = Uy, 2 = Uz`），不依赖求解器内部的 DOF 编号；
- 多点约束保存在 `boundary.links` 中，每项是 `u_slave = u_master + offset`。offset 显式保存，因此一个文件对应一个完整载荷工况；周期边界生成器可以按宏观应变重建 offset；
- 结果按节点/单元顺序保存原始量，von Mises、σz、最大位移在读取时按全局平面假设和逐单元材料重算，避免文件内部自相矛盾；
- 读取时检查 magic、版本号和负载后的多余字节；版本不匹配会返回 `FormatError::UnsupportedVersion`，不会静默解析出错；
- 格式仍在早期开发阶段，版本号暂时固定为 **1**，字段还可能变化，不保证旧文件兼容。

| 函数 | 用途 |
| --- | --- |
| `format::read` / `read_bytes` | 从 reader / 内存读取 |
| `format::write` | 写到任意 writer |
| `format::read_from_path` / `write_to_path` | 路径版本，写入使用临时文件 + rename |
| `File::from_model` / `File::model` | 完整模型与文件表示互转（mesh-only 文件返回 `None`） |
| `Results::from_solve` / `to_solve` | 求解结果与文件表示互转 |
| `BoundaryConditions::to_file` / `from_file` | 求解器边界条件与文件表示互转 |

GUI 的“打开…”同时接受 `.fem` 和 `.stl`：`.fem` 会恢复网格、材料库、逐单元材料分配、边界条件以及（若存在）结果，`.stl` 则走导入流程并清空边界条件与材料分配（材料库保留）。保存时始终写 `.fem`，第一次保存会建议与 STL 同名的文件名。

## 已知限制 / 后续方向

- 只支持 2D、3 节点三角形、小变形线弹性静力；
- 材料目前只有各向同性（`Material::Isotropic`），但 enum 已为各向异性等类型预留扩展点；
- GUI 可以编辑材料库中的每个材料，但还没有“选择单元并赋予材料”的交互；逐单元分配目前主要来自外部 `.fem` 文件，GUI 会保留并求解；
- 周期约束目前需要外部程序生成 `.fem` 文件；GUI 还没有边界自动配对、宏观应变加载和均匀化后处理；
- 无网格生成、无分布载荷、无重力、无接触、无非线性；
- 打开 `.fem` 时会对网格做拓扑校验，但结果只检查尺寸一致性、不重新求解（避免“以文件为准”和“重算”产生分歧）；
- 视角为固定 XY 平面，没有 3D 旋转；
- 渲染的分级颜色只按 von Mises 应力，色标范围取当前结果的最小/最大值；
- 后续若要扩展 3D 或其他单元，`Mesh` 已保留 3D 坐标，`dof` 与单元/组装层可分别扩展；
- 3D 阶段需要离屏颜色/深度附件或由 egui 渲染器提供深度，当前 2D 回调方案未使用深度缓冲；
- Web 版依赖 WebGPU，旧浏览器无法运行；保存使用浏览器下载而不是覆盖原文件，且已下载的 `.fem` 不能再次读取，除非重新上传；
- Web 版求解仍在主线程执行，大网格会短暂卡住页面。文件任务已经是异步/可轮询结构，之后可以把求解也移到 worker。

## 测试

```bash
cargo test
cargo clippy --all-targets
cargo check -p fem-web --target wasm32-unknown-unknown
```

测试覆盖：

- `fem-solver`：
  - 全节点给定位移的 patch test（应力与 D·ε 一致）；
  - 刚体平移零应力、约束不足报错；
  - 悬臂梁位移与竖向反力平衡；
  - 双材料 patch test：不同单元使用各自的 D 矩阵；
  - 线性约束的链式叠加、一致/冲突环、prescribed 与 link 混合；
  - 周期边界 patch test（单元应变与宏观应变一致）、均匀 RUC 的 `C_eff = D`、角点 offset 不一致时拒绝；
  - 自带示例 `assets/plate.stl` 的导入与悬臂求解（通过 `fem-import`）；
- `fem-import`：
  - STL 平面导入、顶点焊接（含跨量化单元边界的焊接）；
  - 从内存字节导入 STL（Web 使用的路径）；
  - 非平面 STL 拒绝；
- `fem-types`：
  - 文件往返、文件头校验（magic / 版本 / 截断 / 尾部字节）；
  - `to_bytes` 与 writer API 一致、材料库 / 逐单元分配往返、非法长度和未知 `MaterialId` 的拒绝；
  - 从文件加载网格后求解、边界条件与多点约束往返、非法 3D 分量与越界节点的拒绝。

GUI 测试覆盖相机平移/缩放方向、屏幕空间框选，以及在存在可用适配器时创建 shader 与两条渲染管线并验证渲染方向；没有 GPU 适配器的环境会自动跳过渲染测试。

## 依赖

| crate | 用途 |
| --- | --- |
| `faer` | 稀疏 LU |
| `nalgebra` | 固定尺寸小矩阵 |
| `stl_io` | STL 解析（仅 `fem-import`） |
| `thiserror` | 错误类型 |
| `serde` / `rmp-serde` | 共享类型序列化与 MessagePack 文件格式 |
| `eframe` / `egui` | 窗口与 UI（桌面 wgpu + x11/wayland；Web 仅 wgpu/WebGPU） |
| `rfd` | 系统原生 / 浏览器打开、保存文件对话框 |
| `bytemuck` | 顶点/Uniform 字节转换 |
| `wasm-bindgen` | Web 入口导出（仅 `fem-web`） |
| `wasm-pack` | 构建工具，不是运行时依赖 |

依赖保持在必要范围内，没有引入额外的网格库、颜色库或图片库。
