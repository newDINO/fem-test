use std::cell::RefCell;
use std::path::{Path, PathBuf};

use eframe::egui;
use rfd::FileDialog;

use super::{FileRef, TaskOutcome, ensure_fem_extension};

thread_local! {
    static LAST_DIRECTORY: RefCell<Option<PathBuf>> = const { RefCell::new(None) };
}

fn file_dialog() -> FileDialog {
    let dialog = FileDialog::new()
        .add_filter("FEM 项目 / STL 网格", &["fem", "stl"])
        .add_filter("FEM 项目", &["fem"])
        .add_filter("STL 网格", &["stl"]);
    LAST_DIRECTORY.with(|directory| match &*directory.borrow() {
        Some(directory) => dialog.set_directory(directory),
        None => dialog,
    })
}

fn remember_directory(path: Option<&Path>) {
    if let Some(path) = path.and_then(Path::parent) {
        LAST_DIRECTORY.with(|directory| {
            *directory.borrow_mut() = Some(path.to_path_buf());
        });
    }
}

/// Native file tasks are completed synchronously inside the dialog call, just
/// like the original desktop implementation. The task wrapper keeps the app
/// code identical to the web build.
pub struct FileTask {
    outcome: Option<TaskOutcome>,
}

impl FileTask {
    fn ready(outcome: TaskOutcome) -> Self {
        Self {
            outcome: Some(outcome),
        }
    }

    pub fn poll(&mut self) -> Option<TaskOutcome> {
        self.outcome.take()
    }
}

pub fn open(_ctx: &egui::Context) -> FileTask {
    let Some(path) = file_dialog().pick_file() else {
        return FileTask::ready(TaskOutcome::Cancelled);
    };
    FileTask::ready(read_path(path))
}

pub fn open_dropped(_ctx: &egui::Context, file: egui::DroppedFileHandle) -> FileTask {
    let path = file.path().to_path_buf();
    match file.bytes() {
        Ok(bytes) => {
            remember_directory(Some(&path));
            FileTask::ready(TaskOutcome::Opened {
                file: FileRef::from_path(path),
                bytes,
            })
        }
        Err(error) => FileTask::ready(TaskOutcome::Failed(format!(
            "无法读取 {}：{error}",
            path.display()
        ))),
    }
}

pub fn save(
    _ctx: &egui::Context,
    current: Option<&FileRef>,
    save_as: bool,
    suggested_name: String,
    bytes: Vec<u8>,
) -> FileTask {
    let target = if save_as {
        None
    } else {
        current.and_then(FileRef::path).map(Path::to_path_buf)
    };

    let path = match target {
        Some(path) => path,
        None => {
            let dialog = FileDialog::new()
                .add_filter("FEM 项目", &["fem"])
                .set_file_name(&suggested_name);
            let dialog = LAST_DIRECTORY.with(|directory| match &*directory.borrow() {
                Some(directory) => dialog.set_directory(directory),
                None => dialog,
            });
            match dialog.save_file() {
                Some(path) => path,
                None => return FileTask::ready(TaskOutcome::Cancelled),
            }
        }
    };

    let path = ensure_fem_extension(&path);
    match fem_types::format::write_bytes_to_path(&path, &bytes) {
        Ok(()) => {
            remember_directory(Some(&path));
            FileTask::ready(TaskOutcome::Saved {
                file: FileRef::from_path(path),
            })
        }
        Err(error) => FileTask::ready(TaskOutcome::Failed(format!(
            "保存 {} 失败：{error}",
            path.display()
        ))),
    }
}

fn read_path(path: PathBuf) -> TaskOutcome {
    match std::fs::read(&path) {
        Ok(bytes) => {
            remember_directory(Some(&path));
            TaskOutcome::Opened {
                file: FileRef::from_path(path),
                bytes,
            }
        }
        Err(error) => TaskOutcome::Failed(format!("无法读取 {}：{error}", path.display())),
    }
}
