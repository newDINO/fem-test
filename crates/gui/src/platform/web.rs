use std::future::Future;
use std::pin::Pin;
use std::task::{Context, Poll, Waker};

use eframe::egui;
use rfd::AsyncFileDialog;

use super::{FileRef, TaskOutcome};

/// A web file task that owns an async `rfd` future.
///
/// The future is polled immediately on creation and then once per frame. The
/// first poll must happen synchronously from the click handler: browsers only
/// allow opening a file picker during transient user activation.
pub struct FileTask {
    state: TaskState,
}

enum TaskState {
    Pending(Pin<Box<dyn Future<Output = TaskOutcome>>>),
    Ready(TaskOutcome),
    Done,
}

impl FileTask {
    fn new(ctx: &egui::Context, future: impl Future<Output = TaskOutcome> + 'static) -> Self {
        let ctx = ctx.clone();
        let future = async move {
            let outcome = future.await;
            // egui is demand-driven; wake it so the completed task is applied
            // even if the user does not touch the page again.
            ctx.request_repaint();
            outcome
        };
        let mut task = Self {
            state: TaskState::Pending(Box::pin(future)),
        };
        task.poll();
        task
    }

    pub fn poll(&mut self) -> Option<TaskOutcome> {
        if let TaskState::Pending(future) = &mut self.state {
            let mut context = Context::from_waker(Waker::noop());
            if let Poll::Ready(outcome) = future.as_mut().poll(&mut context) {
                self.state = TaskState::Ready(outcome);
            }
        }

        match std::mem::replace(&mut self.state, TaskState::Done) {
            TaskState::Ready(outcome) => Some(outcome),
            state => {
                self.state = state;
                None
            }
        }
    }
}

fn file_name(handle: &rfd::FileHandle) -> String {
    let name = handle.file_name();
    if name.is_empty() {
        "file".to_owned()
    } else {
        name
    }
}

pub fn open(ctx: &egui::Context) -> FileTask {
    FileTask::new(ctx, async {
        let handle = AsyncFileDialog::new()
            .add_filter("FEM 项目 / STL 网格", &["fem", "stl"])
            .add_filter("FEM 项目", &["fem"])
            .add_filter("STL 网格", &["stl"])
            .pick_file()
            .await;

        match handle {
            Some(handle) => {
                let name = file_name(&handle);
                let bytes = handle.read().await;
                TaskOutcome::Opened {
                    file: FileRef::from_bytes(name, &bytes),
                    bytes,
                }
            }
            None => TaskOutcome::Cancelled,
        }
    })
}

pub fn open_dropped(ctx: &egui::Context, file: egui::DroppedFileHandle) -> FileTask {
    let name = file
        .path()
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| "file".to_owned());

    FileTask::new(ctx, async move {
        match file.bytes_async().await {
            Ok(bytes) => TaskOutcome::Opened {
                file: FileRef::from_bytes(name, &bytes),
                bytes,
            },
            Err(error) => TaskOutcome::Failed(format!("无法读取 {name}：{error}")),
        }
    })
}

pub fn save(
    ctx: &egui::Context,
    _current: Option<&FileRef>,
    _save_as: bool,
    suggested_name: String,
    bytes: Vec<u8>,
) -> FileTask {
    FileTask::new(ctx, async move {
        // Browsers cannot silently overwrite a project path. `rfd` turns the
        // save dialog into a download, so both "保存" and "另存为…" prompt the
        // user (or reuse the browser's download settings).
        let handle = AsyncFileDialog::new()
            .add_filter("FEM 项目", &["fem"])
            .set_file_name(&suggested_name)
            .save_file()
            .await;

        match handle {
            Some(handle) => {
                let name = file_name(&handle);
                match handle.write(&bytes).await {
                    Ok(()) => TaskOutcome::Saved {
                        file: FileRef::unreadable(name),
                    },
                    Err(error) => TaskOutcome::Failed(format!("保存 {name} 失败：{error}")),
                }
            }
            None => TaskOutcome::Cancelled,
        }
    })
}
