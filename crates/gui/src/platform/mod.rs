//! Platform-specific file dialogs and file I/O.
//!
//! The application only ever deals with file *contents*; a [`FileRef`] is an
//! opaque handle that knows how to display itself and, when possible, re-read
//! its contents. Native builds keep a filesystem path, while the web build
//! keeps the bytes that were uploaded (or saved) in memory.
//!
//! File operations are modelled as a single [`FileTask`]. The app owns at most
//! one task at a time and polls it once per frame. On native platforms the
//! dialogs are synchronous and the task completes immediately; on the web the
//! task wraps an async `rfd` future and is polled until it finishes.

use std::path::{Path, PathBuf};
use std::sync::Arc;

#[cfg(not(target_family = "wasm"))]
mod native;
#[cfg(not(target_family = "wasm"))]
pub use native::{FileTask, open, open_dropped, save};

#[cfg(target_family = "wasm")]
mod web;
#[cfg(target_family = "wasm")]
pub use web::{FileTask, open, open_dropped, save};

/// The result of one open/save task.
pub enum TaskOutcome {
    Opened { file: FileRef, bytes: Vec<u8> },
    Saved { file: FileRef },
    Cancelled,
    Failed(String),
}

/// A file the app can refer to after the dialog/task has finished.
#[derive(Clone)]
pub struct FileRef {
    name: String,
    source: Source,
}

#[derive(Clone)]
enum Source {
    /// Contents are still available (web uploads, embedded assets).
    Memory(Arc<[u8]>),
    /// A native filesystem path.
    #[cfg(not(target_family = "wasm"))]
    Path(PathBuf),
    /// A remote/browser file that cannot be read again, such as a project that
    /// was just downloaded.
    #[cfg(target_family = "wasm")]
    Unreadable,
}

impl std::fmt::Debug for FileRef {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FileRef")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

impl FileRef {
    pub fn name(&self) -> &str {
        &self.name
    }

    /// A human-readable identity for status messages.
    pub fn display(&self) -> String {
        #[cfg(not(target_family = "wasm"))]
        if let Source::Path(path) = &self.source {
            return path.display().to_string();
        }
        self.name.clone()
    }

    /// Re-read the file, used by "重新导入 STL". Fails for files that only
    /// exist on the browser side.
    pub fn read(&self) -> Result<Vec<u8>, String> {
        match &self.source {
            Source::Memory(bytes) => Ok(bytes.to_vec()),
            #[cfg(not(target_family = "wasm"))]
            Source::Path(path) => {
                std::fs::read(path).map_err(|error| format!("无法读取 {}：{error}", path.display()))
            }
            #[cfg(target_family = "wasm")]
            Source::Unreadable => Err(format!(
                "{} 的内容已不在内存中，请重新打开文件",
                self.display()
            )),
        }
    }

    #[cfg(not(target_family = "wasm"))]
    pub(crate) fn from_path(path: PathBuf) -> Self {
        let name = path
            .file_name()
            .map(|name| name.to_string_lossy().into_owned())
            .unwrap_or_else(|| path.display().to_string());
        Self {
            name,
            source: Source::Path(path),
        }
    }

    pub(crate) fn from_bytes(name: impl Into<String>, bytes: &[u8]) -> Self {
        Self {
            name: name.into(),
            source: Source::Memory(Arc::from(bytes)),
        }
    }

    /// A browser download that has no readable handle afterwards.
    #[cfg(target_family = "wasm")]
    pub(crate) fn unreadable(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            source: Source::Unreadable,
        }
    }

    /// A file compiled into the application, such as the sample plate.
    pub fn embedded(name: impl Into<String>, bytes: &'static [u8]) -> Self {
        Self::from_bytes(name, bytes)
    }

    #[cfg(not(target_family = "wasm"))]
    pub(crate) fn path(&self) -> Option<&Path> {
        match &self.source {
            Source::Path(path) => Some(path),
            _ => None,
        }
    }
}

/// Force a `.fem` extension without clobbering a name that already has one.
pub fn ensure_fem_extension(path: &Path) -> PathBuf {
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
