//! Locates the local GPX corpora.
//!
//! The traces are personal activity history and never enter the repository.
//! They are gitignored, and `scripts/check-no-private-data.sh` refuses to stage
//! them. Set `TRACEMATCH_CORPUS` to keep them outside the working tree
//! entirely, which is the arrangement no gitignore mistake can undo.
//!
//! Targets that need a corpus are gated behind the `real-corpus` feature, so a
//! machine without the data does not build them. Once built, a missing or empty
//! corpus is a hard failure: a test that quietly returns reports the same green
//! as one that ran.

// Each test target compiles this module separately and uses a subset of it.
#![allow(dead_code)]

use std::path::{Path, PathBuf};

pub const ENV: &str = "TRACEMATCH_CORPUS";

/// Directory holding the corpus subdirectories.
pub fn root() -> PathBuf {
    match std::env::var(ENV) {
        Ok(p) if !p.trim().is_empty() => PathBuf::from(p),
        _ => PathBuf::from(env!("CARGO_MANIFEST_DIR")),
    }
}

/// Absolute path to one corpus, verified to exist and hold GPX files.
pub fn dir(name: &str) -> PathBuf {
    let path = root().join(name);

    assert!(
        path.is_dir(),
        "corpus {name:?} not found at {}\n\
         Set {ENV} to the directory holding the corpora, or drop them in the \
         crate root. Building this target without the data is what the \
         `real-corpus` feature exists to prevent.",
        path.display()
    );

    let count = gpx_paths(&path).len();
    assert!(
        count > 0,
        "corpus {name:?} at {} holds no .gpx files. An empty corpus makes every \
         assertion below vacuous.",
        path.display()
    );

    path
}

fn gpx_paths(dir: &Path) -> Vec<PathBuf> {
    let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("read {}: {e}", dir.display()))
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|e| e.eq_ignore_ascii_case("gpx")))
        .collect();
    paths.sort();
    paths
}

/// GPX paths from one corpus in a stable order, capped at `limit`.
///
/// Sorted, so a test that takes the first N takes the same N on every machine.
pub fn gpx_files(name: &str, limit: usize) -> Vec<PathBuf> {
    let dir = dir(name);
    let mut paths = gpx_paths(&dir);
    paths.truncate(limit);

    assert!(
        !paths.is_empty(),
        "corpus {name:?} yielded no files after sorting"
    );
    paths
}

/// Asserts a corpus holds at least `minimum` files before a test leans on it.
pub fn require_at_least(name: &str, minimum: usize) -> Vec<PathBuf> {
    let paths = gpx_files(name, usize::MAX);
    assert!(
        paths.len() >= minimum,
        "corpus {name:?} holds {} GPX files, this test needs {minimum}. A short \
         corpus changes what detection finds, so the thresholds below would be \
         measuring a different dataset.",
        paths.len()
    );
    paths
}
