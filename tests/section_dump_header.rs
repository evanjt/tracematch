//! The section dump's first line names the fold it was written from, and
//! `section_compare.py` will not read two dumps of one fold as a comparison.
//!
//! Scenario: a dump of an empty fold is written, and two dumps carrying one
//! digest are compared.
//! Expected behaviour: the dump opens with a header whose digest is the
//! catalogue digest, and the comparison exits non-zero and says the two are
//! one fold unless `--same-fold` is passed.

mod bitwise;
mod section_rows;

use std::path::{Path, PathBuf};
use std::process::Command;

fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("tracematch-dump-header-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn first_line(path: &Path) -> Option<String> {
    std::fs::read_to_string(path)
        .unwrap()
        .lines()
        .next()
        .map(str::to_owned)
}

#[test]
fn dump_opens_with_a_header_naming_the_fold() {
    let out = scratch("empty.jsonl");
    section_rows::write_dump(
        &[],
        &[],
        out.to_str().unwrap(),
        "synthetic",
        bitwise::catalogue_digest(&[]),
    );
    let header: serde_json::Value =
        serde_json::from_str(&first_line(&out).expect("the dump has no header line")).unwrap();
    assert_eq!(
        header["digest"].as_str(),
        Some(format!("{:016x}", bitwise::catalogue_digest(&[])).as_str()),
    );
    assert_eq!(header["corpus"].as_str(), Some("synthetic"));
    let tree = header["tree"].as_str().expect("the header names a tree");
    assert!(
        tree.len() >= 40 && tree[..40].bytes().all(|b| b.is_ascii_hexdigit()),
        "tree is not a commit: {tree}"
    );
}

#[test]
fn dirty_tree_carries_a_plus() {
    assert_eq!(section_rows::tree_label("abc", true), "abc+");
    assert_eq!(section_rows::tree_label("abc", false), "abc");
}

fn row(id: &str) -> String {
    format!(
        r#"{{"id":"{id}","sport":"All","line_hash":"0000000000000001","line_points":2,"first":[0.0,0.0],"last":[0.0,0.001],"distance_m":111.0,"rep":"a1","rep_range":null,"rep_reversed":false,"members":["a1","a2"],"portions":[["a1",0,1,"Same"],["a2",0,1,"Same"]],"visit_count":2,"trace_ids":["a1","a2"],"confidence":1.0,"observation_count":2,"polyline":[[0.0,0.0],[0.0,0.001]],"portion_m":[111.0,111.0]}}"#
    )
}

fn dump(name: &str, digest: &str, tree: &str) -> PathBuf {
    let path = scratch(name);
    let body = format!(
        "{{\"digest\":\"{digest}\",\"tree\":\"{tree}\",\"corpus\":\"synthetic\"}}\n{}\n",
        row("sec_All_0")
    );
    std::fs::write(&path, body).unwrap();
    path
}

fn compare(before: &Path, after: &Path, extra: &[&str]) -> (bool, String) {
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/section_compare.py");
    let out = Command::new("python3")
        .arg("-I")
        .arg(&script)
        .arg(before)
        .arg(after)
        .args(extra)
        .output()
        .expect("python3 runs section_compare.py");
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    if !out.status.success() {
        eprintln!("{}", String::from_utf8_lossy(&out.stderr));
    }
    (out.status.success(), stdout)
}

#[test]
fn compare_refuses_two_dumps_of_one_fold() {
    let a = dump("one-a.jsonl", "00000000000000aa", "t1");
    let b = dump("one-b.jsonl", "00000000000000aa", "t2");
    let (ok, stdout) = compare(&a, &b, &[]);
    let first = stdout.lines().next().unwrap_or_default();
    assert!(!ok, "one fold compared with itself exited zero");
    assert!(first.contains("one fold"), "first line was {first:?}");
}

#[test]
fn compare_runs_with_same_fold_flag() {
    let a = dump("flag-a.jsonl", "00000000000000aa", "t1");
    let b = dump("flag-b.jsonl", "00000000000000aa", "t1");
    let (ok, stdout) = compare(&a, &b, &["--same-fold"]);
    assert!(ok, "--same-fold did not compare: {stdout}");
    assert!(
        stdout.contains("sections"),
        "no comparison printed: {stdout}"
    );
}

#[test]
fn compare_distinct_folds_prints_both_and_compares() {
    let a = dump("two-a.jsonl", "00000000000000aa", "t1");
    let b = dump("two-b.jsonl", "00000000000000bb", "t2");
    let (ok, stdout) = compare(&a, &b, &[]);
    assert!(ok, "distinct folds did not compare: {stdout}");
    assert!(
        stdout.contains("00000000000000aa"),
        "before digest missing: {stdout}"
    );
    assert!(
        stdout.contains("00000000000000bb"),
        "after digest missing: {stdout}"
    );
    let first = stdout.lines().next().unwrap_or_default();
    assert!(!first.contains("one fold"), "first line was {first:?}");
}
