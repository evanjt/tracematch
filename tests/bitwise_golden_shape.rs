//! What the golden comparison does when the golden is not this harness's.
//!
//! Scenario: the golden travels beside the corpus, outside the repository, and
//! the harness that writes it changes. A golden written by a version that
//! emitted one line this one does not is then compared line by line against a
//! shorter list, and every comparison after the missing line is offset.
//!
//! Expected behaviour: that reads as a golden to re-record, never as a bitwise
//! divergence. The two mean opposite things: one is a stale file, the other is
//! detector output that moved and needs a signed-off rebase.
//!
//! `baseline.rs` alone, so these run under a plain `cargo test` with no corpus
//! and no feature. The bitwise harness itself needs one of both.

// Only the golden comparison is exercised here; the rest of the module belongs
// to the corpus harnesses that also include it.
#![allow(dead_code)]

#[path = "bitwise/baseline.rs"]
mod baseline;

use std::path::PathBuf;

use baseline::Band;

const BAND: Band = Band {
    time_factor: 1.35,
    time_floor_ms: 50,
    bytes_factor: 1.20,
    bytes_floor: 16 * 1024 * 1024,
};

/// A golden of its own per test: `check` reads the path and writes it when it
/// is absent, so two tests sharing one would race.
fn golden(name: &str, body: &str) -> PathBuf {
    let path = std::env::temp_dir().join(format!("tracematch-golden-shape-{name}.txt"));
    std::fs::write(&path, body).expect("write golden");
    path
}

fn digests(lines: &[&str]) -> Vec<String> {
    lines.iter().map(|l| l.to_string()).collect()
}

#[test]
fn a_golden_matching_this_harness_passes() {
    let path = golden("match", "A 0000000000000001\nC 0000000000000002\n");
    baseline::check(
        &path,
        &digests(&["A 0000000000000001", "C 0000000000000002"]),
        &[],
        &BAND,
    );
}

/// The shape observed on 2026-09-12: the golden opened with a lift-veto line
/// the running harness does not emit, and the offset was reported as a bitwise
/// divergence with a hash nobody could account for.
#[test]
#[should_panic(expected = "the golden's scenarios are not this harness's")]
fn a_golden_with_a_line_this_harness_does_not_emit_says_so() {
    let path = golden(
        "extra",
        "L 151 30163\nA 0000000000000001\nC 0000000000000002\n",
    );
    baseline::check(
        &path,
        &digests(&["A 0000000000000001", "C 0000000000000002"]),
        &[],
        &BAND,
    );
}

#[test]
#[should_panic(expected = "the golden's scenarios are not this harness's")]
fn a_harness_emitting_a_line_the_golden_lacks_says_so_too() {
    let path = golden("missing", "A 0000000000000001\nC 0000000000000002\n");
    baseline::check(
        &path,
        &digests(&["L 151 30163", "A 0000000000000001", "C 0000000000000002"]),
        &[],
        &BAND,
    );
}

/// The case the tripwire exists for, which must keep its own message: same
/// scenarios, one different digest.
#[test]
#[should_panic(expected = "bitwise divergence from the golden baseline")]
fn labels_that_line_up_with_a_different_digest_is_still_a_divergence() {
    let path = golden("moved", "A 0000000000000001\nC 0000000000000002\n");
    baseline::check(
        &path,
        &digests(&["A 00000000000000ff", "C 0000000000000002"]),
        &[],
        &BAND,
    );
}

/// A comment line carries the record of who moved the golden and why, and a
/// cost line is compared against a band rather than for equality. Neither is a
/// scenario, so neither may enter the label comparison.
#[test]
fn comments_and_cost_lines_are_not_scenarios() {
    let path = golden(
        "comments",
        "# moved 2026-01-01: a reason\nA 0000000000000001\nperf_cold_ms 100\n",
    );
    baseline::check(&path, &digests(&["A 0000000000000001"]), &[], &BAND);
}
