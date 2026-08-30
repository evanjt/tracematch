//! What the cost bands accept and what they reject.
//!
//! The bands guard the corpus gates, which need a corpus and a release build
//! to run at all. These cases pin the comparison itself, so the rule a gate
//! applies is checked on every push rather than only where the data lives.

mod bitwise;

use std::sync::Mutex;

use bitwise::baseline::{Band, anchor_peak, peak_bytes, peak_rise_bytes, recorded, regressions};

/// The peak is process-wide, so the two cases that read it take turns. The
/// rest of this file allocates nothing that would move it.
static HEAP: Mutex<()> = Mutex::new(());

const GOLDEN: &str = "\
A 0123456789abcdef
perf_cold_ms 1000
perf_add_p95_ms 8
perf_peak_bytes 100000000
";

fn band() -> Band {
    Band {
        time_factor: 1.5,
        time_floor_ms: 20,
        bytes_factor: 1.5,
        bytes_floor: 8 * 1024 * 1024,
    }
}

#[test]
fn costs_inside_the_band_pass() {
    let measured = [
        ("perf_cold_ms", 1400u64),
        ("perf_add_p95_ms", 11),
        ("perf_peak_bytes", 140_000_000),
    ];
    assert!(regressions(GOLDEN, &measured, &band()).is_empty());
}

#[test]
fn a_faster_run_is_never_a_regression() {
    let measured = [
        ("perf_cold_ms", 1u64),
        ("perf_add_p95_ms", 0),
        ("perf_peak_bytes", 0),
    ];
    assert!(regressions(GOLDEN, &measured, &band()).is_empty());
}

#[test]
fn a_rise_past_both_the_ratio_and_the_floor_fails() {
    let measured = [
        ("perf_cold_ms", 4000u64),
        ("perf_add_p95_ms", 8),
        ("perf_peak_bytes", 100_000_000),
    ];
    let over = regressions(GOLDEN, &measured, &band());
    assert_eq!(over.len(), 1, "{over:?}");
    assert!(over[0].starts_with("perf_cold_ms 4000"), "{}", over[0]);
}

#[test]
fn a_rise_past_the_ratio_but_under_the_floor_is_noise() {
    // 8 ms to 24 ms is 3x, which no machine-speed band can absorb, but 16 ms
    // of absolute drift on a number this small says nothing about the code.
    let measured = [
        ("perf_cold_ms", 1000u64),
        ("perf_add_p95_ms", 24),
        ("perf_peak_bytes", 100_000_000),
    ];
    assert!(regressions(GOLDEN, &measured, &band()).is_empty());
}

#[test]
fn a_rise_past_the_floor_but_under_the_ratio_is_the_machine() {
    let measured = [
        ("perf_cold_ms", 1400u64),
        ("perf_add_p95_ms", 8),
        ("perf_peak_bytes", 100_000_000),
    ];
    assert!(regressions(GOLDEN, &measured, &band()).is_empty());
}

#[test]
fn a_tight_byte_band_rejects_what_a_loose_time_band_would_accept() {
    // The band a CI gate wants: an order of magnitude on the clock, because
    // the runner is not the machine that recorded it, and a fifth on the
    // heap, because the allocation count is the code's alone.
    let ci = Band {
        time_factor: 10.0,
        time_floor_ms: 250,
        bytes_factor: 1.2,
        bytes_floor: 8 * 1024 * 1024,
    };
    let measured = [
        ("perf_cold_ms", 6000u64),
        ("perf_add_p95_ms", 60),
        ("perf_peak_bytes", 130_000_000),
    ];
    let over = regressions(GOLDEN, &measured, &ci);
    assert_eq!(over.len(), 1, "{over:?}");
    assert!(over[0].starts_with("perf_peak_bytes"), "{}", over[0]);
}

#[test]
fn the_byte_floor_applies_to_bytes_not_the_millisecond_floor() {
    // 4 MB over a 100 MB baseline clears the ms floor of 20 many times over,
    // and would fail if the units were confused.
    let measured = [
        ("perf_cold_ms", 1000u64),
        ("perf_add_p95_ms", 8),
        ("perf_peak_bytes", 104_000_000),
    ];
    assert!(regressions(GOLDEN, &measured, &band()).is_empty());

    let heavy = [
        ("perf_cold_ms", 1000u64),
        ("perf_add_p95_ms", 8),
        ("perf_peak_bytes", 400_000_000),
    ];
    let over = regressions(GOLDEN, &heavy, &band());
    assert_eq!(over.len(), 1, "{over:?}");
    assert!(
        over[0].starts_with("perf_peak_bytes 400000000"),
        "{}",
        over[0]
    );
}

#[test]
fn a_metric_the_baseline_never_recorded_fails_rather_than_passing_silently() {
    let measured = [
        ("perf_cold_ms", 1000u64),
        ("perf_add_p95_ms", 8),
        ("perf_peak_bytes", 100_000_000),
        ("perf_bulk_ms", 500),
    ];
    let over = regressions(GOLDEN, &measured, &band());
    assert_eq!(over.len(), 1, "{over:?}");
    assert!(
        over[0].contains("perf_bulk_ms is not in the baseline"),
        "{}",
        over[0]
    );
}

#[test]
fn a_baseline_metric_nothing_measures_any_more_is_reported() {
    let measured = [("perf_cold_ms", 1000u64), ("perf_add_p95_ms", 8)];
    let over = regressions(GOLDEN, &measured, &band());
    assert_eq!(over.len(), 1, "{over:?}");
    assert!(
        over[0].contains("perf_peak_bytes is in the baseline"),
        "{}",
        over[0]
    );
}

#[test]
fn an_empty_baseline_reports_every_metric_rather_than_nothing() {
    let measured = [("perf_cold_ms", 1000u64), ("perf_peak_bytes", 1)];
    assert_eq!(
        regressions("A 0123456789abcdef\n", &measured, &band()).len(),
        2
    );
}

#[test]
fn only_cost_lines_are_read_back_out_of_a_golden() {
    let pairs = recorded(GOLDEN);
    assert_eq!(pairs.len(), 3);
    assert_eq!(pairs[0], ("perf_cold_ms".to_string(), 1000));
}

#[test]
fn the_counting_allocator_sees_the_heap() {
    let _turn = HEAP.lock().unwrap_or_else(|e| e.into_inner());
    let big = vec![0u8; 64 * 1024 * 1024];
    assert!(
        peak_bytes() >= big.len() as u64,
        "peak {} did not cover a {} byte allocation",
        peak_bytes(),
        big.len()
    );
    drop(big);
}

#[test]
fn an_anchored_peak_measures_the_rise_not_the_heap_already_held() {
    let _turn = HEAP.lock().unwrap_or_else(|e| e.into_inner());
    let held = vec![0u8; 32 * 1024 * 1024];
    anchor_peak();
    assert!(
        peak_rise_bytes() < held.len() as u64,
        "the anchor carried the {} bytes already held into the rise",
        held.len()
    );
    let added = vec![0u8; 48 * 1024 * 1024];
    assert!(
        peak_rise_bytes() >= added.len() as u64,
        "rise {} did not cover the {} bytes added after the anchor",
        peak_rise_bytes(),
        added.len()
    );
    drop(held);
    drop(added);
}
