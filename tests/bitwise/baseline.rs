//! Cost baselines, recorded beside the bitwise digests.
//!
//! A digest says the fold's output did not change. These numbers say it did
//! not get slower or heavier producing it. A time cannot be compared for
//! equality the way a digest can, so each number is bounded above its
//! recorded baseline by a band: a ratio, which absorbs the machine, and an
//! absolute floor, which absorbs noise on a small number. A rise must clear
//! both to count. A faster or lighter run never fails.
//!
//! Recorded in the same golden file as the digests, under a `perf_` prefix,
//! and rebased by the same `TRACEMATCH_BITWISE_REBASE=1`.
//!
//! Times are only meaningful in a release build, so a debug run compares the
//! digests, skips the numbers, and carries the recorded ones forward rather
//! than overwriting them with figures nobody can use.

use std::alloc::{GlobalAlloc, Layout, System};
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

/// Distinguishes a cost line from a digest line in the golden.
pub const PREFIX: &str = "perf_";

// ---------------------------------------------------------------------------
// Peak heap. Declared by each gate binary that includes this module, never by
// the library, so only the gates pay the two atomics per allocation. The
// baseline is recorded through the same allocator, so both sides of a
// comparison carry the same overhead.
// ---------------------------------------------------------------------------

static LIVE: AtomicU64 = AtomicU64::new(0);
static PEAK: AtomicU64 = AtomicU64::new(0);
static BASE: AtomicU64 = AtomicU64::new(0);

pub struct Counting;

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() {
            let live = LIVE.fetch_add(layout.size() as u64, Relaxed) + layout.size() as u64;
            PEAK.fetch_max(live, Relaxed);
        }
        p
    }

    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        LIVE.fetch_sub(layout.size() as u64, Relaxed);
        unsafe { System.dealloc(p, layout) }
    }
}

#[global_allocator]
static ALLOCATOR: Counting = Counting;

/// Highest live heap the process has reached, in bytes.
pub fn peak_bytes() -> u64 {
    PEAK.load(Relaxed)
}

/// Anchor the peak at what is live now, so what follows is measured against
/// the corpus already in memory rather than including it.
pub fn anchor_peak() {
    let live = LIVE.load(Relaxed);
    BASE.store(live, Relaxed);
    PEAK.store(live, Relaxed);
}

/// How far the peak rose above the last anchor, in bytes.
pub fn peak_rise_bytes() -> u64 {
    PEAK.load(Relaxed).saturating_sub(BASE.load(Relaxed))
}

// ---------------------------------------------------------------------------
// Bands and comparison
// ---------------------------------------------------------------------------

/// How far above its baseline a number may drift before it is a regression.
///
/// Time and memory get their own band because they drift for different
/// reasons. A wall clock varies with the machine, so its band has to hold a
/// slow runner. An allocation count is the code's, not the machine's, so its
/// band can be tight. A metric named `*_bytes` takes the byte band, every
/// other takes the time one.
#[derive(Clone, Copy, Debug)]
pub struct Band {
    pub time_factor: f64,
    pub time_floor_ms: u64,
    pub bytes_factor: f64,
    pub bytes_floor: u64,
}

impl Band {
    fn of(&self, name: &str) -> (f64, u64) {
        if name.ends_with("_bytes") {
            (self.bytes_factor, self.bytes_floor)
        } else {
            (self.time_factor, self.time_floor_ms)
        }
    }
}

/// The cost lines a golden holds, in recorded order.
pub fn recorded(golden: &str) -> Vec<(String, u64)> {
    golden
        .lines()
        .filter_map(|line| line.strip_prefix(PREFIX))
        .filter_map(|rest| {
            let (name, value) = rest.split_once(' ')?;
            Some((format!("{PREFIX}{name}"), value.trim().parse().ok()?))
        })
        .collect()
}

/// The golden's lines that are not costs: the digests, untouched.
pub fn digest_lines(golden: &str) -> Vec<&str> {
    golden.lines().filter(|l| !l.starts_with(PREFIX)).collect()
}

/// Whether the golden's digest lines are not `digests`, so what it recorded
/// was measured on other input. A cost compared across that is a corpus
/// change read as a regression, and the caller re-derives instead.
pub fn digests_differ(golden: &str, digests: &[String]) -> bool {
    let recorded = digest_lines(golden);
    recorded.len() != digests.len() || recorded.iter().zip(digests).any(|(r, d)| r != d)
}

/// `measured` as golden lines.
pub fn lines(measured: &[(&str, u64)]) -> Vec<String> {
    measured
        .iter()
        .map(|(name, value)| format!("{name} {value}"))
        .collect()
}

/// Every measured number that rose past the band, plus every metric the two
/// sides disagree on existing at all. Empty means the run is within budget.
pub fn regressions(golden: &str, measured: &[(&str, u64)], band: &Band) -> Vec<String> {
    let baseline = recorded(golden);
    let mut out = Vec::new();

    for (name, value) in measured {
        let Some((_, was)) = baseline.iter().find(|(n, _)| n == name) else {
            out.push(format!(
                "{name} is not in the baseline. Record it with \
                 TRACEMATCH_BITWISE_REBASE=1"
            ));
            continue;
        };
        let (factor, floor_by) = band.of(name);
        let ceiling = (*was as f64 * factor) as u64;
        let floor = was.saturating_add(floor_by);
        if *value > ceiling && *value > floor {
            out.push(format!(
                "{name} {value} is over the {was} baseline, past both the \
                 {factor:.2}x band ({ceiling}) and the +{floor_by} floor \
                 ({floor})"
            ));
        }
    }

    for (name, _) in &baseline {
        if !measured.iter().any(|(n, _)| n == name) {
            out.push(format!(
                "{name} is in the baseline but nothing measures it any more. \
                 Clear it with TRACEMATCH_BITWISE_REBASE=1"
            ));
        }
    }

    out
}

// ---------------------------------------------------------------------------
// The golden file
// ---------------------------------------------------------------------------

/// Compare `digests` and `measured` with the golden at `path`, or record them
/// there when it is absent or a rebase is asked for.
///
/// Digests are compared for equality, costs against `band`. A debug build
/// cannot say anything useful about a time, so it compares the digests only
/// and preserves whatever costs the golden already holds.
pub fn check(path: &Path, digests: &[String], measured: &[(&str, u64)], band: &Band) {
    let timed = !cfg!(debug_assertions);
    let existing = std::fs::read_to_string(path).ok();
    let rebase = std::env::var("TRACEMATCH_BITWISE_REBASE").is_ok_and(|v| v == "1");

    if let Some(golden) = existing.as_deref()
        && !rebase
    {
        let want = digest_lines(golden);
        let got: Vec<&str> = digests.iter().map(|s| s.as_str()).collect();
        for (w, g) in want.iter().zip(got.iter()) {
            assert_eq!(
                w, g,
                "bitwise divergence from the golden baseline. If this change \
                 is INTENTIONAL, rerun with TRACEMATCH_BITWISE_REBASE=1"
            );
        }
        assert_eq!(want.len(), got.len(), "scenario count changed");
        if !want.is_empty() {
            println!("golden baseline matched: {} digests", got.len());
        }

        if !timed {
            println!("debug build: cost baselines not compared, run --release");
            return;
        }
        let over = regressions(golden, measured, band);
        assert!(
            over.is_empty(),
            "cost regression against the recorded baseline:\n  {}\n\
             Bands are deliberately loose. If this cost is INTENTIONAL, \
             rebase with TRACEMATCH_BITWISE_REBASE=1",
            over.join("\n  ")
        );
        println!("cost baselines within band: {}", describe(measured));
        return;
    }

    let costs = if timed {
        lines(measured)
    } else {
        existing
            .as_deref()
            .map(|g| lines(&borrowed(&recorded(g))))
            .unwrap_or_default()
    };
    let body = digests
        .iter()
        .cloned()
        .chain(costs)
        .collect::<Vec<_>>()
        .join("\n")
        + "\n";
    std::fs::write(path, &body).expect("write golden baseline");
    println!(
        "golden baseline {} at {}",
        if rebase { "rewritten" } else { "recorded" },
        path.display()
    );
}

/// Recorded pairs as the borrowed shape `lines` takes.
fn borrowed(pairs: &[(String, u64)]) -> Vec<(&str, u64)> {
    pairs.iter().map(|(n, v)| (n.as_str(), *v)).collect()
}

fn describe(measured: &[(&str, u64)]) -> String {
    measured
        .iter()
        .map(|(n, v)| format!("{n}={v}"))
        .collect::<Vec<_>>()
        .join(" ")
}
