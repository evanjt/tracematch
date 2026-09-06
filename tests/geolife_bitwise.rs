//! The `bitwise` harness over GeoLife, the public corpus any machine can
//! fetch. The golden baseline lives in the repository, so a change to the
//! fold's output fails CI on every push without touching anyone's private
//! data.
//!
//! One person, the busiest logger, in date order: the shape the app detects
//! in. A change that moves the output attaches its before-and-after report to
//! the item and stops there; the rebase is Evan's call, carries its reason and
//! is committed with the change that moved it. See
//! `bitwise::baseline::REBASE_ENV`.
//!
//! Run: `scripts/fetch_geolife.sh && cargo test --release --features public-corpus \
//!       --test geolife_bitwise -- --nocapture`

mod bitwise;
#[path = "../examples/common/geolife.rs"]
mod geolife;

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use bitwise::{Corpus, Shape, baseline::Band};

const ENV: &str = "LAB_GEOLIFE_DIR";

/// This gate runs on shared CI runners with a quarter of the cores that
/// recorded the golden, so the clock band is wide enough to catch only an
/// order-of-magnitude regression. Allocation does not vary with the runner,
/// so the heap is bounded properly here. The private corpus carries the tight
/// clock band.
const BAND: Band = Band {
    time_factor: 10.0,
    time_floor_ms: 250,
    bytes_factor: 1.25,
    bytes_floor: 8 * 1024 * 1024,
};
const MAX_USERS: usize = 200;
const MAX_PER_USER: usize = 120;

fn user_of(id: &str) -> &str {
    id.split_once('_').map(|(u, _)| u).unwrap_or(id)
}

fn data_dir() -> PathBuf {
    if let Ok(p) = std::env::var(ENV)
        && !p.trim().is_empty()
    {
        let path = PathBuf::from(p);
        assert!(
            path.is_dir(),
            "{ENV} points at {}, which is not a directory",
            path.display()
        );
        return path;
    }
    let default =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("geolife/Geolife Trajectories 1.3/Data");
    assert!(
        default.is_dir(),
        "GeoLife not found at {}. Fetch it with scripts/fetch_geolife.sh, or set {ENV}.",
        default.display()
    );
    default
}

/// Days since 1970-01-01 for `YYYY-MM-DD`.
fn day_of(date: &str) -> Option<i64> {
    let mut it = date.split('-');
    let y: i64 = it.next()?.parse().ok()?;
    let m: i64 = it.next()?.parse().ok()?;
    let d: i64 = it.next()?.get(..2)?.parse().ok()?;
    let yy = if m <= 2 { y - 1 } else { y };
    let era = if yy >= 0 { yy } else { yy - 399 } / 400;
    let yoe = yy - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    Some(era * 146_097 + doe - 719_468)
}

fn load_corpus() -> Corpus {
    let (trajectories, stats) = geolife::load_geolife(&data_dir(), MAX_USERS, MAX_PER_USER);
    assert!(stats.users_loaded > 0, "GeoLife yielded no users");

    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    for t in &trajectories {
        *counts.entry(user_of(&t.id)).or_default() += 1;
    }
    let busiest = counts
        .iter()
        .max_by_key(|(user, n)| (**n, std::cmp::Reverse(*user)))
        .map(|(user, _)| (*user).to_string())
        .expect("at least one user loaded");

    let mut mine: Vec<_> = trajectories
        .into_iter()
        .filter(|t| user_of(&t.id) == busiest)
        .collect();
    mine.sort_by(|a, b| a.date.cmp(&b.date).then(a.id.cmp(&b.id)));

    let sports: HashMap<String, String> = mine
        .iter()
        .map(|t| (t.id.clone(), t.sport.clone()))
        .collect();
    let starts: HashMap<String, i64> = mine
        .iter()
        .filter_map(|t| day_of(&t.date).map(|d| (t.id.clone(), d * 86_400)))
        .collect();
    // GeoLife times every fix and carries no elevation it can trust
    // (`examples/common/geolife.rs`), so these seconds reach the velocity
    // veto but no candidate ever reaches them: `lift_spans_tuned` returns
    // empty below two elevated points. Only the private corpus can gate the
    // veto. They are passed anyway so this gate folds the same inputs the
    // engine does.
    let seconds = mine
        .iter()
        .filter(|t| t.seconds.len() == t.points.len())
        .map(|t| (t.id.clone(), t.seconds.clone()))
        .collect();
    let tracks = mine.into_iter().map(|t| (t.id, t.points)).collect();
    Corpus {
        tracks,
        sports,
        starts,
        seconds,
    }
}

#[test]
fn geolife_output_is_bitwise_stable() {
    let c = load_corpus();
    bitwise::run(
        &c,
        Shape {
            warm_adds: 20,
            bulk_base: 40,
        },
        BAND,
        &PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/geolife_bitwise_golden.txt"),
    );
}
