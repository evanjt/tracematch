//! The GeoLife corpus as `geolife_bitwise.rs` folds it: the busiest logger,
//! in date order, with the sports, start days and per-point seconds the gate
//! passes. A scratch measurement over the public corpus reads the same
//! activities the gate does, so its sections carry the gate's ids.

// Each test target compiles this module separately and uses a subset of it.
#![allow(dead_code)]

#[path = "../../examples/common/geolife.rs"]
mod geolife;

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use crate::bitwise::Corpus;

pub const ENV: &str = "LAB_GEOLIFE_DIR";
pub const MAX_USERS: usize = 200;
pub const MAX_PER_USER: usize = 120;

pub fn user_of(id: &str) -> &str {
    id.split_once('_').map(|(u, _)| u).unwrap_or(id)
}

pub fn data_dir() -> PathBuf {
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
pub fn day_of(date: &str) -> Option<i64> {
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

/// The busiest user's activities, as the gate folds them.
pub fn load_busiest() -> Corpus {
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
