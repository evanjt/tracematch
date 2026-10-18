//! Where a single-ride fold's time goes, phase by phase.
//!
//! A backfill replayed one ride per fold pays the warm-add cost once per
//! ride. These runs fold a base in one call, then the remaining rides one
//! per call in date order, and print each fold's wall time beside a
//! `RIDE` marker on stderr. Under `profile` the phase timers print between
//! the markers, so a script can attribute every phase to its ride.
//!
//! `SINGLE_RIDE_TAIL` sets how many of the last rides fold one by one, and
//! `SINGLE_RIDE_BULK_ONLY` stops after the one-fold comparison. Run with
//! `RAYON_NUM_THREADS=1` for phase times that add up:
//!
//!     cargo run --release --example single_ride_fold_cost \
//!         --features synthetic,profile -- lifecycle 2> lifecycle.prof
//!     cargo run --release --example single_ride_fold_cost \
//!         --features real-corpus,profile -- private 2> private.prof

// Without a corpus feature the harness has nothing to run but its usage line.
#![cfg_attr(
    not(any(feature = "synthetic", feature = "real-corpus")),
    allow(dead_code)
)]

#[cfg(feature = "real-corpus")]
#[path = "../tests/corpus/mod.rs"]
mod corpus;

use std::collections::HashMap;
use std::time::Instant;

use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    detect_sections_incremental_dated,
};

struct Pool {
    tracks: Vec<(String, Vec<GpsPoint>)>,
    sports: HashMap<String, String>,
    starts: HashMap<String, i64>,
    seconds: HashMap<String, Vec<f64>>,
}

impl Pool {
    fn fold(
        &self,
        cache: &mut SectionEvidenceCache,
        existing: &[FrequentSection],
        upto: usize,
        new_ids: &[&str],
        config: &SectionConfig,
    ) -> Vec<FrequentSection> {
        let pool = &self.tracks[..upto];
        let seconds: Vec<&[f64]> = pool
            .iter()
            .map(|(id, _)| self.seconds.get(id).map_or(&[][..], |s| s.as_slice()))
            .collect();
        detect_sections_incremental_dated(
            cache,
            existing,
            pool,
            new_ids,
            &seconds,
            &self.sports,
            &self.starts,
            config,
            &SectionUpdatePolicy::default(),
        )
        .catalogue
    }
}

fn largest_cluster(cache: &SectionEvidenceCache) -> usize {
    cache
        .cluster_footprints(tracematch::sections::POOLED_SPORT)
        .iter()
        .map(|c| c.member_ids.len())
        .max()
        .unwrap_or(0)
}

/// Fold `tracks[..base]` in one call, then the rest one per call, then the
/// rest again as one call over a fresh copy of the base cache.
fn replay(p: &Pool, base: usize, config: &SectionConfig) {
    let head: Vec<&str> = p.tracks[..base].iter().map(|(id, _)| id.as_str()).collect();
    let mut cache = SectionEvidenceCache::new();
    let t = Instant::now();
    eprintln!("BASE start");
    let base_catalogue = p.fold(&mut cache, &[], base, &head, config);
    let base_ms = t.elapsed().as_millis();
    eprintln!("BASE end");
    println!(
        "base {base} rides: {} sections, {base_ms} ms",
        base_catalogue.len()
    );

    let tail: Vec<&str> = p.tracks[base..].iter().map(|(id, _)| id.as_str()).collect();
    let mut bulk = cache.clone();
    eprintln!("BULK start");
    let t = Instant::now();
    let bulk_catalogue = p.fold(&mut bulk, &base_catalogue, p.tracks.len(), &tail, config);
    let bulk_ms = t.elapsed().as_millis();
    eprintln!("BULK end");
    println!(
        "bulk {} rides in one fold: {} sections, {bulk_ms} ms",
        tail.len(),
        bulk_catalogue.len()
    );

    if std::env::var("SINGLE_RIDE_BULK_ONLY").is_ok() {
        return;
    }
    let mut catalogue = base_catalogue;
    let mut total = 0u128;
    for (i, id) in tail.iter().enumerate() {
        let t = Instant::now();
        catalogue = p.fold(&mut cache, &catalogue, base + i + 1, &[id], config);
        let ms = t.elapsed().as_micros();
        total += ms;
        eprintln!(
            "RIDE {i} {ms} {} {}",
            largest_cluster(&cache),
            catalogue.len()
        );
    }
    println!(
        "per ride {} rides: {} sections, {} ms total, {:.0} ms mean",
        tail.len(),
        catalogue.len(),
        total / 1000,
        total as f64 / 1000.0 / tail.len() as f64
    );
}

#[cfg(feature = "synthetic")]
fn lifecycle() {
    use tracematch::scenarios::lifecycle::{LifecycleConfig, LifecycleCorpus};
    let corpus = LifecycleCorpus::generate(&LifecycleConfig::default());
    let tail_len: usize = std::env::var("SINGLE_RIDE_TAIL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(corpus.bucket_e_delta.len());
    let base = corpus.through_d().len() + corpus.bucket_e_delta.len() - tail_len;
    let mut tail: Vec<_> = corpus.bucket_e_delta.iter().collect();
    tail.sort_by(|a, b| {
        a.start_date_unix
            .cmp(&b.start_date_unix)
            .then(a.id.cmp(&b.id))
    });
    let all: Vec<_> = corpus.through_d().into_iter().chain(tail).collect();
    let p = Pool {
        tracks: all
            .iter()
            .map(|a| (a.id.clone(), a.gps_points.clone()))
            .collect(),
        sports: all
            .iter()
            .map(|a| (a.id.clone(), a.sport_type.clone()))
            .collect(),
        starts: all
            .iter()
            .map(|a| (a.id.clone(), a.start_date_unix))
            .collect(),
        seconds: HashMap::new(),
    };
    replay(&p, base, &SectionConfig::default());
}

#[cfg(feature = "real-corpus")]
fn private() {
    let tail_len: usize = std::env::var("SINGLE_RIDE_TAIL")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(100);
    let (mut tracks, starts, seconds) = corpus::load_tracks("fullcorpus", 1000);
    tracks.sort_by(|a, b| {
        let ka = starts.get(&a.0).copied().unwrap_or(i64::MAX);
        let kb = starts.get(&b.0).copied().unwrap_or(i64::MAX);
        ka.cmp(&kb).then(a.0.cmp(&b.0))
    });
    let sports = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "All".to_string()))
        .collect();
    let base = tracks.len() - tail_len;
    let p = Pool {
        tracks,
        sports,
        starts,
        seconds,
    };
    replay(&p, base, &SectionConfig::default());
}

fn main() {
    match std::env::args().nth(1).as_deref() {
        #[cfg(feature = "synthetic")]
        Some("lifecycle") => lifecycle(),
        #[cfg(feature = "real-corpus")]
        Some("private") => private(),
        other => eprintln!(
            "unknown run {other:?}: lifecycle needs --features synthetic, private needs real-corpus"
        ),
    }
}
