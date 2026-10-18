//! The lifecycle corpus dripped in its five buckets against the same corpus
//! in one batch, at the detector layer.
//!
//! `scenario_f` in veloqrs measures the same drift through the engine, which
//! adds persistence, an apply and a pool plan to the picture. This runs the
//! fold and the batch directly, so a failure here names the detector and a
//! pass here would have named the engine.

mod shapes;

use std::collections::HashMap;
use tracematch::scenarios::{LifecycleActivity, LifecycleConfig, LifecycleCorpus};
use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy, Tunables,
    detect_sections, detect_sections_dated, detect_sections_incremental_cached_with_policy,
    detect_sections_incremental_dated,
};

/// Ground, not count: the two paths mint their own ids, so a section is the
/// set of activities that traverse it. Equal counts over different ground
/// would pass a count check and still be two libraries.
fn ground(sections: &[FrequentSection]) -> Vec<Vec<String>> {
    let mut all: Vec<Vec<String>> = sections
        .iter()
        .map(|s| {
            let mut ids = s.activity_ids.clone();
            ids.sort();
            ids
        })
        .collect();
    all.sort();
    all
}

fn tracks_of(activities: &[&LifecycleActivity]) -> Vec<(String, Vec<GpsPoint>)> {
    activities
        .iter()
        .map(|a| (a.id.clone(), a.gps_points.clone()))
        .collect()
}

/// Drip `buckets` through the cached fold, one call per bucket, and return the
/// final catalogue.
fn drip(
    buckets: &[Vec<(String, Vec<GpsPoint>)>],
    sports: &HashMap<String, String>,
    cfg: &SectionConfig,
) -> Vec<FrequentSection> {
    let mut cache = SectionEvidenceCache::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut catalogue: Vec<FrequentSection> = Vec::new();

    for bucket in buckets {
        let arriving: Vec<String> = bucket.iter().map(|(id, _)| id.clone()).collect();
        pool.extend(bucket.iter().cloned());
        let new_ids: Vec<&str> = arriving.iter().map(String::as_str).collect();
        catalogue = detect_sections_incremental_cached_with_policy(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &[],
            sports,
            cfg,
            &SectionUpdatePolicy::default(),
        )
        .catalogue;
    }
    catalogue
}

#[test]
#[ignore] // ~2 min in release on the 550-activity corpus. --ignored --release.
fn lifecycle_buckets_converge_to_the_batch() {
    let cfg = SectionConfig::default();
    let corpus = LifecycleCorpus::generate(&LifecycleConfig::default());
    let sports = corpus.sport_map_through_e();

    let buckets = vec![
        tracks_of(&corpus.through_a()),
        tracks_of(&corpus.bucket_b_delta.iter().collect::<Vec<_>>()),
        tracks_of(&[&corpus.bucket_c_single]),
        tracks_of(&corpus.bucket_d_delta.iter().collect::<Vec<_>>()),
        tracks_of(&corpus.bucket_e_delta.iter().collect::<Vec<_>>()),
    ];

    let dripped = drip(&buckets, &sports, &cfg);
    let batch = detect_sections(&corpus.tracks_through_e(), &[], &sports, &cfg);

    for section in &dripped {
        println!(
            "[drip] {} over {} activities, {} points",
            section.id,
            section.activity_ids.len(),
            section.polyline.len()
        );
    }
    for section in &batch {
        println!(
            "[batch] {} over {} activities, {} points",
            section.id,
            section.activity_ids.len(),
            section.polyline.len()
        );
    }

    assert_eq!(
        ground(&dripped),
        ground(&batch),
        "dripped {} sections, batch {}",
        dripped.len(),
        batch.len()
    );
}

/// A library that syncs one ride a day: the last bucket folds one ride per
/// call, in date order, long enough for the memo maps to reach their caps and
/// be swept part way through.
#[test]
#[ignore] // ~5 min in release on the 550-activity corpus. --ignored --release.
fn one_ride_per_fold_converges_to_the_batch() {
    let cfg = SectionConfig::default();
    let policy = SectionUpdatePolicy::default();
    let corpus = LifecycleCorpus::generate(&LifecycleConfig::default());
    let sports = corpus.sport_map_through_e();
    let starts: HashMap<String, i64> = corpus
        .through_e()
        .iter()
        .map(|a| (a.id.clone(), a.start_date_unix))
        .collect();

    let mut pool = tracks_of(&corpus.through_d());
    let ids: Vec<&str> = pool.iter().map(|(id, _)| id.as_str()).collect();
    let mut cache = SectionEvidenceCache::new();
    let mut catalogue = detect_sections_incremental_dated(
        &mut cache,
        &[],
        &pool,
        &ids,
        &[],
        &sports,
        &starts,
        &cfg,
        &policy,
    )
    .catalogue;
    let mut rides: Vec<&LifecycleActivity> = corpus.bucket_e_delta.iter().collect();
    rides.sort_by_key(|a| a.start_date_unix);
    for ride in rides {
        pool.push((ride.id.clone(), ride.gps_points.clone()));
        catalogue = detect_sections_incremental_dated(
            &mut cache,
            &catalogue,
            &pool,
            &[ride.id.as_str()],
            &[],
            &sports,
            &starts,
            &cfg,
            &policy,
        )
        .catalogue;
    }

    let batch = detect_sections_dated(&pool, &[], &sports, &starts, &cfg, &Tunables::DEFAULT);
    assert_eq!(
        ground(&catalogue),
        ground(&batch.sections),
        "dripped {} sections, batch {}",
        catalogue.len(),
        batch.sections.len()
    );
}
