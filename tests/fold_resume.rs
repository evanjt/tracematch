//! A fold killed after some clusters resumes from its last checkpoint: the
//! clusters already cut stay cut, only the rest are recomputed, and the
//! catalogue is the one an uninterrupted fold emits.

mod bitwise;

use std::collections::HashMap;

use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::{
    GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    detect_sections_unified_incremental_observed,
};

type Tracks = Vec<(String, Vec<GpsPoint>)>;

/// Three far-apart clusters, each its own corridor library.
fn library(n_clusters: usize, bucket_a: usize) -> (Tracks, HashMap<String, String>) {
    let mut tracks: Tracks = Vec::new();
    let mut sports: HashMap<String, String> = HashMap::new();
    for c in 0..n_clusters {
        let origin = GpsPoint::with_elevation(44.0 + c as f64 * 3.0, 8.0, 400.0);
        let corpus = LifecycleCorpus::generate(&LifecycleConfig {
            origin,
            seed: 0x51D + c as u64,
            bucket_a_count: bucket_a,
            bucket_b_delta_count: 0,
            bucket_d_delta_count: 0,
            bucket_e_delta_count: 0,
            one_off_fraction: 0.0,
            parallel_street_count: 0,
            ..LifecycleConfig::default()
        });
        for (id, pts) in corpus.tracks_through_e() {
            let id = format!("c{c}_{id}");
            sports.insert(id.clone(), "Ride".to_string());
            tracks.push((id, pts));
        }
    }
    (tracks, sports)
}

#[test]
fn a_fold_resumed_from_a_checkpoint_cuts_only_what_is_left() {
    let (tracks, sports) = library(3, 12);
    let config = SectionConfig::default();
    let policy = SectionUpdatePolicy::default();
    let starts = HashMap::new();

    // Cold fold over everything but the last two tracks of each cluster.
    let held: Vec<usize> = (0..3)
        .flat_map(|c| {
            let last = tracks
                .iter()
                .enumerate()
                .filter(|(_, (id, _))| id.starts_with(&format!("c{c}_")))
                .map(|(i, _)| i)
                .max()
                .unwrap();
            [last - 1, last]
        })
        .collect();
    let base: Tracks = tracks
        .iter()
        .enumerate()
        .filter(|(i, _)| !held.contains(i))
        .map(|(_, t)| t.clone())
        .collect();
    let base_ids: Vec<&str> = base.iter().map(|(id, _)| id.as_str()).collect();
    let mut cache = SectionEvidenceCache::new();
    let existing = detect_sections_unified_incremental_observed(
        &mut cache,
        &[],
        &base,
        &base_ids,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |_, _, _| {},
    )
    .catalogue;
    assert!(!existing.is_empty(), "the base library cut no sections");

    // The interrupted fold: every cluster gains two tracks; the checkpoint
    // after the first cluster is what a kill would have left on disk.
    let new_ids: Vec<&str> = held.iter().map(|&i| tracks[i].0.as_str()).collect();
    let mut checkpoint: Option<SectionEvidenceCache> = None;
    let mut seen_total = 0;
    let full = detect_sections_unified_incremental_observed(
        &mut cache,
        &existing,
        &tracks,
        &new_ids,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |done, total, cache| {
            seen_total = total;
            if done == 1 {
                checkpoint = Some(cache.checkpoint());
            }
        },
    )
    .catalogue;
    assert_eq!(
        seen_total, 3,
        "every cluster gained a track, so every cluster is dirty"
    );
    let checkpoint = checkpoint.expect("the observer saw the first cluster");
    assert_eq!(
        checkpoint.dirty_clusters(),
        2,
        "two clusters were still owed"
    );
    assert_eq!(cache.dirty_clusters(), 0, "a completed fold owes nothing");

    // Resume: no new activities, only the dirty clusters are cut.
    let mut resumed = checkpoint;
    let mut cuts = Vec::new();
    let again = detect_sections_unified_incremental_observed(
        &mut resumed,
        &existing,
        &tracks,
        &[],
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |done, total, _| cuts.push((done, total)),
    )
    .catalogue;
    assert_eq!(
        cuts,
        vec![(1, 2), (2, 2)],
        "the resume cut exactly the two clusters left"
    );
    assert_eq!(resumed.dirty_clusters(), 0);
    assert_eq!(
        bitwise::catalogue_digest(&again),
        bitwise::catalogue_digest(&full),
        "the resumed catalogue must be the uninterrupted one"
    );
}
