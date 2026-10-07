//! A fold killed after some clusters resumes from its last checkpoint: the
//! clusters already cut stay cut, only the rest are recomputed, and the
//! catalogue is the one an uninterrupted fold emits.

mod bitwise;

use std::collections::HashMap;
use std::ops::ControlFlow;

use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::{
    FoldStopped, GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    detect_sections_incremental_observed,
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
    let existing = detect_sections_incremental_observed(
        &mut cache,
        &[],
        &base,
        &base_ids,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |_, _, _| ControlFlow::Continue(()),
    )
    .unwrap()
    .catalogue;
    assert!(!existing.is_empty(), "the base library cut no sections");

    // The interrupted fold: every cluster gains two tracks; the checkpoint
    // after the first cluster is what a kill would have left on disk.
    let new_ids: Vec<&str> = held.iter().map(|&i| tracks[i].0.as_str()).collect();
    let mut checkpoint: Option<SectionEvidenceCache> = None;
    let mut seen_total = 0;
    let full = detect_sections_incremental_observed(
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
            ControlFlow::Continue(())
        },
    )
    .unwrap()
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
    let again = detect_sections_incremental_observed(
        &mut resumed,
        &existing,
        &tracks,
        &[],
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |done, total, _| {
            cuts.push((done, total));
            ControlFlow::Continue(())
        },
    )
    .unwrap()
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

/// Two clusters gain a track each and a third gains nothing. Returns the
/// pool, the sports, the base catalogue, the cache that built it, and the ids
/// the second fold routes.
fn grown_library() -> (
    Tracks,
    HashMap<String, String>,
    Vec<tracematch::FrequentSection>,
    SectionEvidenceCache,
    Vec<String>,
) {
    let (tracks, sports) = library(3, 12);
    let config = SectionConfig::default();
    let policy = SectionUpdatePolicy::default();
    let last_of = |c: usize| {
        tracks
            .iter()
            .filter(|(id, _)| id.starts_with(&format!("c{c}_")))
            .map(|(id, _)| id.clone())
            .next_back()
            .unwrap()
    };
    let new_ids = vec![last_of(0), last_of(1)];
    let base: Tracks = tracks
        .iter()
        .filter(|(id, _)| !new_ids.contains(id))
        .cloned()
        .collect();
    let base_ids: Vec<&str> = base.iter().map(|(id, _)| id.as_str()).collect();
    let mut cache = SectionEvidenceCache::new();
    let existing = detect_sections_incremental_observed(
        &mut cache,
        &[],
        &base,
        &base_ids,
        &[],
        &sports,
        &HashMap::new(),
        &config,
        &policy,
        &mut |_, _, _| ControlFlow::Continue(()),
    )
    .unwrap()
    .catalogue;
    (tracks, sports, existing, cache, new_ids)
}

#[test]
fn the_resolve_reads_only_the_clusters_a_fold_recomputed() {
    let (tracks, sports, existing, mut cache, new_ids) = grown_library();
    let new: Vec<&str> = new_ids.iter().map(String::as_str).collect();
    let mut pending_at_resolve: Vec<String> = Vec::new();
    detect_sections_incremental_observed(
        &mut cache,
        &existing,
        &tracks,
        &new,
        &[],
        &sports,
        &HashMap::new(),
        &SectionConfig::default(),
        &SectionUpdatePolicy::default(),
        &mut |done, total, cache| {
            if done == total {
                pending_at_resolve = cache.resolve_pending_members();
            }
            ControlFlow::Continue(())
        },
    )
    .unwrap();
    assert!(
        pending_at_resolve.iter().all(|id| !id.starts_with("c2_")),
        "the untouched cluster is not read: {pending_at_resolve:?}"
    );
    assert!(pending_at_resolve.iter().any(|id| id.starts_with("c0_")));
    assert!(pending_at_resolve.iter().any(|id| id.starts_with("c1_")));
    assert!(
        cache.resolve_pending_members().is_empty(),
        "a completed fold leaves nothing awaiting its resolve"
    );
}

#[test]
fn a_resumed_fold_still_reads_the_clusters_cut_before_the_checkpoint() {
    let (tracks, sports, existing, mut cache, new_ids) = grown_library();
    let new: Vec<&str> = new_ids.iter().map(String::as_str).collect();
    let mut checkpoint: Option<SectionEvidenceCache> = None;
    detect_sections_incremental_observed(
        &mut cache,
        &existing,
        &tracks,
        &new,
        &[],
        &sports,
        &HashMap::new(),
        &SectionConfig::default(),
        &SectionUpdatePolicy::default(),
        &mut |done, _, cache| {
            if done == 1 {
                checkpoint = Some(cache.checkpoint());
            }
            ControlFlow::Continue(())
        },
    )
    .unwrap();
    let mut resumed = checkpoint.expect("the observer saw the first cluster");
    assert_eq!(resumed.dirty_clusters(), 1, "one cluster was still owed");
    let before = resumed.resolve_pending_members();
    assert!(
        before.iter().any(|id| id.starts_with("c0_")),
        "the cluster cut before the checkpoint awaits its resolve: {before:?}"
    );
    assert!(
        before
            .iter()
            .all(|id| !id.starts_with("c1_") && !id.starts_with("c2_")),
        "{before:?}"
    );

    let mut pending_at_resolve: Vec<String> = Vec::new();
    detect_sections_incremental_observed(
        &mut resumed,
        &existing,
        &tracks,
        &[],
        &[],
        &sports,
        &HashMap::new(),
        &SectionConfig::default(),
        &SectionUpdatePolicy::default(),
        &mut |done, total, cache| {
            if done == total {
                pending_at_resolve = cache.resolve_pending_members();
            }
            ControlFlow::Continue(())
        },
    )
    .unwrap();
    assert!(pending_at_resolve.iter().any(|id| id.starts_with("c0_")));
    assert!(pending_at_resolve.iter().any(|id| id.starts_with("c1_")));
    assert!(pending_at_resolve.iter().all(|id| !id.starts_with("c2_")));
    assert!(resumed.resolve_pending_members().is_empty());
}

#[test]
fn a_fold_stopped_after_one_cluster_resumes_to_the_uninterrupted_catalogue() {
    let (tracks, sports, existing, cache, new_ids) = grown_library();
    let new: Vec<&str> = new_ids.iter().map(String::as_str).collect();
    let config = SectionConfig::default();
    let policy = SectionUpdatePolicy::default();
    let starts = HashMap::new();

    let mut uninterrupted_cache = cache.clone();
    let full = detect_sections_incremental_observed(
        &mut uninterrupted_cache,
        &existing,
        &tracks,
        &new,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |_, _, _| ControlFlow::Continue(()),
    )
    .expect("a fold that never breaks completes");

    let mut stopped_cache = cache;
    let mut calls = 0;
    let stopped = detect_sections_incremental_observed(
        &mut stopped_cache,
        &existing,
        &tracks,
        &new,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |_, _, _| {
            calls += 1;
            ControlFlow::Break(())
        },
    );
    assert!(
        matches!(stopped, Err(FoldStopped { done: 1, total: 2 })),
        "the fold stops at the first cluster boundary: {:?}",
        stopped.as_ref().map(|r| r.catalogue.len())
    );
    assert_eq!(calls, 1, "no cluster is cut after the observer breaks");
    assert_eq!(
        stopped_cache.dirty_clusters(),
        1,
        "the cut cluster is clean and the other is still owed"
    );

    let mut cuts = Vec::new();
    let resumed = detect_sections_incremental_observed(
        &mut stopped_cache,
        &existing,
        &tracks,
        &[],
        &[],
        &sports,
        &starts,
        &config,
        &policy,
        &mut |done, total, _| {
            cuts.push((done, total));
            ControlFlow::Continue(())
        },
    )
    .expect("the resumed fold completes");
    assert_eq!(cuts, vec![(1, 1)], "only the cluster left is recomputed");
    assert_eq!(
        bitwise::catalogue_digest(&resumed.catalogue),
        bitwise::catalogue_digest(&full.catalogue),
        "the resumed catalogue must be the uninterrupted one"
    );
}
