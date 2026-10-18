//! Scenario: a folded evidence cache is encoded, as the engine persists it,
//! and decoded in the next process.
//!
//! Expected behaviour: the per-track cuts, their cell-set intern table and the
//! per-contributor support cells stay behind, because a restart rebuilds them
//! for a small part of one add. Every other memo comes back whole, and a fold
//! from the decoded cache emits the catalogue the in-memory one does.

mod bitwise;

use std::collections::HashMap;

use bitwise::catalogue_digest;
use tracematch::scenarios::{LifecycleActivity, LifecycleConfig, LifecycleCorpus};
use tracematch::{
    GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    detect_sections_incremental_dated,
};

const NOT_PERSISTED: [&str; 3] = ["cell_sets", "track_portions", "support_cells"];

fn corpus() -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig {
        bucket_a_count: 40,
        bucket_b_delta_count: 0,
        bucket_d_delta_count: 6,
        bucket_e_delta_count: 0,
        parallel_street_count: 0,
        ..LifecycleConfig::default()
    })
}

fn tracks(activities: &[&LifecycleActivity]) -> Vec<(String, Vec<GpsPoint>)> {
    activities
        .iter()
        .map(|a| (a.id.clone(), a.gps_points.clone()))
        .collect()
}

#[test]
fn a_persisted_cache_leaves_out_the_memos_a_restart_rebuilds_cheaply() {
    let corpus = corpus();
    let sports = corpus.sport_map_through_e();
    let starts: HashMap<String, i64> = corpus
        .through_e()
        .iter()
        .map(|a| (a.id.clone(), a.start_date_unix))
        .collect();
    let config = SectionConfig::default();
    let policy = SectionUpdatePolicy::default();

    let mut all = tracks(&corpus.through_e());
    let held = all.pop().expect("a corpus");
    let ids: Vec<&str> = all.iter().map(|(id, _)| id.as_str()).collect();
    let mut warm = SectionEvidenceCache::new();
    let catalogue = detect_sections_incremental_dated(
        &mut warm,
        &[],
        &all,
        &ids,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
    )
    .catalogue;
    assert!(
        !catalogue.is_empty(),
        "the fold cut nothing, so the rest proves nothing"
    );

    let before = warm.memo_counts();
    for name in NOT_PERSISTED
        .iter()
        .chain(&["node_portions", "node_sets", "consensus"])
    {
        assert!(
            before[name] > 0,
            "{name} is empty before the encode, so the test is vacuous"
        );
    }

    let bytes = rmp_serde::to_vec_named(&warm).expect("encode");
    let mut restored: SectionEvidenceCache = rmp_serde::from_slice(&bytes).expect("decode");
    assert!(restored.is_current());

    let after = restored.memo_counts();
    for (name, &count) in &before {
        let expected = if NOT_PERSISTED.contains(name) {
            0
        } else {
            count
        };
        assert_eq!(after[name], expected, "{name} after a round trip");
    }

    let old_bytes = rmp_serde::to_vec_named(&warm.with_rebuildable_memos()).expect("encode");
    let from_old: SectionEvidenceCache =
        rmp_serde::from_slice(&old_bytes).expect("decode an old row");
    assert_eq!(
        from_old.memo_counts(),
        after,
        "an old row decodes as a new one"
    );
    assert!(
        bytes.len() < old_bytes.len(),
        "{} B persisted against {} B with every memo",
        bytes.len(),
        old_bytes.len()
    );

    all.push(held);
    let new_id = [all.last().unwrap().0.as_str()];
    let from_memory = detect_sections_incremental_dated(
        &mut warm,
        &catalogue,
        &all,
        &new_id,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
    )
    .catalogue;
    let from_decoded = detect_sections_incremental_dated(
        &mut restored,
        &catalogue,
        &all,
        &new_id,
        &[],
        &sports,
        &starts,
        &config,
        &policy,
    )
    .catalogue;
    assert_eq!(
        catalogue_digest(&from_decoded),
        catalogue_digest(&from_memory)
    );
}
