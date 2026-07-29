//! Contracts for the fold's update policy: pins, frozen geometry, and
//! the delta lists a list-holding consumer relies on.
//!
//! The scenario throughout is the late-emerging junction
//! (`shapes::late_fork`): a trunk detected from straight-through outings
//! that the evidence later wants to re-cut when branch outings arrive.
//! The default policy follows the evidence (see
//! `b1_lifecycle_scenarios.rs`); these tests hold the POLICY behaviours:
//! a pinned trunk keeps its geometry and its corridor, withheld re-cuts
//! stay observable in `held`, evidence keeps growing on frozen geometry,
//! and dissolves stay live under `freeze_all_geometry`.

mod shapes;

use std::collections::HashMap;
use tracematch::geo_utils::haversine_distance;
use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, SectionEvidenceCache, SectionUpdatePolicy,
    detect_sections_unified_incremental_cached,
    detect_sections_unified_incremental_cached_with_policy,
};

fn endpoints_match(a: &FrequentSection, b: &FrequentSection, tol_m: f64) -> bool {
    let (Some(a0), Some(a1), Some(b0), Some(b1)) = (
        a.polyline.first(),
        a.polyline.last(),
        b.polyline.first(),
        b.polyline.last(),
    ) else {
        return false;
    };
    let fwd = haversine_distance(a0, b0).max(haversine_distance(a1, b1));
    let rev = haversine_distance(a0, b1).max(haversine_distance(a1, b0));
    fwd.min(rev) <= tol_m
}

/// Drip the late-fork corpus one activity at a time. `pin_after` names
/// the step after which every section then in the catalogue is pinned.
/// Returns the final result plus the catalogue as it stood when the pins
/// were taken.
fn drip_with_pins(
    pin_after: usize,
) -> (tracematch::UnifiedIncrementalResult, Vec<FrequentSection>) {
    let tracks = shapes::late_fork(6, 6);
    let sports: HashMap<String, String> = shapes::pooled(&tracks);
    let cfg = SectionConfig::default();
    let mut cache = SectionEvidenceCache::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut catalogue: Vec<FrequentSection> = Vec::new();
    let mut policy = SectionUpdatePolicy::default();
    let mut pinned_snapshot = Vec::new();
    let mut last = None;

    for (step, (id, pts)) in tracks.iter().enumerate() {
        pool.push((id.clone(), pts.clone()));
        let new_ids = [pool.last().unwrap().0.as_str()];
        let res = detect_sections_unified_incremental_cached_with_policy(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &[],
            &sports,
            &cfg,
            &policy,
        );
        catalogue = res.catalogue.clone();
        last = Some(res);
        if step + 1 == pin_after {
            policy.pinned_ids = catalogue.iter().map(|s| s.id.clone()).collect();
            pinned_snapshot = catalogue.clone();
        }
    }
    (last.expect("at least one step"), pinned_snapshot)
}

#[test]
fn pinned_trunk_keeps_its_cut_and_claims_its_corridor() {
    // Pin the trunk after the six straight outings, then let the six
    // branch outings arrive. The evidence wants trunk + tail + branches;
    // the pin holds the whole original corridor as one, so only ground
    // clear of it (the branches past the junction) may land.
    let (res, pinned) = drip_with_pins(6);
    assert_eq!(pinned.len(), 1, "one trunk section at pin time");
    let trunk = &pinned[0];

    let kept = res
        .catalogue
        .iter()
        .find(|s| s.id == trunk.id)
        .expect("pinned section must survive every subsequent fold");
    assert!(
        endpoints_match(kept, trunk, 5.0),
        "pinned geometry must be exactly the pinned cut"
    );
    assert!(
        res.dissolved.iter().all(|d| d.id != trunk.id),
        "a pinned section never dissolves"
    );
    for s in &res.catalogue {
        if s.id != trunk.id {
            assert!(
                !tracematch::shares_ground(&s.polyline, &trunk.polyline),
                "{}: no emitted section may share the pinned corridor",
                s.id
            );
        }
    }
}

#[test]
fn withheld_recuts_are_observable_not_silent() {
    // The branch arrivals re-cut the trunk in the evidence. With the
    // trunk pinned the catalogue must hold still AND the drift must be
    // visible: the withheld cut is reported in `held`, never in
    // `changed`, and `changed` stays empty (nothing visible moved).
    let (res, pinned) = drip_with_pins(6);
    let trunk_id = &pinned[0].id;
    assert!(
        res.held.iter().any(|h| h.previous.id == *trunk_id),
        "the withheld trunk re-cut must appear in held"
    );
    assert!(
        res.changed.iter().all(|c| c.previous.id != *trunk_id),
        "a pinned section never appears in changed"
    );
}

/// Order-insensitive catalogue identity, mirroring the batch-parity
/// helper in `unified_contracts.rs`: everything that defines the
/// catalogue as a pure function of the activity set.
fn normalise(sections: &[FrequentSection]) -> String {
    let mut rows: Vec<String> = sections
        .iter()
        .map(|s| {
            let coords: Vec<(i64, i64)> = s
                .polyline
                .iter()
                .map(|p| {
                    (
                        (p.latitude * 1e6).round() as i64,
                        (p.longitude * 1e6).round() as i64,
                    )
                })
                .collect();
            let mut acts = s.activity_ids.clone();
            acts.sort();
            acts.dedup();
            format!(
                "{}|{}|{}|{}|{:.0}|{:?}",
                s.id,
                s.representative_activity_id,
                s.visit_count,
                acts.join(","),
                s.distance_meters,
                coords
            )
        })
        .collect();
    rows.sort();
    rows.join("\n")
}

#[test]
fn unpin_releases_the_withheld_recut() {
    // Pin the trunk after the six straight outings, drip the six branch
    // outings (the junction re-cut is withheld into `held`), then clear
    // the pin and fold once more over the same pool with no new
    // activity. The policy is per call and discovery was never altered
    // by the pin, so the very next fold emits the released re-cut: the
    // trunk lands in `changed`, nothing stays in `held`, and the
    // catalogue converges to the from-scratch batch over the pool.
    use tracematch::detect_sections_unified;

    let tracks = shapes::late_fork(6, 6);
    let sports: HashMap<String, String> = shapes::pooled(&tracks);
    let cfg = SectionConfig::default();
    let mut cache = SectionEvidenceCache::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut catalogue: Vec<FrequentSection> = Vec::new();
    let mut policy = SectionUpdatePolicy::default();
    let mut trunk_id = String::new();
    let mut withheld = false;

    for (step, (id, pts)) in tracks.iter().enumerate() {
        pool.push((id.clone(), pts.clone()));
        let new_ids = [pool.last().unwrap().0.as_str()];
        let res = detect_sections_unified_incremental_cached_with_policy(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &[],
            &sports,
            &cfg,
            &policy,
        );
        if !trunk_id.is_empty() {
            withheld |= res.held.iter().any(|h| h.previous.id == trunk_id);
        }
        catalogue = res.catalogue;
        if step + 1 == 6 {
            policy.pinned_ids = catalogue.iter().map(|s| s.id.clone()).collect();
            assert_eq!(policy.pinned_ids.len(), 1, "one trunk section at pin time");
            trunk_id = policy.pinned_ids[0].clone();
        }
    }
    assert!(
        withheld,
        "the trunk re-cut must have been withheld while pinned"
    );

    policy.pinned_ids.clear();
    let res = detect_sections_unified_incremental_cached_with_policy(
        &mut cache,
        &catalogue,
        &pool,
        &[],
        &[],
        &sports,
        &cfg,
        &policy,
    );
    assert!(res.held.is_empty(), "an unpinned fold withholds nothing");
    assert!(
        res.changed.iter().any(|c| c.previous.id == trunk_id),
        "the released re-cut must land as a visible geometry change on the trunk"
    );
    assert!(
        res.carried.iter().any(|(p, _)| p == &trunk_id),
        "the trunk's ground survives under the fresh cut"
    );

    let batch = detect_sections_unified(&pool, &[], &sports, &cfg);
    assert_eq!(
        normalise(&res.catalogue),
        normalise(&batch),
        "clearing the pin must converge the catalogue to the from-scratch batch"
    );
}

#[test]
fn pinned_evidence_grows_append_only_on_frozen_geometry() {
    // Branch outings still traverse the trunk's first kilometre, so the
    // pinned trunk's traversal evidence must keep rising while its
    // geometry, id, and name stay frozen.
    let (res, pinned) = drip_with_pins(6);
    let trunk = &pinned[0];
    let kept = res
        .catalogue
        .iter()
        .find(|s| s.id == trunk.id)
        .expect("pinned section survives");
    assert!(
        kept.visit_count > trunk.visit_count,
        "visit count must grow on frozen geometry: {} -> {}",
        trunk.visit_count,
        kept.visit_count
    );
    assert!(
        kept.activity_ids.len() > trunk.activity_ids.len(),
        "membership must grow append-only"
    );
    for aid in &trunk.activity_ids {
        assert!(
            kept.activity_ids.contains(aid),
            "graft must never remove a member ({aid})"
        );
    }
    let grafted: Vec<_> = kept
        .activity_portions
        .iter()
        .filter(|p| !trunk.activity_ids.contains(&p.activity_id))
        .collect();
    assert!(
        !grafted.is_empty(),
        "new members must carry portions computed against the frozen line"
    );
    for p in grafted {
        assert!(
            p.distance_meters > 0.0 && p.end_index > p.start_index,
            "grafted portion must be a real range"
        );
    }
}

#[test]
fn freeze_all_holds_geometry_but_not_existence() {
    // Freeze everything after the straight-only phase, then drip the
    // branches. Geometry holds (held reports the withheld re-cut), but
    // the catalogue is not silenced into monotone growth: new clear
    // ground still lands in `added`.
    let tracks = shapes::late_fork(6, 6);
    let sports: HashMap<String, String> = shapes::pooled(&tracks);
    let cfg = SectionConfig::default();
    let mut cache = SectionEvidenceCache::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut catalogue: Vec<FrequentSection> = Vec::new();
    let policy_frozen = SectionUpdatePolicy {
        pinned_ids: Vec::new(),
        freeze_all_geometry: true,
    };

    let mut frozen_at = Vec::new();
    let mut held_seen = false;
    let mut added_after_freeze = 0usize;
    for (step, (id, pts)) in tracks.iter().enumerate() {
        pool.push((id.clone(), pts.clone()));
        let new_ids = [pool.last().unwrap().0.as_str()];
        let policy = if step < 6 {
            SectionUpdatePolicy::default()
        } else {
            policy_frozen.clone()
        };
        let res = detect_sections_unified_incremental_cached_with_policy(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &[],
            &sports,
            &cfg,
            &policy,
        );
        if step == 5 {
            frozen_at = res.catalogue.clone();
        }
        if step >= 6 {
            held_seen |= !res.held.is_empty();
            added_after_freeze += res.added.len();
        }
        catalogue = res.catalogue;
    }

    for f in &frozen_at {
        if let Some(now) = catalogue.iter().find(|s| s.id == f.id) {
            assert!(
                endpoints_match(now, f, 5.0),
                "{}: frozen geometry must not move",
                f.id
            );
        }
    }
    assert!(held_seen, "withheld re-cuts must be reported in held");
    assert!(
        added_after_freeze > 0,
        "genuinely new ground (the branches) must still land while frozen"
    );
}

#[test]
fn default_policy_delta_is_internally_consistent() {
    // Under the default policy the three lists plus carried must account
    // for every prior and every catalogue entry exactly: nothing silently
    // appears or disappears.
    let tracks = shapes::late_fork(6, 6);
    let sports: HashMap<String, String> = shapes::pooled(&tracks);
    let cfg = SectionConfig::default();
    let mut cache = SectionEvidenceCache::new();
    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut catalogue: Vec<FrequentSection> = Vec::new();

    for (id, pts) in &tracks {
        pool.push((id.clone(), pts.clone()));
        let new_ids = [pool.last().unwrap().0.as_str()];
        let prior = catalogue.clone();
        let res = detect_sections_unified_incremental_cached(
            &mut cache,
            &catalogue,
            &pool,
            &new_ids,
            &[],
            &sports,
            &cfg,
        );
        assert!(res.held.is_empty(), "default policy holds nothing back");
        assert_eq!(
            prior.len(),
            res.carried.len() + res.dissolved.len() + res.merged.len(),
            "every prior is carried, dissolved, or merged"
        );
        assert_eq!(
            res.catalogue.len(),
            res.carried.len() + res.added.len(),
            "every catalogue entry is carried or added"
        );
        for (prior_id, cat_id) in &res.carried {
            assert!(prior.iter().any(|p| p.id == *prior_id));
            assert!(res.catalogue.iter().any(|c| c.id == *cat_id));
        }
        for c in &res.changed {
            assert!(
                res.carried.iter().any(|(p, _)| p == &c.previous.id),
                "changed pairs must be carried pairs"
            );
        }
        catalogue = res.catalogue;
    }
}

#[test]
fn merged_ground_is_not_reported_as_dissolved() {
    // A caller holding two half-corridor sections whose evidence now
    // supports one continuous cut: the senior half carries, the junior's
    // id leaves the list. Its ground did NOT leave, so it must be
    // reported in `merged` (with the surviving id) and never in
    // `dissolved`.
    use tracematch::{detect_sections_unified, detect_sections_unified_incremental};

    let cfg = SectionConfig::default();
    let half = |x0: f64, x1: f64, n: usize, tag: &str| -> Vec<(String, Vec<GpsPoint>)> {
        (0..n)
            .map(|i| {
                let path = shapes::densify(&[(x0, 0.0), (x1, 0.0)]);
                (
                    format!("{tag}_{i}"),
                    shapes::track(&shapes::wobble(&path, shapes::HUMAN_WOBBLE_M, i as f64)),
                )
            })
            .collect()
    };

    let west = half(0.0, 700.0, 4, "west");
    let east = half(700.0, 1400.0, 4, "east");
    let west_sections = detect_sections_unified(&west, &[], &shapes::pooled(&west), &cfg);
    let east_sections = detect_sections_unified(&east, &[], &shapes::pooled(&east), &cfg);
    assert_eq!(west_sections.len(), 1, "west half detects as one section");
    assert_eq!(east_sections.len(), 1, "east half detects as one section");
    let mut existing = west_sections;
    let mut junior = east_sections;
    // Distinct ids: the two separate detects both number from zero.
    junior[0].id = format!("{}_east", junior[0].id);
    let junior_id = junior[0].id.clone();
    let senior_id = existing[0].id.clone();
    existing.append(&mut junior);

    let full = half(0.0, 1400.0, 6, "full");
    let res =
        detect_sections_unified_incremental(&existing, &full, &[], &shapes::pooled(&full), &cfg);
    assert_eq!(res.catalogue.len(), 1, "one continuous cut wins");
    assert!(
        res.merged
            .iter()
            .any(|m| m.previous.id == junior_id && m.into_id == senior_id),
        "the junior half must be reported merged into the senior ({:?})",
        res.merged
            .iter()
            .map(|m| (&m.previous.id, &m.into_id))
            .collect::<Vec<_>>()
    );
    assert!(
        res.dissolved.is_empty(),
        "surviving ground must not be reported dissolved"
    );
    assert!(
        res.carried.iter().any(|(p, _)| p == &senior_id),
        "the senior half carries onto the continuous cut"
    );
}

#[test]
fn emitted_ids_stay_unique_under_pins() {
    // A pinned id from an earlier fold is positional (`sec_<sport>_<n>`),
    // exactly the scheme fresh sections are renumbered with, so a fresh
    // section on other ground can collide with the held id. The fold must
    // disambiguate deterministically, never emit two sections with one id,
    // and never re-id the pinned section itself.
    let (res, pinned) = drip_with_pins(6);
    let mut seen = std::collections::HashSet::new();
    for s in &res.catalogue {
        assert!(seen.insert(s.id.clone()), "duplicate emitted id {}", s.id);
    }
    assert!(
        res.catalogue.iter().any(|s| s.id == pinned[0].id),
        "the pinned id must survive verbatim"
    );
}
