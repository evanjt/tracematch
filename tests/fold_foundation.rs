//! The incremental fold's foundation: the CONVERGENCE TARGET it must hit, on the seeded
//! synthetic corpus, no veloqrs, no SQLite.
//!
//! PART B pins the CONVERGENCE TARGET. A naive re-batch drip (re-run
//! `detect_sections` over the whole accumulated pool on every add) is by
//! construction the exact catalogue the fold must reproduce, and its per-add
//! cost grows with N, which is why an incremental is mandatory.
//!
//! PART C gates the cached fast path against that target, and gates its cost by
//! counting the work each add does rather than timing it. The timings, and the
//! cost curves this file used to print, are in `benches/fold_cost.rs`.

use std::collections::HashMap;

use tracematch::scenarios::{LifecycleConfig, LifecycleCorpus};
use tracematch::{
    FrequentSection, GpsPoint, SectionConfig, detect_sections, detect_sections_incremental,
};

type Tracks = Vec<(String, Vec<GpsPoint>)>;

// ============================================================================
// Ground-match maths — inlined so this test binary owns its geometry (Rust
// integration test files are independent crates; the sibling parity test keeps
// its own identical copy).
// ============================================================================

const GROUND_TOL_M: f64 = 50.0;
const COVERAGE_FRAC: f64 = 0.6;

fn haversine_m(a: &GpsPoint, b: &GpsPoint) -> f64 {
    let r = 6_371_000.0_f64;
    let (la1, lo1) = (a.latitude.to_radians(), a.longitude.to_radians());
    let (la2, lo2) = (b.latitude.to_radians(), b.longitude.to_radians());
    let dla = la2 - la1;
    let dlo = lo2 - lo1;
    let h = (dla / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlo / 2.0).sin().powi(2);
    2.0 * r * h.sqrt().asin()
}

/// Fraction of `samples` within `tol_m` of any point on `line`.
fn coverage(samples: &[GpsPoint], line: &[GpsPoint], tol_m: f64) -> f64 {
    if samples.is_empty() || line.is_empty() {
        return 0.0;
    }
    let covered = samples
        .iter()
        .filter(|s| {
            line.iter()
                .map(|p| haversine_m(s, p))
                .fold(f64::INFINITY, f64::min)
                <= tol_m
        })
        .count();
    covered as f64 / samples.len() as f64
}

fn ground_matches(a: &FrequentSection, b: &FrequentSection) -> bool {
    coverage(&a.polyline, &b.polyline, GROUND_TOL_M) >= COVERAGE_FRAC
        || coverage(&b.polyline, &a.polyline, GROUND_TOL_M) >= COVERAGE_FRAC
}

/// Greedy 1:1 ground pairing normalised by the larger catalogue. Penalises
/// count mismatch, so a fragmented or padded catalogue scores below 1.0.
fn catalogue_overlap(a: &[FrequentSection], b: &[FrequentSection]) -> f64 {
    if a.is_empty() && b.is_empty() {
        return 1.0;
    }
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let mut used = vec![false; b.len()];
    let mut matched = 0usize;
    for sa in a {
        for (j, sb) in b.iter().enumerate() {
            if !used[j] && ground_matches(sa, sb) {
                used[j] = true;
                matched += 1;
                break;
            }
        }
    }
    matched as f64 / a.len().max(b.len()) as f64
}

// ============================================================================
// Corpus + synthetic new-activity construction
// ============================================================================

/// 24-activity slice (bucket A 20 + C 1 + D 3): fast, still carrying real
/// Unified corridors to seed the matcher with.
fn reduced_corpus() -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig {
        bucket_a_count: 20,
        bucket_b_delta_count: 0,
        bucket_d_delta_count: 0,
        bucket_e_delta_count: 0,
        ..LifecycleConfig::default()
    })
}

/// Larger corpus for the cost curve. bucket A n -> through_e = n + 4.
fn corpus_with_bucket_a(n: usize) -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig {
        bucket_a_count: n,
        bucket_b_delta_count: 0,
        bucket_d_delta_count: 0,
        bucket_e_delta_count: 0,
        ..LifecycleConfig::default()
    })
}

// ============================================================================
// PART B — pure-layer parity + cost contract for the incremental fold
// ============================================================================

/// GATE. The order-free Unified-aware incremental drips the pool one activity
/// at a time, folding each into the prior catalogue, and must converge to the
/// from-scratch Unified batch at >= 0.95 ground overlap. It runs by default.
///
/// `detect_sections_incremental` re-batches the accumulated pool on every fold,
/// so the final fold is the batch by construction. The cached path is held to
/// the same target and to a flat per-add cost by the `gate_cached_*` tests below.
#[test]
fn gate_unified_incremental_converges_to_batch() {
    let corpus = reduced_corpus();
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();
    let cfg = SectionConfig::default();

    let batch = detect_sections(&tracks, &[], &sports, &cfg);

    // Order-free incremental drip: fold one activity at a time into the prior
    // catalogue. `pool` is the accumulated prefix (the new activity included);
    // the fold converges to the batch over that prefix, so the last fold equals
    // the full batch.
    let mut catalogue: Vec<FrequentSection> = Vec::new();
    for n in 1..=tracks.len() {
        let pool = &tracks[..n];
        let result = detect_sections_incremental(&catalogue, pool, &[], &sports, &cfg);
        catalogue = result.catalogue;
    }

    let overlap = catalogue_overlap(&catalogue, &batch);
    assert!(
        overlap >= 0.95,
        "The incremental must converge to the Unified batch: ground overlap {overlap:.3} < 0.95 \
         (batch sections = {}, incremental sections = {}).",
        batch.len(),
        catalogue.len(),
    );
}

// ============================================================================
// PART C — the CACHED cluster-recompute fast path: oracle + cost
// ============================================================================
//
// The optimisation under the same convergence contract. The oracle drips a
// corpus one activity at a time through BOTH the cached fast path (threading one
// &mut cache) and the naive re-batch baseline, and asserts cached == naive ==
// batch at EVERY step — on a single-cluster corpus and on a two-cluster corpus
// (so routing, verbatim reuse, and the bridge merge are all exercised). The
// cost test proves the add cost is flat/sublinear in library size, not linear
// like the naive.

/// A far-apart second corpus, id-prefixed so two single-origin corpora combine
/// into one pool with two geographically disjoint clusters (origins ~100 km
/// apart, far past the 50 km cluster gap).
fn prefixed_tracks(corpus: &LifecycleCorpus, prefix: &str) -> Vec<(String, Vec<GpsPoint>)> {
    corpus
        .tracks_through_e()
        .into_iter()
        .map(|(id, pts)| (format!("{prefix}{id}"), pts))
        .collect()
}

/// A small single-origin corpus for the two-cluster oracle.
fn small_corpus(origin: GpsPoint, seed: u64) -> LifecycleCorpus {
    LifecycleCorpus::generate(&LifecycleConfig {
        origin,
        seed,
        bucket_a_count: 8,
        bucket_b_delta_count: 0,
        bucket_d_delta_count: 0,
        bucket_e_delta_count: 0,
        ..LifecycleConfig::default()
    })
}

/// A coarse straight track spanning the gap between the two origins: its
/// bounding box overlaps both clusters, so adding it forces a bridge merge
/// (and the batch's `geo_clusters` unions them the same way).
fn bridge_track(lat0: f64, lat1: f64, lng: f64) -> Vec<GpsPoint> {
    let n = 90;
    (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64;
            GpsPoint::with_elevation(lat0 + (lat1 - lat0) * t, lng, 400.0)
        })
        .collect()
}

/// Drip `tracks` one at a time through the cached fast path and the naive
/// baseline in lockstep, asserting cached == naive == batch at every step.
/// `sports` must cover every id. Returns nothing; it asserts.
fn assert_cached_tracks_naive_and_batch(
    tracks: &[(String, Vec<GpsPoint>)],
    sports: &HashMap<String, String>,
    cfg: &SectionConfig,
    label: &str,
) {
    use tracematch::{
        SectionEvidenceCache, SectionUpdatePolicy, detect_sections_incremental_cached_with_policy,
    };

    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::with_capacity(tracks.len());
    let mut cache = SectionEvidenceCache::new();
    let mut cached_cat: Vec<FrequentSection> = Vec::new();
    let mut naive_cat: Vec<FrequentSection> = Vec::new();

    for (step, (id, pts)) in tracks.iter().enumerate() {
        pool.push((id.clone(), pts.clone()));
        let new_ids = [pool.last().unwrap().0.as_str()];

        let cached = detect_sections_incremental_cached_with_policy(
            &mut cache,
            &cached_cat,
            &pool,
            &new_ids,
            &[],
            sports,
            cfg,
            &SectionUpdatePolicy::default(),
        );
        cached_cat = cached.catalogue;

        // Naive re-batches the whole pool: naive_cat is the batch by construction.
        let naive = detect_sections_incremental(&naive_cat, &pool, &[], sports, cfg);
        naive_cat = naive.catalogue;

        let cached_vs_naive = catalogue_overlap(&cached_cat, &naive_cat);
        if cached_vs_naive < 0.95 || cached_cat.len() != naive_cat.len() {
            eprintln!(
                "[{label}] MISMATCH step {step} N={}: cached {} vs naive/batch {} sections",
                pool.len(),
                cached_cat.len(),
                naive_cat.len()
            );
            for (sp, m, rl, se) in cache.debug_summary() {
                eprintln!("  cached cluster: {sp} members={m} ref_lat={rl:.6} sections={se}");
            }
        }
        assert!(
            cached_vs_naive >= 0.95,
            "[{label}] step {step} (N={}): cached diverged from naive/batch: ground overlap \
             {cached_vs_naive:.3} < 0.95 (cached {} sections, naive/batch {} sections)",
            pool.len(),
            cached_cat.len(),
            naive_cat.len(),
        );
        // Frozen ref-lat is sub-cell, so the section COUNT must match exactly:
        // a fragmented or padded catalogue would fail this even at 0.95 overlap.
        assert_eq!(
            cached_cat.len(),
            naive_cat.len(),
            "[{label}] step {step} (N={}): cached section count {} != batch {}",
            pool.len(),
            cached_cat.len(),
            naive_cat.len(),
        );
    }

    // Anchor the whole chain to a from-scratch batch: naive's final catalogue IS
    // the batch, and the cached tracked it the whole way.
    let batch = detect_sections(&pool, &[], sports, cfg);
    assert_eq!(
        naive_cat.len(),
        batch.len(),
        "[{label}] naive final != from-scratch batch"
    );
    let final_overlap = catalogue_overlap(&cached_cat, &batch);
    assert!(
        final_overlap >= 0.95 && cached_cat.len() == batch.len(),
        "[{label}] cached final vs batch: overlap {final_overlap:.3}, counts {} vs {}",
        cached_cat.len(),
        batch.len(),
    );
    println!(
        "[{label}] {} activities dripped; final catalogue {} sections, cached==naive==batch \
         at every step.",
        tracks.len(),
        cached_cat.len(),
    );
}

/// ORACLE (single cluster). Every add touches the one home cluster, so the
/// cached path recomputes it wholesale each fold (no untouched cluster to
/// reuse) — this is the pure fold-and-recompute correctness proof, tracking the
/// batch's non-monotone dissolve/reform walk step for step.
#[test]
fn gate_cached_incremental_single_cluster_matches_batch() {
    let corpus = reduced_corpus();
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();
    let cfg = SectionConfig::default();
    assert_cached_tracks_naive_and_batch(&tracks, &sports, &cfg, "single-cluster");
}

/// ORACLE (multi cluster). Two disjoint origins interleaved, then a bridge
/// track that merges them. Exercises cluster routing, verbatim reuse of the
/// untouched cluster on each add, and the bridge merge — all under the same
/// cached == naive == batch assertion.
#[test]
fn gate_cached_incremental_multi_cluster_and_bridge_matches_batch() {
    let north = small_corpus(GpsPoint::with_elevation(47.0, 8.0, 400.0), 0xC0FFEE);
    let south = small_corpus(GpsPoint::with_elevation(47.9, 8.0, 400.0), 0xBEEF);
    let north_tracks = prefixed_tracks(&north, "N_");
    let south_tracks = prefixed_tracks(&south, "S_");

    // Interleave the two clusters so each add reuses the other verbatim.
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut ni = north_tracks.into_iter();
    let mut si = south_tracks.into_iter();
    loop {
        match (ni.next(), si.next()) {
            (Some(n), Some(s)) => {
                tracks.push(n);
                tracks.push(s);
            }
            (Some(n), None) => tracks.push(n),
            (None, Some(s)) => tracks.push(s),
            (None, None) => break,
        }
    }
    // The bridge: one track spanning the ~100 km gap, overlapping both clusters.
    tracks.push(("BRIDGE".to_string(), bridge_track(47.05, 47.85, 8.0)));

    // One sport so the geography alone drives the two clusters and their merge.
    let sports: HashMap<String, String> = tracks
        .iter()
        .map(|(id, _)| (id.clone(), "Ride".to_string()))
        .collect();
    let cfg = SectionConfig::default();
    assert_cached_tracks_naive_and_batch(&tracks, &sports, &cfg, "multi-cluster+bridge");
}

/// A library spread over many far-apart clusters, dripped cluster by cluster, so
/// every add lands in a cluster bounded by one corpus's size however large the
/// whole library grows. `bucket_a` is each corpus's cold-start count (its total
/// through-E size is a little larger). Returns `(tracks in drip order, sport map)`.
fn multi_cluster_library(n_clusters: usize, bucket_a: usize) -> (Tracks, HashMap<String, String>) {
    let mut tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
    let mut sports: HashMap<String, String> = HashMap::new();
    for c in 0..n_clusters {
        // 3° apart (~330 km), far past the 50 km cluster gap. No one-offs or
        // parallel streets: those carry wide random offsets that would stretch a
        // cluster's bbox toward its neighbour and bridge them, defeating the
        // point of measuring the bounded-cluster case.
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

/// COST gate, the O(touched-cluster) property. A cached add recomputes only
/// the cluster the new activity touches, never the whole library: the win the
/// naive re-batch cannot have, since it re-processes the entire pool on every
/// add. The library grows across many far-apart clusters, each the same fresh
/// corpus at a new origin, dripped cluster by cluster. The work of every add,
/// counted as clusters recomputed and the member tracks they held, must stay
/// bounded by one cluster however many clusters already exist.
///
/// A single unbounded cluster is a different, honest story: there the touched
/// cluster IS the whole library, so the add is O(cluster) = O(N). The timings
/// of both shapes are in `benches/fold_cost.rs`.
#[test]
fn gate_cached_incremental_cost_is_flat() {
    use tracematch::{
        SectionEvidenceCache, SectionUpdatePolicy, detect_sections_incremental_cached_with_policy,
    };

    let cfg = SectionConfig::default();
    let n_clusters = 9;
    let (tracks, sports) = multi_cluster_library(n_clusters, 8);
    let cluster_size = tracks.len() / n_clusters;

    let mut pool: Vec<(String, Vec<GpsPoint>)> = Vec::with_capacity(tracks.len());
    let mut cache = SectionEvidenceCache::new();
    let mut cached_cat: Vec<FrequentSection> = Vec::new();
    for (id, pts) in &tracks {
        pool.push((id.clone(), pts.clone()));
        let new_ids = [id.as_str()];
        let before = cache.recompute_work();
        let res = detect_sections_incremental_cached_with_policy(
            &mut cache,
            &cached_cat,
            &pool,
            &new_ids,
            &[],
            &sports,
            &cfg,
            &SectionUpdatePolicy::default(),
        );
        cached_cat = res.catalogue;
        let after = cache.recompute_work();
        let (clusters, members) = (after.0 - before.0, after.1 - before.1);

        assert_eq!(
            clusters,
            1,
            "add of {id} at pool {} recomputed {clusters} clusters; only the one it touches may be",
            pool.len(),
        );
        assert!(
            members <= cluster_size,
            "add of {id} at pool {} fed {members} tracks to the recompute, more than one cluster \
             of {cluster_size}: the work grew with the library",
            pool.len(),
        );
    }
    assert_eq!(
        cache.debug_summary().len(),
        n_clusters,
        "the library must stay {n_clusters} disjoint clusters, or the bound above proves nothing",
    );
}

/// COLD-COST gate. A cold (empty) cache, which is every app start before the
/// cache is persisted and every bulk window-expand, receiving N brand-new
/// activities in ONE call must recompute each touched cluster exactly once over
/// its final membership, so the tracks fed to the recompute sum to N. The
/// earlier recompute-per-activity shape was O(N²): k recomputes of a growing
/// cluster, so the same count came to about N²/2. Uses one growing home
/// cluster, the worst case for that shape. The timings against the plain batch
/// are in `benches/fold_cost.rs`.
#[test]
fn gate_cached_cold_cache_cost_is_linear() {
    use tracematch::{
        SectionEvidenceCache, SectionUpdatePolicy, detect_sections_incremental_cached_with_policy,
    };

    let cfg = SectionConfig::default();
    let corpus = corpus_with_bucket_a(39);
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();

    for n in [20usize, 40] {
        let prefix: Vec<(String, Vec<GpsPoint>)> = tracks[..n].to_vec();
        let new_ids: Vec<&str> = prefix.iter().map(|(id, _)| id.as_str()).collect();
        let mut cache = SectionEvidenceCache::new();
        let _ = detect_sections_incremental_cached_with_policy(
            &mut cache,
            &[],
            &prefix,
            &new_ids,
            &[],
            &sports,
            &cfg,
            &SectionUpdatePolicy::default(),
        );
        let (clusters, members) = cache.recompute_work();
        let formed = cache.debug_summary().len();
        assert_eq!(
            clusters, formed,
            "cold detect over {n} recomputed {clusters} clusters for the {formed} it formed; \
             each must be cut once",
        );
        assert_eq!(
            members, n,
            "cold detect over {n} fed {members} tracks to the recompute; each activity must be \
             read once, not once per activity that arrived after it",
        );
    }
}
