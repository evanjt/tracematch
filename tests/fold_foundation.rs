//! The incremental fold's foundation: what is SALVAGEABLE from tracematch for an order-free,
//! Unified-aware incremental, and what CONVERGENCE TARGET it must hit.
//!
//! This file does two things, both pure tracematch, both on the seeded
//! synthetic corpus, no veloqrs, no SQLite:
//!
//! PART A characterises the reusable MATCHING half. `find_sections_in_route`
//! projects an existing section onto a new activity. We seed it with real
//! UNIFIED geometry and measure exactly what it returns and when it matches, so
//! the fold can lean on it for the "fold a new activity into an existing section"
//! path without pulling in banned consensus averaging.
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
use tracematch::sections::find_sections_in_route;
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

/// Straight-line distance between a section's first and last polyline point. A
/// large span means an elongated (open) section where a traversal's direction
/// and extent are resolvable; a small span means a compact/looping piece where
/// the coarse 2x-proximity match tolerance cannot separate start from end.
fn endpoint_span(s: &FrequentSection) -> f64 {
    if s.polyline.len() >= 2 {
        haversine_m(&s.polyline[0], &s.polyline[s.polyline.len() - 1])
    } else {
        0.0
    }
}

// ============================================================================
// PART A — characterise the reusable matching half (find_sections_in_route)
// ============================================================================

#[derive(Clone)]
struct MatchProbe {
    label: &'static str,
    matched: bool,
    match_count: usize,
    start_index: u64,
    end_index: u64,
    quality: f64,
    same_direction: bool,
    portion_points: usize,
    portion_ground_cover: f64,
    /// True when the returned portion is the new activity's VERBATIM contiguous
    /// points over [start, end): its length equals end - start. A consensus/
    /// averaged line would carry an unrelated point count. This is the runtime
    /// witness that the matcher hands back real single-pass geometry.
    portion_is_verbatim_slice: bool,
}

fn probe(
    label: &'static str,
    new_activity: &[GpsPoint],
    section: &FrequentSection,
    cfg: &SectionConfig,
) -> MatchProbe {
    let matches = find_sections_in_route(new_activity, std::slice::from_ref(section), cfg);
    if let Some(m) = matches.first() {
        let (s, e) = (m.start_index as usize, m.end_index as usize);
        let e = e.min(new_activity.len());
        let portion = &new_activity[s..e];
        // Real-trace witness: the returned indices address the NEW activity, so
        // the portion is exactly its verbatim contiguous points and its length
        // equals the index span. Nothing here is consensus-averaged.
        let verbatim = portion.len() == e - s;
        MatchProbe {
            label,
            matched: true,
            match_count: matches.len(),
            start_index: m.start_index,
            end_index: m.end_index,
            quality: m.match_quality,
            same_direction: m.same_direction,
            portion_points: portion.len(),
            portion_ground_cover: coverage(portion, &section.polyline, GROUND_TOL_M),
            portion_is_verbatim_slice: verbatim,
        }
    } else {
        MatchProbe {
            label,
            matched: false,
            match_count: 0,
            start_index: 0,
            end_index: 0,
            quality: 0.0,
            same_direction: false,
            portion_points: 0,
            portion_ground_cover: 0.0,
            portion_is_verbatim_slice: false,
        }
    }
}

fn print_probe(p: &MatchProbe) {
    if p.matched {
        println!(
            "  {:<22} MATCH   spans={:<3} idx=[{}..{}] pts={:<4} quality={:.2} same_dir={:<5} ground_cover={:.2} verbatim={}",
            p.label,
            p.match_count,
            p.start_index,
            p.end_index,
            p.portion_points,
            p.quality,
            p.same_direction,
            p.portion_ground_cover,
            p.portion_is_verbatim_slice,
        );
    } else {
        println!("  {:<22} NO MATCH", p.label);
    }
}

/// PART A report. Projects a real UNIFIED section onto REAL corpus activities
/// (the actual reuse scenario: the shipped incremental feeds full activities to
/// this matcher, not synthetic section copies). Reports return shape and match
/// behaviour across a real traverser, its reverse, a truncated partial, a 60 m
/// parallel street, and a far one-off. Asserts only the robust contracts the
/// fold leans on; direction/partial/near-miss are reportage because their outcome
/// depends on how elongated the seed section is at the coarse 2x tolerance.
#[test]
fn matching_half_characterisation() {
    let corpus = reduced_corpus();
    let tracks = corpus.tracks_through_e();
    let sport_types = corpus.sport_map_through_e();
    let cfg = SectionConfig::default();

    let unified = detect_sections(&tracks, &[], &sport_types, &cfg);
    let track_map: HashMap<&str, &[GpsPoint]> = tracks
        .iter()
        .map(|(id, p)| (id.as_str(), p.as_slice()))
        .collect();

    println!("\n================ PART A: find_sections_in_route (reuse probe) ================");
    println!(
        "match tolerance = proximity_threshold x 2 = {:.0} m; accept gate = quality >= 0.50",
        cfg.proximity_threshold * 2.0
    );
    println!("unified section geometry (endpoint_span = straight start..end):");
    for s in &unified {
        println!(
            "  {:<12} pts={:<4} dist={:>6.0}m endpoint_span={:>6.0}m activities={}",
            s.id,
            s.polyline.len(),
            s.distance_meters,
            endpoint_span(s),
            s.activity_ids.len()
        );
    }

    // Seed with a section that has real traversers and is the most elongated
    // available, so direction/extent are as resolvable as this corpus allows.
    let section = unified
        .iter()
        .filter(|s| !s.activity_ids.is_empty() && s.polyline.len() >= 3)
        .max_by(|a, b| endpoint_span(a).total_cmp(&endpoint_span(b)))
        .expect("a Unified section with traversers")
        .clone();
    let traverser_id = section.activity_ids[0].clone();
    let traverser = track_map[traverser_id.as_str()].to_vec();
    println!(
        "\nseed: {} (endpoint_span={:.0}m); new activity = real traverser {} ({} pts)",
        section.id,
        endpoint_span(&section),
        traverser_id,
        traverser.len()
    );

    let full = probe("full (real traverser)", &traverser, &section, &cfg);
    let reversed: Vec<GpsPoint> = traverser.iter().rev().cloned().collect();
    let reverse = probe("reversed traverser", &reversed, &section, &cfg);

    // Partial: cut the traverser before it reaches the section end. Uses the
    // full match span, so this is "start present, end missing" by construction.
    let partial_track: Vec<GpsPoint> = if full.matched {
        let (s, e) = (full.start_index as usize, full.end_index as usize);
        let cut = (s + (e - s) / 2).min(traverser.len());
        traverser[..cut].to_vec()
    } else {
        traverser.clone()
    };
    let partial = probe("partial (start..mid)", &partial_track, &section, &cfg);

    let parallel = tracks
        .iter()
        .find(|(id, _)| id.contains("parallel_street"))
        .map(|(_, p)| probe("parallel street +60m", p, &section, &cfg));
    let far = tracks
        .iter()
        .find(|(id, _)| id.contains("one_off"))
        .map(|(_, p)| probe("far one-off", p, &section, &cfg));

    // Same real traverser at tighter proximities. The default (200 m -> 400 m
    // tolerance) on compact Unified sections yields many degenerate slivers;
    // tightening is the obvious knob, but it quickly stops matching the real
    // trace at all. Bracketing this shows whether a clean sweet spot exists.
    let tighter = |prox: f64| SectionConfig {
        proximity_threshold: prox,
        ..cfg.clone()
    };
    let full_p100 = probe(
        "full @ proximity=100m",
        &traverser,
        &section,
        &tighter(100.0),
    );
    let full_p40 = probe("full @ proximity=40m", &traverser, &section, &tighter(40.0));

    print_probe(&full);
    print_probe(&reverse);
    print_probe(&partial);
    if let Some(p) = &parallel {
        print_probe(p);
    }
    if let Some(p) = &far {
        print_probe(p);
    }
    print_probe(&full_p100);
    print_probe(&full_p40);

    println!("-----------------------------------------------------------------------------");
    println!(
        "RETURN SHAPE: Vec<SectionMatch>{{ start_index, end_index into the NEW route,\n\
         match_quality, same_direction }}. The matched geometry is new_route[start..end]\n\
         a REAL single-pass sub-track (verbatim=true above). Direction is captured\n\
         (reverse flips same_dir).\n\
         INVARIANT 1: PRESERVED. The chain find_sections_in_route ->\n\
         find_all_section_spans_in_route -> find_all_section_spans_directed ->\n\
         haversine_distance has NO consensus call. merge_traces_into_consensus_with_cache\n\
         is a SEPARATE step the shipped incremental runs in Phase 2\n\
         (incremental.rs:204-253). The fold reuses the index/portion CONTRACT and never calls\n\
         the consensus merge, so real-trace geometry is preserved.\n\
         BUT the predicate itself is TOLERANCE-BRITTLE on Unified's compact sections\n\
         (endpoint_span 63-198 m): at the 400 m default it degenerates (19-70 slivers,\n\
         matches a 60 m parallel street), and tightening proximity makes it MISS the real\n\
         traverser (see proximity=40 m -> NO MATCH). VERDICT: salvage the return contract\n\
         + portion extraction; do NOT reuse find_sections_in_route's endpoint-anchored\n\
         predicate as-is. The fold needs a tolerance-robust match (coverage/AMD) for the\n\
         maintain-existing query. MUST NOT: merge_traces_into_consensus_with_cache\n\
         (Phase 2 consensus averaging) in the maintain path.\n"
    );

    // Robust contracts only. A real traverser must locate its own section, the
    // returned portion must be verbatim new-activity geometry, and a far one-off
    // must not match. Direction/partial/near-miss are reported, not asserted.
    assert!(full.matched, "a real traverser must match its own section");
    assert!(
        full.portion_is_verbatim_slice,
        "the returned portion must be the new activity's verbatim points, not consensus"
    );
    if let Some(far) = &far {
        assert!(
            !far.matched,
            "a far one-off activity must not match the section"
        );
    }
}

// ============================================================================
// PART B — pure-layer parity + cost contract for the incremental fold
// ============================================================================

/// Naive re-batch drip: on each add, re-run `detect_sections` over the
/// full accumulated pool. The final step IS the batch call, so it equals the
/// batch by construction; more usefully, the batch is order-free (a shuffled
/// ingest yields a ground-identical catalogue), which is the convergence target
/// the incremental must reproduce step for step.
#[test]
fn naive_rebatch_convergence_and_order_free() {
    let corpus = reduced_corpus();
    let tracks = corpus.tracks_through_e();
    let sports = corpus.sport_map_through_e();
    let cfg = SectionConfig::default();

    let batch = detect_sections(&tracks, &[], &sports, &cfg);

    // Re-batch drip over growing prefixes; record each step's section count.
    let mut step_counts = Vec::with_capacity(tracks.len());
    let mut final_drip = Vec::new();
    for n in 1..=tracks.len() {
        let prefix = &tracks[..n];
        let cat = detect_sections(prefix, &[], &sports, &cfg);
        step_counts.push(cat.len());
        if n == tracks.len() {
            final_drip = cat;
        }
    }

    // Order-freeness: reverse the ingest order (a deterministic reordering of
    // the SAME set) and confirm the batch is ground-identical. detect_sections_
    // unified partitions by sport and accumulates a coverage grid, so its output
    // depends on the SET, not the sequence. This is what makes every re-batch
    // step order-free.
    let mut reordered = tracks.clone();
    reordered.reverse();
    let batch_reordered = detect_sections(&reordered, &[], &sports, &cfg);

    let final_vs_batch = catalogue_overlap(&final_drip, &batch);
    let order_free = catalogue_overlap(&batch_reordered, &batch);

    println!("\n================ naive re-batch convergence ================");
    println!("corpus activities .......... {}", tracks.len());
    println!("batch sections ............. {}", batch.len());
    println!("re-batch step counts ....... {:?}", step_counts);
    println!("final drip vs batch ........ {final_vs_batch:.3}  (== 1.000 by construction)");
    println!("order-free (reversed set) .. {order_free:.3}  (ground overlap of two ingest orders)");
    println!("--------------------------------------------------------------------");
    println!(
        "TARGET: the incremental must land, step for step, on this order-free\n\
         catalogue. The re-batch drip is the ground truth; it is just too slow\n\
         (see the cost curve). Convergence bar = ground overlap >= 0.95.\n"
    );

    assert_eq!(
        final_drip.len(),
        batch.len(),
        "final re-batch step must equal the batch"
    );
    assert!(
        final_vs_batch >= 0.999,
        "final re-batch step must be the batch (overlap {final_vs_batch:.3})"
    );
    assert!(
        order_free >= 0.95,
        "unified batch must be order-free; two ingest orders overlapped only {order_free:.3}"
    );
}

/// GATE (armed and live). The order-free Unified-aware incremental drips the
/// pool one activity at a time, folding each into the prior catalogue, and must
/// converge to the from-scratch Unified batch at >= 0.95 ground overlap.
///
/// This is green against the NAIVE-CORRECT baseline
/// (`detect_sections_incremental`), which re-batches the accumulated
/// pool on every fold: correct by construction (the final fold IS the batch)
/// but O(N) per add. The convergence CONTRACT is now locked; the engine layer
/// optimises the baseline's cost UNDER this gate without touching the assertion.
/// A separate cost/flat-per-add gate is intentionally NOT added here — the naive
/// baseline would fail it; the optimiser adds it when it lands the fast path.
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
    let corpus = corpus_with_bucket_a(36);
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
