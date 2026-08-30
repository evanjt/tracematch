//! Subset-preview fidelity probe.
//!
//! Answers one question: if a live parameter preview runs detection on
//! only the tracks near a dense centre, how far does the result drift
//! from what the full global detect would draw on that same ground?
//!
//!     cargo run --release --example preview_bbox -- citycorpus
//!
//! Prints, per radius: subset size, detect time, and the ground-match
//! rate between the subset catalogue and the full catalogue inside the
//! core (the inner half of the box, so box-edge truncation is scored
//! separately from core fidelity).

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::time::Instant;

use tracematch::{FrequentSection, GpsPoint, SectionConfig, geo_utils::haversine_distance};

struct Activity {
    id: String,
    sport: String,
    date: String,
    points: Vec<GpsPoint>,
    seconds: Vec<f64>,
}

fn point_seconds(trimmed: &str) -> Option<f64> {
    let inner = &trimmed[trimmed.find("<time>")? + 6..trimmed.find("</time>")?];
    let day: f64 = inner.get(8..10)?.parse().ok()?;
    let h: f64 = inner.get(11..13)?.parse().ok()?;
    let m: f64 = inner.get(14..16)?.parse().ok()?;
    let s: f64 = inner.get(17..19)?.parse().ok()?;
    Some((day - 1.0) * 86_400.0 + h * 3600.0 + m * 60.0 + s)
}

fn load_gpx_full(path: &Path) -> (Vec<GpsPoint>, Vec<f64>, String) {
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(_) => return (Vec::new(), Vec::new(), String::new()),
    };
    let mut date = String::new();
    let mut points: Vec<GpsPoint> = Vec::new();
    let mut times: Vec<Option<f64>> = Vec::new();
    let mut pending: Option<(f64, f64)> = None;

    for line in content.lines() {
        let trimmed = line.trim();
        if date.is_empty()
            && let Some(start) = trimmed.find("<time>")
            && let Some(end) = trimmed.find("</time>")
            && start + 6 <= end
        {
            date = trimmed[start + 6..end].to_string();
        }
        if trimmed.contains("<trkpt") {
            if let Some((lat, lon)) = pending.take() {
                points.push(GpsPoint::new(lat, lon));
                times.push(None);
            }
            if let (Some(lat_start), Some(lon_start)) =
                (trimmed.find("lat=\""), trimmed.find("lon=\""))
                && let (Some(lat_end), Some(lon_end)) = (
                    trimmed[lat_start + 5..].find('"'),
                    trimmed[lon_start + 5..].find('"'),
                )
                && let (Ok(lat), Ok(lon)) = (
                    trimmed[lat_start + 5..lat_start + 5 + lat_end].parse::<f64>(),
                    trimmed[lon_start + 5..lon_start + 5 + lon_end].parse::<f64>(),
                )
            {
                pending = Some((lat, lon));
            }
        } else if let Some((lat, lon)) = pending
            && let Some(start) = trimmed.find("<ele>")
            && let Some(end) = trimmed.find("</ele>")
            && start + 5 <= end
        {
            let ele = trimmed[start + 5..end].parse::<f64>().ok();
            points.push(match ele {
                Some(e) => GpsPoint::with_elevation(lat, lon, e),
                None => GpsPoint::new(lat, lon),
            });
            times.push(None);
            pending = None;
        } else if pending.is_none()
            && let Some(t) = times.last_mut()
            && t.is_none()
            && trimmed.starts_with("<time>")
        {
            *t = point_seconds(trimmed);
        }
    }
    if let Some((lat, lon)) = pending.take() {
        points.push(GpsPoint::new(lat, lon));
        times.push(None);
    }
    let seconds = if !times.is_empty() && times.iter().all(Option::is_some) {
        times.into_iter().flatten().collect()
    } else {
        Vec::new()
    };
    (points, seconds, date)
}

fn sport_from_name(name: &str) -> &'static str {
    let lower = name.to_lowercase();
    if lower.contains("cycl") || lower.contains("ride") || lower.contains("vélo") {
        "Ride"
    } else if lower.contains("run") || lower.contains("course") {
        "Run"
    } else if lower.contains("hik") || lower.contains("walk") || lower.contains("march") {
        "Walk"
    } else {
        "Other"
    }
}

fn load_meta(dir: &Path) -> HashMap<String, (String, String)> {
    let Ok(content) = std::fs::read_to_string(dir.join("_meta.json")) else {
        return HashMap::new();
    };
    let Ok(v) = serde_json::from_str::<serde_json::Value>(&content) else {
        return HashMap::new();
    };
    let Some(map) = v.as_object() else {
        return HashMap::new();
    };
    map.iter()
        .map(|(id, m)| {
            let bucket = match m.get("type").and_then(|t| t.as_str()).unwrap_or("") {
                "Run" | "TrailRun" | "VirtualRun" => "Run",
                "Ride" | "GravelRide" | "MountainBikeRide" | "VirtualRide" => "Ride",
                "Walk" | "Hike" | "Snowshoe" => "Walk",
                "OpenWaterSwim" | "Swim" => "Swim",
                "Snowboard" | "AlpineSki" | "NordicSki" | "BackcountrySki" => "Snow",
                _ => "Other",
            };
            let date = m
                .get("date")
                .and_then(|d| d.as_str())
                .unwrap_or("")
                .to_string();
            (id.clone(), (bucket.to_string(), date))
        })
        .collect()
}

fn load_corpus(dir: &Path) -> Vec<Activity> {
    let meta = load_meta(dir);
    let mut activities = Vec::new();
    for entry in std::fs::read_dir(dir).expect("read_dir").flatten() {
        let path = entry.path();
        if !path.extension().is_some_and(|e| e == "gpx") {
            continue;
        }
        let (points, seconds, date) = load_gpx_full(&path);
        if points.len() < 50 {
            continue;
        }
        let name = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        let aid = name.split('_').next().unwrap_or("");
        let (sport, date) = match meta.get(aid) {
            Some((bucket, meta_date)) => (
                bucket.clone(),
                if meta_date.is_empty() {
                    date
                } else {
                    meta_date.clone()
                },
            ),
            None => (sport_from_name(&name).to_string(), date),
        };
        activities.push(Activity {
            id: name,
            sport,
            date,
            points,
            seconds,
        });
    }
    activities.sort_by(|a, b| a.date.cmp(&b.date));
    activities
}

fn day_of(date: &str) -> Option<i64> {
    let y: i64 = date.get(0..4)?.parse().ok()?;
    let m: i64 = date.get(5..7)?.parse().ok()?;
    let d: i64 = date.get(8..10)?.parse().ok()?;
    let y2 = if m <= 2 { y - 1 } else { y };
    let era = if y2 >= 0 { y2 } else { y2 - 399 } / 400;
    let yoe = y2 - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    Some(era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468)
}

fn epoch_of(date: &str) -> Option<i64> {
    if date.get(0..4)?.parse::<i64>().ok()? < 2000 {
        return None;
    }
    let days = day_of(date)?;
    let tod = (|| {
        let h: i64 = date.get(11..13)?.parse().ok()?;
        let m: i64 = date.get(14..16)?.parse().ok()?;
        let s: i64 = date.get(17..19)?.parse().ok()?;
        Some(h * 3600 + m * 60 + s)
    })()
    .unwrap_or(12 * 3600);
    Some(days * 86400 + tod)
}

// ------------------------------------------------------------- geometry

/// Share of `a`'s points within `tol` metres of any point of `b`.
fn coverage(a: &[GpsPoint], b: &[GpsPoint], tol: f64) -> f64 {
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let hit = a
        .iter()
        .filter(|p| b.iter().any(|q| haversine_distance(p, q) <= tol))
        .count();
    hit as f64 / a.len() as f64
}

fn centroid(pts: &[GpsPoint]) -> (f64, f64) {
    let n = pts.len().max(1) as f64;
    (
        pts.iter().map(|p| p.latitude).sum::<f64>() / n,
        pts.iter().map(|p| p.longitude).sum::<f64>() / n,
    )
}

fn within(p: &GpsPoint, c: (f64, f64), half_lat: f64, half_lng: f64) -> bool {
    (p.latitude - c.0).abs() <= half_lat && (p.longitude - c.1).abs() <= half_lng
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let dir = PathBuf::from(args.get(1).map(|s| s.as_str()).unwrap_or("citycorpus"));
    let activities = load_corpus(&dir);
    println!("corpus {} : {} activities", dir.display(), activities.len());

    let tracks: Vec<(String, Vec<GpsPoint>)> = activities
        .iter()
        .map(|a| (a.id.clone(), a.points.clone()))
        .collect();
    let seconds: Vec<&[f64]> = activities.iter().map(|a| a.seconds.as_slice()).collect();
    let sport_types: HashMap<String, String> = activities
        .iter()
        .map(|a| (a.id.clone(), a.sport.clone()))
        .collect();
    let start_epochs: HashMap<String, i64> = activities
        .iter()
        .filter_map(|a| epoch_of(&a.date).map(|e| (a.id.clone(), e)))
        .collect();

    let config = SectionConfig::default();
    let tun = tracematch::Tunables::DEFAULT;

    let t = Instant::now();
    let full = tracematch::detect_sections_unified_dated(
        &tracks,
        &seconds,
        &sport_types,
        &start_epochs,
        &config,
        &tun,
    )
    .sections;
    let full_ms = t.elapsed().as_millis();
    println!("full detect: {} sections in {} ms", full.len(), full_ms);

    // Dense centre: 1 km cell grid over all track points, distinct tracks per cell.
    let ref_lat = tracks
        .iter()
        .flat_map(|(_, p)| p.iter())
        .map(|p| p.latitude)
        .sum::<f64>()
        / tracks.iter().map(|(_, p)| p.len()).sum::<usize>().max(1) as f64;
    let cell_lat = 1000.0 / 111_320.0;
    let cell_lng = 1000.0 / (111_320.0 * ref_lat.to_radians().cos());
    let mut cells: HashMap<(i64, i64), HashSet<usize>> = HashMap::new();
    for (i, (_, pts)) in tracks.iter().enumerate() {
        for p in pts {
            let key = (
                (p.latitude / cell_lat).floor() as i64,
                (p.longitude / cell_lng).floor() as i64,
            );
            cells.entry(key).or_default().insert(i);
        }
    }
    let mut ranked: Vec<((i64, i64), usize)> = cells.iter().map(|(k, v)| (*k, v.len())).collect();
    ranked.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    let (best_cell, best_n) = ranked[0];
    let centre = (
        (best_cell.0 as f64 + 0.5) * cell_lat,
        (best_cell.1 as f64 + 0.5) * cell_lng,
    );
    println!(
        "densest 1 km cell: {:.5},{:.5} with {} distinct tracks (top 5: {:?})",
        centre.0,
        centre.1,
        best_n,
        ranked.iter().take(5).map(|(_, n)| *n).collect::<Vec<_>>()
    );

    // Warm-cache re-preview: how cheap is a parameter change that misses only
    // the consensus memo (min_activities) versus one that misses the portion
    // memo too (min_section_length) versus the grid itself (proximity)?
    {
        let ids: Vec<&str> = tracks.iter().map(|(i, _)| i.as_str()).collect();
        let mut cache = tracematch::SectionEvidenceCache::new();
        let t = Instant::now();
        let base = tracematch::detect_sections_unified_incremental_dated(
            &mut cache,
            &[],
            &tracks,
            &ids,
            &seconds,
            &sport_types,
            &start_epochs,
            &config,
            &tracematch::SectionUpdatePolicy::default(),
        );
        println!(
            "\nwarm cache: cold fold {} sections in {} ms",
            base.catalogue.len(),
            t.elapsed().as_millis()
        );
        for (label, cfg) in [
            ("same config", config.clone()),
            (
                "min_activities 2->3",
                SectionConfig {
                    min_activities: 3,
                    ..config.clone()
                },
            ),
            (
                "min_section_length 150->300",
                SectionConfig {
                    min_section_length: 300.0,
                    ..config.clone()
                },
            ),
            (
                "proximity 200->150",
                SectionConfig {
                    proximity_threshold: 150.0,
                    ..config.clone()
                },
            ),
        ] {
            let mut c = cache.clone();
            let t = Instant::now();
            let r = tracematch::detect_sections_unified_incremental_dated(
                &mut c,
                &[],
                &tracks,
                &[],
                &seconds,
                &sport_types,
                &start_epochs,
                &cfg,
                &tracematch::SectionUpdatePolicy::default(),
            );
            println!(
                "  {:<28} {:>6} ms -> {} sections",
                label,
                t.elapsed().as_millis(),
                r.catalogue.len()
            );
        }
    }

    // Cluster-honest subset: replicate the detector's own per-sport geographic
    // clustering (padded bbox chaining at cluster_gap_m) and keep every member
    // of every component that touches the centre. If the clusters really are
    // independent this must reproduce the full catalogue over that ground.
    {
        let gap_m = tun.cluster_gap_m;
        let pad_lat = gap_m * 0.5 / 111_132.0;
        let bbox = |pts: &[GpsPoint]| {
            let mut bb = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for p in pts {
                bb.0 = bb.0.min(p.latitude);
                bb.1 = bb.1.max(p.latitude);
                bb.2 = bb.2.min(p.longitude);
                bb.3 = bb.3.max(p.longitude);
            }
            let mid = ((bb.0 + bb.1) * 0.5).to_radians();
            let pad_lng = gap_m * 0.5 / (111_320.0 * mid.cos().abs().max(0.01));
            (
                bb.0 - pad_lat,
                bb.1 + pad_lat,
                bb.2 - pad_lng,
                bb.3 + pad_lng,
            )
        };
        let mut keep: Vec<usize> = Vec::new();
        let sports: HashSet<&String> = sport_types.values().collect();
        for sp in sports {
            let members: Vec<usize> = (0..tracks.len())
                .filter(|&i| &sport_types[&tracks[i].0] == sp && !tracks[i].1.is_empty())
                .collect();
            let boxes: Vec<_> = members.iter().map(|&i| bbox(&tracks[i].1)).collect();
            let mut comp: Vec<usize> = (0..members.len()).collect();
            let mut changed = true;
            while changed {
                changed = false;
                for a in 0..members.len() {
                    for b in (a + 1)..members.len() {
                        let (x, y) = (boxes[a], boxes[b]);
                        if x.0 <= y.1 && y.0 <= x.1 && x.2 <= y.3 && y.2 <= x.3 {
                            let (ca, cb) = (comp[a].min(comp[b]), comp[a].max(comp[b]));
                            if ca != cb {
                                for c in comp.iter_mut() {
                                    if *c == cb {
                                        *c = ca;
                                    }
                                }
                                changed = true;
                            }
                        }
                    }
                }
            }
            let touching: HashSet<usize> = (0..members.len())
                .filter(|&k| {
                    let b = boxes[k];
                    b.0 <= centre.0 && centre.0 <= b.1 && b.2 <= centre.1 && centre.1 <= b.3
                })
                .map(|k| comp[k])
                .collect();
            for k in 0..members.len() {
                if touching.contains(&comp[k]) {
                    keep.push(members[k]);
                }
            }
        }
        keep.sort_unstable();
        let kt: Vec<(String, Vec<GpsPoint>)> = keep.iter().map(|&i| tracks[i].clone()).collect();
        let ks: Vec<&[f64]> = keep.iter().map(|&i| seconds[i]).collect();
        let t = Instant::now();
        let cs = tracematch::detect_sections_unified_dated(
            &kt,
            &ks,
            &sport_types,
            &start_epochs,
            &config,
            &tun,
        )
        .sections;
        let ms = t.elapsed().as_millis();
        // Compare over the whole region the kept clusters cover.
        let mut agree = 0;
        let mut region = 0;
        let kept_ids: HashSet<&str> = kt.iter().map(|(i, _)| i.as_str()).collect();
        for f in &full {
            // Only sections whose evidence is entirely inside the kept clusters
            // can be reproduced at all; anything else is a different cluster.
            if !f.activity_ids.iter().all(|a| kept_ids.contains(a.as_str())) {
                continue;
            }
            region += 1;
            if cs.iter().any(|s| {
                s.sport_type == f.sport_type
                    && coverage(&f.polyline, &s.polyline, 50.0).min(coverage(
                        &s.polyline,
                        &f.polyline,
                        50.0,
                    )) >= 0.8
            }) {
                agree += 1;
            }
        }
        println!(
            "\ncluster-honest subset: {} of {} tracks, {} ms, {} sections; region sections {} agree {}",
            kt.len(),
            tracks.len(),
            ms,
            cs.len(),
            region,
            agree
        );
    }

    println!(
        "\n{:>8} {:>7} {:>8} {:>7} {:>7} {:>7} {:>8} {:>8} {:>8}",
        "radius", "tracks", "detect", "fullC", "subC", "matched", "medCov", "lenRatio", "novel"
    );

    for radius_km in [2.0f64, 5.0, 10.0, 20.0, 40.0, 100.0, 20000.0] {
        let half_lat = radius_km * 1000.0 / 111_320.0;
        let half_lng = radius_km * 1000.0 / (111_320.0 * ref_lat.to_radians().cos());
        // Core = inner half of the box. Sections fully inside the core are the
        // ones a preview would legitimately claim to show.
        let core_lat = half_lat * 0.5;
        let core_lng = half_lng * 0.5;

        let idx: Vec<usize> = (0..tracks.len())
            .filter(|&i| {
                tracks[i]
                    .1
                    .iter()
                    .any(|p| within(p, centre, half_lat, half_lng))
            })
            .collect();
        if idx.is_empty() {
            continue;
        }
        let sub_tracks: Vec<(String, Vec<GpsPoint>)> =
            idx.iter().map(|&i| tracks[i].clone()).collect();
        let sub_seconds: Vec<&[f64]> = idx.iter().map(|&i| seconds[i]).collect();

        let t = Instant::now();
        let sub = tracematch::detect_sections_unified_dated(
            &sub_tracks,
            &sub_seconds,
            &sport_types,
            &start_epochs,
            &config,
            &tun,
        )
        .sections;
        let ms = t.elapsed().as_millis();

        let in_core = |s: &FrequentSection| {
            !s.polyline.is_empty()
                && s.polyline
                    .iter()
                    .all(|p| within(p, centre, core_lat, core_lng))
        };
        let full_core: Vec<&FrequentSection> = full.iter().filter(|s| in_core(s)).collect();
        let sub_core: Vec<&FrequentSection> = sub.iter().filter(|s| in_core(s)).collect();

        let mut covs: Vec<f64> = Vec::new();
        let mut ratios: Vec<f64> = Vec::new();
        let mut matched = 0usize;
        let mut claimed: HashSet<usize> = HashSet::new();
        for f in &full_core {
            let mut best = (0.0f64, usize::MAX);
            for (j, s) in sub_core.iter().enumerate() {
                if s.sport_type != f.sport_type {
                    continue;
                }
                let c = coverage(&f.polyline, &s.polyline, 50.0).min(coverage(
                    &s.polyline,
                    &f.polyline,
                    50.0,
                ));
                if c > best.0 {
                    best = (c, j);
                }
            }
            covs.push(best.0);
            if best.0 >= 0.8 {
                matched += 1;
                claimed.insert(best.1);
                ratios.push(sub_core[best.1].distance_meters / f.distance_meters.max(1.0));
            }
        }
        covs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med_cov = covs.get(covs.len() / 2).copied().unwrap_or(0.0);
        let med_ratio = {
            ratios.sort_by(|a, b| a.partial_cmp(b).unwrap());
            ratios.get(ratios.len() / 2).copied().unwrap_or(0.0)
        };
        let novel = sub_core.len().saturating_sub(claimed.len());
        let bands = |lo: f64, hi: f64| covs.iter().filter(|c| **c >= lo && **c < hi).count();
        println!(
            "   coverage bands of full-core sections: [0,.2)={} [.2,.5)={} [.5,.8)={} [.8,1]={}",
            bands(0.0, 0.2),
            bands(0.2, 0.5),
            bands(0.5, 0.8),
            covs.iter().filter(|c| **c >= 0.8).count()
        );

        println!(
            "{:>6.0}km {:>7} {:>7}ms {:>7} {:>7} {:>7} {:>8.2} {:>8.2} {:>8}",
            radius_km,
            sub_tracks.len(),
            ms,
            full_core.len(),
            sub_core.len(),
            matched,
            med_cov,
            med_ratio,
            novel
        );

        // Clipped variant: keep only the contiguous point runs inside the box,
        // each run a pseudo-track. This is the cheap preview anyone would reach
        // for, so measure both its speed and its honesty.
        if radius_km <= 10.0 {
            let mut clip_tracks: Vec<(String, Vec<GpsPoint>)> = Vec::new();
            let mut clip_sports: HashMap<String, String> = HashMap::new();
            let mut clip_epochs: HashMap<String, i64> = HashMap::new();
            for &i in &idx {
                let (id, pts) = &tracks[i];
                let mut run: Vec<GpsPoint> = Vec::new();
                let mut n = 0;
                let mut flush =
                    |run: &mut Vec<GpsPoint>,
                     n: &mut i32,
                     out: &mut Vec<(String, Vec<GpsPoint>)>| {
                        if run.len() >= 20 {
                            let nid = format!("{}#{}", id, n);
                            clip_sports.insert(nid.clone(), sport_types[id].clone());
                            if let Some(e) = start_epochs.get(id) {
                                clip_epochs.insert(nid.clone(), *e);
                            }
                            out.push((nid, std::mem::take(run)));
                            *n += 1;
                        } else {
                            run.clear();
                        }
                    };
                for p in pts {
                    if within(p, centre, half_lat, half_lng) {
                        run.push(*p);
                    } else {
                        flush(&mut run, &mut n, &mut clip_tracks);
                    }
                }
                flush(&mut run, &mut n, &mut clip_tracks);
            }
            let clip_pts: usize = clip_tracks.iter().map(|(_, p)| p.len()).sum();
            let sub_pts: usize = sub_tracks.iter().map(|(_, p)| p.len()).sum();
            let empty: Vec<&[f64]> = vec![&[]; clip_tracks.len()];
            let t = Instant::now();
            let clipped = tracematch::detect_sections_unified_dated(
                &clip_tracks,
                &empty,
                &clip_sports,
                &clip_epochs,
                &config,
                &tun,
            )
            .sections;
            let cms = t.elapsed().as_millis();
            let clip_core: Vec<&FrequentSection> = clipped.iter().filter(|s| in_core(s)).collect();
            let mut agree = 0;
            for f in &full_core {
                if clip_core.iter().any(|p| {
                    p.sport_type == f.sport_type
                        && coverage(&f.polyline, &p.polyline, 50.0).min(coverage(
                            &p.polyline,
                            &f.polyline,
                            50.0,
                        )) >= 0.8
                }) {
                    agree += 1;
                }
            }
            println!(
                "   clipped: {} runs / {} pts (vs {} tracks / {} pts), {} ms, core {} vs full-core {}, agree {}",
                clip_tracks.len(),
                clip_pts,
                sub_tracks.len(),
                sub_pts,
                cms,
                clip_core.len(),
                full_core.len(),
                agree
            );
        }

        // Isolation probe: does adding ONE far-away track, which touches no
        // ground in the box, change the catalogue inside the core? If yes the
        // drift is cluster/projection level, not local traffic.
        if (radius_km - 100.0).abs() < 0.5 {
            let inside: HashSet<usize> = idx.iter().copied().collect();
            let far = (0..tracks.len())
                .filter(|i| !inside.contains(i))
                .max_by(|a, b| {
                    let d = |i: &usize| {
                        let (lat, lng) = centroid(&tracks[*i].1);
                        (lat - centre.0).abs() + (lng - centre.1).abs()
                    };
                    d(a).partial_cmp(&d(b)).unwrap()
                });
            if let Some(f) = far {
                let mut plus_tracks = sub_tracks.clone();
                let mut plus_seconds = sub_seconds.clone();
                plus_tracks.push(tracks[f].clone());
                plus_seconds.push(seconds[f]);
                let plus = tracematch::detect_sections_unified_dated(
                    &plus_tracks,
                    &plus_seconds,
                    &sport_types,
                    &start_epochs,
                    &config,
                    &tun,
                )
                .sections;
                let plus_core: Vec<&FrequentSection> = plus.iter().filter(|s| in_core(s)).collect();
                let mut agree = 0;
                for s in &sub_core {
                    if plus_core.iter().any(|p| {
                        p.sport_type == s.sport_type
                            && coverage(&s.polyline, &p.polyline, 50.0).min(coverage(
                                &p.polyline,
                                &s.polyline,
                                50.0,
                            )) >= 0.8
                    }) {
                        agree += 1;
                    }
                }
                let (flat, flng) = centroid(&tracks[f].1);
                println!(
                    "   isolation probe: +1 track at {:.3},{:.3} ({:.0} km away) -> core {} vs {}, agree {}",
                    flat,
                    flng,
                    haversine_distance(
                        &GpsPoint::new(centre.0, centre.1),
                        &GpsPoint::new(flat, flng)
                    ) / 1000.0,
                    sub_core.len(),
                    plus_core.len(),
                    agree
                );
            }
        }
    }
}
