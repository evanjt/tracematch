//! Answer, for named (line, activity) pairs, the passes the detector counts
//! over the private corpus, so a section-dump move can be explained with the
//! detector's own judgement rather than a re-implementation. A scratch
//! measurement beside `section_dump.rs`, not a gate.
//!
//! Each request line is JSON: `{"key": "...", "activity": "<id>", "line":
//! [[lat, lon], ...], "lateral_m": 0.0, "slice": [lo, hi] | null}`. The
//! track is cut to `slice` when given, and indices in the answer are into the
//! full track either way. Each answer line is `{"key", "activity", "passes":
//! [[start, end, "Same" | "Reverse", metres], ...]}`.
//!
//! Run: `SECTION_PASSES_IN=/tmp/req.jsonl SECTION_PASSES_OUT=/tmp/ans.jsonl \
//!       cargo test --release --features real-corpus --test section_passes -- --nocapture`
#![cfg(feature = "real-corpus")]

mod corpus;

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io::{BufRead, Write};
use tracematch::{GpsPoint, SectionConfig};

#[derive(Deserialize)]
struct Request {
    key: String,
    activity: String,
    line: Vec<[f64; 2]>,
    #[serde(default)]
    lateral_m: f64,
    #[serde(default)]
    slice: Option<(usize, usize)>,
}

#[derive(Serialize)]
struct Answer {
    key: String,
    activity: String,
    passes: Vec<(u32, u32, String, f64)>,
}

#[test]
fn answer_passes() {
    let input =
        std::env::var("SECTION_PASSES_IN").expect("SECTION_PASSES_IN names the request file");
    let output =
        std::env::var("SECTION_PASSES_OUT").expect("SECTION_PASSES_OUT names the answer file");
    let (tracks, _, _) = corpus::load_tracks("fullcorpus", 1000);
    let by_id: HashMap<&str, &[GpsPoint]> = tracks
        .iter()
        .map(|(id, t)| (id.as_str(), t.as_slice()))
        .collect();
    let config = SectionConfig::default();
    let reader = std::io::BufReader::new(std::fs::File::open(&input).unwrap());
    let mut out = std::fs::File::create(&output).unwrap();
    let mut prepared: HashMap<(String, u64), (Vec<GpsPoint>, f64)> = HashMap::new();
    let mut n = 0usize;
    for line in reader.lines() {
        let line = line.unwrap();
        if line.trim().is_empty() {
            continue;
        }
        let req: Request = serde_json::from_str(&line).unwrap();
        let Some(track) = by_id.get(req.activity.as_str()) else {
            writeln!(
                out,
                "{}",
                serde_json::to_string(&Answer {
                    key: req.key,
                    activity: req.activity,
                    passes: vec![]
                })
                .unwrap()
            )
            .unwrap();
            continue;
        };
        let (pts, lateral) = prepared
            .entry((req.key.clone(), req.lateral_m.to_bits()))
            .or_insert_with(|| {
                (
                    req.line.iter().map(|p| GpsPoint::new(p[0], p[1])).collect(),
                    req.lateral_m,
                )
            });
        let Some(line) =
            tracematch::sections::PreparedLine::with_lateral_tolerance(pts, &config, *lateral)
        else {
            writeln!(
                out,
                "{}",
                serde_json::to_string(&Answer {
                    key: req.key,
                    activity: req.activity,
                    passes: vec![]
                })
                .unwrap()
            )
            .unwrap();
            continue;
        };
        let (offset, view): (usize, &[GpsPoint]) = match req.slice {
            Some((lo, hi)) => {
                let lo = lo.min(track.len());
                let hi = hi.min(track.len()).max(lo);
                (lo, &track[lo..hi])
            }
            None => (0, track),
        };
        let passes = line
            .portions(&req.activity, view)
            .into_iter()
            .map(|p| {
                (
                    p.start_index + offset as u32,
                    p.end_index + offset as u32,
                    format!("{:?}", p.direction),
                    p.distance_meters,
                )
            })
            .collect();
        writeln!(
            out,
            "{}",
            serde_json::to_string(&Answer {
                key: req.key,
                activity: req.activity,
                passes
            })
            .unwrap()
        )
        .unwrap();
        n += 1;
    }
    println!("answered {n} requests into {output}");
}
