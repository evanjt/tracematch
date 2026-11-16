//! The section-dump row and the pass-probe loop shared by the private and
//! public corpus measurements (`section_dump*.rs`, `section_passes*.rs`), so
//! the two corpora write one format and `section_spatial.py` reads either.

// Each test target compiles this module separately and uses a subset of it.
#![allow(dead_code)]

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io::{BufRead, Write};
use std::path::Path;
use std::process::Command;
use tracematch::sections::{BoundaryRecord, FrequentSection, PreparedLine};
use tracematch::{GpsPoint, SectionConfig};

/// The first line of a dump: the fold the rows were written from. `digest` is
/// the catalogue digest the gate prints as `A`, `tree` the commit the crate
/// was checked out at (with `+` when its tree is dirty), `corpus` the data
/// directory the tracks were read from.
#[derive(Serialize)]
pub struct Header {
    pub digest: String,
    pub tree: String,
    pub corpus: String,
}

/// The tree a dump records: the commit, with `+` appended when the tree is dirty.
pub fn tree_label(head: &str, dirty: bool) -> String {
    if dirty {
        format!("{head}+")
    } else {
        head.to_string()
    }
}

fn git(dir: &str, args: &[&str]) -> String {
    let out = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(args)
        .output()
        .expect("git runs to label the dump's tree");
    assert!(
        out.status.success(),
        "git {args:?} failed in {dir}, so the dump cannot record its tree"
    );
    String::from_utf8(out.stdout).expect("git prints utf-8")
}

fn crate_tree() -> String {
    let dir = env!("CARGO_MANIFEST_DIR");
    let head = git(dir, &["rev-parse", "HEAD"]);
    let dirty = !git(dir, &["status", "--porcelain"]).trim().is_empty();
    tree_label(head.trim(), dirty)
}

#[derive(Serialize)]
struct Row {
    id: String,
    sport: String,
    line_hash: String,
    line_points: usize,
    first: [f64; 2],
    last: [f64; 2],
    distance_m: f64,
    rep: String,
    rep_range: Option<(u32, u32)>,
    rep_reversed: bool,
    members: Vec<String>,
    portions: Vec<(String, u32, u32, String)>,
    visit_count: u32,
    trace_ids: Vec<String>,
    confidence: f64,
    observation_count: u32,
    polyline: Vec<[f64; 2]>,
    portion_m: Vec<f64>,
}

/// A header line, then one JSON line per section into `out`, and the boundary
/// records beside it as `<out>.boundaries`. `digest` is the fold's
/// `catalogue_digest`, which the caller computes so this module does not
/// depend on the bitwise gate's module.
pub fn write_dump(
    catalogue: &[FrequentSection],
    boundaries: &[BoundaryRecord],
    out: &str,
    corpus: &str,
    digest: u64,
) {
    let mut bf = std::fs::File::create(format!("{out}.boundaries")).unwrap();
    for b in boundaries {
        writeln!(
            bf,
            "{}",
            serde_json::to_string(&(
                &b.section_ids,
                format!("{:?}", b.reason),
                b.latitude,
                b.longitude
            ))
            .unwrap()
        )
        .unwrap();
    }
    let mut f = std::fs::File::create(out).unwrap();
    let header = Header {
        digest: format!("{digest:016x}"),
        tree: crate_tree(),
        corpus: corpus.to_string(),
    };
    let header = serde_json::to_string(&header).unwrap();
    eprintln!("{header}");
    writeln!(f, "{header}").unwrap();
    for s in catalogue {
        let mut h: u64 = 0xcbf29ce484222325;
        for p in &s.polyline {
            for b in p
                .latitude
                .to_bits()
                .to_le_bytes()
                .iter()
                .chain(p.longitude.to_bits().to_le_bytes().iter())
            {
                h ^= *b as u64;
                h = h.wrapping_mul(0x100000001b3);
            }
        }
        let mut members = s.activity_ids.clone();
        members.sort();
        let mut portions: Vec<(String, u32, u32, String)> = s
            .activity_portions
            .iter()
            .map(|p| {
                (
                    p.activity_id.clone(),
                    p.start_index,
                    p.end_index,
                    format!("{:?}", p.direction),
                )
            })
            .collect();
        portions.sort();
        let mut trace_ids: Vec<String> = s.activity_traces.keys().cloned().collect();
        trace_ids.sort();
        let fp = s.polyline.first().unwrap();
        let lp = s.polyline.last().unwrap();
        let row = Row {
            id: s.id.clone(),
            sport: s.sport_type.clone(),
            line_hash: format!("{h:016x}"),
            line_points: s.polyline.len(),
            first: [fp.latitude, fp.longitude],
            last: [lp.latitude, lp.longitude],
            distance_m: s.distance_meters,
            rep: s.representative_activity_id.clone(),
            rep_range: s.representative_range,
            rep_reversed: s.representative_reversed,
            members,
            portions,
            visit_count: s.visit_count,
            trace_ids,
            confidence: s.confidence,
            observation_count: s.observation_count,
            polyline: s
                .polyline
                .iter()
                .map(|p| [p.latitude, p.longitude])
                .collect(),
            portion_m: s
                .activity_portions
                .iter()
                .map(|p| p.distance_meters)
                .collect(),
        };
        writeln!(f, "{}", serde_json::to_string(&row).unwrap()).unwrap();
    }
}

/// The folded tracks as a GPX directory `section_spatial.py` can read, one
/// file per activity named `<id>__<name>.gpx`. The double underscore keeps an
/// id that itself holds an underscore whole.
pub fn write_gpx_corpus(tracks: &[(String, Vec<GpsPoint>)], dir: &Path) {
    std::fs::create_dir_all(dir).unwrap();
    for (id, pts) in tracks {
        let mut f = std::fs::File::create(dir.join(format!("{id}__track.gpx"))).unwrap();
        writeln!(
            f,
            "<?xml version=\"1.0\"?><gpx version=\"1.1\"><trk><trkseg>"
        )
        .unwrap();
        for p in pts {
            writeln!(
                f,
                "<trkpt lat=\"{}\" lon=\"{}\"></trkpt>",
                p.latitude, p.longitude
            )
            .unwrap();
        }
        writeln!(f, "</trkseg></trk></gpx>").unwrap();
    }
}

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

/// Answer each request line in `input` with the detector's passes over the
/// named line, into `output`. The format is `section_passes.rs`'s.
pub fn answer_passes(by_id: &HashMap<&str, &[GpsPoint]>, input: &str, output: &str) -> usize {
    let config = SectionConfig::default();
    let reader = std::io::BufReader::new(std::fs::File::open(input).unwrap());
    let mut out = std::fs::File::create(output).unwrap();
    let mut prepared: HashMap<(String, u64), (Vec<GpsPoint>, f64)> = HashMap::new();
    let mut n = 0usize;
    for line in reader.lines() {
        let line = line.unwrap();
        if line.trim().is_empty() {
            continue;
        }
        let req: Request = serde_json::from_str(&line).unwrap();
        let empty = |req: Request| Answer {
            key: req.key,
            activity: req.activity,
            passes: vec![],
        };
        let Some(track) = by_id.get(req.activity.as_str()) else {
            writeln!(out, "{}", serde_json::to_string(&empty(req)).unwrap()).unwrap();
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
        let Some(line) = PreparedLine::with_lateral_tolerance(pts, &config, *lateral) else {
            writeln!(out, "{}", serde_json::to_string(&empty(req)).unwrap()).unwrap();
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
    n
}
