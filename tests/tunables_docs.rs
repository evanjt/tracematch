//! Every free constant in `Tunables` is documented with the evidence its
//! default rests on: a plateau, a measured band on both corpora, or a
//! calibration with the failure on each side. A field added without that
//! is a number nobody can revisit.

const SOURCE: &str = include_str!("../src/sections/unified.rs");

const EVIDENCE: [&str; 8] = [
    "derived",
    "Derived",
    "plateau",
    "Plateau",
    "both corpora",
    "Calibrated",
    "measured",
    "Derivation-anchored",
];

#[test]
fn every_tunable_cites_its_evidence() {
    let start = SOURCE
        .find("pub struct Tunables {")
        .expect("Tunables struct");
    let body = &SOURCE[start..];
    let end = body.find("\n}\n").expect("struct end");
    let body = &body[..end];
    let mut doc = String::new();
    let mut fields = 0;
    let mut bare: Vec<String> = Vec::new();
    for line in body.lines().skip(1) {
        let t = line.trim();
        if let Some(d) = t.strip_prefix("///") {
            doc.push_str(d);
            doc.push(' ');
        } else if t.starts_with("pub ") {
            fields += 1;
            let flat = doc.split_whitespace().collect::<Vec<_>>().join(" ");
            if !EVIDENCE.iter().any(|w| flat.contains(w)) {
                bare.push(t.trim_end_matches(',').to_string());
            }
            doc.clear();
        }
    }
    assert!(fields >= 10, "the struct parsed into {fields} fields");
    assert!(
        bare.is_empty(),
        "tunables documented without evidence: {bare:?}"
    );
}

/// Numeric constants that are storage layout or memory bounds, not detection
/// tunables, so there is no measured plateau to cite.
const NOT_TUNABLES: [&str; 8] = [
    "EVIDENCE_CACHE_VERSION",
    "MEMO_CELL_SETS_CAP",
    "MEMO_TRACK_PORTIONS_CAP",
    "MEMO_CONSENSUS_CAP",
    "MEMO_RENDER_CAP",
    "MEMO_DRAWN_CAP",
    "MEMO_COVERAGE_CAP",
    "MEMO_UNIFY_CAP",
];

#[test]
fn every_free_numeric_const_cites_its_derivation() {
    let end = SOURCE.find("#[cfg(test)]").expect("test module");
    let lines: Vec<&str> = SOURCE[..end].lines().collect();
    let mut seen = 0;
    let mut bare: Vec<String> = Vec::new();
    for (i, line) in lines.iter().enumerate() {
        let t = line.trim();
        let Some(rest) = t
            .strip_prefix("pub const ")
            .or_else(|| t.strip_prefix("const "))
        else {
            continue;
        };
        let Some((name, tail)) = rest.split_once(':') else {
            continue;
        };
        let Some((ty, value)) = tail.split_once('=') else {
            continue;
        };
        let numeric = matches!(ty.trim(), "f64" | "usize" | "u32" | "u8")
            && value.trim().starts_with(|c: char| c.is_ascii_digit());
        if !numeric || NOT_TUNABLES.contains(&name) {
            continue;
        }
        seen += 1;
        // Adjacent constants share the block above the first.
        let mut doc = String::new();
        for above in lines[..i].iter().rev() {
            let a = above.trim();
            if a.starts_with("//") {
                doc.push_str(a);
                doc.push(' ');
            } else if a.starts_with("#[") || a.contains("const ") {
                continue;
            } else {
                break;
            }
        }
        if !EVIDENCE.iter().any(|w| doc.contains(w)) {
            bare.push(name.to_string());
        }
    }
    assert!(seen >= 10, "the scan found {seen} constants");
    assert!(bare.is_empty(), "constants without a derivation: {bare:?}");
}
