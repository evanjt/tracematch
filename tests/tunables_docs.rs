//! Every free constant in `Tunables` is documented with the evidence its
//! default rests on: a plateau, a measured band on both corpora, or a
//! calibration with the failure on each side. A field added without that
//! is a number nobody can revisit.

const SOURCE: &str = include_str!("../src/sections/unified.rs");

const EVIDENCE: [&str; 6] = [
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
