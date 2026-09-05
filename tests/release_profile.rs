//! The crate carries its own release profile.
//!
//! Cargo honours `[profile.release]` only at a workspace root. `B154` put one
//! at veloq's root after the defaults cost ~200 ms on the warm-add median, but
//! every release build outside that workspace, the standalone checkout, this
//! crate's own CI and the wasm build, still took cargo's defaults. The corpus
//! gates are the ones that suffer: a baseline recorded under one profile and
//! judged under another reports the build, not the detector.
//!
//! A crate manifest is not ignored when the crate is the root, so declaring it
//! here pins the standalone build and the workspace root merely agrees. If this
//! crate is ever moved under a workspace that declares its own profile, cargo
//! ignores this one silently, which is what this test is for.

const MANIFEST: &str = include_str!("../Cargo.toml");

/// The section body of `[profile.release]`, up to the next section header.
fn release_profile() -> &'static str {
    let start = MANIFEST
        .find("[profile.release]")
        .expect("Cargo.toml declares [profile.release]");
    let body = &MANIFEST[start + "[profile.release]".len()..];
    match body.find("\n[") {
        Some(end) => &body[..end],
        None => body,
    }
}

fn setting(key: &str) -> String {
    release_profile()
        .lines()
        .map(str::trim)
        .find(|line| line.starts_with(key))
        .unwrap_or_else(|| panic!("[profile.release] sets {key}"))
        .split_once('=')
        .expect("a key = value line")
        .1
        .trim()
        .to_string()
}

#[test]
fn the_release_profile_pins_one_codegen_unit() {
    assert_eq!(
        setting("codegen-units"),
        "1",
        "at the default 16 the warm-add median moves ~200 ms on partitioning luck alone"
    );
}

#[test]
fn the_release_profile_asks_for_link_time_optimisation() {
    assert_eq!(setting("lto"), "true");
}

/// `opt-level = "s"` cost 2,297 ms against 700 ms at level 3 on the same
/// median, so the size it buys is not affordable on the per-activity path.
/// Cargo's own default for release is 3, so saying nothing is correct here,
/// and what must never appear is a smaller one.
#[test]
fn the_release_profile_never_optimises_for_size() {
    let opt = release_profile()
        .lines()
        .map(str::trim)
        .find(|line| line.starts_with("opt-level"));
    if let Some(line) = opt {
        let value = line.split_once('=').expect("a key = value line").1.trim();
        assert!(
            value == "3" || value == "\"3\"",
            "opt-level must stay at 3, found {value}"
        );
    }
}

/// The reason has to travel with the setting. A profile with no explanation is
/// the state `B154` was reverted out of once already.
#[test]
fn the_release_profile_says_why_it_is_here() {
    let start = MANIFEST
        .find("[profile.release]")
        .expect("Cargo.toml declares [profile.release]");
    let preamble = &MANIFEST[..start];
    let comment: String = preamble
        .lines()
        .rev()
        .take_while(|line| line.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join(" ");
    assert!(
        comment.contains("codegen-units"),
        "the comment above [profile.release] must name what it pins and why"
    );
}
