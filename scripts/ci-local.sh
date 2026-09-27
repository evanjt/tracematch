#!/usr/bin/env bash
# Run CI's test job locally, static lanes first, so a lockfile, fmt or clippy
# failure costs seconds rather than a push and an eight-minute wait. Stops at
# the first failure and names it.
#
# Usage: scripts/ci-local.sh

set -uo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

# Cargo's default is one job per logical CPU, which starves every other
# session sharing the machine.
export CARGO_BUILD_JOBS=8
export CARGO_TERM_COLOR=always

step() {
    local name="$1"
    shift
    echo "==> $name"
    local started=$SECONDS
    if ! "$@"; then
        echo
        echo "ci-local: FAILED at '$name' after $((SECONDS - started)) s"
        exit 1
    fi
    echo "    $name: ok in $((SECONDS - started)) s"
}

# The release packages what is committed, and a working tree carries ignored
# files cargo would package or refuse. Export the tracked and untracked
# non-ignored files, which is what a checkout of the next commit holds.
EXPORT_DIR=""
trap '[ -n "$EXPORT_DIR" ] && rm -rf "$EXPORT_DIR"' EXIT

publish_dry_run() {
    EXPORT_DIR="$(mktemp -d)"
    local export_dir="$EXPORT_DIR"
    git ls-files -z --cached --others --exclude-standard \
        | while IFS= read -r -d '' f; do
            [ -e "$f" ] && printf '%s\0' "$f"
        done \
        | xargs -0 cp --parents -t "$export_dir" || return 1
    # Keep the build warm between runs. It lives outside the export, which is
    # deleted on exit.
    (cd "$export_dir" \
        && CARGO_TARGET_DIR="$ROOT/target/ci-local-publish" \
            cargo publish --dry-run --locked)
}

started=$SECONDS

step "Fetch (locked)" cargo fetch --locked
step "Check formatting" cargo fmt --check
step "Clippy" cargo clippy --all-targets --all-features -- -D warnings
step "Clippy (detection ordering)" cargo clippy -- -D clippy::iter_over_hash_type
step "Check (no default features)" cargo check -p tracematch --no-default-features
step "Docs" env RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --features synthetic
step "Publish (dry run)" publish_dry_run
step "Test (synthetic)" cargo test --features synthetic

echo
echo "ci-local: all lanes passed in $((SECONDS - started)) s"
