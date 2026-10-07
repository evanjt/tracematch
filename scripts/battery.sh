#!/usr/bin/env bash
# The tally counts failures only. There is no pass count anywhere in this
# script, because a hand-summed pass total is what mistallied the E tranche.
set -uo pipefail

TM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$(cd "$TM_DIR/.." && pwd)"
if grep -q '^\[workspace\]' "$WS_DIR/Cargo.toml" 2>/dev/null; then
    RUN_DIR="$WS_DIR"
else
    RUN_DIR="$TM_DIR"
fi
cd "$RUN_DIR" || exit 1

failed=0

run() {
    local label="$1"
    shift
    printf '\n=== %s\n' "$label"
    local output
    output="$("$@" --no-fail-fast 2>&1)"
    local status=$?
    printf '%s\n' "$output"
    local before=$failed
    local counted
    counted="$(printf '%s\n' "$output" | awk '/^test result:/ { for (i = 1; i < NF; i++) if ($(i + 1) == "failed;" || $(i + 1) == "failed") sum += $i } END { print sum + 0 }')"
    failed=$((failed + counted))
    # A binary killed by a signal prints no result line, so the tally alone misses it.
    local signalled
    signalled="$(printf '%s\n' "$output" | grep -c "process didn't exit successfully.*(signal:")"
    failed=$((failed + signalled))
    if [ "$status" -ne 0 ] && [ "$failed" -eq "$before" ]; then
        failed=$((failed + 1))
    fi
}

if [ "${BASH_SOURCE[0]}" != "$0" ]; then
    return 0
fi

run "workspace" cargo test --workspace
run "veloqrs synthetic" cargo test -p veloqrs --features synthetic
run "tracematch" cargo test -p tracematch

run "tracematch examples" cargo test -p tracematch --examples

if [ -n "${LAB_CORPUS_DIR:-}" ]; then
    printf '\n=== lab\n'
    LAB_OUT_DIR="${LAB_OUT_DIR:-$RUN_DIR/target/battery/lab}"
    mkdir -p "$LAB_OUT_DIR"
    if ! cargo run -p tracematch --release --example unified_lab -- \
        "$LAB_CORPUS_DIR" --out "$LAB_OUT_DIR"; then
        failed=$((failed + 1))
    fi
else
    printf '\n=== lab skipped, LAB_CORPUS_DIR unset\n'
fi

printf '\nfailed=%d\n' "$failed"
[ "$failed" -eq 0 ] || exit 1
