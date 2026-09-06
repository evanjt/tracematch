#!/usr/bin/env bash
# The corpus run as one command.
#
# Runs the validation lab over the private corpus and GeoLife, writes one
# table per run with a row per section, runs both bitwise gates read-only
# and prints what diverged, and rebases a golden only when told to, with a
# dated line in the golden saying which range moved it.
#
# Usage:
#   scripts/corpus_run.sh [RANGE] [OPTIONS]
#
#   RANGE                 A..B. Each end is built in its own worktree and run,
#                         and the two summaries are printed side by side.
#                         Omit it to run the working tree as it stands.
#
# Options:
#   --corpus ROOT         directory holding fullcorpus/. Default
#                         $TRACEMATCH_CORPUS, else the crate root.
#   --out DIR             where the runs land. Default
#                         ROOT/unified-lab/runs/<stamp>. Never the repository.
#   --rebase              after a gate diverges, copy what it measured over
#                         its golden, with a dated header line naming the
#                         range. Without this the goldens are never written.
#   --no-geolife          skip GeoLife, lab and gate.
#   --no-gates            lab and tables only.
#   --lab-args "..."      extra flags for every lab invocation, quoted.
#   -h, --help            this text.
#
# Each run directory holds the lab's stdout in lab.log, the GeoJSON, the
# tables all_unified_sections.tsv, all_unified_dropped.tsv and replay.tsv,
# and summary.tsv with the counts the open questions ask for. Gate output is
# under gates/: the recorded golden each gate measured and the diff against
# the one it was compared to.
#
# GeoLife is looked for at $LAB_GEOLIFE_DIR, then under the crate root, then
# under ROOT, and fetched into ROOT/geolife if absent.

set -uo pipefail

TM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$(cd "$TM_DIR/.." && pwd)"
if grep -q '^\[workspace\]' "$WS_DIR/Cargo.toml" 2>/dev/null; then
  RUN_DIR="$WS_DIR"
  PKG=(-p tracematch)
else
  RUN_DIR="$TM_DIR"
  PKG=()
fi

RANGE=""
ROOT="${TRACEMATCH_CORPUS:-$TM_DIR}"
OUT=""
REBASE=0
GEOLIFE=1
GATES=1
LAB_ARGS=""

while [ $# -gt 0 ]; do
  case "$1" in
    --corpus) ROOT="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --rebase) REBASE=1; shift ;;
    --no-geolife) GEOLIFE=0; shift ;;
    --no-gates) GATES=0; shift ;;
    --lab-args) LAB_ARGS="$2"; shift 2 ;;
    -h|--help) sed -n '2,38p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "unknown option: $1" >&2; exit 2 ;;
    *)
      if [ -n "$RANGE" ]; then echo "unexpected argument: $1" >&2; exit 2; fi
      RANGE="$1"; shift ;;
  esac
done

ROOT="$(cd "$ROOT" && pwd)" || { echo "corpus root not found: $ROOT" >&2; exit 2; }
CORPUS="$ROOT/fullcorpus"
if ! ls "$CORPUS"/*.gpx >/dev/null 2>&1; then
  echo "no GPX files under $CORPUS. Set TRACEMATCH_CORPUS or pass --corpus." >&2
  exit 2
fi
PRIVATE_GOLDEN="$CORPUS/_bitwise_golden.txt"
GEOLIFE_GOLDEN="$TM_DIR/tests/fixtures/geolife_bitwise_golden.txt"

STAMP="$(date +%Y%m%d-%H%M%S)"
[ -n "$OUT" ] || OUT="$ROOT/unified-lab/runs/$STAMP"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
case "$OUT" in
  "$TM_DIR"/unified-lab/*|"$TM_DIR"/target/*) ;;
  "$TM_DIR"/*)
    echo "refusing to write runs inside the repository: $OUT" >&2
    echo "use --out for a directory outside it, or under unified-lab/" >&2
    exit 2 ;;
esac

# --------------------------------------------------------------- geolife

GEOLIFE_DATA=""
if [ "$GEOLIFE" -eq 1 ]; then
  for candidate in "${LAB_GEOLIFE_DIR:-}" \
      "$TM_DIR/geolife/Geolife Trajectories 1.3/Data" \
      "$ROOT/geolife/Geolife Trajectories 1.3/Data"; do
    [ -n "$candidate" ] && [ -d "$candidate" ] && { GEOLIFE_DATA="$candidate"; break; }
  done
  if [ -z "$GEOLIFE_DATA" ]; then
    echo "GeoLife not found, fetching into $ROOT/geolife"
    if GEOLIFE_DIR="$ROOT/geolife" "$TM_DIR/scripts/fetch_geolife.sh"; then
      GEOLIFE_DATA="$ROOT/geolife/Geolife Trajectories 1.3/Data"
    else
      echo "GeoLife fetch failed, continuing without it" >&2
      GEOLIFE=0
    fi
  fi
fi

# ------------------------------------------------------------------ ends

# Where each end is built and what to call it in the output.
declare -a END_NAMES=() END_DIRS=() END_PKGS=()
WORKTREES=()
if [ -n "$RANGE" ]; then
  case "$RANGE" in
    *..*) ;;
    *) echo "RANGE must be A..B, got $RANGE" >&2; exit 2 ;;
  esac
  for ref in "${RANGE%%..*}" "${RANGE##*..}"; do
    sha="$(git -C "$TM_DIR" rev-parse --short "$ref")" || {
      echo "not a tracematch revision: $ref" >&2; exit 2
    }
    wt="$OUT/worktrees/$sha"
    if [ ! -d "$wt" ]; then
      git -C "$TM_DIR" worktree add --detach "$wt" "$sha" >/dev/null || exit 2
      WORKTREES+=("$wt")
    fi
    END_NAMES+=("$sha")
    END_DIRS+=("$wt")
  done
else
  END_NAMES+=("tree")
  END_DIRS+=("$RUN_DIR")
fi

cleanup() {
  for wt in "${WORKTREES[@]:-}"; do
    [ -n "$wt" ] && git -C "$TM_DIR" worktree remove --force "$wt" >/dev/null 2>&1
  done
}
trap cleanup EXIT

# ------------------------------------------------------------------- lab

failed=0

# A worktree end builds into its own target directory. Two ends sharing one
# ran the first end's lab binary for the second: cargo judged it fresh, and
# the run reported the wrong revision's output under the right name.
target_dir() {
  local build="$1"
  if [ "$build" = "$RUN_DIR" ]; then
    echo ""
  else
    echo "CARGO_TARGET_DIR=$build/target"
  fi
}

# Run the lab once. $1 build dir, $2 output dir, then the environment for
# it as VAR=value words. Fails the run when the lab exits non-zero.
run_lab() {
  local build="$1" out="$2"
  shift 2
  mkdir -p "$out"
  local pkg=()
  if [ "$build" = "$RUN_DIR" ]; then pkg=("${PKG[@]}"); fi
  # shellcheck disable=SC2086
  if ! (cd "$build" && env "$@" $(target_dir "$build") \
      cargo run "${pkg[@]}" --release --example unified_lab -- \
      "$CORPUS" --sport All --replay-identity --out "$out" $LAB_ARGS) \
      > "$out/lab.log" 2>&1; then
    echo "  lab failed, see $out/lab.log" >&2
    failed=$((failed + 1))
    return 1
  fi
}

# The counts the open questions ask for, from the tables, as key<TAB>value.
summarise() {
  local out="$1"
  local sections="$out/all_unified_sections.tsv"
  local dropped="$out/all_unified_dropped.tsv"
  local replay="$out/replay.tsv"
  if [ ! -f "$sections" ]; then
    printf 'sections\tabsent\n' > "$out/summary.tsv"
    echo "  no tables: this end's lab predates them"
    return
  fi
  {
    awk -F'\t' 'NR > 1 {
        n++
        if ($8 == 0) under_activity++
        if ($9 == 0) under_support++
        if ($10 == 0) under_support_base++
        if ($11 > 0) { clipped++; clip_m += $11 }
        if ($12 == 1) drawn_empty++
      }
      END {
        printf "sections\t%d\n", n
        printf "under_activity_floor\t%d\n", under_activity
        printf "under_support_floor\t%d\n", under_support
        printf "under_support_floor_base\t%d\n", under_support_base
        printf "seam_clipped\t%d\n", clipped
        printf "seam_clipped_metres\t%.0f\n", clip_m
        printf "drawn_empty\t%d\n", drawn_empty
      }' "$sections"
    if [ -f "$dropped" ]; then
      printf 'dropped_by_activity_floor\t%d\n' "$(($(wc -l < "$dropped") - 1))"
    fi
    if [ -f "$replay" ]; then
      awk -F'\t' 'NR == 2 {
        printf "phantom_mints\t%d\n", $13
        printf "captures\t%d\n", $14
        printf "final_visible\t%d\n", $3
        printf "overlapping\t%d\n", $4
      }' "$replay"
    fi
  } > "$out/summary.tsv"
  sed 's/^/  /' "$out/summary.tsv"
}

for i in "${!END_NAMES[@]}"; do
  name="${END_NAMES[$i]}"
  build="${END_DIRS[$i]}"
  echo
  echo "=== $name: private corpus ($CORPUS)"
  if run_lab "$build" "$OUT/$name/private" LAB_GEOLIFE_DIR=; then
    summarise "$OUT/$name/private"
  fi
  if [ "$GEOLIFE" -eq 1 ]; then
    echo
    echo "=== $name: GeoLife ($GEOLIFE_DATA)"
    if run_lab "$build" "$OUT/$name/geolife" LAB_GEOLIFE_DIR="$GEOLIFE_DATA"; then
      summarise "$OUT/$name/geolife"
    fi
  fi
done

if [ "${#END_NAMES[@]}" -eq 2 ]; then
  a="${END_NAMES[0]}"
  b="${END_NAMES[1]}"
  for corpus in private geolife; do
    [ -f "$OUT/$a/$corpus/summary.tsv" ] && [ -f "$OUT/$b/$corpus/summary.tsv" ] || continue
    echo
    echo "=== $corpus: $a -> $b"
    join -t "$(printf '\t')" -a 1 -a 2 -e '-' -o '0,1.2,2.2' \
      <(sort "$OUT/$a/$corpus/summary.tsv") <(sort "$OUT/$b/$corpus/summary.tsv") \
      | awk -F'\t' -v a="$a" -v b="$b" 'BEGIN { printf "  %-28s %10s %10s\n", "metric", a, b }
        { printf "  %-28s %10s %10s%s\n", $1, $2, $3, ($2 != $3 ? "  *" : "") }'
    sa="$OUT/$a/$corpus/all_unified_sections.tsv"
    sb="$OUT/$b/$corpus/all_unified_sections.tsv"
    if [ -f "$sa" ] && [ -f "$sb" ]; then
      gone="$(comm -23 <(cut -f1 "$sa" | tail -n +2 | sort) <(cut -f1 "$sb" | tail -n +2 | sort) | wc -l)"
      new="$(comm -13 <(cut -f1 "$sa" | tail -n +2 | sort) <(cut -f1 "$sb" | tail -n +2 | sort) | wc -l)"
      echo "  section ids only at $a: $gone, only at $b: $new"
    fi
  done
fi

# ----------------------------------------------------------------- gates

# Run one gate read-only and diff what it measured against its golden.
# $1 label, $2 golden path, $3 cargo feature, $4 test target, then env words.
gate_failed=0
run_gate() {
  local label="$1" golden="$2" feature="$3" target="$4"
  shift 4
  local dir="$OUT/gates"
  mkdir -p "$dir"
  local record="$dir/$label.txt"
  echo
  echo "=== gate: $label"
  local build="${END_DIRS[-1]}"
  local pkg=()
  if [ "$build" = "$RUN_DIR" ]; then pkg=("${PKG[@]}"); fi
  (cd "$build" && env "$@" $(target_dir "$build") TRACEMATCH_BITWISE_RECORD="$record" \
      cargo test "${pkg[@]}" --release --features "$feature" --test "$target" -- --nocapture) \
      > "$dir/$label.log" 2>&1
  local status=$?
  grep -E '^(A|B|C|L) |^lift veto|^golden|^cost|^debug|^measured' "$dir/$label.log" | sed 's/^/  /'
  if [ ! -f "$record" ]; then
    echo "  the gate recorded nothing, see $dir/$label.log" >&2
    failed=$((failed + 1))
    return 1
  fi
  if [ ! -f "$golden" ]; then
    echo "  no golden at $golden: the gate recorded one there on this run"
    return 0
  fi
  local digest_diff
  digest_diff="$(diff <(grep -vE '^(perf_|#)' "$golden") <(grep -vE '^(perf_|#)' "$record"))"
  if [ -z "$digest_diff" ]; then
    echo "  digests match the golden"
  else
    echo "  digests DIVERGED from $golden:"
    echo "$digest_diff" | sed 's/^/    /'
  fi
  echo "  costs, golden then measured:"
  join -t "$(printf '\t')" -a 1 -a 2 -e '-' -o '0,1.2,2.2' \
    <(grep '^perf_' "$golden" | tr ' ' '\t' | sort) \
    <(grep '^perf_' "$record" | tr ' ' '\t' | sort) \
    | awk -F'\t' '{ printf "    %-22s %12s %12s\n", $1, $2, $3 }'
  echo "$digest_diff" > "$dir/$label.diff"
  if [ "$status" -ne 0 ]; then
    echo "  gate FAILED (exit $status), see $dir/$label.log"
    gate_failed=$((gate_failed + 1))
    if [ "$REBASE" -eq 1 ]; then
      # The harness writes the golden and its `# rebased` header, so a rebase
      # from here is the same gate run again with the reason in the switch.
      local moved="${RANGE:-working tree at $(git -C "$TM_DIR" rev-parse --short HEAD)}"
      echo "  rebasing $golden through the gate: $moved"
      (cd "$build" && env "$@" $(target_dir "$build") \
          TRACEMATCH_BITWISE_REBASE="corpus_run.sh, $moved" \
          cargo test "${pkg[@]}" --release --features "$feature" --test "$target" -- --nocapture) \
          >> "$dir/$label.log" 2>&1 \
        && echo "  REBASED $golden" \
        || echo "  the rebase run FAILED, see $dir/$label.log"
      case "$golden" in
        "$TM_DIR"/*) echo "    this golden is in the repository: commit it with the change that moved it" ;;
      esac
    fi
    return 1
  fi
  echo "  gate passed"
}

if [ "$GATES" -eq 1 ]; then
  run_gate private "$PRIVATE_GOLDEN" real-corpus full_corpus_bitwise TRACEMATCH_CORPUS="$ROOT"
  if [ "$GEOLIFE" -eq 1 ]; then
    run_gate geolife "$GEOLIFE_GOLDEN" public-corpus geolife_bitwise LAB_GEOLIFE_DIR="$GEOLIFE_DATA"
  fi
fi

echo
echo "runs under $OUT"
if [ "$gate_failed" -gt 0 ] && [ "$REBASE" -eq 0 ]; then
  echo "$gate_failed gate(s) diverged and nothing was rebased. Read the diff, then --rebase if it was meant."
  exit 1
fi
[ "$failed" -eq 0 ] || exit 1
