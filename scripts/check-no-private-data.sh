#!/usr/bin/env bash
# Refuse to stage personal activity data.
#
# gitignore only covers files git is not already tracking, so it cannot stop
# `git add -f` and it does nothing once a file is tracked. This runs against the
# index, which is the last point where the data can still be kept out.
#
# The hook runs only in a clone that ran scripts/setup-hooks.sh, so `--all`
# judges every tracked path instead, and CI runs that on every push.
#
# Usage:
#   check-no-private-data.sh          the staged paths, for pre-commit
#   check-no-private-data.sh --all    every tracked path, for CI

set -euo pipefail

# Git quotes a path holding a non-ASCII byte by default, and the quotes hide the
# extension and the directory from every rule below. The corpora name their
# files after the activity, so that is the common case, not the odd one.
git() { command git -c core.quotePath=off "$@"; }

case "${1:-}" in
  "")
    staged="$(git diff --cached --name-only --diff-filter=ACMR)"
    hint="Unstage them with: git restore --staged <path>"
    ;;
  --all)
    staged="$(git ls-files)"
    hint="Stop tracking them with: git rm --cached <path>, and check whether they were pushed"
    ;;
  *)
    echo "check-no-private-data: unknown argument $1" >&2
    exit 2
    ;;
esac
[ -n "$staged" ] || exit 0

# Track formats, including the compressed and archived forms, and .plt, which
# is what GeoLife ships.
blocked="$(echo "$staged" | grep -iE '\.(gpx|fit|tcx|kml|plt|db|sqlite3?)(\.(gz|bz2|xz|zip))?$' || true)"

# Corpus directories, whatever they hold and whatever it is named.
blocked="$blocked
$(echo "$staged" | grep -E '^(fullcorpus|citycorpus|citycorpus_sections|aussietest|unified-lab|geolife|corpus)/' || true)"

# The fixture directories .gitignore keeps for local data, which hold activity
# streams and raw traces as JSON and text, and any private/ directory.
blocked="$blocked
$(echo "$staged" | grep -E '^tests/fixtures/(raw_traces|private|gpx|streams)/|(^|/)private/' || true)"

blocked="$(echo "$blocked" | grep -v '^$' || true)"

if [ -n "$blocked" ]; then
  echo "Refusing to commit personal activity data:" >&2
  echo "$blocked" | sed 's/^/  /' >&2
  echo >&2
  echo "These are real GPS traces. Once committed they stay in history even after" >&2
  echo "deletion, and rewriting a published branch does not remove the blob from" >&2
  echo "the remote. $hint" >&2
  exit 1
fi
