#!/usr/bin/env bash
# Refuse to stage personal activity data.
#
# gitignore only covers files git is not already tracking, so it cannot stop
# `git add -f` and it does nothing once a file is tracked. This runs against the
# index, which is the last point where the data can still be kept out.

set -euo pipefail

staged="$(git diff --cached --name-only --diff-filter=ACMR)"
[ -n "$staged" ] || exit 0

# Track formats, including the compressed and archived forms, and .plt, which
# is what GeoLife ships.
blocked="$(echo "$staged" | grep -iE '\.(gpx|fit|tcx|kml|plt|db|sqlite3?)(\.(gz|bz2|xz|zip))?$' || true)"

# Corpus directories, whatever they hold and whatever it is named.
blocked="$blocked
$(echo "$staged" | grep -E '^(fullcorpus|citycorpus|citycorpus_sections|aussietest|unified-lab|geolife|corpus)/' || true)"

blocked="$(echo "$blocked" | grep -v '^$' || true)"

if [ -n "$blocked" ]; then
  echo "Refusing to commit personal activity data:" >&2
  echo "$blocked" | sed 's/^/  /' >&2
  echo >&2
  echo "These are real GPS traces. Once committed they stay in history even after" >&2
  echo "deletion, and rewriting a published branch does not remove the blob from" >&2
  echo "the remote. Unstage them with: git restore --staged <path>" >&2
  exit 1
fi
