#!/usr/bin/env bash
# Build a local GPX corpus from your own intervals.icu account.
#
# The corpora these tests read are personal activity history. They are never
# committed and never shared, so every machine builds its own from the account
# that owns the data. Nothing here is specific to any user: supply your own key
# and you get your own traces.
#
# Get an API key from https://intervals.icu/settings under "Developer Settings".
#
# Usage:
#   INTERVALS_API_KEY=xxxxxxxx scripts/fetch_corpus.sh
#   INTERVALS_API_KEY=xxxxxxxx scripts/fetch_corpus.sh --oldest 2024-01-01 --dest mycorpus
#
# Options (env or flag):
#   INTERVALS_API_KEY   required, your key. Never pass it as a flag: flags land
#                       in shell history and in the process list.
#   INTERVALS_ATHLETE   athlete id. Defaults to "0", which the API resolves to
#                       the key's own athlete.
#   --oldest DATE       earliest activity date, YYYY-MM-DD. Default 3 years ago.
#   --newest DATE       latest activity date, YYYY-MM-DD. Default today.
#   --dest DIR          output directory. Default "corpus".
#   --sport NAME        keep only this sport (e.g. Run, Ride). Default all.
#   --limit N           stop after N activities. Default unlimited.
#
# Output: DIR/<activity_id>.gpx, one per activity that has GPS.

set -euo pipefail

API="https://intervals.icu/api/v1"
ATHLETE="${INTERVALS_ATHLETE:-0}"
DEST="corpus"
SPORT=""
LIMIT=0
NEWEST="$(date +%F)"
OLDEST="$(date -d '3 years ago' +%F 2>/dev/null || date -v-3y +%F)"

while [ $# -gt 0 ]; do
  case "$1" in
    --oldest) OLDEST="$2"; shift 2 ;;
    --newest) NEWEST="$2"; shift 2 ;;
    --dest)   DEST="$2";   shift 2 ;;
    --sport)  SPORT="$2";  shift 2 ;;
    --limit)  LIMIT="$2";  shift 2 ;;
    -h|--help) sed -n '2,28p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [ -z "${INTERVALS_API_KEY:-}" ]; then
  echo "INTERVALS_API_KEY is not set." >&2
  echo "Create one at https://intervals.icu/settings under Developer Settings," >&2
  echo "then run:  INTERVALS_API_KEY=xxxx $0" >&2
  exit 2
fi

for tool in curl jq; do
  command -v "$tool" >/dev/null 2>&1 || { echo "error: $tool is required" >&2; exit 1; }
done

# The API takes HTTP Basic with the literal username API_KEY.
auth() { curl -sS --fail-with-body -u "API_KEY:${INTERVALS_API_KEY}" "$@"; }

mkdir -p "$DEST"

echo "Listing activities $OLDEST to $NEWEST"

# INTERVALS_ATHLETE wins. Otherwise try the aliases the API accepts for "the
# athlete this key belongs to", so nobody has to look their own id up.
list=""
for candidate in "${INTERVALS_ATHLETE:-}" 0 me; do
  [ -n "$candidate" ] || continue
  if list="$(auth "$API/athlete/$candidate/activities?oldest=$OLDEST&newest=$NEWEST" 2>/dev/null)"; then
    ATHLETE="$candidate"
    break
  fi
  list=""
done

if [ -z "$list" ]; then
  echo "Could not list activities for any of: ${INTERVALS_ATHLETE:-} 0 me" >&2
  echo "A 401 means the key is wrong. If your account needs an explicit id, set" >&2
  echo "INTERVALS_ATHLETE to it (find it in your intervals.icu profile URL)." >&2
  exit 1
fi
echo "Using athlete $ATHLETE"

filter='.[] | select(.id != null)'
[ -n "$SPORT" ] && filter="$filter | select(.type == \"$SPORT\")"
ids="$(echo "$list" | jq -r "$filter | .id")"

total="$(echo "$ids" | grep -c . || true)"
if [ "$total" -eq 0 ]; then
  echo "No activities matched. Widen --oldest, or check --sport." >&2
  exit 1
fi
echo "$total activities matched"

got=0
skipped=0
for id in $ids; do
  if [ "$LIMIT" -gt 0 ] && [ "$got" -ge "$LIMIT" ]; then
    break
  fi

  out="$DEST/$id.gpx"
  if [ -f "$out" ]; then
    got=$((got + 1))
    continue
  fi

  # Activities without GPS return an error rather than a track. That is normal
  # for indoor work, so it is counted and skipped rather than treated as fatal.
  if auth -o "$out.part" "$API/activity/$id/gpx" 2>/dev/null && [ -s "$out.part" ] \
     && grep -q '<trkpt' "$out.part"; then
    mv "$out.part" "$out"
    got=$((got + 1))
    printf '\r%s downloaded, %s without GPS' "$got" "$skipped"
  else
    rm -f "$out.part"
    skipped=$((skipped + 1))
  fi
done
printf '\n'

echo "$got GPX files in $DEST/ ($skipped activities had no GPS)"
echo
echo "Point the tests at it with:"
echo "  export TRACEMATCH_CORPUS=\"$(cd "$(dirname "$DEST")" && pwd)/$(basename "$DEST")\""
echo
echo "This data is yours and stays local. It is gitignored, and"
echo "scripts/check-no-private-data.sh refuses to stage it."
