#!/usr/bin/env bash
# Drive fetch_corpus.sh against a stub curl that mimics the API's routes: a
# track is served only at /activity/<id>/gpx-file, and /activity/<id>/gpx is
# a 404. An activity whose file holds no <trkpt> counts as having no GPS.
#
# Usage: scripts/test-fetch-corpus.sh

set -uo pipefail

SCRIPT="$(cd "$(dirname "$0")" && pwd)/fetch_corpus.sh"
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT
mkdir -p "$scratch/bin"

cat > "$scratch/bin/curl" <<'STUB'
#!/usr/bin/env bash
out=""; url=""
while [ $# -gt 0 ]; do
  case "$1" in
    -o) out="$2"; shift 2 ;;
    -u) shift 2 ;;
    -*) shift ;;
    *) url="$1"; shift ;;
  esac
done
case "$url" in
  */activities\?*) echo '[{"id":"i1"},{"id":"i2"}]' ;;
  */activity/i1/gpx-file) printf '<gpx><trk><trkseg><trkpt lat="1" lon="2"/></trkseg></trk></gpx>' > "$out" ;;
  */activity/i2/gpx-file) printf '<gpx></gpx>' > "$out" ;;
  *) exit 22 ;;
esac
STUB
chmod +x "$scratch/bin/curl"

PATH="$scratch/bin:$PATH" INTERVALS_API_KEY=k "$SCRIPT" --dest "$scratch/corpus" >/dev/null 2>&1

failures=0
[ -f "$scratch/corpus/i1.gpx" ] || { echo "FAIL: the track at /gpx-file was not downloaded"; failures=1; }
[ -f "$scratch/corpus/i2.gpx" ] && { echo "FAIL: a file without trackpoints was kept"; failures=1; }
[ "$failures" -eq 0 ] && echo "ok"
exit "$failures"
