#!/usr/bin/env bash
#
# Fetch the Microsoft GeoLife GPS Trajectories 1.3 dataset into a gitignored
# geolife/ directory, for the unified-detector public-data battery.
#
# Dataset:  Microsoft GeoLife GPS Trajectories 1.3
# Citation: Yu Zheng, Xing Xie, Wei-Ying Ma. "GeoLife: A Collaborative Social
#           Networking Service among User, Location and Trajectory."
#           IEEE Data Engineering Bulletin, 2010.
#           (see also Zheng et al., WWW 2009; Zheng et al., ACM GIS 2008)
# Source:   https://www.microsoft.com/en-us/download/details.aspx?id=52367
# Licence:  Microsoft Research Licence Agreement. Free for research use.
#           Redistribution is NOT permitted. This script downloads the data
#           locally for evaluation only; it never commits or redistributes it.
#           The geolife/ directory is gitignored.
#
# Idempotent and non-interactive (CI-safe): if the Data tree already exists it
# does nothing; it never re-downloads when the data is already present.
#
# Overrides (env):
#   GEOLIFE_DIR   destination root (default: <repo>/geolife)
#   GEOLIFE_URL   archive URL (default: the Microsoft Download Center link)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST_DIR="${GEOLIFE_DIR:-$REPO_ROOT/geolife}"
DATA_DIR="$DEST_DIR/Geolife Trajectories 1.3/Data"
ZIP_PATH="$DEST_DIR/geolife.zip"
GEOLIFE_URL="${GEOLIFE_URL:-https://download.microsoft.com/download/F/4/8/F4894AA5-FDBC-481E-9285-D5F8C4C4F039/Geolife%20Trajectories%201.3.zip}"

count_users() {
  find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l | tr -d ' '
}

if [ -d "$DATA_DIR" ]; then
  echo "GeoLife already extracted: $DATA_DIR ($(count_users) users). Nothing to do."
  echo "Point the lab at it with:"
  echo "  export LAB_GEOLIFE_DIR=\"$DATA_DIR\""
  exit 0
fi

mkdir -p "$DEST_DIR"

if [ ! -f "$ZIP_PATH" ]; then
  echo "Downloading GeoLife 1.3 -> $ZIP_PATH"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL --retry 3 -o "$ZIP_PATH" "$GEOLIFE_URL"
  elif command -v wget >/dev/null 2>&1; then
    wget -q -O "$ZIP_PATH" "$GEOLIFE_URL"
  else
    echo "error: need curl or wget to download GeoLife" >&2
    exit 1
  fi
else
  echo "Reusing existing archive: $ZIP_PATH"
fi

echo "Extracting -> $DEST_DIR"
if command -v unzip >/dev/null 2>&1; then
  unzip -q -o "$ZIP_PATH" -d "$DEST_DIR"
else
  echo "error: need unzip to extract GeoLife" >&2
  exit 1
fi

if [ ! -d "$DATA_DIR" ]; then
  echo "error: extraction did not produce '$DATA_DIR'" >&2
  exit 1
fi

echo "GeoLife ready: $DATA_DIR ($(count_users) users)."
echo "Point the lab at it with:"
echo "  export LAB_GEOLIFE_DIR=\"$DATA_DIR\""
