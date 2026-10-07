#!/usr/bin/env bash
# Drive check-no-private-data.sh against throwaway repositories: a staged or
# tracked trace fails, a clean tree passes. CI runs this before the guard itself,
# so a guard that silently stopped refusing fails here rather than passing.
#
# Usage: scripts/test-check-no-private-data.sh

set -uo pipefail

GUARD="$(cd "$(dirname "$0")" && pwd)/check-no-private-data.sh"
failures=0
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT

# Git exports these to hooks and they beat cwd, which would point a fixture
# at this repository.
unset GIT_DIR GIT_INDEX_FILE GIT_WORK_TREE GIT_OBJECT_DIRECTORY GIT_COMMON_DIR

fixture() {
    local dir
    dir="$(mktemp -d "$scratch/repo.XXXX")"
    git -C "$dir" init -q
    mkdir -p "$dir/src"
    echo x > "$dir/src/lib.rs"
    for path in "$@"; do
        mkdir -p "$dir/$(dirname "$path")"
        echo x > "$dir/$path"
    done
    git -C "$dir" add -f -A
    git -C "$dir" -c user.name=t -c user.email=t@t -c core.hooksPath=/dev/null \
        commit -q -m base
    echo "$dir"
}

expect() {
    local want="$1" name="$2" dir="$3"
    shift 3
    local got=0
    (cd "$dir" && "$GUARD" "$@") >/dev/null 2>&1 || got=$?
    if [ "$got" -ne "$want" ]; then
        echo "FAIL: $name: exit $got, expected $want"
        failures=$((failures + 1))
    else
        echo "ok: $name"
    fi
}

clean="$(fixture)"
expect 0 "a clean tree passes the whole-tree mode" "$clean" --all
expect 0 "nothing staged passes the staged mode" "$clean"

# The corpus names files after the activity, and git quotes a non-ASCII path
# unless told not to, which hid it from every rule.
for path in tests/fixtures/ride.gpx data/export.fit.gz store.sqlite citycorpus/notes.txt \
    corpus/1.txt "citycorpus/Savièse Hiking.gpx" "data/Course à pied.gpx" \
    tests/fixtures/streams/i1.json tests/fixtures/raw_traces/a.txt private/notes.txt; do
    tracked="$(fixture "$path")"
    expect 1 "$path tracked fails the whole-tree mode" "$tracked" --all
    expect 0 "$path tracked is not staged, so the staged mode passes" "$tracked"

    staged="$(fixture)"
    mkdir -p "$staged/$(dirname "$path")"
    echo x > "$staged/$path"
    git -C "$staged" add -f "$path"
    expect 1 "$path staged fails the staged mode" "$staged"
done

expect 2 "an unknown argument is refused" "$clean" --bogus

if [ "$failures" -gt 0 ]; then
    echo "$failures case(s) failed"
    exit 1
fi
echo "all cases passed"
