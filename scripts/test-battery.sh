#!/usr/bin/env bash
# Scenario: a test binary dies on a signal while the others pass.
# Expected behaviour: run() counts a failure although cargo's result lines sum to zero.
set -uo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/battery.sh"

fake="$(mktemp)"
trap 'rm -f "$fake"' EXIT

check() {
    local name="$1" want="$2"
    if [ "$failed" -ne "$want" ]; then
        printf 'FAIL %s: failed=%d, want %d\n' "$name" "$failed" "$want"
        exit 1
    fi
    printf 'ok %s\n' "$name"
}

fake_run() {
    printf '#!/usr/bin/env bash\ncat <<"EOT"\n%s\nEOT\nexit %d\n' "$1" "$2" > "$fake"
    chmod +x "$fake"
    failed=0
    run "fake" "$fake" > /dev/null
}

fake_run 'test result: ok. 3 passed; 0 failed; 0 ignored
error: test failed, to rerun pass `-p x --test t`
Caused by:
  process didn'"'"'t exit successfully: `t` (signal: 6, SIGABRT: process abort signal)' 101
check "signal with passing siblings" 1

fake_run 'test result: ok. 3 passed; 0 failed; 0 ignored' 0
check "clean run" 0

fake_run 'test result: FAILED. 1 passed; 2 failed; 0 ignored' 101
check "counted failures" 2

fake_run 'error: could not compile `x`' 101
check "no result lines" 1

fake_run 'test result: ok. 3 passed; 0 failed; 0 ignored' 101
check "non-zero status with nothing counted" 1
