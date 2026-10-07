#!/usr/bin/env bash
# The determinism scenes stepped and every body's checksum after every
# step compared, by its bits, with tests/golden/determinism_trace.txt: the
# same scene gives the same bits on every platform and build (#125).
#
#   scripts/golden_trace.sh check    # diff the current build against the golden
#   scripts/golden_trace.sh write    # regenerate it (a deliberate change of results)
#
# Uses target/test_determinism when scripts/test.sh has built it, else
# builds it. Only the scene, the step and the bits are compared: the
# rounded value beside them is for reading.
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
mode="${1:-check}"
golden="tests/golden/determinism_trace.txt"
mkdir -p target
exe="target/test_determinism"
[ -x "$exe" ] || exe="$exe.exe"
if [ ! -x "$exe" ]; then
    export AETHER_AEPHYSICS_CFLAGS="${AETHER_AEPHYSICS_CFLAGS:-}"
    AETHER_LIB_DIR="$root" ae build tests/test_determinism.ae -o target/test_determinism >target/test_determinism.log 2>&1 || {
        echo "FAIL  golden trace (build)"
        exit 1
    }
    exe="target/test_determinism"
    [ -x "$exe" ] || exe="$exe.exe"
fi
AEPHYSICS_TRACE=1 "$exe" | tr -d '\r' | awk '$1 == "trace" { print $2, $3, $5 }' > target/determinism_trace.txt
if [ "$mode" = "write" ]; then
    mkdir -p "$(dirname "$golden")"
    cp target/determinism_trace.txt "$golden"
    echo "wrote $golden ($(wc -l < "$golden") steps)"
    exit 0
fi
if [ ! -s target/determinism_trace.txt ]; then
    echo "FAIL  golden trace (no trace printed)"
    exit 1
fi
if diff -q "$golden" target/determinism_trace.txt >/dev/null; then
    echo "ok    golden trace ($(wc -l < "$golden") steps, bit for bit)"
    exit 0
fi
echo "FAIL  golden trace: the first steps that differ (golden < > this build):"
diff "$golden" target/determinism_trace.txt | head -10
exit 1
