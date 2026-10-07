#!/usr/bin/env bash
# Build and run every test in tests/ with ae: what CI runs through aeb,
# runnable anywhere ae is.
#
#   scripts/test.sh
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
# The build's extra C flags (aether.toml), empty unless the caller sets them:
# AETHER_AEPHYSICS_CFLAGS=-mavx2 builds the wide contact solver at eight lanes a register.
export AETHER_AEPHYSICS_CFLAGS="${AETHER_AEPHYSICS_CFLAGS:-}"
mkdir -p target
failures=0
for test in tests/test_*.ae; do
    name="$(basename "$test" .ae)"
    if ! AETHER_LIB_DIR="$root" ae build "$test" -o "target/$name" >"target/$name.log" 2>&1; then
        echo "FAIL  $name (build)"
        grep -i "error" "target/$name.log" | head -5
        failures=$((failures + 1))
        continue
    fi
    exe="target/$name"
    [ -x "$exe" ] || exe="$exe.exe"
    if "$exe"; then
        echo "ok    $name"
    else
        echo "FAIL  $name"
        failures=$((failures + 1))
    fi
done
if [ "$failures" -ne 0 ]; then
    echo "$failures failure(s)"
    exit 1
fi
echo "every test passed"
