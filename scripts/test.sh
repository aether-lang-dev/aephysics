#!/usr/bin/env bash
# Build and run every test in aephysics/ with ae: what CI runs through aeb,
# runnable anywhere ae is.
#
#   scripts/test.sh
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
mkdir -p target
failures=0
for test in aephysics/test_*.ae; do
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
