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
# Every module checked whole first, and a warning fails it too. A module
# checked on its own can pass what a program importing it is refused
# (aether#2683), so each is also checked as an import: the checks of the
# modules importing it and the tests' builds take every function of every
# imported module (aether#2613), and every module is imported by one.
for module in aephysics/*/module.ae; do
    name="$(basename "$(dirname "$module")")"
    if ! AETHER_LIB_DIR="$root" ae check "$module" >"target/check_$name.log" 2>&1 || grep -q "warning\[" "target/check_$name.log"; then
        echo "FAIL  check $name"
        grep -A3 "error\[\|warning\[" "target/check_$name.log" | head -12
        failures=$((failures + 1))
    fi
done
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
# The determinism scenes' every step against the golden trace: the same
# bits on every platform and build (#125). A deliberate change of results
# regenerates it with scripts/golden_trace.sh and says why in its commit.
if ! scripts/golden_trace.sh check; then
    failures=$((failures + 1))
fi
if [ "$failures" -ne 0 ]; then
    echo "$failures failure(s)"
    exit 1
fi
echo "every test passed"
