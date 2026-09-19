#!/usr/bin/env bash
# The bake-off: the same scenes on every reference engine, single thread,
# each at its own recommended settings for a 60 Hz game step. Builds the
# bench programs against the libraries scripts/fetch_references.sh made.
#
#   scripts/bake.sh [steps]
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
steps="${1:-300}"
mkdir -p target
# Jolt's public defines and instruction flags have to match the library's build
# (the structs change shape with them); read from its ninja file.
jolt_defines="$(grep -m1 'DEFINES = ' reference/jolt/out/build.ninja | sed 's/^ *DEFINES = //')"
jolt_flags="-O3 -fno-rtti -fno-exceptions -ffp-contract=off -mavx2 -mbmi -mpopcnt -mlzcnt -mf16c -mfma -mfpmath=sse"
gcc -O3 -Ireference/box3d/include bench/bake_box3d.c -o target/bake_box3d -Lreference/box3d/out/src -lbox3d -lm || exit 1
g++ -std=c++17 $jolt_flags $jolt_defines -Ireference/jolt bench/bake_jolt.cpp -o target/bake_jolt -Lreference/jolt/out -lJolt || exit 1
for scene in pyramid pile chain; do
    for engine in box3d jolt; do
        exe="target/bake_$engine"
        [ -x "$exe" ] || exe="$exe.exe"
        "$exe" "$scene" "$steps"
    done
done
