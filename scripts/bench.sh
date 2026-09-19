#!/usr/bin/env bash
# Each layer of aephysics against the same code in the reference:
# builds bench/<layer>.ae with ae and bench/<layer>_box3d.c against the
# library scripts/fetch_references.sh made, then runs both.
#
#   scripts/bench.sh [layer]
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
mkdir -p target
layers="${1:-tree}"
for layer in $layers; do
    gcc -O3 -Ireference/box3d/include "bench/${layer}_box3d.c" -o "target/${layer}_box3d" -Lreference/box3d/out/src -lbox3d -lm || exit 1
    AETHER_LIB_DIR="$root" ae build "bench/$layer.ae" -o "target/$layer" >"target/$layer.log" 2>&1 || { cat "target/$layer.log"; exit 1; }
    ref="target/${layer}_box3d"; [ -x "$ref" ] || ref="$ref.exe"
    ours="target/$layer"; [ -x "$ours" ] || ours="$ours.exe"
    "$ref"
    "$ours"
done
