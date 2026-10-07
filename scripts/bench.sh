#!/usr/bin/env bash
# Each layer of aephysics against the same code in the reference:
# builds bench/<layer>.ae with ae and bench/<layer>_box3d.c against the
# library scripts/fetch_references.sh made, then runs both.
#
#   scripts/bench.sh [layer]
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
# The build's extra C flags (aether.toml), empty unless the caller sets them:
# AETHER_AEPHYSICS_CFLAGS=-mavx2 builds the wide contact solver at eight lanes a register.
export AETHER_AEPHYSICS_CFLAGS="${AETHER_AEPHYSICS_CFLAGS:-}"
mkdir -p target
layers="${1:-tree}"
for layer in $layers; do
    # A layer measured against C of our own rather than against the
    # reference (the lanes) builds and runs that first, with the flags
    # aether.toml gives ae build so both sides are compiled alike.
    if [ -f "bench/${layer}_native.c" ]; then
        native_cflags="$(sed -n 's/^cflags = "\(.*\)"/\1/p' aether.toml)"
        # aether.toml's ${AETHER_AEPHYSICS_CFLAGS}, expanded as ae expands it (empty when unset).
        native_cflags="${native_cflags//'${AETHER_AEPHYSICS_CFLAGS}'/${AETHER_AEPHYSICS_CFLAGS:-}}"
        gcc $native_cflags "bench/${layer}_native.c" -o "target/${layer}_native" -lm || exit 1
        native="target/${layer}_native"; [ -x "$native" ] || native="$native.exe"
        "$native"
    fi
    # A layer the reference only has inside its world (the broad phase) runs ours alone.
    if [ -f "bench/${layer}_box3d.c" ]; then
        gcc -O3 -Ireference/box3d/include -Ireference/box3d/shared "bench/${layer}_box3d.c" -o "target/${layer}_box3d" -Lreference/box3d/out/shared -Lreference/box3d/out/src -lshared -lbox3d -lm || exit 1
        ref="target/${layer}_box3d"; [ -x "$ref" ] || ref="$ref.exe"
        "$ref"
    fi
    AETHER_LIB_DIR="$root" ae build "bench/$layer.ae" -o "target/$layer" >"target/$layer.log" 2>&1 || { cat "target/$layer.log"; exit 1; }
    ours="target/$layer"; [ -x "$ours" ] || ours="$ours.exe"
    "$ours"
done
