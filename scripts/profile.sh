#!/usr/bin/env bash
# A sampling profile of an Aether program on Windows, where gprof and perf
# are not available under MinGW: the program's C is emitted with aetherc,
# compiled with the flags aether.toml gives ae build, linked with
# tools/sampler.c (a thread that records the main thread's instruction
# pointer every half millisecond), run, and its samples ranked by symbol
# with tools/rank.ae. The instrumented-call profilers distort the
# picture (a call counter on every tiny math function costs more than the
# function); this one costs nothing but the suspend.
#
#   scripts/profile.sh bench/physics_world.ae [top]
#
# On Linux, `ae build --profile` and perf do the same job.
set -uo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
source="${1:?usage: scripts/profile.sh file.ae [top]}"
top="${2:-30}"
name="$(basename "$source" .ae)"
mkdir -p target/profile
out="target/profile"
cflags="$(sed -n 's/^cflags = "\(.*\)"/\1/p' aether.toml)"
includes="$(ae cflags | tr ' ' '\n' | grep '^-[Ifw]' | tr '\n' ' ')"
libs="$(ae cflags | tr ' ' '\n' | grep '^-[Ll]' | tr '\n' ' ')"
AETHER_LIB_DIR="$root" aetherc "$source" "$out/$name.c" >"$out/$name.log" 2>&1 || { cat "$out/$name.log"; exit 1; }
gcc $cflags -g $includes -c "$out/$name.c" -o "$out/$name.o" 2>>"$out/$name.log" || { cat "$out/$name.log"; exit 1; }
gcc -O2 -c tools/sampler.c -o "$out/sampler.o" || exit 1
gcc "$out/$name.o" "$out/sampler.o" -o "$out/$name.exe" $libs || exit 1
ae build tools/rank.ae -o "$out/rank" >"$out/rank.log" 2>&1 || { cat "$out/rank.log"; exit 1; }
(cd "$out" && "./$name.exe") || exit 1
nm "$out/$name.exe" | awk '$2 ~ /^[tT]$/ {print $1, $3}' | sort > "$out/syms.txt"
(cd "$out" && ./rank.exe "$top")
