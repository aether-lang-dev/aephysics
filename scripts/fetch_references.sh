#!/usr/bin/env bash
# The reference engines this one is measured against, fetched into
# reference/ (ignored by git; pinned to the commits below) and built.
#
#   scripts/fetch_references.sh            # box3d and jolt
#
# Box3D: Erin Catto, C17, MIT. Jolt: Jorrit Rouwe, C++17, MIT.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
mkdir -p reference
BOX3D_REF="${BOX3D_REF:-f555ee4}"
JOLT_REF="${JOLT_REF:-v5.3.0}"

if [ ! -d reference/box3d ]; then
    git clone -q https://github.com/erincatto/box3d.git reference/box3d
fi
git -C reference/box3d checkout -q "$BOX3D_REF"

if [ ! -d reference/jolt ]; then
    git clone -q --depth 1 --branch "$JOLT_REF" https://github.com/jrouwe/JoltPhysics.git reference/jolt
fi

# Both build with CMake; Release, static libraries, no samples.
cmake -S reference/box3d -B reference/box3d/out -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release \
      -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=OFF -DBOX3D_BENCHMARKS=ON -DBOX3D_VALIDATE=OFF >/dev/null
cmake --build reference/box3d/out -j >/dev/null
cmake -S reference/jolt/Build -B reference/jolt/out -G Ninja -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Distribution \
      -DTARGET_HELLO_WORLD=OFF -DTARGET_SAMPLES=OFF -DTARGET_UNIT_TESTS=OFF -DTARGET_VIEWER=OFF \
      -DTARGET_PERFORMANCE_TEST=OFF -DENABLE_ALL_WARNINGS=OFF -DUSE_ASSERTS=OFF -DCROSS_PLATFORM_DETERMINISTIC=OFF >/dev/null
cmake --build reference/jolt/out -j >/dev/null
echo "references built: box3d $BOX3D_REF, jolt $JOLT_REF"
