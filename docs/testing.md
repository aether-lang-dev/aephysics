# Testing and benchmarks

aephysics is held to two references. Box3D's own tests are ported, at
their tolerances, for every layer that has them; where Box3D tests a layer
only through its world, the test here is our own, usually against a brute
force. And every layer is timed against the same code in Box3D, built on
the same machine.

## Running the tests

```sh
scripts/test.sh
```

What it runs, in order, counting every failure and exiting non-zero if
there is any:

1. `ae check` on every module, whole. A warning fails it too.
2. Every benchmark built, not run, so an API change cannot leave one
   behind.
3. Every `tests/test_*.ae` built and run. Each prints its checks and exits
   non-zero on any failure.
4. Every `examples/*.ae` built and run.
5. `scripts/golden_trace.sh check`: the determinism trace (below).

`AETHER_AEPHYSICS_CFLAGS` adds C flags to every build: `-mavx2` makes the
wide stages one AVX2 register in the plain copy too, `-O0` checks the
unoptimized build.

## The tests

**Foundation**

| test | what it holds the engine to |
|---|---|
| `test_math` | Box3D's test_math.c: the deterministic atan2, cosine and sine against libm within their stated accuracy; vectors, quaternions, transforms and matrices |
| `test_basics` | Box3D's test_bitset.c, test_id.c, test_table.c, test_container.c and test_allocator.c; the name cache with Box3D's own hash values |
| `test_hash` | Box3D's test_hash.c's properties on the engine's content hash |
| `test_parallel` | tasks run as the worker they were given; `parallel_for` covers every item of a range exactly once, however the blocks fall |
| `test_host_api` | a host's allocator takes every block a world on four workers makes, and gets each back; the log, the counters, the version |

**Geometry**

| test | what it holds the engine to |
|---|---|
| `test_dynamic_tree` | Box3D's test_dynamic_tree.c: queries against a linear scan, refit and rebuild, save and load, ten corruptions refused, centimetre boxes at 10,000 km |
| `test_hull` | Box3D's test_hull.c: analytic cubes and tetrahedra, the vertex limit, redundant input, the stress builds |
| `test_distance` | Box3D's test_distance.c, plus spheres and capsules at analytic distances and warm against cold caches |
| `test_manifold` | Box3D's test_sat.c (the separating axis against a brute-force oracle over 7,000 random pairs) and test_manifold.c's hull, sphere and capsule parts |
| `test_triangle_manifold` | the triangle parts of Box3D's test_manifold.c |
| `test_mesh` | Box3D's test_mesh.c: strided, welded and clockwise meshes; rays and casts against analytic hits |
| `test_height_field` | Box3D's test_height_field.c, casts against a brute force over a wave, its files read and written in Box3D's own format |
| `test_compound` | Box3D's test_compound.c, the byte round trip, ten corruptions refused |
| `test_shape` | Box3D's test_shape.c: masses, bounds, rays, the far-origin precision |
| `test_collision` | Box3D's test_collision.c: box validity, a ray against a box in every case, fat bounds at 10,000 km |
| `test_mover` | Box3D's test_mover.c: the plane solver and the shapes' mover collisions |

**Collision and the world**

| test | what it holds the engine to |
|---|---|
| `test_broad_phase` | the pairs found against a brute force through every kind of proxy change |
| `test_mesh_contact` | the four-point cull, the triangle cache, spheres across seams, height fields |
| `test_dynamics` | the step-free parts of Box3D's test_body.c, test_world.c and test_joint.c: bodies, shapes, contacts, islands, sleep, joints, sensors |
| `test_sensor_overlaps` | 80 sensors against 240 visitors of every kind, against brute force |
| `test_world_queries` | 300 bodies of every kind; 3,000 rays, overlaps and casts against brute force |
| `test_threads` | four threads querying meshes, height fields, compounds and hulls at once, every answer the serial one |
| `test_mover_world` | the world parts of Box3D's test_mover.c |

**Solving**

| test | what it holds the engine to |
|---|---|
| `test_contact_solver` | a cube on a slab, the solver's passes run by hand with Box3D's numbers |
| `test_contact_solver_wide` | one scene solved one contact at a time and eight at a time lands in the same place |
| `test_joint_solver` | every kind's accessors as Box3D's test_joint.c uses them, every accessor asked of the wrong kind or a stale id, each kind solved by hand |
| `test_joint_frames` | five joint kinds with their frames turned every way on both bodies, 60 of each, every one held |
| `test_solver` | free fall against the closed form, sleeping, a stack, a bounce, a pendulum, a fast sphere stopped by a thin wall, a bullet, two worlds alike |

**Whole scenes**

| test | what it holds the engine to |
|---|---|
| `test_physics_world` | Box3D's test_world.c and test_body_query.c whole: HelloWorld, events, the explosion near and ten million metres away, the hull database |
| `test_determinism` | Box3D's determinism scenes (a wave pile, a query spawn, a mesh drop, falling ragdolls, a rolling mix), each run twice, on four workers and on a host's scheduler, at both SIMD widths, compared bit for bit |
| `test_large_world` | Box3D's test_large_world.c: a stack, a bullet and the queries at x = 0 and x = 10,000 km agree |
| `test_restitution` | Box3D's test_restitution.c whole; its numbers are Box3D's own to every printed digit |
| `test_edge_cases` | every number a world takes refused when it is NaN, infinite or out of range, the world left as it was; degenerate geometry; heavy chains, perfect bounces, free spins |
| `test_soak` | seeded piles of every convex kind thrown into a pen for fifteen seconds: no energy made, nothing through a wall, everything asleep in the end |
| `test_human` | the ragdoll's shape, its fall to rest in one piece, its joint ranges measured through the joints, the pose held on anchors |
| `test_snapshot` | a world of every kind of thing saved at four moments and stepped 150 steps beside its original, the same hash every step; every corruption refused |
| `test_recording` | Box3D's test_recording.c: every one of 160 calls recorded and replayed, seeking, query replay, names, file round trips, every corruption caught |
| `test_debug_draw` | every shape drawn once in its colour, every option counted against the world, the host's drawables made and released |

## The determinism trace

```sh
AEPHYSICS_TRACE=1 target/test_determinism   # prints every scene's checksum after every step
scripts/golden_trace.sh check               # diffs that against tests/golden/determinism_trace.txt
scripts/golden_trace.sh write               # regenerates it
```

The trace is 1,250 lines, one per step of the five determinism scenes,
each a checksum of every body's state with its exact bits, and every
platform CI runs gives the same file. A change that alters
results on purpose regenerates the file and says why in its commit; a
change that alters it by accident is a bug.

## CI

`.github/workflows/test.yaml` runs on every pull request and every push to
`main`:

| job | what it runs |
|---|---|
| Linux | `scripts/test.sh`, then the determinism, wide solver, soak and solver tests again with `-mavx2 -mfma` and the trace checked |
| macOS arm64 | `scripts/test.sh`, where the C compiler would fuse multiply-adds without `-ffp-contract=off` |
| Windows x86-64 | `scripts/test.sh` under MSYS2 UCRT64, then `test_determinism` at `-O0` with the trace checked |

CI installs one Aether release, pinned in the workflow's `env`. Moving the
pin is a pull request of its own, green on every job.

## Benchmarks

The references are fetched and built first, once:

```sh
scripts/fetch_references.sh   # Box3D at the pinned commit and Jolt, into the ignored reference/
```

Then:

```sh
scripts/bench.sh tree           # one layer: bench/tree.ae against bench/tree_box3d.c
scripts/bench.sh suite          # Box3D's benchmark suite: its scenes against ours
scripts/bake.sh 300             # the bake-off: Box3D and Jolt on identical scenes
```

Every pair runs the same scene through the same code path in both engines,
one thread unless stated, and prints the checksums that show the two did
the same work. `bench/physics_world.ae` reads `AEPHYSICS_SIMD_WIDTH` (4 or
8) to time one width. Results go in [bench/RESULTS.md](../bench/RESULTS.md)
with the date, the machine and the build, ratios read within one session:
the machine's speed wanders between sessions, the ratios do not.

## Profiling

On Windows, where gprof and perf are not available under MinGW:

```sh
scripts/profile.sh bench/physics_world.ae 30   # the 30 hottest functions
```

It links the program with `tools/sampler.c`, a thread that records the main
thread's instruction pointer every half millisecond, and ranks the samples
by symbol with `tools/rank.ae`. Unlike an instrumented profiler it costs
the small maths nothing, so the picture is not distorted. On Linux,
`ae build --profile` and perf do the same job.
