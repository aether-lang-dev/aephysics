# The bake-off

The reference engines on identical scenes, on one machine, one thread, each
at its own recommended settings for a game's 60 Hz step. What decided
which design this engine follows.

**Machine:** Intel Core i7-13700K, Windows 11, GCC 14 (msys2 ucrt64),
2026-09-19. The GPU was busy with a game at the time; these are CPU numbers.

**Builds:** Box3D `f555ee4` (Release, SSE2, validation off); Jolt `v5.3.0`
(Distribution: LTO, AVX2/FMA, no profiler, no debug renderer -- its
shipping configuration).

**Settings:** a 1/60 s step. Box3D: 4 sub-steps (its own benchmark
setting). Jolt: 1 collision step, 10 velocity + 2 position iterations (its
defaults). Sleeping off on both, so every body is simulated every step.
Timing excludes the first step (the structures are built there).

| scene | bodies | Box3D ms/step | Jolt ms/step | how it held |
|---|---|---|---|---|
| pyramid, 100 rows of unit boxes | 5,050 | **8.9** | 36.7 | Box3D: worst drift 0.79 m after 300 steps, 0.72 m after 1,000 (the top wobbles, the stack stands). Jolt: 17.5 m after 300, 102 m after 1,000 -- the pyramid collapses. |
| pile, 20 x 25 x 20 unit boxes dropped 2 m | 10,000 | **9.9** | 32.8 | lowest box after 300 steps: Box3D 0.498 m (2 mm into the ground), Jolt 0.473 m (27 mm) |
| chain, a 100 x 100 grid of spheres on point joints, hung from its top row | 10,000 bodies, 19,800 joints | **11.1** | 11.6 | sag of the bottom corner below a taut 99-link chain: Box3D 0.94 m, Jolt 1.97 m |

`scripts/bake.sh 300` reproduces the table; `bench/bake_box3d.c` and
`bench/bake_jolt.cpp` are the scenes.

## Reading it

- On contact-heavy scenes (the pile, the pyramid) Box3D's Soft Step solver
  is three to four times faster than Jolt's sequential impulse solver at
  its defaults, and it is the one that holds a tall stack: sub-stepping
  with soft constraints converges where iterating a single step does not.
  Jolt can hold the pyramid with more iterations, at more cost; the point
  is that Box3D holds it at the speed it already has.
- On the joint grid the two are level in speed and Box3D sags half as much.
- Both scale across threads (Box3D's own records show 7x at 8 threads on a
  7950X; Jolt is known to scale); the port is single-threaded first, so
  the single-thread numbers are the ones that matter for it.

Not measured: PhysX 5 (too heavy to build for a bake-off tick; GPU-first)
and Bullet (an older sequential-impulse design without sub-stepping, the
collision library under GTA IV/V's RAGE -- Euphoria sat on it, but the
solver is not where the field is today). Rapier is Rust and its idioms
would port worst.

## The decision

The engine follows **Box3D's design**: Soft Step (sub-stepped, soft
constraints, relax iterations, restitution pass), graph-coloured
constraint islands, speculative contacts with continuous collision,
island-based sleep, convex hulls / capsules / spheres / triangle meshes /
height fields, a dynamic-tree broad phase, and cross-platform determinism.
Jolt stays as the second baseline in this table.
