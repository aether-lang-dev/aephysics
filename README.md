# aephysics

A rigid-body physics engine written in [Aether](https://github.com/aether-lang-org/aether),
for [ae3d](https://github.com/nicolas-maman/ae3d) and any Aether program.

The brief: the most accurate and most performant rigid-body physics there
is, in the language, not behind a binding. Rather than trust anyone's
README, the candidates were built on one machine and run on identical
scenes -- [`bench/RESULTS.md`](bench/RESULTS.md). By those numbers the
engine follows the design of **Box3D** (Erin Catto, 2026, C17, MIT): its
Soft Step solver held a 100-row pyramid at 10 ms a step where Jolt's
collapsed at 44, dropped ten thousand boxes at 10 ms a step against 43,
and given the same budget as Jolt at its stiffest it was tighter on every
scene.

This is our implementation. Box3D and Jolt are fetched, unmodified, into an
ignored `reference/` (`scripts/fetch_references.sh`) for two things only:
their tests, which every layer here is written against, and the same-machine
benchmarks every layer is measured against. Nothing of theirs is
redistributed here; both are MIT and credited. The naming follows Box3D's
where the function is the same thing, without its prefix, in snake case,
so a test written against the reference reads the same here.

## Modules

| module | holds | state |
|---|---|---|
| `aephysics.math` | vectors, quaternions, transforms, 3x3 matrices, bounding boxes, segment distances, inertia helpers, the deterministic atan2/cos/sin | done, `test_math.ae` (6M checks) |
| `aephysics.core` | bit set, id pool, hash set, arrays, the stack and arena allocators | done, `test_core.ae` (100k checks) |
| `aephysics.dynamic_tree` | the bounding volume hierarchy under the broad phase: SAH insertion, rotations, enlarge, sweep refit, partial rebuild in depth-first order, box / closest / ray / swept-box queries | done, `test_dynamic_tree.ae` (12k checks); [same tree as the reference, ray cast 1.9x its time](bench/RESULTS.md#dynamic_tree) |
| `aephysics.hull` | quickhull with face merging, the half-edge hull with its mass properties, box / cylinder / cone / rock hulls, clone-and-transform with mirroring, support functions, ray cast, the 2D hull | done, `test_hull.ae` (438 checks); [same hulls as the reference, 1.6-2x its time](bench/RESULTS.md#hull) |
| `aephysics.distance` | GJK with the warm-started simplex cache, the shape cast by conservative advancement, the time of impact by separating-axis root finding | done, `test_distance.ae` (1.1k checks); [same results as the reference, 1.3-1.5x its time](bench/RESULTS.md#distance) |
| `aephysics.manifold` | contact manifolds for sphere, capsule and hull in every pairing: the separating axis test with its cache, reference-face clipping, the feature pairs, reduction to four points | done, `test_manifold.ae` (43k checks, 7,000 pairs against a brute-force oracle); [same manifolds as the reference, warm cache at parity](bench/RESULTS.md#manifold) |
| `aephysics.collision` | triangle manifolds, triangle mesh, height field, shapes with mass properties, ray and shape casts | next |
| `aephysics.dynamics` | bodies, contacts, the constraint graph, islands, the Soft Step solver, joints (spherical, revolute, prismatic, distance, motor, weld, wheel), sensors, the character mover, the world | |
| `aephysics` | the public API | |

Deliberate choices:

- **Double precision.** Aether's float is a C double, so everything is
  double; the reference's float/double world-position split collapses into
  one `Vec3` and one `Transform`.
- **Determinism kept.** The approximate atan2, cosine and sine exist for
  cross-platform replay and are kept, folded with the engine's own pi.
- **No SIMD intrinsics.** The wide contact solver is written scalar first
  and measured; where the benchmark says the wide path matters, that loop
  goes native behind the same interface.
- **Threads.** The reference's task scheduler maps onto Aether's actors;
  the single-threaded path comes first and the benchmarks record both.

## Tests and benchmarks

```
scripts/test.sh                 # builds and runs every aephysics/test_*.ae with ae; what CI runs
scripts/fetch_references.sh     # Box3D and Jolt into reference/, built (needs cmake, ninja, gcc)
scripts/bake.sh [steps]         # the bake-off scenes on the references
scripts/bench.sh [layer]        # a layer of aephysics against the same code in the reference
```

## Where it is going

[`design.md`](design.md): the order of the layers, what each is measured
by, and the research this is the ground for -- active ragdolls and
procedural animation in the NaturalMotion/Euphoria line, for the engine
that consumes this.

## Credits

The design and the tests followed are Box3D's, by Erin Catto (MIT), and
the second baseline is Jolt, by Jorrit Rouwe (MIT). aephysics is by
Nicolas Maman, MIT.
