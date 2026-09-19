# The engine: order, measures, and where it leads

## Why an implementation and not a binding

An Aether program could call a C library through externs, the way ae3d
calls its own native layer. The brief is the other thing: the engine
written in Aether, so the language's programs and the agents that write
them read, test and change the physics in the language, and so the library
stands on its own (`ae build --emit=lib`). The reference engines are
fetched, unmodified, beside it as the baseline: the engine is right when it
agrees with them and fast when it keeps up with them. Which design to
follow was decided by measurement, not by reputation: `bench/RESULTS.md`.

## Order

Each layer lands as its own pull request on `main`, with the reference's
tests for that layer written here beside it, and nothing above it is
started until its tests pass.

1. **math** (`math_functions`): done. 6 million checks including a million
   direction pairs, the deterministic atan2/cos/sin within the original's
   stated accuracy against libm.
2. **core**: done. `bitset` over 64-bit longs, `id_pool`, `table` (the
   hash set; the murmur finaliser with logical shifts written out),
   `container` as `IntArray` and a stride-typed `Buffer` (Aether has no
   generics: an array of structs is a block read through a typed cast),
   the stack and arena allocators with the original's counting. 100,000
   checks: the Fibonacci bit set, 50,000 pair keys filled, thinned,
   searched and emptied, every allocation freed.
3. **dynamic_tree** (done): the tree as its own module, Box3D's
   dynamic_tree.c with the traversal stacks in the tree and validation
   returning false where the reference asserts. 12,772 checks against
   test_dynamic_tree.c; the same tree as the reference on the benchmark
   scene (same hits, height, area ratio), insert faster, ray cast 1.9x.
   Save/load not ported; the atomic moved-marking waits for the parallel
   layer.
4. **hull** (done): quickhull as `aephysics.hull`, the builder's
   pointers as indices with the intrusive lists chained through the
   pools and their sentinels in extra slots, int half-edge indices, no
   SOA mirrors, the box hull a heap block. 438 checks from test_hull.c
   at the reference's tolerances; the same hulls on the benchmark
   scenes at 1.6-2x its time.
5. **distance** (done): GJK, the shape cast and the time of impact as
   `aephysics.distance`, the simplex's vertices as named fields and the
   cache's index pairs as named ints. 1,143 checks: the reference's four
   plus spheres and capsules at analytic distances, warm against cold
   caches over a sweep of poses, witness points inside their shapes and
   no axis separating more than the distance, rotating sweeps in the time
   of impact, the hull's overlap and cast through it. The same results as
   the reference at 1.3-1.5x its time.
6. **collision, static**: `manifold` and `convex_manifold`
   (sphere/capsule/hull contact manifolds), `triangle_manifold`, `mesh`,
   `height_field`, `shape` (mass properties, ray and shape casts per
   shape). Tests: `test_collision`, `test_manifold`, `test_sat`,
   `test_shape`, `test_mesh`, `test_height_field`.
7. **dynamics**: `body`, `contact`, `constraint_graph` (graph colouring),
   `solver_set`, `island`, `solver` (the Soft Step: sub-stepping, relax
   iterations, restitution), `contact_solver` (scalar first; the wide SIMD
   path second, measured), the joints (revolute, prismatic, distance,
   motor, weld, wheel, spherical), `sensor`, `mover` (the character mover),
   `physics_world`. Tests: `test_body`, `test_joint`, `test_world`,
   `test_mover`, `test_determinism`, `test_large_world`.
8. **parallel**: `parallel_for` and the scheduler over Aether's actors;
   the benchmarks by thread count as the original records them.
9. **recording and replay**, `world_snapshot`: last, since they are the
   tooling and not the engine.
10. **benchmarks**: `reference/benchmark/main.c`'s nine scenes ported, run
   against the C build on the same machine, recorded under `benchmark/`.

## Measures

- Every ported test passes with the original's tolerances.
- Determinism: `test_determinism` (the same scene twice, bit-identical),
  and the port against the C original on a scene's first frames.
- Speed: each benchmark scene, port against C, single thread first. The
  honest expectation for a scalar double port against a wide-SIMD float
  original is a gap; the work after the port is closing it -- and where
  the gap is the SIMD contact solver, that one loop goes native behind
  the same interface, measured.

## The research this is the ground for

The brief asked after the physics and ragdoll work behind Grand Theft
Auto. That is **NaturalMotion's Euphoria** (used in GTA IV, GTA V, Red Dead
Redemption, Max Payne 3), which Rockstar integrated into RAGE alongside
Bullet for collision. Euphoria is not a physics engine: it is an *active
ragdoll* -- a full rigid-body character driven by controllers ("behaviours")
that model balance, protective reflexes, grabbing, staggering -- what
NaturalMotion called Dynamic Motion Synthesis. The founders' research:

- Torsten Reil and Phil Husbands, "Evolution of Central Pattern Generators
  for Bipedal Walking in a Real-Time Physics Environment", IEEE
  Transactions on Evolutionary Computation, 2002 -- neural oscillators
  evolved to drive a physically simulated biped's joints, which is the
  origin of NaturalMotion (Reil and Colm Massey, Oxford, 2001).
- The Euphoria behaviours themselves are unpublished; what is published
  around them: Yin, Loken, van de Panne, "SIMBICON: Simple Biped Locomotion
  Control" (SIGGRAPH 2007) for balance feedback on a simulated biped;
  Geijtenbeek and Pronost, "Interactive Character Animation Using Simulated
  Physics" (2012 survey); and for the modern procedural locomotion that GTA
  VI is reported to build on, Clavet's "Motion Matching and The Road to
  Next-Gen Animation" (GDC 2016) and Holden et al., "Phase-Functioned
  Neural Networks for Character Control" (SIGGRAPH 2017) and "Learned
  Motion Matching" (SIGGRAPH 2020).

So the question's answer: it is related, and it sits *on top of* this
port. What an active ragdoll needs from the physics is exactly Box3D's
dynamics -- articulated bodies through joints with motors and limits
(revolute, spherical with cone and twist limits, the wheel/motor joints
for drive), joint-space PD control toward a target pose, and a solver that
holds a chain of a dozen bodies stably under those motors, which is what
Soft Step is for. The plan in ae3d, once this port stands:

1. **Ragdoll from the skeleton**: a body per bone with capsule shapes and
   mass from the mesh, joints from the bind pose with limits, a
   `RigidBody`/`Ragdoll` component in the engine's GameObject shape.
2. **Animation-driven ragdoll**: every joint's motor tracks the animation
   clip's pose (PD gains per joint), so a walking zombie is simulated and
   a hit knocks it off the clip and back -- the base of Euphoria's look.
3. **Behaviours**: balance (a SIMBICON-style feedback on the hip and
   stance foot), protective arms toward the ground on a fall, a stagger
   that steps to recover -- each a controller over the ragdoll's motors,
   the way Euphoria composes them.
4. **Locomotion**: motion matching over the clips the pipeline exports,
   feeding the animation-driven ragdoll its target pose.

Each is its own epic in ae3d (nicolas-maman/ae3d#365); none of them can
start before the dynamics layer here passes its tests.
