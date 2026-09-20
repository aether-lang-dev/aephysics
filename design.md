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
6. **manifold** (done): manifold.c and convex_manifold.c as
   `aephysics.manifold`, the reference's scalar SAT path, module scratch
   for the clip buffers (per-worker in the parallel layer). 43,090 checks:
   test_sat.c's oracle over 7,000 random pairs and test_manifold.c's hull,
   sphere and capsule parts. The same manifolds as the reference; warm
   cache at parity, cold SAT 1.6x. Found aether#2119 (array literals of
   float expressions typed as int) on the way.
7. **triangle_manifold** (done): triangle_manifold.c as
   `aephysics.triangle_manifold`, with closest_point_on_triangle and the
   triangle features. 1,540 checks: the triangle parts of test_manifold.c
   (the tipped cube's edge on a tilted triangle edge, the parallel pair
   through the threshold, the edge sweep against the cross product, the
   capsule across an edge and straddling the face) plus the sphere on
   every feature, the resting cube, the back-side hysteresis and the
   cache. The same manifolds as the reference within 10% on hulls.
   aetherc does not resolve a module constant inside a struct literal in
   a return statement; the constants are restated locally.
8. **mesh** (done): mesh.c as `aephysics.mesh`: the BVH (binned SAH or
   median split, triangles sorted depth-first), welding through a spatial
   hash on core's new LongMap, the edge flags, and the overlap, ray,
   shape cast, mover and box query traversals at any scale. 1,594
   checks: test_mesh.c's valley (dense, strided, welded, clockwise,
   composed) and creators, plus the tree's consistency, rays against a
   grid and a wave at analytic hits, the box query against a scan, the
   overlap, the shape cast, the flags of a box and a hollow box, a
   mirrored scale, the mover. The same trees as the reference; traversals
   1.7-2x (its SIMD box tests). The mesh contact's cluster reduction
   (mesh_contact.c) is dynamics-side and comes with the contacts.
9. **height_field** (done): height_field.c as `aephysics.height_field`:
   the heights quantised to a global range, materials and holes per
   cell, the edge flags against the four neighbours, either winding;
   overlap, the ray and shape casts by a DDA walk of the swept box's
   leading corner through the cells, the mover's planes, the box query.
   113 checks: test_height_field.c's create, index mapping, winding,
   flat ray, overlap, straddle, brute-force shape and ray casts over a
   wave, back-side and clockwise culling (the file roundtrip is not
   ported), plus the flags of a ridge and of holes, a scaled field, the
   query and the mover. The same results as the reference; the query at
   parity, the casts 1.4-2x. The heights, materials and flags are ints
   for want of 16- and 8-bit arrays, 3x the reference's bytes.
10. **shape** (done): sphere.c, capsule.c and the geometric half of
   shape.c as `aephysics.shape`: the sphere's and capsule's mass, bounds,
   ray casts (the closest-point forms that hold their precision far from
   the origin), casts, overlap and mover planes, the hull's mover, and
   the Shape of any kind with its dispatch under a transform (bounds,
   swept and fat bounds, centroid, areas, mass, extent, ray and shape
   casts, overlap, mover, proxy) and the collision filters. 440 checks:
   test_shape.c's masses (sphere, analytic and transformed boxes, the
   capsule bracketed by hulls), bounds, the sphere and capsule ray cases,
   the overlap convention, the far-origin precision (our doubles sit
   three orders under the reference's float floor and still hit at ten
   million units), the cast through the dispatch; plus every kind under
   one transform through every query, and the filters. Rays and masses
   at parity; the GJK cast and overlap on proxies with radii 2-2.5x,
   to profile (aephysics#9). The world-bound half of shape.c (creation
   on a body, the broad-phase proxy, materials, events) comes with the
   dynamics.
11. **compound** (done): compound.c as `aephysics.compound`: the
   children's bounds into a tree rebuilt in full and carried in the
   block with its traversal stack, materials deduplicated field by field
   through core's LongMap, hulls and meshes by their own hash and bytes,
   a mesh child's four material slots remapped; overlap, ray and shape
   casts, the box query and the mover through the tree, each child in
   its own frame. 159 checks: test_compound.c's creation, materials,
   sharing, child order, bounds, casts, remap, overlap, query and mover
   subtests (the byte roundtrip is not ported), plus the compound
   through the shape dispatch and a field of a thousand spheres. The
   same results as the reference; the build 0.7x, the queries 1.3-1.6x,
   the ray cast 2.9x with the time in the tree's traversal itself
   (aephysics#11). To make the dispatch reach the compound without a
   cycle, sphere.c and capsule.c became `aephysics.sphere` and
   `aephysics.capsule`, the material `aephysics.material`, and the
   hull's mover moved beside the mesh's; `aephysics.shape` imports them
   all. `tree_validate` accepts a baked tree (no parents).
12. **mover** (done): mover.c as `aephysics.mover`: the character
   mover's plane solver (twenty Gauss-Seidel sweeps with the pushes
   accumulated and clamped to each plane's limit, the slop keeping the
   mover just off the surface) and the velocity clip. 56 checks:
   test_mover.c's solver cases and every mover collision that needs no
   world (sphere, capsule, hull, mesh, height field), plus a corner, a
   soft plane and the clip. 0.9x the reference on a million solves with
   one iteration's difference in 1.7 million.
13. **dynamics**: the reference's world is one mutually recursive body
   of C (body.c, contact.c, joint.c, island.c, solver_set.c,
   constraint_graph.c, sensor.c, broad_phase.c and half of
   physics_world.c call into each other), so the cut into Aether's
   acyclic modules is:
   - `aephysics.mesh_contact` (done): mesh_contact.c's world-free
     part: the triangle cache refreshed when the shape leaves its query
     bounds (each triangle keeping a simplex and a separating-axis
     cache; the reference's union is two fields), the manifold per
     triangle in the convex shape's frame, the acceptance rules against
     ghost collisions (a triangle face always; a hull face when aligned
     or deep; the rest tentative, spheres by nearest-first feature
     ownership, hulls and capsules skipping only owned flat edges),
     clusters within cos 5 degrees, the four-point cull. The results
     live in the caller's arena. What follows in the reference (the
     warm-start matching, materials, rolling resistance, tangent
     velocity) is the contact's and comes with the dynamics. 64 checks
     of our own (the reference tests this through its world): the cull,
     a box on a grid, the cache kept and refreshed, a sphere on seams, a
     capsule, a height field, a ridge, a moved mesh. Two reference
     quirks kept: the cull's tie rule lets zero-area points through on
     their separation (collinear points can keep four), and a hull
     face's far clip points stay as speculative ones.
   - `aephysics.broad_phase` (done): broad_phase.c's trees per body
     type, proxies keyed by type in the low bits, the moved-sibling
     gathering, the self and cross pair walks and the pair set; the pair
     filter and the compound lookups are visitors, and the update leaves
     sorted keys for the client to turn into contacts. 36 checks against
     a brute force: pairs found and not found across moves, the filter,
     a forced static proxy, a destroyed proxy, a compound's children.
     Found and fixed on the way: core's key_hash multiplied signed
     longs (undefined in C; gcc at -O2 made two inlined copies disagree)
     -- it now mixes in 32-bit products.
   - `aephysics.dynamics`: one module for the world's state and its
     bookkeeping -- the World with its arrays (bodies, shapes, contacts,
     joints, islands, solver sets), the ids with generations, the
     body's sims and states, the shape's world half (creation on a
     body, the fat bounds, the proxy, materials, events flags), the
     contact (creation from a pair, the manifold update through the
     manifold functions and the mesh contact), the joints' creation and
     their bases, the constraint graph colouring, the solver sets
     (awake, static, disabled, sleeping islands), the islands (union by
     links, split on wake), the sensors. No stepping. Own tests: the
     world's bookkeeping without a step (bodies and shapes created and
     destroyed, contacts begun from pairs, islands linked and split,
     sets moved on sleep and wake), and the parts of test_body.c and
     test_world.c that need no step (mass data, extents, validity,
     recycling).
   - `aephysics.contact_solver`: contact_solver.c scalar (the prepare,
     warm start, solve, restitution and store passes; the wide SIMD path
     later, measured).
   - `aephysics.joint_solver`: the seven joints' prepare, warm start,
     solve and reaction (distance, motor, prismatic, revolute, spherical,
     weld, wheel) with joint.c's dispatch.
   - `aephysics.solver`: solver.c's stages (the Soft Step: integrate
     velocities, warm start, solve, integrate positions, relax,
     restitution, store impulses, per graph colour), continuous
     collision, the sleep decision, the enlarged bounds and the broad
     phase update. Single-threaded first; the stage/block structure kept
     so the parallel layer only adds workers.
   - `aephysics.physics_world`: the step (collide, solve, events), the
     world queries (overlap, casts, the mover's planes and time of
     impact through the broad phase), the events, the public setters.
     Tests: test_body.c, test_joint.c, test_world.c, test_body_query.c,
     the world parts of test_mover.c, test_determinism.c,
     test_large_world.c; bench pairs on the reference's benchmark
     scenes as each becomes possible.
14. **parallel**: `parallel_for` and the scheduler over Aether's actors;
   the benchmarks by thread count as the original records them.
15. **recording and replay**, `world_snapshot`: last, since they are the
   tooling and not the engine.
16. **benchmarks**: `reference/benchmark/main.c`'s nine scenes ported, run
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
