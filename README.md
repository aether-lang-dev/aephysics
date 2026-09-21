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
| `aephysics.base` | bit set, id pool, hash set, a long-to-int map, arrays, the stack and arena allocators, the block hash (the reference's core.c, named so a host's own `core` module and this one can share a program) | done, `test_base.ae` (100k checks) |
| `aephysics.dynamic_tree` | the bounding volume hierarchy under the broad phase: SAH insertion, rotations, enlarge, sweep refit, partial rebuild in depth-first order, box / closest / ray / swept-box queries | done, `test_dynamic_tree.ae` (12k checks); [same tree as the reference, ray cast 1.9x its time](bench/RESULTS.md#dynamic_tree) |
| `aephysics.hull` | quickhull with face merging, the half-edge hull with its mass properties, box / cylinder / cone / rock hulls, clone-and-transform with mirroring, support functions, ray cast, the 2D hull | done, `test_hull.ae` (438 checks); [same hulls as the reference, 1.6-2x its time](bench/RESULTS.md#hull) |
| `aephysics.distance` | GJK with the warm-started simplex cache, the shape cast by conservative advancement, the time of impact by separating-axis root finding | done, `test_distance.ae` (1.1k checks); [same results as the reference, 1.3-1.5x its time](bench/RESULTS.md#distance) |
| `aephysics.manifold` | contact manifolds for sphere, capsule and hull in every pairing: the separating axis test with its cache, reference-face clipping, the feature pairs, reduction to four points | done, `test_manifold.ae` (43k checks, 7,000 pairs against a brute-force oracle); [same manifolds as the reference, warm cache at parity](bench/RESULTS.md#manifold) |
| `aephysics.triangle_manifold` | one mesh triangle against a sphere, capsule or hull: back-side cull with hysteresis, GJK shallow, the separating axis test deep with the triangle's edges as zero-area faces, the feature recorded for the mesh contact's ghost-collision reduction | done, `test_triangle_manifold.ae` (1.5k checks); [same manifolds as the reference, within 10% on hulls](bench/RESULTS.md#triangle_manifold) |
| `aephysics.mesh` | the triangle mesh: a BVH by binned SAH or median split with the triangles in depth-first order, vertex welding, edge flags, any scale including mirrored; overlap, ray cast, shape cast, the mover's planes, a box query | done, `test_mesh.ae` (1.6k checks); [same trees as the reference, traversals 1.7-2x](bench/RESULTS.md#mesh) |
| `aephysics.height_field` | the height field: quantised heights on a fixed diagonal, materials and holes per cell, edge flags per triangle, either winding; overlap, ray and shape casts by a walk along the grid, the mover's planes, a box query | done, `test_height_field.ae` (113 checks, casts against a brute force over a wave); [same results as the reference, query at parity, casts 1.4-2x](bench/RESULTS.md#height_field) |
| `aephysics.material` | a surface's material (friction, restitution, rolling resistance, tangent velocity, user id) and its default | done |
| `aephysics.sphere` | the sphere: mass, bounds, the ray cast in the closest-point form that keeps its precision far from the origin, the shape cast, overlap, the mover's plane | done, in `test_shape.ae` |
| `aephysics.capsule` | the capsule: mass by cylinder and caps, bounds, the ray cast by the closest points of the lines with the near-parallel fallback, the shape cast, overlap, the mover's plane | done, in `test_shape.ae` |
| `aephysics.compound` | the baked compound: capsules, hulls, meshes and spheres in one block under a static tree, hulls and meshes shared by content, materials by value; bounds, overlap, ray and shape casts, the box query, the mover's planes | done, `test_compound.ae` (159 checks); [same results as the reference, build 0.7x, queries 1.3-1.6x](bench/RESULTS.md#compound) |
| `aephysics.shape` | the shape of any kind (sphere, capsule, hull, mesh, height field, compound) with the dispatch over every kind under a transform: bounds, swept and fat bounds, centroid, areas, mass, extent, ray and shape casts, overlap, the mover's planes, the proxy; the collision filters | done, `test_shape.ae` (440 checks); [same results as the reference, rays and masses at parity, radius casts 2.5x](bench/RESULTS.md#shape) |
| `aephysics.mover` | the character mover's plane solver: pushes accumulated and clamped over twenty sweeps, the velocity clip | done, `test_mover.ae` (56 checks); [same results as the reference, 0.9x its time](bench/RESULTS.md#mover) |
| `aephysics.broad_phase` | the broad phase: a tree per body type, proxies keyed by type, the pair update through the moved siblings and cross-tree seeds with the filter and compound lookups as visitors, the pair set, the keys sorted | done, `test_broad_phase.ae` (36 checks against a brute force); [10,000 moving boxes at 4.6 ms a step](bench/RESULTS.md#broad_phase) |
| `aephysics.mesh_contact` | a convex shape against a mesh or height field: the triangle cache with per-triangle warm starts, the manifolds per triangle, the seam rules against ghost collisions, clusters by normal, the four-point cull | done, `test_mesh_contact.ae` (64 checks); [a box on a wave at 3.2 us a step](bench/RESULTS.md#mesh_contact) |
| `aephysics.dynamics` | the world's state and bookkeeping: worlds, bodies (creation, mass from shapes or by hand, transforms, velocities, extents, locks, sleep and enable flags, the type changed, disabled and enabled) in solver sets, shapes of every kind on bodies with their broad-phase proxies, islands linked by contacts and joints and split by union-find, sleeping sets, waking and merging, the constraint graph's colouring, contacts from the broad phase's pairs with the narrow phase (convex, mesh, height field, compound children) and the state changes with their events, contact recycling, joints of every kind (definitions, creation with the edges, sets and islands, collide-connected in the pair filter, destruction, separations, reactions), sensors with their begin and end events | `test_dynamics.ae` (541 checks: test_body.c, the step-free test_world.c and test_joint.c, contacts, islands, sleep, joints, sensors); [creation at parity, collide 1.7x, joints 0.5x, sensors 1.2x](bench/RESULTS.md#dynamics-bookkeeping-contacts-joints-sensors) |
| `aephysics.contact_solver` | the contact constraints of a step, scalar: prepared from the manifolds (anchors, base separations, normal masses, the friction centre and its tangent mass, twist and rolling masses), warm started, solved per sub-step (soft normal constraints with speculative bias, then central friction, twist friction and rolling resistance in the relax pass), the restitution pass, the impulses stored with the hit events; the step context | `test_contact_solver.ae` (49 checks: the passes by hand on a cube on a slab) | [ours alone](bench/RESULTS.md#contact_solver) |
| `aephysics.contact_solver_wide` | a colour's convex contacts four at a time: the constraint of four contacts in lanes (a structure of arrays over the same prepare, warm start, solve, restitution and store), the bodies gathered into lanes and scattered back; the lanes in Aether, and the same lanes in `lanes.c` as vector code in single precision (the reference's SIMD path), on by default | `test_contact_solver_wide.ae` (19 checks: one scene three ways, the stack, the bounce with hit events, rolling resistance, a conveyor, a slide, agreeing to 1e-9 and 4e-5; the native lanes' layout checked, the same path twice bit for bit) | [scalar 252 ms, lanes in Aether 230, native lanes 187, the reference 157](bench/RESULTS.md#solver) |
| `aephysics.joint_solver` | the seven joints solved: each kind's prepare (frames relative to the centres of mass, effective masses, spring softness), warm start and solve (rigid or soft constraints, speculative limits, springs, motors), the kinds' accessors (limits, springs, motors, current angles and translations, forces and torques), joint.c's dispatch with the constraint hertz clamp | `test_joint_solver.ae` (149 checks: test_joint.c's accessors on every kind, each kind solved by hand) | [ours alone](bench/RESULTS.md#joint_solver) |
| `aephysics.solver` | the Soft Step: the constraints prepared, per sub-step the velocities integrated (gravity, damping, the gyroscopic torque), warm start, solve, positions, relax, colour by colour with the overflow first; restitution, the impulses stored; the bodies finalised (sleep velocities, move events, fast bodies swept for the time of impact, bounds and proxies), the joint and hit events, the trees refit, bullets, the sensors' hits, islands put to sleep | `test_solver.ae` (42 checks: free fall against the closed form, resting and sleeping, a stack, a bounce and its hit event, a joint event, a pendulum, a fast sphere stopped by a thin wall, a bullet, a sensor swept, two worlds bit for bit alike) | [1.2-1.4x the reference's step with the native lanes](bench/RESULTS.md#solver) |
| `aephysics.physics_world` | the world's face: the step (pairs, narrow phase, solve, sensors, events), the events read back, the settings and counters, queries over every shape (overlap of a box or a proxy, the mover's planes, ray, shape and mover casts, the closest ray hit) and against one body at a transform of the caller's, explosions | `test_physics_world.ae` (94 checks: test_world.c's HelloWorld, contact, hit, move and sensor events, the explosion near and far; test_body_query.c's casts, overlaps, mover planes and time of impact; the world queries; a wave pile stepping alike twice) | [the reference's own benchmark scenes with the same checksums at 1.2-1.6x](bench/RESULTS.md#physics_world) |
| `aephysics.human` | the ragdoll: twelve capsule bones on spherical joints with cone and twist limits and revolute joints with angle limits, a spring on every joint toward the reference pose, a motor whose torque limit is joint friction, filter joints for the limbs that clash; the align spring, kinematic anchors through motor or parallel joints (the pose drive of an active ragdoll), velocity, kicks, bullets | `test_human.ae` (46 checks: the figure's shape, a fall to rest in one piece, the setters, a kick, standing under the align spring, the pose held on anchors, the same drop twice bit for bit) | [the reference's rain benchmark, 300 ragdolls over 400 steps, at 1.05x](bench/RESULTS.md#human) |
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
scripts/profile.sh file.ae [top] # a sampling profile on Windows (tools/sampler.c + tools/rank.ae)
```

Aether keeps struct names in one namespace across modules, so a host
engine and this library must agree on every name they both use. The
math types are shared on purpose: `Vec2` and `Vec3` as `{x, y, z}`,
`Quat` as `{x, y, z, w}` (the maths reads it through `math.qv` and
`math.quat_vs`, the reference's `{v, s}`), the same definitions ae3d's
core has, so a transform crosses between the two without conversion.
The rest (`World`, `Plane`, `Buffer`, `HumanBone`, ...) is this
library's; a host names its own types for what they are (ae3d's voxel
world, frustum plane and glTF buffer are `VoxelWorld`, `FrustumPlane`
and `GltfBuffer`).

`aether.toml` gives `ae build` the flags the benchmarks are measured
with (`-O3` and a wider inlining budget, so the small maths inline as
the reference's `static inline` headers do; [why](bench/RESULTS.md#build-flags-inlining)).
The contact lanes' vector code is C beside its module, so a build adds
it: `ae build app.ae --extra aephysics/contact_solver_wide/lanes.c`
(or an `extra_sources` entry in the project's `aether.toml`); the
scripts do. `contact_solver_wide.set_native_lanes(false)` runs the
same lanes in Aether instead, and `solver.set_wide_contacts(false)`
the scalar solve ([what each buys](bench/RESULTS.md#solver)).

Beyond the modules' own tests, the reference's scene tests run on the
world as a whole: `test_mover_world.ae` (the mover through a world:
which material a plane came from, for meshes, compounds and convex
shapes; 38 checks), `test_determinism.ae` (the reference's wave pile,
query spawn and mesh drop with its own random numbers, and the
falling ragdolls, each run to sleep twice and compared bit for bit; the
query spawn sleeps a step after the reference's with its 59 query hits,
the mesh drop two; 11 checks) and `test_large_world.ae` (a stack, a bullet and the
origin-relative queries at x = 0 and at x = 1e7 agree, the whole
engine being in doubles; 43 checks).

The step is deterministic across platforms, not only across runs:
`AEPHYSICS_TRACE=1 target/test_determinism` prints every body's
checksum after every step with its bits, CI records the Linux trace in
its log, and the Windows trace (MinGW, a different gcc) matches it bit
for bit on every step of the four scenes. The engine's own maths
(the reference's cos, sin and atan2 approximations, `sqrt`, `floor`
and `remainder` as the only libm calls, no fused multiply-adds in the
lanes) is what makes that hold; the one divergence seen so far was an
older `aetherc` folding `1.0 / 60.0` to ten digits (issue #27).

## Where it is going

[`design.md`](design.md): the order of the layers, what each is measured
by, and the research this is the ground for -- active ragdolls and
procedural animation in the NaturalMotion/Euphoria line, for the engine
that consumes this.

## Credits

The design and the tests followed are Box3D's, by Erin Catto (MIT), and
the second baseline is Jolt, by Jorrit Rouwe (MIT). aephysics is by
Nicolas Maman, MIT.
