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
| `aephysics.math` | vectors, quaternions, transforms, 3x3 matrices, bounding boxes, segment distances, inertia helpers, the deterministic atan2/cos/sin, and the same vectors in single precision (`Vec3f`, `Quatf`, `Matrix3f`) for the geometry the reference keeps in `float` ([#42](https://github.com/aether-lang-dev/aephysics/issues/42)) | done, `test_math.ae` (6M checks) |
| `aephysics.basics` | bit set, id pool, hash set, a long-to-int map, arrays, the stack and arena allocators, the host's hooks (its allocator takes every block the engine makes, its log the engine's messages, its failure function the one hard stop: b3SetAllocator, b3SetLogFcn, b3SetAssertFcn), the content hash and the reference's djb2 `b3Hash`, the name cache (each body and shape name kept once per world under its 32-bit hash, the reference's name_cache.c, a collision taking the next free id instead of sharing the first name) (the reference's core.c, named so a host's own `core` module and this one can share a program) | done, `test_basics.ae` (100k checks, test_container.c and test_allocator.c among them, test_name_cache.c's CacheUnit with the reference's own hash values); `test_hash.ae` (36 checks: test_hash.c's properties on our hash, every bit and the length reaching it, families of real geometry never colliding); `test_host_api.ae` (31 checks: a host's allocator, handing out dirty memory, takes every block of a world on four workers with a ragdoll, a snapshot and a recording and gets each back; its log takes the memory part by part, adding up to the counters' byte count; the counters, capacity, contact by id, world counts, version, and `b3Hash` against the reference's own values) |
| `aephysics.native` | the Aether face of the one C file built with every program: the threads' helpers Aether has not (a thread-local worker index, yield, pause, the processor count, atomics on an int in place, a semaphore) and the per-worker scratch every module keeps a block of | done |
| `aephysics.parallel` | the reference's scheduler and `parallel_for`: worker threads made once and waiting on a semaphore, tasks in slots claimed by compare-and-swap, the main thread helping while it waits; a range in blocks the tasks claim, the caller working as worker 0 | done, `test_parallel.ae` (18 checks); [the step by worker count against the reference's](bench/RESULTS.md#parallel) |
| `aephysics.dynamic_tree` | the bounding volume hierarchy under the broad phase: SAH insertion, rotations, enlarge, sweep refit, partial rebuild in depth-first order, box / closest / ray / swept-box queries, the tree saved and loaded as bytes (b3DynamicTree_Save and _Load; every corrupt input refused); its boxes in single precision rounded outward from double, as the reference's large-world build keeps them (32-byte nodes; the costs and rays measured on the widened boxes) | done, `test_dynamic_tree.ae` (15k checks: TreeSaveLoadRoundtrip among them, the loaded tree's queries against a scan, a scale, ten corruptions refused, and centimetre boxes at 1e7 each held by its leaf and found by every query that touches it); [same tree as the reference; inserts 0.4x its time, queries 1.1x, rays 0.9x](bench/RESULTS.md#dynamic_tree) |
| `aephysics.hull` | quickhull with face merging, the half-edge hull with its mass properties, box / cylinder / cone / rock hulls and PEEL's convex (the reference's complex hull, the same 32 points), clone-and-transform with mirroring, support functions, ray cast, the 2D hull; every hull carries single-precision rows of its vertices and normals and its edges' cosines, padded to eight lanes, for the wide separating axis test | done, `test_hull.ae` (445 checks); [same hulls as the reference; sphere hulls at parity, merge-heavy cubes 2x, box hulls 3.6x (a heap block against its stack value)](bench/RESULTS.md#hull) |
| `aephysics.distance` | GJK with the warm-started simplex cache, the shape cast by conservative advancement, the time of impact by separating-axis root finding | done, `test_distance.ae` (1.1k checks); [same results as the reference; 1.1-1.2x its time, the time of impact faster](bench/RESULTS.md#distance) |
| `aephysics.manifold` | contact manifolds for sphere, capsule and hull in every pairing: the separating axis test with its cache (support searches eight vertices a pass over the hulls' rows, edge candidates culled by the inscribed spheres and a probe point, edge pairs eight a pass, every winner measured in double), reference-face clipping, the feature pairs, reduction to four points | done, `test_manifold.ae` (43k checks, 7,000 pairs against a brute-force oracle); [same manifolds as the reference, warm cache at parity](bench/RESULTS.md#manifold) |
| `aephysics.triangle_manifold` | one mesh triangle against a sphere, capsule or hull: back-side cull with hysteresis, GJK shallow, the separating axis test deep with the triangle's edges as zero-area faces, the feature recorded for the mesh contact's ghost-collision reduction | done, `test_triangle_manifold.ae` (1.5k checks); [same manifolds as the reference, within 10% on hulls](bench/RESULTS.md#triangle_manifold) |
| `aephysics.mesh` | the triangle mesh: a BVH by binned SAH or median split with the triangles in depth-first order, vertex welding, edge flags, any scale including mirrored; overlap, ray cast, shape cast, the mover's planes, a box query | done, `test_mesh.ae` (1.6k checks); [same trees as the reference; the SAH build at parity, the median build 1.2-1.4x, rays and casts faster, the box query 1.5x](bench/RESULTS.md#mesh) |
| `aephysics.height_field` | the height field: quantised heights on a fixed diagonal, materials and holes per cell, edge flags per triangle, either winding; overlap, ray and shape casts by a walk along the grid, the mover's planes, a box query | done, `test_height_field.ae` (158 checks, casts against a brute force over a wave, the file roundtrip through `aephysics.files`: the definition back byte for byte, the reference's own text with CRLF lines, and every bad file refused); [same results and the same 1.3 MB as the reference; rays at parity, queries and casts faster, builds 1.1x](bench/RESULTS.md#height_field) |
| `aephysics.files` | what the reference keeps on disk: a dynamic tree saved and loaded (b3DynamicTree_Save and _Load) and a height field's definition as text (b3DumpHeightData and b3LoadHeightField) in the reference's own layout, so its files load here and these load there; apart so a program that keeps nothing on disk does not build std.fs | done, through `test_dynamic_tree.ae` and `test_height_field.ae` |
| `aephysics.material` | a surface's material (friction, restitution, rolling resistance, tangent velocity, user id) and its default | done |
| `aephysics.sphere` | the sphere: mass, bounds, the ray cast in the closest-point form that keeps its precision far from the origin, the shape cast, overlap, the mover's plane | done, in `test_shape.ae` |
| `aephysics.capsule` | the capsule: mass by cylinder and caps, bounds, the ray cast by the closest points of the lines with the near-parallel fallback, the shape cast, overlap, the mover's plane | done, in `test_shape.ae` |
| `aephysics.compound` | the baked compound: capsules, hulls, meshes and spheres in one block under a static tree, hulls and meshes shared by content, materials by value; bounds, overlap, ray and shape casts, the box query, the mover's planes; the compound as bytes and back (b3ConvertCompoundToBytes, every section, shared hull and mesh, material and child checked against the bytes) | done, `test_compound.ae` (200 checks: the three serialize tests, the same bytes for the same compound, and ten corruptions refused); [same results as the reference; builds 0.9x, rays 1.1x, box queries 1.3x](bench/RESULTS.md#compound) |
| `aephysics.shape` | the shape of any kind (sphere, capsule, hull, mesh, height field, compound) with the dispatch over every kind under a transform: bounds, swept and fat bounds, centroid, areas, mass, extent, ray and shape casts, overlap, the mover's planes, the proxy; the collision filters | done, `test_shape.ae` (443 checks); [same results as the reference; rays, shape casts and overlaps at or under its time](bench/RESULTS.md#shape) |
| `aephysics.mover` | the character mover's plane solver: pushes accumulated and clamped over twenty sweeps, the velocity clip | done, `test_mover.ae` (56 checks); [same results as the reference, 0.9x its time](bench/RESULTS.md#mover) |
| `aephysics.broad_phase` | the broad phase: a tree per body type, proxies keyed by type, the pair update through the moved siblings and cross-tree seeds with the filter and compound lookups as visitors, candidates checked against the pair set 32 at a time with every slot prefetched first (b3FlushCandidatePairs), the keys sorted | done, `test_broad_phase.ae` (36 checks against a brute force); [10,000 moving boxes at 4.6 ms a step](bench/RESULTS.md#broad_phase) |
| `aephysics.mesh_contact` | a convex shape against a mesh or height field: the triangle cache with per-triangle warm starts, the manifolds per triangle, the seam rules against ghost collisions, clusters by normal, the four-point cull | done, `test_mesh_contact.ae` (64 checks); [a box on a wave at 3.2 us a step](bench/RESULTS.md#mesh_contact) |
| `aephysics.dynamics` | the world's state and bookkeeping: worlds, bodies (creation, mass from shapes or by hand, transforms, velocities, extents, locks, sleep and enable flags, the type changed, disabled and enabled) in solver sets, shapes of every kind on bodies with their broad-phase proxies, the hull database (a hull shape holds the world's copy, equal hulls share one, transformed hulls baked into their own), islands linked by contacts and joints and split by union-find, sleeping sets, waking and merging, the constraint graph's colouring, contacts from the broad phase's pairs with the narrow phase (convex, mesh, height field, compound children) and the state changes with their events, contact recycling, joints of every kind (definitions, creation with the edges, sets and islands, collide-connected in the pair filter, destruction, separations, reactions), sensors with their begin and end events | `test_dynamics.ae` (641 checks: test_body.c, the step-free test_world.c and test_joint.c, test_shape.c's names and flags, contacts, islands, sleep, joints, sensors); [creation 0.9x, the first collide at parity, recycling 1.1-1.2x, joints 0.6x, sensors 0.75-1.0x](bench/RESULTS.md#dynamics-bookkeeping-contacts-joints-sensors) |
| `aephysics.contact_solver` | the contact constraints of a step, scalar: prepared from the manifolds (anchors, base separations, normal masses, the friction centre and its tangent mass, twist and rolling masses), warm started, solved per sub-step (soft normal constraints with speculative bias, then central friction, twist friction and rolling resistance in the relax pass), the restitution pass, the impulses stored with the hit events; the step context | `test_contact_solver.ae` (50 checks: the passes by hand on a cube on a slab) | [ours alone](bench/RESULTS.md#contact_solver) |
| `aephysics.contact_solver_wide` | a colour's convex contacts eight at a time: the constraint of eight contacts as one single-precision record in lanes (`f32x8`, std.lanes -- the reference's SIMD path; one AVX2 register with `AETHER_AEPHYSICS_CFLAGS=-mavx2`, two four-lane halves without), the same prepare, warm start, solve, restitution and store, the bodies gathered into lanes and scattered back | `test_contact_solver_wide.ae` (one scene two ways -- the stack, the bounce with hit events, rolling resistance, a conveyor, a slide -- the lanes within a millimetre of the scalar solve, and the same lanes twice bit for bit) | [the falling grid: scalar 241 ms, lanes 142, the reference 155](bench/RESULTS.md#solver) |
| `aephysics.joint_solver` | the seven joints solved: each kind's prepare (frames relative to the centres of mass, effective masses, spring softness), warm start and solve (rigid or soft constraints, speculative limits, springs, motors), the kinds' accessors (limits, springs, motors, current angles and translations, forces and torques), joint.c's dispatch with the constraint hertz clamp; beyond the reference, a spherical joint's cone may be centred off frame A's z and may be an ellipse of two half-angles (the circle stays the default, bit for bit) | `test_joint_solver.ae` (181 checks: test_joint.c's accessors on every kind, each kind solved by hand) | [ours alone](bench/RESULTS.md#joint_solver) |
| `aephysics.solver` | the Soft Step over the workers, the reference's stages: the work in blocks (the bodies', each colour's joints, wide and scalar constraints) that workers claim by compare-and-swap, the stages published as sync bits by worker 0 -- the constraints prepared, per sub-step the velocities integrated (gravity, damping, the gyroscopic torque), warm start, solve, positions, relax, colour by colour with the overflow first on worker 0; restitution, the impulses stored; the bodies finalised in parallel (sleep velocities, move events, fast bodies swept for the time of impact, bounds and proxies), the joint and hit events, the trees refit, bullets, the sensors' hits, islands put to sleep | `test_solver.ae` (45 checks: free fall against the closed form, resting and sleeping, a stack, a bounce and its hit event, a joint event, a pendulum, a fast sphere stopped by a thin wall, a bullet, a sensor swept, two worlds bit for bit alike) | [1.2-1.3x the reference's step, stage by stage](bench/RESULTS.md#stage-by-stage) |
| `aephysics.physics_world` | the world's face: the step (pairs, narrow phase, solve, sensors, events), the events read back, the settings and counters (the reference's whole `b3Counters`: byte count, tasks, manifold counts, time of impact iterations), the maximum capacity kept at each step's start, kinematic speculation (off by default, #167: a fast kinematic body's contacts reach out over its travel, so what it strikes is not found inside it), the memory part by part to the host's log, the world counts, version and CPU queries, queries over every shape (overlap of a box or a proxy, the mover's planes, ray, shape and mover casts, the closest ray hit) and against one body at a transform of the caller's, explosions | `test_physics_world.ae` (259 checks: test_world.c whole, among it the hull database (equal hulls one world-owned copy, a caller's hull freed at once, transformed hulls baked once, every holder's release), empty and recycled worlds, every setting, the worker count set between steps, contact, hit, move and sensor events, a compound child's hit events and materials, the overflow colour pile, the explosion near and far, the bullet and sleep flags, the compound count, an enlarged proxy destroyed; test_body_query.c whole: casts, overlaps of hull proxies and turned compounds, mover planes, and the mover's time of impact against 2,000 random sweeps; test_broad_phase.c's static rebuild and its stress test, a swarm through every proxy change with the pair set checked by brute force every step; test_id.c; the world queries; a wave pile stepping alike twice) | [the reference's own benchmark scenes with the same checksums: 1.2x its AVX2 build on the large pyramid, 1.15x on the many pyramids, 1.05x on the joint grid, ours at eight lanes in AVX2 with float records (#121)](bench/RESULTS.md#the-collide-pass-as-the-reference-keeps-it-2026-10-09-121); [the rest of its suite, nine scenes and the SAT runs, at 1.04-1.21x](bench/RESULTS.md#suite-the-references-benchmark-suite) |
| `aephysics.debug_draw` | what a host draws a world with (the reference's b3DebugDraw): callbacks for a user shape, segments, transforms, points, spheres, capsules, bounds, boxes and strings, the options, the reference's 145 named colours, material presets and graph colours; `physics_world.world_draw` walks the shapes in the drawing bounds (each coloured by its body's state as the reference picks it, or by its material's own colour) and draws what the options ask of their bodies (names, mass, sleep, each joint kind once, contacts by state or graph colour with normals, forces or features, islands); the world def's debug shape callbacks make the host's drawable for a shape when it is first drawn and give it back when the shape goes, its geometry changes (the reference keeps the stale one) or the world goes (the reference leaves those to the host), and an in-place restore keeps the drawables of the shapes still there; `recording.player_draw_frame_queries` draws a replayed frame's queries, the selected one labelled with its caller | done, `test_debug_draw.ae` (41 checks: every shape once with its colour, a drawable each made once and kept, every option counted against the world (a point per manifold point, a weld's two boxes, a label per joint), the drawing bounds and the mask culling, the drawables' lives through destroy, geometry change, restore and the world's end, a player's world and its queries drawn, the labels' number formats) |
| `aephysics.snapshot` | a world as bytes and back (the reference's world_snapshot.c): `save_world` and `load_world` keep everything a step reads (the settings, the id pools, the solver sets, bodies, shapes and their geometry once each, contacts with their manifolds and mesh caches, joints, sensors, islands, the trees and pair set, the graph's colours), so a loaded world steps on as the one it was taken from, bit for bit; `restore_world` loads one into an existing world in place, its id, workers and callbacks kept (what a replay rewinds with); a snapshot of another layout or any damage to its bytes loads no world | done, `test_snapshot.ae` (63 checks: a world of every kind of thing loaded at four moments and stepped 150 steps beside its original, the same hash and events every step; a loaded world saves to the same bytes; one worker's world loaded with four steps alike; a restore in place stepping alike and saving to the same bytes, a refused restore leaving the world empty and whole; every truncation and corruption refused, nothing leaked) |
| `aephysics.recording` | recording and replay (the reference's recording.c and recording_replay.c): a world records every call that changes it, 160 kinds, field by field, a create's record with the id it made, a state hash after every step and the geometry the calls name once each, and every world query with each visit and the caller's answer (a caller named by the query filter's id and name gets a key a viewer follows it by); a player replays it into a world of its own, restored in place so its id stays, checking every step's hash and every created id, asking every recorded query again with the recorded answers and comparing each visit, with keyframes under a memory budget for seeking, bodies in the order they were made, the queries of the frame last played with their hits, a pause before a step that makes bodies, and the recording's info; records it does not know are skipped, as a newer minor version may add some | `test_recording.ae` (152 checks: the snapshot test's scene recorded from an empty world and from mid stream with the host changing it between steps, played on one worker and several, every frame's hash as recorded; every one of the 160 calls recorded and replayed (the reference's AllOps); geometry kept once from two copies; seeking back and forth with and without keyframes and under a tight budget; the reference's QueryReplay and TaggedQuery, every query kind with answers that stop and clip them, a changed hit caught at its frame; the reference's NameRoundTrip, RollbackNames, GeometryHashCollision and ReservedHeaderBytes; a file round trip; every truncation, a skipped call, a changed id, a short record and a changed hash each caught, nothing leaked); `test_threads.ae` (four threads querying a recording world, every record whole and replayed alike) |
| `aephysics.human` | the ragdoll: twelve capsule bones on spherical joints with cone and twist limits and revolute joints with angle limits -- a person's ranges by default (shoulders that reach overhead on a cone centred off the rest, hips on an elliptical cone through 125 degrees ahead, 25 back, 45 out and 30 across, knees and elbows that never pass straight), the reference's tight ones on request -- soles that grip, and on request (`Human.ankles`, past the reference) the feet as bones of their own on ankles, an elliptical cone through 50 degrees down, 35 up and 15 aside with 15 of roll, so a crouched figure's soles lie flat, a spring on every joint toward the reference pose, a motor whose torque limit is joint friction, filter joints for the limbs that clash; the align spring, kinematic anchors through motor or parallel joints and their targets driven over a step (the pose drive of an active ragdoll), velocity, kicks, bullets | `test_human.ae` (115 checks: the figure's shape, a fall to rest in one piece, the setters, a kick, standing under the align spring, the pose held on anchors, the anchors driven two metres and the figure walked behind them, the same drop twice bit for bit, a fitted shape, each arm reaching flexion 170, abduction 160 and extension 45 degrees standing and prone, the hips' cone through a person's four ranges and short of a split, the hinges stopping at straight and folding to their range, the soles' grip, the ankles' four ranges measured through the joint and the soles flat under shins tipped 25 degrees over them) | [the reference's rain benchmark, 300 ragdolls over 400 steps, at 0.9x](bench/RESULTS.md#human) |
| `aephysics` | the public API | |

Deliberate choices:

- **Double precision.** Aether's float is a C double, so everything is
  double; the reference's float/double world-position split collapses into
  one `Vec3` and one `Transform`.
- **Determinism kept.** The approximate atan2, cosine and sine exist for
  cross-platform replay and are kept, folded with the engine's own pi.
- **SIMD in the language.** The wide contact solver is the reference's
  eight-lane path in Aether's own lanes (`f32x8`, std.lanes): one AVX2
  register a lane group when built with `AETHER_AEPHYSICS_CFLAGS=-mavx2`,
  two four-lane halves on any other x86-64 or arm64, the same bits either
  way, and it agrees with the scalar solve.
- **Threads.** The reference's task scheduler is ported as it is, on
  threads Aether's std.worker makes, with the little C the threads need
  (aephysics.native); the single-threaded path came first and the
  benchmarks record both.

## Tests and benchmarks

Install the released toolchain with the shared, version-pinned installer
(the same bootstrap used by selaenium):

```sh
curl -fsSL https://raw.githubusercontent.com/aether-lang-dev/aeb/main/get.sh \
  | AE_PIN=0.795.0 AEB_REF=v0.325 sh
export PATH="$HOME/.local/bin:$PATH"
```

This installs `ae` 0.795.0, the release CI pins, and `aeb` v0.325 under
`~/.local` (`PREFIX` overrides it), using release binaries where available.
The code needs 0.788.0 or newer (the eight-wide lanes, `f32x8`).

```
scripts/test.sh                 # builds and runs every tests/test_*.ae with ae; what CI runs
scripts/fetch_references.sh     # Box3D and Jolt into reference/, built (needs cmake, ninja, gcc)
scripts/bake.sh [steps]         # the bake-off scenes on the references
scripts/bench.sh [layer]        # a layer of aephysics against the same code in the reference
scripts/bench.sh suite          # the rest of the reference's benchmark suite, its own scenes against ours
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
One C file goes with the library, `aephysics/native/aephysics_native.c`:
the threads' helpers Aether has not (a thread-local worker index, the
atomics in place, a semaphore). `aephysics.native` names it (`@source`,
Aether 0.704), so every program that imports the library builds it in
with no flag of its own. The contact lanes lived there as GCC vector
code until Aether had lanes of its own; they are the module's `f32x8`
now. `solver.set_wide_contacts(false)` runs the scalar solve instead
([what the lanes buy](bench/RESULTS.md#solver)).

A world steps over as many threads as its definition asks
(`WorldDef.worker_count`, one by default; `native.processor_count()`
for the machine's): the narrow phase, the solver's stages, the bodies'
finalize and the bullets go in blocks over the workers, the reference's
way, with every worker writing its own context and the step merging
them after. The result does not depend on the count: within a colour
no two constraints move the same body, so a step is the same to the
bit at one worker or twenty-four (`test_determinism.ae` runs each
scene over four workers too). The worker threads are made when the
world is and wait on a semaphore between steps; a world of one worker
never makes a thread. A host with a job system of its own lends it
instead (`WorldDef.scheduler`, an `aephysics.parallel` scheduler the
host made and keeps; its worker count then rules), as the reference
takes a game's task callbacks, so an engine runs its crowd, its
weather and its physics on one pool. `physics_world.set_worker_count`
changes a world's own count between steps, clamped to the cap as the
reference's `b3World_SetWorkerCount` clamps it, and the steps after give
the same bits. [What the workers buy, against
the reference's own scaling](bench/RESULTS.md#parallel).

A world's queries may be called from any thread at once, as the
reference's may (but not while that world steps). Every module keeps
its scratch per worker slot, where the reference keeps it on each
thread's stack, and every running thread holds a slot of its own: a
world's workers hold the world's, and any other thread claims one the
first time it asks and gives it back when it exits (64 in all, the
reference's own cap on workers). `test_threads.ae` casts rays and
spheres and overlaps boxes from four threads at once against a mesh, a
height field, a compound and hulls, and checks every answer against the
serial one. A query's visitor may also run more queries on its own
thread, an overlap checking each find's line of sight: a traversal nested
inside another of the same stacks takes stacks of its own for its length,
as each of the reference's queries has its stack on the C stack.

Beyond the modules' own tests, the reference's scene tests run on the
world as a whole: `test_mover_world.ae` (the mover through a world:
which material a plane came from, for meshes, compounds and convex
shapes; 40 checks), `test_determinism.ae` (the reference's wave pile,
query spawn and mesh drop with its own random numbers, and the
falling ragdolls, each run to sleep twice and over four workers and
compared bit for bit; the query spawn sleeps a step after the
reference's with its 59 query hits, the mesh drop two; and its rolling
mix of spheres, capsules and boxes with rolling resistance, whose trace
folds the manifolds' impulses in, the scene of its SIMD width test: CI
runs the golden trace on the four-lane and the AVX2 builds; 22 checks) and `test_large_world.ae` (a stack, a bullet and the
origin-relative queries at x = 0 and at x = 1e7 agree, the whole
engine being in doubles; 43 checks), `test_collision.ae` (the reference's
test_collision.c: boxes valid and not, a ray against a box in its twelve
cases, a hull manifold and a shape's fat bounds at 1e7 against the origin;
56 checks), `test_restitution.ae` (the reference's test_restitution.c
whole: the coefficient head on at three mass ratios and at sixteen impact
phases, a ball on a column of dead balls with propagation off and on, flat
and spinning landings, elastic drops from 40, 20 and 10 m, a cube dropped
flat four times, the threshold, a resting stack that stays put, the
published approach speed, the narrow-floor overshoot, the energy of
thirteen elastic scenes, every step's impulse balanced on drops either side
of continuous collision, and one, two and four workers hashing alike; its
numbers are Box3D's own run's to every printed digit, the few that differ
in the last digit differing between the reference's single and double
builds too; 162 checks) and `test_edge_cases.ae` (every
number a world takes refused when it is NaN, infinite or negative where
it must not be, as the reference asserts it, the world left as it was;
a world or a ragdoll of such numbers never made, a ragdoll made whole
or not at all and never made over, its calls doing nothing unspawned;
a perfect bounce holding its height over twelve bounces, a free spin
keeping its energy, a box a hundred times heavier resting on a light one,
centimetre boxes, a kilometre of ground, a capsule of no length, meshes,
hulls, height fields and compounds of indices past their vertices or
NaN numbers never made, steps of no, negative and infinite time,
geometry that cannot collide, a heavy chain, an unlimited motor, a
hinge whose limits meet, a hinge's motor pressing past its limit settling
where the reference's does, an elliptical cone stopping a swing at each
half-angle, a kinematic body's contacts made stiff on request, and a
hull rocking about a vertex on the floor coming to rest, and the
contacts' sim indices kept current through every move of a sim (and
test_world.c's five locator scenes, at one worker and four alike), and
a weld and a hinge between static and kinematic bodies surviving a
disable and an enable; 158 checks).

A joint's limits are soft constraints, as the reference's are: driven
past one by a motor or held under a steady load, it settles past itself
by the torque over its stiffness (a 100 N m motor on a 2.6 kg capsule
leaves 1.14 degrees, as in the reference). The stiffness is the joint's
`constraint_hertz` (60 by default), which the step caps at a quarter of
the sub-step rate: at eight sub-steps, 120 hertz takes three quarters of
that excess away. A body the game drives hard into others (a kinematic
car) sinks into them while the soft contact pushes them out;
`WorldDef.enable_kinematic_static_softness` (off by default, the
reference's choice) gives its contacts the static softness, twice as
stiff, and a struck ball is out to within a centimetre four frames on
instead of three centimetres.

One place goes past the reference on purpose. A hull resting on an edge
or vertex can rock between two of its faces, and the reference (and this
engine before) rocked it for good, in a cycle of 400 steps: the corner
about to land was on the face the contact had not picked, so it was found
a step deep when the incident face flipped, and the soft contact lifted
it back out. The face contact now also takes the hull's vertices that
already lie within the speculative distance over the reference face, so
the corner lands as a speculative point and the hull sleeps (#134). A
box resting flat has no such vertex, so the reference's stacks give the
same checksums.

`test_soak.ae` throws seeded piles of every convex kind into a walled
pen and holds them to what a sane world does: no energy made, no body
faster than its fall, none through the walls, overlaps no deeper than a
step of speed while falling and the slop at rest, every pile asleep in
the end -- [the reference on the same dice agrees on each](bench/RESULTS.md#soak-a-mixed-pile).

The step is deterministic across platforms and worker counts, not
only across runs: `AEPHYSICS_TRACE=1 target/test_determinism` prints
every body's checksum after every step with its bits, and
`scripts/golden_trace.sh check` (run by `scripts/test.sh`) diffs that
against `tests/golden/determinism_trace.txt` on every step of the four
scenes. CI checks it on Linux, on Linux with `-mavx2 -mfma` and on macOS
arm64; Windows (MinGW) gives the same bits. The engine's own maths (the
reference's cos, sin and atan2 approximations, `sqrt`, `floor` and
`remainder` as the only libm calls) and `-ffp-contract=off` are what make
that hold: without the flag GCC and clang fuse multiply-adds wherever the
target has FMA, and arm64 parted from Linux on the first step (#125). A
program that builds aephysics into itself must pass `-ffp-contract=off`
in its own `[build] cflags`, as `ae` reads only the root manifest's.

## Where it is going

[`design.md`](design.md): the order of the layers, what each is measured
by, and the research this is the ground for -- active ragdolls and
procedural animation in the NaturalMotion/Euphoria line, for the engine
that consumes this.

## Credits

The design and the tests followed are Box3D's, by Erin Catto (MIT), and
the second baseline is Jolt, by Jorrit Rouwe (MIT). aephysics is by
Nicolas Maman, MIT.
