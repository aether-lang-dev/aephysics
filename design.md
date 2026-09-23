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
2. **core** (as `aephysics.basics`, since a host engine has a `core` of its own and Aether binds modules by their last name): done. `bitset` over 64-bit longs, `id_pool`, `table` (the
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
   Save/load not ported; the moved-marking is atomic, as the reference's,
   since the parallel layer (14).
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
   - `aephysics.dynamics` (in progress): one module for the world's
     state and its bookkeeping -- the World with its sparse arrays and
     id pools, the ids with generations, the body's sims and states in
     solver sets (static, disabled, awake, one per sleeping island),
     the shape's world half (creation on a body of every kind, the fat
     bounds, the proxy in the tree of the body's type, materials, event
     flags), the islands, the mass from the shapes or by hand, the
     transforms and velocities, the validation. Done so far: worlds,
     bodies, shapes, islands of one body, sleeping sets and waking them
     (bodies and islands only), 156 checks: all of test_body.c and the
     step-free parts of test_world.c, plus every set and shape kind,
     transforms, velocities, locks, destruction in every order with the
     sets validated throughout. The reference's hull database (hulls
     deduplicated by content and refcounted across shapes) and its name
     cache are not ported; a hull shape keeps the pointer it was given.
     Second slice: the contact (creation from the broad phase's pair
     keys through the pair visitors, the edges on both bodies, the
     manifold update -- convex pairs through the manifold module,
     mesh and height field pairs and a compound's children through the
     mesh contact with the reference's tail: the old manifolds matched
     by normal and the points by feature and triangle, the materials
     mixed or averaged per triangle, rolling resistance, tangent
     velocity -- the recycling of a barely-moved pair, the collide pass
     with the state changes in id order and the begin/end events), the
     constraint graph's colouring (dynamic pairs from the front, static
     pairs from the back, an overflow colour), the islands linked by
     touching contacts, merged, and split by union-find, sleeping sets
     with their touching contacts and waking them back into the graph.
     231 checks in all, contacts and islands checked against the
     reference's rules on a cube on a slab, a stack, a row split, a
     sphere on a mesh and a cube on a compound child. Bench: the
     reference's zero-time step collides without solving, so the pass
     compares (1.7x, aephysics#17 to profile with the solver).
     Third slice: joint.c's base (the joint definitions of every kind,
     creation on two bodies with the edges on both, the sim in the
     disabled, static, awake (the graph, a sleeping set woken) or
     sleeping set (two sleeping sets merged), the island link, the
     collide-connected rule in the pair filter with the world's custom
     filter, destruction, the accessors, the linear and angular
     separations, the reaction for the joint events), the kinds' data
     laid over a block at the end of the sim (the reference's union),
     solver_set.c's transfers and merge, body.c's type change, disable
     and enable (which move the joints along), sensor.c (a sensor per
     sensor shape, the overlap pass over the three trees with the
     visitors sorted and made unique, the begin and end events in id
     order, the hits the continuous pass will feed it) and shape.c's
     sensor accessors. 541 checks in all: test_joint.c's shared API on
     every kind and the contacts a joint clears, the sets and islands
     under sleep, wake, disable, enable and type changes, and
     test_world.c's sensor moved by hand. Found and fixed on the way:
     core's buffers grew from a capacity of one to one (the sleeping
     sets of one body). The solving of the kinds (prepare, warm start,
     solve, the per-kind accessors and constraint forces) comes with
     joint_solver.
   - `aephysics.contact_solver` (done, PR #19): contact_solver.c in its
     scalar form (the reference's mesh and overflow path) for every
     contact: ContactConstraint and ManifoldConstraint over a colour's
     convex contacts and specs, the step context (b3StepContext, one
     worker), prepare (the split separation, friction centre decay,
     tangent/twist/rolling masses), warm start, the merged normal and
     friction solve with the relax pass, restitution, store with the hit
     events. 49 checks by hand on a cube on a slab: masses, the falling
     cube stopped with its momentum spread over four points, the push-out
     capped by the contact speed, the speculative gap, the friction bound
     mu N and the friction centre, the twist bound, restitution above and
     below its threshold, the warm start and the hit event.
   - `aephysics.contact_solver_wide` (done, PR #25): the reference's
     wide path (b3ContactConstraintWide, b3PrepareContacts_Convex and
     its siblings) for a colour's convex contacts: FloatW lanes of four,
     the constraint as a structure of arrays, gather and scatter of the
     bodies' states, prepare, warm start, solve, restitution and store;
     the mesh and overflow contacts stay scalar. The lanes are the
     language's `f32x4` (std.lanes, Aether 0.706) over one
     single-precision constraint record: four floats in a register, the
     reference's SSE and NEON lanes. They were written twice for a while
     -- in Aether as plain code over doubles, and in aephysics_native.c
     as GCC vector code in single precision, which is where the speed
     was (issue #22: the layout alone bought 10%, the vector lanes
     another 20-30%). Once Aether had lanes, a kernel measured at what
     the same C costs (bench/lanes.ae), the module's lanes became f32x4,
     computed the native file's record to the bit and ran faster on the
     pyramids (PR #48), and the C copy went (#46). The solve is then at
     parity with the reference's; the rest of the step's gap is the
     collide pass and the prepare (bench/RESULTS.md, stage by stage).
   - `aephysics.joint_solver` (done, PR #20): the seven joints' prepare,
     warm start and solve (distance, motor, parallel, prismatic,
     revolute, spherical, weld, wheel: distance_joint.c and its
     siblings), the kinds' accessors and their constraint forces and
     torques, joint.c's dispatch with the constraint hertz clamped to a
     quarter of the sub-step rate. Shared pieces factored once: the
     prepare's base (masses, fixed rotation, awake indices), the world
     frames, the hinge's perpendicular axes, the point constraint's mass
     matrix, the limits' bias scales. math gains skew, blend3 and the
     quaternion delta. 149 checks: test_joint.c's accessor round trips
     on every kind, then each kind solved by hand on a cube hung from a
     static ground (the velocities a rigid joint removes and keeps, the
     limits' push back on a body turned past them and the relax undoing
     it before the bodies move, springs, motors at their speed and
     bounded by their force, the forces and torques from the impulses,
     the warm start).
   - `aephysics.solver` (done, PR #21): solver.c on one thread in the
     reference's stage order (prepare joints and contacts; per sub-step
     integrate velocities with the gyroscopic Newton step, warm start,
     solve, integrate positions with the locks and speed caps, relax;
     restitution; store), the overflow constraints first in every pass
     and joints before contacts in a colour; finalize bodies (transforms
     from the deltas, sleep velocities, move events, the transient flags,
     fast bodies swept in solve_continuous with the time of impact
     against the static tree -- bullets against all three, later -- the
     bounds and proxies), the joint and hit events, the tree refits, the
     sensors' hits, the island split the last step asked for and the
     sleep decision. The shape module gained shape_time_of_impact (mesh,
     height field and compound sweeps with the reference's early outs and
     centroid-sphere fallback); dynamics the events, the per-step bit
     sets, the pre-solve callback and the user material ids; math the
     rotation integration and the modified cross; core stack_grow. 42
     checks stepping by hand: free fall against the sub-stepped closed
     form, a cube settling and sleeping with its contact, a push waking
     it, a stack of three, a bounce with its hit event, a weld's joint
     event and its force, a pendulum's length, a sphere at 200 m/s
     stopped by a wall 0.2 thick (and tunnelling with continuous off),
     a bullet stopping at a dynamic cube, a sensor swept through, and two
     worlds of twenty cubes alike bit for bit after two seconds. Found on
     the way: putting islands to sleep creates solver sets and can move
     the sets' array, so the awake set is looked up again in that loop.
     The bench pair against b3World_Step landed at 1.9-2.0x with the
     same heights, awake and contact counts (the reference's convex
     contacts go four wide in SIMD and it computes in floats; ours were
     scalar doubles), 1.2-1.4x once contact_solver_wide took them.
   - `aephysics.physics_world` (done, PR #23): the step (the events
     cleared, the pairs, the context with the contact hertz reduced for
     large steps, the narrow phase, the solve when time passes, the
     sensors, the stack grown, the end events swapped, the world locked
     throughout), the events read back (end events from the buffer the
     last step filled), the settings (sleeping off wakes every set), the
     counters and bounds, the queries over the three trees re-centred on
     their origin (overlap of a box or a proxy, the mover's planes, ray,
     shape and mover casts, the closest ray hit) and body.c's queries at
     a transform of the caller's (ray, shape, overlap, mover planes, the
     mover's time of impact), explosions. 94 checks: test_world.c's
     HelloWorld, contact and hit events with their materials, the
     continuous move event matching the transform, the bullet through a
     sensor, the explosion near and a ten million away; test_body_query.c
     end to end; the world queries; a wave pile of sixty cubes stepping
     alike twice with its sleep step as the checksum. Found on the way:
     struct names are one namespace across modules (a QueryContext
     already lived in compound), and a struct literal in a return with
     module calls inside reaches C undeclared, as constants did. The
     pair on the reference's own benchmark scenes matches its checksums
     (heights, contacts, joints) at 1.8-2.4x.
     PR #26 ported the rest of the reference's scene tests: the world
     parts of test_mover.c (test_mover_world.ae, 38 checks), the
     determinism scenes (test_determinism.ae: the reference's wave pile,
     query spawn and mesh drop with its xorshift random numbers and
     seeds, each run twice and compared bit for bit since its golden
     hashes are single precision and ours is doubles; the query spawn
     sleeps on the reference's own step with its 59 query hits, the
     mesh drop one step apart), and test_large_world.c
     (test_large_world.ae: a stack, a bullet and the origin-relative
     queries at x = 0 and 1e7 agree, which the reference only asks of
     its double precision build; b3Shape_RayCast came with it as
     shape_ray_cast). Found on the way: a function named spawn_* loses
     its arguments in the emitted C (aether#2126). Still to port: the
     falling ragdolls (they need the human of shared/human.c, the
     ground of the Euphoria work), test_world.c's compound hit events,
     overflow colour pile and hull database.
   - `aephysics.human` (done, PR #29): the reference's ragdoll
     (shared/human.c) as a module, with the body forces, torques and
     impulses, the bullet setter and body_get_contact_data it needed on dynamics. 44 checks
     (the figure's shape, a fall to rest in one piece, the setters, a
     kick, the align spring standing it, the anchors holding its pose,
     the same drop twice bit for bit); the falling ragdolls scene of
     test_determinism.c (eight figures on grid meshes and tori) added to
     test_determinism.ae; the reference's "rain" benchmark (300 ragdolls
     dropped in columns over 400 steps) as a pair at 1.05x its time. The
     research note below says what an active ragdoll takes from it.
14. **parallel** (done, PR #31): `aephysics.parallel` is the reference's
   scheduler.c and parallel_for.c -- worker threads made once with the
   world (std.worker's run_detached, a thread each) that wait on a
   semaphore, tasks in slots claimed by compare-and-swap, the main
   thread helping while it waits, a range in blocks the tasks claim --
   and `aephysics.native` what Aether has not: a thread-local worker
   index, atomics on an int in place, the semaphore. Every module's
   scratch became a block per worker chosen by the thread's index
   (Aether has neither stack arrays nor thread-local variables), the
   dynamic tree's traversal stacks and the moved-marking included. The
   narrow phase, the bodies' finalize and the bullets are parallel_for
   tasks; the solver is the reference's stage machine (solver blocks
   with an atomic sync index, stages published as sync bits by worker
   0, the overflow colour serial on it); each worker writes its own
   task context (contact state, joint and hit bits, awake islands,
   split candidate, sensor hits) and the step merges them. A step is
   the same to the bit at any worker count, which test_determinism
   checks; bench/parallel.ae against bench/parallel_box3d.c records
   the scaling. Found on the way: every task on a fresh thread costs
   180 us a round (the persistent threads cost 8), and the stack
   allocator's zeroing was a millisecond a step (the reference's does
   not zero; poisoning the memory shows nothing reads it first).
15. **recording and replay**, `world_snapshot`: last, since they are the
   tooling and not the engine.
16. **benchmarks**: `reference/benchmark/main.c`'s nine scenes ported, run
   against the C build on the same machine, recorded under `benchmark/`.

## Measures

- Every ported test passes with the original's tolerances.
- Determinism: `test_determinism` (the same scene twice, bit-identical),
  and the port against the C original on a scene's first frames; across
  platforms too: the per-step trace CI records on Linux matches the
  Windows run bit for bit (PR #28), which the reference pins with golden
  hashes and this port pins by diffing the traces.
- Speed: each benchmark scene, port against C, single thread first,
  then by worker count. The honest expectation for a scalar double port
  against a wide-SIMD float original is a gap; the work after the port
  is closing it -- and where the gap is the SIMD contact solver, that
  one loop goes native behind the same interface, measured.

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

### What the ragdoll layer gives them (PR #29)

`aephysics.human` is the reference's figure (shared/human.c) as a module:
twelve capsule bones, spherical joints with cone and twist limits at the
spine, neck, hips and shoulders, revolute joints at the knees and elbows,
a spring on every joint toward the reference pose, a motor on every
joint whose torque limit is its friction, and filter joints for the
limbs that would clash. The pieces an active ragdoll composes are all
there and measured:

- **The pose drive.** Every joint has a spring (`hertz`, `damping_ratio`)
  toward a target: `target_angle` on the hinges, `target_rotation` on
  the spherical joints; `human_set_joint_spring_hertz` and
  `human_set_joint_damping_ratio` tune the whole figure. Setting the
  targets from an animation clip each step is the animation-driven
  ragdoll (plan item 2); the springs are soft constraints, so a chain
  of twelve holds under them without exploding, which the rain
  benchmark shows at 300 figures.
- **The muscle budget.** The motors' `max_motor_torque` per joint
  (`human_set_joint_friction_torque` scales all of them by a per-joint
  share) is the torque limit a behaviour works within; a spring's
  `max_spring_torque` on the motor joints bounds the drive itself. A
  behaviour raises the budget on the limbs it needs (arms toward the
  ground on a fall) and drops it on the rest (going limp).
- **The pose target as bodies.** `human_create_motor_anchors` hangs
  every bone from a kinematic anchor through a motor joint's position
  and rotation springs: move the anchors (from a clip, from a controller)
  and the figure follows, with the world pushing back. The parallel
  anchors (`human_create_parallel_anchors`) drive rotation only, with a
  torque cap, and let the figure fall while it keeps its shape -- the
  shape of Euphoria's "keep the pose while you tumble" reflexes.
- **Balance's inputs.** A controller in the SIMBICON line needs the centre
  of mass and its velocity (the bones' masses and velocities are
  readable), the support polygon (the feet's contact points and normal
  impulses come through the contact events and `body_get_contact_data`, PR #29),
  and the joint reactions (`joint_get_constraint_force/torque`); it
  writes the hip and stance-ankle targets. The align spring
  (`human_align_spring`) is the crudest balance: a parallel joint to the
  ground that springs the pelvis upright, which the sample uses at 25 Hz.
- **Actuation beyond springs.** `body_apply_torque`,
  `body_apply_force`, and the impulses (PR #29 added them) let a
  controller act directly on a bone for the cases a joint spring cannot
  express (a shove, a step's push-off).
- **Determinism.** The falling ragdolls repeat bit for bit and match
  across platforms, so a behaviour's tests can pin outcomes.

What is not there yet, and belongs to the ae3d epics: a skeleton-to-
ragdoll builder (this figure's frames were measured from one rig), the
per-step controller hook (a `before_step` callback so behaviours set
targets from the state the step will use), and the behaviours
themselves.
