# How aephysics is built

aephysics is Box3D's design written in Aether: the same data layout, the
same step, the same solver, checked against Box3D's own tests and timed
against Box3D on the same machine. This document is the map: the modules
and what each ports, how a step runs, how the data is laid out, how the
work goes over threads and SIMD lanes, and why the results are the same
everywhere. The [guide](guide.md) is the user's view;
[testing.md](testing.md) says how all of it is checked.

## Why an implementation and not a binding

An Aether program could call Box3D through externs. The engine is written
in Aether instead so that Aether's programs, and the people and agents who
write them, can read, test and change the physics in the language, and so
that the library stands alone (`ae build --emit=lib`). Box3D and Jolt are
fetched unmodified beside it as the baseline: the engine is right when it
agrees with Box3D's tests and fast when it keeps up with Box3D's
benchmarks. Which design to follow was itself decided by measurement: on
identical scenes Box3D's solver was both stiffer and faster than Jolt's
([the bake-off](../bench/RESULTS.md#the-bake-off)).

## The modules

Aether modules cannot import each other in a cycle, and Box3D's world is
one mutually recursive body of C (body.c, contact.c, joint.c, island.c,
solver_set.c, constraint_graph.c, sensor.c and half of physics_world.c call
each other). So the geometry and the solvers are modules of their own, and
the world's bookkeeping is one module, `dynamics`. From the bottom up:

| layer | module | ports | holds |
|---|---|---|---|
| foundation | `math` | math_functions | vectors, quaternions, transforms, 3x3 matrices, boxes, the deterministic atan2/cos/sin, single-precision twins (`Vec3f`, `Quatf`, `Matrix3f`) |
| | `basics` | core.c, bitset, id_pool, table, container, arena | bit sets, id pools, hash sets, arrays, the stack and arena allocators, the host's hooks, the content hash, the name cache, the SIMD width |
| | `native` | (C) | the one C file: a worker index per thread, atomics on an int in place, a semaphore, the processor count |
| | `parallel` | scheduler.c, parallel_for.c | worker threads made once that wait on a semaphore, tasks claimed by compare-and-swap, ranges in blocks |
| geometry | `dynamic_tree` | dynamic_tree.c | the bounding volume hierarchy: SAH insertion, rotations, refit, rebuild, box, ray and swept queries, save and load |
| | `hull` | hull.c, quickhull | quickhull with face merging, the half-edge hull, its mass, box/cylinder/cone/rock hulls, single-precision rows for the wide SAT |
| | `distance` | distance.c | GJK with its simplex cache, the shape cast, the time of impact |
| | `sat` | convex_manifold_wide.inl | the separating axis test between hulls, eight lanes wide |
| | `manifold` | manifold.c, convex_manifold.c | contact manifolds for every convex pairing: clipping, feature ids, reduction to four points |
| | `triangle_manifold` | triangle_manifold.c | one mesh triangle against a convex shape |
| | `mesh`, `height_field` | mesh.c, height_field.c | triangle meshes under a BVH, height fields on a grid, their queries |
| | `sphere`, `capsule`, `compound` | sphere.c, capsule.c, compound.c | the primitives' mass, bounds and casts; the baked compound under a static tree |
| | `shape` | shape.c (geometry half) | any shape kind under a transform: bounds, mass, casts, overlap, filters |
| | `material`, `files`, `mover` | material.h, file I/O, mover.c | surface materials; trees and height fields on disk; the character mover's plane solver |
| collision | `broad_phase` | broad_phase.c | a tree per body type, the moved proxies, pair finding against the pair set |
| | `mesh_contact` | mesh_contact.c | a convex shape against a mesh: the triangle cache, the ghost-collision rules, clusters |
| world | `dynamics` | body.c, shape.c, contact.c, joint.c, island.c, solver_set.c, constraint_graph.c, sensor.c, world.c | worlds, bodies, shapes, contacts, joints, islands, solver sets, the constraint graph, sensors, events |
| solving | `contact_solver` | contact_solver.c (scalar) | contact constraints one at a time: meshes and the overflow colour |
| | `contact_solver_wide` | contact_solver.c (SIMD) | a colour's convex contacts eight at a time |
| | `joint_solver` | joint.c and the nine joint files | each joint kind's prepare, warm start, solve and accessors |
| | `solver` | solver.c | the Soft Step stages over the workers |
| face | `physics_world` | physics_world.c | the step, settings, events, queries, explosions, counters, drawing |
| tools | `debug_draw` | debug_draw | the host's drawing callbacks and colours |
| | `snapshot` | world_snapshot.c | a world as bytes and back |
| | `recording` | recording.c, recording_replay.c | every call recorded, and a player that replays and seeks |
| | `human` | shared/human.c | the ragdoll |

The naming follows Box3D: where a function is the same thing it has
Box3D's name without the prefix, in snake case, so a reader can hold a
Box3D source file next to its port.

## The data

**Worlds and ids.** Worlds live in a fixed table of 128. Every object is
reached through an id that carries its slot and the slot's generation, so
an id kept past its object's life fails validation instead of reaching the
next occupant.

**Solver sets.** A body's simulation state lives in one of the world's
solver sets: the static set, the disabled set, the awake set, or one set
per sleeping island. The awake set is the only one a step touches; it holds
each body's `BodySim` (transform, centre, mass, forces) and `BodyState`
(the velocities and deltas the solver reads and writes, kept small and
dense). Sleeping and waking move whole islands between sets.

**Islands.** Touching contacts and joints link bodies into islands, merged
as links appear and split by union-find when they go. An island that has
rested long enough goes to sleep as a unit.

**The constraint graph.** Awake contacts and joints are coloured so that
no two constraints of a colour share a dynamic body: 24 colours, the
first 20 for pairs of dynamic bodies, pairs with a static body taken from
the back, and an overflow colour for what does not fit. Within a colour
every constraint can be solved at the same time, which is what makes both
the lanes and the threads safe, and the result independent of their
number.

**Contacts.** A contact keeps its manifold (up to four points with their
impulses for warm starting), its bodies' sim indices (refreshed whenever a
sim moves, so the collide pass never walks from shape to body to set), and
its shapes' fat boxes in a dense array by shape id.

## A step

`physics_world.step(world, dt, sub_steps)` runs Box3D's step:

1. **Pairs.** The broad phase walks the proxies that moved since the last
   step through the trees (one per body type), checks candidate pairs
   against the pair set in batches with their slots prefetched, and new
   pairs become contacts.
2. **Collide.** The narrow phase computes every awake contact's manifold
   in parallel: convex pairs through `manifold` and `sat`, meshes and
   height fields through `mesh_contact`, compounds child by child. Contacts
   that begin or stop touching are flagged per worker and applied after,
   in contact order.
3. **Solve.** The Soft Step: the joints and contacts are prepared (anchors,
   effective masses, softness); then for each sub-step the velocities are
   integrated (gravity, damping, the gyroscopic torque), the constraints
   warm started and solved with a soft bias, the positions integrated, and
   the constraints relaxed without bias. Restitution runs after the
   sub-steps, and the impulses are stored for the next step's warm start.
4. **Finalize.** Bodies get their new transforms, sleep velocities and move
   events; fast bodies are swept for continuous collision, bullets against
   dynamic bodies too; the trees are refit, joint and hit events gathered,
   sensors updated, and resting islands put to sleep.

Within the solve, joints come before contacts in each colour and the
overflow colour comes first. The wide stages take a colour's convex
contacts eight at a time; mesh contacts and the overflow are solved by the
scalar `contact_solver`.

## Precision

Positions are double: a body's transform and centre of mass, the geometry,
and every query, so a world far from the origin behaves as one at it.
Box3D keeps double positions only in its large-world build.

Box3D's single precision is kept wherever its speed comes from the bytes
moved: the solver's body states (velocities and the step's deltas, one
cache line a body), a body's inertias, forces and dampings, the stored
manifolds and the contacts' cached values, the wide contact records, the
broad-phase tree's boxes (rounded outward from the doubles, as Box3D's
large-world build rounds them), and the hulls' rows for the separating
axis test. The narrow phase computes in double and narrows once as it
stores; the scalar solvers widen to double for their arithmetic.

## SIMD

Aether has SIMD types of its own (`f32x4`, `f32x8`, std.lanes), so the wide
paths are Aether, not C. `contact_solver_wide` keeps a colour's convex
contacts as one single-precision record per eight contacts and solves them
in `f32x8` lanes; the bodies are gathered into lanes and scattered back
through 8x8 register transposes, as Box3D's `b3GatherBodies` and
`b3ScatterBodies` do. `sat` runs Box3D's wide separating axis test, its
support search keeping the vertex index in the low mantissa bits so one
horizontal minimum finds it.

Both modules are imported twice: plain, and with `@target("avx2")`, which
compiles a second copy for AVX2 inside the same program. A world picks its
copy when it is created (`basics.get_simd_width`: 8 where the CPU has AVX2,
4 otherwise, and `basics.set_simd_width` to choose), so one binary runs
eight lanes to a register on AVX2 and two four-lane halves elsewhere. The
two copies give the same bits; `test_determinism` steps every scene at
both widths in one process to hold them to that.

## Threads

`aephysics.parallel` is Box3D's scheduler: worker threads made with the
world, parked on a semaphore between steps, tasks in slots claimed by
compare-and-swap, the calling thread working as worker 0 while it waits.
The narrow phase, the bodies' finalize and the bullets are parallel loops
over blocks; the solver is Box3D's stage machine, where worker 0 publishes
each stage as a sync bit and every worker claims blocks of it. Each worker
writes its own task context (contact flags, joint and hit bits, awake
islands, sensor hits) and the step merges them after, in id order.

Box3D keeps its scratch on each thread's stack. Aether has neither stack
arrays nor thread-local variables, so every module keeps a scratch block
per worker slot instead, chosen by the calling thread's index from
`native`. There are 64 slots, Box3D's own cap on workers; a world's workers
hold the world's, and any other thread claims one the first time it
queries and returns it when it exits. That is what lets queries run from
many threads at once.

## Determinism

A step gives the same bits on every platform, at any worker count and
either SIMD width, because:

- the order of every floating-point sum is fixed: colours are solved in
  order, workers' results are merged in id order, and lane reductions run
  in a fixed order;
- the C compiler never fuses a multiply and an add (`-ffp-contract=off`);
  GCC and Clang do by default wherever the target has FMA, and arm64 parted
  from Linux on the first step until the flag was set;
- the trigonometry is the engine's own (Box3D's approximations), and the
  square root is std.math's, exact at every optimization level (MinGW's
  library square root rounds twice at `-O0`).

`tests/golden/determinism_trace.txt` holds every body's checksum after
every step of five scenes, and CI diffs it on Linux, Linux with AVX2 and
FMA, macOS arm64, and Windows optimized and not.

## Memory

Every block the engine allocates goes through `basics`, which counts it and
hands it to the host's allocator when one is set. A step's temporary memory
comes from the world's stack allocator and each worker's arena, as in
Box3D. `physics_world.get_counters` reports the total; `dump_memory_stats`
breaks it down.

## The C file

`aephysics/native/aephysics_native.c` holds what Aether cannot express:
the thread-local worker index (a native TLS slot on Windows, where MinGW's
emulated TLS costs a call per read), atomics on an int in place, a
semaphore, and the processor count. `aephysics.native` names it with
`@source`, so `ae` builds it into every program that imports the engine.
The contact lanes lived in that file as GCC vector code until Aether had
lanes of its own; nothing else ever did.

## Where it departs from Box3D

The user-visible additions (travel and kinematic speculation, the
kinematic-static softness, elliptical and off-axis cones, hulls that come
to rest on an edge) are in the [guide](guide.md#where-aephysics-goes-past-box3d).
Inside, the differences are of form, not of result:

- A hull shape holds the world's copy of its hull (Box3D's hull database),
  so a caller frees its own at once.
- The name cache gives a second name that collides on Box3D's 32-bit hash
  the next free id, where Box3D shares the first name.
- A host's debug drawable for a shape is released when its geometry
  changes or its world goes, where Box3D keeps a stale one or leaves it to
  the host.
- Bad input is refused and the world left as it was, where Box3D asserts.
- A mesh query has no 256-triangle cap.

## What it is the ground for

aephysics exists for [ae3d](https://github.com/aether-lang-dev/ae3d) and
the games built on it, and above all for active ragdolls: characters that
are simulated bodies driven toward an animation, the line NaturalMotion's
Euphoria made famous in Grand Theft Auto IV and V and Red Dead Redemption.
Euphoria's behaviours are unpublished; the research around them is
public: Reil and Husbands on evolved central pattern generators (2002),
SIMBICON's balance feedback (Yin, Loken and van de Panne, 2007), the
survey by Geijtenbeek and Pronost (2012), and for locomotion, motion
matching (Clavet, 2016) and learned motion matching (Holden et al., 2020).

What an active ragdoll needs from the physics is what Box3D's dynamics
gives: articulated bodies on joints with limits, springs and motors, a
solver that holds a chain of a dozen bodies stable under those motors,
and determinism, so a behaviour's tests can pin outcomes. `aephysics.human`
provides the figure, its pose drive and its anchors (see
[the guide](guide.md#ragdolls)); the behaviours themselves (balance,
protective reflexes, stepping to recover) are ae3d's.
