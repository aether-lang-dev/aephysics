# The bake-off

The reference engines on identical scenes, on one machine, one thread, each
at its own recommended settings for a game's 60 Hz step. What decided
which design this engine follows.

**Machine:** Intel Core i7-13700K, Windows 11, GCC 14 (msys2 ucrt64),
2026-09-19. The GPU was busy with a game at the time; these are CPU numbers.

**Builds:** Box3D `f555ee4` (Release, SSE2, validation off); Jolt `v5.3.0`
(Distribution: LTO, AVX2/FMA, no profiler, no debug renderer -- its
shipping configuration). Box3D's build here has neither AVX nor LTO, so
if anything the table favours Jolt.

**Settings:** a 1/60 s step. Each engine at its own recommended setting
first -- Box3D: 4 sub-steps (its own benchmark setting); Jolt: 1 collision
step, 10 velocity + 2 position iterations (its defaults) -- and then each at
the other's budget: Jolt at 4 collision steps, Box3D at 16 sub-steps.
Sleeping off on both, so every body is simulated every step. Timing
excludes the first step (the structures are built there). The machine was
also running a game, so the times wander by about 20% between runs; the
ratios do not.

### Each at its recommended setting

| scene | bodies | Box3D 4 sub-steps | Jolt 1 step | how it held |
|---|---|---|---|---|
| pyramid, 100 rows of unit boxes | 5,050 | **9.6 ms/step** | 44.5 | Box3D: worst drift 0.79 m after 300 steps, 0.72 m after 1,000 (the top wobbles, the stack stands). Jolt: 17.5 m after 300, 102 m after 1,000 -- the pyramid collapses. |
| pile, 20 x 25 x 20 unit boxes dropped 2 m | 10,000 | **10.2** | 42.5 | lowest box after 300 steps: Box3D 0.498 m (2 mm into the ground), Jolt 0.473 m (27 mm) |
| chain, a 100 x 100 grid of spheres on point joints, hung from its top row | 10,000 bodies, 19,800 joints | **11.1** | 11.3 | sag of the bottom corner below a taut 99-link chain: Box3D 0.94 m, Jolt 1.97 m |

### Each given the other's budget

| scene | Jolt 4 steps | Box3D 16 sub-steps | how it held |
|---|---|---|---|
| pyramid | 157 ms/step | **28.2** | Jolt now stands: drift 0.05 m. Box3D 0.68 m (its contacts are soft, 30 Hz by default; see below). |
| pile | 60.8 | **24.4** | Jolt 0.4995 m (0.5 mm in), Box3D 0.498 m (2 mm) |
| chain | 42.8 | **39.1** | Jolt 0.094 m. Box3D 0.79 m at its default 60 Hz joints; **0.047 m** with the joints set to 240 Hz (`BAKE_JOINT_HERTZ=240`), same 39 ms. |

`scripts/bake.sh 300` reproduces both tables; `bench/bake_box3d.c` and
`bench/bake_jolt.cpp` are the scenes.

## Reading it

- At its recommended setting Box3D's Soft Step solver is four times faster
  than Jolt's sequential impulse solver on the contact-heavy scenes, and it
  is the one that holds a tall stack at that speed: sub-stepping with soft
  constraints converges where iterating a single step does not.
- Given four times the collision steps Jolt holds everything too, at four
  to sixteen times Box3D's cost. Box3D at sixteen sub-steps -- about a
  third to two-thirds of Jolt's 4-step time -- matches or beats it on every
  scene.
- The residual drift and sag in Box3D are its soft constraints, not a
  convergence limit: the joint stiffness is a per-joint hertz (default
  60 Hz, capped at a quarter of the sub-step rate), and raising it with the
  sub-step count took the chain's sag from 0.79 m to 0.047 m at no extra
  cost. The same knob exists for contacts (`contactHertz`, 30 Hz). This is
  the design working as intended: stiffness is a choice made per scene,
  bounded by the sub-step rate, rather than an iteration count.
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

# Layer by layer

Each layer of aephysics against the same code in the reference, on this
machine, one thread. `scripts/bench.sh` builds and runs every pair.

## dynamic_tree

`bench/tree.ae` and `bench/tree_box3d.c`: 10,000 boxes (1 m, in a 200 m
cube) inserted one by one, 100 full rebuilds, 100,000 box queries (2 m),
10,000 ray casts (up to 200 m), then ten rounds of enlarging every proxy
by a random metre and a partial rebuild. The same random sequence on both.

| phase | aephysics | Box3D |
|---|---|---|
| insert 10,000 | **5.0 ms** | 9.4 |
| 100 full rebuilds | 46 | **42** |
| 100,000 box queries | 12.5 | **11.9** |
| 10,000 ray casts | 13.4 | **7.2** |
| 10 x enlarge all | **2.5** | 2.9 |
| 10 x partial rebuild | 5.9 | **5.0** |

Both report 3,965 query hits, 1,950 ray hits, height 17 and area ratio
63.45: the trees are the same tree. The ray cast is where the reference's
SSE2 box tests and 24-byte float boxes show against the port's scalar
tests and 48-byte double boxes; that is the number for the native wide
path to beat, if the whole-step benchmark says the tree's ray cast
matters.

## hull

`bench/hull.ae` and `bench/hull_box3d.c`: 200 hulls of 64 points on a
sphere capped at 32 vertices, 20 hulls of 4,096 points inside a cube with
the corners stamped last (every interior point through the conflict lists,
most cone faces merged out), and 2,000 box hulls. The same random sequence
on both.

| phase | aephysics | Box3D |
|---|---|---|
| 200 sphere hulls, 64 points to 32 | 4.7 ms | **2.9** |
| 20 cube hulls, 4,096 points to 8 | 5.2 | **2.5** |
| 2,000 box hulls | 2.0 | **0.17** |

Both produce 6,554 vertices and 12,106 faces in total: the same hulls.
The builder runs at 1.6-2x the reference's time (doubles, indices in
place of pointers, structs passed by value); the box hull is a heap block
here where the reference's is a stack value, so its cost is the
allocation and the hash. Hull construction is a load-time cost, not a
per-step one, so this is recorded rather than chased.

## distance

`bench/distance.ae` and `bench/distance_box3d.c`: two boxes (1 x 0.5 x
0.75 and 0.6 x 0.8 x 0.4) over 100,000 poses along a sweep that passes
through overlap, each pose a GJK query from a cold cache and then from a
cache warmed by the pose before; 10,000 shape casts of the second box at
the first; 10,000 times of impact of the second box falling and turning
onto a slab.

| phase | aephysics | Box3D |
|---|---|---|
| 100,000 GJK queries, cold cache | 22.9 ms | **17.3** |
| 100,000 GJK queries, warm cache | 18.2 | **11.9** |
| 10,000 shape casts | 5.0 | **3.5** |
| 10,000 times of impact | 13.8 | **9.4** |

The sums of the results agree to the precision of the reference's floats
(distances 26,291.3 on both, cast fractions 2,780.06, impact fractions
5,523.1) and the GJK iteration counts within 0.01% (335,952 against
335,990): the same algorithm taking the same paths. The port runs at
1.3-1.5x the reference's time; the simplex is passed by value here
where the reference works on it in place.

## manifold

`bench/manifold.ae` and `bench/manifold_box3d.c`: 20,000 hull-hull
collisions of two boxes (1 x 0.5 x 0.75 and 0.6 x 0.8 x 0.4) along a sweep
through overlap with a cold SAT cache each time; the same 20,000 with the
cache carried from pose to pose (a stack's steady state); 20,000
hull-capsule and 20,000 hull-sphere collisions along a sweep.

| phase | aephysics | Box3D |
|---|---|---|
| 20,000 hull-hull, cold cache | 15.3 ms | **9.5** |
| 20,000 hull-hull, warm cache | 3.7 | **3.6** |
| 20,000 hull-capsule | 7.2 | **5.0** |
| 20,000 hull-sphere | 4.0 | **2.6** |

The same manifolds come out: 52,386 / 52,074 / 23,482 / 11,500 contact
points, 19,706 cache hits, and equal separation sums on every phase. With
the cache warm -- the state a resting stack is in -- the port is at parity;
the cold separating axis test is 1.6x, the reference's being SIMD four
edge pairs at a time, which is the wide path to consider if a step
benchmark ever shows the cold SAT.

## triangle_manifold

`bench/triangle.ae` and `bench/triangle_box3d.c`: 20,000 triangle-hull
collisions of a box at a hundred attitudes sinking onto a big triangle
with a cold SAT cache each time; 20,000 of a box resting on a creeping
triangle with the cache carried; 20,000 triangle-capsule collisions of a
capsule swung across a triangle and its edge; 20,000 triangle-sphere
collisions of a sphere swept over the triangle and off it.

| phase | aephysics | Box3D |
|---|---|---|
| 20,000 triangle-hull, cold cache | 2.8 ms | **2.6** |
| 20,000 triangle-hull, warm cache | 2.6 | **2.4** |
| 20,000 triangle-capsule | 5.1 | **3.8** |
| 20,000 triangle-sphere | **0.32** | 0.61 |

The same manifolds come out: 39,536 / 80,000 / 29,778 / 6,422 contact
points, 19,999 cache hits, equal separation sums. Triangle against hull
is within 10% of the reference both cold and warm; the capsule is 1.35x
(its GJK); the sphere, which is the closest point on a triangle and
nothing else, is faster here.

## mesh

`bench/mesh.ae` and `bench/mesh_box3d.c`: a 200 x 200 wave mesh of
80,000 triangles built ten times with the median split and ten with the
binned SAH, edges identified each time; 100,000 rays cast down onto it;
100,000 box queries over it; 10,000 sphere shape casts onto it.

| phase | aephysics | Box3D |
|---|---|---|
| 10 builds, median split (80,000 triangles) | 221 ms | **129** |
| 10 builds, binned SAH | 335 | **256** |
| 100,000 ray casts | 20.8 | **10.2** |
| 100,000 box queries | 70.4 | **39.5** |
| 10,000 shape casts | 24.7 | **14.3** |

The same trees come out: 43,135 nodes of height 16 from the median
split and 48,063 of height 16 from the SAH on both; every ray and cast
hits on both with equal sums; the box query reports 2,218,859 triangles
here against 2,218,676 there (0.008% more, boundary cases of the
reference's SIMD box-triangle test against the scalar one -- false
positives the query permits). The build is 1.2-1.5x, with the welding
map and the edge map through core's LongMap; the traversals are 1.7-2x,
the reference's SIMD box tests against scalar ones on 48-byte double
boxes, the same gap the dynamic tree's ray cast showed.

## height_field

`bench/height_field.ae` and `bench/height_field_box3d.c`: a 512 x 512
wave field of 522,242 triangles with a hole every sixteenth cell, built
ten times; 100,000 rays cast down onto it at slight angles; 100,000 box
queries over it; 10,000 sphere shape casts onto it; 10,000 sphere
overlaps at its surface.

| phase | aephysics | Box3D |
|---|---|---|
| 10 builds (522,242 triangles) | 200 ms | **131** |
| 100,000 ray casts | 16.0 | **11.3** |
| 100,000 box queries | **8.3** | 9.0 |
| 10,000 shape casts | 49.5 | **23.5** |
| 10,000 overlaps | 3.7 | **2.0** |

The same fields come out: every ray hits on both (94,058, the sum of hit
heights within 0.1%, the quantised heights held as doubles here and
floats there), every shape cast hits on both with equal fraction sums,
the overlaps agree (3,160), the box query reports 2,171,942 triangles
here against 2,171,914 there (0.001% more, boundary cases of the cell
bounds test). The build is 1.5x, the quantisation and the edge flags of
half a million triangles; the ray walk 1.4x; the box query at parity
(no SIMD in the reference's); the shape cast 2.1x, the GJK cast per
straddled cell on top of distance's 1.3-1.5x; the overlap 1.85x. The
field is 4.2 MB here against 1.3 MB there: the reference packs 16-bit
heights and 8-bit materials and flags, which Aether cannot yet address,
so they are ints.

## shape

`bench/shape.ae` and `bench/shape_box3d.c`: 1,000,000 rays at a unit
sphere and 1,000,000 at a capsule from a fan of origins, a third of them
missing; 100,000 casts of a sphere at a capsule under a transform;
100,000 overlaps of a sphere with it; 100,000 mover planes against it;
100,000 capsule masses.

| phase | aephysics | Box3D |
|---|---|---|
| 1,000,000 sphere ray casts | 33.2 ms | **31.5** |
| 1,000,000 capsule ray casts | **41.0** | 49.7 |
| 100,000 shape casts | 44.6 | **18.0** |
| 100,000 overlaps | 12.2 | **5.9** |
| 100,000 mover planes | 6.9 | **4.8** |
| 100,000 capsule masses | 4.5 | **4.3** |

The same answers: the ray hits agree to one boundary case in a million
(349,452 sphere hits here against 349,451, the same 595,654 on the
capsule) with equal fraction sums, the casts hit the same 89,582 times
with equal sums, the overlaps (51,518), the mover's planes (45,071) and
the masses agree. The sphere ray and the masses are at parity, the
capsule ray is 0.8x (the reference's float division by the axis length
against our double), the mover 1.45x. The sphere-against-capsule cast
is 2.5x and the overlap 2x: both are one GJK call on proxies with
radii, where the box-against-box work of the distance layer sat at
1.3-1.5x; the radius handling in `distance.shape_cast` and
`shape_distance` is the place to profile (aephysics#9).

## compound

`bench/compound.ae` and `bench/compound_box3d.c`: a compound of 2,000
children (1,000 spheres, 500 capsules and 500 instances of one box hull
on a 20 x 10 x 10 grid, two materials) built ten times; 100,000 rays
through it; 100,000 box queries over it; 10,000 sphere shape casts down
through it; 10,000 overlaps and 10,000 mover planes under a transform.

| phase | aephysics | Box3D |
|---|---|---|
| 10 builds (2,000 children) | **9.9 ms** | 14.3 |
| 100,000 ray casts | 75.4 | **25.7** |
| 100,000 box queries | 8.7 | **6.5** |
| 10,000 shape casts | 10.7 | **5.0** |
| 10,000 overlaps | 1.5 | **0.9** |
| 10,000 mover planes | 1.2 | **0.9** |

The same answers: 50,450 ray hits with equal sums of fraction and child
index, 777,892 query visits, 5,098 cast hits, 3,278 overlaps, 5,090
mover planes with equal offset sums. The build is faster here (0.7x;
the content maps are core's LongMap on the hull's and the mesh's own
hash, the reference rehashes every block through a generic table); the
queries, overlaps and movers 1.3-1.6x; the shape cast 2.1x (issue #9's
radius GJK). The ray cast is 2.9x: a probe with a visitor that does
nothing shows the time in `dynamic_tree.tree_ray_cast` itself (77 ms
for 520,000 leaf visits), not in the children -- the tree layer showed
the same cast at 1.9x on a sparser scene, the reference's SIMD slab
test against scalar doubles. The traversal is the place to profile
(aephysics#11). The compound is 379 KB here against 231 KB there: the
tree's nodes and proxies are our wider doubles and longs, and the
block carries the traversal stack.

## mover

`bench/mover.ae` and `bench/mover_box3d.c`: 1,000,000 solves of a
target against six planes tilted around it (a floor, four walls leaning
in, a soft ceiling), each followed by a velocity clip.

| phase | aephysics | Box3D |
|---|---|---|
| 1,000,000 plane solves and clips | **64.6 ms** | 73.2 |

The same answers: equal sums of the deltas and of the clipped
velocities, 1,662,623 iterations here against 1,662,624 there (one
convergence test on the float's side of the slop). The solver runs
at 0.9x: the reference reads its planes through a pointer per pass
where the loop here indexes the array.

## broad_phase

`bench/broad_phase.ae`: 10,000 dynamic unit boxes on a jittered 25 x 16
x 25 grid at 1.4 spacing over a static ground, the first update finding
every pair, then 20 steps in which every box drifts a little and only
the new pairs come out, each update's keys adopted into the pair set.
The reference finds its pairs inside its world (broad_phase.c filters
through the shapes and creates the contacts itself), so it has no
free-standing counterpart; its pair update is measured against ours
through the world benchmarks once the world steps.

| phase | aephysics |
|---|---|
| 10,000 proxies created | 3.5 ms |
| first update (2,900 pairs) | 2.2 |
| 20 steps of 10,000 moves and an update (757 new pairs) | 91 |

A step is 4.6 ms, most of it the 10,000 proxy moves (a leaf removed and
re-inserted each); the update itself walks only the sibling pairs a
moved node touched. The test checks every update against a brute force
over the boxes.

The hash under the pair set and every map (core's `key_hash`) was
rewritten during this layer: the previous 64-bit mixer multiplied
signed longs, which is undefined in the C underneath, and gcc at -O2
made two inlined copies of it disagree, so a key stored by one copy was
not found by the other. The new mixer stays in 32-bit products; the
mesh builds above moved from 199 to 221 ms and 316 to 335 ms with it.

## mesh_contact

`bench/mesh_contact.ae`: a box, a sphere and a capsule each dragged
100,000 steps across a 100 x 100 wave mesh, riding a hundredth inside
the surface with a small move each step, the manifolds computed every
step. The reference's mesh contact lives inside its world (it needs a
contact, a worker context and the world's material callbacks), so it
has no free-standing counterpart; it is measured against ours through
the world benchmarks once the world steps.

| shape, 100,000 steps | aephysics | per step |
|---|---|---|
| box (0.8 wide): 876,526 clusters, 3.17 M points, 910,432 cache hits | 322 ms | 3.2 us |
| sphere: 127,842 clusters, 129,088 points | 69 ms | 0.7 us |
| capsule: 797,124 clusters, 1.35 M points | 371 ms | 3.7 us |

The wave's cells are half the box's width, so a box straddles several
triangles of different normals and the clusters stay many (the cluster
threshold is cos 5 degrees); the far clip points of a hull face are
kept as speculative ones, as the reference keeps them. The test checks
the seams (a sphere on an interior edge or vertex gives one point), the
cache's persistence and the cull.

## dynamics (bookkeeping, contacts, joints, sensors)

`bench/dynamics.ae` and `bench/dynamics_box3d.c`: ten rounds of a world
with 5,000 dynamic bodies (a box hull and an offset sphere each, the
mass computed from both) and 500 static ones, the mass summed, every
dynamic body's transform set once (its shapes' bounds and proxies
updated), then the world destroyed; then 5,000 cubes resting on a slab
through eleven collide passes with no solve (the reference's step with
a zero time step does exactly that: the pairs, the narrow phase, the
state changes): the first begins every contact, the rest recycle them;
then those cubes chained by 4,999 revolute joints (each created into the
constraint graph and linked into the islands, which merge into one),
then every joint destroyed; then 2,500 sensor spheres over 2,500 static
boxes through eleven sensor passes (each with the pairs and the narrow
phase before it, as the reference's zero-time step runs them).

| phase | aephysics | Box3D |
|---|---|---|
| 5,500 bodies and 10,500 shapes created, 10 rounds | **112 ms** | 120 |
| 5,000 transforms set, 10 rounds | **39** | 55 |
| the world destroyed, 10 rounds | 5.6 | **4.7** |
| first collide: 5,000 contacts begun, islands linked | 7.2 | **4.6** |
| 10 recycling collides | 4.0 | **2.5** |
| 4,999 revolute joints created, 5,000 islands merged into one | **2.9** | 5.7 |
| the joints destroyed | 0.23 | **0.11** |
| first sensor pass: 2,500 begin events | 1.4 | **1.0** |
| 10 more sensor passes | 8.7 | **7.0** |

The same mass sum (53,272.5), the same 5,000 contacts and 5,000
islands, 4,999 joints and one island, 2,500 begin events.
Creation is at parity: the reference's hull database hashes every hull
shape's bytes and refcounts them, which this layer does not do (a hull
shape keeps the pointer it was given). The transforms are 0.7x: the
reference's b3Body_SetTransform also walks the body's joints, which
arrive with the joint layer. The collide passes are 1.7-1.8x: each
contact is reached through its id into a 300-byte struct here where the
reference's narrow phase prefetches the next contact and packs its
state; the first pass also computes 5,000 hull-hull manifolds (the
distance layer's 1.3-1.5x) and links 5,000 islands. The wide contact
solver will read these arrays, so this is the place to come back to
with the solver's profile. Joint creation is 0.5x: the reference's
b3CreateJoint records the call for its replay and validates the joint
definition's cookie; the destruction is 2x on a tenth of a millisecond.
The sensor passes are 1.2-1.4x: each pass queries the three trees per
sensor and runs GJK on every candidate, the distance layer's gap.

## contact_solver

`bench/contact_solver.ae` (ours alone: the reference has no solve
without its step, which integrates too; the step's pair comes with the
world layer): 5,000 cubes resting on a slab and 1,000 stacked in pairs,
collided once, then ten steps of the passes as the solver runs them:
the colours' constraints prepared, warm started, four sub-steps of a
biased solve and four relaxes (the relax carries the friction, twist
and rolling), the restitution pass and the impulses stored.

| pass, 10 steps of 6,000 contacts | aephysics |
|---|---|
| prepare | 7.0 ms |
| warm start | 1.9 |
| 4 biased solves | 20.1 |
| 4 relaxes (with friction) | 27.6 |
| restitution | 0.13 |
| store | 2.5 |

About 85 ns per contact per biased solve and 115 ns with friction, on
one core, scalar. The reference solves its convex contacts eight at a
time in SIMD lanes (b3ContactConstraintWide); that path is the layer
to add and measure once the step exists to compare against.

## joint_solver

`bench/joint_solver.ae` (ours alone, as the contact solver's): 5,000
cubes in a row chained by 4,999 joints of one kind, the row given a
sideways velocity gradient, ten steps of the passes: prepared, warm
started, four biased sub-step solves and four relaxes.

| pass, 10 steps of 4,999 joints | revolute (limited) | spherical (cone and twist) | weld | prismatic (limited, sprung) |
|---|---|---|---|---|
| prepare | 4.1 ms | 3.9 | 2.8 | 2.9 |
| warm start | 1.0 | 1.0 | 0.9 | 1.5 |
| 4 biased solves | 24.7 | 21.5 | 22.0 | 19.2 |
| 4 relaxes | 25.2 | 21.7 | 16.0 | 13.6 |

About 0.5 to 0.6 microseconds per joint per solve, on one core. The
reference solves joints the same way (scalar, per graph colour); its
numbers come with the step's pair.

## solver

`bench/solver.ae` and `bench/solver_box3d.c`: the step end to end (the
pairs, the narrow phase, the Soft Step) against the reference's
b3World_Step, one thread, 120 steps of 1/60 with four sub-steps: 5,000
cubes dropped from a metre onto a slab in a 100 by 50 grid (they land,
slide and settle), and 100 stacks of 10 cubes (dynamic pairs in every
colour).

| scene, 120 steps | scalar | lanes in Aether | native lanes | Box3D |
|---|---|---|---|---|
| 5,000 cubes falling onto a slab | 252 ms | 230 | 187 | **157** |
| 100 stacks of 10 cubes | 50 | 46 | 40 | **28** |

The same height sums (2,499.65 and 4,983.87), every body asleep at the
end, the same contact counts (5,000 and 1,000), in every column. The
convex contacts three ways (issue #22's measure): solved one at a time
through `contact_solver` (scalar; 308 and 61 ms before the [build
flags](#build-flags-inlining)); four at a time through
`contact_solver_wide`'s lanes in Aether, the reference's layout as
plain code (the layout alone: 10%); and through the same lanes in
`aephysics_native.c`, GCC vector code in single precision (another 20-30%). The
falling grid is mostly the narrow phase and the pairs (the cubes land
and settle); the stacks are the solver's, 1.4x.

## physics_world

`bench/physics_world.ae` and `bench/physics_world_box3d.c`: the
reference's own benchmark scenes (its shared/benchmarks.c) stepped by
our world and by b3World_Step, one thread, 1/60 with four sub-steps,
sleeping off as the reference runs them.

| scene | aephysics per step | Box3D per step |
|---|---|---|
| large pyramid: 5,050 cubes on a base of 100, 200 steps | 14.1 ms | **9.0** |
| many pyramids: 196 pyramids of base 10, 10,780 cubes, 100 steps | 31.6 | **19.3** |
| joint grid: 10,000 spheres on 19,800 spherical joints, 100 steps | 11.7 | **10.7** |

The same height sums (167,556, 37,695 and -496,893), contact counts
(14,950 and 28,420) and joint count. One run of the pair, on a machine
whose other work moves both columns by 10-20% between runs (the joint
grid has measured 11.1 to 14.1 ms, the reference 9.1 to 11.8): the
ratios hold, 1.1x on the joint grid (no contacts: single against
double precision and what the emitted C still loses) and 1.6x on the
pyramids (24.5 and 53.0 ms before the wide path and the flags,
2.2-2.4x; 17.1 and 41.5 before the parallel layer's stack allocator
stopped zeroing). `scripts/profile.sh bench/physics_world.ae` on the large
pyramid alone, and the reference's benchmark under the same sampler
(`tools/sampler.c` linked into its C), ms per step at 60 steps:

| stage | aephysics | Box3D |
|---|---|---|
| contact solve (with the gather and scatter) | 4.5 | 4.5 |
| contact prepare | 2.5 | 1.3 |
| narrow phase and recycling | 1.9 | 1.2 |
| warm start | 1.0 | 0.7 |
| pack to floats and unpack | 1.0 | -- |
| the solver's own stages (integrate, finalize, blocks) | 1.9 | 1.0 |
| store | 0.3 | 0.5 |
| step | 14.7 | 9.7 |

The solve is at parity: the same lanes, the same precision. What is
left is the prepare (Aether writing a lane at a time into a
double-sized structure), the packing the doubles need, and a narrow
phase in doubles (issue #17).

## parallel

`bench/parallel.ae` and `bench/parallel_box3d.c`: the three scenes
above by worker count, ours through `WorldDef.worker_count` and the
reference's through `b3WorldDef.workerCount` on its built-in
scheduler, sleeping off, ms per step; 1, 2, 4 and 8 workers and the
machine's 24 processors. The height sums are the same at every count
on both sides.

| scene | workers | aephysics | Box3D |
|---|---|---|---|
| large pyramid | 1 | 13.3 | **9.1** |
| | 2 | 8.0 | **4.8** |
| | 4 | 4.6 | **2.9** |
| | 8 | 3.3 | **2.0** |
| | 24 | 2.9 | **1.8** |
| many pyramids | 1 | 32.0 | **20.6** |
| | 2 | 17.6 | **10.4** |
| | 4 | 10.8 | **5.3** |
| | 8 | 6.8 | **3.5** |
| | 24 | 5.6 | **2.8** |
| joint grid | 1 | 12.2 | **11.4** |
| | 2 | 6.2 | **5.6** |
| | 4 | 3.8 | **3.3** |
| | 8 | 2.4 | **2.1** |
| | 24 | 2.2 | **1.6** |

The scaling is the reference's (4.0x, 4.7x and 5.1x at eight workers
against its 4.4x, 5.8x and 5.4x) and the ratio between the columns at
eight workers is the single-thread one, so what is left is the
per-thread work, not the layer. The profile the world keeps
(`physics_world.get_profile`, the reference's b3Profile) says where
the step goes at eight workers on the large pyramid: constraints 2.2
ms (the stages), collide 0.5, pairs 0.3 (the broad phase, serial here
and a task in the reference), transforms 0.14, setup 0.01.

Two things the layer found on the way. A task on a fresh thread
(std.worker's run_detached spawns one per call) costs 180 us a round
of seven; the scheduler now makes its threads once and they wait on a
semaphore, and a round is 8 us. And the stack allocator zeroed every
allocation -- the constraint blocks are tens of megabytes a step --
where the reference's does not: a millisecond a step on the pyramids,
gone, with the memory poisoned under the determinism scenes to show
nothing reads it before writing.

## human

`bench/human.ae` and `bench/human_box3d.c`: the reference's "rain"
benchmark (shared/benchmarks.c, its shared library linked for the
figure and the scene): a ten by ten grid of cells, each a grid mesh with
a torus on it; every 48 steps a column of cells gets three ragdolls
(twelve capsule bones, eleven joints with limits, springs and motors)
dropped from twenty metres, the columns recycled once full. 400 steps
of 1/60 with four sub-steps, one thread.

| | aephysics | Box3D |
|---|---|---|
| 400 steps, 3,700 bodies and 4,200 joints at the end | 4.61 ms per step | **4.40** |

1.05x, the closest pair yet: the step is joints and capsule contacts,
which the flags of PR #24 and the lanes of PR #25 brought to parity,
and the meshes' contacts, the bookkeeping of 300 figures created and
destroyed in turn. Both build the same 3,700 bodies and 4,200 joints
on the same steps; the resting state differs (7,208 contacts and 2,892
awake against 6,831 and 3,144, the moved bodies' height sum 24,019
against 24,187): eight figures tumbling for six seconds is chaotic,
and single against double precision parts them, where the resting
pyramids of physics_world agree to six digits.

## Build flags (inlining)

The generated C of every Aether function is a plain `static` function,
never `static inline`, and gcc's -O2 keeps the small maths (a 3x3
product, a rotation, a quaternion product) out of line: a sampling
profile of the joint grid at -O2 had `math_mul_mm` at 15%,
`math_rotate_vector` at 8%, `math_solve3`, `math_mul_quat` and
`math_inv_mul_quat` at 4-6% each, all as calls, where the reference's
`static inline` headers vanish into the joint solve. Marking the math
module's functions `static inline` by hand in the emitted C took the
joint grid from 18.9 to 11.9 ms per step; the same effect from the
outside is gcc's inlining budget, so `aether.toml` builds this repo
with

    -O3 --param max-inline-insns-auto=400 --param inline-unit-growth=400

(`ae build` reads it for the tests and the benchmarks; a program that
uses aephysics as a library sets its own). Measured on the joint grid,
one run each, ms per step: -O2 18.9; -O3 16.9; -O3 -march=native 16.2;
-O2 with the inline parameters at 200: 12.5; -O3 with them at 400:
11.2; the hand-inlined C at -O2: 11.9. The pyramids gain 10% from the
same flags (their time is the contact solver's loops, already inlined).
The compile is a third slower and the executable 1.7x larger. Turning
every double into a float in the emitted C (an experiment, not a
change) made the grid slower, 26 ms, so precision is not where the time
goes at present. An emitter-side `static inline` for small leaf
functions is asked of Aether in aether-lang-dev/aether#2123.
