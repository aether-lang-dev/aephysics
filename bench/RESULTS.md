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

## dynamics (bookkeeping)

`bench/dynamics.ae` and `bench/dynamics_box3d.c`: ten rounds of a world
with 5,000 dynamic bodies (a box hull and an offset sphere each, the
mass computed from both) and 500 static ones, the mass summed, every
dynamic body's transform set once (its shapes' bounds and proxies
updated), then the world destroyed. No step.

| phase, 10 rounds | aephysics | Box3D |
|---|---|---|
| 5,500 bodies and 10,500 shapes created | **113 ms** | 122 |
| 5,000 transforms set | **38** | 55 |
| the world destroyed | 5.0 | **5.1** |

The same mass sum (53,272.5). Creation is 0.93x: the reference's hull
database hashes every hull shape's bytes and refcounts them, which this
layer does not do (a hull shape keeps the pointer it was given). The
transforms are 0.7x: the reference's b3Body_SetTransform also invalidates
the body's contacts and validates, which arrive with the contact layer;
the comparison will be redone then.
