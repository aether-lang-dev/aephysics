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
| 10 builds, median split (80,000 triangles) | 199 ms | **129** |
| 10 builds, binned SAH | 316 | **256** |
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
