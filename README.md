# aephysics

[![Test](https://github.com/aether-lang-dev/aephysics/actions/workflows/test.yaml/badge.svg)](https://github.com/aether-lang-dev/aephysics/actions/workflows/test.yaml)

A 3D rigid-body physics engine written in [Aether](https://github.com/aether-lang-dev/aether).

aephysics follows the design of [Box3D](https://github.com/erincatto/box3d),
Erin Catto's engine: the Soft Step solver with sub-stepping, a
graph-coloured contact solver eight lanes wide, sleeping islands and
continuous collision. It was written module by module against Box3D's own
tests, and is measured against Box3D built on the same machine: on Box3D's
benchmark suite it runs within 4 to 21 percent of Box3D's time per step on
one thread.

It is written in Aether rather than wrapped around a C library (one small C
file holds the thread helpers Aether lacks), so an Aether program can read,
test and change its physics in its own language. It is the physics engine
of [ae3d](https://github.com/aether-lang-dev/ae3d).

## Features

- **Bodies:** static, kinematic and dynamic, with damping, gravity scale,
  motion locks, sleeping, bullets and continuous collision for fast bodies.
- **Shapes:** spheres, capsules, convex hulls (quickhull), triangle meshes,
  height fields and baked compounds. Materials carry friction, restitution,
  rolling resistance and conveyor speeds.
- **Joints:** distance, motor, filter, parallel, prismatic, revolute,
  spherical, weld and wheel, with springs, motors and limits. A spherical
  joint's cone can sit off its axis and be elliptical.
- **Solver:** Soft Step with sub-steps; convex contacts solved eight at a
  time in Aether's SIMD lanes; an AVX2 build picked at run time when the
  CPU has it.
- **Threads:** a step runs over any number of worker threads, or on a job
  system the host lends it. The result is the same at any worker count.
- **Queries:** ray casts, shape casts, overlaps and the character mover,
  callable from any thread between steps.
- **Events:** contacts beginning and ending, hits, sensors, bodies moving and
  joints under load; pre-solve and custom filter callbacks.
- **Ragdolls:** a twelve-bone figure with a person's joint ranges, optional
  ankles, and pose drive through joint springs or kinematic anchors.
- **Tooling:** snapshots that load and step on bit for bit, recording and
  replay of every call with seeking, debug drawing, and host hooks for the
  allocator, the log and assertions.
- **Large worlds:** positions in double precision, so a scene ten thousand
  kilometres from the origin behaves as one at it.
- **Determinism:** the same scene gives the same bits on Linux, macOS and
  Windows, at any worker count and SIMD width, optimized or not. CI checks
  every step of five scenes against a golden trace.

## Quick start

Install Aether with the version-pinned installer (it puts `ae` and `aeb` in
`~/.local/bin`):

```sh
curl -fsSL https://raw.githubusercontent.com/aether-lang-dev/aeb/main/get.sh \
  | AE_PIN=0.839.0 AEB_REF=v0.330 sh
export PATH="$HOME/.local/bin:$PATH"
```

Add aephysics to your project:

```sh
ae add github.com/aether-lang-dev/aephysics
```

`ae` reads C flags only from your own `aether.toml`, so give your build the
one flag the engine's determinism needs, and the inlining budget its
benchmarks are measured with:

```toml
[build]
cflags = "-O3 --param max-inline-insns-auto=400 --param inline-unit-growth=400 -ffp-contract=off"
```

Then drop a box on the ground (a shortened [examples/hello_world.ae](examples/hello_world.ae)):

```aether
import aephysics.math
import aephysics.hull
import aephysics.dynamics
import aephysics.physics_world

main() {
    world_def = dynamics.default_world_def()
    world_def.gravity = math.vec3(0.0, 0.0 - 10.0, 0.0)
    world = dynamics.create_world(&world_def)

    ground_def = dynamics.default_body_def()
    ground_def.position = math.vec3(0.0, 0.0 - 10.0, 0.0)
    ground = dynamics.create_body(world, &ground_def)
    ground_box = hull.make_box_hull(50.0, 10.0, 50.0)
    ground_shape_def = dynamics.default_shape_def()
    dynamics.create_hull_shape(ground, &ground_shape_def, ground_box)

    body_def = dynamics.default_body_def()
    body_def.kind = dynamics.BODY_DYNAMIC
    body_def.position = math.vec3(0.0, 4.0, 0.0)
    body = dynamics.create_body(world, &body_def)
    cube = hull.make_box_hull(1.0, 1.0, 1.0)
    shape_def = dynamics.default_shape_def()
    shape_def.density = 1.0
    dynamics.create_hull_shape(body, &shape_def, cube)

    step = 0
    while step < 90 {
        physics_world.step(world, 1.0 / 60.0, 4)
        step = step + 1
    }
    p = dynamics.body_get_position(body)
    println("the cube rests at y = ${p.y}")

    dynamics.destroy_world(world)
    hull.destroy_hull(ground_box)
    hull.destroy_hull(cube)
}
```

```sh
ae build hello_world.ae -o hello_world && ./hello_world
```

If you know Box3D, you already know the API. Each function is Box3D's
without the `b3` prefix, in snake case. Creation and the body, shape and
joint calls are in `dynamics` (`b3Body_GetPosition` is
`dynamics.body_get_position`). Stepping, settings, events and queries are
in `physics_world` (`b3World_Step` is `physics_world.step`).

## Documentation

- [The guide](docs/guide.md): worlds, bodies, shapes, joints, events,
  queries, threads, ragdolls, snapshots and replay.
- [Examples](examples/): the hello world above; a chain of joints with
  events and queries; a ragdoll saved and replayed bit for bit. CI builds
  and runs every one.
- [Architecture](docs/architecture.md): the modules, how a step runs, and
  where aephysics departs from Box3D.
- [Testing](docs/testing.md): the test suite, the determinism trace, CI and
  the benchmarks.
- [Benchmark record](bench/RESULTS.md): every measurement against Box3D and
  Jolt, with its date and machine.

## Performance

Box3D's own benchmark scenes, one thread, both engines built with AVX2 on
the same machine (Box3D at commit `e77352c`), aephysics' time per step over
Box3D's:

| scene | aephysics / Box3D |
|---|---|
| large pyramid: 5,050 boxes | 1.20x |
| many pyramids: 10,780 boxes | 1.16x |
| joint grid: 10,000 spheres on 19,800 joints | 1.05x |
| junkyard: 10,584 rocks swept by a kinematic cylinder | 1.04x |
| washer: 8,000 cubes in a kinematic drum | 1.11x |
| convex pile: 5,120 hulls of 32 points | 1.21x |
| rain: 300 ragdolls dropped on meshes | 1.05x |

The milliseconds, the other scenes and how each was run are in the record:
[the pyramids and the grid](bench/RESULTS.md#the-collide-pass-as-the-reference-keeps-it-2026-10-09-121),
[the rest of the suite](bench/RESULTS.md#suite-the-references-benchmark-suite)
and [the rain](bench/RESULTS.md#human). The rain was timed when the ragdoll
layer landed and the rest on 2026-10-09; the wide separating axis test and
the wide solver's transposes have narrowed the gaps since. Where nothing is
chaotic, the two engines agree to the printed digit.

Box3D's design was chosen by measurement. On identical scenes its solver
held a 100-row pyramid at 9.6 ms a step where Jolt's collapsed at 44.5, and
given the same budget as Jolt at its stiffest it was tighter on every scene
([the bake-off](bench/RESULTS.md#the-bake-off)).

## Building and testing

```sh
scripts/test.sh                  # every module checked, every test and example built and run
scripts/golden_trace.sh check    # the determinism trace against tests/golden (test.sh runs it)
scripts/fetch_references.sh      # Box3D and Jolt into reference/, built (needs cmake, ninja, gcc)
scripts/bench.sh [layer|suite]   # a layer, or Box3D's suite, against the reference
scripts/profile.sh file.ae       # a sampling profile on Windows
```

CI runs the suite on Linux, on Linux with AVX2 and FMA, on macOS arm64, and
on Windows at `-O3` and `-O0`. It pins one Aether release, 0.839.0; older
ones are not tested.

## Credits

The design, and the tests aephysics is held to, are Box3D's, by Erin Catto
(MIT). Jolt, by Jorrit Rouwe (MIT), is the second baseline. Neither is
redistributed here: `scripts/fetch_references.sh` fetches both, unmodified,
into an ignored `reference/`.

aephysics is by Nicolas Maman, under the [MIT license](LICENSE).
