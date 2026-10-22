# Using aephysics

This guide walks through a program's view of the engine: worlds, bodies,
shapes, joints, stepping, events, queries, threads and the tools around
them. Every call named here is real; the [examples](../examples) put most of
them in programs that CI builds and runs.

The API is Box3D's. A function is Box3D's without its `b3` prefix, in snake
case, and a test or sample written against Box3D reads the same here:

| Box3D | aephysics |
|---|---|
| `b3CreateWorld(&def)` | `dynamics.create_world(&def)` |
| `b3World_Step(id, dt, n)` | `physics_world.step(id, dt, n)` |
| `b3CreateBody(world, &def)` | `dynamics.create_body(world, &def)` |
| `b3Body_GetPosition(id)` | `dynamics.body_get_position(id)` |
| `b3CreateHullShape(body, &def, hull)` | `dynamics.create_hull_shape(body, &def, hull)` |
| `b3RevoluteJoint_SetMotorSpeed(id, s)` | `joint_solver.revolute_joint_set_motor_speed(id, s)` |

## Contents

1. [Modules a program imports](#modules-a-program-imports)
2. [Units and ids](#units-and-ids)
3. [Worlds and stepping](#worlds-and-stepping)
4. [Bodies](#bodies)
5. [Shapes](#shapes)
6. [Materials and filtering](#materials-and-filtering)
7. [Joints](#joints)
8. [Events](#events)
9. [Queries](#queries)
10. [Threads](#threads)
11. [Determinism](#determinism)
12. [Ragdolls](#ragdolls)
13. [Snapshots, recording and replay](#snapshots-recording-and-replay)
14. [Debug drawing](#debug-drawing)
15. [Fitting into a host engine](#fitting-into-a-host-engine)
16. [Where aephysics goes past Box3D](#where-aephysics-goes-past-box3d)

## Modules a program imports

Aether binds a module by its last name, so `import aephysics.dynamics` is
used as `dynamics.`. Most programs need four or five of these:

| module | what a program uses it for |
|---|---|
| `aephysics.math` | `Vec3`, `Quat`, `Transform`, `AABB` and their arithmetic (`math.vec3`, `math.quat_identity`, `math.transform`) |
| `aephysics.dynamics` | creating and destroying worlds, bodies, shapes and joints; every body, shape and joint accessor; the definitions and their defaults |
| `aephysics.physics_world` | stepping, the world's settings and callbacks, events, queries, explosions, counters, drawing |
| `aephysics.hull` | convex hulls: boxes, cylinders, cones, rocks, and hulls of any points |
| `aephysics.manifold` | the `Sphere` and `Capsule` values shapes are made from (`manifold.sphere`, `manifold.capsule`) |
| `aephysics.mesh`, `aephysics.height_field`, `aephysics.compound` | triangle meshes, height fields and baked compounds |
| `aephysics.shape` | collision and query filters (`shape.default_filter`, `shape.default_query_filter`) |
| `aephysics.material` | `SurfaceMaterial` and `material.default_surface_material` |
| `aephysics.joint_solver` | each joint kind's accessors: limits, springs, motors, angles, forces |
| `aephysics.human` | the ragdoll |
| `aephysics.snapshot`, `aephysics.recording` | saving and loading worlds; recording and replaying them |
| `aephysics.debug_draw` | the callbacks a renderer draws a world with |
| `aephysics.basics` | the host's allocator, log and assert hooks; the SIMD width |

The rest (`solver`, `contact_solver`, `broad_phase`, `dynamic_tree`,
`distance`, `sat`, ...) are the engine's own layers. A program can import
them, and the tests do, but nothing above needs to.

## Units and ids

Lengths are in metres, masses in kilograms, time in seconds and angles in
radians. The engine is tuned, as Box3D is, for objects between about 10 cm
and 10 m. Positions are double precision, so a world can sit far from the
origin (the tests run a stack and a bullet at x = 10,000 km).

Gravity defaults to (0, -10, 0). Nothing else assumes which way is up.

Worlds, bodies, shapes and joints are handed out as ids (`WorldId`,
`BodyId`, `ShapeId`, `JointId`): small values that carry a slot and its
generation. Once its object is destroyed an id fails
`dynamics.body_is_valid` (or `shape_is_valid`, `joint_is_valid`,
`physics_world.is_valid`), even after the slot is reused. Check an id you
kept from an earlier step before you use it.

## Worlds and stepping

```aether
world_def = dynamics.default_world_def()
world_def.gravity = math.vec3(0.0, 0.0 - 9.81, 0.0)
world_def.worker_count = 4
world = dynamics.create_world(&world_def)

while running {
    physics_world.step(world, 1.0 / 60.0, 4)
}

dynamics.destroy_world(world)
```

`step(world, time_step, sub_step_count)` advances the world by
`time_step` seconds, solved in `sub_step_count` sub-steps. Four sub-steps at
60 Hz is Box3D's recommendation and what every benchmark here uses; more
sub-steps make stacks and joint chains stiffer at a proportional cost.
Destroying a world destroys everything in it.

The world definition's fields, with their defaults:

| field | default | meaning |
|---|---|---|
| `gravity` | (0, -10, 0) | m/s² |
| `worker_count` | 1 | threads a step runs over (see [Threads](#threads)) |
| `scheduler` | null | a host's job system to step on instead |
| `enable_sleep` | true | resting islands go to sleep |
| `enable_continuous` | true | fast bodies are swept against static geometry |
| `contact_hertz`, `contact_damping_ratio` | 30, 10 | the contact spring |
| `contact_speed` | 3 | the fastest a contact pushes overlapping bodies apart, m/s |
| `restitution_threshold` | 1 | approach speed below which nothing bounces |
| `hit_event_threshold` | 1 | approach speed that makes a hit event |
| `maximum_linear_speed` | 400 | m/s |
| `friction_callback`, `restitution_callback` | geometric mean, maximum | how two materials mix |
| `enable_kinematic_static_softness`, `enable_kinematic_speculation`, `enable_travel_speculation` | false | see [Where aephysics goes past Box3D](#where-aephysics-goes-past-box3d) |

Most settings can change between steps through `physics_world`:
`enable_sleeping`, `enable_continuous`, `enable_warm_starting`,
`set_gravity`, `set_contact_tuning`, `set_restitution_threshold`,
`set_hit_event_threshold`, `set_maximum_linear_speed`,
`set_worker_count`, and the callbacks `set_friction_callback`,
`set_restitution_callback`, `set_custom_filter_callback` and
`set_pre_solve_callback`.

`physics_world.get_counters(world)` reports the world's sizes (bodies,
shapes, contacts, joints, islands, the bytes it holds, the tasks it ran),
`get_profile(world)` the milliseconds each stage of the last step took, and
`dump_memory_stats(world)` the memory part by part to the host's log.
`physics_world.explode(world, &def)` pushes bodies away from a point
(`default_explosion_def()`: position, radius, falloff, impulse per area).

## Bodies

```aether
body_def = dynamics.default_body_def()
body_def.kind = dynamics.BODY_DYNAMIC
body_def.position = math.vec3(0.0, 2.0, 0.0)
body_def.linear_velocity = math.vec3(1.0, 0.0, 0.0)
body = dynamics.create_body(world, &body_def)
```

A body is one of three kinds:

- `BODY_STATIC` (the default): never moves; the ground and walls. Static
  bodies cost nothing to step.
- `BODY_KINEMATIC`: moved by its velocity, unaffected by forces and
  contacts; platforms, doors, a character's capsule driven by the game.
  `body_set_target_transform(body, target, time_step, wake)` sets the
  velocity that reaches a transform in one step.
- `BODY_DYNAMIC`: moved by gravity, forces, contacts and joints. Its mass
  comes from its shapes' densities unless set by hand.

What a body definition holds besides the kind: `position`, `rotation`,
`linear_velocity`, `angular_velocity`, `linear_damping`,
`angular_damping`, `gravity_scale`, `sleep_threshold`, `motion_locks` (any
of the six axes held fixed), `enable_sleep`, `is_awake`, `is_bullet`,
`is_enabled`, `allow_fast_rotation`, `name` and `user_data`.

The calls on a body, all in `dynamics`:

| what | calls |
|---|---|
| where it is | `body_get_position`, `body_get_rotation`, `body_get_transform`, `body_set_transform`, `body_get_world_point`, `body_get_local_point`, `body_get_world_center` |
| how it moves | `body_get_linear_velocity`, `body_set_linear_velocity`, `body_get_angular_velocity`, `body_set_angular_velocity`, `body_get_world_point_velocity` |
| pushing it | `body_apply_force(body, force, point, wake)`, `body_apply_force_to_center`, `body_apply_torque`, `body_apply_linear_impulse`, `body_apply_linear_impulse_to_center`, `body_apply_angular_impulse` |
| its mass | `body_get_mass`, `body_get_mass_data`, `body_set_mass_data`, `body_apply_mass_from_shapes`, `body_get_local_rotational_inertia` |
| its behaviour | `body_set_linear_damping`, `body_set_angular_damping`, `body_set_gravity_scale`, `body_set_motion_locks`, `body_set_bullet`, `body_set_type`, `body_disable`, `body_enable` |
| sleep | `body_is_awake`, `body_set_awake`, `body_enable_sleep`, `body_set_sleep_threshold` |
| what it holds | `body_get_shapes`, `body_get_joints`, `body_get_contact_data`, `body_set_user_data`, `body_set_name` |

Forces last one step; apply them every step they act. A force or impulse
with `wake` false on a sleeping body does nothing.

A body that moves more than half its size in a step is swept against static
geometry so it cannot tunnel through a wall. A bullet (`is_bullet`) is
swept against dynamic and kinematic bodies too, at more cost.

## Shapes

A body holds any number of shapes. Each is made from its geometry and a
shape definition:

```aether
shape_def = dynamics.default_shape_def()
shape_def.density = 500.0
shape_def.base_material.friction = 0.4

dynamics.create_sphere_shape(body, &shape_def, manifold.sphere(math.vec3_zero(), 0.5))
dynamics.create_capsule_shape(body, &shape_def, manifold.capsule(math.vec3(0.0, 0.0 - 0.5, 0.0), math.vec3(0.0, 0.5, 0.0), 0.25))

box = hull.make_box_hull(0.5, 0.25, 1.0)       // half extents
dynamics.create_hull_shape(body, &shape_def, box)
hull.destroy_hull(box)                          // the world kept its own copy
```

The six kinds:

| kind | geometry | made with |
|---|---|---|
| sphere | a centre and a radius | `manifold.sphere(center, radius)` |
| capsule | two centres and a radius | `manifold.capsule(center1, center2, radius)` |
| convex hull | up to 128 vertices | `hull.make_box_hull`, `make_cube_hull`, `make_offset_box_hull`, `create_cylinder`, `create_cone`, `create_rock`, or `hull.create_hull(points, count, max_vertex_count)` for any points |
| triangle mesh | vertices and triangles under a tree | `mesh.mesh_def()` filled in, then `mesh.create_mesh(&def, null, 0)`; also `create_grid_mesh`, `create_wave_mesh`, `create_box_mesh` |
| height field | a grid of heights with materials and holes | `height_field.height_field_def()` filled in, then `height_field.create_height_field(&def)` |
| compound | spheres, capsules, hulls and meshes baked into one shape | `compound.compound_def()` filled in, then `compound.create_compound(&def)` |

Meshes and height fields, and a compound's mesh children, collide only with
spheres, capsules and hulls, never with each other, so they belong on
static and kinematic bodies.

Who owns the geometry: a hull shape holds the world's own copy (equal hulls
share one), so the caller may destroy its hull once the shape is made. A
mesh, height field or compound stays the caller's, and must outlive every
shape made from it; destroy it after the world. A hull shape can also be
made moved and scaled: `create_transformed_hull_shape(body, &def, hull,
transform, scale)`. A mesh shape takes a scale, which may mirror.

The shape definition's fields: `density` (kg/m³, default 1000),
`base_material` (or `materials` and `material_count` for a mesh's
per-triangle materials), `filter`, `is_sensor`, the event switches
`enable_contact_events`, `enable_hit_events`, `enable_sensor_events` and
`enable_pre_solve_events` (all off by default), `enable_custom_filtering`,
`update_body_mass`, `name` and `user_data`.

A sensor shape (`is_sensor`) detects overlaps without colliding. It reports
only when it has `enable_sensor_events`, and then only the shapes that have
it too.

After creation, a shape's density, filter, friction, restitution and
material can change (`shape_set_density`, `shape_set_filter`,
`shape_set_friction`, `shape_set_restitution`,
`shape_set_surface_material`), and so can its geometry
(`shape_set_sphere`, `shape_set_capsule`, `shape_set_hull`,
`shape_set_mesh`). `destroy_shape(shape, update_body_mass)` removes one.

## Materials and filtering

A `SurfaceMaterial` has `friction` (default 0.6), `restitution` (0),
`rolling_resistance` (spheres and capsules), `tangent_velocity` (a
conveyor's surface speed in the shape's frame), `user_material_id` (passed
through to events and query results) and `custom_color` for drawing. Two
touching materials mix through the world's friction and restitution
callbacks, by default the geometric mean of the frictions and the larger
restitution.

A shape's `Filter` decides what it collides with: `category_bits` (what it
is), `mask_bits` (what it collides with) and `group_index`. Two shapes
collide when each one's category is in the other's mask. A shared positive
group always collides, a shared negative group never does; a group overrides
the bits.

Two bodies joined by a joint do not collide unless the joint's
`collide_connected` is set. For anything else, a shape with
`enable_custom_filtering` is passed to the world's custom filter callback
(`physics_world.set_custom_filter_callback(world, fn, context)`), which
answers per pair. A pre-solve callback (`set_pre_solve_callback`) sees each
contact of a shape with `enable_pre_solve_events` before it is solved and
can switch it off for that step: one-way platforms.

## Joints

A joint ties two bodies. Each kind has a definition whose `base` holds what
every joint has: the two bodies, a frame on each (`local_frame_a`,
`local_frame_b`, in each body's own coordinates), `collide_connected`, the
force and torque thresholds for joint events, and the constraint's
stiffness.

```aether
hinge_def = dynamics.default_revolute_joint_def()
hinge_def.base.body_id_a = door_frame
hinge_def.base.body_id_b = door
hinge_def.base.local_frame_a.p = math.vec3(0.5, 1.0, 0.0)
hinge_def.base.local_frame_b.p = math.vec3(0.0 - 0.5, 0.0, 0.0)
hinge_def.enable_limit = true
hinge_def.lower_angle = 0.0
hinge_def.upper_angle = 1.5
hinge = dynamics.create_revolute_joint(world, &hinge_def)

joint_solver.revolute_joint_enable_motor(hinge, true)
joint_solver.revolute_joint_set_motor_speed(hinge, 0.5)
```

The kinds, each with `default_<kind>_joint_def` and `create_<kind>_joint`:

| kind | what it does |
|---|---|
| distance | keeps two points apart: rigid, or a spring with a length range, a motor |
| motor | drives B toward a target offset from A: a force and torque budget, springs |
| filter | no constraint; only stops the two bodies colliding |
| parallel | keeps B's axis parallel to A's, with a spring and a torque cap |
| prismatic | slides along an axis: limits, a spring, a motor |
| revolute | a hinge about frame A's z axis: limits, a spring toward a target angle, a motor |
| spherical | a ball joint: a cone limit about frame A's z axis, a twist limit, a spring toward a target rotation, a motor |
| weld | holds B fixed to A, rigid or springy |
| wheel | a suspension along an axis, a spinning motor, steering with its own spring and limits |

The kinds' accessors are in `joint_solver`, named for the kind:
`revolute_joint_get_angle`, `prismatic_joint_set_limits`,
`spherical_joint_set_cone_limit`, `wheel_joint_set_spin_motor_speed`, and
so on. The calls common to every joint are in `dynamics`:
`joint_get_body_a`, `joint_set_local_frame_a`, `joint_set_collide_connected`,
`joint_wake_bodies`, `joint_set_constraint_tuning`, `destroy_joint(joint,
wake_attached)`. `joint_solver.joint_get_constraint_force` and
`joint_get_constraint_torque` give what the joint applied in the last step.

Limits and springs are soft constraints, as in Box3D. A joint driven past
its limit by a motor, or loaded steadily, settles slightly past it, by the
torque over its stiffness. The stiffness is `constraint_hertz` (60 by
default), capped at a quarter of the sub-step rate. Raising it, or the
sub-step count, holds limits tighter.

## Events

Events are collected during a step and read after it, from `physics_world`.
They stay readable until the next step.

| event | count and read | the shapes or bodies that ask |
|---|---|---|
| contact began touching | `contact_begin_event_count`, `contact_begin_event` | either shape has `enable_contact_events` |
| contact stopped touching | `contact_end_event_count`, `contact_end_event` | either shape has `enable_contact_events` |
| hit: a contact that closed faster than the hit threshold | `contact_hit_event_count`, `contact_hit_event` | either shape has `enable_hit_events` |
| a shape entered a sensor | `sensor_begin_event_count`, `sensor_begin_event` | both the sensor and the shape have `enable_sensor_events` |
| a shape left a sensor | `sensor_end_event_count`, `sensor_end_event` | both the sensor and the shape have `enable_sensor_events` |
| a body moved | `body_event_count`, `body_event` | every awake body that moved |
| a joint pulled harder than its threshold | `joint_event_count`, `joint_event` | joints with a force or torque threshold |

```aether
k = 0
while k < physics_world.contact_hit_event_count(world) {
    hit = physics_world.contact_hit_event(world, k)
    // hit.shape_id_a, hit.shape_id_b, hit.point, hit.normal, hit.approach_speed
    k = k + 1
}
```

A body move event carries the body's id, its new transform, its user data
and whether it fell asleep: the one list a renderer needs to update what
moved.

## Queries

The world answers queries between steps, from any thread. Every query takes
a `QueryFilter` (`shape.default_query_filter()` sees everything; its
`category_bits` and `mask_bits` work as a shape's filter does).

| query | call |
|---|---|
| the closest hit along a ray | `cast_ray_closest(world, origin, translation, filter)` returns a `RayResult` (`hit`, `shape_id`, `point`, `normal`, `fraction`, `user_material_id`) |
| every hit along a ray | `cast_ray(world, origin, translation, filter, visitor, context)` |
| shapes whose bounds overlap a box | `overlap_aabb(world, box, filter, visitor, context)` |
| shapes that overlap a convex proxy | `overlap_shape(world, origin, proxy, filter, visitor, context)` |
| a convex proxy swept along a line | `cast_shape(world, origin, proxy, translation, filter, visitor, context)` |
| a character capsule swept, and the planes it rests on | `cast_mover`, `collide_mover` |
| the same against one body at a transform of your choosing | `body_cast_ray`, `body_cast_shape`, `body_overlap_shape`, `body_collide_mover` |

A visitor is a plain function. An overlap visitor
`fn(ShapeId, ptr) -> bool` returns false to stop. A cast visitor
`fn(ShapeId, point, normal, fraction, user_material_id, triangle_index,
child_index, ptr) -> float` returns where to clip the cast: the hit's
fraction to find the closest, 1 to collect every hit, 0 to stop, -1 to
ignore this shape. The `context` pointer is passed to every call:

```aether
count_shape(id: ShapeId, context: ptr) -> bool {
    count = context as int[]
    count[0] = count[0] + 1
    return true
}

found = 0
physics_world.overlap_aabb(world, box, shape.default_query_filter(), count_shape, (&found) as ptr)
```

A proxy is a convex point cloud with a radius:
`distance.shape_proxy(points, count, radius)`. One point and a radius is a
sphere; two points a capsule.

The character mover is Box3D's: `cast_mover` sweeps a capsule and returns
the fraction it can travel, `collide_mover` gathers the planes it touches,
and `mover.solve_planes` and `mover.clip_vector` turn those into a
movement that slides along walls and stands on slopes.

## Threads

`world_def.worker_count` sets how many threads a step uses; 1, the default,
is the calling thread alone and never makes a thread. With more, the world
makes its worker threads when it is created and parks them between steps.
`native.processor_count()` gives the machine's count, and
`physics_world.set_worker_count` changes it between steps.

The narrow phase, the solver's stages, the bodies' finalize and the bullets
are split over the workers in Box3D's way. The result does not depend on
the count: a world steps to the same bits on one worker or twenty-four.

A host with its own job system lends it instead: create an
`aephysics.parallel` scheduler, keep it, and set `world_def.scheduler` to
it. The scheduler's worker count then rules.

Queries may run on any number of threads at once, but not while that world
steps. Each thread takes one of 64 scratch slots the first time it queries,
and gives it back when it exits.

## Determinism

The same world stepped the same way gives the same bits:

- every run, on every platform CI covers (Linux and Windows x86-64, macOS
  arm64), optimized or not;
- at any worker count, and at either SIMD width;
- after a snapshot is saved and loaded.

`dynamics.hash_world_state(physics_world.world_of(world))` is a hash of
every body's state, for checking that two runs agree.

That holds only if the C compiler never fuses a multiply and an add into
one rounding, which GCC and Clang do by default wherever the target has
FMA. A program that builds aephysics in must pass `-ffp-contract=off` in its
own `aether.toml`, since `ae` reads only the root project's C flags. The
engine's trigonometry is its own (Box3D's approximations), and its square
root is std.math's, exact at every optimization level.

The bits are the same across platforms but not across engine versions: a
change that alters results says so in its commit and regenerates the
golden trace.

## Ragdolls

`aephysics.human` is Box3D's ragdoll, grown for games: twelve capsule bones
(pelvis, two spine bones, the neck and head, thighs, calves, upper and lower
arms) on spherical and revolute joints.

```aether
figure = Human { bones: null, filter_joints: null, filter_joint_count: 0, friction_torque: 0.0, is_spawned: false, ankles: false, bone_count: 0 }
human.create_human(&figure, world, feet_position, 5.0, 1.0, 0.7, 1, null)
// joint friction 5 N m, springs toward the standing pose at 1 Hz and a
// damping ratio of 0.7, and filter group 1 (see below)

pelvis = human.bone_at(&figure, human.BONE_PELVIS).body_id
human.destroy_human(&figure)
```

- **Self-collision.** The lower spine and the thighs share the negative
  filter group of the index given, so they never collide with each other,
  and filter joints keep the other limbs that would clash apart. Give each
  figure its own index, or two figures' hips pass through each other.
- **Joint ranges.** By default the joints have a person's ranges: shoulders
  that reach overhead, hips on an elliptical cone, knees and elbows that
  never pass straight. `human_tight_limits(shape)` puts Box3D's tighter
  ones on a figure's shape before `create_human_shaped`.
- **Feet.** With `figure.ankles = true` before creation, the feet are bones
  of their own on elliptical ankle cones, so a crouched figure's soles lie
  flat.
- **Pose drive.** Every joint has a spring toward a target
  (`human_set_joint_spring_hertz`, `human_set_joint_damping_ratio`) and a
  motor whose torque limit is the joint's friction
  (`human_set_joint_friction_torque`). Setting the targets from an
  animation each step drives the ragdoll.
- **Anchors.** `human_create_motor_anchors` hangs every bone from a
  kinematic anchor through a motor joint; move the anchors with
  `human_set_anchor_target` and the figure follows, with the world pushing
  back. `human_create_parallel_anchors` drives rotation only, so the figure
  falls while it keeps its shape. `human_align_spring` springs the pelvis
  upright.
- **A figure fitted to a rig.** `human_reference_shape` gives the figure as
  data, `human_fit_shape` moves its joints and bones onto a rig's
  skeleton, and `create_human_shaped` builds the fitted figure.

## Snapshots, recording and replay

A snapshot is a whole world as bytes: settings, bodies, shapes and their
geometry, contacts with their warm starts, joints, islands, the trees.
A loaded world steps on as the one it was taken from, bit for bit.

```aether
data, size = snapshot.save_world(world)
copy = snapshot.load_world(&world_def, data, size)
ok = snapshot.restore_world(world, &world_def, data, size)   // into an existing world, its id kept
snapshot.free_snapshot(data, size)
```

A snapshot of another layout, or one damaged in any byte, loads no world.

A recording keeps every call that changes a world, field by field, with a
state hash after each step and every query with its answers.

```aether
rec = recording.create_recording()
recording.start_recording(world, rec)
// ... create, step, query ...
recording.stop_recording(world)
recording.save_recording_to_file(rec, "session.rec")

player = recording.create_player(recording.recording_data(rec), recording.recording_size(rec), 1)
frames = 0
while recording.player_step_frame(player) { frames = frames + 1 }
diverged = recording.player_has_diverged(player)
recording.destroy_player(player)
recording.destroy_recording(rec)
```

The player replays into a world of its own (`player_world`), checks every
step's hash and every created id against the recording, and asks every
recorded query again. It seeks (`player_seek_frame`) through keyframes kept
under a memory budget. A recording made on several workers replays on one,
and the other way round.

## Debug drawing

`debug_draw.default_debug_draw()` gives a `DebugDraw` whose callbacks a
renderer fills in (segments, transforms, points, spheres, capsules, boxes,
strings, its own drawable per shape) and whose switches choose what to
draw: shapes, joints, contacts with their normals and forces, bounds,
islands, names and masses. `physics_world.world_draw(world, &draw,
mask_bits)` walks the world. The world definition's `create_debug_shape`
and `destroy_debug_shape` give a renderer its own mesh per shape, made once
and released when the shape, its geometry or the world goes.

## Fitting into a host engine

- **Memory.** `basics.set_allocator(allocate, release)` routes every block
  the engine makes through the host. `physics_world.get_counters` and
  `dump_memory_stats` say where the memory went.
- **Logging and failure.** `basics.set_log_fn` takes the engine's messages
  and `basics.set_assert_fn` its one hard stop. Bad input (a NaN, a negative
  radius, a hull of points that make no hull) is refused, never asserted:
  the call does nothing and the world stays as it was.
- **Shared types.** Aether keeps struct names in one namespace across
  modules. aephysics' math types (`Vec2`, `Vec3` as `{x, y, z}`, `Quat` as
  `{x, y, z, w}`, `Transform`) are the same definitions ae3d's core uses,
  so a transform crosses between the two without conversion. A host must
  not define its own `World`, `Body`, `Plane` or `Buffer`; ae3d names its
  own `VoxelWorld`, `FrustumPlane`, `GltfBuffer`.
- **User data.** Worlds, bodies, shapes and joints each carry a `user_data`
  pointer and a name.
- **The C file.** `aephysics/native/aephysics_native.c` (threads, atomics,
  the semaphore) is named by `aephysics.native` with `@source`, so `ae`
  builds it into any program that imports the engine. A host that calls gcc
  itself compiles it too, with the same `-ffp-contract=off`.
- **SIMD width.** A world runs its wide contact stages and its separating
  axis test eight lanes at a time, on AVX2 where the CPU has it.
  `basics.set_simd_width(4)` before a world is made forces the four-lane
  path, and `physics_world.enable_sse2_fallback` switches an existing
  world. Both give the same bits.

## Where aephysics goes past Box3D

The engine is Box3D's, with these additions. Each is off by default where
it changes results, so Box3D's own scenes give Box3D's checksums.

- **Double positions.** Box3D keeps double world positions only in its
  large-world build; here they are always double, so a transform is one
  `Vec3` and one `Quat`. The solver's own data stays single precision, as
  Box3D's.
- **Kinematic speculation** (`enable_kinematic_speculation`): a fast
  kinematic body's contacts reach out over its travel in the step, so what
  it strikes is brought up to speed in the step the gap closes instead of
  being found inside it on the next.
- **Travel speculation** (`enable_travel_speculation`): the same for every
  moving body, for fast dynamic bodies below Box3D's continuous threshold,
  such as a car.
- **Kinematic-static softness** (`enable_kinematic_static_softness`): a
  kinematic body's contacts take the stiffer static softness, so a body the
  game drives hard does not sink into what it pushes.
- **Hulls that settle on an edge.** A hull resting on an edge or vertex
  rocked forever in Box3D, in a 400-step cycle. Its face contact here also
  takes the vertices already within the speculative distance, so it comes
  to rest. A box resting flat has no such vertex, so stacks are unchanged.
- **Elliptical and off-axis cones** on spherical joints
  (`spherical_joint_set_cone_ellipse`, `spherical_joint_set_cone_axis`),
  for hips and shoulders. A circle about z stays the default, bit for bit.
- **Mesh queries without a cap.** Box3D's mesh contact fills a buffer of 256
  triangles and warns; here the buffer grows.
