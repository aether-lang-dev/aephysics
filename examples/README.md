# Examples

Small, complete programs. `scripts/test.sh` builds and runs every one, so
they keep working as the engine changes.

| program | shows |
|---|---|
| [hello_world.ae](hello_world.ae) | Box3D's HelloWorld: a world, a static ground, a dynamic box, stepping, reading a position |
| [events_and_queries.ae](events_and_queries.ae) | a chain of capsules on spherical joints, contact and hit events from a bouncing ball, a ray cast and a box overlap with a visitor |
| [ragdoll_and_snapshot.ae](ragdoll_and_snapshot.ae) | the ragdoll shoved over, its world saved as a snapshot mid-fall and loaded into a second world, both stepped on to the same bits |

From the repository root:

```sh
ae build examples/hello_world.ae -o target/hello_world && target/hello_world
```

[The guide](../docs/guide.md) explains every call they make.
