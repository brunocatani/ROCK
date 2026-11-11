# Touch (interface 5, major 2)

Header: `include/ROCK/Touch.h`. Lets a mod make bodies and surfaces holdable
by ROCK's hands (doors, levers, sliders, climbable surfaces) and reserve a
hand for its own authored interaction.

## Discover and bind

Interface `Touch`, major 2, minor 0; major 1 is refused with
`UnsupportedMajor`. Permissions: `Read` (1) for states and samples; `Write`
(2) for targets, yields and authored hands.

## Touch-grab targets

| Function | Thread | Result |
| --- | --- | --- |
| `setTouchGrabTargetsForScope(owner, scope, targets, count)` | game | Replaces every target of `scope` (nonzero, owner-chosen). Up to `kMaxTargets` per call and `kMaxScopes` scopes. `CapacityFull`, `OwnerConflict` (scope owned by another mod), `GenerationMismatch`. |
| `clearTouchGrabTargetsForScope(owner, scope)` | game | Removes the scope's targets; held targets are released. |
| `copyTouchGrabStatesForScope(owner, scope, out, max, count*)` | game | Copies up to `max` target states and writes how many were copied. |
| `requestTouchGrabYield(owner, scope, targetId, generation)` | game | Asks ROCK to let go of a held target (release reason `OwnerYield`). |

`TouchGrabTarget`:
- `targetId`/`targetGeneration`: your identity for the target.
- `kind`: `FixedAnchor` (hand latches to a point or surface), `LimitedHinge`
  or `LimitedPrismatic` (hand drives a one-axis mechanism around
  `pivotWorldGame`/`axisWorldGame` between `minimumCoordinate` and
  `maximumCoordinate`, starting at `currentCoordinate`).
- `bodyId` names one body; with `MatchAnyBody`, a `FixedAnchor` target instead
  matches any body whose layer is in `allowedLayerMask` and whose motion type
  is enabled by the `Match...Motion` flags.
- A `FixedAnchor` latch follows its body's live transform and never moves the
  body. Besides world surfaces and fixed props, the dynamic hand latches onto
  bodies on the actor layers NativeInteraction extensions admit (8, 32, 33 and
  `kPassiveObstacleLayer` 55: ragdolls, corpses, native limbs and extension
  bodies), but only for a target that names the body; ROCK's built-in surface
  grab never takes those layers.
- `flags` (`TouchGrabTargetFlag`): allowed hands, two-hand use,
  `LatchOnRelease`, `FallbackOnly` (wildcard surfaces considered last),
  `ExcludePowerArmor`, `AcquireWhileGripHeld` (wildcard `FixedAnchor` only: a
  grip that is already held latches the moment that hand touches a matching
  surface; every other target is taken only on the grip press).
- `leaseFrames` (1..`kMaxLeaseFrames`): the target expires unless set again.
- World, skeleton and provider generations from the current publication.

`TouchGrabState` reports `phase` (`TouchGrabPhase`: `Inactive`, `Armed`,
`Held`, `Latched`, `Yielded`, `Invalidated`), `releaseReason`,
`activeHandMask` (`TouchGrabHandMask`: `Right`, `Left`), `flags`, the current
coordinate and its velocity, the contact point and normal, `surfaceGripMode`,
`frameIndex`, the four generations, and a `sequence` that increases on every
change of that target's state.

| Enum | Values |
| --- | --- |
| `TouchGrabReleaseReason` | `None`, `GripReleased`, `OwnerYield`, `TargetRemoved`, `RegistrationExpired`, `GenerationChanged`, `WorldLost`, `TargetInvalid`, `HandUnavailable` |
| `TouchGrabStateFlag` | `ContactPointValid`, `ContactNormalValid`, `CoordinateValid`, `FixedAnchor`, `OriginalMotionKeyframed`, `OriginalMotionDynamic`, `MeshSurfaceAnchor`, `MeshFingerPose`, `MeshCollisionFallback` |
| `SurfaceGripMode` | `CollisionAnchor`, `MeshAnchor`, `CollisionFallback`, `AnimatedArmorBone` |

## Authored hand

| Function | Thread | Result |
| --- | --- | --- |
| `setAuthoredHand(owner, const AuthoredHand*)` | game, including BeforeRock | Takes or refreshes the lease on one hand. `OwnerConflict` if another mod holds it; `GenerationMismatch` for a stale sample. |
| `clearAuthoredHand(owner, hand)` | game | Releases the lease. |

Refresh the lease every frame (`leaseFrames` 1..3) with `sample` copied from
the current publication. Flags (`AuthoredFlag`):
- `Indicator`: ROCK shows its authored marker at `contactWorldGame`.
- `ReserveGrab`: new ROCK grabs on that hand are consumed before
  static/climbing acquisition. Existing holds are not revoked.
- `Attached` (requires `ReserveGrab`): only that hand's dynamic collider
  stops colliding with world surfaces.

Your mod owns reach and occupancy checks, the grip pose and the mechanism's
movement.

## Sample

`getSample` returns the current publication identity.

## Example

```cpp
rock::api::touch::TouchGrabTarget lever{};
lever.targetId = 1;
lever.kind = rock::api::touch::TouchGrabKind::LimitedHinge;
lever.bodyId = leverBody;
lever.flags = static_cast<std::uint32_t>(rock::api::touch::TouchGrabTargetFlag::AllowRightHand);
lever.leaseFrames = 2;
// pivot, axis, coordinate range and generations from the current sample
touch->setTouchGrabTargetsForScope(client.owner(), myScope, &lever, 1);
```
