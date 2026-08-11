# Runtime contract

The API is small at the binary boundary but intentionally strict at runtime. Correct consumers respect ownership, callback thread, generations, leases, bounded copies, and fail-closed lifecycle state.

## Thread and callback model

Owner frame callbacks run at a ROCK-owned game-thread frame boundary. Calls documented as game-thread-only return `WrongThread` elsewhere. Do not block, perform file I/O, allocate without a bounded reason, or throw across the callback boundary.

Callback removal prevents future dispatch copies; it is not a quiescence barrier for an invocation already copied for dispatch. Keep `userData` alive until the current invocation has returned.

A callback fault revokes the owning consumer's callbacks and stateful resources. Catch failures inside your callback and fail closed.

## Lifecycle gates

Use snapshot flags, not inference:

- `WorldAvailable`, `SkeletonReady`, and `ProviderReady` describe prerequisites.
- `MenuBlocking`, `ConfigBlocking`, and `LoadingOrWorldTransition` describe blocked/unstable state.
- `GeneratedBodiesValid` describes generated collision availability.
- `PhysicsWriteAllowed` is required for physics/control publication.
- `VisualWriteAllowed` is required for visual/animation/debug publication.

When a required flag disappears, stop publishing or explicitly clear the affected scope/authority. Do not keep driving stale state.

## Generation identity

`worldGeneration`, `skeletonGeneration`, `providerGeneration`, `collisionGeneration`, and `weaponGenerationKey` prevent cross-lifetime reuse. Copy the current values into requests that expose those guards. Treat a mismatch as normal invalidation and reacquire current state.

Form IDs, body IDs, target IDs, and generation keys solve different identity problems. A body ID without its collision/weapon generation is not durable identity.

## Pointer rule

Legacy pointer-sized fields are non-owning witnesses. They may be compared only on the provider callback/query frame while all accompanying generation identities match. They must not be retained or dereferenced by consumers. Newer calls use value identity; command `targetRefr` inputs are ignored in favor of form/body IDs.

## Leases and publications

Stateful publications are bounded rolling leases. Refresh them only while the feature is active and its lifecycle gate is present. They expire or invalidate on lease expiry, generation change, explicit clear, unregister, callback fault, or provider loss.

This pattern applies to input suppression, offhand reservation, weapon-part drives, animation/visual authority, handling authority, debug overlay, native animation runtime, touch-grab targets, and collider focus.

Use the limits returned by `getProviderLimitsExtV1`; do not assume the header's current constants will remain the provider's effective policy forever.

## Bounded copy/cursor calls

Copy functions never transfer ownership. Provide fixed or otherwise bounded storage and honor the returned count. Cursor streams expose retained sequence bounds and overwritten/gap state; advance your cursor to `lastCopiedSequence` only after processing the copied batch.

## Result handling

`RockProviderResultV1` distinguishes expected state such as `NotReady`, `UnsupportedVersion`, `OwnerNotRegistered`, `PermissionDenied`, `WorldNotReady`, `TargetUnavailable`, `HandBusy`, `WrongThread`, and capacity/validation failures. Branch on the enum; do not reduce every non-`Ok` result to a retry.

Queued interaction commands have a second lifecycle. Admission success only returns a command ID. Poll `getInteractionCommandResultV1` until the command reaches a terminal result, or cancel it when the owning feature shuts down.
