# V1 function table index

`RockProviderApi` is append-only. The x64 V1 table currently contains 90 function pointers (720 bytes). Slot order is ABI and is mechanically checked against this document.

The signature, enum values, structure defaults, flags, and inline support helpers in `ROCKProviderApi.h` are normative. This index explains intent and the principal gate; it does not replace the header.

| Slot | Function | Purpose |
| ---: | --- | --- |
| 0 | `getVersion` | Return the provider API version. |
| 1 | `getModVersion` | Return the loaded ROCK mod version. |
| 2 | `isProviderReady` | Report whether the provider currently has usable runtime state. |
| 3 | `registerFrameCallback` | Register a legacy unowned frame callback. Prefer slot 56. |
| 4 | `unregisterFrameCallback` | Remove a legacy frame callback. |
| 5 | `getFrameSnapshot` | Copy the latest coherent frame/lifecycle snapshot. |
| 6 | `queryWeaponContactAtPoint` | Query current weapon evidence near a game-space point. |
| 7 | `clearExternalBodies` | Clear all external bodies owned by a token. |
| 8 | `setOffhandInteractionReservation` | Set the legacy reservation state. Prefer leased slots 77–80. |
| 9 | `registerExternalBodiesV1` | Replace the legacy per-owner external body set. |
| 10 | `getWeaponEvidenceDetailCountV1` | Return the current detailed weapon-evidence count. |
| 11 | `copyWeaponEvidenceDetailsV1` | Copy bounded weapon-evidence records. |
| 12 | `getWeaponEvidenceDetailPointCountV1` | Return a weapon body's evidence point count. |
| 13 | `copyWeaponEvidenceDetailPointsV1` | Copy a weapon body's local evidence point cloud. |
| 14 | `getBodyContactSnapshotV1` | Copy recent player-body contact records. |
| 15 | `getPrimaryHandV1` | Return the current primary hand. |
| 16 | `getOffhandHandV1` | Return the current offhand. |
| 17 | `getHandFrameV1` | Copy a root-flattened hand physics frame. |
| 18 | `registerConsumerV1` | Register a named consumer and negotiate capabilities. |
| 19 | `unregisterConsumerV1` | Revoke an owner and all of its resources. |
| 20 | `getGrantedCapabilitiesV1` | Return the current granted mask for an owner. |
| 21 | `getProviderLimitsV1` | Copy the stable limits/feature prefix. |
| 22 | `getExternalContactSnapshotForOwnerV1` | Copy legacy owner-filtered external contacts. |
| 23 | `requestForceGrabV1` | Queue a near force-grab command. |
| 24 | `getInteractionCommandResultV1` | Poll an admitted command through its terminal result. |
| 25 | `requestForceReleaseV1` | Queue a normal-path physical release. |
| 26 | `requestThrownDropV1` | Queue a release using captured or supplied throw velocity. |
| 27 | `setHandInputSuppressionV1` | Publish/refresh a per-hand input suppression lease. |
| 28 | `clearHandInputSuppressionV1` | Clear the caller's suppression for one hand. |
| 29 | `setWeaponPartTargetsV1` | Replace semantic weapon-part targets for an owner. |
| 30 | `clearWeaponPartTargetsV1` | Clear the caller's part targets. |
| 31 | `setWeaponPartDriveTargetsV1` | Replace bounded weapon-part drive publications. |
| 32 | `clearWeaponPartDriveTargetsV1` | Clear the caller's weapon-part drives. |
| 33 | `queryEquippedWeaponClassificationV1` | Copy equipped-weapon classification. |
| 34 | `getWeaponPartGripStateV1` | Read active/resolved weapon-part grip state. |
| 35 | `getRawWandButtonStateV1` | Read raw controller button/touch sample metadata. |
| 36 | `isNativePipboyInputSuppressedV1` | Report effective native Pip-Boy suppression. |
| 37 | `getWeaponEmitterCountV1` | Return the current emitter count. |
| 38 | `copyWeaponEmittersV1` | Copy bounded muzzle/beam emitter snapshots. |
| 39 | `setNativeAnimationAuthorityV1` | Acquire or refresh native animation authority. |
| 40 | `clearNativeAnimationAuthorityV1` | Release native animation authority. |
| 41 | `getNativeAnimationAuthorityStateV1` | Read authority ownership/lease state. |
| 42 | `registerAnimationPhaseCallbackV1` | Subscribe an owner to animation phase callbacks. |
| 43 | `unregisterAnimationPhaseCallbackV1` | Remove an animation phase callback. |
| 44 | `getEquippedWeaponGripStateV1` | Read canonical grip, muzzle, and weapon-frame state. |
| 45 | `setHandVisualAuthorityV1` | Publish/refresh hand and finger visual authority. |
| 46 | `clearHandVisualAuthorityV1` | Clear visual authority for one hand. |
| 47 | `publishNativeAnimationRuntimeV1` | Publish/refresh native animation runtime state. |
| 48 | `setEquippedWeaponHandlingAuthorityV1` | Acquire/refresh equipped-weapon handling policy. |
| 49 | `clearEquippedWeaponHandlingAuthorityV1` | Release handling authority. |
| 50 | `getEquippedWeaponHandlingStateV1` | Read current handling ownership, policy, and lease. |
| 51 | `publishDebugOverlayV1` | Publish bounded owner-scoped lines/text to ROCK's stereo overlay. |
| 52 | `clearDebugOverlayV1` | Clear the caller's overlay publication. |
| 53 | `getPresentedHandFrameV1` | Copy the currently presented hand frame after visual writers. |
| 54 | `getProviderLimitsExtV1` | Prefix-copy the complete extensible limits and feature words. |
| 55 | `getPublicStructureSizeV1` | Return ROCK's byte size for a public structure ID. |
| 56 | `registerFrameCallbackForOwnerV1` | Register a game-thread frame callback bound to an owner. |
| 57 | `unregisterFrameCallbackForOwnerV1` | Remove an owner-bound callback. |
| 58 | `getHandInteractionStateV1` | Read coherent owner-filtered per-hand interaction state. |
| 59 | `copyProviderEventsSinceV1` | Copy owner-filtered provider events after a sequence cursor. |
| 60 | `getEquippedWeaponStateV1` | Read weapon transition identity and terminal state. |
| 61 | `registerExternalBodiesForScopeV1` | Transactionally replace bodies in one owner scope. |
| 62 | `clearExternalBodiesForScopeV1` | Clear one external body scope. |
| 63 | `copyExternalContactsSinceV1` | Copy scoped enriched contacts after a sequence cursor. |
| 64 | `queryWeaponPartTargetResolutionV1` | Resolve a semantic/matcher target to current weapon identity. |
| 65 | `copyWeaponPartPoseSnapshotV1` | Copy current generated weapon-part poses. |
| 66 | `copyWeaponPartDriveApplicationResultsV1` | Copy what the provider actually applied for part drives. |
| 67 | `getScopeSightStateV1` | Read current sight bounds, anchor, body, and activation source. |
| 68 | `getWeaponCompositionStateV1` | Read composition signature, counts, and coverage. |
| 69 | `copyWeaponCompositionEntriesV1` | Copy installed composition entries. |
| 70 | `getSelectedAuthoredGripPoseV1` | Read the selected authored two-hand/finger pose. |
| 71 | `getPresentedHandPoseV1` | Read current presented hand and finger local transforms. |
| 72 | `copySemanticHandContactsV1` | Copy semantic finger/hand contacts for one hand. |
| 73 | `copyPlayerColliderDescriptorsV1` | Copy value descriptors for generated hand/body colliders. |
| 74 | `getHandCollisionAvailabilityV1` | Read per-hand collision readiness and counts. |
| 75 | `cancelInteractionCommandV1` | Cancel a caller-owned queued command when still cancellable. |
| 76 | `getHandInputSuppressionStateV1` | Read caller and effective suppression/lease state. |
| 77 | `acquireOffhandReservationV1` | Acquire a generation-bound offhand reservation lease. |
| 78 | `renewOffhandReservationV1` | Refresh the caller's reservation lease. |
| 79 | `releaseOffhandReservationV1` | Release the caller's leased reservation. |
| 80 | `getOffhandReservationStateV1` | Read ownership, effective mode, expiry, and invalidation. |
| 81 | `clearNativeAnimationRuntimeV1` | Clear the caller's native animation runtime publication. |
| 82 | `setTouchGrabTargetsForScopeV1` | Transactionally replace fixed/hinge/prismatic touch targets in a scope. |
| 83 | `clearTouchGrabTargetsForScopeV1` | Clear one touch-grab scope. |
| 84 | `copyTouchGrabStatesForScopeV1` | Copy owner-scoped target phase/contact snapshots. |
| 85 | `requestTouchGrabYieldV1` | Asynchronously yield a target before native/scripted motion. |
| 86 | `requestEquippedWeaponHandV1` | Request exact-hand ownership for the current weapon under handling authority. |
| 87 | `queryWorldRaycastV1` | Perform a bounded provider-filtered game-thread world raycast. |
| 88 | `setColliderVisualizationOverrideV1` | Focus overlay visualization on one current weapon body. |
| 89 | `clearColliderVisualizationOverrideV1` | Clear the caller's collider visualization focus. |

## Exports

`ROCK.dll` exposes three C exports:

- `ROCKAPI_GetDescriptorV1`: preferred safe discovery path.
- `ROCKAPI_GetProviderApi`: legacy provider-table accessor.
- `ROCKAPI_GetApi`: alias returning the same table.

Consumers normally call `RockProviderApi::initialize` instead of resolving these manually.
