# Discovery, capabilities, features, and limits

There are three independent compatibility layers:

1. API version: currently `ROCK_PROVIDER_API_VERSION == 1`.
2. Function-table byte extent: proves an appended slot exists.
3. Feature/capability negotiation: proves the loaded provider implements the behavior and grants this owner permission to use it.

Passing only one layer is insufficient for optional appended surfaces.

## Safe descriptor

`RockProviderApi::initialize` prefers `ROCKAPI_GetDescriptorV1`. The immutable descriptor reports its own size, API version, provider mod version, exact table byte size, both feature words, and the table pointer. The legacy export is used only when the caller did not request a minimum table extent.

After initialization, the header exposes `negotiatedApiVersion`, `negotiatedTableByteSize`, `negotiatedFeatureBits`, and `negotiatedFeatureBits2` for diagnostics. Prefer the named `supports...V1` helpers for decisions.

## Consumer capabilities

Capabilities are owner permissions requested at registration and returned in `grantedCapabilities`.

| Capability | Enables |
| --- | --- |
| `FrameSnapshots` | Owner frame callbacks and coherent frame/lifecycle snapshots. |
| `ExternalBodies` | Register consumer-owned physics bodies. |
| `ExternalContacts` | Read contacts against registered external bodies. |
| `OffhandReservation` | Reserve offhand interaction with rolling leases. |
| `InteractionCommands` | Queue grab, release, thrown-drop, and cancellation commands. |
| `HandInputSuppression` | Publish per-hand input suppression leases. |
| `WeaponPartInteraction` | Publish semantic part targets and drive targets. |
| `NativeAnimationAuthority` | Acquire bounded native animation authority. |
| `AnimationPhases` | Subscribe to animation phase callbacks. |
| `EquippedWeaponGripState` | Read resolved weapon/grip/muzzle state. |
| `HandVisualAuthority` | Publish presented hand/finger transforms. |
| `NativeAnimationRuntimeProvider` | Publish or clear native animation runtime state. |
| `EquippedWeaponHandlingAuthority` | Own equipped-weapon handling policy and exact-hand requests. |
| `DebugOverlayPublication` | Publish bounded stereo overlay lines/text. |
| `ProviderEvents` | Consume the owner-filtered provider event cursor. |
| `HandInteractionState` | Read coherent per-hand interaction state. |
| `ExternalBodyScopes` | Replace/clear external bodies independently by scope. |
| `WeaponPartObservability` | Resolve targets and copy part poses/drive results. |
| `WeaponComposition` | Read installed composition and semantic coverage. |
| `PoseReadback` | Read authored grip and presented hand poses. |
| `SemanticHandContacts` | Read semantic finger/hand contacts. |
| `PlayerColliderDescriptors` | Discover generated player colliders by value. |
| `ScopeSightState` | Read current sight/scope activation and anchor state. |
| `InputObservability` | Read raw controller state and effective suppression state. |
| `TouchGrabTargets` | Publish fixed/hinge/prismatic touch-grab targets and read their states. |
| `WorldRaycasts` | Issue bounded provider-filtered world raycasts in owner callbacks. |
| `ColliderVisualizationOverride` | Focus debug visualization on one current weapon body. |

Request only what the mod actually uses. Registration can succeed with a subset; the granted mask is authoritative.

## Feature word 1

The first feature word currently defines:

`FrameCallbacks`, `LifecycleFields`, `HandFrames`, `WeaponEvidence`, `BodyContacts`, `ExternalContacts`, `ConsumerRegistrationV1`, `OwnerFilteredExternalContactsV1`, `InteractionCommandQueue`, `ForceGrabCommand`, `ForceReleaseCommand`, `ThrownDropCommand`, `HandInputSuppression`, `WeaponPartInteraction`, `WeaponPartGripState`, `WeaponPartRecordIdentity`, `WeaponPartTargetNonExclusive`, `RawWandButtonState`, `PipboyInputSuppression`, `WeaponEmitters`, `NativeAnimationAuthority`, `AnimationPhases`, `EquippedWeaponGripState`, `HandVisualAuthority`, `NativeAnimationRuntimeProvider`, `EquippedWeaponHandlingAuthority`, `DebugOverlayPublication`, `PresentedHandFrames`, `EquippedWeaponHandRequest`, and `ColliderVisualizationOverride`.

## Feature word 2

The second feature word currently defines:

`SafeDescriptor`, `ExtendedLimits`, `PublicStructureSizes`, `OwnerFrameCallbacks`, `HandInteractionState`, `ProviderEvents`, `EquippedWeaponState`, `ExternalBodyScopes`, `ExternalContactCursor`, `WeaponPartResolution`, `WeaponPartPoses`, `WeaponPartDriveResults`, `ScopeSightState`, `WeaponComposition`, `AuthoredGripSnapshot`, `PresentedHandPose`, `SemanticHandContacts`, `PlayerColliderDescriptors`, `HandCollisionAvailability`, `CommandCancellation`, `InputSuppressionState`, `OffhandReservationLeases`, `SnapshotEnrichment`, `NativeAnimationRuntimeLeases`, `StatefulPublicationLeases`, `CommandLifecycle`, `InputSampleMetadata`, `WeaponClassificationEnrichment`, `ExternalContactEnrichment`, `TouchGrabTargets`, `NativeVatsVansInputSuppression`, and `WorldRaycasts`.

## Table guards

Every appended family has a named `ROCK_PROVIDER_API_V1_*_TABLE_BYTES` constant and usually a `supports...V1` helper. Initialization with the family constant is the simplest hard requirement. A plugin supporting older providers can initialize without a minimum, fetch limits, and conditionally enable each family through the helper.

## Limits

Call `getProviderLimitsV1` for the stable prefix and `getProviderLimitsExtV1` for the complete current set. The extended structure covers consumers, callbacks, bodies/scopes/contacts, commands/results, input leases, weapon targets/drives/poses, animation authorities, overlay budgets, event retention, composition, semantic contacts, player colliders, touch targets/scopes, evidence catalogs, and per-owner raycast budgets.

Call `getPublicStructureSizeV1` when interoperating across header revisions and prefix-copy only the provider-supported bytes of extensible value structures.
