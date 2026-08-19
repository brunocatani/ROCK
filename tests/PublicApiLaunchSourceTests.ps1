param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$Root = [System.IO.Path]::GetFullPath($Root)

$failures = [System.Collections.Generic.List[string]]::new()
$rpsSdkRoot = if ($env:RPS_SDK_ROOT) {
    [System.IO.Path]::GetFullPath($env:RPS_SDK_ROOT)
} else {
    [System.IO.Path]::GetFullPath((Join-Path (Split-Path $Root -Parent) 'RPS_SDK'))
}
$rockSdkRoot = Join-Path $rpsSdkRoot 'SDK/ROCK'

function Require-Path {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: missing file for text check")
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Require-SdkPath {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $rockSdkRoot $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("RPS_SDK/SDK/ROCK/$RelativePath`: $Message")
    }
}

function Require-SdkFileEqual {
    param(
        [string]$SourceRelativePath,
        [string]$SdkRelativePath,
        [string]$Message
    )

    $sourcePath = Join-Path $Root $SourceRelativePath
    $sdkPath = Join-Path $rockSdkRoot $SdkRelativePath
    if (-not (Test-Path -LiteralPath $sourcePath) -or
        -not (Test-Path -LiteralPath $sdkPath)) {
        $failures.Add("$SourceRelativePath / RPS_SDK/SDK/ROCK/$SdkRelativePath`: missing file for sync check")
        return
    }

    $source = Get-Content -Raw -LiteralPath $sourcePath
    $sdk = Get-Content -Raw -LiteralPath $sdkPath
    if ($source -cne $sdk) {
        $failures.Add("$SourceRelativePath / RPS_SDK/SDK/ROCK/$SdkRelativePath`: $Message")
    }
}

function Get-ProviderFunctionNames {
    param([string]$Text)

    $names = [System.Collections.Generic.List[string]]::new()
    foreach ($entry in [regex]::Matches($Text, 'ROCK_PROVIDER_CALL\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)')) {
        $names.Add($entry.Groups[1].Value)
    }
    return [string[]]$names
}

function Require-SequenceEqual {
    param(
        [string]$Name,
        [string[]]$Actual,
        [string[]]$Expected
    )

    if ($Actual.Count -ne $Expected.Count) {
        $failures.Add("$Name`: expected $($Expected.Count) entries, found $($Actual.Count)")
        return
    }

    for ($i = 0; $i -lt $Expected.Count; ++$i) {
        if ($Actual[$i] -ne $Expected[$i]) {
            $failures.Add("$Name`: mismatch at index $i; expected '$($Expected[$i])', found '$($Actual[$i])'")
            return
        }
    }
}

Require-SdkPath 'include/ROCKProviderApi.h' 'Independent public SDK must ship the provider header.'
Require-SdkPath 'include/ROCKApi.h' 'Independent public SDK must ship the API alias header.'
Require-SdkPath 'docs/PublicApi.md' 'Independent public SDK must ship its API documentation.'
Require-SdkPath 'examples/CMakeLists.txt' 'Independent public SDK must ship buildable consumers.'
Require-SdkPath 'examples/cmake/VerifyFo4VrLoader.cmake' 'Independent examples must retain the build-enforced FO4VR loader gate.'

Require-SdkFileEqual 'src/api/ROCKProviderApi.h' 'include/ROCKProviderApi.h' `
    'SDK provider header must stay byte-for-byte synced with the source ABI header.'
Require-SdkFileEqual 'src/api/ROCKApi.h' 'include/ROCKApi.h' `
    'SDK API alias header must stay byte-for-byte synced with the source ABI header.'

Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_VERSION\s*=\s*1' `
    'Provider API version must be v1.'
Require-Text 'src/api/ROCKProviderApi.h' 'struct\s+RockProviderApiDescriptorV1[\s\S]*providerApiByteSize[\s\S]*featureBits2' `
    'V1 must expose an immutable descriptor for safe table-extent discovery.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCKAPI_GetDescriptorV1' `
    'The public SDK must declare the independent V1 descriptor accessor.'
Require-Text 'src/exports.def' 'ROCKAPI_GetDescriptorV1' `
    'The provider descriptor must be exported independently of the function table.'
Require-Text 'src/api/ROCKProviderApi.h' 'sizeof\(RockProviderApi\)\s*==\s*720' `
    'The append-only V1 function table must retain its exact 90-slot x64 extent.'
Require-Text 'src/api/ROCKProviderApi.h' 'struct\s+RockProviderLimitsExtV1' `
    'Fixed capacities omitted by the legacy limits prefix must be discoverable through extended limits.'
Require-Text 'src/api/ROCKProviderApi.h' 'RockProviderStructureIdV1[\s\S]*getPublicStructureSizeV1' `
    'Consumers must be able to query exact public POD sizes.'
Require-Path 'src/api/ROCKProviderApiInternal.h' `
    'Runtime-only provider declarations must live outside the public SDK header.'
Reject-Text 'src/api/ROCKProviderApi.h' 'setPhysicsInteractionInstance|resolveWeaponPartTargetV1|currentHandInputSuppressionFlagsV1' `
    'The public SDK header must not leak ROCK runtime-only declarations.'
Require-Text 'src/api/ProviderLeasePolicy.h' 'exclusiveExpiryFrame[\s\S]*isActive[\s\S]*remainingFrames' `
    'All stateful publications must share the named exclusive lease-boundary policy.'
Require-Text 'src/api/ROCKApi.h' 'ROCK_API_VERSION\s*=\s*rock::provider::ROCK_PROVIDER_API_VERSION' `
    'ROCKApi alias must use the same version constant as the provider API.'
Require-Text 'src/api/ROCKProviderApi.h' 'enum\s+class\s+RockProviderResultV1' `
    'v1 must expose explicit result codes.'
Require-Text 'src/api/ROCKProviderApi.h' 'struct\s+RockProviderConsumerRegistrationV1' `
    'v1 must expose consumer registration.'
Require-Text 'src/api/ROCKProviderApi.h' 'struct\s+RockProviderLimitsV1' `
    'v1 must expose provider limits.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_MAX_CONSUMERS_V1\s*=\s*64' `
    'Public consumer registry capacity must be an explicit SDK limit.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_MAX_WEAPON_BODIES\s*=\s*8[\s\S]{0,400}ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1\s*=\s*100' `
    'The complete weapon evidence catalog must remain independent from the compact frame-snapshot body array.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' 'getProviderWeaponEvidenceDetailCountV1[\s\S]{0,500}ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1[\s\S]{0,700}copyProviderWeaponEvidenceDetailsV1[\s\S]{0,700}ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1' `
    'Weapon evidence count and copy queries must both enforce the advertised catalog bound.'
Require-Text 'src/api/ROCKApi.cpp' 'ROCKAPI_GetProviderApi\(\)' `
    'ROCKAPI_GetApi must return the same table as ROCKAPI_GetProviderApi.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'kRockIssuedOwnerTokenNamespace\s*=\s*0xA000''0000''0000''0000ull' `
    'Registered public owner tokens must be ROCK-issued and namespaced.'
Require-Text 'src/api/ROCKProviderApi.cpp' 's_externalBodies\.clearOwner\(ownerToken\)' `
    'Unregistering a consumer must release that owner external-body state.'
Require-Text 'src/physics-interaction/object/ExternalBodyRegistry.h' 'copyContactsForOwnerV1' `
    'Owner-filtered contact polling must be implemented in the external-body registry.'
Require-Text 'cmake/package.cmake' 'RPS_SDK_ROOT' `
    'Release packaging must consume the independent RPS_SDK repository.'
Require-Text 'cmake/package.cmake' 'file\(SHA256[\s\S]*SOURCE_HEADER_HASH[\s\S]*SDK_HEADER_HASH' `
    'Release packaging must reject drift between runtime and independent SDK headers.'
Require-Text 'cmake/package.cmake' 'file\(COPY "\$\{ROCK_PUBLIC_SDK_DIR\}"' `
    'Release packaging must copy the independently owned ROCK SDK tree.'
Reject-Text 'cmake/package.cmake' 'ROOT_DIR\}/SDK/ROCK' `
    'Release packaging must not consume the removed embedded SDK copy.'

Require-Text 'src/api/ROCKProviderApi.h' 'InteractionCommands' `
    'v1 must expose the interaction command consumer capability.'
Require-Text 'src/api/ROCKProviderApi.h' 'InteractionCommandQueue' `
    'v1 must expose the queued interaction command feature bit.'
Require-Text 'src/api/ROCKProviderApi.h' 'ForceGrabCommand' `
    'v1 must expose the force-grab command feature bit.'
Require-Text 'src/api/ROCKProviderApi.h' 'ForceReleaseCommand' `
    'v1 must expose the force-release command feature bit.'
Require-Text 'src/api/ROCKProviderApi.h' 'ThrownDropCommand' `
    'v1 must expose the thrown-drop command feature bit.'
Require-Text 'src/api/ROCKProviderApi.h' 'requestForceGrabV1' `
    'v1 function table must expose queued force-grab requests.'
Require-Text 'src/api/ROCKProviderApi.h' 'requestForceReleaseV1' `
    'v1 function table must expose queued force-release requests.'
Require-Text 'src/api/ROCKProviderApi.h' 'requestThrownDropV1' `
    'v1 function table must expose queued thrown-drop requests.'
Require-Text 'src/api/ROCKProviderApi.h' 'getInteractionCommandResultV1' `
    'v1 function table must expose queued command result polling.'
Require-Text 'src/api/ROCKProviderApi.h' 'HandInputSuppression' `
    'v1 must expose hand input suppression capability and feature names.'
Require-Text 'src/api/ROCKProviderApi.h' 'RockProviderHandInputSuppressionRequestV1' `
    'v1 must expose owner-token hand input suppression lease requests.'
Require-Text 'src/api/ROCKProviderApi.h' 'SuppressConfigModeChord' `
    'v1 must expose external config chord suppression flags.'
Require-Text 'src/api/ROCKProviderApi.h' 'SuppressNativeVats[\s\S]{0,300}SuppressNativeVans' `
    'v1 must expose independent native VATS-release and V.A.N.S.-hold suppression flags.'
Require-Text 'src/api/ROCKProviderApi.h' 'NativeVatsVansInputSuppression[\s\S]*supportsNativeVatsVansInputSuppressionV1' `
    'Consumers must be able to negotiate support for the new V1 suppression flags.'
Require-Text 'src/api/ROCKProviderApi.h' 'setHandInputSuppressionV1' `
    'v1 function table must append hand input suppression lease setup.'
Require-Text 'src/api/ROCKProviderApi.h' 'clearHandInputSuppressionV1' `
    'v1 function table must append hand input suppression lease clearing.'
Require-Text 'src/api/ROCKProviderApi.h' 'providerApiByteSize' `
    'v1 provider limits must expose the returned function table byte size for appended-slot negotiation.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_V1_HAND_INPUT_SUPPRESSION_TABLE_BYTES' `
    'SDK must expose a table-size guard for hand input suppression slots.'
Require-Text 'src/api/ROCKProviderApi.h' 'WeaponPartInteraction' `
    'v1 must expose weapon part interaction capability and feature names.'
Require-Text 'src/api/ROCKProviderApi.h' 'RockProviderWeaponPartTargetV1' `
    'v1 must expose weapon part target registrations.'
Require-Text 'src/api/ROCKProviderApi.h' 'RockProviderWeaponPartDriveTargetV1' `
    'v1 must expose weapon part drive targets.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_V1_WEAPON_PART_INTERACTION_TABLE_BYTES' `
    'SDK must expose a table-size guard for weapon part interaction slots.'
Require-Text 'src/api/ROCKProviderApi.h' 'supportsWeaponPartInteractionV1' `
    'SDK must expose safe feature/table helpers for weapon part interaction.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'kImplementedWeaponPartDriveMatcherFlagsV1' `
    'Weapon part drive targets must only accept concrete body/source/name matcher flags.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'hasValidWeaponPartTargetSemantics' `
    'Weapon part target API must range-check public semantic enum values before runtime casts.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'availableWeaponPartTargetSlotsForOwnerLocked' `
    'Weapon part target replacement must prove capacity before clearing existing owner registrations.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'availableWeaponPartDriveSlotsForOwnerLocked' `
    'Weapon part drive replacement must prove capacity before clearing existing owner registrations.'
Require-Text 'src/api/ROCKProviderApi.h' 'supportsForceGrabCommandV1' `
    'SDK must expose safe feature/table helpers for force-grab commands.'
Require-Text 'src/api/ROCKProviderApi.h' 'WeaponEmitters[\s\S]*RockProviderWeaponEmitterV1[\s\S]*ROCK_PROVIDER_API_V1_WEAPON_EMITTERS_TABLE_BYTES[\s\S]*supportsWeaponEmittersV1' `
    'Weapon emitter snapshots must remain feature- and table-size-gated inside API V1.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'maxWeaponEmitters\s*=\s*ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1' `
    'Provider limits must publish the bounded emitter capacity.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'maxAnimationPhaseCallbacks\s*=\s*[\s\S]{0,100}ROCK_PROVIDER_MAX_ANIMATION_PHASE_CALLBACKS_V1[\s\S]{0,250}maxHandVisualAuthorityPublications[\s\S]{0,250}maxNativeAnimationRuntimeProviders\s*=\s*1' `
    'Provider limits must publish every bounded animation-support capacity.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'providerApiByteSize\s*=\s*static_cast<std::uint32_t>\(sizeof\(RockProviderApi\)\)' `
    'Provider limits must report the current function table byte size.'
Require-Text 'src/api/ROCKProviderApi.h' 'EquippedWeaponHandlingAuthority[\s\S]*RockProviderEquippedWeaponHandlingRequestV1[\s\S]*RockProviderEquippedWeaponHandlingStateV1' `
    'API V1 must expose owner-bound equipped-weapon policy authority and runtime state.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_V1_EQUIPPED_WEAPON_HANDLING_AUTHORITY_TABLE_BYTES[\s\S]*supportsEquippedWeaponHandlingAuthorityV1' `
    'Equipped-weapon policy consumers must negotiate both feature bit and appended table size.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'maxEquippedWeaponHandlingAuthorities\s*=\s*1[\s\S]*maxEquippedWeaponHandlingLeaseFrames\s*=[\s\S]*ROCK_PROVIDER_MAX_EQUIPPED_WEAPON_HANDLING_LEASE_FRAMES_V1' `
    'Provider limits must publish the single-owner authority capacity and maximum lease.'
Require-Text 'src/api/ROCKProviderApi.h' 'EquippedWeaponHandRequest[\s\S]*RockProviderEquippedWeaponHandRequestV1[\s\S]*requestEquippedWeaponHandV1[\s\S]*ROCK_PROVIDER_API_V1_EQUIPPED_WEAPON_HAND_REQUEST_TABLE_BYTES[\s\S]*supportsEquippedWeaponHandRequestV1' `
    'V1 must expose an append-only, feature-gated exact-hand request for the currently equipped weapon.'
Require-Text 'src/api/ROCKProviderApi.h' 'WorldRaycasts[\s\S]*RockProviderWorldRaycastRequestV1[\s\S]*RockProviderWorldRaycastResultV1[\s\S]*queryWorldRaycastV1[\s\S]*ROCK_PROVIDER_API_V1_WORLD_RAYCASTS_TABLE_BYTES[\s\S]*supportsWorldRaycastsV1' `
    'V1 must expose an append-only, feature-gated, pointer-free world-raycast query.'
Require-Text 'src/api/ROCKProviderApi.h' 'ColliderVisualizationOverride[\s\S]*RockProviderColliderVisualizationRequestV1[\s\S]*setColliderVisualizationOverrideV1[\s\S]*clearColliderVisualizationOverrideV1[\s\S]*ROCK_PROVIDER_API_V1_COLLIDER_VISUALIZATION_OVERRIDE_TABLE_BYTES[\s\S]*supportsColliderVisualizationOverrideV1' `
    'V1 must expose an append-only, feature-gated exact-collider visualization override.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'provider_collider_visualization::copySnapshot[\s\S]{0,900}FocusedWeaponPart[\s\S]{0,400}PublishFrame\(focusedFrame\)[\s\S]{0,100}return' `
    'Focused collider visualization must replace the complete config-driven overlay frame.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiSetColliderVisualizationOverrideV1[\s\S]{0,1200}onAnimationOwnerThread\(\)[\s\S]{0,1800}ColliderVisualizationOverride[\s\S]{0,1200}s_lastSnapshot\.weaponGenerationKey[\s\S]{0,900}isProviderWeaponBodyCurrentV1[\s\S]{0,600}provider_collider_visualization::set' `
    'Collider focus requests must be owner-thread/capability gated and bind exact identity from the complete current weapon-body catalog.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' 'isProviderWeaponBodyCurrentV1[\s\S]{0,500}getWeaponBodySnapshotAtomic[\s\S]{0,500}snapshot\.bodyIds' `
    'Collider focus identity validation must scan the complete generation-bound weapon body publication.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'copySnapshot\([\s\S]{0,300}isProviderWeaponBodyCurrentV1[\s\S]{0,500}provider_collider_visualization::prune\([\s\S]{0,500}colliderVisualizationBodyCurrent' `
    'Collider focus leases must invalidate against the complete current weapon body set before callbacks.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiUnregisterConsumerV1[\s\S]{0,3500}provider_collider_visualization::clear\(ownerToken\)' `
    'Consumer teardown must release its exclusive collider focus override.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'maxWorldRaycastsPerOwnerPerFrame\s*=\s*[\s\S]{0,100}ROCK_PROVIDER_MAX_WORLD_RAYCASTS_PER_OWNER_PER_FRAME_V1' `
    'Extended limits must publish the per-owner per-frame raycast budget.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiRequestEquippedWeaponHandV1[\s\S]{0,500}onAnimationOwnerThread\(\)[\s\S]{0,1800}EquippedWeaponHandlingAuthority[\s\S]{0,1000}FiringGripOwnership[\s\S]{0,800}AmbidextrousHandoff[\s\S]{0,1000}requestProviderEquippedWeaponHandV1' `
    'Exact-hand requests must remain game-thread-only and bound to the caller''s active handling authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'requestProviderEquippedWeaponHandV1[\s\S]{0,1800}currentEquippedWeaponForm\(\)[\s\S]{0,900}request\.weaponGenerationKey[\s\S]{0,1500}EquippedWeaponHandAssignmentSource::Provider' `
    'Exact-hand requests must bind value identity before arming the canonical equipped-weapon assignment path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'serviceEquippedWeaponHandAssignment\([\s\S]{0,16000}ownsEquippedWeaponHandlingAuthorityV1[\s\S]{0,12000}beginPersistentEquippedCarry' `
    'Provider hand assignments must continuously revalidate authority and reuse ROCK''s canonical left-carry executor.'
Require-Text 'src/api/ROCKProviderApi.h' 'DebugOverlayPublication[\s\S]*RockProviderDebugOverlayLineV1[\s\S]*RockProviderDebugOverlayTextV1[\s\S]*RockProviderDebugOverlayPublicationV1' `
    'API V1 must expose bounded owner-scoped debug overlay publication values.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_V1_DEBUG_OVERLAY_PUBLICATION_TABLE_BYTES[\s\S]*supportsDebugOverlayPublicationV1' `
    'Debug publishers must negotiate both the V1 feature bit and appended table size.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'maxDebugOverlayPublishers\s*=[\s\S]*ROCK_PROVIDER_MAX_DEBUG_OVERLAY_PUBLISHERS_V1[\s\S]*maxDebugOverlayLinesPerPublisher[\s\S]*maxDebugOverlayTextPerPublisher[\s\S]*maxDebugOverlayLines\s*=[\s\S]*ROCK_PROVIDER_MAX_DEBUG_OVERLAY_LINES_V1[\s\S]*maxDebugOverlayText\s*=[\s\S]*ROCK_PROVIDER_MAX_DEBUG_OVERLAY_TEXT_V1' `
    'Provider limits must publish every bounded debug-overlay capacity.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiPublishDebugOverlayV1[\s\S]*validateRegisteredOwnerCapabilityLocked[\s\S]*DebugOverlayPublication[\s\S]*provider_debug_overlay::publish' `
    'Debug overlay publication must remain registered-owner and capability gated.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'unregisterConsumer[\s\S]*provider_debug_overlay::clear\(ownerToken\)' `
    'Unregistering a consumer must clear its debug overlay publication.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'provider_debug_overlay::hasContent[\s\S]*static provider_debug_overlay::Snapshot[\s\S]*provider_debug_overlay::copySnapshot[\s\S]*drawProviderOverlay[\s\S]*coloredLineEntries[\s\S]*RockProviderDebugOverlayTextFlagV1::WorldAnchored' `
    'The existing ROCK stereo renderer must consume copied provider lines and text.'
Require-Text 'src/api/ROCKProviderApi.h' 'PresentedVisual[\s\S]*PresentedHandFrames[\s\S]*ROCK_PROVIDER_API_V1_PRESENTED_HAND_FRAMES_TABLE_BYTES[\s\S]*supportsPresentedHandFramesV1' `
    'API V1 must distinguish the final hFRIK-presented hand from ROCK''s root-flattened authority frame.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiGetPresentedHandFrameV1[\s\S]*onAnimationOwnerThread\(\)[\s\S]*frik_visual_authority::getHandWorldTransform[\s\S]*RockProviderHandFrameFlagV1::PresentedVisual' `
    'Presented hand queries must read the final hFRIK transform on the animation owner thread.'
Require-Text 'src/api/ROCKProviderApi.h' 'NativeAnimationAuthority[\s\S]*RockProviderNativeAnimationAuthorityRequestV1' `
    'API V1 must expose selective native animation authority as a registered consumer capability.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_V1_NATIVE_ANIMATION_AUTHORITY_TABLE_BYTES[\s\S]*supportsNativeAnimationAuthorityV1' `
    'Native animation authority consumers must negotiate both feature bit and appended table size.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'clearNativeAnimationAuthorityForOwnerLocked\(ownerToken\)' `
    'Unregistering a consumer must clear its native animation authority lease.'
Require-Text 'src/api/ROCKProviderApi.h' 'RockProviderForceReleaseFlagV1[\s\S]*UseVelocityHavok' `
    'Force release must expose an explicit trusted Havok velocity flag.'
Require-Text 'src/api/ROCKProviderApi.h' 'RockProviderForceReleaseRequestV1[\s\S]*linearVelocityHavok[\s\S]*angularVelocityRadiansPerSecond' `
    'Force release must expose trusted linear and angular Havok velocity payloads.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiRequestForceGrabV1' `
    'Provider glue must implement queued force-grab request validation.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'request->targetFormId\s*==\s*0' `
    'Force-grab API validation must require stable FormID identity before queueing.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'isFiniteVector3\(request->preferredGrabPointGame\)' `
    'Force-grab API validation must finite-check caller-supplied preferred grab points.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'TESForm::GetFormByID<RE::TESObjectREFR>' `
    'Force-grab execution must resolve targets on the ROCK update path by FormID.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiRequestForceReleaseV1' `
    'Provider glue must implement queued force-release request validation.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiRequestThrownDropV1' `
    'Provider glue must implement queued thrown-drop request validation.'
Require-Text 'src/api/ROCKProviderApi.cpp' 's_interactionCommands' `
    'Provider glue must use a bounded interaction command queue.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'processProviderInteractionCommands' `
    'ROCK runtime must execute provider commands from the update path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.grabSelectedObject' `
    'Force grab execution must commit through the existing dynamic grab path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'releaseGrabbedObject' `
    'Force release and thrown drop must use the existing release path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'applyReleaseVelocitySnapshot' `
    'Trusted release/drop velocity must apply through the existing release velocity path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'applyCapturedReleaseVelocity\s*=\s*isThrownDropCommand\s*&&\s*!applyRequestedVelocity' `
    'Provider force release must not reuse captured controller throw velocity by default.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'currentHandInputSuppressionFlagsV1' `
    'ROCK runtime must observe provider hand input suppression leases.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'deferredGrabRelease' `
    'Suppressed grab releases must be deferred instead of dropping held objects during external config chords.'
Reject-Text 'src/api/ROCKProviderApi.cpp' 'grabSelectedObject|releaseGrabbedObject|applyReleaseVelocitySnapshot' `
    'Provider API glue must only enqueue commands, not mutate hand state directly.'
Reject-Text 'src/api/ROCKProviderApi.cpp' 'reinterpret_cast<RE::TESObjectREFR\*>\(request->targetRefr\)' `
    'Provider API glue must not dereference caller-supplied reference pointers.'
Reject-Text 'src/api/ROCKProviderApi.h' 'getWeaponEvidenceDescriptors|RockProviderWeaponEvidenceDescriptor|getExternalContactSnapshotV1' `
    'Public API must not expose redundant shallow weapon evidence or unowned contact snapshots.'
Reject-Text 'src/api/ROCKProviderApi.h' 'DiagnosticOverlay|DiagnosticInput|publishDiagnosticOverlay|getDiagnosticInputSnapshotV1|setDiagnosticInputSuppressionV1' `
    'Public API must not expose diagnostic/probe control surfaces.'
Require-Text 'src/api/ROCKProviderApi.h' 'Live scene/physics readbacks[\s\S]{0,220}WrongThread' `
    'The public header must document the fail-closed game-thread contract for live readbacks.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiCopyWeaponPartDriveApplicationResultsV1[\s\S]{0,900}!onAnimationOwnerThread\(\)[\s\S]{0,120}WrongThread' `
    'Weapon-part drive result readback must reject non-owner threads.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiGetScopeSightStateV1[\s\S]{0,520}queryProviderScopeSightStateV1[\s\S]{0,80}true\);' `
    'Scope sight readback must require the animation owner thread.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiCopySemanticHandContactsV1[\s\S]{0,1000}!onAnimationOwnerThread\(\)[\s\S]{0,120}WrongThread' `
    'Semantic hand-contact readback must reject non-owner threads.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiCopyPlayerColliderDescriptorsV1[\s\S]{0,900}!onAnimationOwnerThread\(\)[\s\S]{0,120}WrongThread' `
    'Player collider readback must reject non-owner threads.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiGetHandCollisionAvailabilityV1[\s\S]{0,620}queryProviderHandCollisionAvailabilityV1[\s\S]{0,80}true\);' `
    'Hand collision availability must require the animation owner thread.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'registerBodiesForScopeDetailed[\s\S]{0,600}RegistrationResult::CapacityFull[\s\S]{0,220}RockProviderResultV1::CapacityFull[\s\S]{0,220}RegistrationResult::OwnerConflict[\s\S]{0,220}RockProviderResultV1::OwnerConflict' `
    'Scoped external-body registration must preserve capacity and ownership failure semantics.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'clearScope\(ownerToken, scopeToken\)[\s\S]{0,160}RockProviderResultV1::TargetUnavailable' `
    'Clearing an unknown external-body scope must report target unavailability.'
Require-Text 'src/api/ROCKProviderApi.h' 'TouchGrabTargets[\s\S]*RockProviderTouchGrabTargetV1[\s\S]*RockProviderTouchGrabStateV1' `
    'API V1 must expose bounded provider-scoped touch-grab targets and states.'
Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_V1_TOUCH_GRAB_TARGETS_TABLE_BYTES[\s\S]*supportsTouchGrabTargetsV1' `
    'Touch-grab consumers must negotiate both the V1 feature bit and appended table extent.'
Require-Text 'src/api/ROCKProviderApi.h' 'maxTouchGrabTargets[\s\S]*maxTouchGrabScopes[\s\S]*maxTouchGrabTargetLeaseFrames' `
    'The extended V1 limits query must expose every bounded touch-grab registry capacity.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiSetTouchGrabTargetsForScopeV1[\s\S]{0,1500}validateRegisteredOwnerCapabilityLocked[\s\S]{0,180}TouchGrabTargets' `
    'Touch-grab target publication must remain registered-owner and capability gated.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'unregisterConsumer[\s\S]*s_touchGrabTargets\.clearOwner\(ownerToken\)' `
    'Unregistering a consumer must revoke all of its touch-grab targets.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'collectFreshSurfaceContacts\([\s\S]*ContactSource::[\s\S]{0,80}DynamicSurface' `
    'Grip-edge touch acquisition must consume the dedicated dynamic surface contact channel.'
Require-Text 'src/api/ROCKProviderApi.h' 'TouchGrab\s*=\s*1u\s*<<\s*12[\s\S]*FixedSurfaceLatch\s*=\s*1u\s*<<\s*13[\s\S]*GlobalSurfaceLatch\s*=\s*1u\s*<<\s*14' `
    'Hand-interaction state must distinguish touch grabs, fixed surface latches, and the INI global mode.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' '_touchGrabRuntime\.getHandReport\([\s\S]*Phase::Holding[\s\S]*reservedTargetIdentity[\s\S]*referenceNativeHandle[\s\S]*targetFormId[\s\S]*primaryBodyId[\s\S]*TargetKind::WorldSurface[\s\S]*Flag::TouchGrab[\s\S]*Flag::FixedSurfaceLatch[\s\S]*Flag::GlobalSurfaceLatch' `
    'The public hand state must identify every active surface latch and its resolved target body.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'sameHandTarget[\s\S]{0,500}reservedTargetIdentity[\s\S]{0,300}primaryBodyId' `
    'Touch-grab native identity and body changes must advance the public target sequence.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'Releasing[\s\S]{0,1500}touchGrabClassificationFlags[\s\S]{0,1500}previousHand\.flags' `
    'The one-frame release state must retain touch-grab and global-surface classification.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' 'outState\.authoredSupportGrip[\s\S]{0,120}report\.authoredSupportGrip[\s\S]*out\.actionRole[\s\S]{0,120}descriptor\.semantic\.actionRole' `
    'ROCK V1 must publish exact authored-grip provenance and semantic action roles without consumer inference.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'tryTargetClass\([\s\S]{0,180}TargetClass::[\s\S]{0,80}Explicit\)\s*\|\|[\s\S]{0,180}tryTargetClass\([\s\S]{0,180}TargetClass::[\s\S]{0,80}Wildcard\)' `
    'Exact mechanism targets must be attempted before wildcard fixed-surface targets.'
Require-Text 'src/physics-interaction/object/PhysicsBodyClassifier.h' 'motionType\s*==\s*BodyMotionType::Static[\s\S]{0,120}BodyRejectReason::StaticMotion' `
    'Ordinary loose-object classification must continue rejecting static motion.'

$providerHeader = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/api/ROCKProviderApi.h')
$expectedProviderFunctions = [string[]]@(
    'getVersion',
    'getModVersion',
    'isProviderReady',
    'registerFrameCallback',
    'unregisterFrameCallback',
    'getFrameSnapshot',
    'queryWeaponContactAtPoint',
    'clearExternalBodies',
    'setOffhandInteractionReservation',
    'registerExternalBodiesV1',
    'getWeaponEvidenceDetailCountV1',
    'copyWeaponEvidenceDetailsV1',
    'getWeaponEvidenceDetailPointCountV1',
    'copyWeaponEvidenceDetailPointsV1',
    'getBodyContactSnapshotV1',
    'getPrimaryHandV1',
    'getOffhandHandV1',
    'getHandFrameV1',
    'registerConsumerV1',
    'unregisterConsumerV1',
    'getGrantedCapabilitiesV1',
    'getProviderLimitsV1',
    'getExternalContactSnapshotForOwnerV1',
    'requestForceGrabV1',
    'getInteractionCommandResultV1',
    'requestForceReleaseV1',
    'requestThrownDropV1',
    'setHandInputSuppressionV1',
    'clearHandInputSuppressionV1',
    'setWeaponPartTargetsV1',
    'clearWeaponPartTargetsV1',
    'setWeaponPartDriveTargetsV1',
    'clearWeaponPartDriveTargetsV1',
    'queryEquippedWeaponClassificationV1',
    'getWeaponPartGripStateV1',
    'getRawWandButtonStateV1',
    'isNativePipboyInputSuppressedV1',
    'getWeaponEmitterCountV1',
    'copyWeaponEmittersV1',
    'setNativeAnimationAuthorityV1',
    'clearNativeAnimationAuthorityV1',
    'getNativeAnimationAuthorityStateV1',
    'registerAnimationPhaseCallbackV1',
    'unregisterAnimationPhaseCallbackV1',
    'getEquippedWeaponGripStateV1',
    'setHandVisualAuthorityV1',
    'clearHandVisualAuthorityV1',
    'publishNativeAnimationRuntimeV1',
    'setEquippedWeaponHandlingAuthorityV1',
    'clearEquippedWeaponHandlingAuthorityV1',
    'getEquippedWeaponHandlingStateV1',
    'publishDebugOverlayV1',
    'clearDebugOverlayV1',
    'getPresentedHandFrameV1',
    'getProviderLimitsExtV1',
    'getPublicStructureSizeV1',
    'registerFrameCallbackForOwnerV1',
    'unregisterFrameCallbackForOwnerV1',
    'getHandInteractionStateV1',
    'copyProviderEventsSinceV1',
    'getEquippedWeaponStateV1',
    'registerExternalBodiesForScopeV1',
    'clearExternalBodiesForScopeV1',
    'copyExternalContactsSinceV1',
    'queryWeaponPartTargetResolutionV1',
    'copyWeaponPartPoseSnapshotV1',
    'copyWeaponPartDriveApplicationResultsV1',
    'getScopeSightStateV1',
    'getWeaponCompositionStateV1',
    'copyWeaponCompositionEntriesV1',
    'getSelectedAuthoredGripPoseV1',
    'getPresentedHandPoseV1',
    'copySemanticHandContactsV1',
    'copyPlayerColliderDescriptorsV1',
    'getHandCollisionAvailabilityV1',
    'cancelInteractionCommandV1',
    'getHandInputSuppressionStateV1',
    'acquireOffhandReservationV1',
    'renewOffhandReservationV1',
    'releaseOffhandReservationV1',
    'getOffhandReservationStateV1',
    'clearNativeAnimationRuntimeV1',
    'setTouchGrabTargetsForScopeV1',
    'clearTouchGrabTargetsForScopeV1',
    'copyTouchGrabStatesForScopeV1',
    'requestTouchGrabYieldV1',
    'requestEquippedWeaponHandV1',
    'queryWorldRaycastV1',
    'setColliderVisualizationOverrideV1',
    'clearColliderVisualizationOverrideV1'
)
Require-SequenceEqual 'ROCKProviderApi function pointer order' (Get-ProviderFunctionNames $providerHeader) $expectedProviderFunctions

if ($failures.Count -gt 0) {
    Write-Host 'PublicApiLaunchSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'PublicApiLaunchSourceTests passed.' -ForegroundColor Green
