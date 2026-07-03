param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

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

function Require-FilesEqual {
    param(
        [string]$LeftRelativePath,
        [string]$RightRelativePath,
        [string]$Message
    )

    $leftPath = Join-Path $Root $LeftRelativePath
    $rightPath = Join-Path $Root $RightRelativePath
    if (-not (Test-Path -LiteralPath $leftPath) -or -not (Test-Path -LiteralPath $rightPath)) {
        $failures.Add("$LeftRelativePath / $RightRelativePath`: missing file for sync check")
        return
    }

    $left = Get-Content -Raw -LiteralPath $leftPath
    $right = Get-Content -Raw -LiteralPath $rightPath
    if ($left -cne $right) {
        $failures.Add("$LeftRelativePath / $RightRelativePath`: $Message")
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

Require-Path 'SDK/ROCK/include/ROCKProviderApi.h' 'Public SDK must ship the provider header.'
Require-Path 'SDK/ROCK/include/ROCKApi.h' 'Public SDK must ship the API alias header.'
Require-Path 'SDK/ROCK/examples/MinimalProviderConsumer.cpp' 'A minimal provider consumer example must be packaged with the SDK.'

Require-FilesEqual 'src/api/ROCKProviderApi.h' 'SDK/ROCK/include/ROCKProviderApi.h' `
    'SDK provider header must stay byte-for-byte synced with the source ABI header.'
Require-FilesEqual 'src/api/ROCKApi.h' 'SDK/ROCK/include/ROCKApi.h' `
    'SDK API alias header must stay byte-for-byte synced with the source ABI header.'

Require-Text 'src/api/ROCKProviderApi.h' 'ROCK_PROVIDER_API_VERSION\s*=\s*1' `
    'Provider API version must be v1.'
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
Require-Text 'src/api/ROCKApi.cpp' 'ROCKAPI_GetProviderApi\(\)' `
    'ROCKAPI_GetApi must return the same table as ROCKAPI_GetProviderApi.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'kRockIssuedOwnerTokenNamespace\s*=\s*0xA000''0000''0000''0000ull' `
    'Registered public owner tokens must be ROCK-issued and namespaced.'
Require-Text 'src/api/ROCKProviderApi.cpp' 's_externalBodies\.clearOwner\(ownerToken\)' `
    'Unregistering a consumer must release that owner external-body state.'
Require-Text 'src/physics-interaction/object/ExternalBodyRegistry.h' 'copyContactsForOwnerV1' `
    'Owner-filtered contact polling must be implemented in the external-body registry.'
Require-Text 'cmake/package.cmake' 'SDK/ROCK' `
    'Release packaging must include the SDK directory.'
Require-Text 'cmake/package.cmake' 'src/api/ROCKProviderApi\.h' `
    'Release packaging must copy the source provider ABI header into the SDK include directory.'
Require-Text 'cmake/package.cmake' 'src/api/ROCKApi\.h' `
    'Release packaging must copy the API alias header into the SDK include directory.'

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
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'providerPartAuthorityStillCurrent' `
    'Provider-authorized weapon part grips must revalidate owner target authority while active.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'resolveCurrentSupportAttachmentRoot' `
    'Weapon part support attachment roots must be current-tree validated before dereference.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'attachmentWeaponLocal' `
    'Part-carry source-part grips must solve weapon authority from the captured source-to-weapon frame.'
Require-Text 'src/api/ROCKProviderApi.h' 'supportsForceGrabCommandV1' `
    'SDK must expose safe feature/table helpers for force-grab commands.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'providerApiByteSize\s*=\s*static_cast<std::uint32_t>\(sizeof\(RockProviderApi\)\)' `
    'Provider limits must report the current function table byte size.'
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
    'getWeaponPartGripStateV1'
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
