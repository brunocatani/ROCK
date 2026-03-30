param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$ResolvedRoot = (Resolve-Path -LiteralPath $Root).Path
$NativeSourceRoot = (Resolve-Path -LiteralPath (Join-Path $ResolvedRoot 'src/physics-interaction/native')).Path.TrimEnd([char[]]@('\', '/')) + [System.IO.Path]::DirectorySeparatorChar

function Get-RelativePathCompat {
    param([string]$Path)

    $fullPath = (Resolve-Path -LiteralPath $Path).Path
    $rootPath = $ResolvedRoot.TrimEnd([char[]]@('\', '/'))
    if ($fullPath.StartsWith($rootPath, [System.StringComparison]::OrdinalIgnoreCase)) {
        return $fullPath.Substring($rootPath.Length).TrimStart([char[]]@('\', '/'))
    }

    return $fullPath
}

function Read-SourceFiles {
    param([string]$RelativeRoot)

    Get-ChildItem -LiteralPath (Join-Path $Root $RelativeRoot) -Recurse -File |
        Where-Object { $_.Extension -in @('.h', '.cpp', '.hpp', '.inl') }
}

$nonNativeFiles = Read-SourceFiles 'src/physics-interaction' |
    Where-Object { -not $_.FullName.StartsWith($NativeSourceRoot, [System.StringComparison]::OrdinalIgnoreCase) }

$forbiddenOutsideNative = @(
    @{
        Pattern = 'REL::Offset\(offsets::kFunc_ActivateBody\)'
        Message = 'Direct hknp ActivateBody calls must stay behind havok_runtime::activateBody.'
    },
    @{
        Pattern = 'REL::Offset\(offsets::kFunc_SetBodyVelocityDeferred\)'
        Message = 'Direct SetBodyVelocityDeferred calls must stay behind havok_runtime::setBodyVelocityDeferred.'
    },
    @{
        Pattern = 'REL::Offset\(offsets::kFunc_SetBodyTransformDeferred\)'
        Message = 'Direct SetBodyTransformDeferred calls must stay behind havok_runtime::setBodyTransformDeferred.'
    },
    @{
        Pattern = 'REL::Offset\(offsets::kFunc_EnableBodyFlags\)'
        Message = 'Direct EnableBodyFlags calls must stay behind havok_runtime::enableBodyFlags.'
    },
    @{
        Pattern = 'REL::Offset\(offsets::kFunc_DisableBodyFlags\)'
        Message = 'Direct DisableBodyFlags calls must stay behind havok_runtime::disableBodyFlags.'
    },
    @{
        Pattern = 'REL::Offset\(offsets::kData_CollisionFilterSingleton\)'
        Message = 'Collision filter singleton fallback must stay behind havok_runtime::getQueryFilterRefWithFallback.'
    },
    @{
        Pattern = '->GetBody\s*\('
        Message = 'High-level modules must use guarded havok_runtime body access instead of direct hknpWorld::GetBody.'
    },
    @{
        Pattern = 'kCollisionObject_PhysSystemPtr'
        Message = 'Collision-object physics-system layout reads must stay behind havok_runtime helpers.'
    },
    @{
        Pattern = 'kFilter_CollisionMatrix'
        Message = 'Collision filter matrix layout access must stay behind havok_runtime::getCollisionFilterMatrix.'
    },
    @{
        Pattern = 'kMotion_PropertiesId'
        Message = 'hknp motion-property id reads must stay behind havok_runtime::tryReadBodyMotionPropertiesId.'
    },
    @{
        Pattern = 'kMotion_MaxLinearVelocityPacked|kMotion_MaxAngularVelocityPacked'
        Message = 'hknp motion velocity-cap diagnostics must stay behind havok_runtime helpers.'
    },
    @{
        Pattern = 'motionPtr\s*\+\s*0x3A|motionPtr\s*\+\s*0x3C|worldPtr\s*\+\s*0x5D0|libraryPtr\s*\+\s*0x28|libraryPtr\s*\+\s*0x30'
        Message = 'Raw hknp motion diagnostic offsets must stay behind havok_runtime helpers.'
    },
    @{
        Pattern = 'kFunc_MaterialCtor|kFunc_MaterialLibrary_AddMaterial'
        Message = 'Generated collider material registration must stay behind havok_material_registry::registerGeneratedBodyMaterial.'
    },
    @{
        Pattern = 'reinterpret_cast<char\*>\(world\)\s*\+\s*0x5C8'
        Message = 'hknpWorld material-library layout reads must stay behind havok_material_registry::registerGeneratedBodyMaterial.'
    }
)

foreach ($file in $nonNativeFiles) {
    $text = Get-Content -Raw -LiteralPath $file.FullName
    foreach ($rule in $forbiddenOutsideNative) {
        if ($text -match $rule.Pattern) {
            $relative = Get-RelativePathCompat $file.FullName
            $failures.Add("$relative`: $($rule.Message)")
        }
    }
}

$requiredNativeText = @{
    'src/physics-interaction/native/HavokRuntime.h' = @(
        'getHknpWorldFromBhk',
        'getQueryFilterRefWithFallback',
        'getPhysicsSystemFromCollisionObject',
        'getPhysicsSystemInstance',
        'forEachPhysicsSystemBodyId',
        'getCollisionFilterMatrix',
        'tryReadBodyMotionPropertiesId',
        'tryReadMotionVelocityCaps',
        'snapshotMotionPropertiesLibrary',
        'setBodyTransformDeferred',
        'enableBodyFlags',
        'disableBodyFlags'
    )
    'src/physics-interaction/core/PhysicsInteraction.cpp' = @(
        'havok_runtime::getHknpWorldFromBhk',
        'havok_runtime::getCollisionFilterMatrix',
        'havok_runtime::tryReadBodyMotionPropertiesId',
        'PhysicsInteractionFrame.inl'
    )
    'src/physics-interaction/hand/HandGrab.cpp' = @(
        'havok_runtime::tryReadMotionVelocityCaps',
        'havok_runtime::snapshotMotionPropertiesLibrary'
    )
    'src/physics-interaction/native/HavokMaterialRegistry.h' = @(
        'registerGeneratedBodyMaterial'
    )
    'src/physics-interaction/native/HavokMaterialRegistry.cpp' = @(
        'kHknpWorldMaterialLibraryOffset',
        'kFunc_MaterialCtor',
        'kFunc_MaterialLibrary_AddMaterial'
    )
    'src/physics-interaction/native/CharacterControllerRuntime.cpp' = @(
        'tryGetPlayerCharacterController',
        'currentProcess',
        'middleHigh',
        'charController.get()',
        '__try'
    )
    'src/physics-interaction/hand/HandBoneColliderSet.cpp' = @(
        'havok_material_registry::registerGeneratedBodyMaterial'
    )
    'src/physics-interaction/body/BodyBoneColliderSet.cpp' = @(
        'havok_material_registry::registerGeneratedBodyMaterial'
    )
}

foreach ($entry in $requiredNativeText.GetEnumerator()) {
    $path = Join-Path $Root $entry.Key
    $text = Get-Content -Raw -LiteralPath $path
    foreach ($token in $entry.Value) {
        if ($text -notmatch [regex]::Escape($token)) {
            $failures.Add("$($entry.Key): expected native-boundary token '$token'.")
        }
    }
}

$physicsHooksPath = 'src/physics-interaction/core/PhysicsHooks.cpp'
$physicsHooksText = Get-Content -Raw -LiteralPath (Join-Path $Root $physicsHooksPath)
foreach ($token in @(
        'constexpr std::array<std::uint8_t, 14> expectedPrefix',
        '"ProcessConstraintsCallback"',
        'offsets::kFunc_ProcessConstraintsCallback',
        'expectedPrefix.data()',
        'character_controller_runtime::tryGetPlayerCharacterController')) {
    if ($physicsHooksText -notmatch [regex]::Escape($token)) {
        $failures.Add("$physicsHooksPath`: player character-controller hooks must keep validated ProcessConstraintsCallback trampoline and native runtime controller helper token '$token'.")
    }
}
if ($physicsHooksText -match 'currentProcess->middleHigh') {
    $failures.Add("$physicsHooksPath`: direct Actor currentProcess/middleHigh character-controller layout access must stay behind character_controller_runtime.")
}
if ($physicsHooksText -match 'memcpy\(trampolineMem,\s*targetAddr,\s*STOLEN_BYTES\)') {
    $failures.Add("$physicsHooksPath`: ProcessConstraintsCallback must use validated installEntryTrampolineHook, not unvalidated byte copying.")
}

$nearbyDampingPath = 'src/physics-interaction/grab/NearbyGrabDamping.cpp'
$nearbyDampingText = Get-Content -Raw -LiteralPath (Join-Path $Root $nearbyDampingPath)
foreach ($token in @(
        'reserveMotionDampingWriteOrAttachExisting',
        'abortMotionDampingWrite',
        'commitMotionDampingWrite',
        'retryPendingFinalLeaseRestores',
        'tryFindLiveBodyForMotion')) {
    if ($nearbyDampingText -notmatch [regex]::Escape($token)) {
        $failures.Add("$nearbyDampingPath`: nearby damping lease writes must keep the two-phase native write/restore scaffolding token '$token'.")
    }
}
if ($nearbyDampingText -match 'getOrCreateDampedPropertyIdLocked') {
    $failures.Add("$nearbyDampingPath`: hknp motion-property addEntry must not be hidden behind a locked helper.")
}

<#
    File consolidation is guarded as an architecture boundary, not a cosmetic
    preference. These headers were intentionally folded into domain-owned packs
    so weapon/grab/hand policy surfaces stay discoverable while preserving the
    original namespaces and behavior. Reintroducing one-off files with the same
    roles makes the ownership split drift back toward the pre-refactor layout.
#>
$consolidatedHeaders = @(
    @{ Domain = 'weapon'; Name = 'WeaponSemanticTypes.h' },
    @{ Domain = 'weapon'; Name = 'WeaponCollisionEvidence.h' },
    @{ Domain = 'weapon'; Name = 'WeaponCollisionLimits.h' },
    @{ Domain = 'weapon'; Name = 'WeaponPartClassifier.h' },
    @{ Domain = 'weapon'; Name = 'WeaponSemanticHullBudget.h' },
    @{ Domain = 'weapon'; Name = 'WeaponCollisionGroupingPolicy.h' },
    @{ Domain = 'weapon'; Name = 'WeaponCollisionGeometryMath.h' },
    @{ Domain = 'weapon'; Name = 'WeaponInteractionProbeMath.h' },
    @{ Domain = 'weapon'; Name = 'WeaponSupportAuthorityPolicy.h' },
    @{ Domain = 'weapon'; Name = 'WeaponSupportGripPolicy.h' },
    @{ Domain = 'weapon'; Name = 'WeaponSupportThumbPosePolicy.h' },
    @{ Domain = 'weapon'; Name = 'WeaponTwoHandedGripMath.h' },
    @{ Domain = 'weapon'; Name = 'WeaponTwoHandedSolver.h' },
    @{ Domain = 'weapon'; Name = 'WeaponAuthorityLifecyclePolicy.h' },
    @{ Domain = 'weapon'; Name = 'WeaponVisualAuthorityMath.h' },
    @{ Domain = 'weapon'; Name = 'WeaponMuzzleAuthorityMath.h' },
    @{ Domain = 'weapon'; Name = 'WeaponVisualCompositionPolicy.h' },
    @{ Domain = 'weapon'; Name = 'WeaponDebugNotificationPolicy.h' },
    @{ Domain = 'weapon'; Name = 'WeaponInteractionRouter.h' },
    @{ Domain = 'weapon'; Name = 'OffhandInteractionReservation.h' },
    @{ Domain = 'grab'; Name = 'ActiveObjectPrepPolicy.h' },
    @{ Domain = 'grab'; Name = 'ActiveGrabBodyLifecycle.h' },
    @{ Domain = 'grab'; Name = 'CanonicalGrabFrame.h' },
    @{ Domain = 'grab'; Name = 'GrabInteractionPolicy.h' },
    @{ Domain = 'grab'; Name = 'GrabFrameMath.h' },
    @{ Domain = 'grab'; Name = 'PullMotionMath.h' },
    @{ Domain = 'grab'; Name = 'GrabContactSourcePolicy.h' },
    @{ Domain = 'grab'; Name = 'GrabContactEvidencePolicy.h' },
    @{ Domain = 'grab'; Name = 'GrabContactPatchMath.h' },
    @{ Domain = 'grab'; Name = 'GrabMultiFingerContactMath.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerPoseMath.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerPoseRuntime.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerLocalTransformMath.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerLocalTransformRuntime.h' },
    @{ Domain = 'grab'; Name = 'HeldObjectBodySetPolicy.h' },
    @{ Domain = 'grab'; Name = 'HeldObjectDampingMath.h' },
    @{ Domain = 'grab'; Name = 'HeldObjectPhysicsMath.h' },
    @{ Domain = 'grab'; Name = 'GrabHeldResponse.h' },
    @{ Domain = 'grab'; Name = 'HeldPlayerSpaceMath.h' },
    @{ Domain = 'grab'; Name = 'HeldGrabCharacterControllerPolicy.h' },
    @{ Domain = 'grab'; Name = 'GrabTransformTelemetry.h' },
    @{ Domain = 'grab'; Name = 'GrabTransformTelemetryOverlay.h' },
    @{ Domain = 'hand'; Name = 'DirectSkeletonBoneReader.h' },
    @{ Domain = 'hand'; Name = 'HandBoneCache.h' },
    @{ Domain = 'hand'; Name = 'RootFlattenedFingerSkeletonRuntime.h' },
    @{ Domain = 'hand'; Name = 'HandFrameResolver.h' },
    @{ Domain = 'hand'; Name = 'HandColliderSemanticTypes.h' },
    @{ Domain = 'hand'; Name = 'HandBoneColliderGeometryMath.h' },
    @{ Domain = 'hand'; Name = 'HandBoneGrabPivotMath.h' },
    @{ Domain = 'hand'; Name = 'HandVisualLerpMath.h' },
    @{ Domain = 'hand'; Name = 'SelectionQueryPolicy.h' },
    @{ Domain = 'hand'; Name = 'SelectionHighlightPolicy.h' },
    @{ Domain = 'hand'; Name = 'SelectionStatePolicy.h' },
    @{ Domain = 'hand'; Name = 'SelectedCloseFingerPolicy.h' },
    @{ Domain = 'hand'; Name = 'VatsSelectionHighlight.h' },
    @{ Domain = 'hand'; Name = 'HandspaceConvention.h' },
    @{ Domain = 'hand'; Name = 'PalmTransform.h' },
    @{ Domain = 'hand'; Name = 'PointingDirectionMath.h' },
    @{ Domain = 'hand'; Name = 'HandLifecyclePolicy.h' },
    @{ Domain = 'hand'; Name = 'HandState.h' },
    @{ Domain = 'hand'; Name = 'HandSemanticContactState.h' },
    @{ Domain = 'hand'; Name = 'HandCollisionSuppressionMath.h' }
)

foreach ($header in $consolidatedHeaders) {
    $legacyRootPath = "src/physics-interaction/$($header.Name)"
    $domainPath = "src/physics-interaction/$($header.Domain)/$($header.Name)"
    if (Test-Path -LiteralPath (Join-Path $Root $legacyRootPath)) {
        $failures.Add("$legacyRootPath`: consolidated policy/math headers must stay in grouped domain headers.")
    }
    if (Test-Path -LiteralPath (Join-Path $Root $domainPath)) {
        $failures.Add("$domainPath`: consolidated policy/math headers must stay in grouped domain headers.")
    }
}

$domainHeaderRules = @(
    @{
        Domain = 'weapon'
        Patterns = @('^Weapon.*Policy\.h$', '^Weapon.*Math\.h$', '^Weapon.*Types\.h$', '^Weapon.*Evidence\.h$', '^Weapon.*Limits\.h$', '^Weapon.*Router\.h$', '^Offhand.*\.h$')
        Allow = @(
            'WeaponTypes.h',
            'WeaponSemantics.h',
            'WeaponGeometry.h',
            'WeaponSupport.h',
            'WeaponAuthority.h',
            'WeaponDebug.h',
            'WeaponInteraction.h',
            'WeaponCollision.h',
            'TwoHandedGrip.h'
        )
    },
    @{
        Domain = 'grab'
        Patterns = @('^Grab.*Policy\.h$', '^Grab.*Math\.h$', '^Grab.*Runtime\.h$', '^Grab.*Telemetry.*\.h$', '^Held.*Policy\.h$', '^Held.*Math\.h$', '^Active.*Policy\.h$', '^Active.*Lifecycle\.h$', '^Canonical.*\.h$', '^Pull.*\.h$')
        Allow = @(
            'GrabCore.h',
            'GrabContact.h',
            'GrabFinger.h',
            'GrabHeldObject.h',
            'GrabTelemetry.h',
            'GrabConstraint.h',
            'GrabConstraintMath.h',
            'GrabMotionController.h',
            'GrabNodeInfoMath.h',
            'GrabNodeNamePolicy.h',
            'GrabHapticPolicy.h',
            'HeldPlayerSpaceRegistry.h',
            'MeshGrab.h',
            'NearbyGrabDamping.h'
        )
    },
    @{
        Domain = 'hand'
        Patterns = @('^Hand.*Policy\.h$', '^Hand.*Math\.h$', '^Hand.*Types\.h$', '^Hand.*State\.h$', '^Hand.*Runtime\.h$', '^Selection.*Policy\.h$', '^Selected.*Policy\.h$', '^Palm.*\.h$', '^Pointing.*\.h$', '^Direct.*\.h$', '^RootFlattened.*\.h$', '^Vats.*\.h$')
        Allow = @(
            'HandSkeleton.h',
            'HandColliderTypes.h',
            'HandVisual.h',
            'HandSelection.h',
            'HandFrame.h',
            'HandLifecycle.h',
            'Hand.h',
            'HandBoneColliderSet.h',
            'HandInteractionStateMachine.h'
        )
    }
)

foreach ($rule in $domainHeaderRules) {
    $domainRoot = Join-Path $Root "src/physics-interaction/$($rule.Domain)"
    if (-not (Test-Path -LiteralPath $domainRoot)) {
        continue
    }

    Get-ChildItem -LiteralPath $domainRoot -File -Filter '*.h' | ForEach-Object {
        $name = $_.Name
        if ($rule.Allow -notcontains $name) {
            foreach ($pattern in $rule.Patterns) {
                if ($name -match $pattern) {
                    $relative = Get-RelativePathCompat $_.FullName
                    $failures.Add("$relative`: one-off policy/math/type headers must be folded into the grouped $($rule.Domain) domain headers or explicitly allowlisted with a boundary rationale.")
                    break
                }
            }
        }
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'Physics interaction module boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Physics interaction module boundary passed.'
