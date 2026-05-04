param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-SourceFiles {
    param([string]$RelativeRoot)

    Get-ChildItem -LiteralPath (Join-Path $Root $RelativeRoot) -Recurse -File |
        Where-Object { $_.Extension -in @('.h', '.cpp', '.hpp', '.inl') }
}

$nonNativeFiles = Read-SourceFiles 'src/physics-interaction' |
    Where-Object { $_.FullName -notlike (Join-Path $Root 'src/physics-interaction/native/*') }

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
    }
)

foreach ($file in $nonNativeFiles) {
    $text = Get-Content -Raw -LiteralPath $file.FullName
    foreach ($rule in $forbiddenOutsideNative) {
        if ($text -match $rule.Pattern) {
            $relative = [System.IO.Path]::GetRelativePath($Root, $file.FullName)
            $failures.Add("$relative`: $($rule.Message)")
        }
    }
}

$requiredNativeText = @{
    'src/physics-interaction/native/HavokRuntime.h' = @(
        'getHknpWorldFromBhk',
        'getQueryFilterRefWithFallback',
        'setBodyTransformDeferred',
        'enableBodyFlags',
        'disableBodyFlags'
    )
    'src/physics-interaction/core/PhysicsInteraction.cpp' = @(
        'havok_runtime::getHknpWorldFromBhk',
        'havok_runtime::getQueryFilterRefWithFallback'
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
    @{ Domain = 'grab'; Name = 'GrabSurfaceFrameMath.h' },
    @{ Domain = 'grab'; Name = 'GrabOppositionFrameMath.h' },
    @{ Domain = 'grab'; Name = 'GrabMultiFingerContactMath.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerPoseMath.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerPoseRuntime.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerLocalTransformMath.h' },
    @{ Domain = 'grab'; Name = 'GrabFingerLocalTransformRuntime.h' },
    @{ Domain = 'grab'; Name = 'HeldObjectBodySetPolicy.h' },
    @{ Domain = 'grab'; Name = 'HeldObjectDampingMath.h' },
    @{ Domain = 'grab'; Name = 'HeldObjectPhysicsMath.h' },
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
    @{ Domain = 'hand'; Name = 'HandVisualAuthorityMath.h' },
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
            'GrabVisualAuthorityPolicy.h',
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
                    $relative = [System.IO.Path]::GetRelativePath($Root, $_.FullName)
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
