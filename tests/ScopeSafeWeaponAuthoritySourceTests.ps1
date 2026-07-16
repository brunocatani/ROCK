param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/core/RockRuntimeState.cpp' 'localScopeMenuOpen\s*=\s*s_menuHandlerInitialized\s*&&\s*s_gameMenus\.isInScopeMenu\(\)' `
    'Runtime state must sample FO4VR ScopeMenu explicitly instead of treating it as a generic blocking menu.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'leftHandedMode\s*!=\s*isLeft[\s\S]*SecondaryMeleeWeaponOffsetNode2\s*:\s*playerNodes->primaryWeaponOffsetNOde[\s\S]*scopeMenuOpen\s*=\s*runtime\.localScopeMenuOpen[\s\S]*leftHandDriverFrame\s*=\s*leftHandDriverFrame[\s\S]*rightHandDriverFrame\s*=\s*rightHandDriverFrame' `
    'Two-hand authority must receive hFRIK-damped physical left/right arm-driver frames together with explicit ScopeMenu state.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct NativeScopeSightAnchorSnapshot[\s\S]*weaponGenerationKey[\s\S]*anchorWeaponLocal[\s\S]*sightBodyCount[\s\S]*getNativeScopeSightAnchorSnapshot' `
    'Generated weapon evidence must publish a generation-keyed native-scope sight anchor snapshot.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'descriptor\.semantic\.partKind\s*!=\s*WeaponPartKind::Sight[\s\S]*rearPlaneCenterFromSightBounds[\s\S]*_nativeScopeSightAnchorSnapshot\s*=\s*nativeScopeSightAnchorSnapshot' `
    'Native-scope placement must aggregate only validated Sight geometry and publish its rear-center anchor with the weapon generation.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' '_hasSolvedWeaponTransform\s*=\s*false;[\s\S]*refreshNativeScopeSightAnchor\(weaponNode,\s*currentWeaponGenerationKey,\s*weaponCollision\);[\s\S]*refreshScopeSafeHandFrames\(weaponNode,\s*frameInput,\s*dt\);[\s\S]*if\s*\(!runtime_state::isLocalSkeletonReady\(\)\s*\|\|\s*!weaponNode\)' `
    'Scope geometry and hFRIK-driver calibration must refresh before the grip state machine and its early return.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'snapshot\.weaponGenerationKey\s*!=\s*currentWeaponGenerationKey[\s\S]*_nativeScopeSightAnchorGenerationKey\s*=\s*0[\s\S]*_nativeScopeSightAnchorValid\s*=\s*true' `
    'Native-scope geometry must fail closed across publication races and become usable only after an exact generation match.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'scopeAnchorMatchesAuthority[\s\S]*captureNativeScopeRigidFrame[\s\S]*rigidFrameMatchesAuthority[\s\S]*resolveRigidSightFrameWorld' `
    'Native-scope camera authority must resolve one generation-bound rigid sight frame for every weapon authority mode.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'followWeaponWorldChangeFromSightAnchor[\s\S]*followWeaponWorldChange\(' `
    'Native-scope camera authority must preserve the calibrated rigid-delta fallback when sight geometry is unavailable.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'rootHandValid\s*=\s*!_scopeDriverFrameAuthorityActive\s*&&[\s\S]*tryGetRootFlattenedHandBoneTransform[\s\S]*captureDriverToHandLocal\(driverFrame\.world,\s*resolvedHandWorld\)' `
    'Collapsed ScopeMenu root frames must never overwrite hFRIK-driver-to-hand calibration.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'ResolutionMode::DriverReconstructed[\s\S]*currentHandWorld\s*=\s*reconstructedHandWorld' `
    'Scoped hand authority must reconstruct the authored hand frame from hFRIK damped arm drivers.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'SCOPE_DRIVER_MISS_GRACE_FRAMES[\s\S]*consecutiveDriverMissFrames[\s\S]*ResolutionMode::LastKnown[\s\S]*\+\+state\.consecutiveDriverMissFrames' `
    'The last valid scoped hand frame fallback must be explicitly bounded across driver loss.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'retainDriverFrameAuthority\([\s\S]*isManualOwnershipActive\(\)[\s\S]*driverFrameAuthorityWasActive[\s\S]*driverFrameAuthorityStoppedThisFrame' `
    'A manual weapon-grip session must retain its driver-relative solver basis across ScopeMenu close/reopen pulses.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'rootHandValid\s*=\s*!_scopeDriverFrameAuthorityActive[\s\S]*resolveMode\([\s\S]*_scopeDriverFrameAuthorityActive' `
    'The latched scope-driver authority, not presentation visibility, must select the weapon-solver hand basis.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'driverFrameAuthorityStoppedThisFrame\s*&&\s*\(reconstructedHandValid\s*\|\|\s*recentScopedHandAvailable\)[\s\S]*continuityHandWorld[\s\S]*rootRebaseLocalStart[\s\S]*interpolateRebaseTransform' `
    'Scope exit must rebase smoothly from reconstructed or recent scoped authority to the restored hFRIK root hand.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'recentScopedHandAvailable\s*=\s*state\.hasLastHandWorld\s*&&[\s\S]*state\.consecutiveDriverMissFrames\s*<\s*SCOPE_DRIVER_MISS_GRACE_FRAMES;[\s\S]*state\.consecutiveDriverMissFrames\s*=\s*0;' `
    'Scope-exit history age must be checked before the normal root path resets its driver-miss counter.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' '_scopeHandAuthorityCleanupPending\s*=\s*true[\s\S]*!_scopeMenuOpenThisFrame[\s\S]*clearExternalHandWorldTransform\(PRIMARY_GRIP_TAG[\s\S]*clearExternalHandWorldTransform\(SUPPORT_GRIP_TAG' `
    'Persistent hFRIK wrist-authority entries must be cleared once the visible root returns after ScopeMenu.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'refreshHand\(true,\s*frameInput\.leftHandDriverFrame\);[\s\S]*refreshHand\(false,\s*frameInput\.rightHandDriverFrame\);[\s\S]*clearExternalHandWorldTransform\(PRIMARY_GRIP_TAG' `
    'Scope-exit cleanup must not mutate hFRIK root hands before ROCK captures its fully adjusted rebase frames.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'captureScopeHandAuthorityCleanupVisuals\(weaponNode\)[\s\S]*clearExternalHandWorldTransform\(PRIMARY_GRIP_TAG[\s\S]*restoreScopeHandAuthorityCleanupVisuals\(visualSnapshot\)' `
    'Scope-exit tag cleanup must preserve the adjusted weapon and native scope-camera world baselines across hFRIK arm restoration.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::clearSupportGripPose[\s\S]*if \(_scopeMenuOpenThisFrame\)[\s\S]*_scopeHandAuthorityCleanupPending\s*=\s*true;[\s\S]*clearExternalHandWorldTransform\(SUPPORT_GRIP_TAG' `
    'Support-hand release while scoped must defer hFRIK wrist cleanup instead of restoring against the collapsed root.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::clearPrimaryGripPose[\s\S]*if \(_scopeMenuOpenThisFrame\)[\s\S]*_scopeHandAuthorityCleanupPending\s*=\s*true;[\s\S]*clearExternalHandWorldTransform\(PRIMARY_GRIP_TAG' `
    'Primary-hand release while scoped must defer hFRIK wrist cleanup instead of restoring against the collapsed root.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::reset\(\)[\s\S]*clearSupportGripPose\(false\);[\s\S]*_scopeMenuOpenThisFrame\s*=\s*false;' `
    'Reset must preserve the active ScopeMenu flag until all hand-authority clears have been safely deferred.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'applyWeaponVisualAuthority\(weaponNode,\s*solved\.weaponWorldTransform\)[\s\S]*applyLockedHandVisualAuthority\(weaponNode,\s*applyPrimaryHandAuthority' `
    'Full two-hand solve must publish weapon and native scope-camera authority before considering hidden hand IK.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'bool TwoHandedGrip::applyLockedHandVisualAuthority\([\s\S]*shouldPublishLockedHandVisualAuthority\(_scopeMenuOpenThisFrame\)[\s\S]*return true;[\s\S]*frik_visual_authority::isAvailable\(\)' `
    'Hidden ScopeMenu hand IK must be a successful no-op so it cannot revoke weapon ownership.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'shouldReuseRightFiringCanonicalGrip[\s\S]*_rightFiringGripCanonicalWeaponLocal[\s\S]*pre-scope-canonical' `
    'A support grip acquired while scoped must reuse the generation-matched visible firing-hand hold instead of hidden driver geometry.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'canRefreshRightFiringCanonicalFrame[\s\S]*rootRebaseActive[\s\S]*_rightFiringGripCanonicalWeaponLocal\s*=\s*canonicalGrip' `
    'ScopeMenu and scope-exit rebase frames must not poison the right-hand canonical grip.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'static bool tryGetHandBoneTransform' `
    'The old finite-only root-hand reader must not remain as the two-hand solver authority path.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Scope-safe weapon authority source boundaries passed.'
