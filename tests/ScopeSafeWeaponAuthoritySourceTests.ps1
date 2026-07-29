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
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'scopeHandDriverNode\s*=\s*\[playerNodes\]\(bool isLeft\)[\s\S]*return isLeft\s*\?[\s\S]*SecondaryMeleeWeaponOffsetNode2\s*:[\s\S]*primaryWeaponOffsetNOde[\s\S]*scopeMenuOpen\s*=\s*runtime\.localScopeMenuOpen[\s\S]*leftHandDriverFrame\s*=\s*leftHandDriverFrame[\s\S]*rightHandDriverFrame\s*=\s*rightHandDriverFrame' `
    'Two-hand authority must receive fixed physical-left secondary and physical-right primary hFRIK driver frames together with explicit ScopeMenu state.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'isLeftHandedMode' `
    'Scope authority must not reinterpret ROCK controller identity through Fallout 4 VR native handedness.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct NativeScopeSightAnchorSnapshot[\s\S]*weaponGenerationKey[\s\S]*equippedWeaponOwnershipKey[\s\S]*weaponFormID[\s\S]*anchorWeaponLocal[\s\S]*sightBodyCount[\s\S]*getNativeScopeSightAnchorSnapshot' `
    'Generated weapon evidence must publish a generation-, ownership-, and form-keyed native-scope sight anchor snapshot.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'accumulatePartKind\(WeaponPartKind::Scope\)[\s\S]*accumulatePartKind\(WeaponPartKind::Sight\)[\s\S]*rearPlaneCenterFromSightBounds[\s\S]*_nativeScopeSightAnchorSnapshot\s*=\s*nativeScopeSightAnchorSnapshot' `
    'Native-scope placement must prefer validated native-overlay Scope geometry, retain Sight only as an evidence fallback, and publish its rear-center anchor with the weapon generation.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' '_hasSolvedWeaponTransform\s*=\s*false;[\s\S]*refreshNativeScopeAnchor\([\s\S]*currentWeaponGenerationKey,[\s\S]*currentEquippedWeaponOwnershipKey,[\s\S]*getCurrentObservedEquippedWeaponFormID\(\)[\s\S]*refreshScopeSafeHandFrames\(frameInput,\s*dt\);[\s\S]*if\s*\(!runtime_state::isLocalSkeletonReady\(\)\s*\|\|\s*!weaponNode\)' `
    'Scope anchor and hFRIK-driver calibration must refresh before the grip state machine and its early return.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'snapshot\.weaponGenerationKey\s*!=[\s\S]*matchesCurrentEquippedWeapon\([\s\S]*publishedIdentity,[\s\S]*currentIdentity[\s\S]*_nativeScopeAnchorWeaponNode\s*=\s*nullptr[\s\S]*native_scope_sight_anchor_policy::resolve[\s\S]*_nativeScopeAnchorValid\s*=\s*true' `
    'Native-scope geometry must reject publication races before selecting either generated geometry or the current firing-grip fallback.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'tryGetManualScopeDirectTransitionTarget[\s\S]*matchesCurrentEquippedWeapon\(publishedIdentity,\s*currentIdentity\)' `
    'Manual direct scope transitions must reject a previous weapon body-set publication during equip replacement.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'struct NativeScopeResolvedAnchorSnapshot[\s\S]*weaponGenerationKey[\s\S]*equippedWeaponOwnershipKey[\s\S]*weaponFormID[\s\S]*anchorWeaponLocal' `
    'The selected native-scope anchor must retain generation, ownership, and form identity across internal and provider readbacks.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'scopeAnchorMatchesAuthority[\s\S]*captureNativeScopeRigidFrame[\s\S]*rigidFrameMatchesAuthority[\s\S]*resolveRigidAnchorFrameWorld' `
    'Native-scope camera authority must resolve one generation-bound rigid selected-anchor frame for every weapon authority mode.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'rockNativeScopeFiringGripFallbackOffsetXGameUnits[\s\S]*rockNativeScopeFiringGripFallbackPitchDegrees[\s\S]*hasRightFiringHandCanonicalFrame[\s\S]*_primaryGripLocal[\s\S]*FiringGripFallback[\s\S]*leaving native camera untouched' `
    'Missing or forced-bad optic geometry must use the current firing-grip frame and fail closed when no such origin exists.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'rebuildNativeScopeRigidFrameTarget[\s\S]*nativeCameraWeaponLocal[\s\S]*targetCameraWeaponLocal\.translate\s*=\s*_nativeScopeAnchorWeaponLocal[\s\S]*FiringGripFallback[\s\S]*applyWeaponLocalRotationOffset[\s\S]*cameraWeaponLocal\s*=' `
    'Fallback position and rotation tuning must be rebuilt from immutable native calibration without orbiting or compounding the anchor.'
Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'followWeaponWorldChange\s*\(' `
    'Native-scope camera authority must not retain the controller-relative rigid-delta fallback.'
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
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'enum class HandAuthorityRole[\s\S]*DesiredHandAuthorityInput[\s\S]*desiredRolesForHand[\s\S]*DeferredClearAction[\s\S]*resolveDeferredClearAction' `
    'Scope-exit cleanup must derive role-aware live authority and wait for replacement publication before clearing stale tags.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'updateHandVisualReturns\(dt\);[\s\S]{0,500}reconcileDeferredScopeHandAuthority\(weaponNode\);' `
    'Deferred scope authority must reconcile only after the current state has published its replacement hand roles.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::reconcileDeferredScopeHandAuthority[\s\S]*captureScopeHandAuthorityCleanupVisuals\(weaponNode\)[\s\S]*desiredRolesForHand[\s\S]*resolveDeferredClearAction[\s\S]*restoreScopeHandAuthorityCleanupVisuals\(visualSnapshot\)' `
    'Role-aware scope cleanup must preserve weapon and scope-camera baselines if clearing a stale tag restores an hFRIK arm.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'scopeStateChanged\s*&&\s*!_scopeMenuOpenThisFrame[\s\S]*void TwoHandedGrip::clearSupportGripPose[\s\S]*if \(_scopeMenuOpenThisFrame\s*\|\|\s*_scopeMenuClosedThisFrame\)[\s\S]*deferScopeHandAuthorityClear\(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,\s*isLeft\)' `
    'Support-hand release while scoped must defer hFRIK wrist cleanup instead of restoring against the collapsed root.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::clearPrimaryGripPose[\s\S]*if \(_scopeMenuOpenThisFrame\s*\|\|\s*_scopeMenuClosedThisFrame\)[\s\S]*deferScopeHandAuthorityClear\(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,\s*isLeft\)' `
    'Primary-hand release while scoped must defer hFRIK wrist cleanup instead of restoring against the collapsed root.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::reset\(\)[\s\S]*clearSupportGripPose\(false\);[\s\S]*_scopeMenuOpenThisFrame\s*=\s*false;' `
    'Reset must preserve the active ScopeMenu flag until all hand-authority clears have been safely deferred.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'applyWeaponVisualAuthority\(weaponNode,\s*appliedWeaponWorld\)[\s\S]*applyLockedHandVisualAuthority\(weaponNode,\s*applyPrimaryHandAuthority' `
    'Full two-hand solve must publish its final immediate-or-acquired weapon and native scope-camera authority before considering hidden hand IK.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'bool TwoHandedGrip::applyLockedHandVisualAuthority\([\s\S]*shouldPublishLockedHandVisualAuthority\(_scopeMenuOpenThisFrame\)[\s\S]*return true;[\s\S]*frik_visual_authority::isAvailable\(\)' `
    'Hidden ScopeMenu hand IK must be a successful no-op so it cannot revoke weapon ownership.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'shouldReuseRightFiringCanonicalGrip[\s\S]*_rightFiringGripCanonicalWeaponLocal[\s\S]*pre-scope-canonical' `
    'A support grip acquired while scoped must reuse the generation-matched visible firing-hand hold instead of hidden driver geometry.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'canRefreshRightFiringCanonicalFrame[\s\S]*rootRebaseActive[\s\S]*_rightFiringGripCanonicalWeaponLocal\s*=\s*canonicalGrip' `
    'ScopeMenu and scope-exit rebase frames must not poison the right-hand canonical grip.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'static bool tryGetHandBoneTransform' `
    'The old finite-only root-hand reader must not remain as the two-hand solver authority path.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' '_scopeHandAuthorityCleanupPending|for\s*\(const bool isLeft\s*:\s*\{\s*true,\s*false\s*\}\)[\s\S]{0,500}PRIMARY_GRIP_TAG[\s\S]{0,500}SUPPORT_GRIP_TAG[\s\S]{0,500}PRIMARY_DETACH_TAG' `
    'Scope exit must not blanket-clear every hand-authority role for both hands.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Scope-safe weapon authority source boundaries passed.'
