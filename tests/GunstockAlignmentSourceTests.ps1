param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source([string]$RelativePath) {
    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $failures.Add("Missing source file: $RelativePath")
        return ''
    }
    return Get-Content -LiteralPath $path -Raw
}

function Require-Text(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    $text = Read-Source $RelativePath
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    $text = Read-Source $RelativePath
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/RockConfig.h' `
    'rockGunstockModeEnabled\s*=\s*false' `
    'Gunstock mode must remain an explicit opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockGunstockModeEnabled\s*=\s*false[\s\S]*GetBoolValue\([\s\S]*GUNSTOCK_SECTION,[\s\S]*"bAlignBarrelToControllerForward",[\s\S]*rockGunstockModeEnabled' `
    'The historical production key must load both gunstock stages without changing the external INI contract.'

Require-Text 'src/RockConfig.h' `
    'rockDebugDrawGunstockAlignment\s*=\s*false' `
    'Gunstock diagnostics must remain independently opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockDebugDrawGunstockAlignment\s*=\s*false[\s\S]*"bDebugDrawGunstockAlignment",\s*rockDebugDrawGunstockAlignment' `
    'The historical diagnostic key must load the combined alignment visualizer.'

foreach ($iniPath in @(
    'data/config/ROCK.ini',
    'data/mod/ROCK_Config/ROCK.ini'
)) {
    Require-Text $iniPath `
        '\[Gunstock\][\s\S]*two independent stages[\s\S]*neutral fire-node[\s\S]*firing-wrist \+X[\s\S]*bAlignBarrelToControllerForward\s*=\s*false' `
        'Both shipped INIs must describe and expose the disabled-by-default support baseline and final wrist alignment.'
    Require-Text $iniPath `
        '\[PhysicsInteraction\][\s\S]*firing/support bone triads[\s\S]*support attach-baseline[\s\S]*bDebugDrawGunstockAlignment\s*=\s*false' `
        'Both shipped INIs must describe and expose the combined disabled-by-default visualizer.'
}

Require-Text 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h' `
    'kRequiredStableSamples\s*=\s*6[\s\S]*ModeToggleState[\s\S]*observeModeToggle[\s\S]*WeaponEligibilityState[\s\S]*observeWeaponEligibility[\s\S]*gunTypeWitnessObserved[\s\S]*validFireNodeObserved[\s\S]*gunTypeWitnessObserved\s*&&\s*validFireNodeObserved[\s\S]*isWeaponEligible[\s\S]*neutralHandLocal[\s\S]*tryCaptureHandLocalBore[\s\S]*localForward\s*\{\s*0\.0f,\s*1\.0f,\s*0\.0f\s*\}[\s\S]*tryBuildWorldCorrection[\s\S]*targetForwardWorld[\s\S]*rotateRigidlyAroundPivot[\s\S]*precompensateWorldTarget' `
    'The value-only policy must own live mode edges, generation-bound kGun-plus-fire-node eligibility, neutral +Y capture, wrist correction, rigid grouping, and recoil precompensation.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'tryCaptureGunstockSupportBaseline\([\s\S]*invertTransform\(supportInputWorld\)[\s\S]*supportGripTargetWorld[\s\S]*tryResolveGunstockSupportTarget\([\s\S]*supportInputWorld,[\s\S]*inputToGripTargetLocal' `
    'The support policy must freeze input-to-grip relation and resolve only later support motion through it.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'getEquippedProjectileNode\(\)[\s\S]*getCurrentObservedEquippedWeaponFormID\(\)\s*==[\s\S]*observedEquippedWeapon->formID[\s\S]*weaponData\.type\s*==[\s\S]*WEAPON_TYPE::kGun[\s\S]*_twoHandedGrip\.update\([\s\S]*gunstockProjectileNode,[\s\S]*gunstockGunTypeObserved[\s\S]*prepareGunstockAlignmentDebugSnapshot\([\s\S]*applyGunstockAlignment\([\s\S]*reconcileEquippedWeaponHandAssignmentAfterGrip\(\)[\s\S]*updateBodiesFromCurrentSourceTransforms[\s\S]*applyFinalWeaponMuzzleAuthority\(\)[\s\S]*finalizeGunstockAlignmentDebugSnapshot\(' `
    'The shared kGun and fire-node observations must feed support solving only after the collision form boundary matches; final alignment remains ahead of collision and final muzzle/debug synchronization.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'gunstockPresentationBlocked\s*=\s*frame\.menuBlocked[\s\S]*gunstockNeutralSampleBlocked\s*=[\s\S]*isRawButtonPhysicallyHeld\([\s\S]*kOpenVrSteamVrTriggerButtonId\)[\s\S]*\|\|\s*frame\.reloadBoundaryActive[\s\S]*prepareGunstockAlignmentDebugSnapshot\([\s\S]*gunstockNeutralSampleBlocked,\s*frame\.menuBlocked\)[\s\S]*applyGunstockAlignment\([\s\S]*gunstockNeutralSampleBlocked,\s*frame\.menuBlocked\)' `
    'Trigger and arms/hands animation authority must block only neutral calibration while the final latched presentation yields only at hard menu authority.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'gunstock(?:Alignment|Presentation)Blocked\s*=\s*frame\.reloadBoundaryActive|gunstockNeutralSampleBlocked,\s*frame\.reloadBoundaryActive' `
    'Arms/hands animation authority must never suppress an already-latched gunstock presentation.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryResolveGunstockPhysicalFiringFrame[\s\S]*SecondaryMeleeWeaponOffsetNode2[\s\S]*primaryWeaponOffsetNOde[\s\S]*_leftNaturalBoneInDampedDriver[\s\S]*_rightNaturalBoneInDampedDriver[\s\S]*localWristForward\s*\{\s*1\.0f,\s*0\.0f,\s*0\.0f\s*\}[\s\S]*applyGunstockAlignment\([\s\S]*tryCaptureHandLocalBore\([\s\S]*tryResolveGunstockPrimaryGroupCorrection[\s\S]*rotateRigidlyAroundPivot[\s\S]*applyWeaponVisualAuthority' `
    'The runtime must reconstruct a clean damped wrist, target wrist +X, rotate the complete posed group, and publish the weapon coherently.'

Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'tryGetGunstockTrackedFiringHandWorld\([\s\S]*input\.weaponNode,[\s\S]*input\.weaponGenerationKey,[\s\S]*trackedHandWorld\s*=\s*gunstockTrackedHandWorld' `
    'Authored primary alignment must consume the clean damped gunstock frame only for the matching eligible weapon generation.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState[\s\S]*activationWeaponWorld[\s\S]*tryResolveGunstockPrimaryGroupCorrection[\s\S]*resolveAuthoredSupportPalmSeatProximity\([\s\S]*activationWeaponWorld[\s\S]*reframeAuthoredSupportGripDebugSnapshot' `
    'Authored support seats and cones must evaluate in the final gunstock-presented frame.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyGunstockAlignment[\s\S]*projectileWitnessUsable[\s\S]*!directionLatch\.latched[\s\S]*projectile witness unavailable during firing animation; retaining and publishing the latched correction[\s\S]*publishAuthoredPrimaryFiringGripFingerPose' `
    'Transient fire-node loss must retain the latched correction and exact authored firing-hand finger pose.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'firingGripPublishedThisFrame[\s\S]*_lastPublishedHandWorld\[firingIndex\][\s\S]*authoredRightCanonicalCurrent[\s\S]*_rightFiringHandCanonicalWeaponLocal[\s\S]*firingGroupHandWorld[\s\S]*captureRenderedFiringRelation[\s\S]*getHandWorldTransform[\s\S]*renderedFiringRelationError[\s\S]*rotateRigidlyAroundPivot[\s\S]*pivotWorld[\s\S]*captureRenderedFiringRelation\(\)' `
    'Final alignment must rotate the exact posed firing-hand target with the weapon and diagnose the rendered, not algebraically requested, relation.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'supportGripPublishedThisFrame[\s\S]*HandAuthorityRole::[\s\S]*SupportGrip[\s\S]*_hasLastPublishedHandWorld\[supportIndex\][\s\S]*supportHandWorld\s*=\s*_lastPublishedHandWorld\[supportIndex\][\s\S]*else\s*\{[\s\S]*getHandWorldTransform' `
    'Final alignment must rotate the exact same-frame support target, using rendered readback only when no support role was published.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'leftManualFiringRecoil[\s\S]*out\.reusedGripRole[\s\S]*recoilRequestedWorld\s*=[\s\S]*_lastPublishedHandWorld\[index\][\s\S]*recoilAppliedWorld\s*=[\s\S]*getHandWorldTransform\([\s\S]*deriveAppliedWorldDelta[\s\S]*precompensateWorldTarget[\s\S]*PRIMARY_GRIP_TAG[\s\S]*SUPPORT_GRIP_TAG' `
    'Left manual firing must derive recoil from synchronous rendered-hand readback even when the existing primary role tag is reused.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'GUNSTOCK_ALIGNMENT_TAG\s*=[\s\S]*"ROCK_GunstockAlignment"[\s\S]*clearGunstockDedicatedHandAuthority[\s\S]*clearExternalHandWorldTransform' `
    'Native/tracked hand correction must use and explicitly clear a dedicated authority tag.'

$gunstockYield = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.cpp'),
    '(?ms)TwoHandedGrip::currentGunstockAlignmentYieldReason\s*\(.*?^\s{4}bool\s+TwoHandedGrip::tryResolveGunstockPhysicalFiringFrame')
if (-not $gunstockYield.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Could not isolate gunstock yield policy.')
} else {
    if ($gunstockYield.Value -match 'isWeaponVisualReturnActive|isHandVisualReturnActive') {
        $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Final alignment must compose after weapon and hand returns instead of exposing an unaligned frame.')
    }
    if ($gunstockYield.Value -match '_scopeMenuOpenThisFrame|_scopeMenuClosedThisFrame|ScopeTransition') {
        $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Scope presentation must retain an already-latched gunstock correction.')
    }
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'neutralSampleBlocked\s*=\s*calibrationSampleBlocked\s*\|\|\s*_scopeMenuOpenThisFrame\s*\|\|\s*_scopeMenuClosedThisFrame[\s\S]*if \(!directionLatch\.latched\)[\s\S]*if \(neutralSampleBlocked\)[\s\S]*gunstock retaining latched presentation while neutral sampling is blocked[\s\S]*tryResolveGunstockPrimaryGroupCorrection' `
    'Scope/firing instability must stop only pre-latch sampling and the latched correction must continue through that boundary.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    '(?m)^\s*ScopeTransition\s*,' `
    'The debug contract must not retain a stale scope-yield state after scope continuity is enforced.'

$gunstockApply = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.cpp'),
    '(?ms)bool\s+TwoHandedGrip::applyGunstockAlignment\s*\(.*?^\s{4}bool\s+TwoHandedGrip::applyWeaponVisualAuthority')
if (-not $gunstockApply.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Could not isolate the gunstock alignment implementation.')
} elseif ($gunstockApply.Value -match 'SecondaryWandNode|primaryWandNode|firingController|controllerWorld') {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Production correction must not mix raw controller transforms into the damped wrist frame.')
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryGetSolverHandTransform\(isLeft, handTransform\)[\s\S]*if \(outCapturedHandWorld\)[\s\S]*\*outCapturedHandWorld\s*=\s*handTransform' `
    'Grip capture must return the exact damped support sample used by acquisition.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'initializeGunstockSupportRole\([\s\S]*rockGunstockModeEnabled[\s\S]*FullTwoHandedSolver[\s\S]*isGunstockWeaponGenerationEligible\([\s\S]*tryCaptureGunstockSupportBaseline\([\s\S]*supportInputWorld,[\s\S]*supportGripTargetWorld,[\s\S]*weaponWorldAtCapture\s*=\s*weaponNode->world[\s\S]*weaponGenerationKey\s*=\s*supportGrip\.weaponGenerationKey[\s\S]*gripSequence\s*=\s*supportGrip\.gripSequence[\s\S]*firstPublicationPending\s*=\s*true' `
    'The centralized support-role initializer must require shared firearm eligibility and bind the damped input, weapon transform, generation, and grip sequence atomically.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'transitionToGripping\([\s\S]*initializeGunstockSupportRole\([\s\S]*supportCaptureHandWorld,[\s\S]*"support-attach"[\s\S]*updatePartCarryGrip\([\s\S]*_state\s*=\s*TwoHandedState::Gripping[\s\S]*initializeGunstockSupportRole\([\s\S]*"part-carry-firing-grip-reattach"[\s\S]*updateFullWeaponAuthorityGrip' `
    'Normal support attach and PartCarry role swap must enter the tandem solver through the same gunstock support-role initializer.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'reconcileGunstockModeState\([\s\S]*observeModeToggle\([\s\S]*ModeToggleEdge::Disabled[\s\S]*resetGunstockAlignment\("config-disabled-edge"\)[\s\S]*clearGunstockSupportBaselines\(\)[\s\S]*beginDynamicSupportAcquisition\([\s\S]*ModeToggleEdge::Enabled[\s\S]*clearDynamicSupportAcquisition\([\s\S]*initializeGunstockSupportRole\(' `
    'Live INI edges must clear both stages atomically, rebase ordinary acquisition on disable, and capture an existing eligible support role on enable.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'observeGunstockWeaponEligibility\([\s\S]*observeWeaponEligibility\([\s\S]*isGunstockSupportBaselineActive\([\s\S]*isGunstockWeaponGenerationEligible\([\s\S]*tryGetGunstockTrackedFiringHandWorld\([\s\S]*isGunstockWeaponEligible\([\s\S]*applyGunstockAlignment\([\s\S]*weapon-not-eligible' `
    'Support calibration, authored firing input, and final alignment must consume the same generation-bound kGun-plus-native-fire-node eligibility witness.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    '!gunstockBaselineActive[\s\S]*shouldUseDynamicSupportAcquisition[\s\S]*_rotationBlend\s*=\s*gunstockBaselineActive\s*\?\s*1\.0f\s*:\s*0\.0f[\s\S]*if \(gunstockBaselineActive\)[\s\S]*updateFullWeaponAuthorityGrip\(weaponNode, 0\.0f\)' `
    'Gunstock attach must bypass absolute acquisition blending and publish one immediate baseline transaction.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'calibratedSupportTransform\s*=\s*supportTransform[\s\S]*tryResolveGunstockSupportTarget\([\s\S]*inputToGripTargetLocal[\s\S]*computeGrabLegacyPalmPivotAWorldFromHandBasis\([\s\S]*calibratedSupportTransform[\s\S]*supportNormalTargetWorld\s*=\s*computePalmNormalFromHandBasis\([\s\S]*calibratedSupportTransform' `
    'The tandem solver must receive position and twist from the calibrated support target.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'gunstockAttachPublication[\s\S]*appliedWeaponWorld\s*=[\s\S]*weaponWorldAtCapture[\s\S]*applyWeaponVisualAuthority\(weaponNode, appliedWeaponWorld\)[\s\S]*applyLockedHandVisualAuthority\([\s\S]*firstPublicationPending\s*=\s*false' `
    'The first support publication must retain the captured primary weapon transform before final group alignment.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockAlignmentDebugSnapshot[\s\S]*weaponNodeIdentity[\s\S]*fireNodeIdentity[\s\S]*dampedDriverWorld[\s\S]*firingHandWorld[\s\S]*renderedFiringHandWorld[\s\S]*weaponWorldBefore[\s\S]*finalFireNodeWorld[\s\S]*renderedFiringRelationPositionErrorGameUnits[\s\S]*weaponEligible[\s\S]*renderedFiringRelationValid[\s\S]*published' `
    'The renderer bridge must retain separate damped-input and post-publication firing-hand witnesses with explicit relation validity.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'controllerWorld|controllerValid|ControllerUnavailable' `
    'The alignment snapshot must describe the damped driver and bone frame without retaining misleading controller fields.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockSupportBaselineDebugSnapshot[\s\S]*supportInputWorld[\s\S]*calibratedSupportWorld[\s\S]*weaponWorldBefore[\s\S]*weaponWorldAfter[\s\S]*attachRotationDegrees[\s\S]*published' `
    'The support-baseline renderer bridge must expose coherent attach and tandem values.'

foreach ($snapshotName in @(
    'GunstockAlignmentDebugSnapshot',
    'GunstockSupportBaselineDebugSnapshot'
)) {
    $snapshot = [regex]::Match(
        (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.h'),
        "(?ms)struct\s+$snapshotName\s*\{(?<body>.*?)^\s{4}\};")
    if (-not $snapshot.Success) {
        $failures.Add("src/physics-interaction/weapon/TwoHandedGrip.h: Could not isolate $snapshotName.")
    } elseif ($snapshot.Groups['body'].Value -match 'RE::Ni(?:Node|AVObject)\s*\*') {
        $failures.Add("src/physics-interaction/weapon/TwoHandedGrip.h: $snapshotName must not retain engine pointers.")
    }
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'populateGunstockAlignmentDebugPrediction[\s\S]*prepareGunstockAlignmentDebugSnapshot[\s\S]*AlignmentDisabled[\s\S]*finalizeGunstockAlignmentDebugSnapshot[\s\S]*finalLiveFireWorld' `
    'Behavior-off diagnostics must compute a read-only alignment preview and final live readback.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'prepareGunstockAlignmentDebugSnapshot[\s\S]*tryResolveGunstockPhysicalFiringFrame\([\s\S]*snapshot\.dampedDriverWorld\s*=\s*firingDriverWorld[\s\S]*snapshot\.firingHandWorld\s*=\s*firingHandWorld[\s\S]*snapshot\.pivotWorld\s*=\s*firingDriverWorld\.translate' `
    'The diagnostic snapshot must use the same damped driver and reconstructed wrist bone as production alignment.'

$gunstockDebugPrepare = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.cpp'),
    '(?ms)void\s+TwoHandedGrip::prepareGunstockAlignmentDebugSnapshot\s*\(.*?^\s{4}void\s+TwoHandedGrip::finalizeGunstockAlignmentDebugSnapshot')
if (-not $gunstockDebugPrepare.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Could not isolate gunstock debug preparation.')
} elseif ($gunstockDebugPrepare.Value -match 'SecondaryWandNode|primaryWandNode|controllerWorld|controllerValid') {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Debug preparation must not depend on raw controller/wand frames.')
}

Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' `
    'GunstockLeftHand[\s\S]*GunstockFiringHand[\s\S]*GunstockRenderedFiringHand[\s\S]*GunstockSupportInputBone[\s\S]*GunstockFireNodeFinal[\s\S]*GunstockWeaponAfter[\s\S]*GunstockWristForward[\s\S]*GunstockCorrectionArc[\s\S]*GunstockSupportTargetPoint' `
    'The bounded overlay must distinguish physical and rendered firing-hand bones while retaining final-alignment and support-baseline roles.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'drawGunstockAlignment[\s\S]*getGunstockAlignmentDebugSnapshot[\s\S]*GunstockLeftHand[\s\S]*GunstockFiringHand[\s\S]*GunstockRenderedFiringHand[\s\S]*RENDERED FIRING HAND BONE[\s\S]*FIRING WRIST \+X - GUNSTOCK TARGET[\s\S]*ACTUAL FINAL LIVE FIRE \+Y[\s\S]*GunstockCorrectionArc[\s\S]*renderedRelation=[\s\S]*getGunstockSupportBaselineDebugSnapshot[\s\S]*GunstockSupportInputBone[\s\S]*GunstockCalibratedSupportBone[\s\S]*GunstockWeaponAfter[\s\S]*attachWeaponDelta' `
    'The visualizer must render input/rendered firing bones, actual relation error, final wrist alignment, and support attach baseline together.'

Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' `
    'GunstockFiringController|GunstockLeftController' `
    'Gunstock diagnostics must not retain controller-triad roles.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'GunstockFiringController|GunstockLeftController|CONTROLLER \+Y - GUNSTOCK FORWARD|controllerAxis=\+Y' `
    'Gunstock diagnostics must use bone triads and wrist +X, not controller-axis tripods.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock live enable produces one edge[\s\S]*gunstock fire node alone does not classify thrown weapons as firearms[\s\S]*gunstock gun type plus valid fire node establishes generation eligibility[\s\S]*gunstock eligibility survives transient fire-node loss[\s\S]*gunstock eligibility never crosses weapon generations[\s\S]*gunstock neutral direction latches on sixth stable sample[\s\S]*gunstock correction sends neutral bore to firing wrist plus-X[\s\S]*gunstock rigid correction preserves firing-hand weapon relation[\s\S]*gunstock precompensation survives noncommuting recoil delta[\s\S]*unchanged support input reproduces exact authored target[\s\S]*calibrated attach target leaves weapon unchanged[\s\S]*post-attach support delta drives existing tandem solver' `
    'Pure regressions must cover mode edges, eligibility lifetime, final alignment, recoil continuity, support attach identity, and tandem delta.'

Reject-Text 'src/api/ROCKProviderApi.h' `
    'Gunstock' `
    'Gunstock mode is internal presentation policy and must not alter ROCK API V1.'

if ($failures.Count -gt 0) {
    Write-Host 'GunstockAlignmentSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GunstockAlignmentSourceTests passed.' -ForegroundColor Green
