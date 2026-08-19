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
    'rockGunstockAlignmentPitchDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentYawDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentRollDegrees\s*=\s*0\.0f' `
    'Gunstock fine tuning must remain zero-default so existing alignment is unchanged.'

Require-Text 'src/RockConfig.cpp' `
    'rockGunstockAlignmentPitchDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentYawDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentRollDegrees\s*=\s*0\.0f[\s\S]*"fAlignmentPitchDegrees"[\s\S]*-180\.0f,[\s\S]*180\.0f[\s\S]*"fAlignmentYawDegrees"[\s\S]*-180\.0f,[\s\S]*180\.0f[\s\S]*"fAlignmentRollDegrees"[\s\S]*-180\.0f,[\s\S]*180\.0f' `
    'All three gunstock fine-tune axes must load as finite clamped degree values.'

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
        '\[Gunstock\][\s\S]*same damped-driver pivot[\s\S]*yaw about bone \+Z[\s\S]*pitch about bone \+Y[\s\S]*roll about aligned \+X[\s\S]*fAlignmentPitchDegrees\s*=\s*0\.0[\s\S]*fAlignmentYawDegrees\s*=\s*0\.0[\s\S]*fAlignmentRollDegrees\s*=\s*0\.0' `
        'Both shipped INIs must document and expose zero-default wrist-space fine tuning.'
    Require-Text $iniPath `
        '\[PhysicsInteraction\][\s\S]*firing/support bone triads[\s\S]*support attach-baseline[\s\S]*bDebugDrawGunstockAlignment\s*=\s*false' `
        'Both shipped INIs must describe and expose the combined disabled-by-default visualizer.'
}

Require-Text 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h' `
    'kRequiredStableSamples\s*=\s*6[\s\S]*ModeToggleState[\s\S]*observeModeToggle[\s\S]*WeaponEligibilityState[\s\S]*observeWeaponEligibility[\s\S]*gunTypeWitnessObserved[\s\S]*validFireNodeObserved[\s\S]*gunTypeWitnessObserved\s*&&\s*validFireNodeObserved[\s\S]*isWeaponEligible[\s\S]*neutralHandLocal[\s\S]*tryCaptureHandLocalBore[\s\S]*localForward\s*\{\s*0\.0f,\s*1\.0f,\s*0\.0f\s*\}[\s\S]*tryBuildWorldCorrection[\s\S]*targetForwardWorld[\s\S]*rotateRigidlyAroundPivot[\s\S]*deriveAppliedWorldDelta[\s\S]*precompensateWorldTarget' `
    'The value-only policy must own mode edges, firearm eligibility, neutral +Y capture, wrist correction, rigid grouping, and controlled-recoil precompensation.'

Reject-Text 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h' `
    'deriveAppliedLocalDelta|precompensateLocalTarget' `
    'Gunstock alignment must not infer an unobservable same-frame palm residual from root-tree readback.'

Require-Text 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h' `
    'struct\s+FineTuneDegrees[\s\S]*pitchDegrees[\s\S]*yawDegrees[\s\S]*rollDegrees[\s\S]*hasFineTune[\s\S]*tryBuildWorldFineTuneRotation[\s\S]*yawAxisWorld[\s\S]*fineTune\.yawDegrees[\s\S]*pitchAxisWorld[\s\S]*fineTune\.pitchDegrees[\s\S]*rollAxisWorld[\s\S]*fineTune\.rollDegrees[\s\S]*tryBuildFineTunedWorldCorrection[\s\S]*automaticCorrection[\s\S]*fineTuneWorld[\s\S]*weaponSolverApplyWorldRotationToStoredBasis' `
    'The pure policy must compose yaw, pitch, then roll in the untrimmed wrist frame after automatic alignment.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'tryCaptureSupportInputBaseline\([\s\S]*invertTransform\(supportInputWorld\)[\s\S]*supportGripTargetWorld[\s\S]*tryResolveSupportInputTarget\([\s\S]*supportInputWorld,[\s\S]*inputToGripTargetLocal' `
    'The support policy must freeze input-to-grip relation and resolve only later support motion through it.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'getEquippedProjectileNode\(\)[\s\S]*getCurrentObservedEquippedWeaponFormID\(\)\s*==[\s\S]*observedEquippedWeapon->formID[\s\S]*weaponData\.type\s*==[\s\S]*WEAPON_TYPE::kGun[\s\S]*_twoHandedGrip\.update\([\s\S]*gunstockProjectileNode,[\s\S]*gunstockGunTypeObserved[\s\S]*prepareGunstockAlignmentDebugSnapshot\([\s\S]*applyGunstockAlignment\([\s\S]*reconcileEquippedWeaponHandAssignmentAfterGrip\(\)[\s\S]*updateBodiesFromCurrentSourceTransforms[\s\S]*applyFinalWeaponMuzzleAuthority\(\)[\s\S]*finalizeGunstockAlignmentDebugSnapshot\(' `
    'The shared kGun and fire-node observations must feed support solving only after the collision form boundary matches; final alignment remains ahead of collision and final muzzle/debug synchronization.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'gunstockPresentationBlocked\s*=[\r\n\s]*frame\.menuBlocked\s*\|\|\s*frame\.reloadBoundaryActive[\s\S]*gunstockNeutralSampleBlocked\s*=[\s\S]*isRawButtonPhysicallyHeld\([\s\S]*kOpenVrSteamVrTriggerButtonId\)[\s\S]*\|\|\s*frame\.reloadBoundaryActive[\s\S]*const bool gunstockPresentationBlocked\s*=[\r\n\s]*frame\.menuBlocked\s*\|\|\s*frame\.reloadBoundaryActive[\s\S]*prepareGunstockAlignmentDebugSnapshot\([\s\S]*gunstockNeutralSampleBlocked,\s*gunstockPresentationBlocked\)[\s\S]*applyGunstockAlignment\([\s\S]*gunstockNeutralSampleBlocked,\s*gunstockPresentationBlocked\)' `
    'Trigger input must block neutral calibration, while menu or native reload authority must yield the complete latched gunstock hand presentation.'

Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' `
    'alignOpticalAxesToBore|camera \+X[\s\S]*bore \+Y' `
    'Gunstock scopes must not replace the captured native camera calibration with an unverified axis mapping.'

Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'tryGetGunstockTrackedFiringHandWorld\([\s\S]*input\.weaponNode,[\s\S]*input\.weaponGenerationKey,[\s\S]*trackedHandWorld\s*=\s*gunstockTrackedHandWorld' `
    'Authored primary alignment must consume the clean damped gunstock frame only for the matching eligible weapon generation.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockFramePresentationState[\s\S]*runtimeFrameIndex[\s\S]*correctionWorld[\s\S]*pivotWorld[\s\S]*sourceWeaponWorld[\s\S]*correctedWeaponWorld[\s\S]*finalizedAfterNativeAnimation[\s\S]*valid' `
    'The normal gunstock pass must retain a bounded same-frame correction transaction for a later native Weapon writer.'

Require-Text 'src/ROCKMain.cpp' `
    'dispatchAnimationPhaseCallbacksV1\([\s\S]*AfterRock[\s\S]*finalizeGunstockPresentationAfterNativeAnimation\(\)[\s\S]*dispatchAnimationPhaseCallbacksV1\([\s\S]*Complete' `
    'The full-reload correction must run after animation AfterRock publication and before the Complete phase.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'finalizeGunstockPresentationAfterNativeAnimation[\s\S]*currentNativeAnimationAuthorityFlagsV1\(\)[\s\S]*kWeapon[\s\S]*resolveEquippedWeaponInteractionNode\(\)[\s\S]*finalizeGunstockPresentationAfterNativeWeaponAnimation' `
    'The post-animation bridge must fail closed unless the current animation lease authors Weapon.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    '(?m)^\s*ScopeTransition\s*,' `
    'The debug contract must not retain a stale scope-yield state after scope continuity is enforced.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockAlignmentDebugSnapshot[\s\S]*weaponNodeIdentity[\s\S]*fireNodeIdentity[\s\S]*dampedDriverWorld[\s\S]*firingHandWorld[\s\S]*renderedFiringHandWorld[\s\S]*weaponWorldBefore[\s\S]*finalFireNodeWorld[\s\S]*fineTunedTargetForwardWorld[\s\S]*fineTunePitchDegrees[\s\S]*fineTuneYawDegrees[\s\S]*fineTuneRollDegrees[\s\S]*renderedFiringRelationPositionErrorGameUnits[\s\S]*weaponEligible[\s\S]*renderedFiringRelationValid[\s\S]*published' `
    'The renderer bridge must retain fine-tuned intent plus separate damped-input and post-publication firing-hand witnesses with explicit relation validity.'

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

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'drawGunstockAlignment[\s\S]*getGunstockAlignmentDebugSnapshot[\s\S]*GunstockLeftHand[\s\S]*GunstockFiringHand[\s\S]*GunstockRenderedFiringHand[\s\S]*RENDERED FIRING HAND BONE[\s\S]*FIRING WRIST \+X - AUTOMATIC TARGET[\s\S]*PREDICTED FINE-TUNED NEUTRAL[\s\S]*ACTUAL FINAL LIVE FIRE \+Y[\s\S]*GunstockCorrectionArc[\s\S]*fineTune pitch\(\+Y\)[\s\S]*liveResidual=[\s\S]*renderedRelation=[\s\S]*getGunstockSupportBaselineDebugSnapshot[\s\S]*GunstockSupportInputBone[\s\S]*GunstockCalibratedSupportBone[\s\S]*GunstockWeaponAfter[\s\S]*attachWeaponDelta' `
    'The visualizer must render input/rendered firing bones, configured fine-tuned intent, actual relation error, and support attach baseline together.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock zero fine tune preserves automatic correction exactly[\s\S]*gunstock positive yaw sends plus-X toward plus-Y[\s\S]*gunstock positive pitch sends plus-X toward minus-Z[\s\S]*gunstock pure roll fine tune remains active with aligned bore[\s\S]*gunstock combined correction reaches fine-tuned target[\s\S]*gunstock fine tune keeps the damped-driver reference pivot fixed[\s\S]*gunstock fine tune preserves the firing-grip weapon relation[\s\S]*gunstock fine-tuned weapon preserves retained rigid scope frame' `
    'Pure regressions must cover zero compatibility, axis signs, pure roll, target composition, fixed pivot, firing-grip rigidity, and rigid fine-tuned scope transport.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'GunstockFiringController|GunstockLeftController|CONTROLLER \+Y - GUNSTOCK FORWARD|controllerAxis=\+Y' `
    'Gunstock diagnostics must use bone triads and wrist +X, not controller-axis tripods.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock live enable produces one edge[\s\S]*gunstock fire node alone does not classify thrown weapons as firearms[\s\S]*gunstock gun type plus valid fire node establishes generation eligibility[\s\S]*gunstock eligibility survives transient fire-node loss[\s\S]*gunstock eligibility never crosses weapon generations[\s\S]*gunstock neutral direction latches on sixth stable sample[\s\S]*gunstock correction sends neutral bore to firing wrist plus-X[\s\S]*gunstock rigid correction preserves firing-hand weapon relation[\s\S]*gunstock precompensation survives noncommuting recoil delta[\s\S]*gunstock post-animation correction preserves authored left-hand relation[\s\S]*gunstock post-animation correction preserves authored right-hand relation[\s\S]*unchanged support input reproduces exact authored target[\s\S]*calibrated attach target leaves weapon unchanged[\s\S]*post-attach support delta drives existing tandem solver' `
    'Pure regressions must cover mode edges, eligibility lifetime, final alignment, recoil continuity, full-reload rigid hand relations, support attach identity, and tandem delta.'

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
