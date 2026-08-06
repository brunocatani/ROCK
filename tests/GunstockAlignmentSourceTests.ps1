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
    'rockGunstockAlignBarrelToControllerForward\s*=\s*false' `
    'Gunstock correction must remain an explicit opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'GUNSTOCK_SECTION\s*=\s*"Gunstock"[\s\S]*rockGunstockAlignBarrelToControllerForward\s*=\s*false[\s\S]*GetBoolValue\([\s\S]*GUNSTOCK_SECTION,[\s\S]*"bAlignBarrelToControllerForward"' `
    'Gunstock configuration must have a false default and load from its dedicated INI section.'

Require-Text 'src/RockConfig.h' `
    'rockDebugDrawGunstockAlignment\s*=\s*false' `
    'The gunstock orientation visualizer must remain independently opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockDebugDrawGunstockAlignment\s*=\s*false[\s\S]*GetBoolValue\([\s\S]*"bDebugDrawGunstockAlignment"' `
    'The visualizer must load from the existing PhysicsInteraction section with a false default.'

foreach ($iniPath in @(
    'data/config/ROCK.ini',
    'data/mod/ROCK_Config/ROCK.ini'
)) {
    Require-Text $iniPath `
        '\[Gunstock\][\s\S]*bAlignBarrelToControllerForward\s*=\s*false' `
        'Both shipped INIs must expose the disabled-by-default gunstock option.'
    Require-Text $iniPath `
        '\[PhysicsInteraction\][\s\S]*bDebugDrawGunstockAlignment\s*=\s*false' `
        'Both shipped INIs must expose the independent disabled-by-default orientation visualizer.'
}

Require-Text 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h' `
    'kRequiredStableSamples\s*=\s*6[\s\S]*kStableSampleCosine[\s\S]*neutralHandLocal[\s\S]*tryCaptureHandLocalBore[\s\S]*localForward\s*\{\s*0\.0f,\s*1\.0f,\s*0\.0f\s*\}[\s\S]*tryBuildWorldCorrection[\s\S]*targetForwardWorld[\s\S]*tryBuildProjectedUpBisectorTwist[\s\S]*weaponSolverProjectOntoPlane[\s\S]*weaponSolverAxisAngleStored[\s\S]*rotateRigidlyAroundPivot[\s\S]*precompensateWorldTarget' `
    'The value-only policy must latch six stable projectile +Y samples, build the primary correction, constrain authored support roll to the bore axis, and retain recoil precompensation.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_twoHandedGrip\.update\([\s\S]*getEquippedProjectileNode\(\)[\s\S]*_twoHandedGrip\.applyGunstockAlignment\([\s\S]*reconcileEquippedWeaponHandAssignmentAfterGrip\(\)[\s\S]*_weaponCollision\.update' `
    'Gunstock alignment must run after the existing grip solve, use the verified projectile node, and precede downstream weapon collision consumption.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'prepareGunstockAlignmentDebugSnapshot\([\s\S]*applyGunstockAlignment\([\s\S]*updateBodiesFromCurrentSourceTransforms[\s\S]*applyFinalWeaponMuzzleAuthority\(\)[\s\S]*finalizeGunstockAlignmentDebugSnapshot\(' `
    'The visualizer must capture before alignment and publish final readback only after collision and muzzle authority.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'isRawButtonPhysicallyHeld\([\s\S]*kOpenVrSteamVrTriggerButtonId[\s\S]*applyGunstockAlignment\([\s\S]*gunstockNeutralSampleBlocked[\s\S]*frame\.reloadBoundaryActive\s*\|\|\s*frame\.menuBlocked' `
    'Neutral calibration must be blocked by physical trigger state while correction authority yields across animation/menu boundaries.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryResolveGunstockPrimaryGroupCorrection[\s\S]*localWristForward\s*\{\s*1\.0f,\s*0\.0f,\s*0\.0f\s*\}[\s\S]*tryBuildWorldCorrection<[\s\S]*SecondaryMeleeWeaponOffsetNode2[\s\S]*primaryWeaponOffsetNOde[\s\S]*outPivotWorld\s*=\s*dampedDriver->world\.translate[\s\S]*applyGunstockAlignment\([\s\S]*directionLatch\.latched[\s\S]*tryCaptureHandLocalBore\([\s\S]*tryResolveGunstockPrimaryGroupCorrection[\s\S]*rotateRigidlyAroundPivot[\s\S]*applyWeaponVisualAuthority' `
    'The runtime must derive its fixed correction from the damped firing-hand frame, orbit the complete group around the damped physical driver, and publish the weapon last.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState[\s\S]*activationWeaponWorld[\s\S]*tryResolveGunstockPrimaryGroupCorrection[\s\S]*resolveAuthoredSupportPalmSeatProximity\([\s\S]*activationWeaponWorld[\s\S]*reframeAuthoredSupportGripDebugSnapshot[\s\S]*authoredPalmSeatWeaponLocal[\s\S]*liveTouchProbeWorld' `
    'Authored support activation and cone diagnostics must use the gunstock-presented weapon frame while retaining the physical support-hand probe.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyGunstockAlignment[\s\S]*projectile witness unavailable[\s\S]*not a weapon-session identity change[\s\S]*keep publishing it[\s\S]*projectileWitnessUsable[\s\S]*!_gunstockAlignment\.directionLatch\.latched[\s\S]*publishAuthoredPrimaryFiringGripFingerPose' `
    'Transient firing-node loss must keep the latched correction active, and gunstock publication must retain the exact authored firing-hand finger pose.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockAlignmentState[\s\S]{0,500}canonicalCaptureSequence' `
    'Gunstock session identity must not churn when a fire animation publishes a new authored capture sequence.'

$gunstockYield = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.cpp'),
    '(?ms)TwoHandedGrip::currentGunstockAlignmentYieldReason\s*\(.*?^\s{4}bool\s+TwoHandedGrip::tryResolveGunstockPrimaryGroupCorrection')
if (-not $gunstockYield.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Could not isolate gunstock yield policy.')
} elseif ($gunstockYield.Value -match 'isWeaponVisualReturnActive|isHandVisualReturnActive') {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Gunstock must compose after bounded weapon/hand returns instead of yielding to an unaligned frame.')
}

$gunstockApply = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.cpp'),
    '(?ms)bool\s+TwoHandedGrip::applyGunstockAlignment\s*\(.*?^\s{4}bool\s+TwoHandedGrip::applyWeaponVisualAuthority')
if (-not $gunstockApply.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Could not isolate the gunstock alignment implementation.')
} elseif ($gunstockApply.Value -match 'SecondaryWandNode|primaryWandNode|firingController|controllerWorld') {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.cpp: Production gunstock correction must not mix raw controller transforms back into the damped firing-hand frame.')
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyGunstockAlignment[\s\S]*authoredSupportRollRequested[\s\S]*supportGrip\.authoredSupportGrip[\s\S]*primaryWeaponOffsetNOde[\s\S]*SecondaryMeleeWeaponOffsetNode2[\s\S]*_rightNaturalBoneInDampedDriver[\s\S]*_leftNaturalBoneInDampedDriver[\s\S]*correctedProjectileWorld[\s\S]*localUp\s*\{\s*0\.0f,\s*0\.0f,\s*1\.0f\s*\}[\s\S]*tryBuildProjectedUpBisectorTwist[\s\S]*correctedFiringHandWorld[\s\S]*correctedWeaponWorld[\s\S]*correctedSupportHandWorld' `
    'Authored support grips alone must add a two-damped-hand +Z bisector twist to the entire already-aim-aligned rigid group.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshNaturalHandInWandFrames[\s\S]*hasVisualAuthorityForHand\(isLeft\)[\s\S]*captureRelation[\s\S]*primaryWeaponOffsetNOde[\s\S]*_rightNaturalBoneInDampedDriver[\s\S]*SecondaryMeleeWeaponOffsetNode2[\s\S]*_leftNaturalBoneInDampedDriver' `
    'Natural hand-bone axes must be reconstructed from hFRIK damped drivers using relations captured only outside ROCK hand authority.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'supportGripPublishedThisFrame[\s\S]*HandAuthorityRole::[\s\S]*SupportGrip[\s\S]*_hasLastPublishedHandWorld\[supportIndex\][\s\S]*supportHandWorld\s*=\s*_lastPublishedHandWorld\[supportIndex\][\s\S]*else\s*\{[\s\S]*getHandWorldTransform' `
    'Gunstock correction must rotate the exact same-frame support-grip target, using rendered hand readback only when no support role was published.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'leftManualFiringRecoil[\s\S]*deriveAppliedWorldDelta[\s\S]*precompensateWorldTarget[\s\S]*PRIMARY_GRIP_TAG[\s\S]*SUPPORT_GRIP_TAG' `
    'Left manual firing must derive and precompensate hFRIK recoil while existing grip-role tags remain reusable.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'GUNSTOCK_ALIGNMENT_TAG\s*=\s*"ROCK_GunstockAlignment"[\s\S]*clearGunstockDedicatedHandAuthority[\s\S]*clearExternalHandWorldTransform' `
    'Native/tracked hand correction must use and explicitly clear a dedicated authority tag.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockAlignmentDebugSnapshot[\s\S]*std::uintptr_t\s+weaponNodeIdentity[\s\S]*std::uintptr_t\s+fireNodeIdentity[\s\S]*controllerWorld[\s\S]*leftHandWorld[\s\S]*weaponWorldBefore[\s\S]*finalFireNodeWorld[\s\S]*leftHandValid[\s\S]*published' `
    'The renderer bridge must be a value-only coherent snapshot with non-dereferenced identity witnesses.'

$gunstockSnapshot = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.h'),
    '(?ms)struct\s+GunstockAlignmentDebugSnapshot\s*\{(?<body>.*?)^\s{4}\};')
if (-not $gunstockSnapshot.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.h: Could not isolate the gunstock debug snapshot.')
} elseif ($gunstockSnapshot.Groups['body'].Value -match 'RE::Ni(?:Node|AVObject)\s*\*') {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.h: The gunstock debug snapshot must not retain engine pointers.')
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'populateGunstockAlignmentDebugPrediction[\s\S]*prepareGunstockAlignmentDebugSnapshot[\s\S]*AlignmentDisabled[\s\S]*finalizeGunstockAlignmentDebugSnapshot[\s\S]*finalLiveFireWorld' `
    'Behavior-off diagnostics must compute a read-only correction preview and final live readback.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'prepareGunstockAlignmentDebugSnapshot[\s\S]*getHandWorldTransform\([\s\S]*handFromBool\(_firingHandIsLeft\)[\s\S]*if\s*\(_firingHandIsLeft\)[\s\S]*snapshot\.leftHandWorld\s*=\s*firingHandWorld[\s\S]*if\s*\(!_firingHandIsLeft\)[\s\S]*getHandWorldTransform\([\s\S]*handFromBool\(true\)[\s\S]*snapshot\.leftHandWorld\s*=\s*leftHandWorld[\s\S]*snapshot\.leftHandValid\s*=\s*true' `
    'The gunstock snapshot must capture the left hand from the same hFRIK hand-bone authority as the firing-hand triad and reuse the firing sample when left-handed.'

Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' `
    'GunstockFiringController[\s\S]*GunstockLeftHand[\s\S]*GunstockFireNodeFinal[\s\S]*GunstockWristForward[\s\S]*GunstockCorrectionArc[\s\S]*GunstockCorrectionAxis' `
    'The existing bounded overlay must own explicit gunstock axis, ray, pivot, and correction-arc roles.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'drawGunstockAlignment[\s\S]*getGunstockAlignmentDebugSnapshot[\s\S]*kCorrectionArcSegments\s*=\s*16[\s\S]*leftHandValid[\s\S]*!snapshot\.firingHandIsLeft[\s\S]*GunstockLeftHand[\s\S]*FIRING WRIST \+X - GUNSTOCK FORWARD[\s\S]*ACTUAL FINAL LIVE FIRE \+Y[\s\S]*neutralResidual[\s\S]*liveDeviation[\s\S]*dataAge=0' `
    'The visualizer must render bounded tripods, comparison rays, a fixed correction arc, and current-frame status.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'leftControllerWorld|leftControllerValid' `
    'The gunstock snapshot must not retain the superseded left-controller diagnostic sample.'

Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' `
    'GunstockLeftController' `
    'The gunstock overlay must not retain the superseded left-controller triad role.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'leftControllerWorld|leftControllerValid|GunstockLeftController' `
    'The gunstock visualizer must draw the left hand-bone triad instead of the superseded left-controller triad.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'CONTROLLER \+Y - GUNSTOCK FORWARD|controllerAxis=\+Y' `
    'Gunstock diagnostics must not present controller +Y as the alignment target.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock neutral direction latches on sixth stable sample[\s\S]*gunstock correction sends neutral bore to firing wrist plus-X[\s\S]*gunstock authored support aligns weapon up to both-hand bisector[\s\S]*gunstock authored support roll preserves aligned barrel axis[\s\S]*gunstock firing hand orbits the damped physical driver[\s\S]*gunstock rigid correction preserves firing-hand weapon relation[\s\S]*gunstock precompensation survives noncommuting recoil delta[\s\S]*gunstock fixed neutral correction does not erase live recoil' `
    'Pure regression tests must lock calibration, authored support roll, barrel preservation, damped-driver grouping, recoil precompensation, and live-recoil preservation.'

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
