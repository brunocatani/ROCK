param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

$removedRuntimeFiles = @(
    'src/physics-interaction/animation/NativeAnimationAuthority.cpp',
    'src/physics-interaction/animation/NativeAnimationAuthority.h',
    'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h'
)
foreach ($relativePath in $removedRuntimeFiles) {
    if (Test-Path -LiteralPath (Join-Path $Root $relativePath)) {
        $failures.Add("ROCK still owns removed reload/bolt runtime file '$relativePath'.")
    }
}

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_UpdateFirstPersonArm\s*=\s*0xEF6280[\s\S]*kCallsite_UpdateFirstPersonArmPrimaryReturn\s*=\s*0xEF610D[\s\S]*kCallsite_UpdateFirstPersonArmSecondaryReturn\s*=\s*0xEF6150[\s\S]*kFunc_PlayerPostUpdateAnimationGraphManager\s*=\s*0xF2F0A0' 'Authored grip capture must retain the independently verified graph-output and paired arm offsets.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'kExpectedUpdateFirstPersonArmPrefix[\s\S]*0x48,\s*0x8B,\s*0xC4,\s*0x55,\s*0x53,\s*0x41,\s*0x56,[\s\S]*0x48,\s*0x8D,\s*0xA8,\s*0xF8,\s*0xFE,\s*0xFF,\s*0xFF[\s\S]*kFunc_UpdateFirstPersonArm' 'The ROCK-owned arm hook must retain byte validation.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'kExpectedPostFrikPrefix[\s\S]*kFunc_PlayerPostUpdateAnimationGraphManager[\s\S]*onPostUpdateAnimationGraphManager' 'The shared graph-output coordinator must retain byte validation against hFRIK''s verified patch identity.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'onPostUpdateAnimationGraphManager[\s\S]*NativeGraphOutput[\s\S]*captureAuthoredSupportGraphPose\(\)[\s\S]*s_originalPostUpdate' 'Addon capture and ROCK grip capture must share the proven pre-presentation graph-output boundary.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 's_originalUpdateFirstPersonArm[\s\S]*captureNativePrimaryFiringGrip' 'The later primary arm pass must pair the graph sample with the exact weapon-specific primary relation.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'currentNativeAnimationAuthorityFlagsV1\(\)\s*!=\s*0' 'Authored grip capture must yield to ROCK V1 animation authority owners.'
Reject-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'WeaponFireHandler|ReloadStateChangeHandler|setLocalReload|ManualCycle|applyCapturedPose' 'ROCK-authored grip capture must not retain reload or bolt runtime ownership.'

Require-Text 'src/ROCKMain.cpp' 'dispatchAnimationPhaseCallbacksV1\([\s\S]*BeforeRock[\s\S]*onFrameUpdate\(\)[\s\S]*AfterRock[\s\S]*Complete' 'ROCK must expose ordered addon animation phases around its update.'
Reject-Text 'src/ROCKMain.cpp' 'setLocalReloadTestEnabled|setLocalManualCycleTestEnabled|applyCapturedPose|installPostUpdateHook' 'ROCK main must not execute the separated reload/bolt runtime.'
Require-Text 'src/ROCKMain.cpp' 'authored_weapon_grip_capture::installHook\(\)' 'ROCK must retain its authored equipped-weapon grip capture hook.'
Require-Text 'src/ROCKMain.cpp' 'authored_weapon_grip_capture::setEnabled\(\s*authoredGripCaptureRuntimeEnabled\s*\)' 'ROCK must unconditionally run authored equipped-weapon grip capture while the runtime is eligible.'
Require-Text 'src/api/ROCKProviderApi.h' 'enum class RockProviderAnimationPhaseV1[\s\S]*NativeGraphOutput' 'ROCK V1 must expose the proven native graph-output capture phase.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'phaseFrameIndex\s*=\s*s_activeAnimationPhaseFrameIndex[\s\S]*if \(phaseFrameIndex == 0\)' 'NativeGraphOutput and the later ROCK phases must share one frame identity.'
Reject-Text 'src/api/ROCKProviderApi.cpp' 'phase == RockProviderAnimationPhaseV1::BeforeRock\s*\|\|' 'BeforeRock must not replace the frame identity opened by NativeGraphOutput.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'phase == RockProviderAnimationPhaseV1::Complete[\s\S]*s_activeAnimationPhaseFrameIndex\.store\(0' 'The shared animation phase frame must close deterministically at Complete.'

Require-Text 'src/api/ROCKProviderApi.h' 'AnimationPhases[\s\S]*EquippedWeaponGripState[\s\S]*HandVisualAuthority[\s\S]*NativeAnimationRuntimeProvider' 'ROCK V1 must publish the complete Reanimate support capability surface.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiSetNativeAnimationAuthorityV1[\s\S]*apiIsProviderReady\(\)[\s\S]*NativeAnimationAuthoritySlot' 'ROCK must coordinate animation leases without requiring an in-process animation hook.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiPublishNativeAnimationRuntimeV1[\s\S]*s_nativeAnimationRuntimePublication' 'ROCK must accept status publication from the runtime addon.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' 'queryProviderEquippedWeaponGripStateV1[\s\S]*RightHandInWeaponValid[\s\S]*LeftHandInWeaponValid' 'ROCK must expose exact equipped-weapon grip baselines to addons.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'leftHandHoldingObject\s*=\s*_leftHand\.isHolding\(\)[\s\S]*rightHandHoldingObject\s*=\s*_rightHand\.isHolding\(\)[\s\S]*setGrabbedObjectHandPoseOwnership\(\s*leftHandHoldingObject,\s*rightHandHoldingObject\s*\)[\s\S]*_authoredPrimaryFiringGrip\.update' 'The authored-grip phase must publish current per-physical-hand ROCK grab ownership before an equipped firing pose can be selected.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'TwoHandedGripUpdateResult TwoHandedGrip::update\([\s\S]*setGrabbedObjectHandPoseOwnership\(\s*frameInput\.leftHandHoldingObject,\s*frameInput\.rightHandHoldingObject\s*\)[\s\S]*switch \(_state\)' 'The equipped-weapon state machine must refresh per-hand ROCK grab ownership before any state can publish a firing pose.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'publishAuthoredPrimaryFiringGripFingerPose[\s\S]*targetHandHoldingObject[\s\S]*shouldPublishAuthoredFiringFingerPose\(\s*targetHandHoldingObject\s*\)' 'Every authored firing-finger publication must yield to an ordinary ROCK grab on the target physical hand.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'setGrabbedObjectHandPoseOwnership[\s\S]*publishedHandHoldingObject[\s\S]*clearAuthoredPrimaryFiringGripFingerPose\(\)' 'Acquiring a ROCK object must immediately clear an already-published equipped firing pose without ending equipped-weapon transform ownership.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h' 'AuthoredSupportGripCandidateInput[\s\S]*interactionAcquisitionValid[\s\S]*activationZoneValid[\s\S]*authoredPoseSurfaceEvidenceValid[\s\S]*shouldUseAuthoredSupportGrip\([\s\S]*input\.activationZoneValid[\s\S]*input\.authoredPoseSurfaceEvidenceValid' 'Authored support selection must require the enforced activation zone and distributed current-weapon pose evidence.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'findCurrentWeaponSurfaceNearPoints\([\s\S]*poseEvidencePass\s*=\s*\(snapshot\.poseSurfaceWitnessMask\s*&\s*0x01u\)\s*!=\s*0\s*&&\s*snapshot\.poseSurfaceWitnessCount\s*>=\s*3' 'The authored palm and at least two distal fingertips must witness current weapon geometry.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'authoredInteractionAcquisitionValid\s*=\s*decision\.acquisitionSource\s*==[\s\S]*PhysicalContact\s*\|\|[\s\S]*ProximityProbe[\s\S]*shouldUseAuthoredSupportGrip\([\s\S]*if \(useAuthoredSupportGrip\)[\s\S]*return true;[\s\S]*tryGetSupportGripEvidenceView\(decision\.bodyId, weaponNode, evidenceView\)' 'Inside the gate authored must win over physical/probe acquisition, while every rejection must fall through to the existing dynamic mesh/finger solver.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'if \(useAuthoredSupportGrip\)[\s\S]*?grip\.attachmentRoot\s*=\s*weaponNode;[\s\S]*?grip\.hasSourceFrames\s*=\s*false;' 'Authored support grips must retain weapon-root-local physical authority.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryRebindPartGripToCurrentGeneration\([\s\S]*grip\.attachmentRoot\s*=\s*grip\.authoredSupportGrip\s*\?[\s\S]*_activeWeaponNode\s*:[\s\S]*bestSourceNode' 'Generation rebinding must preserve authored weapon-root authority while dynamic grips follow their current part source.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'handVisualSource|resolveCurrentHandVisualSourceRoot|captureAuthoredSupportHandVisualAnchor|authored pump visual anchor' 'ROCK must not retain the superseded authored-pump dual-anchor path.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' 'if \(input\.nativeReloadAuthorityActive\)[\s\S]{0,350}clearStableAuthoredSupportGripSnapshot\(\)[\s\S]{0,200}return;' 'Native reload authority must invalidate the stable authored support snapshot before the reload animation can replace its pose.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' 'if \(!publishLiveAuthoredSupportCandidate\(resolvedCaptureSequence\)\)[\s\S]{0,180}publishStableAuthoredSupportCandidate\([\s\S]{0,80}resolvedCaptureSequence' 'Right-handed firing must retain the identity-bound authored support candidate across transient live support-pass gaps.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h' 'shouldReuseStableAuthoredSupportGrip[\s\S]*currentWeaponOwnershipKey[\s\S]*snapshotWeaponOwnershipKey[\s\S]*currentWeaponGenerationKey[\s\S]*snapshotWeaponGenerationKey[\s\S]*currentPrimaryGripCaptureSequence[\s\S]*snapshotPrimaryGripCaptureSequence[\s\S]*snapshotSupportGripCaptureSequence[\s\S]*kCompleteAuthoredSupportFingerLocalTransformMask' 'Stable authored support reuse must remain guarded by complete weapon, generation, canonical, support-capture, and finger-pose identity.'

Reject-Text 'src/RockConfig.h' 'rockNativeReloadAnimationAuthorityTestEnabled|rockNativeReloadAnimationPartialAuthorityTestEnabled' 'Reload validation configuration must not remain in ROCK.'
Reject-Text 'src/RockConfig.h' 'rockAuthoredPrimaryFiringGripTestEnabled' 'The production authored primary/equipped-grip path must not retain an experimental config switch.'
Reject-Text 'data/config/ROCK_example.ini' 'bAuthoredPrimaryFiringGripTestEnabled' 'Users must not be able to disable the production authored primary/equipped-grip path.'
Reject-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' 'input\.enabled|experiment-disabled' 'The production authored primary grip runtime must not retain its removed experimental gate.'

foreach ($configPath in @('src/RockConfig.h', 'src/RockConfig.cpp', 'data/config/ROCK_example.ini')) {
    Reject-Text $configPath 'ExperimentalAuthoredGripPositionOnlyAlignment' 'Mandatory authored position-only alignment must not remain configurable.'
    Reject-Text $configPath 'LeftFiring.*Smooth|Smooth.*LeftFiring|LeftFiring.*Dampen|Dampen.*LeftFiring' 'Left firing must reuse hFRIK''s actual damped driver output instead of introducing a second ROCK smoothing configuration.'
}
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'positionOnlyAlignmentRequested|rockExperimentalAuthoredGripPositionOnlyAlignment' 'The frame coordinator must not carry selectable authored-alignment mode state.'
Reject-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.h' 'positionOnlyAlignmentRequested|_positionOnlyAlignmentActive' 'The authored runtime must not retain selectable position-only mode state.'
Reject-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' '_positionOnlyAlignmentActive|presented-hand-unavailable|resolveAuthoredPrimaryWeaponWorld\(|tryResolvePrimaryFiringGripAlignment' 'Steady authored alignment must not retain the removed full-rigid branch or presented-hand fallback.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' 'tryGetAuthoredPrimaryTrackedFiringHandWorld[\s\S]*resolveAuthoredPrimaryWeaponWorldPositionOnly[\s\S]*applyAuthoredPrimaryGripWeaponAlignment\([\s\S]{0,300}solvedFiringHandWorld' 'Steady authored alignment must always consume the physical driver, preserve native weapon rotation, and publish the authored hand target.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' '_rightFiringCanonicalPositionOnlyAlignment|bool positionOnlyAlignment|NiTransform\* solvedFiringHandWorld' 'The authored canonical and publisher interfaces must not expose a selectable or nullable full-rigid path.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' '_rightFiringCanonicalPositionOnlyAlignment|resolveAuthoredPrimaryWeaponWorld\(|if\s*\(\s*!solvedFiringHandWorld' 'Collision intent, canonical reuse, and return transitions must not retain the removed full-rigid path.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryResolveAuthoredPrimaryWeaponReturnTargetLocal[\s\S]*tryGetAuthoredPrimaryTrackedFiringHandWorld[\s\S]*resolveAuthoredPrimaryWeaponWorldPositionOnly' 'Authored weapon return must preserve the same physical-driver position-only contract as steady carry.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'captureRightNativeWeaponAimFrame[\s\S]{0,1800}invertTransform\(rightWand->world\)[\s\S]{0,300}weaponNode->world[\s\S]{0,1200}_rightNativeWeaponAimFrame' 'Left firing must capture actual native weapon-in-right-wand orientation without deriving aim from the authored wrist.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'canCaptureRightNativeWeaponAimFrame[\s\S]{0,1600}_returningWeaponVisual\.localTransition\.active[\s\S]{0,500}_weaponCollisionHandPresentationFromPreviousFrame\[1\][\s\S]{0,1000}TwoHandedState::PrimaryOnly[\s\S]{0,700}supportGripOwnsWeaponTransform' 'Native aim capture must run only while the right-side weapon orientation is still native, never during visual return, collision presentation, left carry, or full solver ownership.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'hasRightNativeWeaponAimFrame[\s\S]{0,1200}weaponNodeIdentity\s*==\s*weaponNode[\s\S]{0,300}weaponGenerationKey\s*==[\s\S]{0,300}weaponOwnershipKey\s*==' 'The transient native aim baseline must be rejected across weapon-node, collision-generation, or equipped-owner boundaries.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryResolveLeftPositionOnlyCarryFrames[\s\S]{0,5000}mirrorRightWeaponInWandOrientation[\s\S]{0,2500}resolveWeaponWorldPositionOnly[\s\S]{0,1200}outPresentedHandWorld\s*=\s*transform_math::composeTransforms' 'Left carry must separate mirrored native weapon aim from the authored left-hand presentation target.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'captureLeftFiringDampedFollowFrame[\s\S]{0,1800}invertTransform\(leftWandWorld\)[\s\S]{0,300}physicalLeftHandWorld[\s\S]{0,1200}handInWandOrientation' 'ROCK must freeze a left hand-in-raw-wand reference from the uncontaminated physical hand when damped follow begins.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'hasLeftFiringDampedFollowFrame[\s\S]{0,1200}weaponNodeIdentity\s*==\s*weaponNode[\s\S]{0,300}weaponGenerationKey\s*==[\s\S]{0,300}weaponOwnershipKey\s*==' 'The frozen damped-follow reference must reject unrelated weapon nodes, collision generations, and equipped owners.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryResolveLeftPositionOnlyCarryFrames[\s\S]{0,5200}resolveDampedAimCarrierWorld[\s\S]{0,3000}resolveWeaponWorldPositionOnly\([\s\S]{0,300}dampedAimCarrierWorld[\s\S]{0,1200}outPresentedHandWorld\s*=\s*transform_math::composeTransforms' 'Left one-hand carry must apply hFRIK''s observed damping delta to the weapon carrier before seating position and deriving the authored hand.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'if\s*\(firingHandIsLeft\)[\s\S]{0,1200}currentCanonicalResolved[\s\S]{0,700}!retainUntilPhysicalGrip[\s\S]{0,900}capturedFiringHandWeaponLocal' 'Equipped normalization may reuse only a committed equipped transfer, never a loose-model relation that already contains legacy aim trim.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'solveLeftFiringWeaponCarry[\s\S]{0,2500}applyExternalHandWorldTransform[\s\S]{0,1800}applyWeaponVisualAuthority\(weaponNode, solvedWeaponWorld\)' 'Left one-hand carry must publish the authored wrist before the final position-only weapon frame.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'left position-only normalization[\s\S]{0,500}rightNativeAxes=\(\+X:[\s\S]{0,300}leftAppliedAxes=\(-X:[\s\S]{0,500}gripError=[\s\S]{0,300}authoredHandCorrection=' 'The one-shot takeover trace must expose the full bilateral weapon basis, grip seat error, and hand-only authored correction.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'left position-only normalization[\s\S]{0,1800}dampedFollow=\{:\.2f\}deg' 'The takeover trace must expose the hFRIK damping rotation reused by the shared weapon/hand carrier.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'solveLeftFiringWeaponCarry[\s\S]{0,1200}composeTransforms\(firingHandTransform,\s*transform_math::invertTransform\(_primaryHandWeaponLocal\)\)' 'Left carry must never invert the authored wrist relation directly into weapon rotation again.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'applyLockedHandVisualAuthority\(weaponNode,[\s\S]{0,900}_firingHandIsLeft\s*&&\s*applyPrimaryHandAuthority[\s\S]{0,500}applyWeaponVisualAuthority' 'Dynamic and authored full-support left carry must restore the normalized weapon after publishing its authored firing wrist.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'support-released-primary-held[\s\S]{0,400}_firingHandIsLeft[\s\S]{0,200}solveLeftFiringWeaponCarry' 'Support release must publish the normalized left hand/weapon pair in the transition frame.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'part-carry-reattached-firing-grip[\s\S]{0,400}_firingHandIsLeft[\s\S]{0,200}solveLeftFiringWeaponCarry' 'Firing-grip reattach must publish the normalized left hand/weapon pair in the transition frame.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryPromoteSupportGripToFiringGrip[\s\S]{0,6500}_primaryHandWeaponLocal\s*=\s*newFiringHandWeaponLocal[\s\S]{0,500}solveLeftFiringWeaponCarry\(weaponNode\)' 'Support-hand promotion must publish the normalized left hand/weapon pair in the transition frame.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'positionOnlyHandWorldActive[\s\S]{0,1200}tryResolvePhysicalHandFrame\([\s\S]{0,100}isLeft' 'Both right and left position-only hand presentations must keep solver input on the physical damped-driver wrist.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'grip\.authoredSupportGrip\s*=\s*true;[\s\S]{0,900}grip\.disableAuthoredSupportNormalTwist\s*=\s*[\s\r\n]*_rightFiringHandCanonicalSource\s*==[\s\r\n]*RightFiringCanonicalSource::AuthoredAnimation' 'Both authored support topologies must inherit palm-normal-twist suppression from the authored firing canonical.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'grip\.disableAuthoredSupportNormalTwist\s*=\s*[\s\r\n]*!_firingHandIsLeft' 'Palm-normal-twist suppression must never be gated to right-primary firing again.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'if \(supportGrip\.disableAuthoredSupportNormalTwist\)[\s\S]{0,1600}solverInput\.useSupportNormalTwist\s*=\s*false;[\s\S]{0,200}solverInput\.supportNormalTwistFactor\s*=\s*0\.0f;' 'Bilateral authored support must keep axis aiming while permanently removing palm-normal twist.'
foreach ($removedFullRigidPath in @(
        'src/physics-interaction/animation/AuthoredWeaponGripCapture.h',
        'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp',
        'src/physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h')) {
    Reject-Text $removedFullRigidPath 'tryResolvePrimaryFiringGripAlignment|resolveAuthoredPrimaryHandWorld|resolveAuthoredPrimaryWeaponWorld\(' 'The superseded full-rigid authored capture resolver must remain removed.'
}

if ($failures.Count -gt 0) {
    Write-Host 'AuthoredWeaponGripCaptureSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'AuthoredWeaponGripCaptureSourceTests passed.' -ForegroundColor Green
