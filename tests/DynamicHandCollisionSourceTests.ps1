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

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Require-OrderedText {
    param(
        [string]$Path,
        [string[]]$Patterns,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $remaining = $text.Substring($offset)
        $match = [regex]::Match($remaining, $pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

# Canonical free-hand world collision uses dynamic velocity-driven proxies.

# The shared wrapper must not run FO4VR's keyframed initializer over dynamic
# twins. That routine zeros inverse mass, allowing tracked bodies to move while
# preventing the solver from displacing hands at static-world contact.
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'deriveMotionCinfo\s*\(' `
    'Dynamic hand twins must retain the motion-cinfo constructor inverse mass.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    '0x1417A3A90 is initializeAsKeyFramed[\s\S]{0,500}motionCinfoCtor\(motionCinfo\);' `
    'The shared body wrapper must preserve the dynamic-safe native constructor profile.'

# Live-world teardown must use deferred retirement (2026-07-08 UAF lesson).
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'body\.retireDeferred\(' `
    'Dynamic hand twin teardown must go through retireDeferred.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'body\.destroy\(' `
    'Dynamic hand twins must never destroy() a live-world body immediately.'

# The twins must mirror the production collider conventions: hand frames/shapes
# come from HandBoneColliderSet and forearm frames/shapes from BodyBoneColliderSet,
# never from independently re-derived geometry.
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'hand\.dynamicTwinTargets\(\)' `
    'Dynamic hand twins must consume the HandBoneColliderSet role-frame publication.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'hand\.buildDynamicTwinShape\(' `
    'Dynamic hand twins must build shapes through the shared collider hull construction.'
Require-OrderedText 'src/physics-interaction/hand/HandBoneColliderSet.cpp' @(
    'publishTwinSlot\(twinTargets\.palm',
    'hand_collider_semantics::isFingerRole\(instance\.role\)',
    'publishTwinSlot\(\s*twinTargets\.fingers\[fingerIndex\]\[segmentIndex\]',
    '_dynamicTwinTargets = twinTargets;'
) 'HandBoneColliderSet must publish the palm and all 15 finger-segment twin frames every update.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'makeDescriptorFrame\(',
    'collectForearmTwinMergeSource\(forearmTwinMergeSources, descriptor, frame\);',
    'queueBodyTarget\(instance\.body, frame\.transform',
    'publishMergedForearmTwinTargets\('
) 'BodyBoneColliderSet must merge the exact three frames queued to its keyframed forearm/wrist bodies.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'BoneColliderRole::ForearmSegment',
    'kForearmUpperMergeSource',
    'kForearmLowerMergeSource',
    'BoneColliderRole::HandSegment',
    'kWristMergeSource'
) 'The merged dynamic forearm must include ForeArm1->2, ForeArm2->3, and ForeArm3->Hand sources.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'forearmStartBone = isLeft \? "LArm_ForeArm1" : "RArm_ForeArm1"',
    'handBone = isLeft \? "LArm_Hand" : "RArm_Hand"',
    'mergedFrame\.length = sources\[0\]\.length \+ sources\[1\]\.length \+ sources\[2\]\.length'
) 'The single dynamic forearm must span ForeArm1->Hand and retain all three tuned source lengths.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' `
    'buildDynamicForearmTwinShape[\s\S]*buildShapeForFrame\(frame\)' `
    'Forearm twins must share the production body-collider hull construction.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'bodyBoneColliders\.dynamicForearmTwinTargets\(\)',
    'twinFrameForSlot\(handTwins, forearmTwins, isLeft, bodyIndex\)'
) 'Dynamic collision must consume the body-collider forearm frame publication.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'bodyBoneColliders\.buildDynamicForearmTwinShape\(twinFrame\)' `
    'Dynamic forearm twins must use the body-collider shared hull builder.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'dimensionsDrifted|twinFrame\.length - slot\.createdLength|twinFrame\.radius - slot\.createdRadius|twinFrame\.convexRadius - slot\.createdConvexRadius' `
    'Live pose dimensions must never trigger dynamic twin body reconstruction.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    '_canonicalForearmTwinDimensions = forearmTwinTargets;',
    'applyCanonicalForearmDimensions\(',
    '_dynamicForearmTwinTargets = forearmTwinTargets;'
) 'Forearm twins must retain generation-canonical dimensions while publishing live rigid targets.'
Require-OrderedText 'src/physics-interaction/hand/HandBoneColliderSet.cpp' @(
    '_canonicalDynamicTwinDimensions = canonicalTwinTargets;',
    'applyCanonicalHandDimensions\(',
    '_dynamicTwinTargets = twinTargets;'
) 'Palm and all finger twins must retain generation-canonical dimensions while publishing live rigid targets.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollisionTelemetry.h' `
    'kFingerSlotCount\s*=\s*[\s\S]*kHandFingerCount\s*\*[\s\S]*kHandFingerSegmentCount[\s\S]*kBodiesPerHand\s*=\s*kFirstForearmSlot\s*\+[\s\S]*static_assert\(static_cast<std::size_t>\(TwinRole::Forearm\)\s*\+\s*1\s*==\s*kBodiesPerHand\)' `
    'Dynamic hand telemetry and storage must account for all 15 finger segments plus palm and forearm.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'slot\.createdGeometryGeneration == geometryGeneration' `
    'Real source/tuning geometry generations must still rebuild dynamic twins once.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'if \(_transitionCollisionSuppressed \|\|',
    'slot\.createdGeometryGeneration == geometryGeneration',
    'retireSlot\(slot, frame\.bhkWorld\);'
) 'Queued geometry rebuilds must coalesce behind animation suspension and commit only after stable resume.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollisionTelemetry.h' `
    'kForearmSlot\s*=\s*kFirstForearmSlot[\s\S]*Forearm,[\s\S]*return "FARM"' `
    'Dynamic hand telemetry must expose one stable merged-forearm slot.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'DynHandTwin[RL]\.Forearm(?:Upper|Lower)|ROCK_DynHandTwin_[RL]_Forearm(?:Upper|Lower)' `
    'Dynamic collision must not retain the superseded split forearm twin bodies.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    'updateBodyBoneCollisions\(frame\);',
    '_dynamicHandCollision\.updateFrame\(',
    '_bodyBoneColliders,'
) 'Body forearm frames must publish before dynamic hand collision consumes them in the same game frame.'

# Finger residuals must be evaluated against immutable pre-correction intent.
# Only helpful anatomical motion may replace the established tip pushback;
# base/middle probes must never multiply the rigid whole-hand correction.
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'candidate\.intentFramesInHand\[linearIndex\]',
    'candidate\.closingProbeTravelInHand\[linearIndex\]',
    'candidate\.openingProbeTravelInHand\[linearIndex\]',
    'candidate\.active = true'
) 'Surface finger response must capture stable hand-local intent and calibrated opening/closing travel on contact entry.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'closingProbeTravelGameUnits',
    'openingProbeTravelGameUnits',
    'surface_finger_collision_policy::solve\(',
    'response\.lastDirections'
) 'Every contact must evaluate both anatomical directions against the solver-safe displacement.'
Require-Text 'src/physics-interaction/hand/SurfaceFingerCollisionPolicy.h' `
    'baselineCost[\s\S]*evaluateDirection[\s\S]*closing\.valid && opening\.valid[\s\S]*directionSwitchHysteresisFraction[\s\S]*selected->direction' `
    'Conflicting phalanx contacts must choose one coherent per-finger direction with switch hysteresis.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'handSlots\.surfaceFingerResponse\.intentFramesInHand',
    'composeTransforms\(\s*handInput\.rawHandWorld',
    'queueGeneratedKeyframedBodyTarget\('
) 'Contacted finger twins must chase immutable intent composed with the raw tracked hand, not corrected presentation.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'updateSurfaceFingerResponse\(',
    'isFingerTipSlot\(',
    'rigidPrimary \|\| unresolvedLegacyTip',
    'combineTwinDeviations\('
) 'Helpful curl must replace only its fingertip rigid fallback before whole-hand deviation combination.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'const bool rigidPrimary = bodyIndex == kPalmSlot \|\|[\s\S]{0,180}bodyIndex >= kFirstForearmSlot' `
    'Palm and forearm must remain the only unconditional rigid-hand collision channels.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'setHandPoseCustomWithPriority\([\s\S]{0,400}rockHandCollisionDynamicVisualPriority' `
    'Surface finger response must use the existing priority-arbitrated FRIK pose authority.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'setHandPoseCustomWithPriority\(',
    'isHandPoseTagActive\(',
    'response\.lastHelpfulDynamicSlotMask = 0'
) 'Fingertip rigid fallback may be suppressed only while the surface finger pose actually owns hFRIK authority.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'clearSurfaceFingerResponse\(_hands\[0\], false\)[\s\S]*clearSurfaceFingerResponse\(_hands\[1\], true\)' `
    'World/menu shutdown must deterministically release both surface finger pose claims.'

# Render-follow pipeline: combine per-body deviations, apply normal contact
# exactly, and reserve smoothing for explicit teleport recovery only.
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'sanitizeHandTargetResponseScale\(twinFrame->handTargetResponseScale\)',
    'handTargetCorrectionWorldGame',
    'combineTwinDeviations\(',
    'float smoothingSpeed = 0\.0f',
    'teleportRecoverySecondsRemaining > 0\.0f',
    'smoothAppliedDeviation\(',
    'applyExternalHandWorldTransform\('
) 'Dynamic hand render-follow must map forearm leverage, combine contacts, and smooth only teleport recovery before publication.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'dynamicInteractionFingerContact[\s\S]{0,500}dynamicInteractionFingerContact\s*\?[\s\S]{0,80}0\.0f[\s\S]{0,120}rockHandCollisionSurfaceFingerSmoothingSpeed' `
    'Hand/hand and hand/weapon finger contacts must publish their collision pose without presentation lag.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'shoulderBone = isLeft \? "LArm_UpperArm" : "RArm_UpperArm"',
    'forearmHandTargetResponseScale\(',
    'shoulder\.translate',
    'input\.end\.translate',
    'mergedFrame\.transform\.translate'
) 'Merged forearm response must derive its IK leverage from the live shoulder, hand, and proxy center.'

# Physics-owned telemetry must cross to the main frame through atomics and
# expose requested/commanded/live positions without changing provider API V1.
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.h' `
    'struct AtomicPhysicsTelemetry' `
    'Dynamic hand telemetry must have an explicit physics-to-main atomic publication boundary.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'publishPhysicsTelemetry\(',
    'requestedTargetWorldGame',
    'commandedTargetWorldGame',
    'liveBodyWorldGame',
    'getTelemetrySnapshot\('
) 'Dynamic hand telemetry must publish requested, commanded, and live proxy state to a main-frame snapshot.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollisionTelemetry.h' `
    'struct TwinSample' `
    'Dynamic hand telemetry must keep a fixed per-twin sample contract for future API adaptation.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollisionTelemetry.h' `
    'struct HandSample' `
    'Dynamic hand telemetry must keep aggregate per-hand contact and visual state.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollisionTelemetry.h' `
    'bool targetVelocityValid' `
    'Dynamic hand telemetry must distinguish a stationary target from an unavailable velocity sample.'

# Haptics are generated from real post-solve contact entry, consumed under
# stronger ownership, and delivered through the shared main-thread mixer.
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'samplePostSolveDeviations\(',
    'contactEntrySequenceAtomic\.fetch_add'
) 'Dynamic hand contact entry must be published by the post-solve physics phase.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'updateHandHaptic\(' `
    'Dynamic hand contact entry must be consumed by the main-frame haptic policy.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    '_dynamicHandCollision\.updateFrame\(',
    '_dynamicHandCollision\.consumeHapticEvents\(\)',
    '_feedbackHaptics\.queue\(',
    'updateFeedbackHaptics\(frame\.deltaSeconds\);'
) 'Dynamic hand haptics must flow through the shared main-thread FeedbackHaptics queue.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'VRControllers\.triggerHaptic' `
    'Dynamic hand collision must never trigger controller haptics from its runtime or physics callbacks.'

# Debug visualization must consume the same telemetry snapshot intended for a
# later API adapter, rather than re-reading live bodies through a second path.
Require-OrderedText 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' @(
    'if \(drawDynamicHandColliders\)',
    'getTelemetrySnapshot\(telemetry\)',
    'requestedGapGameUnits',
    'approachSpeedGameUnitsPerSecond'
) 'Dynamic hand overlay must visualize the canonical collision telemetry snapshot.'

# Profiling distinguishes frame work, pre-collide drive, and post-solve reads.
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'DynamicHandCollisionFrame' `
    'Dynamic hand main-frame work must have a dedicated profiler scope.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'DynamicHandCollisionPhysicsDrive' `
    'Dynamic hand physics drive must have a dedicated profiler scope.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'DynamicHandCollisionPostSolve' `
    'Dynamic hand post-solve sampling must have a dedicated profiler scope.'

# Every shipped config enables the canonical runtime and carries its haptics.
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'bHandCollisionDynamicDrive\s*=\s*true' `
        "$configPath must enable canonical dynamic world collision by default."
    Require-Text $configPath `
        'bHandCollisionDynamicHapticsEnabled\s*=\s*true' `
        "$configPath must ship the dynamic hand haptic enable key."
    Require-Text $configPath `
        'fHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond' `
        "$configPath must document the dynamic hand haptic speed units."
    Reject-Text $configPath `
        'SoftContact|ContactTargetIdentity' `
        "$configPath must not retain legacy soft-contact or target-identity keys."
}

# Deviation is a two-stage POST-SOLVE measurement against the same substep's
# targets: the residual vs the COMMANDED (velocity-limited) target detects
# contact, and only in contact is the render deviation published, measured vs
# the REQUESTED (pre-limit) target. Pre-collide sampling leaks tracking lag;
# rendering the commanded residual saturates at the dt-dependent limiter
# distance (framerate-modulated milli-punch pulsing).
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'void DynamicHandCollisionRuntime::samplePostSolveDeviations\(',
    'tryResolveLiveBodyWorldTransform\(',
    'commandedTargetGame',
    'requestedTargetGame'
) 'Dynamic hand deviation must detect contact vs the commanded target and measure vs the requested target.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    'void PhysicsInteraction::observeCustomGrabAuthorityAfterSolve\(',
    '_dynamicHandCollision\.samplePostSolveDeviations\([\s\S]*world,[\s\S]*completedSolveSequence,[\s\S]*timing\);'
) 'Dynamic hand post-solve sampling must run in the after-solve physics phase.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'liveBodyGamePosition\.x - result\.targetGamePosition' `
    'Dynamic hand deviation must not be derived from the pre-collide drive telemetry.'

# The divergence dwell and the drive-side recovery teleport must run on the
# REQUESTED-target gap: the commanded-target delta (bodyDeltaGameUnits)
# saturates at maxLinearVelocity * driveDt and can never cross a divergence
# threshold, which silently makes the recovery teleport dead code.
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'bodyDeltaGameUnits > divergenceThreshold' `
    'Dynamic hand divergence dwell must not gate on the saturating commanded-target delta.'
Require-OrderedText 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' @(
    'result\.requestedTargetGamePosition = requestedTarget\.translate;',
    'requestedGapGameUnits > mode\.divergenceTeleportGameUnits',
    'target = requestedTarget;'
) 'Dynamic drive divergence teleport must measure against and place at the requested target.'

# Contact press cap: an established contact must lean, not slam. The drive
# clamps only the velocity component along the press direction, after the
# hard-keyframe computation; the caller feeds the direction from the last
# post-solve deviation.
Require-OrderedText 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' @(
    'kFunc_ComputeHardKeyFrame',
    'hasContactPressDirection',
    'contactPressMaxVelocityHavok',
    'setVelocity\('
) 'Dynamic drive must clamp the contact press velocity after the hard-keyframe computation.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'lastPostSolveDeviationValid',
    'mode\.hasContactPressDirection = true;',
    'driveGeneratedKeyframedBody\('
) 'Dynamic hand flush must arm the press cap from the last post-solve deviation.'

# The twins get their own visualization flag, independent of the keyframed
# collider debug draws.
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'if \(drawDynamicHandColliders\)' `
    'Dynamic hand twins must draw behind their own bDebugDrawDynamicHandColliders flag.'

# The drive keeps chasing the wand while another system owns the hand pose:
# target queueing must happen BEFORE the ownership gate.
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'queueGeneratedKeyframedBodyTarget\(',
    'ownedByStrongerSystem'
) 'Dynamic hand drive must queue the wand target before evaluating visual ownership gates.'

# Dynamic collision is the only free-hand world-collision implementation.
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'SoftContactRuntime|_softContactRuntime|NativeContactEvidence' `
    'PhysicsInteraction must not retain a legacy soft-contact fallback or evidence cache.'
Reject-Text 'CMakeLists.txt' `
    'ROCKSoftContact|SoftContactWorld' `
    'The build must not register legacy soft-contact tests or targets.'
foreach ($legacyPath in @(
        'src/physics-interaction/contact/SoftContactMath.h',
        'src/physics-interaction/contact/SoftContactRuntime.cpp',
        'src/physics-interaction/contact/SoftContactRuntime.h',
        'src/physics-interaction/contact/SoftContactWorldPolicy.h',
        'src/physics-interaction/contact/NativeContactEvidence.h',
        'src/physics-interaction/contact/ContactTargetIdentity.cpp',
        'src/physics-interaction/contact/ContactTargetIdentity.h',
        'tests/SoftContactWorldOnlySourceTests.ps1',
        'tests/SoftContactWorldPolicyTests.cpp')) {
    if (Test-Path -LiteralPath (Join-Path $Root $legacyPath)) {
        $failures.Add("Legacy soft-contact file must be deleted: $legacyPath")
    }
}

# Existing keyframed callers must keep their behavior: the drive-mode parameter
# stays defaulted to keyframe placement.
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' `
    'const GeneratedBodyDriveMode& mode = \{\}' `
    'driveGeneratedKeyframedBody must default to keyframe placement for existing callers.'

# Each hand keeps a stable row. Both retain world/car collision while the
# experimental graph gates only the opposite hand and weapon edges.
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'ROCK_LAYER_DYNAMIC_HAND_PROXY = 48',
    'ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER = 49',
    'ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER = 50',
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY = 51',
    'ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY = 52',
    'buildRockDynamicHandProxyExpectedMask\(',
    'isWorldSurfaceLayer\(layer\)',
    'withLayer\(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER\)',
    'withLayer\(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER\)',
    'interactionsEnabled',
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY'
) 'Dynamic hand proxy rows must preserve world/car collision and explicitly gate cross-owner interaction edges.'
Require-Text 'src/physics-interaction/object/DynamicWorldCarCollision.cpp' `
    'isExplodableCarReference[\s\S]*dynamicWorldCarLayerForNativeLayer[\s\S]*setFilterInfo' `
    'Only verified ExplodableCar references may be tagged onto dynamic-world car layers.'
Require-Text 'src/physics-interaction/object/DynamicWorldCarCollision.cpp' `
    'currentFilterInfo != tagged\.taggedFilterInfo[\s\S]*resolveBodyToRef[\s\S]*tagged\.originalFilterInfo' `
    'Car filter restoration must verify both current filter ownership and native body identity.'
Require-Text 'src/physics-interaction/object/DynamicWorldCarCollision.cpp' `
    'DynamicWorldCarCollisionRuntime::restoreSlot[\s\S]*restoreTaggedBodiesForReference\(bhkWorld, hknpWorld, ref, seedBodyId, reason\)' `
    'Car teardown must restore the native layer even if another owner changed non-layer filter bits.'
Require-Text 'src/physics-interaction/collision/CollisionLayerPolicy.h' `
    'nativeCharacterControllerObjectSuppressionLayerMask[\s\S]*FO4_LAYER_CLUTTER[\s\S]*FO4_LAYER_CLUTTER_LARGE[\s\S]*originalMask & ~nativeCharacterControllerObjectSuppressionLayerMask' `
    'Ordinary clutter and large clutter must be rejected at the character-controller matrix.'
Reject-Text 'src/physics-interaction/collision/CollisionLayerPolicy.h' `
    'nativeCharacterControllerBodyFilteredLayerMask|isNativeCharacterControllerBodyFilteredLayer' `
    'Native clutter must not be globally re-enabled for a late per-body character-controller filter.'
Require-Text 'src/physics-interaction/collision/CollisionLayerPolicy.h' `
    'buildRockDynamicWorldCarExpectedMask[\s\S]*withoutLayer\(mask, ROCK_LAYER_HAND\)[\s\S]*withoutLayer\(mask, ROCK_LAYER_WEAPON\)[\s\S]*withoutLayer\(mask, ROCK_LAYER_BODY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_WEAPON_PROXY\)' `
    'Car-only rows must reject generated gameplay colliders while admitting both hands and the weapon solver proxy.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'kNearbyCarCollisionRadiusGameUnits[\s\S]*isExplodableCar[\s\S]*synchronizeNearbyTargets' `
    'Verified nearby cars must be proactively tagged before player contact.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' `
    'isDynamicWorldCarLayer\(layer\)[\s\S]*targetIsCar = targetIdentity\.isCar' `
    'Character-controller contact identity must be evaluated only on dedicated car rows.'
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'inline void applyRockGeneratedLayerPolicies\(',
    'applyRockDynamicHandProxyLayerPolicies\(',
    'dynamicHandInteractionsEnabled'
) 'Both dynamic hand rows must be applied with the other generated layer rows.'

# The proxy drive flush must run beside the other generated collider flushes.
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    '_weaponCollision\.flushPendingPhysicsDrive\(world, timing\);',
    '_dynamicHandCollision\.flushPendingPhysicsDrive\(world, timing\);'
) 'Dynamic hand proxy drive must flush in the generated collider physics substep.'

# Fixed-surface grabs use a separate bounded contact channel. Palm and
# fingertips are eligible; the forearm and ordinary loose-object semantic set
# remain excluded.
Require-Text 'src/physics-interaction/hand/DynamicHandSurfaceContactState.h' `
    'collectFresh[\s\S]*atomic_flag writer[\s\S]*sequence' `
    'Dynamic surface contact publication must be bounded and non-blocking on the physics callback.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'for \(std::size_t slot = 0; slot < kBodiesPerHand; \+\+slot\)[\s\S]{0,180}isSurfaceGrabSourceSlot\(slot\)' `
    'Only dynamic palm and fingertip twins may seed fixed-surface grabs.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteractionContacts.inl' @(
    'tryClassifySurfaceContactSourceAtomic\(',
    'isDynamicHandProxySurfaceLayer\(',
    'recordSurfaceContactCallback\(',
    '_generatedBodyContactRegistry\.tryClassify\('
) 'Dynamic surface evidence must publish before the ordinary generated-body prefilter discards proxy pairs.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'kRaiseManifoldProcessedEvents\s*=\s*0x40u[\s\S]*isSurfaceGrabSourceSlot\([\s\S]*enableBodyFlags\([\s\S]{0,350}kRaiseManifoldProcessedEvents[\s\S]{0,200}kRebuildBodyCollisionState[\s\S]*flaggedBody\.body->flags\s*&\s*kRaiseManifoldProcessedEvents' `
    'Palm and fingertip twins must opt into the verified key-2 processed-manifold event path without flagging forearms.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteractionContacts.inl' @(
    'handleManifoldProcessedEvent\(',
    'tryClassifySurfaceContactSourceAtomic\(',
    'recordSurfaceManifoldProcessedCallback\(',
    'recordObstacleManifoldProcessedCallback\('
) 'Processed manifolds must publish dynamic-hand surface evidence while preserving the dynamic-weapon route.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'recordSurfaceManifoldProcessedCallback\([\s\S]*isDynamicHandProxySurfaceLayer\(otherLayer\)[\s\S]*_surfaceContacts\.record\(source, otherBodyId, nullptr, nullptr\)' `
    'Processed hand manifolds must validate the surface layer before publishing point-free latch evidence.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'surfaceCallbacks\(impulse/manifold/eligible/published\)' `
    'The rate-limited dynamic-hand trace must distinguish callback, layer, and publication failures.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'beginSurfaceLatch\(',
    'tryResolveLiveBodyWorldTransform\(',
    'invertTransform\(targetWorld\)',
    'composeTransforms\(',
    'candidate\.active = true'
) 'Surface latch acquisition must capture the presented hand and live proxy transforms relative to the target body.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'if \(handSlots\.surfaceLatch\.active\)',
    'targetSnapshot\.body == latch\.targetBodyIdentity',
    'latch\.lastProxyWorld\[bodyIndex\]',
    'RE::NiTransform driveTarget',
    'queueGeneratedKeyframedBodyTarget\('
) 'A held surface latch must follow target-body motion and drive every proxy from the captured rigid relationship.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'kSurfaceLatchVisualPriority[\s\S]*applyExternalHandWorldTransform\(' `
    'Surface latches must hold the rendered hand through the existing FRIK visual authority bridge.'

# The opt-in API remains authoritative, while the shipped INI enables a
# built-in world-surface fallback for direct play and testing.
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'bGlobalSurfaceGrabEnabled\s*=\s*true' `
        "$configPath must globally enable fixed-surface grabs."
}
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'bHandCollisionSurfaceFingerResponseEnabled\s*=\s*true[\s\S]*fHandCollisionSurfaceFingerProbeDeltaOpenUnits[\s\S]*fHandCollisionSurfaceFingerResponseGain[\s\S]*fHandCollisionSurfaceFingerMaximumDeflectionOpenUnits[\s\S]*fHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits[\s\S]*fHandCollisionSurfaceFingerDirectionSwitchHysteresisFraction[\s\S]*fHandCollisionSurfaceFingerSmoothingSpeed[\s\S]*fHandCollisionSurfaceFingerReleaseDelaySeconds' `
        "$configPath must ship the globally enabled, bounded experimental surface finger response."
}
Require-Text 'src/RockConfig.h' `
    'rockHandCollisionSurfaceFingerResponseEnabled\s*=\s*true' `
    'Older INIs must inherit the enabled surface finger response default.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(\s*SECTION,\s*"bHandCollisionSurfaceFingerResponseEnabled"' `
    'The surface finger feature switch must load through the ROCK INI path.'
Require-OrderedText 'src/RockConfig.cpp' @(
    'fHandCollisionSurfaceFingerProbeDeltaOpenUnits',
    'fHandCollisionSurfaceFingerResponseGain',
    'fHandCollisionSurfaceFingerMaximumDeflectionOpenUnits',
    'fHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits',
    'fHandCollisionSurfaceFingerDirectionSwitchHysteresisFraction',
    'fHandCollisionSurfaceFingerSmoothingSpeed',
    'fHandCollisionSurfaceFingerReleaseDelaySeconds'
) 'Every bounded surface finger control must load through the ROCK INI path.'
Require-Text 'src/RockConfig.h' `
    'rockGlobalSurfaceGrabEnabled\s*=\s*true' `
    'The compiled global surface-grab default must remain enabled when an older INI lacks the key.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(SECTION,\s*"bGlobalSurfaceGrabEnabled",\s*rockGlobalSurfaceGrabEnabled\)' `
    'The global surface-grab switch must load through the normal ROCK INI path.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    'setGlobalSurfaceGrabEnabled\(',
    '_touchGrabRuntime\.service\('
) 'INI surface-grab state must update before active latch validation and acquisition.'
Require-Text 'src/physics-interaction/grab/GlobalSurfaceGrabPolicy.h' `
    'enabled\s*&&[\s\S]*!providerMatched[\s\S]*wildcardPass[\s\S]*dynamicSurfaceContact[\s\S]*isDynamicHandProxySurfaceLayer' `
    'The global path must be a dynamic-surface wildcard fallback that never overrides a provider match.'
Require-OrderedText 'src/physics-interaction/grab/TouchGrabRuntime.cpp' @(
    'resolveTouchGrabTargetV1\(',
    'if \(!providerMatched\)',
    'shouldUseFallback\(',
    'makeGlobalSurfaceTarget\(',
    'canFollowUnclassifiedMotion\('
) 'Provider-authored touch targets must resolve before global fallback synthesis.'
Reject-Text 'src/physics-interaction/grab/TouchGrabRuntime.cpp' `
    '!snapshot\.valid\s*\|\|\s*motionClass\s*==\s*provider::TouchGrabMotionClassV1::Other' `
    'Noncanonical motion-property handles must not be rejected before the non-mutating global FixedAnchor fallback is resolved.'
Require-Text 'src/physics-interaction/grab/GlobalSurfaceGrabPolicy.h' `
    'canFollowUnclassifiedMotion[\s\S]{0,220}globalSurfaceFallback\s*&&\s*fixedAnchor' `
    'Only a built-in global FixedAnchor may follow a body whose motion-property handle is not classified.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'Touch grab edge gated:[\s\S]{0,900}touchGrabPhysicsWritesAllowed' `
    'A rejected grip edge must identify the orchestration gate that blocked acquisition.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'Touch grab attempt rejected:[\s\S]{0,900}latchFailure' `
    'A failed contact candidate must report its target and surface-latch rejection stage.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'SurfaceLatchFailure::PrerequisiteUnavailable[\s\S]*SurfaceLatchFailure::ContactSourceMismatch[\s\S]*SurfaceLatchFailure::HandTransformUnavailable[\s\S]*SurfaceLatchFailure::TargetTransformUnavailable[\s\S]*SurfaceLatchFailure::SourceProxyUnavailable' `
    'Surface latch diagnostics must distinguish every acquisition prerequisite without hot-path retry logging.'
Require-Text 'src/physics-interaction/grab/TouchGrabRuntime.cpp' `
    'active\.globalSurface[\s\S]*_globalSurfaceGrabEnabled[\s\S]*releaseTarget\(' `
    'Disabling the INI setting must retire active global latches through normal cleanup.'

# A successful fixed-surface latch owns the feedback for that hand. Its
# one-shot confirmation is intentionally stronger and longer than the dynamic
# touch pulse, and both values remain user-tunable.
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'bSurfaceGrabHapticsEnabled\s*=\s*true[\s\S]*fSurfaceGrabHapticDurationSeconds\s*=\s*0\.075[\s\S]*fSurfaceGrabHapticIntensity\s*=\s*0\.85' `
        "$configPath must ship the distinct surface-latch confirmation pulse."
}
Require-Text 'src/RockConfig.h' `
    'rockSurfaceGrabHapticsEnabled\s*=\s*true[\s\S]{0,180}rockSurfaceGrabHapticDurationSeconds\s*=\s*0\.075f[\s\S]{0,180}rockSurfaceGrabHapticIntensity\s*=\s*0\.85f' `
    'The compiled surface-latch pulse must remain stronger and longer than the touch maximum.'
Require-Text 'src/RockConfig.cpp' `
    'bSurfaceGrabHapticsEnabled[\s\S]{0,500}fSurfaceGrabHapticDurationSeconds[\s\S]{0,500}fSurfaceGrabHapticIntensity' `
    'Surface-latch haptic controls must load through the normal ROCK INI path.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(touchGrabAcquired\)[\s\S]{0,900}getHandReport\([\s\S]{0,240}FixedAnchor[\s\S]{0,400}_feedbackHaptics\.queue\(' `
    'Fixed-surface acquisition must queue its confirmation only after the latch succeeds.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'dynamicHandHapticEvents[\s\S]{0,700}surfaceGrabOwnsFeedback[\s\S]{0,400}FixedAnchor[\s\S]{0,300}continue;[\s\S]{0,200}_feedbackHaptics\.queue\(' `
    'An active fixed-surface latch must suppress the lower-strength touch pulse before mixer delivery.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic hand collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic hand collision source boundary passed.'
