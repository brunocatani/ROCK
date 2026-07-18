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
    'HandFingerSegment::Tip',
    '_dynamicTwinTargets = twinTargets;'
) 'HandBoneColliderSet must publish palm anchor and fingertip twin frames every update.'
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
) 'Palm and fingertip twins must retain generation-canonical dimensions while publishing live rigid targets.'
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

# Render-follow pipeline: combine per-body deviations, smooth (rest twitch), gate.
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'sanitizeHandTargetResponseScale\(twinFrame->handTargetResponseScale\)',
    'handTargetCorrectionWorldGame',
    'combineTwinDeviations\(',
    'smoothAppliedDeviation\(',
    'applyExternalHandWorldTransform\('
) 'Dynamic hand render-follow must map forearm leverage, combine contacts, smooth, then apply the deviation.'
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
Require-OrderedText 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' @(
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
    '_dynamicHandCollision\.samplePostSolveDeviations\(world\);'
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
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
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

# The proxy layer must remain world-surface-only and applied with the other
# generated rows.
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'ROCK_LAYER_DYNAMIC_HAND_PROXY = 48',
    'buildRockDynamicHandProxyExpectedMask\(\)',
    'isWorldSurfaceLayer\(layer\)'
) 'Dynamic hand proxy layer must be 48 with a world-surface-only mask.'
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'inline void applyRockGeneratedLayerPolicies\(',
    'applyRockDynamicHandProxyLayerPolicy\(matrix\);'
) 'Dynamic hand proxy layer row must be applied with the other generated layer rows.'

# The proxy drive flush must run beside the other generated collider flushes.
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    '_weaponCollision\.flushPendingPhysicsDrive\(world, timing\);',
    '_dynamicHandCollision\.flushPendingPhysicsDrive\(world, timing\);'
) 'Dynamic hand proxy drive must flush in the generated collider physics substep.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic hand collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic hand collision source boundary passed.'
