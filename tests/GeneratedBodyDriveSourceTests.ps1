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

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_BhkWorldRawDeltaSeconds\s*=\s*0x65A3D70' 'Havok raw frame delta global must be named from the FO4VR SetDeltaTime path.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_BhkWorldSubstepDeltaSeconds\s*=\s*0x65A3D74' 'Havok substep delta global must be named from the FO4VR SetDeltaTime path.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_BhkWorldAccumulatedDeltaSeconds\s*=\s*0x65A3D84' 'Havok accumulated delta global must be named from the FO4VR SetDeltaTime path.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_BhkWorldSubstepCount\s*=\s*0x65A3D8C' 'Havok substep count global must be named from the FO4VR SetDeltaTime path.'

Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'struct PhysicsTimingSample' 'Generated-body drive must use an explicit Havok physics timing sample.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'simulatedDeltaSeconds' 'Havok timing sample must expose the simulated delta consumed by bhkWorld::Update.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'enum class PhysicsStepPhase' 'Havok timing sample must identify whole-step versus substep callback phase.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'SubstepPreCollide' 'Generated collider drive must be able to run at FO4VR before-any-physics-step phase.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'makeSubstepTimingSample' 'Step coordinator must build an explicit substep timing sample for generated collider drive.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.cpp' 'sampleCurrentTiming' 'Havok timing sample must be read from binary-backed globals, not FRIK frame time.'

Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'struct GeneratedKeyframedBodyDriveState' 'Generated-body target state must be shared by hand and weapon colliders.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'mutable std::mutex mutex' 'Generated-body frame-to-physics target state must be synchronized across frame sampling and Havok-step flushing.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'queueGeneratedKeyframedBodyTarget' 'Generated bodies must queue sampled targets before the Havok step consumes them.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'driveGeneratedKeyframedBody' 'Generated bodies must share one keyframe-drive policy.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'std::scoped_lock lock\(state\.mutex\)' 'Generated-body queue, clear, snapshot, and drive operations must lock the shared target state.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'snapshotGeneratedKeyframedBodyDriveSampledVelocity' 'Generated-body contact velocity publication must read a locked target-state snapshot.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'kPredictionEnabled\s*=\s*false' 'Generated-body source prediction must remain disabled while the runtime issue is isolated.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'kVelocityHardSyncEnabled\s*=\s*false' 'Generated-body velocity hard-sync must remain disabled while the runtime issue is isolated.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'shouldUseSubstepInterpolatedTarget' 'Generated-body substep interpolation policy must be directly testable.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'shouldFinalizePlacedTargetForNextSource' 'Generated-body substep target promotion policy must be directly testable.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'selectGeneratedDriveTarget\(state,\s*timing\)' 'Normal generated-body movement must choose a phase-aware queued or retained game-space target while prediction is disabled.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'selectGeneratedImmediatePlacementTarget\(state\)' 'Generated-body teleport/reset fallback must place the final queued target instead of a substep-interpolated target.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'interpolateTargetTransform\(state\.previousTarget,\s*state\.pendingTarget,\s*timing\.substepProgress\)' 'Generated collider substep drive must interpolate from previous source target to pending source target.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'const RE::hkTransformf targetHavok = makeHavokTransform\(target\)' 'Normal generated-body movement must convert exact game-space targets to Havok-space hkTransformf before Bethesda keyframe drive.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'driveToKeyFrame\(targetHavok,\s*driveDelta\)' 'Normal generated-body movement must pass the converted Havok-space target to Bethesda keyframe drive.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'placeGeneratedKeyframedBodyImmediately' 'Transform writes must be isolated to initial placement and teleport/reset fallback.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'setVelocity\(zeroLinear,\s*zeroAngular\)' 'Immediate generated-body placement must clear stale keyframed velocity after transform fallback.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'inline void markGeneratedKeyframedBodyDrivePlaced' 'Generated-body placement bookkeeping must stay directly testable without linking runtime Havok wrappers into policy tests.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'shouldFinalizePlacedTargetForNextSource\(timing,\s*immediatePlacement\)' 'Successful generated-body drive must promote the final target immediately for teleport/reset, otherwise only at the final substep.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'driveToKeyFrame\(const RE::hkTransformf& target,\s*float dt\)' 'Bethesda keyframe-drive wrapper must accept the native Havok-space transform type, not game-space NiTransform.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'Generated keyframed body drive owner=.*targetGame=' 'Generated-body successful drive telemetry must include target/body positions for runtime verification.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'bodyDeltaGame=' 'Generated-body telemetry must report live body to target distance in game units.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'predictLead=' 'Generated-body telemetry must show prediction is disabled with a zero lead.'

Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.h' 'class PhysicsStepDriveCoordinator' 'Generated-body drives must be coordinated through a Havok step listener.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.h' 'setDriveCallbacks' 'Step coordinator must split whole-step native grab drive from substep generated collider drive.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'kFunc_World_AddStepListener' 'Step coordinator must register with bhkWorld::AddStepListener.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'sampleCurrentTiming' 'Step coordinator must sample Havok timing inside the physics-step callback.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'onBeforeAnyPhysicsStep' 'Step coordinator must consume FO4VR before-any-physics-step callbacks.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'makeSubstepTimingSample' 'Step coordinator must convert native before-any callback floats into a typed timing sample.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'allocateProcessLifetimeListener' 'Step listener storage must outlive PhysicsInteraction so bhkWorld cannot call into freed inline coordinator memory.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' '_nativeListener->owner = nullptr' 'Coordinator reset must leave queued bhkWorld listener callbacks inert while preserving a valid vtable.'

Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.h' 'flushPendingPhysicsDrive\(RE::hknpWorld\* world,\s*const havok_physics_timing::PhysicsTimingSample& timing,\s*BethesdaPhysicsBody& palmAnchorBody\)' 'Hand generated colliders must expose a Havok-step flush method.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.h' 'flushPendingPhysicsDrive\(RE::hknpWorld\* world,\s*const havok_physics_timing::PhysicsTimingSample& timing\)' 'Body generated colliders must expose a Havok-step flush method.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'kBodyFilterInfo\s*=\s*\(0x000B << 16\) \| \(collision_layer_policy::ROCK_LAYER_BODY & 0x7F\)' 'Body generated colliders must use the ROCK body collision layer, not the hand layer.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'bodyDescriptorsForPowerArmor\(snapshot\.inPowerArmor\)' 'Body generated colliders must select the power-armor descriptor profile from the live skeleton snapshot.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'rockBodyBoneColliderPowerArmorRadiusScale' 'Power armor body collider dimensions must have runtime config tuning on top of the power-armor descriptor profile.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'flushPendingPhysicsDrive\(RE::hknpWorld\* world,\s*const havok_physics_timing::PhysicsTimingSample& timing\)' 'Weapon generated colliders must expose a Havok-step flush method.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'registerForNextStep\(bhk,\s*hknp\)' 'PhysicsInteraction must register the generated-body step coordinator every update.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'driveGeneratedCollidersFromPhysicsSubstep' 'PhysicsInteraction must own the generated collider substep-drive callback.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_bodyBoneColliders\.flushPendingPhysicsDrive\(world,\s*timing\)' 'PhysicsInteraction generated collider callback must drive full body bone colliders in the Havok substep.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'if \(!_rightHand\.hasCollisionBody\(\) \|\| !_leftHand\.hasCollisionBody\(\)\)[\s\S]*createHandCollisions\(frame\.hknpWorld,\s*frame\.bhkWorld\)' 'Runtime scale invalidation or partial hand body loss must recreate hand generated colliders from updateHandCollisions.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' 'int _handColliderCreateRetryFrames = 0;' 'Hand generated collider runtime recreation must have a retry cooldown instead of failing permanently after scale changes.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'driveNativeGrabFromPhysicsStep' 'PhysicsInteraction must keep native mouse-spring grab on the whole-step callback.'
Require-Text 'src/physics-interaction/native/HavokPhysicsTiming.h' 'return timing\.substepDeltaSeconds' 'Generated-body keyframe drive must use FO4VR native substep delta when running from SubstepPreCollide.'

Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'if \(!placeGeneratedKeyframedBodyImmediately\(instance\.body,\s*frame\.transform\)\)' 'Hand segment body creation must fail if native initial placement fails.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'if \(!placeGeneratedKeyframedBodyImmediately\(palmAnchorBody,\s*anchorFrame\.transform\)\)' 'Hand palm anchor creation must fail if native initial placement fails.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'if \(!placeGeneratedKeyframedBodyImmediately\(instance\.body,\s*frame\.transform\)\)' 'Body bone collider creation must fail if native initial placement fails.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'if \(!placeGeneratedKeyframedBodyImmediately\(instance\.body,\s*initialTransform\)\)' 'Weapon body creation must fail if native initial placement fails.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'handleGeneratedBodyDriveResult' 'Hand generated collider physics-step drive results must be consumed.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'handleGeneratedBodyDriveResult' 'Body generated collider physics-step drive results must be consumed.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'handleGeneratedBodyDriveResult' 'Weapon generated collider physics-step drive results must be consumed.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'updateBodiesFromCurrentSourceTransforms\(RE::hknpWorld\* world,\s*RE::NiAVObject\* fallbackWeaponNode,\s*float sourceDeltaSeconds\)' 'Weapon generated collider source sampling must pass its source frame delta into the shared drive.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'const RE::NiAVObject\* packageDriveRoot = resolvePackageDriveNode' 'Weapon interaction probe math must resolve one package root before testing hull bounds.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'outContact\.interactionRoot = const_cast<RE::NiAVObject\*>\(packageDriveRoot\)' 'Weapon probe contacts must publish the same package root used for generated hull probing.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'RE::NiAVObject\* packageDriveNode = resolvePackageDriveNode\(bank,\s*nullptr\)' 'Weapon contact atomics must resolve one package root before publishing body contact metadata.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' '_weaponBodyInteractionRootsAtomic\[count\]\.store\(reinterpret_cast<std::uintptr_t>\(packageDriveNode\)' 'Weapon contact atomics must publish package-root interaction authority for every generated body.'

Reject-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'kFunc_ComputeHardKeyFrame' 'Hand generated collider movement must not compute hard-keyframe velocity in the frame-update source path.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'kFunc_ComputeHardKeyFrame' 'Weapon generated collider movement must not compute hard-keyframe velocity in the frame-update source path.'
Reject-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'body\.setTransform\(hkTransform\);\s*body\.setVelocity' 'Hand generated collider normal movement must not write transform plus velocity in one frame-update path.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'instance\.body\.setTransform\(hkTransform\);\s*[\s\S]{0,120}instance\.body\.setVelocity' 'Weapon generated collider normal movement must not write transform plus velocity in one frame-update path.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'getFrameTime\(\)[\s\S]{0,220}_weaponCollision\.updateBodiesFromCurrentSourceTransforms' 'Weapon generated collider drive must not use FRIK frame time as its Havok velocity timestep.'
Reject-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'driveToKeyFrame\(state\.pendingTarget,\s*driveDelta\)' 'Normal generated-body movement must not pass raw game-space NiTransform to Bethesda keyframe drive.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'state\.hasPendingTarget\s*=\s*true' 'Generated-body drive must retain the last exact target so Havok substeps keep moving when source sampling is between frames.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'refreshGeneratedKeyframedBodySourceClockForDrive' 'Generated-body drive must advance source age when physics flushes without a fresh source sample.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'state\.previousTarget\s*=\s*state\.hasPendingTarget\s*\?\s*state\.pendingTarget\s*:\s*state\.previousTarget' 'Exact-target drive must promote the placed target to the previous target for the next teleport-distance check.'
Reject-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'predictTargetTransform' 'Prediction transform extrapolation must not run while this experiment is disabled.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'evaluatePackageDriveHardSync\(' 'Weapon package velocity hard-sync must not run while this experiment is disabled.'
Reject-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'rockHandBoneColliderMaxLinearVelocity' 'Hand generated collider drive must not consume velocity hard-sync caps while disabled.'
Reject-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'rockHandBoneColliderMaxAngularVelocity' 'Hand generated collider drive must not consume velocity hard-sync caps while disabled.'
Reject-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'rockHandBoneColliderMaxLinearVelocity' 'Body generated collider drive must not consume hand velocity hard-sync caps while disabled.'
Reject-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'rockHandBoneColliderMaxAngularVelocity' 'Body generated collider drive must not consume hand velocity hard-sync caps while disabled.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'rockWeaponCollisionMaxLinearVelocity' 'Weapon generated collider drive must not consume velocity hard-sync caps while disabled.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'rockWeaponCollisionMaxAngularVelocity' 'Weapon generated collider drive must not consume velocity hard-sync caps while disabled.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'result\.rotate\s*=\s*weapon_collision_geometry_math::transposeRotation\(weaponRootTransform\.rotate\)' 'Weapon generated hull targets must transpose the package basis so centered hulls rotate with the same effective frame as their package-local centers.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'outContact\.interactionRoot = bestInstance->driveNode' 'Weapon probe contacts must not route support grip through per-body roots.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' '_weaponBodyInteractionRootsAtomic\[count\]\.store\(reinterpret_cast<std::uintptr_t>\(instance\.driveNode\)' 'Weapon contact atomics must not publish per-body roots as gameplay interaction authority.'

if ($failures.Count -gt 0) {
    Write-Host 'Generated body drive source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Generated body drive source boundary passed.'
