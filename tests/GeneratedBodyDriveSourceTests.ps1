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

Require-Text 'src/physics-interaction/HavokOffsets.h' 'kData_BhkWorldRawDeltaSeconds\s*=\s*0x65A3D70' 'Havok raw frame delta global must be named from the FO4VR SetDeltaTime path.'
Require-Text 'src/physics-interaction/HavokOffsets.h' 'kData_BhkWorldSubstepDeltaSeconds\s*=\s*0x65A3D74' 'Havok substep delta global must be named from the FO4VR SetDeltaTime path.'
Require-Text 'src/physics-interaction/HavokOffsets.h' 'kData_BhkWorldAccumulatedDeltaSeconds\s*=\s*0x65A3D84' 'Havok accumulated delta global must be named from the FO4VR SetDeltaTime path.'
Require-Text 'src/physics-interaction/HavokOffsets.h' 'kData_BhkWorldSubstepCount\s*=\s*0x65A3D8C' 'Havok substep count global must be named from the FO4VR SetDeltaTime path.'

Require-Text 'src/physics-interaction/HavokPhysicsTiming.h' 'struct PhysicsTimingSample' 'Generated-body drive must use an explicit Havok physics timing sample.'
Require-Text 'src/physics-interaction/HavokPhysicsTiming.h' 'simulatedDeltaSeconds' 'Havok timing sample must expose the simulated delta consumed by bhkWorld::Update.'
Require-Text 'src/physics-interaction/HavokPhysicsTiming.cpp' 'sampleCurrentTiming' 'Havok timing sample must be read from binary-backed globals, not FRIK frame time.'

Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.h' 'struct GeneratedKeyframedBodyDriveState' 'Generated-body target state must be shared by hand and weapon colliders.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.h' 'queueGeneratedKeyframedBodyTarget' 'Generated bodies must queue sampled targets before the Havok step consumes them.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.h' 'driveGeneratedKeyframedBody' 'Generated bodies must share one keyframe-drive policy.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.cpp' 'const RE::hkTransformf targetHavok = makeHavokTransform\(state\.pendingTarget\)' 'Normal generated-body movement must convert game-space targets to Havok-space hkTransformf before Bethesda keyframe drive.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.cpp' 'driveToKeyFrame\(targetHavok,\s*driveDelta\)' 'Normal generated-body movement must pass the converted Havok-space target to Bethesda keyframe drive.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.cpp' 'placeGeneratedKeyframedBodyImmediately' 'Transform writes must be isolated to initial placement and teleport/reset fallback.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.cpp' 'setVelocity\(zeroLinear,\s*zeroAngular\)' 'Immediate generated-body placement must clear stale keyframed velocity after transform fallback.'
Require-Text 'src/physics-interaction/BethesdaPhysicsBody.h' 'driveToKeyFrame\(const RE::hkTransformf& target,\s*float dt\)' 'Bethesda keyframe-drive wrapper must accept the native Havok-space transform type, not game-space NiTransform.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.cpp' 'Generated keyframed body drive owner=.*targetGame=' 'Generated-body successful drive telemetry must include target/body positions for runtime verification.'
Require-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.cpp' 'bodyDeltaGame=' 'Generated-body telemetry must report live body to target distance in game units.'

Require-Text 'src/physics-interaction/PhysicsStepDriveCoordinator.h' 'class PhysicsStepDriveCoordinator' 'Generated-body drives must be coordinated through a Havok step listener.'
Require-Text 'src/physics-interaction/PhysicsStepDriveCoordinator.cpp' 'kFunc_World_AddStepListener' 'Step coordinator must register with bhkWorld::AddStepListener.'
Require-Text 'src/physics-interaction/PhysicsStepDriveCoordinator.cpp' 'sampleCurrentTiming' 'Step coordinator must sample Havok timing inside the physics-step callback.'

Require-Text 'src/physics-interaction/HandBoneColliderSet.h' 'flushPendingPhysicsDrive\(RE::hknpWorld\* world,\s*const havok_physics_timing::PhysicsTimingSample& timing,\s*BethesdaPhysicsBody& palmAnchorBody\)' 'Hand generated colliders must expose a Havok-step flush method.'
Require-Text 'src/physics-interaction/WeaponCollision.h' 'flushPendingPhysicsDrive\(RE::hknpWorld\* world,\s*const havok_physics_timing::PhysicsTimingSample& timing\)' 'Weapon generated colliders must expose a Havok-step flush method.'
Require-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'registerForNextStep\(bhk,\s*hknp\)' 'PhysicsInteraction must register the generated-body step coordinator every update.'
Require-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'driveGeneratedBodiesFromPhysicsStep' 'PhysicsInteraction must own the actual generated-body step-drive callback.'
Require-Text 'src/physics-interaction/HavokPhysicsTiming.h' 'return timing\.rawDeltaSeconds' 'Generated-body keyframe drive must use the whole bhkWorld frame delta, not an implicit multiplied substep delta.'

Require-Text 'src/physics-interaction/HandBoneColliderSet.cpp' 'if \(!placeGeneratedKeyframedBodyImmediately\(instance\.body,\s*frame\.transform\)\)' 'Hand segment body creation must fail if native initial placement fails.'
Require-Text 'src/physics-interaction/HandBoneColliderSet.cpp' 'if \(!placeGeneratedKeyframedBodyImmediately\(palmAnchorBody,\s*anchorFrame\.transform\)\)' 'Hand palm anchor creation must fail if native initial placement fails.'
Require-Text 'src/physics-interaction/WeaponCollision.cpp' 'if \(!placeGeneratedKeyframedBodyImmediately\(instance\.body,\s*initialTransform\)\)' 'Weapon body creation must fail if native initial placement fails.'
Require-Text 'src/physics-interaction/HandBoneColliderSet.cpp' 'handleGeneratedBodyDriveResult' 'Hand generated collider physics-step drive results must be consumed.'
Require-Text 'src/physics-interaction/WeaponCollision.cpp' 'handleGeneratedBodyDriveResult' 'Weapon generated collider physics-step drive results must be consumed.'

Reject-Text 'src/physics-interaction/HandBoneColliderSet.cpp' 'kFunc_ComputeHardKeyFrame' 'Hand generated collider movement must not compute hard-keyframe velocity in the frame-update source path.'
Reject-Text 'src/physics-interaction/WeaponCollision.cpp' 'kFunc_ComputeHardKeyFrame' 'Weapon generated collider movement must not compute hard-keyframe velocity in the frame-update source path.'
Reject-Text 'src/physics-interaction/HandBoneColliderSet.cpp' 'body\.setTransform\(hkTransform\);\s*body\.setVelocity' 'Hand generated collider normal movement must not write transform plus velocity in one frame-update path.'
Reject-Text 'src/physics-interaction/WeaponCollision.cpp' 'instance\.body\.setTransform\(hkTransform\);\s*[\s\S]{0,120}instance\.body\.setVelocity' 'Weapon generated collider normal movement must not write transform plus velocity in one frame-update path.'
Reject-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'getFrameTime\(\)[\s\S]{0,220}_weaponCollision\.updateBodiesFromCurrentSourceTransforms' 'Weapon generated collider drive must not use FRIK frame time as its Havok velocity timestep.'
Reject-Text 'src/physics-interaction/GeneratedKeyframedBodyDrive.cpp' 'driveToKeyFrame\(state\.pendingTarget,\s*driveDelta\)' 'Normal generated-body movement must not pass raw game-space NiTransform to Bethesda keyframe drive.'
Reject-Text 'src/physics-interaction/WeaponCollision.cpp' 'result\.rotate\s*=\s*weapon_collision_geometry_math::transposeRotation\(weaponRootTransform\.rotate\)' 'Weapon generated body targets must remain ordinary Ni world transforms; Havok rotation conversion belongs in the shared generated-body drive.'

if ($failures.Count -gt 0) {
    Write-Host 'Generated body drive source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Generated body drive source boundary passed.'
