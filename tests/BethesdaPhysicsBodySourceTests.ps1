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
    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_BethesdaAllocatorInit' 'Bethesda allocator init offset is missing.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_BethesdaAllocatorState' 'Bethesda allocator state offset is missing.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_BethesdaTlsIndex' 'Bethesda TLS index offset is missing.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kBethesdaTlsAllocatorContext' 'Bethesda TLS allocator context offset is missing.'

Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'std::size_t,\s*std::uint32_t,\s*char' 'Bethesda allocator wrapper must call the verified four-argument allocator signature.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kFunc_BethesdaAllocatorInit' 'Bethesda allocator wrapper must initialize the native allocator before allocation.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' '__readgsqword\(0x58\)' 'Bethesda allocator wrapper must use the game TLS block for allocator context.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kBethesdaTlsAllocatorContext' 'Collision object allocation must set the native allocator TLS context.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'SetBodyKeyframed_t' 'Generated keyframed bodies must explicitly synchronize the hknp body motion state.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kFunc_SetBodyKeyframed' 'Generated keyframed bodies must call the native hknp keyframed-body function.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_MotionCinfo_Ctor\s*=\s*0x17A2FC0' 'Generated wrapper bodies must expose the verified hknpMotionCinfo constructor.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kSysData_MotionCinfos\s*=\s*kSysData_Array2' 'Generated wrapper bodies must name the system-data motion-cinfo array instead of using an anonymous array slot.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kFunc_MotionCinfo_Ctor' 'Generated wrapper bodies must initialize a local motion cinfo before AddToWorld.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'motionCinfo = hkArrayAppendOne\(motionCinfoArray,\s*0x70\)' 'Generated wrapper bodies must append a 0x70-byte motion cinfo for non-static bodies.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'motionCinfoCtor\(motionCinfo\)' 'Generated wrapper bodies must run the native hknpMotionCinfo constructor.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'ci \+ 0x0C\)\s*=\s*generatedLocalMotionIndex' 'Generated wrapper body cinfo motionId must be the local motion-cinfo index consumed by bhkPhysicsSystem::CreateInstance.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' '!validateGeneratedBodyMotion\(world,\s*bodyId,\s*motionType\)[\s\S]{0,180}destroy\(bhkWorld\)' 'Post-AddToWorld generated motion validation failure must remove the native physics-system instance.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_HknpWorld_SetBodyMaterial\s*=\s*0x153AFC0' 'Generated body material assignment must expose the verified hknpWorld::setBodyMaterial offset.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kGeneratedSystemLocalMaterialIndex\s*=\s*0' 'Generated hknpPhysicsSystemData must use local material index 0 for its single local material.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'ci \+ 0x12\)\s*=\s*kGeneratedSystemLocalMaterialIndex' 'Generated body cinfo material field must be a local system-data material index, not a global world material ID.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'case BethesdaMotionType::Keyframed:[\s\S]{0,80}return 0xFF' 'Generated keyframed wrapper bodies must keep the direct-call default quality byte and apply keyframed state after AddToWorld.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kFunc_HknpWorld_SetBodyMaterial' 'Generated bodies must set the desired global world material after native add-to-world.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'setBodyMaterial\(world,\s*bodyId\.value,\s*materialId\.value,\s*0\)' 'Generated bodies must use the immediate native cache-update mode when assigning their desired global world material.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_CollisionObject_AddToWorld' 'Verified bhkNPCollisionObject add-to-world offset is missing.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'CollisionObjectAddToWorld_t' 'Generated collider bodies must call the verified bhkNPCollisionObject add-to-world phase.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kFunc_CollisionObject_AddToWorld' 'Generated collider bodies must use bhkNPCollisionObject::AddToWorld/vfunction49, not CreateInstance alone.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'createNiNode\(name\)[\s\S]{0,600}addToWorld\(_collisionObject,\s*bhkWorld\)' 'Generated collider bodies must link an owner NiNode before native world insertion.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'getBodyId\(_physicsSystem,\s*&bodyId' 'Generated collider bodies must resolve their hknp body id from the native bhkPhysicsSystem instance.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'setBodyKeyframed\(world,\s*bodyId\.value\)' 'Generated keyframed bodies must apply hknp keyframed state after native add-to-world returns a body id.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_BhkWorld_RemovePhysicsSystemInstance' 'Generated collider teardown must name the bhkWorld removal wrapper by its runtime physics-system instance contract.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'nativePhysicsSystemInstance\(_physicsSystem\)' 'Generated collider teardown must read the runtime hknpPhysicsSystemInstance pointer from bhkPhysicsSystem before removal.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'removePhysicsSystem\(bhkWorld,\s*physicsSystemInstance\)' 'Generated collider teardown must pass the runtime physics-system instance to bhkWorld::RemovePhysicsSystem.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'bodyId\.value == kInvalidGeneratedId[\s\S]{0,260}destroy\(bhkWorld\)' 'Post-AddToWorld invalid body-id failure must remove the native physics-system instance.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' '!applyGeneratedBodyMaterial\(world,\s*bodyId,\s*materialId\)[\s\S]{0,180}destroy\(bhkWorld\)' 'Post-AddToWorld material assignment failure must remove the native physics-system instance.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_NiNode_Ctor\s*=\s*0x1C17D30' 'Generated collider owner nodes must call the verified NiNode constructor, not the destructor.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'NiNodeCtor_t\s*=\s*void\*\s*\(\*\)\(void\*,\s*std::uint16_t\)' 'NiNode constructor wrapper must include the native child-array capacity argument.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'niNodeCtor\(mem,\s*0\)' 'Generated owner NiNodes must be constructed with zero initial child capacity.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'GetFilterInfo_t\s*=\s*std::uint32_t\*\s*\(\*\)\(void\*,\s*std::uint32_t\*\)' 'bhkNPCollisionObject::GetFilterInfo must be called with the verified out-param signature.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'getFilter\(_collisionObject,\s*&filterInfo\)' 'Generated body filter reads must pass a valid out-param to bhkNPCollisionObject::GetFilterInfo.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'DriveToKeyFrame_t\s*=\s*std::uint8_t\s*\(\*\)\(void\*,\s*const void\*,\s*float\)' 'bhkNPCollisionObject::DriveToKeyFrame must use the verified low-byte status return.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'SetTransform_t\s*=\s*std::uint8_t\s*\(\*\)\(void\*,\s*const void\*\)' 'bhkNPCollisionObject::SetTransform must use the verified low-byte status return.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'SetVelocity_t\s*=\s*std::uint8_t\s*\(\*\)\(void\*,\s*const float\*,\s*const float\*\)' 'bhkNPCollisionObject::SetVelocity must use the verified low-byte status return.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'ApplyImpulse_t\s*=\s*std::uint8_t\s*\(\*\)\(void\*,\s*const float\*\)' 'bhkNPCollisionObject::ApplyLinearImpulse must use the verified low-byte status return.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'ApplyPointImpulse_t\s*=\s*std::uint8_t\s*\(\*\)\(void\*,\s*const float\*,\s*const float\*\)' 'bhkNPCollisionObject::ApplyPointImpulse must use the verified low-byte status return.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'GetCOM_t\s*=\s*std::uint8_t\s*\(\*\)\(void\*,\s*float\*\)' 'bhkNPCollisionObject::GetCenterOfMassWorld must use the verified low-byte status return.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'bool setTransform\(const RE::hkTransformf& transform\)' 'SetTransform wrapper must expose native failure to callers.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'bool setVelocity\(const float\* linVel,\s*const float\* angVel\)' 'SetVelocity wrapper must expose native failure to callers.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'bool applyLinearImpulse\(const float\* impulse\)' 'ApplyLinearImpulse wrapper must expose native failure to callers.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.h' 'bool applyPointImpulse\(const float\* impulse,\s*const float\* worldPoint\)' 'ApplyPointImpulse wrapper must expose native failure to callers.'

Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'world->CreateBody' 'Generated collider bodies must not bypass the Bethesda wrapper with direct hknpWorld::CreateBody.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'Physics system instance constructed' 'Generated collider bodies must not fabricate a bhkPhysicsSystem instance.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'body\+0x88 back-pointer set manually' 'Generated collider bodies must not manually patch hknpBody userData/back-pointers.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'DestroyBodies' 'Generated collider body destruction must not manually destroy wrapper-owned hknp bodies.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'CreateInstance_t' 'Generated collider body creation must not return to the non-contacting bhkNPCollisionObject::CreateInstance-only path.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'createInstance\(_collisionObject,\s*bhkWorld\)' 'Generated collider body creation must not attach through bhkNPCollisionObject::CreateInstance.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kFunc_World_SetMotion' 'Generated body motion promotion must not use the recursive bhkWorld::SetMotion wrapper offset.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'kFunc_HknpWorld_SetBodyMotion' 'Generated wrapper bodies must not patch in a world motion after AddToWorld; they must provide a local motion cinfo before creation.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'setBodyMotion\(' 'Generated wrapper bodies must not call hknpWorld::setBodyMotion as a post-creation promotion path.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'world->AllocateMotion' 'Generated wrapper bodies must not write a direct world motion id into system-data bodyCinfo; bhkPhysicsSystem maps local motion-cinfo indices itself.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'ensureGeneratedBodyMotion' 'Generated wrapper body creation must validate wrapper-created motion, not promote static bodies after creation.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'removePhysicsSystem\(bhkWorld,\s*_physicsSystem\)' 'Generated collider teardown must not pass the bhkPhysicsSystem wrapper object to bhkWorld::RemovePhysicsSystem.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'ci \+ 0x12\)\s*=\s*materialId\.value' 'Generated body cinfo must not store a global material ID in the local material index field.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'ci \+ 0x50\)' 'Generated wrapper body cinfo must not write the shape/body-specialization seed byte as if it were hkp quality.'
Reject-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_NiNode_Ctor\s*=\s*0x1C17DD0' 'NiNode constructor offset must not point at the verified NiNode destructor.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'GetFilterInfo_t\s*=\s*std::uint32_t\s*\(\*\)\(void\*\)' 'bhkNPCollisionObject::GetFilterInfo must not be called as a return-value-only function.'
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' 'return getFilter\(_collisionObject\)' 'bhkNPCollisionObject::GetFilterInfo must not write through an uninitialized native out pointer.'

$handSet = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandBoneColliderSet.cpp')
if ($handSet -match 'createBodyForRole[\s\S]{0,260}else\s+if\s*\(\s*g_rockConfig\.rockHandBoneCollidersRequireAllFingerBones\s*\)') {
    $failures.Add('Generated hand collider body allocation failure must not be silently accepted when bHandBoneCollidersRequireAllFingerBones=false.')
}

if ($failures.Count -gt 0) {
    Write-Host 'Bethesda physics body source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Bethesda physics body source boundary passed.'
