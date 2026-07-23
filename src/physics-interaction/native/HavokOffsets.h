#pragma once

#include <cstddef>
#include <cstdint>

namespace rock::offsets
{

    constexpr std::uintptr_t kCollisionObject_OwnerNode = 0x10;

    constexpr std::uintptr_t kCollisionObject_PhysSystemPtr = 0x20;

    constexpr std::uintptr_t kBhkPhysicsSystem_Instance = 0x18;

    constexpr std::uintptr_t kHknpPhysicsSystemInstance_World = 0x18;

    constexpr std::uintptr_t kBhkWorld_HknpWorldPtr = 0x60;

    constexpr std::uintptr_t kNiAVObject_CollisionObject = 0x100;

    constexpr std::uintptr_t kHknpWorld_ModifierManager = 0x150;

    constexpr std::uintptr_t kModifierMgr_FilterPtr = 0x5E8;

    constexpr std::uintptr_t kFilter_CollisionMatrix = 0x1A0;

    constexpr std::uintptr_t kHknpWorld_MotionArrayPtr = 0xE0;

    constexpr std::uintptr_t kHknpWorld_MotionPropertiesLibraryPtr = 0x5D0;

    /*
     * FO4VR's world reader-writer lock (BSReadWriteLock: u32 owner thread id,
     * u32 state). Engine mutation wrappers write-lock it (raw-disasm verified in
     * bhkWorld::RemovePhysicsSystemInstance @0x1DFAD00: rbx=[bhkWorld+0x60],
     * lock at rbx+0x6D8); [bhkWorld+0x60] is the hknpWorld
     * (kBhkWorld_HknpWorldPtr), so the lock lives inside hknpWorld itself.
     * Threads inside the physics step publish a nonzero TLS byte at
     * kTlsSlot_InPhysicsStepFlag (exe TLS index at kGlobal_ExeTlsIndex) and the
     * engine skips locking there; ROCK must do the same.
     * See Docs/ROCK/lessons/2026-07-13-hknp-world-query-lock-discipline.md.
     */
    constexpr std::uintptr_t kHknpWorld_AccessLock = 0x6D8;

    constexpr std::uintptr_t kGlobal_ExeTlsIndex = 0x689CACC;

    constexpr std::uintptr_t kTlsSlot_InPhysicsStepFlag = 0x1529;

    constexpr std::uintptr_t kBody_CollisionFilterInfo = 0x44;

    constexpr std::uintptr_t kBody_CollisionObjectBackPointer = 0x88;

    constexpr std::uintptr_t kMotion_PropertiesId = 0x38;

    constexpr std::uintptr_t kMotion_MaxLinearVelocityPacked = 0x3A;

    constexpr std::uintptr_t kMotion_MaxAngularVelocityPacked = 0x3C;

    constexpr std::uintptr_t kMotionPropertiesLibrary_Entries = 0x28;

    constexpr std::uintptr_t kMotionPropertiesLibrary_Count = 0x30;

    constexpr std::size_t kMotionProperties_RecordSize = 0x40;

    constexpr std::uintptr_t kMotionProperties_LinearDamping = 0x18;

    constexpr std::uintptr_t kMotionProperties_AngularDamping = 0x1C;

    constexpr int kTransformA_Col0 = 0x30;
    constexpr int kTransformA_Col1 = 0x40;
    constexpr int kTransformA_Col2 = 0x50;
    constexpr int kTransformA_Pos = 0x60;

    constexpr int kTransformB_Col0 = 0x70;
    constexpr int kTransformB_Col1 = 0x80;
    constexpr int kTransformB_Col2 = 0x90;
    constexpr int kTransformB_Pos = 0xA0;

    constexpr std::uintptr_t kFunc_SetBodyCollisionFilterInfo = 0x1DF5B80;

    constexpr std::uintptr_t kFunc_SetBodyVelocity = 0x1539F30;

    constexpr std::uintptr_t kFunc_SetBodyTransformDeferred = 0x1DF55F0;

    constexpr std::uintptr_t kFunc_SetBodyVelocityDeferred = 0x1DF56F0;

    constexpr std::uintptr_t kFunc_SetBodyKeyframed = 0x1DF5CB0;

    constexpr std::uintptr_t kFunc_ComputeHardKeyFrame = 0x153a6a0;

    constexpr std::uintptr_t kFunc_RebuildMotionMassProperties = 0x1546570;

    constexpr std::uintptr_t kFunc_ConvexBuildConfig_Init = 0x16D4AB0;

    constexpr std::uintptr_t kFunc_ConvexShape_FromPoints = 0x16D4B30;

    constexpr std::uintptr_t kFunc_CompoundShapeCinfo_FromInstances = 0x16E1CF0;

    constexpr std::uintptr_t kFunc_StaticCompoundShape_Ctor = 0x1E9C950;

    constexpr std::uintptr_t kFunc_ShapeInstance_SetShape = 0x16E1780;

    constexpr std::uintptr_t kFunc_ShapeInstance_SetTransform = 0x16E1840;

    constexpr std::uintptr_t kFunc_ShapeInstance_SetScale = 0x16E1910;

    constexpr std::uintptr_t kFunc_CollisionObject_SetMotionType = 0x1E07300;

    constexpr std::uintptr_t kFunc_CollisionObject_Ctor = 0x1E07710;

    constexpr std::uintptr_t kFunc_CollisionObject_CreateInstance = 0x1E07AC0;

    constexpr std::uintptr_t kFunc_CollisionObject_AddToWorld = 0x1E07BE0;

    constexpr std::uintptr_t kFunc_PhysicsSystem_Ctor = 0x1E0C2B0;

    constexpr std::uintptr_t kFunc_PhysicsSystem_GetBodyId = 0x1E0C460;

    constexpr std::uintptr_t kFunc_PhysicsSystemData_Ctor = 0x5EAB0;

    constexpr std::uintptr_t kFunc_BodyCinfo_Ctor = 0x1561DD0;

    constexpr std::uintptr_t kFunc_MaterialCtor = 0x1536CB0;

    constexpr std::uintptr_t kFunc_CollisionObject_LinkObject = 0x2996CB0;

    constexpr std::uintptr_t kFunc_CollisionObject_DriveToKeyFrame = 0x1E086E0;

    constexpr std::uintptr_t kFunc_CollisionObject_SetTransform = 0x1E08A70;

    constexpr std::uintptr_t kFunc_CollisionObject_SetVelocity = 0x1E082A0;

    constexpr std::uintptr_t kFunc_CollisionObject_SetLinearVelocity = 0x1E08050;

    constexpr std::uintptr_t kFunc_CollisionObject_SetAngularVelocity = 0x1E08170;

    constexpr std::uintptr_t kFunc_CollisionObject_ApplyLinearImpulse = 0x1E08520;

    constexpr std::uintptr_t kFunc_CollisionObject_ApplyPointImpulse = 0x1E08640;

    constexpr std::uintptr_t kFunc_CollisionObject_SetMass = 0x1E08C00;

    constexpr std::uintptr_t kFunc_CollisionObject_GetFilterInfo = 0x1E08D60;

    constexpr std::uintptr_t kFunc_CollisionObject_GetCOMWorld = 0x1E08EF0;

    constexpr std::uintptr_t kFunc_CollisionObject_GetShape = 0x1E07F30;

    constexpr std::uintptr_t kFunc_IsBodyConstrained = 0x1E09170;

    constexpr std::uintptr_t kFunc_World_AddPhysicsSystem = 0x1DFAC30;

    constexpr std::uintptr_t kFunc_BhkWorld_RemovePhysicsSystemInstance = 0x1DFAD00;

    constexpr std::uintptr_t kFunc_HknpWorld_SetBodyMotion = 0x153BAE0;

    constexpr std::uintptr_t kFunc_HknpWorld_SetBodyMaterial = 0x153AFC0;

    constexpr std::uintptr_t kFunc_BhkWorld_SetMotionRecursive = 0x1DF95B0;

    constexpr std::uintptr_t kFunc_World_EnableCollision = 0x1DF9940;

    constexpr std::uintptr_t kFunc_World_PickObject = 0x1DF8D60;

    constexpr std::uintptr_t kFunc_World_AddStepListener = 0x1DFA7B0;

    constexpr std::uintptr_t kData_BhkWorldRawDeltaSeconds = 0x65A3D70;

    constexpr std::uintptr_t kData_BhkWorldSubstepDeltaSeconds = 0x65A3D74;

    constexpr std::uintptr_t kData_BhkWorldRemainderDeltaSeconds = 0x65A3D7C;

    constexpr std::uintptr_t kData_BhkWorldAccumulatedDeltaSeconds = 0x65A3D84;

    constexpr std::uintptr_t kData_BhkWorldSubstepCount = 0x65A3D8C;

    constexpr std::uintptr_t kData_BethesdaAllocatorPool = 0x392E400;

    constexpr std::uintptr_t kData_BethesdaAllocatorState = 0x392E880;

    constexpr std::uintptr_t kData_BethesdaTlsIndex = 0x689CACC;

    constexpr std::uintptr_t kBethesdaTlsAllocatorContext = 0x9C0;

    constexpr std::uintptr_t kFunc_BethesdaAlloc = 0x1B91950;

    constexpr std::uintptr_t kFunc_BethesdaAllocatorInit = 0x1B91DD0;

    constexpr std::uintptr_t kData_HavokTlsAllocKey = 0x5B63B20;

    constexpr std::uintptr_t kFunc_HkArray_ReserveMore = 0x155D820;

    constexpr std::uintptr_t kData_HkArrayAllocatorGlobal = 0x3866310;

    constexpr std::uintptr_t kFunc_MotionCinfo_Ctor = 0x17A2FC0;

    constexpr std::uintptr_t kData_HavokGameToHavokScale = 0x5A38628;

    constexpr std::uintptr_t kData_HavokToGameScale = 0x3718110;

    constexpr std::uintptr_t kData_VRScalePrimary = 0x5B29178;

    constexpr std::uintptr_t kData_RaycastResultScale = 0x37CEA5C;

    constexpr std::uintptr_t kFunc_EnableBodyFlags = 0x153C090;

    constexpr std::uintptr_t kFunc_DisableBodyFlags = 0x153C150;

    constexpr std::uintptr_t kFunc_ActivateBody = 0x1546EF0;

    constexpr std::uintptr_t kFunc_NiNode_Ctor = 0x1C17D30;

    constexpr std::uintptr_t kFunc_NiNode_Dtor = 0x1C17DD0;

    constexpr std::uintptr_t kFunc_NiNode_SetName = 0x1C16C30;

    constexpr std::uintptr_t kFunc_BSFixedString_Create = 0x1BC1650;

    constexpr std::uintptr_t kData_NiNode_Vtable = 0x2E57A68;

    constexpr std::size_t kNiNodeSize = 0x180;

    constexpr int kNiNodeAlignment = 0x10;

    constexpr int kSysData_Materials = 0x10;
    constexpr int kSysData_Array1 = 0x20;
    constexpr int kSysData_Array2 = 0x30;
    constexpr int kSysData_MotionCinfos = kSysData_Array2;
    constexpr int kSysData_BodyCinfos = 0x40;
    constexpr int kSysData_ConstraintInfos = 0x50;
    constexpr int kSysData_Shapes = 0x60;

    constexpr std::uintptr_t kFunc_World_CastRay = 0x15A6B10;

    constexpr std::uintptr_t kFunc_World_GetClosestPoints = 0x15A6DF0;

    constexpr std::uintptr_t kFunc_World_QueryAabb = 0x15A64B0;

    constexpr std::uintptr_t kFunc_World_QueryAabbBroadphaseOnly = 0x15A6330;

    constexpr std::uintptr_t kFunc_CreateSphereShape = 0x15FF4E0;

    constexpr std::uintptr_t kFunc_NativeVRGrabDrop = 0xF1AB90;

    /*
     * Equipped-weapon 3D attach task submission. Blind raw-disassembly
     * verification against Fallout4VR.exe 1.2.72 on 2026-07-22 confirmed
     * 0x140DAB8F0 submits task type 0x12 and retains both the actor and the
     * BGSObjectInstance payload. The task re-resolves and exact-compares the
     * actor's current form/instance before calling Actor::AttachWeapon, so a
     * stale recovery request fails closed after a later weapon switch.
     */
    constexpr std::uintptr_t kFunc_QueueEquippedWeaponAttach = 0xDAB8F0;
    constexpr std::uintptr_t kData_EquippedWeaponAttachManager = 0x5B279E0;

    constexpr std::uintptr_t kFunc_SetBodyMotionProperties = 0x153B2F0;

    constexpr std::uintptr_t kFunc_MotionPropertiesLibrary_AddEntry = 0x1767A70;

    constexpr std::uintptr_t kData_ConvexPolytopeVtable = 0x2C9A108;

    constexpr std::uintptr_t kFunc_MaterialLibrary_AddMaterial = 0x1537840;

    constexpr std::uintptr_t kFunc_GetConstraintInfoUtil = 0x1A4AD20;

    /*
     * Stock FO4VR constraint-data constructors and world-frame setters.
     * Blind Ghidra verification on 2026-07-23 established the exact object
     * sizes and setter ABIs used by TouchGrabRuntime:
     *   Ball-and-socket: 0x70
     *   Limited hinge:   0x130
     *   Prismatic:       0x120
     */
    constexpr std::uintptr_t kFunc_BallAndSocketConstraintData_Ctor = 0x19AF690;
    constexpr std::uintptr_t kFunc_BallAndSocketConstraintData_SetPivots = 0x19AF6E0;
    constexpr std::uintptr_t kFunc_LimitedHingeConstraintData_Ctor = 0x19ACA30;
    constexpr std::uintptr_t kFunc_LimitedHingeConstraintData_SetInWorldSpace = 0x19ACD90;
    constexpr std::uintptr_t kFunc_PrismaticConstraintData_Ctor = 0x19B1350;
    constexpr std::uintptr_t kFunc_PrismaticConstraintData_SetInWorldSpace = 0x19B1520;

    constexpr std::uintptr_t kFunc_HandleBumpedCharacter = 0x1E24980;

    constexpr std::uintptr_t kFunc_VRGrabInitiate = 0xF19250;

    constexpr std::uintptr_t kFunc_ProcessConstraintsCallback = 0x1E4B7E0;

    constexpr std::uintptr_t kFunc_WeaponSwingHandler_Handle = 0x0FEF820;

    constexpr std::uintptr_t kFunc_HitFrameHandler_Handle = 0x0FEFFB0;

    constexpr std::uintptr_t kFunc_AttackBlockHandler_ShouldHandleEvent = 0x0FCD770;

    constexpr std::uintptr_t kFunc_PlayerCharacter_WeaponSwingCallBack = 0x0F23E00;

    constexpr std::uintptr_t kFunc_VRMeleeImpactCallback = 0x0EFF000;

    constexpr std::uintptr_t kVtableEntry_WeaponSwingHandler_Handle = 0x2D8CA00;

    constexpr std::uintptr_t kVtableEntry_HitFrameHandler_Handle = 0x2D8CB98;

    constexpr std::uintptr_t kVtableEntry_AttackBlockHandler_ShouldHandleEvent = 0x2D8A350;

    constexpr std::uintptr_t kVtableEntry_PlayerCharacter_WeaponSwingCallBack = 0x2D817A8;

    constexpr std::uintptr_t kData_PlayerActorSingleton = 0x5A38518;

    constexpr std::uintptr_t kHookSite_MainLoop = 0xD8405E;

    /*
     * FO4VR's recurring first-person node-chain alignment helper. Blind raw
     * disassembly verification on 2026-07-17 confirmed the entry at
     * 0x140EF6280 and the paired calls at 0x140EF6108/0x140EF614B. In
     * right-handed mode the first call uses PlayerNodes +0x718 while the
     * second uses +0x790; module+0xEF610D is therefore the exact return site
     * for Bethesda's primary-hand pass and module+0xEF6150 is the exact
     * return site for the paired secondary/offhand pass. Its player-update caller at
     * module+0xD83F0F precedes the framework/FRIK main-loop injection at
     * module+0xD8405E in the same native routine, proving this snapshot is
     * taken before hFRIK's arm/weapon pass. ROCK validates the complete
     * position-independent 14-byte prologue before intercepting the helper.
     */
    constexpr std::uintptr_t kFunc_UpdateFirstPersonArm = 0xEF6280;
    constexpr std::uintptr_t kCallsite_UpdateFirstPersonArmPrimaryReturn = 0xEF610D;
    constexpr std::uintptr_t kCallsite_UpdateFirstPersonArmSecondaryReturn = 0xEF6150;
    constexpr std::uintptr_t kFunc_PlayerPostUpdateAnimationGraphManager = 0xF2F0A0;

    /*
     * FO4VR native-scope geometry boundary. Raw-disassembly verified
     * 2026-07-15: CALL at 0x140EF851F targets 0x140EFAA60 with ABI
     * (PlayerCharacter*, bool). The surrounding routine has already applied
     * the native menu/weapon/VATS gates; only its final cone decision is
     * replaced. The renderer accessor reads request byte +0x3 from the state
     * object at 0x146239340.
     */
    constexpr std::uintptr_t kHookSite_NativeScopeGeometryDecision = 0xEF851F;

    // PipboyInventoryMenu::DoSelectItem call to UseItem. Raw-disassembly
    // verified against Fallout4VR.exe 1.2.72; the runtime hook validates both
    // the E8 opcode and decoded destination before patching.
    constexpr std::uintptr_t kHookSite_PipboyInventoryUseItem = 0xB9CFBA;

    constexpr std::uintptr_t kFunc_PipboyInventoryUseItem = 0xB9B890;

    constexpr std::uintptr_t kFunc_PipboyInventoryUpdateData = 0xB99C30;
    constexpr std::uintptr_t kPatchSite_NativeScopePostDecisionTest = 0xEF8528;
    constexpr std::uintptr_t kFunc_NativeScopeStateTransition = 0xEFAA60;
    constexpr std::uintptr_t kFunc_NativeScopeRequestStateGet = 0x1D947B0;
    constexpr std::uintptr_t kData_NativeScopeRendererState = 0x6239340;

    /*
     * Native WSScope presentation setup skipped by the equip path when an
     * otherwise valid magnified optic omits WEAPON_FLAGS::kHasScope. The
     * configure entry consumes the ZOOM record's overlay index. ROCK validates
     * its prologue, the singleton pointer, and the singleton's primary vtable
     * before invoking it for an unflagged manual-scope generation.
     */
    constexpr std::uintptr_t kFunc_NativeWorldScopeConfigure = 0xC8DC60;
    constexpr std::uintptr_t kData_NativeWorldScopeSingleton = 0x5ACBF58;
    constexpr std::uintptr_t kData_NativeWorldScopePrimaryVtable = 0x2D68718;

    /*
     * BGSModelMaterialSwap application used by TryAttach3DRecurse immediately
     * after cloning an OMOD model. Manual physical-housing enrichment calls the
     * same entry so recovered geometry keeps the equipped instance's skins.
     */
    constexpr std::uintptr_t kFunc_ApplyOmodModelCustomization = 0x53CD0;

    /*
     * PlayerCharacter flag storage is independently witnessed in the scope
     * update at 0x140EF84AF and the state transition at 0x140EFAAF7. Bit 0x08
     * is the native force-true branch immediately before the cone decision and
     * must retain priority over ROCK's replacement geometry.
     */
    constexpr std::ptrdiff_t kPlayerCharacter_NativeScopeFlags = 0x12A1;
    constexpr std::uint8_t kPlayerCharacter_NativeScopeForceDecisionMask = 0x08;

    /* Engine Setting objects; the live float value is the first four bytes. */
    constexpr std::uintptr_t kSetting_HmdScopeOffsetX = 0x37CF468;
    constexpr std::uintptr_t kSetting_HmdScopeOffsetY = 0x37CF480;
    constexpr std::uintptr_t kSetting_HmdScopeOffsetZ = 0x37CF498;
    constexpr std::uintptr_t kSetting_HmdScopeAngleEnterDegrees = 0x37CF4F8;
    constexpr std::uintptr_t kSetting_HmdScopeAngleExitDegrees = 0x37CF528;
    constexpr std::uintptr_t kSetting_WeaponScopeAngleEnterDegrees = 0x37CF540;
    constexpr std::uintptr_t kSetting_WeaponScopeAngleExitDegrees = 0x37CF570;
    constexpr std::uintptr_t kSetting_WeaponScopeDistanceEnter = 0x37CF5A0;
    constexpr std::uintptr_t kSetting_WeaponScopeDistanceExit = 0x37CF5B8;
    constexpr std::uintptr_t kSetting_ScopeWeaponAngleWideningFactor = 0x37CF5E8;
    constexpr std::uintptr_t kSetting_ScopeWeaponAngleExponent = 0x37CF600;

    constexpr std::uintptr_t kData_CollisionFilterSingleton = 0x59429B8;

    constexpr std::uintptr_t kFunc_SubscribeContactEvent = 0x3B9E50;

    constexpr std::uintptr_t kFunc_UnsubscribeSignalCallback = 0x1725B70;

    constexpr std::uintptr_t kFunc_ExtractContactSignalPoints = 0x175C650;

    /*
     * BSReadWriteLock read-side pair used with kHknpWorld_AccessLock. Raw-disasm
     * verified 2026-07-13: LockForRead CAS-increments the reader count while no
     * writer bit (0x80000000) is set and is recursive for the writer-owning
     * thread; UnlockForRead decrements. 40+ engine callers share the pair.
     */
    constexpr std::uintptr_t kFunc_BSReadWriteLock_LockForRead = 0x1B932B0;

    constexpr std::uintptr_t kFunc_BSReadWriteLock_UnlockForRead = 0x1B93570;
}
