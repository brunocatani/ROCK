#pragma once

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

    constexpr std::uintptr_t kBody_CollisionFilterInfo = 0x44;

    constexpr std::uintptr_t kBody_CollisionObjectBackPointer = 0x88;

    constexpr std::uintptr_t kMotion_PropertiesId = 0x38;

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

    constexpr std::uintptr_t kFunc_SetBodyMotionProperties = 0x153B2F0;

    constexpr std::uintptr_t kData_ConvexPolytopeVtable = 0x2C9A108;

    constexpr std::uintptr_t kFunc_MaterialLibrary_AddMaterial = 0x1537840;

    constexpr std::uintptr_t kFunc_GetConstraintInfoUtil = 0x1A4AD20;

    constexpr std::uintptr_t kFunc_HandleBumpedCharacter = 0x1E24980;

    constexpr std::uintptr_t kFunc_VRGrabInitiate = 0xF19250;

    constexpr std::uintptr_t kFunc_ProcessConstraintsCallback = 0x1E4B7E0;

    constexpr std::uintptr_t kFunc_WeaponSwingHandler_Handle = 0x0FEF820;

    constexpr std::uintptr_t kFunc_HitFrameHandler_Handle = 0x0FEFFB0;

    constexpr std::uintptr_t kVtableEntry_WeaponSwingHandler_Handle = 0x2D8CA00;

    constexpr std::uintptr_t kVtableEntry_HitFrameHandler_Handle = 0x2D8CB98;

    constexpr std::uintptr_t kHookSite_MainLoop = 0xD8405E;

    constexpr std::uintptr_t kData_CollisionFilterSingleton = 0x59429B8;

    constexpr std::uintptr_t kFunc_SubscribeContactEvent = 0x3B9E50;

    constexpr std::uintptr_t kFunc_UnsubscribeSignalCallback = 0x1725B70;

    constexpr std::uintptr_t kFunc_ExtractContactSignalPoints = 0x175C650;
}
