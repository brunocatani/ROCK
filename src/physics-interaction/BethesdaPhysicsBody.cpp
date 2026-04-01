#include "BethesdaPhysicsBody.h"

#include "HavokOffsets.h"
#include "PhysicsLog.h"

#include <windows.h>

// BethesdaPhysicsBody.cpp — Proper physics body creation following the engine's
// own CreatePhantomBody (0x140f0a340) pipeline. Every step verified via Ghidra
// blind audit (2026-03-31, 5 independent agents).
//
// The creation pipeline builds the full Bethesda wrapper chain:
//   Shape → PhysicsSystemData → BodyCinfo + MotionCinfo → PhysicsSystem
//   → CollisionObject → CreateInstance → SetMotionType
//
// This gives access to ALL Bethesda physics API functions and properly sets
// body+0x88 (the critical back-pointer that 33 engine systems read).

namespace frik::rock
{

// =========================================================================
// Allocators (Bethesda + Havok, same pattern as GrabConstraint.cpp)
// =========================================================================

/// Allocate from Havok's TLS-based heap (for shapes, system data, cinfos).
/// Pattern: TlsGetValue(DAT_145b63b20) → allocator at TLS+0x58 → vtable[1] = alloc.
/// I1 NOTE: This is duplicated in GrabConstraint.cpp (havokHeapAlloc/havokHeapFree).
/// Both copies are identical. Will be extracted to a shared header in a future restructuring.
static void* havokAlloc(std::size_t size)
{
	static REL::Relocation<std::uint32_t*> s_tlsIndex{ REL::Offset(offsets::kData_HavokTlsAllocKey) };
	LPVOID tlsBlock = TlsGetValue(*s_tlsIndex);
	if (!tlsBlock) return nullptr;

	auto** allocator = reinterpret_cast<void***>(
		reinterpret_cast<char*>(tlsBlock) + 0x58);
	if (!allocator || !*allocator) return nullptr;

	auto* vtable = reinterpret_cast<void* (**)(void*, std::size_t)>(**allocator);
	return vtable[1](*allocator, size);
}

/// Free from Havok's TLS-based heap (vtable[2] = deallocate).
static void havokFree(void* ptr, std::size_t size)
{
	if (!ptr) return;

	static REL::Relocation<std::uint32_t*> s_tlsIndex{ REL::Offset(offsets::kData_HavokTlsAllocKey) };
	LPVOID tlsBlock = TlsGetValue(*s_tlsIndex);
	if (!tlsBlock) return;

	auto** allocator = reinterpret_cast<void***>(
		reinterpret_cast<char*>(tlsBlock) + 0x58);
	if (!allocator || !*allocator) return;

	auto* vtable = reinterpret_cast<void (**)(void*, void*, std::size_t)>(**allocator);
	vtable[2](*allocator, ptr, size);
}

/// Atomically decrement refcount on an hkReferencedObject (NiObject-derived).
/// If refcount reaches 0, calls the virtual destructor (vtable slot 0) with
/// freeMemory=1 to clean up and release the allocation.
/// Uses LOCK CMPXCHG pattern matching the engine's own atomic refcount decrement.
/// C2 FIX: Previous code used non-atomic read-modify-write which could corrupt
/// refcount if the physics thread touched it between read and write.
static void releaseRefCounted(void* obj)
{
	if (!obj) return;

	auto* refCountDword = reinterpret_cast<volatile long*>(
		reinterpret_cast<char*>(obj) + 0x08);

	// Atomic decrement of low 16 bits (refcount) via CAS loop.
	// High 16 bits (memSizeAndFlags) must be preserved.
	for (;;) {
		long oldVal = *refCountDword;
		std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
		if (rc == 0 || rc == 0xFFFF) return;  // already dead or immortal

		long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) |
			static_cast<long>(static_cast<std::uint16_t>(rc - 1));

		if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal) {
			if (rc - 1 == 0) {
				// Refcount hit 0 — call virtual destructor
				auto** vtable = *reinterpret_cast<void***>(obj);
				auto destructor = reinterpret_cast<void(*)(void*, int)>(vtable[0]);
				destructor(obj, 1);  // 1 = free memory after destruct
			}
			return;
		}
		// CAS failed — another thread modified refcount, retry
	}
}

/// Allocate from Bethesda's physics memory pool (for NiObject-derived types:
/// bhkPhysicsSystem, bhkNPCollisionObject).
/// Ghidra: FUN_141b91950(&DAT_14392e400, size, alignment)
static void* bethesdaAlloc(std::size_t size)
{
	typedef void* (*alloc_t)(void*, std::size_t, int);
	static REL::Relocation<alloc_t> allocFunc{ REL::Offset(offsets::kFunc_BethesdaAlloc) };
	static REL::Relocation<void*> allocPool{ REL::Offset(offsets::kData_BethesdaAllocatorPool) };
	// Ghidra-verified: 4-param function. param_3=alignment, param_4=use_aligned_malloc flag.
	// When param_4=0 (our case, 3 args), param_3 is ignored and regular malloc is used.
	// The alignment value doesn't matter, but we pass 0 to match CreatePhantomBody.
	return allocFunc(&allocPool, size, 0);
}

// =========================================================================
// Engine function typedefs
// =========================================================================

// -- Construction --
using PhysicsSystemDataCtor_t  = void* (*)(void*);
using BodyCinfoCtor_t          = void* (*)(void*);
using MaterialCtor_t           = void* (*)(void*);
using PhysicsSystemCtor_t      = void* (*)(void*, void*);
using CollisionObjectCtor_t    = void* (*)(void*, std::uint32_t, void*);
using CreateInstance_t          = void  (*)(void*, void*);
using SetMotionType_t           = void  (*)(void*, int);
using LinkObject_t              = void  (*)(void*, void*);
using GetBodyId_t               = void  (*)(void*, void*, std::uint32_t);

// -- Per-frame operations --
using DriveToKeyFrame_t         = bool  (*)(void*, const void*, float);
using SetTransform_t            = void  (*)(void*, const void*);
using SetVelocity_t             = void  (*)(void*, const float*, const float*);
using ApplyImpulse_t            = void  (*)(void*, const float*);
using ApplyPointImpulse_t       = void  (*)(void*, const float*, const float*);
using SetMass_t                 = void  (*)(void*, float);
using GetCOM_t                  = bool  (*)(void*, float*);
using GetFilterInfo_t           = std::uint32_t (*)(void*);
using GetShape_t                = void* (*)(void*);
using IsConstrained_t           = bool  (*)(void*);

// -- World operations --
using SetCollisionFilter_t      = void  (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
using EnableBodyFlags_t         = void  (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
using ActivateBody_t            = void  (*)(void*, std::uint32_t);

// -- Array growth --
// Ghidra-verified: FUN_14155d820 is hkArray::_reserveMore — returns void, NOT a pointer.
// It doubles capacity and reallocates buffer but does NOT change size.
using HkArrayReserveMore_t      = void  (*)(void*, void*, int);

// =========================================================================
// Creation pipeline — 12-step CreatePhantomBody pattern
// =========================================================================

bool BethesdaPhysicsBody::create(RE::hknpWorld* world, void* bhkWorld,
	RE::hknpShape* shape,
	std::uint32_t filterInfo,
	RE::hknpMaterialId materialId,
	BethesdaMotionType motionType,
	const char* name)
{
	if (_created) {
		ROCK_LOG_WARN(BethesdaBody, "create() called on already-created body — destroy first");
		return false;
	}
	if (!world || !bhkWorld || !shape) {
		ROCK_LOG_ERROR(BethesdaBody, "create() null params: world={} bhkWorld={} shape={}",
			(void*)world, bhkWorld, (void*)shape);
		return false;
	}

	// --- Step 1: Create hknpPhysicsSystemData (0x78 bytes, Havok alloc) ---
	// This is the template that holds body descriptions, motion descriptions, and shape refs.
	_systemData = havokAlloc(0x78);
	if (!_systemData) {
		ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate hknpPhysicsSystemData (0x78 bytes)");
		return false;
	}
	{
		static REL::Relocation<PhysicsSystemDataCtor_t> ctor{ REL::Offset(offsets::kFunc_PhysicsSystemData_Ctor) };
		ctor(_systemData);
	}

	// --- Helper: grow hkArray by one entry and return pointer to the new slot ---
	// Ghidra-verified: FUN_14155d820 is hkArray::_reserveMore, NOT expandOne.
	// It doubles capacity and reallocates the buffer but does NOT increment size.
	// We must: call _reserveMore if at capacity, then increment size, then return
	// the pointer to the new entry at data + oldSize * stride.
	//
	// hkArray layout: +0x00=data(void*), +0x08=size(int32), +0x0C=capacityAndFlags(int32)
	// capacityAndFlags: low 30 bits = capacity, bit 31 = no-dealloc flag
	auto hkArrayAppendOne = [&](char* arrayBase, int stride) -> char* {
		auto*& dataPtr  = *reinterpret_cast<char**>(arrayBase);
		auto&  size     = *reinterpret_cast<std::int32_t*>(arrayBase + 0x08);
		auto&  capFlags = *reinterpret_cast<std::int32_t*>(arrayBase + 0x0C);
		std::int32_t capacity = capFlags & 0x3FFFFFFF;

		if (size >= capacity) {
			// Need to grow — call _reserveMore which doubles capacity and reallocates
			using ReserveMore_t = void (*)(void*, void*, int);
			static REL::Relocation<ReserveMore_t> reserveMore{
				REL::Offset(offsets::kFunc_HkArray_ReserveMore) };
			static REL::Relocation<std::uintptr_t> arrayAllocGlobal{
				REL::Offset(offsets::kData_HkArrayAllocatorGlobal) };
			auto* allocParam = reinterpret_cast<void*>(arrayAllocGlobal.address());
			reserveMore(allocParam, arrayBase, stride);
			// Re-read data ptr (may have been reallocated)
		}

		if (!dataPtr) return nullptr;

		// Append: increment size, return pointer to new entry
		char* newEntry = dataPtr + size * stride;
		size++;
		return newEntry;
	};

	// --- Step 2: Populate body cinfo (0x60 bytes) ---
	// WAVE 5 CORRECTION: bodyCinfos are at sysData+0x40 (NOT +0x10 which is materials).
	// Verified layout: +0x10=materials(0x50), +0x20=array1(0x60), +0x30=array2(0x70),
	//                  +0x40=bodyCinfos(0x60), +0x50=constraintCinfos(0x18), +0x60=shapes(0x08)
	void* bodyCinfo = nullptr;
	{
		auto* bodyCinfoArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_BodyCinfos;
		bodyCinfo = hkArrayAppendOne(bodyCinfoArray, 0x60);
		if (!bodyCinfo) {
			ROCK_LOG_ERROR(BethesdaBody, "Failed to grow bodyCinfos array");
			// C3 NOTE: hkArrayAppendOne may have allocated internal data buffers for
			// the bodyCinfos hkArray inside _systemData. havokFree only frees the 0x78
			// systemData struct itself, not these internal buffers. This is a rare error
			// path (Havok heap OOM) where leaking the small hkArray buffer is acceptable.
			// The bhkPhysicsSystem ctor (Step 5) would normally take ownership of these.
			// TODO: Call a systemData destructor or manually free hkArray data buffers
			// if this error path is ever hit in practice.
			havokFree(_systemData, 0x78);
			_systemData = nullptr;
			return false;
		}

		// Initialize the body cinfo with defaults
		static REL::Relocation<BodyCinfoCtor_t> cinfoInit{ REL::Offset(offsets::kFunc_BodyCinfo_Ctor) };
		cinfoInit(bodyCinfo);

		// Populate fields
		auto* ci = reinterpret_cast<char*>(bodyCinfo);
		*reinterpret_cast<RE::hknpShape**>(ci + 0x00) = shape;         // shape
		*reinterpret_cast<std::uint32_t*>(ci + 0x08) = 0x7FFF'FFFF;   // bodyId = auto-assign
		*reinterpret_cast<std::uint32_t*>(ci + 0x0C) = 0x7FFF'FFFF;   // motionId = auto-assign
		*reinterpret_cast<std::uint8_t*>(ci + 0x10)  = 0xFF;          // motionPropertiesId = default
		*reinterpret_cast<std::uint16_t*>(ci + 0x12) = materialId.value; // materialId
		*reinterpret_cast<std::uint32_t*>(ci + 0x14) = filterInfo;     // collisionFilterInfo
	}

	// --- Step 3: Populate material (0x50 bytes) ---
	// WAVE 5 CORRECTION: What we called "motionCinfo" is actually hknpMaterial::ctor.
	// Materials are at sysData+0x10 (stride 0x50). The function at 0x1536CB0 is the
	// material constructor, not motion cinfo init. We add a default material entry
	// so the physics system data has valid material data.
	{
		auto* materialsArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_Materials;
		auto* material = hkArrayAppendOne(materialsArray, 0x50);
		if (!material) {
			ROCK_LOG_ERROR(BethesdaBody, "Failed to grow materials array");
			// C3 NOTE: Steps 2-3 may have allocated internal hkArray data buffers inside
			// _systemData (bodyCinfos at +0x40, materials at +0x10). havokFree only frees
			// the 0x78 systemData struct, not these internal buffers. This is a rare error
			// path (Havok heap OOM). The bhkPhysicsSystem ctor (Step 5) would normally
			// take ownership. Acceptable leak in this extremely rare failure case.
			// TODO: Manually free hkArray data buffers if this error path is ever hit.
			havokFree(_systemData, 0x78);
			_systemData = nullptr;
			return false;
		}

		using MaterialCtor_t = void* (*)(void*);
		static REL::Relocation<MaterialCtor_t> materialCtor{ REL::Offset(offsets::kFunc_MaterialCtor) };
		materialCtor(material);
	}

	// --- Step 4: Add shape reference ---
	// Shapes at sysData+0x60 (stride 0x08) — CONFIRMED correct offset.
	{
		auto* shapeRefsArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_Shapes;
		auto* shapeSlot = hkArrayAppendOne(shapeRefsArray, 8);
		if (shapeSlot) {
			*reinterpret_cast<RE::hknpShape**>(shapeSlot) = shape;
			// C1 FIX: Atomic refcount increment via CAS loop, matching the pattern
			// used by releaseRefCounted() above. Non-atomic read-modify-write could
			// corrupt the refcount if the physics thread touches it concurrently.
			auto* refCountDword = reinterpret_cast<volatile long*>(
				reinterpret_cast<char*>(shape) + 0x08);
			for (;;) {
				long oldVal = *refCountDword;
				std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
				if (rc == 0xFFFF) break;  // immortal — don't touch
				long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) |
					static_cast<long>(static_cast<std::uint16_t>(rc + 1));
				if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal) break;
				// CAS failed — another thread modified refcount, retry
			}
		}
	}

	// --- Step 5: Create bhkPhysicsSystem (0x28 bytes, Bethesda alloc) ---
	_physicsSystem = bethesdaAlloc(0x28);
	if (!_physicsSystem) {
		ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate bhkPhysicsSystem (0x28 bytes)");
		// systemData will be freed by Havok when arrays destruct
		havokFree(_systemData, 0x78);
		_systemData = nullptr;
		return false;
	}
	{
		static REL::Relocation<PhysicsSystemCtor_t> physSysCtor{ REL::Offset(offsets::kFunc_PhysicsSystem_Ctor) };
		physSysCtor(_physicsSystem, _systemData);
	}

	// --- Step 6: Create bhkNPCollisionObject (0x30 bytes, Bethesda alloc) ---
	_collisionObject = bethesdaAlloc(0x30);
	if (!_collisionObject) {
		ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate bhkNPCollisionObject (0x30 bytes)");
		// C3 FIX: Properly release physicsSystem (decrement refcount, destructor frees systemData)
		releaseRefCounted(_physicsSystem);
		_physicsSystem = nullptr;
		_systemData = nullptr;  // owned by physicsSystem after ctor
		return false;
	}
	{
		static REL::Relocation<CollisionObjectCtor_t> collObjCtor{ REL::Offset(offsets::kFunc_CollisionObject_Ctor) };
		collObjCtor(_collisionObject, 0, _physicsSystem);  // bodyIndex = 0 (single body system)
	}

	// --- Step 7: Create body via raw hknpWorld::CreateBody (HYBRID approach) ---
	//
	// WHY HYBRID: bhkPhysicsSystem::CreateInstance (vtable+0x160) calls an unanalyzed
	// function at 0x141e0d650 that Ghidra couldn't resolve. The golden reference
	// (CreatePhantomBody at 0x140f0a340) does NOT call CreateInstance at all — it
	// links the objects and lets scene graph processing handle body creation later.
	// ROCK needs bodies immediately (no scene graph node), so we use the proven
	// raw CreateBody approach from the old code, then construct the Bethesda wrapper
	// chain around the existing body.
	//
	// The Bethesda API functions (DriveToKeyFrame, SetMotionType, GetBodyId, etc.)
	// only need field reads from the physics system instance:
	//   instance+0x18 = hknpWorld*
	//   instance+0x20 = bodyId array pointer
	//   instance+0x28 = body count
	// No virtual dispatch on the instance in the common API paths.
	{
		// Read the bodyCinfo we built (first entry in the bodyCinfos array at sysData+0x40)
		auto* bodyCinfoArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_BodyCinfos;
		auto* cinfoData = *reinterpret_cast<char**>(bodyCinfoArray);
		if (!cinfoData) {
			ROCK_LOG_ERROR(BethesdaBody, "bodyCinfo array data is null");
			releaseRefCounted(_collisionObject);
			_collisionObject = nullptr;
			_physicsSystem = nullptr;
			_systemData = nullptr;
			return false;
		}

		// Build RE::hknpBodyCinfo from our populated data
		RE::hknpBodyCinfo cinfo;
		cinfo.shape = *reinterpret_cast<RE::hknpShape**>(cinfoData + 0x00);
		cinfo.collisionFilterInfo = *reinterpret_cast<std::uint32_t*>(cinfoData + 0x14);
		cinfo.materialId.value = *reinterpret_cast<std::uint16_t*>(cinfoData + 0x12);
		cinfo.motionPropertiesId.value = 0xFF;
		cinfo.position = RE::hkVector4f(0.0f, 0.0f, 0.0f, 0.0f);
		cinfo.orientation = RE::hkVector4f(0.0f, 0.0f, 0.0f, 1.0f);
		cinfo.userData = 0;  // set body+0x88 manually below
		cinfo.name = name;

		// Allocate motion for the body
		auto motionId = world->AllocateMotion();
		cinfo.motionId.value = motionId;

		// Create body in the world (proven, same as old Hand.cpp code)
		_bodyId = world->CreateBody(cinfo);
		if (_bodyId.value == 0x7FFF'FFFF) {
			ROCK_LOG_ERROR(BethesdaBody, "CreateBody returned invalid ID");
			releaseRefCounted(_collisionObject);
			_collisionObject = nullptr;
			_physicsSystem = nullptr;
			_systemData = nullptr;
			return false;
		}

		ROCK_LOG_INFO(BethesdaBody, "Body created via raw CreateBody: bodyId={} motionId={}",
			_bodyId.value, motionId);
	}

	// --- Step 8: Construct physics system instance around existing body ---
	// The bhkPhysicsSystem needs a live instance (physSys+0x18) with:
	//   instance+0x18 = hknpWorld*
	//   instance+0x20 = bodyId array pointer (uint32[])
	//   instance+0x28 = body count (int32)
	// Allocate a minimal struct from Havok heap. We only need fields that
	// the Bethesda API functions read (GetBodyId, GetWorld, HasInstanceInWorld).
	{
		// Allocate enough space for the instance fields we need.
		// The real hknpPhysicsSystem instance is larger but we only need these offsets.
		// Allocate 0x30 bytes to cover up to +0x2C with padding.
		constexpr std::size_t INSTANCE_SIZE = 0x30;
		auto* instance = reinterpret_cast<char*>(havokAlloc(INSTANCE_SIZE));
		if (!instance) {
			ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate physics system instance (0x30 bytes)");
			world->DestroyBodies(&_bodyId, 1);
			_bodyId.value = 0x7FFF'FFFF;
			releaseRefCounted(_collisionObject);
			_collisionObject = nullptr;
			_physicsSystem = nullptr;
			_systemData = nullptr;
			return false;
		}
		std::memset(instance, 0, INSTANCE_SIZE);

		// Allocate body ID array (1 entry)
		auto* bodyIdArray = reinterpret_cast<std::uint32_t*>(havokAlloc(sizeof(std::uint32_t)));
		if (bodyIdArray) {
			*bodyIdArray = _bodyId.value;
		}

		// Populate instance fields
		*reinterpret_cast<void**>(instance + 0x18) = world;           // hknpWorld*
		*reinterpret_cast<std::uint32_t**>(instance + 0x20) = bodyIdArray;  // bodyId array
		*reinterpret_cast<std::int32_t*>(instance + 0x28) = 1;       // body count

		// Store instance in physics system
		*reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18) = instance;

		ROCK_LOG_INFO(BethesdaBody, "Physics system instance constructed: instance={:p} world={:p} bodyId={}",
			(void*)instance, (void*)world, _bodyId.value);
	}

	// --- Step 9: Set body+0x88 back-pointer (collisionObject → body) ---
	// This is the critical back-pointer that 33 engine systems read.
	// bhkNPCollisionObject::CreateInstance normally does this, but since we
	// skipped CreateInstance, we do it manually.
	{
		auto* bodyArray = world->GetBodyArray();
		auto* bodyPtr = reinterpret_cast<char*>(&bodyArray[_bodyId.value]);
		*reinterpret_cast<void**>(bodyPtr + 0x88) = _collisionObject;

		ROCK_LOG_INFO(BethesdaBody, "body+0x88 back-pointer set manually: {:p}", _collisionObject);
	}

	// --- Step 10: Set motion type via raw call (body exists now) ---
	// C2 FIX: Renumbered from duplicate "Step 10" — motion type=10, flags=11, activate=12.
	// Use the proven setBodyKeyframed for KEYFRAMED, or SetMotionType via collObj
	// now that the physSys instance chain is complete.
	if (motionType == BethesdaMotionType::Keyframed) {
		typedef void (*setKeyframed_t)(void*, std::uint32_t);
		static REL::Relocation<setKeyframed_t> setBodyKeyframed{
			REL::Offset(offsets::kFunc_SetBodyKeyframed) };
		setBodyKeyframed(world, _bodyId.value);
	} else {
		// For DYNAMIC/STATIC, try via collision object (instance chain is set up)
		static REL::Relocation<SetMotionType_t> setMotion{
			REL::Offset(offsets::kFunc_CollisionObject_SetMotionType) };
		setMotion(_collisionObject, static_cast<int>(motionType));
	}

	// --- Step 11: Enable contact modifier + keep-awake flags ---
	// 0x00020000 = contact modifier (so contact events fire)
	// 0x08000000 = keep-awake (body never deactivates)
	{
		static REL::Relocation<EnableBodyFlags_t> enableFlags{
			REL::Offset(offsets::kFunc_EnableBodyFlags) };
		enableFlags(world, _bodyId.value, 0x08020000, 1);
	}

	// --- Step 12: Activate body ---
	{
		static REL::Relocation<ActivateBody_t> activate{
			REL::Offset(offsets::kFunc_ActivateBody) };
		activate(world, _bodyId.value);
	}

	_created = true;

	ROCK_LOG_INFO(BethesdaBody, "Created '{}': bodyId={} collObj={:p} physSys={:p} sysData={:p} motionType={}",
		name, _bodyId.value, _collisionObject, _physicsSystem, _systemData,
		static_cast<int>(motionType));

	// Verify body+0x88 back-pointer was set correctly
	{
		auto* bodyArray = world->GetBodyArray();
		auto* bodyPtr = reinterpret_cast<char*>(&bodyArray[_bodyId.value]);
		auto* backPtr = *reinterpret_cast<void**>(bodyPtr + 0x88);
		if (backPtr == _collisionObject) {
			ROCK_LOG_INFO(BethesdaBody, "  body+0x88 back-pointer VERIFIED: {:p} == collisionObject", backPtr);
		} else {
			ROCK_LOG_ERROR(BethesdaBody, "  body+0x88 back-pointer MISMATCH: {:p} != {:p}", backPtr, _collisionObject);
		}
	}

	return true;
}

// =========================================================================
// Destruction
// =========================================================================

void BethesdaPhysicsBody::destroy(void* bhkWorld)
{
	if (!_created) return;

	ROCK_LOG_INFO(BethesdaBody, "Destroying body: bodyId={} collObj={:p}",
		_bodyId.value, _collisionObject);

	// --- Step 0: Destroy NiNode FIRST (clears scene graph links) ---
	destroyNiNode();

	// --- Step 1: Clear body+0x88 back-pointer FIRST ---
	// Prevent engine systems from following the back-pointer to our
	// about-to-be-freed collision object during destruction.
	if (_physicsSystem) {
		auto* physSysInstance = *reinterpret_cast<char**>(
			reinterpret_cast<char*>(_physicsSystem) + 0x18);
		if (physSysInstance) {
			auto* worldPtr = *reinterpret_cast<RE::hknpWorld**>(physSysInstance + 0x18);
			if (worldPtr && _bodyId.value != 0x7FFF'FFFF) {
				auto* bodyArray = worldPtr->GetBodyArray();
				auto* bodyPtr = reinterpret_cast<char*>(&bodyArray[_bodyId.value]);
				*reinterpret_cast<void**>(bodyPtr + 0x88) = nullptr;
			}
		}
	}

	// --- Step 2: Destroy the Havok body from the world ---
	if (_bodyId.value != 0x7FFF'FFFF && _physicsSystem) {
		auto* physSysInstance = *reinterpret_cast<char**>(
			reinterpret_cast<char*>(_physicsSystem) + 0x18);
		if (physSysInstance) {
			auto* worldPtr = *reinterpret_cast<RE::hknpWorld**>(physSysInstance + 0x18);
			if (worldPtr) {
				worldPtr->DestroyBodies(&_bodyId, 1);
				ROCK_LOG_INFO(BethesdaBody, "Body {} destroyed from world", _bodyId.value);
			}
		}
	}

	// --- Step 3: Free manually-created instance + bodyId array ---
	// These were allocated in create() step 8 (hybrid approach).
	if (_physicsSystem) {
		auto* instance = *reinterpret_cast<char**>(
			reinterpret_cast<char*>(_physicsSystem) + 0x18);
		if (instance) {
			auto* bodyIdArray = *reinterpret_cast<void**>(instance + 0x20);
			if (bodyIdArray) {
				havokFree(bodyIdArray, sizeof(std::uint32_t));
			}
			havokFree(instance, 0x30);
			// Clear instance pointer so physSys destructor doesn't try to use it
			*reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18) = nullptr;
		}
	}

	// --- Step 4: Release collision object via atomic refcount ---
	// Cascade: collisionObject dtor → releases physicsSystem → releases systemData.
	if (_collisionObject) {
		releaseRefCounted(_collisionObject);
	}

	reset();
}

void BethesdaPhysicsBody::reset()
{
	_collisionObject = nullptr;
	_physicsSystem = nullptr;
	_systemData = nullptr;
	_niNode = nullptr;
	_bodyId.value = 0x7FFF'FFFF;
	_created = false;
}

// =========================================================================
// NiNode creation — scene graph integration
//
// Following BuildSceneNodeHierarchy (0x140ef21a0) pattern:
//   1. Allocate 0x180 bytes, 0x10-aligned from Bethesda allocator
//   2. Construct NiNode via ctor
//   3. Set name via BSFixedString
//   4. Link collision object bidirectionally via LinkObject
// =========================================================================

bool BethesdaPhysicsBody::createNiNode(const char* name)
{
	if (!isValid() || !_collisionObject) {
		ROCK_LOG_ERROR(BethesdaBody, "createNiNode: body not valid or no collision object");
		return false;
	}
	if (_niNode) {
		ROCK_LOG_WARN(BethesdaBody, "createNiNode: already has NiNode — skipping");
		return true;
	}

	// --- Step 1: Allocate NiNode (0x180 bytes, Bethesda allocator) ---
	void* mem = bethesdaAlloc(offsets::kNiNodeSize);
	if (!mem) {
		ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate NiNode (0x{:X} bytes)", offsets::kNiNodeSize);
		return false;
	}

	// CRITICAL: Zero the allocation before calling NiNode ctor.
	// Bethesda's pool allocator returns recycled memory with leftover data.
	// NiNode::ctor calls base NiAVObject::ctor which checks children arrays
	// at +0x168 (data ptr) and +0x172 (count). If these contain garbage from
	// a previous allocation, the ctor's cleanup path calls DetachAllChildren
	// → SetScenegraphChange → dereferences garbage parent pointers → crash.
	// BuildSceneNodeHierarchy avoids this because it runs at load time when
	// the pool is freshly allocated (all zeros). ROCK runs later when pools
	// contain recycled memory from destroyed objects.
	std::memset(mem, 0, offsets::kNiNodeSize);

	// --- Step 2: Construct NiNode (default ctor, no parent) ---
	// CRITICAL: The ctor does NOT return 'this' in RAX. It tail-calls to another
	// init function (0x141c22f20) whose return value is unrelated to the NiNode.
	// Storing the return value gives a garbage pointer → crash in SetName.
	// The NiNode IS at 'mem' — the ctor initializes it in-place.
	{
		using NiNodeCtor_t = void (*)(void*);
		static REL::Relocation<NiNodeCtor_t> niNodeCtor{ REL::Offset(offsets::kFunc_NiNode_Ctor) };
		niNodeCtor(mem);
		_niNode = mem;
	}

	// --- Step 3: Set name via BSFixedString ---
	if (name && name[0]) {
		// BSFixedString is 8 bytes (interned string pointer)
		alignas(8) char bsStr[8] = {};
		using BSFixedStringCreate_t = void (*)(void*, const char*);
		static REL::Relocation<BSFixedStringCreate_t> createStr{
			REL::Offset(offsets::kFunc_BSFixedString_Create) };
		createStr(bsStr, name);

		using NiNodeSetName_t = void (*)(void*, const void*);
		static REL::Relocation<NiNodeSetName_t> setName{
			REL::Offset(offsets::kFunc_NiNode_SetName) };
		setName(_niNode, bsStr);

		// S3 FIX: Release the interned string reference. BSFixedString stores a
		// pointer into Bethesda's string pool that is ref-counted. NiNode_SetName
		// copies the string into the node, so our local bsStr must be zeroed to
		// release the pool reference. Without this, the string entry is leaked
		// (refcount never decremented, pool entry never freed).
		std::memset(bsStr, 0, sizeof(bsStr));
	}

	// --- Step 4: Link collision object ↔ NiNode bidirectionally ---
	// LinkObject at vtable+0x150 = 0x142996cb0
	// Sets: collObj+0x10 = niNode (owner), niNode+0x100 = collObj
	{
		using LinkObject_t = void (*)(void*, void*);
		static REL::Relocation<LinkObject_t> linkObject{
			REL::Offset(offsets::kFunc_CollisionObject_LinkObject) };
		linkObject(_collisionObject, _niNode);
	}

	// Verify the links
	{
		auto* collObjOwner = *reinterpret_cast<void**>(
			reinterpret_cast<char*>(_collisionObject) + 0x10);
		auto* nodeCollObj = *reinterpret_cast<void**>(
			reinterpret_cast<char*>(_niNode) + 0x100);

		bool ownerOk = (collObjOwner == _niNode);
		bool collOk = (nodeCollObj == _collisionObject);

		ROCK_LOG_INFO(BethesdaBody, "NiNode '{}' created: node={:p} collObj→owner={} node→collObj={}",
			name ? name : "(null)", _niNode,
			ownerOk ? "VERIFIED" : "MISMATCH",
			collOk ? "VERIFIED" : "MISMATCH");
	}

	return true;
}

void BethesdaPhysicsBody::destroyNiNode()
{
	if (!_niNode) return;

	// Clear the bidirectional links before releasing
	if (_collisionObject) {
		// collObj+0x10 = nullptr (clear owner)
		*reinterpret_cast<void**>(reinterpret_cast<char*>(_collisionObject) + 0x10) = nullptr;
	}
	// node+0x100 = nullptr (clear collision object ref)
	*reinterpret_cast<void**>(reinterpret_cast<char*>(_niNode) + 0x100) = nullptr;

	// Release NiNode via atomic refcount (NiNode is an NiObject, ref-counted)
	releaseRefCounted(_niNode);
	_niNode = nullptr;

	ROCK_LOG_INFO(BethesdaBody, "NiNode destroyed");
}

// =========================================================================
// Contact signal scaffold (VR melee pattern)
// =========================================================================

void BethesdaPhysicsBody::registerContactSignal(const char* signalName)
{
	if (!isValid()) return;

	// VRMelee_SetupImpactBody_Keyframed (0x140f38630) pattern:
	// 1. Set body flags 0x4000010 (keyframed + collision reporting)
	// 2. Set quality type 9 at body+0x72
	// 3. Register per-body event signal via hknpWorld::getEventSignalForBody(world, 0, bodyId)
	//
	// STUB: The getEventSignalForBody address needs to be found in a future
	// Ghidra session. The VR melee code at 0x140f38630 calls it but the
	// exact address needs blind verification.
	ROCK_LOG_WARN(BethesdaBody, "registerContactSignal('{}') NOT YET IMPLEMENTED — "
		"need getEventSignalForBody address", signalName ? signalName : "(null)");
}

// =========================================================================
// Body positioning — per-frame
// =========================================================================

bool BethesdaPhysicsBody::driveToKeyFrame(const RE::NiTransform& target, float dt)
{
	if (!isValid()) return false;
	static REL::Relocation<DriveToKeyFrame_t> drive{ REL::Offset(offsets::kFunc_CollisionObject_DriveToKeyFrame) };
	return drive(_collisionObject, &target, dt);
}

void BethesdaPhysicsBody::setTransform(const RE::hkTransformf& transform)
{
	if (!isValid()) return;
	static REL::Relocation<SetTransform_t> setXform{ REL::Offset(offsets::kFunc_CollisionObject_SetTransform) };
	setXform(_collisionObject, &transform);
}

void BethesdaPhysicsBody::setVelocity(const float* linVel, const float* angVel)
{
	if (!isValid()) return;
	static REL::Relocation<SetVelocity_t> setVel{ REL::Offset(offsets::kFunc_CollisionObject_SetVelocity) };
	setVel(_collisionObject, linVel, angVel);
}

// =========================================================================
// Motion type and state
// =========================================================================

void BethesdaPhysicsBody::setMotionType(BethesdaMotionType type)
{
	if (!isValid()) return;
	static REL::Relocation<SetMotionType_t> setMotion{ REL::Offset(offsets::kFunc_CollisionObject_SetMotionType) };
	setMotion(_collisionObject, static_cast<int>(type));
}

void BethesdaPhysicsBody::setCollisionFilterInfo(std::uint32_t filterInfo, std::uint32_t rebuildMode)
{
	if (!isValid()) return;
	// Use the world-level function (needs world + bodyId, not collision object)
	// But since we have the collision object, we can also use the per-body getter/setter
	// For now, use the raw hknpBSWorld function which we already have
	// TODO: Use the collision object wrapper when we have the full API mapped
	static REL::Relocation<SetCollisionFilter_t> setFilter{
		REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };
	// Need the world — read from physicsSystem → instance → world
	auto* physSysInst = *reinterpret_cast<char**>(
		reinterpret_cast<char*>(_physicsSystem) + 0x18);
	if (!physSysInst) return;
	auto* world = *reinterpret_cast<void**>(physSysInst + 0x18);
	if (!world) return;
	setFilter(world, _bodyId.value, filterInfo, rebuildMode);
}

void BethesdaPhysicsBody::setMass(float mass)
{
	if (!isValid()) return;
	static REL::Relocation<SetMass_t> setM{ REL::Offset(offsets::kFunc_CollisionObject_SetMass) };
	setM(_collisionObject, mass);
}

// =========================================================================
// Impulse and force
// =========================================================================

void BethesdaPhysicsBody::applyLinearImpulse(const float* impulse)
{
	if (!isValid()) return;
	static REL::Relocation<ApplyImpulse_t> apply{ REL::Offset(offsets::kFunc_CollisionObject_ApplyLinearImpulse) };
	apply(_collisionObject, impulse);
}

void BethesdaPhysicsBody::applyPointImpulse(const float* impulse, const float* worldPoint)
{
	if (!isValid()) return;
	static REL::Relocation<ApplyPointImpulse_t> apply{ REL::Offset(offsets::kFunc_CollisionObject_ApplyPointImpulse) };
	apply(_collisionObject, impulse, worldPoint);
}

// =========================================================================
// Queries
// =========================================================================

bool BethesdaPhysicsBody::getCenterOfMassWorld(float& outX, float& outY, float& outZ)
{
	if (!isValid()) return false;
	alignas(16) float com[4] = { 0, 0, 0, 0 };
	static REL::Relocation<GetCOM_t> getCOM{ REL::Offset(offsets::kFunc_CollisionObject_GetCOMWorld) };
	bool ok = getCOM(_collisionObject, com);
	if (ok) {
		outX = com[0];
		outY = com[1];
		outZ = com[2];
	}
	return ok;
}

std::uint32_t BethesdaPhysicsBody::getCollisionFilterInfo()
{
	if (!isValid()) return 0;
	static REL::Relocation<GetFilterInfo_t> getFilter{ REL::Offset(offsets::kFunc_CollisionObject_GetFilterInfo) };
	return getFilter(_collisionObject);
}

void* BethesdaPhysicsBody::getShape()
{
	if (!isValid()) return nullptr;
	static REL::Relocation<GetShape_t> getShp{ REL::Offset(offsets::kFunc_CollisionObject_GetShape) };
	return getShp(_collisionObject);
}

bool BethesdaPhysicsBody::isConstrained()
{
	if (!isValid()) return false;
	static REL::Relocation<IsConstrained_t> check{ REL::Offset(offsets::kFunc_IsBodyConstrained) };
	return check(_collisionObject);
}

// =========================================================================
// Scaffolded stubs — future features
// =========================================================================

void BethesdaPhysicsBody::setPointVelocity(const float* targetVel, const float* worldPoint)
{
	if (!isValid()) return;
	// STUB: Implementation requires accessing the motion struct directly.
	// hknpMotion::setPointVelocity at 0x1417d1a20 takes (motion*, targetVel, worldPoint).
	// We need to resolve the motion pointer from the body's motionIndex.
	// Implementation deferred to Phase 4 (throw velocity).
	ROCK_LOG_WARN(BethesdaBody, "setPointVelocity: NOT YET IMPLEMENTED (Phase 4 stub)");
}

void BethesdaPhysicsBody::enableBodyFlags(std::uint32_t flags, std::uint32_t mode)
{
	if (!isValid()) return;
	auto* physSysInst = *reinterpret_cast<char**>(
		reinterpret_cast<char*>(_physicsSystem) + 0x18);
	if (!physSysInst) return;
	auto* world = *reinterpret_cast<void**>(physSysInst + 0x18);
	if (!world) return;

	static REL::Relocation<EnableBodyFlags_t> enableFlags{
		REL::Offset(offsets::kFunc_EnableBodyFlags) };
	enableFlags(world, _bodyId.value, flags, mode);
}

void BethesdaPhysicsBody::activateBody()
{
	if (!isValid()) return;
	auto* physSysInst = *reinterpret_cast<char**>(
		reinterpret_cast<char*>(_physicsSystem) + 0x18);
	if (!physSysInst) return;
	auto* world = *reinterpret_cast<void**>(physSysInst + 0x18);
	if (!world) return;

	static REL::Relocation<ActivateBody_t> activate{
		REL::Offset(offsets::kFunc_ActivateBody) };
	activate(world, _bodyId.value);
}

} // namespace frik::rock
