// WeaponCollision.cpp — Equipped weapon physics collision body.
//
// Creates a KEYFRAMED body that mirrors the equipped weapon's collision shape
// on layer 44. Follows HIGGS weaponBody pattern (hand.cpp:720-791).
// Clones Hand.h::createCollision/updateCollisionTransform patterns for FRIK.

#include "WeaponCollision.h"

#include "HavokOffsets.h"
#include "RockConfig.h"

#include "RE/Bethesda/bhkPhysicsSystem.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hkReferencedObject.h"

#include "f4vr/PlayerNodes.h"

namespace frik::rock
{
	// =========================================================================
	// Shape ref counting helpers
	// =========================================================================

	// hknpShape inherits from hkReferencedObject. memSizeAndRefCount at +0x08:
	//   bits 0-15 = reference count, bits 16-30 = memory size, bit 31 = flag.
	// We manipulate the refcount directly because CommonLibF4VR doesn't expose
	// addReference/removeReference methods.

	// C4 FIX: Only modify the lower 16 bits (refcount), preserving upper bits
	// (memory size + flags). Previous code did += 1 on the full uint32, which
	// corrupts the memory size field if refcount overflows past 0xFFFF.
	// Also skip if refcount is 0xFFFF (immortal sentinel — should not be touched).
	static void shapeAddRef(const RE::hknpShape* shape) {
		if (!shape) return;
		auto* refObj = const_cast<RE::hkReferencedObject*>(
			static_cast<const RE::hkReferencedObject*>(shape));
		std::uint32_t val = refObj->memSizeAndRefCount;
		std::uint16_t refCount = static_cast<std::uint16_t>(val & 0xFFFF);
		if (refCount == 0xFFFF) return;  // immortal — don't touch
		refObj->memSizeAndRefCount = (val & 0xFFFF0000u) | static_cast<std::uint32_t>(refCount + 1);
	}

	static void shapeRemoveRef(const RE::hknpShape* shape) {
		if (!shape) return;
		auto* refObj = const_cast<RE::hkReferencedObject*>(
			static_cast<const RE::hkReferencedObject*>(shape));
		std::uint32_t val = refObj->memSizeAndRefCount;
		std::uint16_t refCount = static_cast<std::uint16_t>(val & 0xFFFF);
		if (refCount == 0xFFFF || refCount == 0) return;  // immortal or already zero
		refObj->memSizeAndRefCount = (val & 0xFFFF0000u) | static_cast<std::uint32_t>(refCount - 1);
		// Note: we don't delete on zero — the engine manages shape memory.
		// We just need to keep the refcount > 0 while we hold a pointer.
	}

	// =========================================================================
	// Init / Shutdown
	// =========================================================================

	void WeaponCollision::init(RE::hknpWorld* world)
	{
		if (!g_rockConfig.rockWeaponCollisionEnabled) {
			ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via config — skipping init");
			return;
		}
		_cachedWorld = world;
		_cachedWeaponKey = 0;
		_weaponBodyPending = false;
		_retryCounter = 0;
		_hasPrevTransform = false;
		ROCK_LOG_INFO(Weapon, "WeaponCollision initialized");
	}

	void WeaponCollision::shutdown()
	{
		// Don't call DestroyBodies — world may already be gone during cell transition.
		_weaponBodyIdAtomic.store(INVALID_BODY_ID, std::memory_order_release);
		_weaponBodyId.value = INVALID_BODY_ID;

		if (_cachedShape) {
			shapeRemoveRef(_cachedShape);
			_cachedShape = nullptr;
		}

		_cachedWeaponKey = 0;
		_cachedWorld = nullptr;
		_hasPrevTransform = false;
		_weaponBodyPending = false;
		_retryCounter = 0;
		_dominantHandDisabled = false;
		_disabledHandBodyId.value = INVALID_BODY_ID;

		ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown");
	}

	// =========================================================================
	// Per-Frame Update
	// =========================================================================

	void WeaponCollision::update(RE::hknpWorld* world, RE::NiAVObject* weaponNode,
		RE::hknpBodyId dominantHandBodyId, float dt)
	{
		if (!g_rockConfig.rockWeaponCollisionEnabled) return;
		if (!world) return;

		// Detect world change (power armor entry/exit can recreate hknpWorld)
		if (world != _cachedWorld) {
			ROCK_LOG_INFO(Weapon, "hknpWorld changed — resetting weapon collision state");
			_weaponBodyIdAtomic.store(INVALID_BODY_ID, std::memory_order_release);
			_weaponBodyId.value = INVALID_BODY_ID;
			if (_cachedShape) {
				shapeRemoveRef(_cachedShape);
				_cachedShape = nullptr;
			}
			_cachedWeaponKey = 0;
			_hasPrevTransform = false;
			_weaponBodyPending = false;
			_cachedWorld = world;
		}

		// --- Equip detection (per-frame FormID comparison) ---
		std::uint64_t currentKey = getEquippedWeaponKey();

		if (currentKey != _cachedWeaponKey) {
			ROCK_LOG_INFO(Weapon, "Weapon changed: {:016X} -> {:016X}", _cachedWeaponKey, currentKey);

			if (hasWeaponBody()) {
				destroyWeaponBody(world);
			}

			_cachedWeaponKey = currentKey;
			_weaponBodyPending = (currentKey != 0);
		}

		// --- Deferred body creation (retry if weapon 3D wasn't ready last frame) ---
		if (_weaponBodyPending && weaponNode) {
			// Throttle retries — only try once per second to avoid log spam
			if (++_retryCounter >= 90) {
				_retryCounter = 0;
				auto* shape = findWeaponShape(weaponNode, world);
				if (shape) {
					createWeaponBody(world, shape, weaponNode->world);
					_weaponBodyPending = false;
				}
			} else if (_retryCounter == 1) {
				// First frame after weapon change — try immediately
				auto* shape = findWeaponShape(weaponNode, world);
				if (shape) {
					createWeaponBody(world, shape, weaponNode->world);
					_weaponBodyPending = false;
					_retryCounter = 0;
				}
			}
		}

		// --- Handle weapon holster (node goes null while FormID unchanged) ---
		if (!weaponNode && hasWeaponBody()) {
			ROCK_LOG_INFO(Weapon, "Weapon node gone (holstered?) — destroying weapon body");
			destroyWeaponBody(world);
		}

		// --- Dominant hand collision management ---
		// Disable dominant hand collider when weapon body exists (prevents overlap spam).
		// Re-enable when weapon body is destroyed.
		if (hasWeaponBody() && !_dominantHandDisabled && dominantHandBodyId.value != INVALID_BODY_ID) {
			disableDominantHandCollision(world, dominantHandBodyId);
		} else if (!hasWeaponBody() && _dominantHandDisabled) {
			enableDominantHandCollision(world);
		}

		// --- Per-frame body positioning ---
		if (hasWeaponBody() && weaponNode) {
			updateBodyTransform(world, weaponNode->world, dt);
		}
	}

	// =========================================================================
	// Equip Detection
	// =========================================================================

	std::uint64_t WeaponCollision::getEquippedWeaponKey() const
	{
		// Use F4VR-CommonFramework's weapon detection.
		// Actor_GetCurrentWeapon at REL::Offset(0xe50da0) returns the equipped weapon form.
		// For simplicity, we use the weapon node pointer as a change-detection key:
		// when the weapon changes, the node is recreated at a different address.
		auto* playerNodes = f4vr::getPlayerNodes();
		if (!playerNodes) return 0;

		auto* weaponNode = playerNodes->primaryWeapontoWeaponNode;
		if (!weaponNode) return 0;

		// Use the weapon node address as a pseudo-FormID for change detection.
		// This detects: equip, unequip, weapon swap, weapon mod change (node recreated).
		// TODO: Replace with actual FormID from Actor_GetCurrentWeapon once confirmed.
		return reinterpret_cast<std::uint64_t>(weaponNode);
	}

	// =========================================================================
	// Shape Discovery
	// =========================================================================

	const RE::hknpShape* WeaponCollision::findWeaponShape(RE::NiAVObject* weaponNode, RE::hknpWorld* world)
	{
		if (!weaponNode || !world) return nullptr;

		// Diagnostic: log root node info
		const char* rootName = weaponNode->name.c_str();
		auto* rootNiNode = weaponNode->IsNode();
		int childCount = rootNiNode ? static_cast<int>(rootNiNode->GetRuntimeData().children.size()) : 0;
		bool hasCollision = (weaponNode->collisionObject.get() != nullptr);
		ROCK_LOG_INFO(Weapon, "findWeaponShape: root='{}' children={} hasCollision={} addr={:x}",
			rootName ? rootName : "(null)", childCount, hasCollision,
			reinterpret_cast<std::uintptr_t>(weaponNode));

		// Log first few children names for debugging
		if (rootNiNode) {
			auto& kids = rootNiNode->GetRuntimeData().children;
			for (std::uint32_t i = 0; i < kids.size() && i < 8; i++) {
				auto* kid = kids[i].get();
				if (kid) {
					const char* kidName = kid->name.c_str();
					bool kidCol = (kid->collisionObject.get() != nullptr);
					auto* kidNode = kid->IsNode();
					int kidChildren = kidNode ? static_cast<int>(kidNode->GetRuntimeData().children.size()) : 0;
					ROCK_LOG_INFO(Weapon, "  child[{}]: '{}' children={} hasCollision={}",
						i, kidName ? kidName : "(null)", kidChildren, kidCol);
				}
			}
		}

		auto* shape = findWeaponShapeRecursive(weaponNode, world, 0);
		if (shape) {
			ROCK_LOG_INFO(Weapon, "Found weapon shape at {:x}", reinterpret_cast<std::uintptr_t>(shape));
		} else {
			ROCK_LOG_INFO(Weapon, "NO collision shape found on weapon node tree (searched from '{}')",
				rootName ? rootName : "(null)");
		}
		return shape;
	}

	const RE::hknpShape* WeaponCollision::findWeaponShapeRecursive(RE::NiAVObject* node, RE::hknpWorld* world, int depth)
	{
		if (!node || depth > 15) return nullptr;

		// Check this node for a collision object (same pattern as Hand.h:112-133)
		auto* colObj = node->collisionObject.get();
		if (colObj) {
			ROCK_LOG_INFO(Weapon, "{}colObj found on '{}' at depth {}",
				std::string(depth * 2, ' '), node->name.c_str(), depth);

			auto* fieldAt20 = *reinterpret_cast<void**>(
				reinterpret_cast<char*>(colObj) + offsets::kCollisionObject_PhysSystemPtr);
			if (fieldAt20 && reinterpret_cast<std::uintptr_t>(fieldAt20) > 0x10000) {
				auto* physSystem = reinterpret_cast<RE::bhkPhysicsSystem*>(fieldAt20);
				auto* inst = physSystem->instance;
				if (inst && reinterpret_cast<std::uintptr_t>(inst) > 0x10000) {
					ROCK_LOG_INFO(Weapon, "{}physSystem OK, bodyCount={}", std::string(depth * 2, ' '), inst->bodyCount);
					if (inst->bodyCount > 0) {
						std::uint32_t bid = inst->bodyIds[0];
						if (bid != INVALID_BODY_ID) {
							auto* bodyArray = world->GetBodyArray();
							auto& body = bodyArray[bid];
							ROCK_LOG_INFO(Weapon, "{}bodyId={}, shape={:x}, flags=0x{:X}",
								std::string(depth * 2, ' '), bid,
								reinterpret_cast<std::uintptr_t>(body.shape),
								body.flags);
							if (body.shape) {
								return body.shape;
							}
						}
					}
				} else {
					ROCK_LOG_INFO(Weapon, "{}instance invalid or null", std::string(depth * 2, ' '));
				}
			} else {
				ROCK_LOG_INFO(Weapon, "{}+0x20 field invalid (proxy?)", std::string(depth * 2, ' '));
			}
		}

		// Recurse into children
		auto* niNode = node->IsNode();
		if (niNode) {
			auto& kids = niNode->GetRuntimeData().children;
			for (std::uint32_t i = 0; i < kids.size(); i++) {
				auto* kid = kids[i].get();
				if (kid) {
					auto* shape = findWeaponShapeRecursive(kid, world, depth + 1);
					if (shape) return shape;
				}
			}
		}

		return nullptr;
	}

	// =========================================================================
	// Body Creation & Destruction
	// =========================================================================

	void WeaponCollision::createWeaponBody(RE::hknpWorld* world, const RE::hknpShape* shape,
		const RE::NiTransform& initialTransform)
	{
		if (hasWeaponBody()) {
			ROCK_LOG_WARN(Weapon, "createWeaponBody called but body already exists — skipping");
			return;
		}

		if (!shape || !world) return;

		// 1. AddReference on shared shape (CRITICAL — prevents premature free)
		shapeAddRef(shape);
		_cachedShape = shape;

		// 2. Allocate motion for keyframed body
		auto motionId = world->AllocateMotion();

		// 3. Build body creation info (follows Hand::createCollision pattern)
		RE::hknpBodyCinfo cinfo;
		cinfo.shape = shape;
		cinfo.motionId.value = motionId;
		cinfo.motionPropertiesId.value = 0xFF;
		cinfo.collisionFilterInfo = (0x000B << 16) | (ROCK_WEAPON_LAYER & 0x7F);
		cinfo.position = RE::hkVector4f(0.0f, 0.0f, 0.0f, 0.0f);  // W=0 pattern
		cinfo.orientation = RE::hkVector4f(0.0f, 0.0f, 0.0f, 1.0f);
		cinfo.userData = 0;  // CRITICAL: FOIslandActivationListener dereferences unconditionally
		cinfo.name = "ROCK_WeaponCollision";

		// 4. Create body
		auto bodyId = world->CreateBody(cinfo);
		if (bodyId.value == INVALID_BODY_ID) {
			ROCK_LOG_ERROR(Weapon, "CreateBody failed for weapon collision");
			shapeRemoveRef(shape);
			_cachedShape = nullptr;
			return;
		}

		_weaponBodyId = bodyId;

		// 5. Enable contact modifier + keep-awake flags
		world->EnableBodyFlags(bodyId, 0x08020000, 1);

		// 6. Set KEYFRAMED via Bethesda's proper API
		{
			typedef void setKeyframed_t(void*, std::uint32_t);
			static REL::Relocation<setKeyframed_t> setBodyKeyframed{ REL::Offset(offsets::kFunc_SetBodyKeyframed) };
			setBodyKeyframed(world, bodyId.value);
		}

		// 7. Unfreeze motion
		auto* motion = world->GetBodyMotion(bodyId);
		if (motion) {
			motion->timeFactor = 1.0f;
			motion->spaceSplitterWeight = 1.0f;
		}

		// 8. Activate body
		world->ActivateBody(bodyId);

		// 9. Update atomic for physics thread
		_weaponBodyIdAtomic.store(bodyId.value, std::memory_order_release);

		// 10. Initial transform
		_hasPrevTransform = false;
		updateBodyTransform(world, initialTransform, 0.016f);

		ROCK_LOG_INFO(Weapon, "Weapon collision body created — bodyId={}, layer=44, shape={:x}",
			bodyId.value, reinterpret_cast<std::uintptr_t>(shape));
	}

	void WeaponCollision::destroyWeaponBody(RE::hknpWorld* world)
	{
		if (!hasWeaponBody()) return;

		// 1. Clear atomic BEFORE destroying (physics thread safety)
		_weaponBodyIdAtomic.store(INVALID_BODY_ID, std::memory_order_release);

		// 2. Destroy body
		if (world) {
			world->DestroyBodies(&_weaponBodyId, 1);
		}
		_weaponBodyId.value = INVALID_BODY_ID;

		// 3. Release shape reference
		if (_cachedShape) {
			shapeRemoveRef(_cachedShape);
			_cachedShape = nullptr;
		}

		// 4. Reset velocity tracking
		_hasPrevTransform = false;

		ROCK_LOG_INFO(Weapon, "Weapon collision body destroyed");
	}

	// =========================================================================
	// Per-Frame Body Positioning
	// =========================================================================

	void WeaponCollision::updateBodyTransform(RE::hknpWorld* world,
		const RE::NiTransform& weaponTransform, float dt)
	{
		if (!hasWeaponBody() || !world) return;

		const auto targetPos = niPointToHkVector(weaponTransform.translate);

		// Throttled diagnostic — log weapon body position every ~1s
		if (++_posLogCounter >= 90) {
			_posLogCounter = 0;
			ROCK_LOG_INFO(Weapon, "updateBodyTransform: niPos=({:.1f},{:.1f},{:.1f}) hkPos=({:.4f},{:.4f},{:.4f}) bodyId={} dt={:.4f}",
				weaponTransform.translate.x, weaponTransform.translate.y, weaponTransform.translate.z,
				targetPos.x, targetPos.y, targetPos.z,
				_weaponBodyId.value, dt);
		}

		// --- Teleport detection (SetOrigin / cell transition) ---
		if (_hasPrevTransform) {
			float dx = weaponTransform.translate.x - _prevPosition.x;
			float dy = weaponTransform.translate.y - _prevPosition.y;
			float dz = weaponTransform.translate.z - _prevPosition.z;
			float distSq = dx*dx + dy*dy + dz*dz;
			if (distSq > 1000.0f * 1000.0f) {
				_prevPosition = weaponTransform.translate;
				RE::hkTransformf hkTransform;
				hkTransform.rotation = weaponTransform.rotate;
				hkTransform.translation = RE::NiPoint4(targetPos.x, targetPos.y, targetPos.z, 0.0f);
				world->SetBodyTransform(_weaponBodyId, hkTransform, 0);
				return;
			}
		}

		// --- Compute velocity via computeHardKeyFrame ---
		alignas(16) float linVelOut[4] = {0,0,0,0};
		alignas(16) float angVelOut[4] = {0,0,0,0};

		if (_hasPrevTransform && dt > 0.0001f) {
			alignas(16) float tgtPos[4] = { targetPos.x, targetPos.y, targetPos.z, 0.0f };
			alignas(16) float tgtQuat[4];
			niRotToHkQuat(weaponTransform.rotate, tgtQuat);

			// CRITICAL FIX (C2): Pass dt, NOT 1/dt.
			// Ghidra-verified: computeHardKeyFrame internally computes rcpps(param5) = 1/param5.
			// For velocity = displacement/dt, param5 must be dt. Previous code passed invDt
			// → velocity ≈ 8000× too low at 90fps. See Hand.h for full explanation.
			typedef void computeHKF_t(void*, std::uint32_t, const float*, const float*,
				float, float*, float*);
			static REL::Relocation<computeHKF_t> computeHardKeyFrame{
				REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
			computeHardKeyFrame(world, _weaponBodyId.value,
				tgtPos, tgtQuat, dt, linVelOut, angVelOut);

			// Clamp velocity to prevent physics explosions
			constexpr float MAX_VEL = 50.0f;
			float speed = std::sqrt(linVelOut[0]*linVelOut[0] + linVelOut[1]*linVelOut[1] +
				linVelOut[2]*linVelOut[2]);
			if (speed > MAX_VEL) {
				float scale = MAX_VEL / speed;
				linVelOut[0] *= scale; linVelOut[1] *= scale; linVelOut[2] *= scale;
			}
		}

		// --- Teleport body to target ---
		RE::hkTransformf hkTransform;
		hkTransform.rotation = weaponTransform.rotate;
		hkTransform.translation = RE::NiPoint4(targetPos.x, targetPos.y, targetPos.z, 0.0f);
		world->SetBodyTransform(_weaponBodyId, hkTransform, 0);

		// --- Set velocity (for collision response) ---
		{
			auto* linVel = reinterpret_cast<RE::hkVector4f*>(linVelOut);
			auto* angVel = reinterpret_cast<RE::hkVector4f*>(angVelOut);
			typedef void setVel_t(void*, std::uint32_t, RE::hkVector4f*, RE::hkVector4f*);
			static REL::Relocation<setVel_t> setBodyVelocity{ REL::Offset(offsets::kFunc_SetBodyVelocity) };
			setBodyVelocity(world, _weaponBodyId.value, linVel, angVel);
		}

		_prevPosition = weaponTransform.translate;
		_hasPrevTransform = true;
	}

	// =========================================================================
	// Dominant Hand Collision Management
	// =========================================================================

	void WeaponCollision::disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId)
	{
		if (!world || handBodyId.value == INVALID_BODY_ID) return;

		typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
		static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{
			REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

		auto* bodyArray = world->GetBodyArray();
		auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[handBodyId.value]);
		auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
		auto newFilter = currentFilter | (1u << 14);
		setBodyCollisionFilterInfo(world, handBodyId.value, newFilter, 0);

		_dominantHandDisabled = true;
		_disabledHandBodyId = handBodyId;

		ROCK_LOG_INFO(Weapon, "Dominant hand collision DISABLED (bit 14) bodyId={} filter=0x{:08X}",
			handBodyId.value, newFilter);
	}

	void WeaponCollision::enableDominantHandCollision(RE::hknpWorld* world)
	{
		if (!world || _disabledHandBodyId.value == INVALID_BODY_ID) return;

		typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
		static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{
			REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

		auto* bodyArray = world->GetBodyArray();
		auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[_disabledHandBodyId.value]);
		auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
		auto newFilter = currentFilter & ~(1u << 14);
		setBodyCollisionFilterInfo(world, _disabledHandBodyId.value, newFilter, 0);

		ROCK_LOG_INFO(Weapon, "Dominant hand collision RE-ENABLED bodyId={} filter=0x{:08X}",
			_disabledHandBodyId.value, newFilter);

		_dominantHandDisabled = false;
		_disabledHandBodyId.value = INVALID_BODY_ID;
	}

} // namespace frik::rock
