#include "Hand.h"

#include "HavokOffsets.h"

namespace frik::rock
{

	/// Create a box-shaped collision body using hknpConvexPolytopeShape.
	/// WHY: hknpBoxShape doesn't exist in FO4VR's hknp physics (Havok 2014).
	/// HIGGS uses hkpBoxShape for flat palm collision -- better contact surface than
	/// capsule's rounded ends. We create a capsule (same 8-vert 6-face topology),
	/// then overwrite vtable to convex base and fill box vertex/normal/face data.
	/// Ghidra RE: capsule vtable=0x142c9a208, convex=0x142c9a108,
	/// polytope factory at FUN_1416ddbc0 confirms convex vtable for polytope shapes.
	RE::hknpShape* CreateBoxShape(float hx, float hy, float hz, float convexRadius)
	{
		// Allocate via CreateCapsuleShape -- identical memory layout (0x1B0 bytes).
		// Both are 8-vertex, 6-quad-face convex shapes with the same allocation.
		RE::hkVector4f start(-hx, 0.0f, 0.0f, 0.0f);
		RE::hkVector4f end(hx, 0.0f, 0.0f, 0.0f);
		float allocRadius = (hy > hz) ? hy : hz;
		auto* capsule = RE::hknpCapsuleShape::CreateCapsuleShape(start, end, allocRadius);
		if (!capsule) return nullptr;

		auto* s = reinterpret_cast<char*>(capsule);

		// --- Overwrite vtable: capsule -> convex polytope ---
		// Capsule vtable (0x142c9a208) has capsule-specific AABB/raycast functions.
		// Convex base vtable (0x142c9a108) iterates vertices generically -- correct for box.
		static REL::Relocation<std::uintptr_t> convexVtable{ REL::Offset(offsets::kData_ConvexPolytopeVtable) };
		*reinterpret_cast<std::uintptr_t*>(s) = convexVtable.address();

		// --- Overwrite type code: 0x01C3 (capsule-specific) -> 0x0103 (convex polytope) ---
		*reinterpret_cast<std::uint16_t*>(s + 0x10) = 0x0103;

		// --- Set convex radius (0 = sharp edges, >0 = rounded corners) ---
		*reinterpret_cast<float*>(s + 0x14) = convexRadius;

		// --- Fill 8 box vertices at +0x70 (128 bytes) ---
		// Layout: V0-V3 at z=-hz, V4-V7 at z=+hz
		// Each vertex: {x, y, z, W} where W encodes vertex index in 0.5f mantissa
		auto* v = reinterpret_cast<float*>(s + 0x70);
		auto setVert = [&](int i, float x, float y, float z) {
			v[i*4+0] = x;
			v[i*4+1] = y;
			v[i*4+2] = z;
			std::uint32_t w = 0x3F000000u | static_cast<std::uint32_t>(i);
			v[i*4+3] = *reinterpret_cast<float*>(&w);
		};
		setVert(0, -hx, -hy, -hz);
		setVert(1, +hx, -hy, -hz);
		setVert(2, -hx, +hy, -hz);
		setVert(3, +hx, +hy, -hz);
		setVert(4, -hx, -hy, +hz);
		setVert(5, +hx, -hy, +hz);
		setVert(6, -hx, +hy, +hz);
		setVert(7, +hx, +hy, +hz);

		// --- Fill 6 face normals at +0xF0 (128 bytes, 8 slots, first 6 used) ---
		// Each normal: {nx, ny, nz, d} where d = distance from origin to face plane
		auto* n = reinterpret_cast<float*>(s + 0xF0);
		auto setNormal = [&](int i, float nx, float ny, float nz, float d) {
			n[i*4+0] = nx;
			n[i*4+1] = ny;
			n[i*4+2] = nz;
			n[i*4+3] = d;
		};
		setNormal(0, +1.0f,  0.0f,  0.0f, hx);  // +X face
		setNormal(1, -1.0f,  0.0f,  0.0f, hx);  // -X face
		setNormal(2,  0.0f, +1.0f,  0.0f, hy);  // +Y face
		setNormal(3,  0.0f, -1.0f,  0.0f, hy);  // -Y face
		setNormal(4,  0.0f,  0.0f, +1.0f, hz);  // +Z face
		setNormal(5,  0.0f,  0.0f, -1.0f, hz);  // -Z face
		// Padding normals 6-7 (copy face 0-1 for alignment)
		setNormal(6, +1.0f, 0.0f, 0.0f, hx);
		setNormal(7, -1.0f, 0.0f, 0.0f, hx);

		// --- Fill face index table at +0x170 (32 bytes) ---
		// Each entry: {int16 edgeOffset, uint8 numEdges=4, uint8 dataSize=4}
		auto* faceTable = reinterpret_cast<std::uint8_t*>(s + 0x170);
		for (int i = 0; i < 6; i++) {
			auto* entry = faceTable + i * 4;
			std::uint16_t offset = static_cast<std::uint16_t>(i * 4);
			*reinterpret_cast<std::uint16_t*>(entry) = offset;
			entry[2] = 4;  // numEdges
			entry[3] = 4;  // data size
		}

		// --- Fill edge data at +0x190 (24 bytes) ---
		// 6 faces x 4 vertex indices, CCW winding from outside
		auto* edges = reinterpret_cast<std::uint32_t*>(s + 0x190);
		edges[0] = 0x05070301;  // Face 0 (+X): V1,V3,V7,V5
		edges[1] = 0x00020604;  // Face 1 (-X): V4,V6,V2,V0
		edges[2] = 0x03070602;  // Face 2 (+Y): V2,V6,V7,V3
		edges[3] = 0x04050100;  // Face 3 (-Y): V0,V1,V5,V4
		edges[4] = 0x06070504;  // Face 4 (+Z): V4,V5,V7,V6
		edges[5] = 0x03020001;  // Face 5 (-Z): V1,V0,V2,V3

		// --- Update header fields for box (may differ from capsule Init values) ---
		// +0x44: numFaces (capsule Init may have set differently)
		*reinterpret_cast<std::uint16_t*>(s + 0x44) = 6;

		ROCK_LOG_INFO(Hand, "Created box shape: hx={:.4f} hy={:.4f} hz={:.4f} radius={:.4f}",
			hx, hy, hz, convexRadius);

		return reinterpret_cast<RE::hknpShape*>(capsule);
	}

	/// Register a custom "ROCK_Hand" material with high friction for natural grip.
	/// WHY: Default material (ID 0) has standard friction that lets objects slide off
	/// the hand. HIGGS sets a skin material on its hand body for proper grip feel.
	/// In hknp, materials are on the BODY (body+0x70), not the shape.
	/// Registered once via hknpMaterialLibrary::addMaterial, cached for reuse.
	RE::hknpMaterialId registerHandMaterial(RE::hknpWorld* world)
	{
		static RE::hknpMaterialId cachedId{ 0xFFFF };
		if (cachedId.value != 0xFFFF) return cachedId;

		if (!world) return { 0 };

		// Material library lives at world+0x5C8
		auto* matLibPtr = reinterpret_cast<void**>(
			reinterpret_cast<char*>(world) + 0x5C8);
		auto* matLib = *matLibPtr;
		if (!matLib) {
			ROCK_LOG_WARN(Hand, "Material library is null -- using default material 0");
			return { 0 };
		}

		// Construct a default material (0x50 bytes) then customize friction
		alignas(16) char matBuffer[0x50];
		memset(matBuffer, 0, 0x50);

		// Call hknpMaterial constructor to set sane defaults
		typedef void (*matCtor_t)(void*);
		static REL::Relocation<matCtor_t> matCtor{ REL::Offset(offsets::kFunc_MaterialCtor) };
		matCtor(matBuffer);

		// Set high friction for grip (prevents objects sliding off palm)
		// +0x11 = dynamicFriction (quantized uint8, 0-255 via LUT)
		//   200/255 ~ 0.78 -- high grip, objects stick to hand on contact
		*reinterpret_cast<std::uint8_t*>(matBuffer + 0x11) = 200;

		// +0x12 = staticFriction (hkHalf16)
		//   0x3C00 = 1.0f in IEEE half-precision -- maximum static grip
		*reinterpret_cast<std::uint16_t*>(matBuffer + 0x12) = 0x3C00;

		// +0x28 = restitution (hkHalf16)
		//   0x0000 = 0.0f -- no bounce on contact
		*reinterpret_cast<std::uint16_t*>(matBuffer + 0x28) = 0x0000;

		// +0x18 = frictionCombinePolicy (uint8): 2 = MAX (use whichever is higher)
		*reinterpret_cast<std::uint8_t*>(matBuffer + 0x18) = 2;

		// +0x10 = triggerType (uint8): 0 = normal (not a trigger volume)
		*reinterpret_cast<std::uint8_t*>(matBuffer + 0x10) = 0;

		// Register via addMaterial (world+0x5C8 = material library)
		typedef void (*addMat_t)(void*, std::uint16_t*, void*);
		static REL::Relocation<addMat_t> addMaterial{ REL::Offset(offsets::kFunc_MaterialLibrary_AddMaterial) };

		std::uint16_t newId = 0xFFFF;
		addMaterial(matLib, &newId, matBuffer);

		if (newId != 0xFFFF) {
			cachedId.value = newId;
			ROCK_LOG_INFO(Hand, "Registered ROCK_Hand material ID={} (dynFriction=200, staticFriction=1.0, restitution=0.0)",
				newId);
		} else {
			ROCK_LOG_WARN(Hand, "Failed to register ROCK_Hand material -- using default 0");
			return { 0 };
		}

		return cachedId;
	}

	// =========================================================================
	// Hand class method implementations
	// =========================================================================

	void Hand::reset()
	{
		stopSelectionHighlight();
		_isHoldingFlag.store(false, std::memory_order_release);
		_heldBodyIdsCount.store(0, std::memory_order_release);
		_heldBodyContactFrame.store(100, std::memory_order_release);
		_state = HandState::Idle;
		_prevState = HandState::Idle;
		_idleDesired = false;
		_grabRequested = false;
		_releaseRequested = false;
		_handBody.reset();
		_currentSelection.clear();
		_cachedFarCandidate.clear();
		_farDetectCounter = 0;
		_selectionHoldFrames = 0;
		_deselectCooldown = 0;
		_lastDeselectedRef = nullptr;
		_lastTouchedRef = nullptr;
		_lastTouchedFormID = 0;
		_lastTouchedLayer = 0;
		_touchActiveFrames = 100;
		_activeConstraint.clear();
		_savedObjectState.clear();
		_grabPointLocal[0] = _grabPointLocal[1] = _grabPointLocal[2] = _grabPointLocal[3] = 0.0f;
		_grabStartTime = 0.0f;
		_heldLogCounter = 0;
		_pivotBLogCounter = 0;
		_notifCounter = 0;
		_heldBodyIds.clear();
		_grabHandSpace = RE::NiTransform();
		_heldNode = nullptr;
	}

	void Hand::collectHeldBodyIds(RE::TESObjectREFR* refr)
	{
		_heldBodyIds.clear();
		if (!refr) return;
		auto* node3D = refr->Get3D();
		if (!node3D) return;
		collectBodyIdsRecursive(node3D);
	}

	void Hand::collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth)
	{
		if (!node || maxDepth <= 0) return;

		auto* collObj = node->collisionObject.get();
		if (collObj) {
			// Verify this is a bhkNPCollisionObject, not a proxy or other type.
			// bhkNPCollisionProxyObject has different field layout and crashes
			// when we access +0x20 as a physics system pointer.
			// Check: a valid bhkNPCollisionObject has a non-null physics system
			// pointer at +0x20 that points to valid memory above the reserved
			// null-page range (first 64KB on Windows). Addresses below this
			// threshold are OS-reserved and never valid heap/global pointers.
			constexpr std::uintptr_t kMinValidPointer = 0x10000;
			auto* fieldAt20 = *reinterpret_cast<void**>(
				reinterpret_cast<char*>(collObj) + offsets::kCollisionObject_PhysSystemPtr);
			if (fieldAt20 && reinterpret_cast<std::uintptr_t>(fieldAt20) > kMinValidPointer) {
				auto* physSystem = reinterpret_cast<RE::bhkPhysicsSystem*>(fieldAt20);
				auto* inst = physSystem->instance;
				if (inst && reinterpret_cast<std::uintptr_t>(inst) > kMinValidPointer) {
					for (std::int32_t i = 0; i < inst->bodyCount && i < 64; i++) {
						std::uint32_t bid = inst->bodyIds[i];
						if (bid != 0x7FFF'FFFF) {
							_heldBodyIds.push_back(bid);
						}
					}
				}
			}
		}

		auto* niNode = node->IsNode();
		if (niNode) {
			auto& kids = niNode->GetRuntimeData().children;
			for (std::uint32_t i = 0; i < kids.size(); i++) {
				auto* kid = kids[i].get();
				if (kid) collectBodyIdsRecursive(kid, maxDepth - 1);
			}
		}
	}

	bool Hand::getAdjustedHandTransform(RE::NiTransform& outTransform) const
	{
		if (!isHolding()) return false;
		// I3 FIX: Re-derive node from refr each call instead of trusting cached _heldNode.
		// If the object was unloaded mid-grab, refr->Get3D() returns nullptr safely.
		RE::NiAVObject* node = nullptr;
		if (_savedObjectState.refr && !_savedObjectState.refr->IsDeleted() &&
			!_savedObjectState.refr->IsDisabled()) {
			node = _savedObjectState.refr->Get3D();
		}
		if (!node) return false;

		// Invert _grabHandSpace: invR = R^T, invPos = -(R^T * pos) / scale
		RE::NiMatrix3 invRot = _grabHandSpace.rotate.Transpose();
		float invScale = (_grabHandSpace.scale > 0.0001f) ? (1.0f / _grabHandSpace.scale) : 1.0f;
		RE::NiPoint3 invPos;
		invPos.x = -(invRot.entry[0][0] * _grabHandSpace.translate.x +
			invRot.entry[0][1] * _grabHandSpace.translate.y +
			invRot.entry[0][2] * _grabHandSpace.translate.z) * invScale;
		invPos.y = -(invRot.entry[1][0] * _grabHandSpace.translate.x +
			invRot.entry[1][1] * _grabHandSpace.translate.y +
			invRot.entry[1][2] * _grabHandSpace.translate.z) * invScale;
		invPos.z = -(invRot.entry[2][0] * _grabHandSpace.translate.x +
			invRot.entry[2][1] * _grabHandSpace.translate.y +
			invRot.entry[2][2] * _grabHandSpace.translate.z) * invScale;

		// Multiply: objectWorld * inverseGrab
		// result.rot = obj.rot * inv.rot
		// result.pos = obj.pos + (obj.rot * inv.pos) * obj.scale
		const auto& obj = node->world;
		outTransform.rotate = obj.rotate * invRot;
		outTransform.scale = obj.scale * invScale;
		RE::NiPoint3 rotatedPos;
		rotatedPos.x = obj.rotate.entry[0][0] * invPos.x + obj.rotate.entry[0][1] * invPos.y + obj.rotate.entry[0][2] * invPos.z;
		rotatedPos.y = obj.rotate.entry[1][0] * invPos.x + obj.rotate.entry[1][1] * invPos.y + obj.rotate.entry[1][2] * invPos.z;
		rotatedPos.z = obj.rotate.entry[2][0] * invPos.x + obj.rotate.entry[2][1] * invPos.y + obj.rotate.entry[2][2] * invPos.z;
		outTransform.translate.x = obj.translate.x + rotatedPos.x * obj.scale;
		outTransform.translate.y = obj.translate.y + rotatedPos.y * obj.scale;
		outTransform.translate.z = obj.translate.z + rotatedPos.z * obj.scale;

		return true;
	}

	void Hand::updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
		const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward,
		float nearRange, float farRange, RE::TESObjectREFR* otherHandRef)
	{
		// Don't run selection while in grab/hold states (Phase 3+)
		if (_state != HandState::Idle && _state != HandState::SelectedClose) return;

		// Near detection -- every frame
		auto nearCandidate = findCloseObject(bhkWorld, hknpWorld,
			palmPos, palmForward, nearRange, _isLeft, otherHandRef);

		// Far detection -- every 3rd frame (90fps -> 30Hz, still responsive)
		SelectedObject farCandidate;
		_farDetectCounter++;
		if (_farDetectCounter >= 3) {
			_farDetectCounter = 0;
			farCandidate = findFarObject(bhkWorld, hknpWorld,
				palmPos, palmForward, farRange, otherHandRef);
			_cachedFarCandidate = farCandidate;
		} else {
			farCandidate = _cachedFarCandidate;
		}

		// Cooldown: block reselection of recently-cleared object
		if (_deselectCooldown > 0) {
			_deselectCooldown--;
			if (nearCandidate.refr == _lastDeselectedRef) nearCandidate.clear();
			if (farCandidate.refr == _lastDeselectedRef) farCandidate.clear();
			if (_deselectCooldown == 0) _lastDeselectedRef = nullptr;
		}

		// Priority: near beats far
		SelectedObject best = nearCandidate.isValid() ? nearCandidate : farCandidate;

		// Sticky selection bias: if we already have a valid selection and the new candidate
		// is a DIFFERENT object, only switch if it's significantly closer (30% closer).
		// This prevents flickering between two equidistant objects every frame.
		if (best.isValid() && _currentSelection.isValid()
			&& best.refr != _currentSelection.refr
			&& !best.isFarSelection && !_currentSelection.isFarSelection) {
			float stickyThreshold = _currentSelection.distance * 0.7f;
			if (best.distance > stickyThreshold) {
				// New candidate is not significantly closer — keep current
				_currentSelection.distance = _currentSelection.distance;  // no-op, keep as-is
				_selectionHoldFrames++;
				return;
			}
		}

		if (best.refr == _currentSelection.refr && best.isValid()) {
			// Same object still detected -- update distance, increment hold counter
			_currentSelection.distance = best.distance;
			_selectionHoldFrames++;
		} else if (best.isValid()) {
			// New/different object detected -- log with object name and type
			auto* baseObj = best.refr->GetObjectReference();
			const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";
			// Get name from base form (TESBoundObject), not the placed reference
			auto objName = baseObj
				? RE::TESFullName::GetFullName(*baseObj, false)
				: std::string_view{};
			const char* nameStr = objName.empty() ? "(unnamed)" : objName.data();

			if (_currentSelection.isValid()) {
				ROCK_LOG_INFO(Hand, "{} hand switched -> {} [{}] '{}' formID={:08X} dist={:.1f}",
					handName(),
					best.isFarSelection ? "far" : "near",
					typeName, nameStr,
					best.refr->GetFormID(), best.distance);
			} else {
				ROCK_LOG_INFO(Hand, "{} hand selected {} [{}] '{}' formID={:08X} dist={:.1f}",
					handName(), best.isFarSelection ? "far" : "near",
					typeName, nameStr,
					best.refr->GetFormID(), best.distance);
			}
			// Stop highlight on previous selection
			stopSelectionHighlight();

			_currentSelection = best;
			// SelectedFar is a stub — no grab trigger or updateSelection guard handles it.
			// Use SelectedClose for both near and far until the far-grab pipeline is built.
			_state = HandState::SelectedClose;
			_selectionHoldFrames = 0;

			// Start highlight on new selection
			playSelectionHighlight(best.refr);
		} else if (_currentSelection.isValid()) {
			// No candidate found -- apply hysteresis with minimum hold time
			constexpr int MIN_HOLD_FRAMES = 15;  // ~0.17s at 90Hz

			if (_selectionHoldFrames < MIN_HOLD_FRAMES) {
				// Too soon to deselect -- keep current, increment hold
				_selectionHoldFrames++;
				return;
			}

			// Hysteresis: 2.5x detection range before clearing
			float hysteresisRange = _currentSelection.isFarSelection
				? farRange * 2.5f : nearRange * 2.5f;

			// Re-check distance to current selection (guard against stale bodyId)
			if (_currentSelection.bodyId.value != 0x7FFF'FFFF) {
				auto& body = hknpWorld->GetBody(_currentSelection.bodyId);
				// Validate body is still active (motionIndex > 0 and reasonable)
				if (body.motionIndex > 0 && body.motionIndex < 4096) {
					auto* motion = hknpWorld->GetBodyMotion(_currentSelection.bodyId);
					if (motion) {
						RE::NiPoint3 objPos = hkVectorToNiPoint(motion->position);
						_currentSelection.distance = (objPos - palmPos).Length();
					}
				} else {
					// Body was unloaded (NPC left cell, etc.) -- clear immediately
					_currentSelection.clear();
					_state = HandState::Idle;
					_selectionHoldFrames = 0;
					return;
				}
			}

			// Check if ref is still valid
			bool refInvalid = !_currentSelection.refr ||
				_currentSelection.refr->IsDeleted() ||
				_currentSelection.refr->IsDisabled();

			if (refInvalid || _currentSelection.distance > hysteresisRange) {
				ROCK_LOG_INFO(Hand, "{} hand cleared (formID={:08X}, dist={:.1f}, held={}f)",
					handName(), _currentSelection.refr ? _currentSelection.refr->GetFormID() : 0,
					_currentSelection.distance, _selectionHoldFrames);
				stopSelectionHighlight();
				_lastDeselectedRef = _currentSelection.refr;
				_deselectCooldown = 10;  // block reselection for 10 frames (~0.11s)
				_currentSelection.clear();
				_state = HandState::Idle;
				_selectionHoldFrames = 0;
			}
		} else {
			// Nothing selected, nothing found -- stay idle
			_state = HandState::Idle;
		}
	}

	bool Hand::createCollision(RE::hknpWorld* world, void* bhkWorld,
		float halfExtentX, float halfExtentY, float halfExtentZ)
	{
		if (hasCollisionBody()) {
			ROCK_LOG_WARN(Hand, "{} hand already has collision body -- skipping create", handName());
			return false;
		}

		if (!world || !bhkWorld) {
			ROCK_LOG_ERROR(Hand, "{} hand createCollision: world={} bhkWorld={}", handName(), (void*)world, bhkWorld);
			return false;
		}

		// Box-shaped palm collider (Havok units). See CreateBoxShape for vertex layout.
		auto* shape = CreateBoxShape(halfExtentX, halfExtentY, halfExtentZ,
			g_rockConfig.rockHandCollisionBoxRadius);
		if (!shape) {
			ROCK_LOG_ERROR(Hand, "{} hand createCollision: box shape creation failed", handName());
			return false;
		}

		// High-friction hand material for natural grip
		auto materialId = registerHandMaterial(world);

		// FilterInfo: bits 0-6 = layer (43), bits 16-31 = collision group (11).
		std::uint32_t filterInfo = (0x000B << 16) | (ROCK_HAND_LAYER & 0x7F);

		// Create via BethesdaPhysicsBody — full 12-step pipeline (CreatePhantomBody pattern).
		// This creates bhkPhysicsSystem + bhkNPCollisionObject + sets body+0x88 back-pointer.
		// Replaces raw CreateBody + manual flag setup + manual velocity writes.
		bool ok = _handBody.create(world, bhkWorld, shape, filterInfo, materialId,
			BethesdaMotionType::Keyframed,
			_isLeft ? "ROCK_LeftHand" : "ROCK_RightHand");

		if (!ok) {
			ROCK_LOG_ERROR(Hand, "{} hand createCollision: BethesdaPhysicsBody::create failed", handName());
			return false;
		}

		// Create NiNode for scene graph integration (proper Bethesda pattern).
		// This sets collisionObject+0x10 = niNode AND niNode+0x100 = collisionObject.
		_handBody.createNiNode(_isLeft ? "ROCK_LeftHand" : "ROCK_RightHand");

		ROCK_LOG_INFO(Hand, "{} hand collision created via BethesdaPhysicsBody — bodyId={}",
			handName(), _handBody.getBodyId().value);

		return true;
	}

	void Hand::destroyCollision(void* bhkWorld)
	{
		if (!hasCollisionBody()) return;

		ROCK_LOG_INFO(Hand, "{} hand collision destroying — bodyId={}", handName(), _handBody.getBodyId().value);
		_handBody.destroy(bhkWorld);
	}

	void Hand::updateCollisionTransform(RE::hknpWorld* world, const RE::NiTransform& handTransform,
		float deltaTime)
	{
		if (!hasCollisionBody() || !world) return;

		// HIGGS pattern (verified from source code analysis):
		// 1. Convert game-space position → Havok space (* havokWorldScale)
		// 2. Compute velocity via computeHardKeyFrame (Havok space)
		// 3. Clamp velocity (ApplyHardKeyframeVelocityClamped)
		// 4. Set transform + velocity via API
		//
		// WHY NOT DriveToKeyFrame: DriveToKeyFrame at 0x141e086e0 operates in
		// GAME space (designed for skeleton bones that don't participate in constraints).
		// Hand bodies MUST be in Havok space because they form constraints with
		// grabbed objects (which are in Havok space). Mixing coordinate spaces
		// produces 70× pivot error and non-functional motors.
		//
		// HIGGS ComputeHandCollisionTransform explicitly does:
		//   transform.m_translation = NiPointToHkVector(
		//       (handTransform * (offset / havokWorldScale)) * havokWorldScale);
		// Net effect: game-space hand position → Havok space.

		const auto bodyId = _handBody.getBodyId();

		// --- Step 1: Convert to Havok space ---
		const float targetX = handTransform.translate.x * kGameToHavokScale;
		const float targetY = handTransform.translate.y * kGameToHavokScale;
		const float targetZ = handTransform.translate.z * kGameToHavokScale;

		// Read current body position for teleport detection (Havok space)
		auto* bodyArray = world->GetBodyArray();
		auto* bodyFloats = reinterpret_cast<float*>(&bodyArray[bodyId.value]);
		float curX = bodyFloats[12], curY = bodyFloats[13], curZ = bodyFloats[14];

		float dx = targetX - curX, dy = targetY - curY, dz = targetZ - curZ;
		float dist = sqrtf(dx*dx + dy*dy + dz*dz);
		bool isTeleport = (dist > 5.0f);

		// --- Step 2: Compute velocity via computeHardKeyFrame (Havok space) ---
		alignas(16) float linVelOut[4] = {0,0,0,0};
		alignas(16) float angVelOut[4] = {0,0,0,0};

		if (deltaTime > 0.0001f && !isTeleport) {
			alignas(16) float tgtPos[4] = { targetX, targetY, targetZ, 0.0f };
			alignas(16) float tgtQuat[4];
			niRotToHkQuat(handTransform.rotate, tgtQuat);

			typedef void (*computeHKF_t)(void*, std::uint32_t, const float*, const float*,
				float, float*, float*);
			static REL::Relocation<computeHKF_t> computeHardKeyFrame{
				REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
			computeHardKeyFrame(world, bodyId.value,
				tgtPos, tgtQuat, deltaTime, linVelOut, angVelOut);
		}

		// --- Step 3: Clamp velocity (HIGGS ApplyHardKeyframeVelocityClamped) ---
		{
			float MAX_LIN_VEL = g_rockConfig.rockMaxLinearVelocity;
			float MAX_ANG_VEL = g_rockConfig.rockMaxAngularVelocity;

			float linSpeed = sqrtf(linVelOut[0]*linVelOut[0] + linVelOut[1]*linVelOut[1] + linVelOut[2]*linVelOut[2]);
			if (linSpeed > MAX_LIN_VEL) {
				float s = MAX_LIN_VEL / linSpeed;
				linVelOut[0] *= s; linVelOut[1] *= s; linVelOut[2] *= s;
			}

			float angSpeed = sqrtf(angVelOut[0]*angVelOut[0] + angVelOut[1]*angVelOut[1] + angVelOut[2]*angVelOut[2]);
			if (angSpeed > MAX_ANG_VEL) {
				float s = MAX_ANG_VEL / angSpeed;
				angVelOut[0] *= s; angVelOut[1] *= s; angVelOut[2] *= s;
			}
		}

		// --- Step 4: Set transform (Havok space, deferred-safe via BethesdaPhysicsBody) ---
		{
			RE::hkTransformf hkTransform;
			hkTransform.rotation = handTransform.rotate;
			hkTransform.translation = RE::NiPoint4(targetX, targetY, targetZ, 0.0f);
			_handBody.setTransform(hkTransform);
		}

		// --- Step 5: Set velocity (deferred-safe via BethesdaPhysicsBody) ---
		_handBody.setVelocity(linVelOut, angVelOut);
	}

	void Hand::updateDebugColliderVis(const RE::NiPoint3& palmPos, bool show, RE::NiNode* parentNode,
		float hx, float hy, float hz, int shapeType)
	{
		// Hot-reload: if shape type changed, destroy old and recreate
		if (_debugColliderVis && _debugColliderVisShape != shapeType) {
			destroyDebugColliderVis();
		}

		if (show && !_debugColliderVis && parentNode) {
			const char* meshPath = nullptr;
			switch (shapeType) {
				case 0:  meshPath = "Data/Meshes/ROCK/DebugBox_Trigger256.nif"; break;
				case 1:  meshPath = "Data/Meshes/ROCK/DebugBox_Trigger512.nif"; break;
				case 2:  meshPath = "Data/Meshes/ROCK/DebugBox_Activator.nif"; break;
				case 3:  meshPath = "Data/Meshes/ROCK/DebugBox_Utility.nif"; break;
				case 4:  meshPath = "Data/Meshes/ROCK/DebugBox_VaultSuit.nif"; break;
				default: meshPath = "Data/Meshes/ROCK/DebugBox_Trigger256.nif"; break;
			}
			_debugColliderVis = f4cf::f4vr::getClonedNiNodeForNifFileSetName(meshPath);
			if (_debugColliderVis) {
				_debugColliderVis->name = RE::BSFixedString(_isLeft ? "ROCK_DebugL" : "ROCK_DebugR");
				parentNode->AttachChild(_debugColliderVis, true);
				_debugColliderVisParent = parentNode;
				_debugColliderVis->flags.flags &= 0xfffffffffffffffe;
				_debugColliderVis->local.scale = 1.0f;
				_debugColliderVisShape = shapeType;
				ROCK_LOG_INFO(Hand, "{} debug box created: type={} mesh={}",
					handName(), shapeType, meshPath);
			} else {
				ROCK_LOG_WARN(Hand, "{} debug box FAILED to load {}", handName(), meshPath);
			}
		}

		// Re-parent if the parent node changed (e.g. switched from FRIK bone to wand node)
		if (_debugColliderVis && parentNode && _debugColliderVisParent != parentNode) {
			if (_debugColliderVisParent) {
				_debugColliderVisParent->DetachChild(_debugColliderVis);
			}
			parentNode->AttachChild(_debugColliderVis, true);
			_debugColliderVisParent = parentNode;
		}

		if (_debugColliderVis && parentNode) {
			if (show) {
				_debugColliderVis->flags.flags &= 0xfffffffffffffffe;
				// Compute local offset: transform world palmPos into parent's local space
				RE::NiPoint3 offset = palmPos - parentNode->world.translate;
				_debugColliderVis->local.translate = parentNode->world.rotate.Transpose() * offset;

				// Non-uniform scale via rotation matrix to match collision box extents.
				// Scale factor depends on mesh size. Trigger boxes are 256 game units per side (half=128).
				float meshHalf = 128.0f;
				if (shapeType == 2) meshHalf = 64.0f;
				if (shapeType == 4) meshHalf = 32.0f;
				const float s = 70.0f / meshHalf;
				RE::NiMatrix3 scaledRot;
				scaledRot.entry[0][0] = hx * s;
				scaledRot.entry[1][0] = 0.0f;
				scaledRot.entry[2][0] = 0.0f;
				scaledRot.entry[0][1] = 0.0f;
				scaledRot.entry[1][1] = hy * s;
				scaledRot.entry[2][1] = 0.0f;
				scaledRot.entry[0][2] = 0.0f;
				scaledRot.entry[1][2] = 0.0f;
				scaledRot.entry[2][2] = hz * s;
				_debugColliderVis->local.rotate = scaledRot;
			} else {
				_debugColliderVis->flags.flags |= 0x1;
				_debugColliderVis->local.scale = 0;
			}
		}
	}

	void Hand::destroyDebugColliderVis()
	{
		if (_debugColliderVis) {
			_debugColliderVis->flags.flags |= 0x1;
			_debugColliderVis->local.scale = 0;
			if (_debugColliderVis->parent) {
				_debugColliderVis->parent->DetachChild(_debugColliderVis);
			}
			_debugColliderVis = nullptr;
			_debugColliderVisParent = nullptr;
		}
	}

} // namespace frik::rock
