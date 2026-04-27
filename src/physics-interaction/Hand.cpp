#include "Hand.h"

#include <cmath>

#include "DebugAxisMath.h"
#include "HavokOffsets.h"
#include "PalmTransform.h"
#include "TransformMath.h"

namespace frik::rock
{
    namespace
    {
        RE::NiPoint3 normalizeDebugDirection(RE::NiPoint3 value)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            if (lengthSquared <= 1.0e-8f) {
                return RE::NiPoint3(0.0f, 0.0f, 1.0f);
            }

            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            value.x *= inverseLength;
            value.y *= inverseLength;
            value.z *= inverseLength;
            return value;
        }

        RE::NiPoint3 crossDebug(const RE::NiPoint3& a, const RE::NiPoint3& b) { return RE::NiPoint3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

        RE::NiMatrix3 buildArrowLocalRotation(const RE::NiPoint3& forwardLocal)
        {
            const RE::NiPoint3 xAxis = normalizeDebugDirection(forwardLocal);
            RE::NiPoint3 upHint = std::fabs(xAxis.z) > 0.95f ? RE::NiPoint3(0.0f, 1.0f, 0.0f) : RE::NiPoint3(0.0f, 0.0f, 1.0f);

            const float upDot = upHint.x * xAxis.x + upHint.y * xAxis.y + upHint.z * xAxis.z;
            RE::NiPoint3 zAxis(upHint.x - xAxis.x * upDot, upHint.y - xAxis.y * upDot, upHint.z - xAxis.z * upDot);
            zAxis = normalizeDebugDirection(zAxis);
            const RE::NiPoint3 yAxis = normalizeDebugDirection(crossDebug(zAxis, xAxis));

            RE::NiMatrix3 result{};
            result.entry[0][0] = xAxis.x;
            result.entry[1][0] = xAxis.y;
            result.entry[2][0] = xAxis.z;
            result.entry[0][1] = yAxis.x;
            result.entry[1][1] = yAxis.y;
            result.entry[2][1] = yAxis.z;
            result.entry[0][2] = zAxis.x;
            result.entry[1][2] = zAxis.y;
            result.entry[2][2] = zAxis.z;
            return result;
        }

        RE::NiMatrix3 makeIdentityRotation()
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = 1.0f;
            result.entry[1][1] = 1.0f;
            result.entry[2][2] = 1.0f;
            return result;
        }

        float matrixDeterminant(const RE::NiMatrix3& matrix)
        {
            return matrix.entry[0][0] * (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
                matrix.entry[0][1] * (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
                matrix.entry[0][2] * (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
        }

        float bodyBasisDeterminant(const float* bodyFloats)
        {
            const float x0 = bodyFloats[0];
            const float x1 = bodyFloats[1];
            const float x2 = bodyFloats[2];
            const float y0 = bodyFloats[4];
            const float y1 = bodyFloats[5];
            const float y2 = bodyFloats[6];
            const float z0 = bodyFloats[8];
            const float z1 = bodyFloats[9];
            const float z2 = bodyFloats[10];

            return x0 * (y1 * z2 - y2 * z1) - y0 * (x1 * z2 - x2 * z1) + z0 * (x1 * y2 - x2 * y1);
        }
    }

    RE::hknpShape* CreateBoxShape(float hx, float hy, float hz, float convexRadius)
    {
        RE::hkVector4f start(-hx, 0.0f, 0.0f, 0.0f);
        RE::hkVector4f end(hx, 0.0f, 0.0f, 0.0f);
        float allocRadius = (hy > hz) ? hy : hz;
        auto* capsule = RE::hknpCapsuleShape::CreateCapsuleShape(start, end, allocRadius);
        if (!capsule)
            return nullptr;

        auto* s = reinterpret_cast<char*>(capsule);

        static REL::Relocation<std::uintptr_t> convexVtable{ REL::Offset(offsets::kData_ConvexPolytopeVtable) };
        *reinterpret_cast<std::uintptr_t*>(s) = convexVtable.address();

        *reinterpret_cast<std::uint16_t*>(s + 0x10) = 0x0103;

        *reinterpret_cast<float*>(s + 0x14) = convexRadius;

        auto* v = reinterpret_cast<float*>(s + 0x70);
        auto setVert = [&](int i, float x, float y, float z) {
            v[i * 4 + 0] = x;
            v[i * 4 + 1] = y;
            v[i * 4 + 2] = z;
            std::uint32_t w = 0x3F000000u | static_cast<std::uint32_t>(i);
            v[i * 4 + 3] = *reinterpret_cast<float*>(&w);
        };
        setVert(0, -hx, -hy, -hz);
        setVert(1, +hx, -hy, -hz);
        setVert(2, -hx, +hy, -hz);
        setVert(3, +hx, +hy, -hz);
        setVert(4, -hx, -hy, +hz);
        setVert(5, +hx, -hy, +hz);
        setVert(6, -hx, +hy, +hz);
        setVert(7, +hx, +hy, +hz);

        auto* n = reinterpret_cast<float*>(s + 0xF0);
        auto setNormal = [&](int i, float nx, float ny, float nz, float d) {
            n[i * 4 + 0] = nx;
            n[i * 4 + 1] = ny;
            n[i * 4 + 2] = nz;
            n[i * 4 + 3] = d;
        };
        setNormal(0, +1.0f, 0.0f, 0.0f, hx);
        setNormal(1, -1.0f, 0.0f, 0.0f, hx);
        setNormal(2, 0.0f, +1.0f, 0.0f, hy);
        setNormal(3, 0.0f, -1.0f, 0.0f, hy);
        setNormal(4, 0.0f, 0.0f, +1.0f, hz);
        setNormal(5, 0.0f, 0.0f, -1.0f, hz);

        setNormal(6, +1.0f, 0.0f, 0.0f, hx);
        setNormal(7, -1.0f, 0.0f, 0.0f, hx);

        auto* faceTable = reinterpret_cast<std::uint8_t*>(s + 0x170);
        for (int i = 0; i < 6; i++) {
            auto* entry = faceTable + i * 4;
            std::uint16_t offset = static_cast<std::uint16_t>(i * 4);
            *reinterpret_cast<std::uint16_t*>(entry) = offset;
            entry[2] = 4;
            entry[3] = 4;
        }

        auto* edges = reinterpret_cast<std::uint32_t*>(s + 0x190);
        edges[0] = 0x05070301;
        edges[1] = 0x00020604;
        edges[2] = 0x03070602;
        edges[3] = 0x04050100;
        edges[4] = 0x06070504;
        edges[5] = 0x03020001;

        *reinterpret_cast<std::uint16_t*>(s + 0x44) = 6;

        ROCK_LOG_INFO(Hand, "Created box shape: hx={:.4f} hy={:.4f} hz={:.4f} radius={:.4f}", hx, hy, hz, convexRadius);

        return reinterpret_cast<RE::hknpShape*>(capsule);
    }

    RE::hknpMaterialId registerHandMaterial(RE::hknpWorld* world)
    {
        static void* cachedMaterialLibrary = nullptr;
        static RE::hknpMaterialId cachedId{ 0xFFFF };

        if (!world)
            return { 0 };

        auto* matLibPtr = reinterpret_cast<void**>(reinterpret_cast<char*>(world) + 0x5C8);
        auto* matLib = *matLibPtr;
        if (!matLib) {
            ROCK_LOG_WARN(Hand, "Material library is null -- using default material 0");
            return { 0 };
        }
        if (cachedMaterialLibrary == matLib && cachedId.value != 0xFFFF) {
            return cachedId;
        }

        alignas(16) char matBuffer[0x50];
        memset(matBuffer, 0, 0x50);

        typedef void (*matCtor_t)(void*);
        static REL::Relocation<matCtor_t> matCtor{ REL::Offset(offsets::kFunc_MaterialCtor) };
        matCtor(matBuffer);

        *reinterpret_cast<std::uint8_t*>(matBuffer + 0x11) = 200;

        *reinterpret_cast<std::uint16_t*>(matBuffer + 0x12) = 0x3C00;

        *reinterpret_cast<std::uint16_t*>(matBuffer + 0x28) = 0x0000;

        *reinterpret_cast<std::uint8_t*>(matBuffer + 0x18) = 2;

        *reinterpret_cast<std::uint8_t*>(matBuffer + 0x10) = 0;

        typedef void (*addMat_t)(void*, std::uint16_t*, void*);
        static REL::Relocation<addMat_t> addMaterial{ REL::Offset(offsets::kFunc_MaterialLibrary_AddMaterial) };

        std::uint16_t newId = 0xFFFF;
        addMaterial(matLib, &newId, matBuffer);

        if (newId != 0xFFFF) {
            cachedId.value = newId;
            cachedMaterialLibrary = matLib;
            ROCK_LOG_INFO(Hand, "Registered ROCK_Hand material ID={} (dynFriction=200, staticFriction=1.0, restitution=0.0)", newId);
        } else {
            ROCK_LOG_WARN(Hand, "Failed to register ROCK_Hand material -- using default 0");
            return { 0 };
        }

        return cachedId;
    }

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
        _grabStartTime = 0.0f;
        _heldLogCounter = 0;
        _notifCounter = 0;
        _transformWitnessLogCounter = 0;
        _heldBodyIds.clear();
        _grabHandSpace = RE::NiTransform();
        _grabConstraintHandSpace = RE::NiTransform();
        _grabBodyLocalTransform = RE::NiTransform();
        _grabRootBodyLocalTransform = RE::NiTransform();
        _grabOwnerBodyLocalTransform = RE::NiTransform();
        _heldNode = nullptr;
        clearGrabHandCollisionSuppressionState();
    }

    void Hand::collectHeldBodyIds(RE::TESObjectREFR* refr)
    {
        _heldBodyIds.clear();
        if (!refr)
            return;
        auto* node3D = refr->Get3D();
        if (!node3D)
            return;
        collectBodyIdsRecursive(node3D);
    }

    void Hand::collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth)
    {
        if (!node || maxDepth <= 0)
            return;

        auto* collObj = node->collisionObject.get();
        if (collObj) {
            constexpr std::uintptr_t kMinValidPointer = 0x10000;
            auto* fieldAt20 = *reinterpret_cast<void**>(reinterpret_cast<char*>(collObj) + offsets::kCollisionObject_PhysSystemPtr);
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
            for (auto i = decltype(kids.size()){ 0 }; i < kids.size(); i++) {
                auto* kid = kids[i].get();
                if (kid)
                    collectBodyIdsRecursive(kid, maxDepth - 1);
            }
        }
    }

    bool Hand::getAdjustedHandTransform(RE::NiTransform& outTransform) const
    {
        if (!isHolding())
            return false;

        RE::NiAVObject* node = nullptr;
        if (_savedObjectState.refr && !_savedObjectState.refr->IsDeleted() && !_savedObjectState.refr->IsDisabled()) {
            node = _heldNode ? _heldNode : _savedObjectState.refr->Get3D();
        }
        if (!node)
            return false;

        outTransform = transform_math::composeTransforms(node->world, transform_math::invertTransform(_grabHandSpace));

        return true;
    }

    void Hand::updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& selectionOrigin, const RE::NiPoint3& palmNormal,
        const RE::NiPoint3& pointingDirection, float nearRange, float farRange, RE::TESObjectREFR* otherHandRef)
    {
        if (_state != HandState::Idle && _state != HandState::SelectedClose)
            return;

        auto nearCandidate = findCloseObject(bhkWorld, hknpWorld, selectionOrigin, palmNormal, nearRange, _isLeft, otherHandRef);

        SelectedObject farCandidate;
        _farDetectCounter++;
        if (_farDetectCounter >= 3) {
            _farDetectCounter = 0;
            farCandidate = findFarObject(bhkWorld, hknpWorld, selectionOrigin, pointingDirection, farRange, otherHandRef);
            _cachedFarCandidate = farCandidate;
        } else {
            farCandidate = _cachedFarCandidate;
        }

        if (_deselectCooldown > 0) {
            _deselectCooldown--;
            if (nearCandidate.refr == _lastDeselectedRef)
                nearCandidate.clear();
            if (farCandidate.refr == _lastDeselectedRef)
                farCandidate.clear();
            if (_deselectCooldown == 0)
                _lastDeselectedRef = nullptr;
        }

        SelectedObject best = nearCandidate.isValid() ? nearCandidate : farCandidate;

        if (best.isValid() && _currentSelection.isValid() && best.refr != _currentSelection.refr && !best.isFarSelection && !_currentSelection.isFarSelection) {
            float stickyThreshold = _currentSelection.distance * 0.7f;
            if (best.distance > stickyThreshold) {
                _currentSelection.distance = _currentSelection.distance;
                _selectionHoldFrames++;
                return;
            }
        }

        if (best.refr == _currentSelection.refr && best.isValid()) {
            _currentSelection.distance = best.distance;
            _selectionHoldFrames++;
        } else if (best.isValid()) {
            auto* baseObj = best.refr->GetObjectReference();
            const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";

            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const char* nameStr = objName.empty() ? "(unnamed)" : objName.data();

            if (_currentSelection.isValid()) {
                ROCK_LOG_INFO(Hand, "{} hand switched -> {} [{}] '{}' formID={:08X} dist={:.1f}", handName(), best.isFarSelection ? "far" : "near", typeName, nameStr,
                    best.refr->GetFormID(), best.distance);
            } else {
                ROCK_LOG_INFO(Hand, "{} hand selected {} [{}] '{}' formID={:08X} dist={:.1f}", handName(), best.isFarSelection ? "far" : "near", typeName, nameStr,
                    best.refr->GetFormID(), best.distance);
            }

            stopSelectionHighlight();

            _currentSelection = best;

            _state = HandState::SelectedClose;
            _selectionHoldFrames = 0;

            playSelectionHighlight(best.refr);
        } else if (_currentSelection.isValid()) {
            constexpr int MIN_HOLD_FRAMES = 15;

            if (_selectionHoldFrames < MIN_HOLD_FRAMES) {
                _selectionHoldFrames++;
                return;
            }

            float hysteresisRange = _currentSelection.isFarSelection ? farRange * 2.5f : nearRange * 2.5f;

            if (_currentSelection.bodyId.value != 0x7FFF'FFFF) {
                auto& body = hknpWorld->GetBody(_currentSelection.bodyId);

                if (body.motionIndex > 0 && body.motionIndex < 4096) {
                    auto* motion = hknpWorld->GetBodyMotion(_currentSelection.bodyId);
                    if (motion) {
                        RE::NiPoint3 objPos = hkVectorToNiPoint(motion->position);
                        _currentSelection.distance = (objPos - selectionOrigin).Length();
                    }
                } else {
                    _currentSelection.clear();
                    _state = HandState::Idle;
                    _selectionHoldFrames = 0;
                    return;
                }
            }

            bool refInvalid = !_currentSelection.refr || _currentSelection.refr->IsDeleted() || _currentSelection.refr->IsDisabled();

            if (refInvalid || _currentSelection.distance > hysteresisRange) {
                ROCK_LOG_INFO(Hand, "{} hand cleared (formID={:08X}, dist={:.1f}, held={}f)", handName(), _currentSelection.refr ? _currentSelection.refr->GetFormID() : 0,
                    _currentSelection.distance, _selectionHoldFrames);
                stopSelectionHighlight();
                _lastDeselectedRef = _currentSelection.refr;
                _deselectCooldown = 10;
                _currentSelection.clear();
                _state = HandState::Idle;
                _selectionHoldFrames = 0;
            }
        } else {
            _state = HandState::Idle;
        }
    }

    bool Hand::createCollision(RE::hknpWorld* world, void* bhkWorld, float halfExtentX, float halfExtentY, float halfExtentZ)
    {
        if (hasCollisionBody()) {
            ROCK_LOG_WARN(Hand, "{} hand already has collision body -- skipping create", handName());
            return false;
        }

        if (!world || !bhkWorld) {
            ROCK_LOG_ERROR(Hand, "{} hand createCollision: world={} bhkWorld={}", handName(), (void*)world, bhkWorld);
            return false;
        }

        auto* shape = CreateBoxShape(halfExtentX, halfExtentY, halfExtentZ, g_rockConfig.rockHandCollisionBoxRadius);
        if (!shape) {
            ROCK_LOG_ERROR(Hand, "{} hand createCollision: box shape creation failed", handName());
            return false;
        }

        auto materialId = registerHandMaterial(world);

        std::uint32_t filterInfo = (0x000B << 16) | (ROCK_HAND_LAYER & 0x7F);

        bool ok = _handBody.create(world, bhkWorld, shape, filterInfo, materialId, BethesdaMotionType::Keyframed, _isLeft ? "ROCK_LeftHand" : "ROCK_RightHand");

        if (!ok) {
            ROCK_LOG_ERROR(Hand, "{} hand createCollision: BethesdaPhysicsBody::create failed", handName());
            return false;
        }

        _handBody.createNiNode(_isLeft ? "ROCK_LeftHand" : "ROCK_RightHand");

        ROCK_LOG_INFO(Hand, "{} hand collision created via BethesdaPhysicsBody — bodyId={}", handName(), _handBody.getBodyId().value);

        return true;
    }

    void Hand::destroyCollision(void* bhkWorld)
    {
        if (!hasCollisionBody())
            return;

        ROCK_LOG_INFO(Hand, "{} hand collision destroying — bodyId={}", handName(), _handBody.getBodyId().value);
        _handBody.destroy(bhkWorld);
    }

    void Hand::updateCollisionTransform(RE::hknpWorld* world, const RE::NiTransform& handTransform, float deltaTime)
    {
        if (!hasCollisionBody() || !world)
            return;

        const auto bodyId = _handBody.getBodyId();

        const float targetX = handTransform.translate.x * kGameToHavokScale;
        const float targetY = handTransform.translate.y * kGameToHavokScale;
        const float targetZ = handTransform.translate.z * kGameToHavokScale;

        auto* bodyArray = world->GetBodyArray();
        auto* bodyFloats = reinterpret_cast<float*>(&bodyArray[bodyId.value]);
        float curX = bodyFloats[12], curY = bodyFloats[13], curZ = bodyFloats[14];

        float dx = targetX - curX, dy = targetY - curY, dz = targetZ - curZ;
        float dist = sqrtf(dx * dx + dy * dy + dz * dz);
        bool isTeleport = (dist > 5.0f);
        const bool logWitness = g_rockConfig.rockDebugVerboseLogging && (++_transformWitnessLogCounter >= 90);
        if (logWitness) {
            _transformWitnessLogCounter = 0;
        }

        alignas(16) float linVelOut[4] = { 0, 0, 0, 0 };
        alignas(16) float angVelOut[4] = { 0, 0, 0, 0 };

        if (deltaTime > 0.0001f && !isTeleport) {
            alignas(16) float tgtPos[4] = { targetX, targetY, targetZ, 0.0f };
            alignas(16) float tgtQuat[4];
            niRotToHkQuat(handTransform.rotate, tgtQuat);

            typedef void (*computeHKF_t)(void*, std::uint32_t, const float*, const float*, float, float*, float*);
            static REL::Relocation<computeHKF_t> computeHardKeyFrame{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            computeHardKeyFrame(world, bodyId.value, tgtPos, tgtQuat, deltaTime, linVelOut, angVelOut);
        }

        {
            float MAX_LIN_VEL = g_rockConfig.rockMaxLinearVelocity;
            float MAX_ANG_VEL = g_rockConfig.rockMaxAngularVelocity;

            float linSpeed = sqrtf(linVelOut[0] * linVelOut[0] + linVelOut[1] * linVelOut[1] + linVelOut[2] * linVelOut[2]);
            if (linSpeed > MAX_LIN_VEL) {
                float s = MAX_LIN_VEL / linSpeed;
                linVelOut[0] *= s;
                linVelOut[1] *= s;
                linVelOut[2] *= s;
            }

            float angSpeed = sqrtf(angVelOut[0] * angVelOut[0] + angVelOut[1] * angVelOut[1] + angVelOut[2] * angVelOut[2]);
            if (angSpeed > MAX_ANG_VEL) {
                float s = MAX_ANG_VEL / angSpeed;
                angVelOut[0] *= s;
                angVelOut[1] *= s;
                angVelOut[2] *= s;
            }
        }

        {
            RE::hkTransformf hkTransform;

            hkTransform.rotation = niRotToHkTransformRotation(handTransform.rotate);
            hkTransform.translation = RE::NiPoint4(targetX, targetY, targetZ, 0.0f);
            _handBody.setTransform(hkTransform);
        }

        _handBody.setVelocity(linVelOut, angVelOut);

        if (logWitness) {
            const auto targetQuat = niRotToHkQuat(handTransform.rotate);
            const auto targetHkRotation = niRotToHkTransformRotation(handTransform.rotate);
            const auto* targetHk = reinterpret_cast<const float*>(&targetHkRotation);
            const float targetDet = matrixDeterminant(handTransform.rotate);
            const float liveDet = bodyBasisDeterminant(bodyFloats);

            ROCK_LOG_INFO(Hand,
                "{} transform witness: bodyId={} niPos=({:.1f},{:.1f},{:.1f}) hkTarget=({:.4f},{:.4f},{:.4f}) "
                "liveOrigin=({:.4f},{:.4f},{:.4f}) delta=({:.4f},{:.4f},{:.4f}) dist={:.4f} dt={:.4f} "
                "teleport={} targetDet={:.4f} liveDet={:.4f}",
                handName(), bodyId.value, handTransform.translate.x, handTransform.translate.y, handTransform.translate.z, targetX, targetY, targetZ, curX, curY, curZ, dx, dy, dz,
                dist, deltaTime, isTeleport ? "yes" : "no", targetDet, liveDet);

            ROCK_LOG_INFO(Hand,
                "{} transform basis: targetNiRows=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "targetHkCols=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "liveHkCols=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "quat=({:.4f},{:.4f},{:.4f},{:.4f}) linVel=({:.4f},{:.4f},{:.4f}) angVel=({:.4f},{:.4f},{:.4f})",
                handName(), handTransform.rotate.entry[0][0], handTransform.rotate.entry[0][1], handTransform.rotate.entry[0][2], handTransform.rotate.entry[1][0],
                handTransform.rotate.entry[1][1], handTransform.rotate.entry[1][2], handTransform.rotate.entry[2][0], handTransform.rotate.entry[2][1],
                handTransform.rotate.entry[2][2], targetHk[0], targetHk[1], targetHk[2], targetHk[4], targetHk[5], targetHk[6], targetHk[8], targetHk[9], targetHk[10],
                bodyFloats[0], bodyFloats[1], bodyFloats[2], bodyFloats[4], bodyFloats[5], bodyFloats[6], bodyFloats[8], bodyFloats[9], bodyFloats[10], targetQuat.x,
                targetQuat.y, targetQuat.z, targetQuat.w, linVelOut[0], linVelOut[1], linVelOut[2], angVelOut[0], angVelOut[1], angVelOut[2]);
        }
    }

    void Hand::updateDebugBasisVis(const RE::NiTransform& colliderTransform, const RE::NiPoint3& grabAnchorWorld, bool show, RE::NiNode* parentNode)
    {
        auto destroyMarker = [](RE::NiNode*& marker) {
            if (!marker) {
                return;
            }

            marker->flags.flags |= 0x1;
            marker->local.scale = 0;
            if (marker->parent) {
                marker->parent->DetachChild(marker);
            }
            marker = nullptr;
        };

        auto createMarker = [&](RE::NiNode*& marker, const char* meshPath, const char* name) {
            if (marker || !parentNode) {
                return;
            }

            marker = f4cf::f4vr::getClonedNiNodeForNifFileSetName(meshPath);
            if (marker) {
                marker->name = RE::BSFixedString(name);
                parentNode->AttachChild(marker, true);
                marker->flags.flags &= 0xfffffffffffffffe;
                marker->local.scale = 0.25f;
            } else {
                ROCK_LOG_WARN(Hand, "{} basis debug marker FAILED to load {}", handName(), meshPath);
            }
        };

        if (!show || !parentNode) {
            destroyDebugBasisVis();
            return;
        }

        if (_debugBasisVisParent && _debugBasisVisParent != parentNode) {
            destroyDebugBasisVis();
        }

        createMarker(_debugHandOriginVis, "Data/Meshes/ROCK/DebugBox_VaultSuit.nif", _isLeft ? "ROCK_HandOriginL" : "ROCK_HandOriginR");
        createMarker(_debugPalmCenterVis, "Data/Meshes/ROCK/1x1Sphere.nif", _isLeft ? "ROCK_GrabAnchorL" : "ROCK_GrabAnchorR");
        createMarker(_debugAxisXVis, "Data/Meshes/ROCK/marker_arrow.nif", _isLeft ? "ROCK_AxisXL" : "ROCK_AxisXR");
        createMarker(_debugAxisYVis, "Data/Meshes/ROCK/marker_travel.nif", _isLeft ? "ROCK_AxisYL" : "ROCK_AxisYR");
        createMarker(_debugAxisZVis, "Data/Meshes/ROCK/marker_decal.nif", _isLeft ? "ROCK_AxisZL" : "ROCK_AxisZR");

        _debugBasisVisParent = parentNode;
        const RE::NiPoint3 colliderCenter = colliderTransform.translate;
        const RE::NiPoint3 axisXEndpoint =
            colliderCenter + normalizeDebugDirection(debug_axis_math::rotateNiLocalToWorld(colliderTransform.rotate, RE::NiPoint3(1.0f, 0.0f, 0.0f))) * 6.0f;
        const RE::NiPoint3 axisYEndpoint =
            colliderCenter + normalizeDebugDirection(debug_axis_math::rotateNiLocalToWorld(colliderTransform.rotate, RE::NiPoint3(0.0f, 1.0f, 0.0f))) * 9.0f;
        const RE::NiPoint3 axisZEndpoint =
            colliderCenter + normalizeDebugDirection(debug_axis_math::rotateNiLocalToWorld(colliderTransform.rotate, RE::NiPoint3(0.0f, 0.0f, 1.0f))) * 12.0f;

        auto updateMarker = [&](RE::NiNode* marker, const RE::NiPoint3& localPos, const RE::NiMatrix3& localRot, float scale) {
            if (!marker) {
                return;
            }

            marker->flags.flags &= 0xfffffffffffffffe;
            marker->local.translate = localPos;
            marker->local.rotate = localRot;
            marker->local.scale = scale;
        };

        const RE::NiPoint3 localColliderCenterPos = transform_math::worldPointToLocal(parentNode->world, colliderCenter);
        const RE::NiPoint3 localGrabAnchorPos = transform_math::worldPointToLocal(parentNode->world, grabAnchorWorld);
        const RE::NiPoint3 localAxisXPos = transform_math::worldPointToLocal(parentNode->world, axisXEndpoint);
        const RE::NiPoint3 localAxisYPos = transform_math::worldPointToLocal(parentNode->world, axisYEndpoint);
        const RE::NiPoint3 localAxisZPos = transform_math::worldPointToLocal(parentNode->world, axisZEndpoint);

        updateMarker(_debugHandOriginVis, localColliderCenterPos, makeIdentityRotation(), 0.05f);
        updateMarker(_debugPalmCenterVis, localGrabAnchorPos, makeIdentityRotation(), 0.03f);
        updateMarker(_debugAxisXVis, localAxisXPos, buildArrowLocalRotation(RE::NiPoint3(1.0f, 0.0f, 0.0f)), 0.035f);
        updateMarker(_debugAxisYVis, localAxisYPos, buildArrowLocalRotation(RE::NiPoint3(0.0f, 1.0f, 0.0f)), 0.045f);
        updateMarker(_debugAxisZVis, localAxisZPos, buildArrowLocalRotation(RE::NiPoint3(0.0f, 0.0f, 1.0f)), 0.055f);
    }

    void Hand::destroyDebugBasisVis()
    {
        auto destroyMarker = [](RE::NiNode*& marker) {
            if (!marker) {
                return;
            }

            marker->flags.flags |= 0x1;
            marker->local.scale = 0;
            if (marker->parent) {
                marker->parent->DetachChild(marker);
            }
            marker = nullptr;
        };

        destroyMarker(_debugHandOriginVis);
        destroyMarker(_debugPalmCenterVis);
        destroyMarker(_debugAxisXVis);
        destroyMarker(_debugAxisYVis);
        destroyMarker(_debugAxisZVis);
        _debugBasisVisParent = nullptr;
    }

}
