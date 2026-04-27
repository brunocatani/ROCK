#include "PhysicsInteraction.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "CollisionLayerPolicy.h"
#include "HavokOffsets.h"
#include "DebugBodyOverlay.h"
#include "DebugOverlayPolicy.h"
#include "PalmTransform.h"
#include "PhysicsHooks.h"
#include "PhysicsUtils.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpWorld.h"

#include "ROCKMain.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "vrcf/VRControllersManager.h"

namespace frik::rock
{
    namespace
    {
        constexpr float kRawParityWarnPosition = 0.10f;
        constexpr float kRawParityWarnRotationDegrees = 0.5f;
        constexpr float kRawParityFailPosition = 0.50f;
        constexpr float kRawParityFailRotationDegrees = 2.0f;
        constexpr int kRawParityWarnFrames = 2;
        constexpr int kRawParityFailFrames = 10;
        constexpr int kRawParitySummaryFrames = 300;
        constexpr int kRawParityLagFrames = 5;
        constexpr float kRawParityLagSlack = 0.05f;

        struct TransformDelta
        {
            float position = 0.0f;
            float rotationDegrees = 0.0f;
        };

        TransformDelta measureTransformDelta(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            const float dx = a.translate.x - b.translate.x;
            const float dy = a.translate.y - b.translate.y;
            const float dz = a.translate.z - b.translate.z;

            const auto qa = niRotToHkQuat(a.rotate);
            const auto qb = niRotToHkQuat(b.rotate);
            const float dot = std::clamp(std::fabs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w), 0.0f, 1.0f);
            const float angleRadians = 2.0f * std::acos(dot);

            return TransformDelta{ .position = std::sqrt(dx * dx + dy * dy + dz * dz), .rotationDegrees = angleRadians * (180.0f / std::numbers::pi_v<float>)};
        }

        float measurePointDelta(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        float measureDirectionDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f);
            return std::acos(dot) * (180.0f / std::numbers::pi_v<float>);
        }
    }

    PhysicsInteraction::PhysicsInteraction()
    {
        s_instance.store(this, std::memory_order_release);

        installBumpHook();
        installCCRadiusHook();
        installNativeGrabHook();
        installRefreshManifoldHook();

        ROCK_LOG_INFO(Init, "ROCK Physics Module v0.1 — created");
    }

    PhysicsInteraction::~PhysicsInteraction()
    {
        s_instance.store(nullptr, std::memory_order_release);

        if (_initialized) {
            shutdown();
        }
        ROCK_LOG_INFO(Init, "ROCK Physics Module — destroyed");
    }

    bool PhysicsInteraction::validateCriticalOffsets() const
    {
        REL::Relocation hookSite{ REL::Offset(offsets::kHookSite_MainLoop) };
        auto* hookByte = reinterpret_cast<const std::uint8_t*>(hookSite.address());
        if (*hookByte != 0xE8 && *hookByte != 0xE9) {
            ROCK_LOG_ERROR(Init, "Hook site 0x{:X} is not a CALL/JMP instruction (found {:#x})", offsets::kHookSite_MainLoop, *hookByte);
            return false;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_INFO(Init, "No bhkWorld available for offset validation (will retry)");
            return true;
        }

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_ERROR(Init, "bhkWorld -> hknpWorld is null — offset may be wrong");
            return false;
        }

        auto* bodyArray = hknp->GetBodyArray();
        if (!bodyArray) {
            ROCK_LOG_ERROR(Init, "hknpWorld::GetBodyArray() returned null");
            return false;
        }

        ROCK_LOG_INFO(Init, "Critical offset validation passed");
        return true;
    }

    bool PhysicsInteraction::refreshHandBoneCache()
    {
        const bool cacheNeedsResolve = !_handBoneCache.isReady() || _handBoneCache.hasSkeletonChanged();
        if (!cacheNeedsResolve) {
            return true;
        }

        if (_handBoneCache.hasSkeletonChanged()) {
            _handBoneCache.reset();
        }

        if (_handBoneCache.resolve()) {
            _handCacheResolveLogCounter = 0;
            return true;
        }

        if (g_rockConfig.rockDebugHandTransformParity) {
            if (++_handCacheResolveLogCounter == 1 || _handCacheResolveLogCounter % 90 == 0) {
                ROCK_LOG_WARN(Hand, "HandBoneCache unresolved; raw parity sampling skipped this frame");
            }
        }

        return false;
    }

    RE::NiTransform PhysicsInteraction::getInteractionHandTransform(bool isLeft) const
    {
        RE::NiNode* handBone = _handBoneCache.isReady() ? _handBoneCache.getNode(isLeft) : nullptr;
        RE::NiNode* frikNode = nullptr;
        RE::NiTransform frikTransform{};
        bool frikValid = false;

        if (frik::api::FRIKApi::inst) {
            const auto hand = handFromBool(isLeft);
            frikTransform = frik::api::FRIKApi::inst->getHandWorldTransform(hand);
            frikNode = frik::api::FRIKApi::inst->getHandNode(hand);
            frikValid = true;
        }

        const auto frame = _handFrameResolver.resolve(isLeft, handBone, frikNode, frikTransform, frikValid);
        if (frame.valid) {
            return frame.transform;
        }

        return RE::NiTransform();
    }

    RE::NiNode* PhysicsInteraction::getInteractionHandNode(bool isLeft) const
    {
        RE::NiNode* handBone = _handBoneCache.isReady() ? _handBoneCache.getNode(isLeft) : nullptr;
        RE::NiNode* frikNode = nullptr;
        RE::NiTransform frikTransform{};
        bool frikValid = false;

        if (frik::api::FRIKApi::inst) {
            const auto hand = handFromBool(isLeft);
            frikTransform = frik::api::FRIKApi::inst->getHandWorldTransform(hand);
            frikNode = frik::api::FRIKApi::inst->getHandNode(hand);
            frikValid = true;
        }

        const auto frame = _handFrameResolver.resolve(isLeft, handBone, frikNode, frikTransform, frikValid);
        if (frame.valid) {
            return frame.node;
        }

        return nullptr;
    }

    void PhysicsInteraction::sampleHandTransformParity()
    {
        if (!g_rockConfig.rockDebugHandTransformParity) {
            _parityEnabledLogged = false;
            _paritySummaryCounter = 0;
            return;
        }

        if (!frik::api::FRIKApi::inst || !_handBoneCache.isReady()) {
            return;
        }

        if (!_parityEnabledLogged) {
            ROCK_LOG_INFO(Init, "Hand-transform parity enabled (raw + derived basis, local cache vs FRIK API, pre-write sampling)");
            _parityEnabledLogged = true;
        }

        const bool playerMoving = frik::api::FRIKApi::inst->isPlayerMoving();
        const bool emitSummary = (++_paritySummaryCounter >= kRawParitySummaryFrames);

        auto sampleHand = [&](bool isLeft) {
            auto& state = _rawHandParityStates[isLeft ? 1 : 0];
            const auto handEnum = handFromBool(isLeft);
            const auto localTransform = _handBoneCache.getWorldTransform(isLeft);
            const auto apiTransform = frik::api::FRIKApi::inst->getHandWorldTransform(handEnum);
            const auto delta = measureTransformDelta(localTransform, apiTransform);
            const auto localCollisionTransform = computeHandCollisionTransformFromHandBasis(localTransform, isLeft);
            const auto apiCollisionTransform = computeHandCollisionTransformFromHandBasis(apiTransform, isLeft);
            const auto localPalmPosition = computePalmPositionFromHandBasis(localTransform, isLeft);
            const auto apiPalmPosition = computePalmPositionFromHandBasis(apiTransform, isLeft);
            const auto localPalmNormal = computePalmNormalFromHandBasis(localTransform, isLeft);
            const auto apiPalmNormal = computePalmNormalFromHandBasis(apiTransform, isLeft);
            const auto localPointing = computePointingVectorFromHandBasis(localTransform, isLeft);
            const auto apiPointing = computePointingVectorFromHandBasis(apiTransform, isLeft);
            state.lastPositionDelta = delta.position;
            state.lastRotationDeltaDegrees = delta.rotationDegrees;

            const bool warnExceeded = delta.position > kRawParityWarnPosition || delta.rotationDegrees > kRawParityWarnRotationDegrees;
            const bool failExceeded = delta.position > kRawParityFailPosition || delta.rotationDegrees > kRawParityFailRotationDegrees;

            state.warnFrames = warnExceeded ? state.warnFrames + 1 : 0;
            state.failFrames = failExceeded ? state.failFrames + 1 : 0;

            const char* handLabel = isLeft ? "Left" : "Right";
            if (state.warnFrames == kRawParityWarnFrames) {
                ROCK_LOG_WARN(Hand, "{} raw hand parity warning: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (state.failFrames == kRawParityFailFrames) {
                ROCK_LOG_ERROR(Hand, "{} raw hand parity failure: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (playerMoving && state.hasPreviousApiTransform) {
                const auto prevApiDelta = measureTransformDelta(localTransform, state.previousApiTransform);
                if (prevApiDelta.position + kRawParityLagSlack < delta.position) {
                    state.lagFrames++;
                    if (state.lagFrames == kRawParityLagFrames) {
                        ROCK_LOG_WARN(Hand, "{} hand parity suggests possible one-frame lag: currentDelta={:.3f} prevApiDelta={:.3f}", handLabel, delta.position,
                            prevApiDelta.position);
                    }
                } else {
                    state.lagFrames = 0;
                }
            } else {
                state.lagFrames = 0;
            }

            state.previousApiTransform = apiTransform;
            state.hasPreviousApiTransform = true;

            if (emitSummary) {
                const char* summaryHandLabel = isLeft ? "L" : "R";
                ROCK_LOG_INFO(Hand, "{} parity: raw(pos={:.3f}, rot={:.3f}deg) basis(collider={:.3f}, palmPos={:.3f}, palmNormal={:.3f}deg, pointing={:.3f}deg)", summaryHandLabel,
                    delta.position, delta.rotationDegrees, measurePointDelta(localCollisionTransform.translate, apiCollisionTransform.translate),
                    measurePointDelta(localPalmPosition, apiPalmPosition), measureDirectionDeltaDegrees(localPalmNormal, apiPalmNormal),
                    measureDirectionDeltaDegrees(localPointing, apiPointing));
            }
        };

        sampleHand(false);
        sampleHand(true);

        if (emitSummary) {
            _paritySummaryCounter = 0;
            const auto& right = _rawHandParityStates[0];
            const auto& left = _rawHandParityStates[1];
            ROCK_LOG_INFO(Hand, "Raw hand parity summary: R(pos={:.3f}, rot={:.3f}deg) L(pos={:.3f}, rot={:.3f}deg)", right.lastPositionDelta, right.lastRotationDeltaDegrees,
                left.lastPositionDelta, left.lastRotationDeltaDegrees);
        }
    }

    void PhysicsInteraction::init()
    {
        if (_initialized) {
            ROCK_LOG_WARN(Init, "init() called but already initialized — skipping");
            return;
        }

        if (!validateCriticalOffsets()) {
            ROCK_LOG_CRITICAL(Init,
                "ROCK DISABLED: critical Havok offset validation failed. "
                "This likely means a game update changed memory layouts.");
            return;
        }

        ROCK_LOG_INFO(Init, "Initializing ROCK physics module...");

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_ERROR(Init, "Failed to get bhkWorld during init — deferring");
            return;
        }

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_ERROR(Init, "Failed to get hknpWorld during init — deferring");
            return;
        }

        _cachedBhkWorld = bhk;
        if (!refreshHandBoneCache()) {
            ROCK_LOG_WARN(Init, "HandBoneCache not ready during init; runtime remains on pre-00 transform paths");
        }

        registerCollisionLayer(hknp);
        if (!_collisionLayerRegistered) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: collision layer registration failed");
            _cachedBhkWorld = nullptr;
            return;
        }

        if (!createHandCollisions(hknp, bhk)) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: hand collision body creation failed");
            _cachedBhkWorld = nullptr;
            return;
        }
        _cachedHalfExtentX = g_rockConfig.rockHandCollisionHalfExtentX;
        _cachedHalfExtentY = g_rockConfig.rockHandCollisionHalfExtentY;
        _cachedHalfExtentZ = g_rockConfig.rockHandCollisionHalfExtentZ;

        subscribeContactEvents(hknp);

        _weaponCollision.init(hknp, bhk);

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_Physics", true);
            ROCK_LOG_INFO(Init, "FRIK offhand grip permanently suppressed");
        }

        {
            auto rightTransform = computeHandCollisionTransformFromHandBasis(getInteractionHandTransform(false), false);
            auto leftTransform = computeHandCollisionTransformFromHandBasis(getInteractionHandTransform(true), true);
            _rightHand.updateCollisionTransform(hknp, rightTransform, 0.011f);
            _leftHand.updateCollisionTransform(hknp, leftTransform, 0.011f);
            ROCK_LOG_INFO(Init, "Initial hand positions set via SetBodyTransform");
        }

        _hasPrevPositions = false;
        _deltaLogCounter = 0;
        _contactLogCounter = 0;

        _initialized = true;

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsInit, false);

        ROCK_LOG_INFO(Init, "ROCK physics module initialized — bhkWorld={}, hknpWorld={}, R_body={}, L_body={}", static_cast<const void*>(bhk), static_cast<const void*>(hknp),
            _rightHand.getCollisionBodyId().value, _leftHand.getCollisionBodyId().value);
    }

    void PhysicsInteraction::update()
    {
        if (!frik::api::FRIKApi::inst) {
            return;
        }

        vrcf::VRControllers.update(f4vr::isLeftHandedMode());

        _deltaTime = frik::api::FRIKApi::inst->getFrameTime();

        if (_deltaTime <= 0.0f || _deltaTime > 0.1f) {
            _deltaTime = 1.0f / 90.0f;
        }

        if (!frik::api::FRIKApi::inst->isSkeletonReady()) {
            if (_initialized) {
                ROCK_LOG_WARN(Update, "Skeleton no longer ready — shutting down");
                shutdown();
            }
            return;
        }

        if (frik::api::FRIKApi::inst->isAnyMenuOpen()) {
            if (_initialized) {
                auto* bhkMenu = getPlayerBhkWorld();
                if (bhkMenu) {
                    auto* hknpMenu = getHknpWorld(bhkMenu);
                    if (hknpMenu) {
                        if (_rightHand.isHolding()) {
                            auto* r = _rightHand.getHeldRef();
                            _rightHand.releaseGrabbedObject(hknpMenu);
                            if (r)
                                releaseObject(r);
                        }
                        if (_leftHand.isHolding()) {
                            auto* r = _leftHand.getHeldRef();
                            _leftHand.releaseGrabbedObject(hknpMenu);
                            if (r)
                                releaseObject(r);
                        }
                    }
                }
            }
            debug::ClearFrame();
            return;
        }

        if (!g_rockConfig.rockEnabled) {
            debug::ClearFrame();
            return;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            if (_initialized) {
                ROCK_LOG_WARN(Update, "bhkWorld became null — shutting down");
                shutdown();
            }
            return;
        }

        if (_initialized && bhk != _cachedBhkWorld) {
            ROCK_LOG_INFO(Update, "bhkWorld changed (cell transition) — reinitializing");

            shutdown();
        }

        if (!_initialized) {
            init();
            if (!_initialized) {
                return;
            }
        }

        _cachedBhkWorld = bhk;

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            debug::ClearFrame();
            return;
        }

        refreshHandBoneCache();
        sampleHandTransformParity();

        if (_collisionLayerRegistered && _expectedHandLayerMask != 0) {
            auto modifierMgr = *reinterpret_cast<std::uintptr_t*>(reinterpret_cast<std::uintptr_t>(hknp) + offsets::kHknpWorld_ModifierManager);
            if (modifierMgr) {
                auto* filterPtr = *reinterpret_cast<void**>(modifierMgr + offsets::kModifierMgr_FilterPtr);
                if (filterPtr) {
                    auto* matrix = reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);
                    if (matrix[ROCK_HAND_LAYER] != _expectedHandLayerMask) {
                        ROCK_LOG_WARN(Config, "Layer {} mask changed! Expected=0x{:016X} Current=0x{:016X} — re-registering", ROCK_HAND_LAYER, _expectedHandLayerMask,
                            matrix[ROCK_HAND_LAYER]);
                        _collisionLayerRegistered = false;
                        registerCollisionLayer(hknp);
                    }
                }
            }
        }

        if (_cachedHalfExtentX != g_rockConfig.rockHandCollisionHalfExtentX || _cachedHalfExtentY != g_rockConfig.rockHandCollisionHalfExtentY ||
            _cachedHalfExtentZ != g_rockConfig.rockHandCollisionHalfExtentZ) {
            ROCK_LOG_INFO(Config, "Collision box config changed — rebuilding (X={:.4f}→{:.4f}, Y={:.4f}→{:.4f}, Z={:.4f}→{:.4f})", _cachedHalfExtentX,
                g_rockConfig.rockHandCollisionHalfExtentX, _cachedHalfExtentY, g_rockConfig.rockHandCollisionHalfExtentY, _cachedHalfExtentZ,
                g_rockConfig.rockHandCollisionHalfExtentZ);

            if (_rightHand.isHolding()) {
                auto* r = _rightHand.getHeldRef();
                _rightHand.releaseGrabbedObject(hknp);
                if (r)
                    releaseObject(r);
            }
            if (_leftHand.isHolding()) {
                auto* r = _leftHand.getHeldRef();
                _leftHand.releaseGrabbedObject(hknp);
                if (r)
                    releaseObject(r);
            }
            destroyHandCollisions(bhk);
            if (createHandCollisions(hknp, bhk)) {
                _cachedHalfExtentX = g_rockConfig.rockHandCollisionHalfExtentX;
                _cachedHalfExtentY = g_rockConfig.rockHandCollisionHalfExtentY;
                _cachedHalfExtentZ = g_rockConfig.rockHandCollisionHalfExtentZ;
            } else {
                ROCK_LOG_ERROR(Config, "Collision body rebuild failed; shutting down physics interaction");
                shutdown();
                return;
            }
        }

        updateHandCollisions(hknp);

        {
            auto* rootNode = f4vr::getRootNode();
            RE::NiAVObject* weaponNode = rootNode ? f4vr::findNode(rootNode, "Weapon") : nullptr;

            bool isLeftHanded = f4vr::isLeftHandedMode();
            auto dominantHandBodyId = isLeftHanded ? _leftHand.getCollisionBodyId() : _rightHand.getCollisionBodyId();

            if (g_rockConfig.rockDebugVerboseLogging) {
                if (++_wpnNodeLogCounter >= 90) {
                    _wpnNodeLogCounter = 0;
                    if (weaponNode) {
                        ROCK_LOG_INFO(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}", weaponNode->name.c_str(), weaponNode->world.translate.x,
                            weaponNode->world.translate.y, weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
                    } else {
                    }
                }
            }
            _weaponCollision.update(hknp, weaponNode, dominantHandBodyId, _deltaTime);
        }

        {
            bool offhandTouching = _offhandTouchingWeapon.exchange(false, std::memory_order_acquire);
            bool isLeftHanded = f4vr::isLeftHandedMode();
            bool gripPressed = vrcf::VRControllers.isPressHeldDown(vrcf::Hand::Offhand, g_rockConfig.rockGrabButtonID);

            auto* rootNode2 = f4vr::getRootNode();
            RE::NiNode* weaponNode = rootNode2 ? f4vr::findNode(rootNode2, "Weapon") : nullptr;

            _twoHandedGrip.update(weaponNode, offhandTouching, gripPressed, isLeftHanded, _deltaTime);
        }

        updateSelection(bhk, hknp);

        updateGrabInput(hknp);

        publishDebugBodyOverlay(hknp);

        resolveContacts(bhk, hknp);

        bool wasTouchingR = _rightHand.isTouching();
        bool wasTouchingL = _leftHand.isTouching();
        _rightHand.tickTouchState();
        _leftHand.tickTouchState();
        if (wasTouchingR && !_rightHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, false, _rightHand.getLastTouchedRef(), _rightHand.getLastTouchedFormID(), _rightHand.getLastTouchedLayer());
        }
        if (wasTouchingL && !_leftHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, true, _leftHand.getLastTouchedRef(), _leftHand.getLastTouchedFormID(), _leftHand.getLastTouchedLayer());
        }

        _deltaLogCounter++;
        if (g_rockConfig.rockDebugVerboseLogging && _deltaLogCounter >= 90) {
            _deltaLogCounter = 0;

            if (frik::api::FRIKApi::inst) {
                const auto smoothPos = frik::api::FRIKApi::inst->getSmoothedPlayerPosition();
                const bool moving = frik::api::FRIKApi::inst->isPlayerMoving();

                if (_hasPrevPositions && moving) {
                    const auto smoothDelta = smoothPos - _prevSmoothedPos;

                    ROCK_LOG_DEBUG(Update, "PlayerSpace: smoothDelta=({:.2f},{:.2f},{:.2f}) moving={}", smoothDelta.x, smoothDelta.y, smoothDelta.z, moving);
                }

                _prevSmoothedPos = smoothPos;
                _hasPrevPositions = true;
            }
        }
    }

    void PhysicsInteraction::shutdown()
    {
        if (!_initialized) {
            return;
        }

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsShutdown, false);

        ROCK_LOG_INFO(Init, "Shutting down ROCK physics module...");

        auto* currentBhk = getPlayerBhkWorld();
        const bool worldValid = _cachedBhkWorld && currentBhk == _cachedBhkWorld;

        if (worldValid) {
            auto* hknp = getHknpWorld(_cachedBhkWorld);
            if (_rightHand.isHolding()) {
                auto* r = _rightHand.getHeldRef();
                _rightHand.releaseGrabbedObject(hknp);
                if (r)
                    releaseObject(r);
            }
            if (_leftHand.isHolding()) {
                auto* r = _leftHand.getHeldRef();
                _leftHand.releaseGrabbedObject(hknp);
                if (r)
                    releaseObject(r);
            }
            _weaponCollision.destroyWeaponBody(hknp);
            destroyHandCollisions(_cachedBhkWorld);
        } else {
            ROCK_LOG_INFO(Init, "World stale or null — skipping Havok body destruction");
        }

        debug::ClearFrame();
        _twoHandedGrip.reset();
        _weaponCollision.shutdown();
        releaseAllObjects();
        _rightHand.reset();
        _leftHand.reset();

        _cachedBhkWorld = nullptr;
        _collisionLayerRegistered = false;
        _initialized = false;
        _hasPrevPositions = false;
        _handBoneCache.reset();
        _handCacheResolveLogCounter = 0;
        _paritySummaryCounter = 0;
        _parityEnabledLogged = false;
        _runtimeScaleLogged = false;
        _rawHandParityStates = {};

        cleanupGrabConstraintVtable();

        ROCK_LOG_INFO(Init, "ROCK physics module shut down");
    }

    void PhysicsInteraction::dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer)
    {
        PhysicsEventData data{ isLeft, refr, formID, layer };

        if (auto* m = ::rock::getROCKMessaging()) {
            m->Dispatch(msgType, &data, sizeof(data), nullptr);
        }
    }

    void PhysicsInteraction::registerCollisionLayer(RE::hknpWorld* world)
    {
        if (!world) {
            ROCK_LOG_ERROR(Config, "registerCollisionLayer: world is null");
            return;
        }

        auto modifierMgr = *reinterpret_cast<std::uintptr_t*>(reinterpret_cast<std::uintptr_t>(world) + offsets::kHknpWorld_ModifierManager);
        if (!modifierMgr) {
            ROCK_LOG_ERROR(Config, "World modifier manager (+0x150) is null — cannot configure layer");
            return;
        }

        auto* filterPtr = *reinterpret_cast<void**>(modifierMgr + offsets::kModifierMgr_FilterPtr);
        if (!filterPtr) {
            static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
            filterPtr = *filterSingleton;
            if (!filterPtr) {
                ROCK_LOG_ERROR(Config, "Both world filter and global singleton are null — cannot configure layer");
                return;
            }
            ROCK_LOG_WARN(Config, "World filter null, fell back to global singleton at {:p}", filterPtr);
        }

        {
            static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
            auto* globalFilter = *filterSingleton;
            ROCK_LOG_INFO(Config, "Filter source: world={:p}, global={:p}, same={}", filterPtr, (void*)globalFilter, filterPtr == globalFilter);
        }

        auto* matrix = reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);

        auto disablePair = [&](std::uint32_t layerA, std::uint32_t layerB) {
            matrix[layerA] &= ~(1ULL << layerB);
            matrix[layerB] &= ~(1ULL << layerA);
        };

        auto enablePair = [&](std::uint32_t layerA, std::uint32_t layerB) {
            matrix[layerA] |= (1ULL << layerB);
            matrix[layerB] |= (1ULL << layerA);
        };

        ROCK_LOG_INFO(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_HAND_LAYER, matrix[ROCK_HAND_LAYER]);
        ROCK_LOG_INFO(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_WEAPON_LAYER, matrix[ROCK_WEAPON_LAYER]);

        for (std::uint32_t i = 0; i < 48; i++) {
            enablePair(ROCK_HAND_LAYER, i);
        }

        disablePair(ROCK_HAND_LAYER, 15);
        disablePair(ROCK_HAND_LAYER, ROCK_HAND_LAYER);
        if (!g_rockConfig.rockCollideWithCharControllers) {
            disablePair(ROCK_HAND_LAYER, 30);
        }

        for (std::uint32_t i = 0; i < 48; i++) {
            enablePair(ROCK_WEAPON_LAYER, i);
        }
        disablePair(ROCK_WEAPON_LAYER, 0);
        disablePair(ROCK_WEAPON_LAYER, 15);
        disablePair(ROCK_WEAPON_LAYER, 30);

        disablePair(ROCK_WEAPON_LAYER, ROCK_WEAPON_LAYER);
        disablePair(ROCK_WEAPON_LAYER, 36);
        disablePair(ROCK_WEAPON_LAYER, 41);
        disablePair(ROCK_WEAPON_LAYER, 42);
        collision_layer_policy::applyWeaponProjectileBlockingPolicy(
            matrix, ROCK_WEAPON_LAYER, g_rockConfig.rockWeaponCollisionBlocksProjectiles, g_rockConfig.rockWeaponCollisionBlocksSpells);

        _expectedHandLayerMask = matrix[ROCK_HAND_LAYER];
        _collisionLayerRegistered = true;

        ROCK_LOG_INFO(Config, "Registered layer {} (hand) mask=0x{:016X}", ROCK_HAND_LAYER, matrix[ROCK_HAND_LAYER]);
        ROCK_LOG_INFO(Config, "Registered layer {} (weapon) mask=0x{:016X}", ROCK_WEAPON_LAYER, matrix[ROCK_WEAPON_LAYER]);
        ROCK_LOG_INFO(Config, "Weapon projectile blocking: projectiles={} spells={} layerMask=0x{:016X}",
            g_rockConfig.rockWeaponCollisionBlocksProjectiles ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksSpells ? "enabled" : "disabled", matrix[ROCK_WEAPON_LAYER]);
    }

    bool PhysicsInteraction::createHandCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
            ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
            return false;
        }

        const bool rightOk = _rightHand.createCollision(world, bhkWorld, g_rockConfig.rockHandCollisionHalfExtentX, g_rockConfig.rockHandCollisionHalfExtentY,
            g_rockConfig.rockHandCollisionHalfExtentZ);

        const bool leftOk = _leftHand.createCollision(world, bhkWorld, g_rockConfig.rockHandCollisionHalfExtentX, g_rockConfig.rockHandCollisionHalfExtentY,
            g_rockConfig.rockHandCollisionHalfExtentZ);

        if (!rightOk || !leftOk) {
            ROCK_LOG_ERROR(Hand, "Hand collision creation failed (rightOk={}, leftOk={})", rightOk, leftOk);
            if (rightOk)
                _rightHand.destroyCollision(bhkWorld);
            if (leftOk)
                _leftHand.destroyCollision(bhkWorld);
            return false;
        }

        return true;
    }

    void PhysicsInteraction::destroyHandCollisions(void* bhkWorld)
    {
        _rightHand.destroyDebugBasisVis();
        _leftHand.destroyDebugBasisVis();
        _rightHand.destroyCollision(bhkWorld);
        _leftHand.destroyCollision(bhkWorld);
    }

    void PhysicsInteraction::updateHandCollisions(RE::hknpWorld* world)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
            return;
        }

        bool rightDisabled = s_rightHandDisabled.load(std::memory_order_acquire);
        bool leftDisabled = s_leftHandDisabled.load(std::memory_order_acquire);

        const RE::NiTransform rightHandTransform = getInteractionHandTransform(false);
        const RE::NiTransform leftHandTransform = getInteractionHandTransform(true);
        RE::NiTransform rightTransform = computeHandCollisionTransformFromHandBasis(rightHandTransform, false);
        RE::NiTransform leftTransform = computeHandCollisionTransformFromHandBasis(leftHandTransform, true);
        const RE::NiPoint3 rightGrabAnchor = computeGrabPivotAPositionFromHandBasis(rightHandTransform, false);
        const RE::NiPoint3 leftGrabAnchor = computeGrabPivotAPositionFromHandBasis(leftHandTransform, true);

        if (!rightDisabled) {
            _rightHand.updateCollisionTransform(world, rightTransform, _deltaTime);
            auto* rightDebugParent = getInteractionHandNode(false);
            _rightHand.updateDebugBasisVis(rightTransform, rightGrabAnchor, g_rockConfig.rockDebugShowPalmBasis, rightDebugParent);
        } else {
            _rightHand.destroyDebugBasisVis();
        }
        if (!leftDisabled) {
            _leftHand.updateCollisionTransform(world, leftTransform, _deltaTime);
            auto* leftDebugParent = getInteractionHandNode(true);
            _leftHand.updateDebugBasisVis(leftTransform, leftGrabAnchor, g_rockConfig.rockDebugShowPalmBasis, leftDebugParent);
        } else {
            _leftHand.destroyDebugBasisVis();
        }
    }

    void PhysicsInteraction::publishDebugBodyOverlay(RE::hknpWorld* hknp)
    {
        const bool drawGrabPivots = g_rockConfig.rockDebugShowGrabPivots;
        const bool drawPalmVectors = g_rockConfig.rockDebugShowPalmVectors;
        if (!g_rockConfig.rockDebugShowColliders && !g_rockConfig.rockDebugShowTargetColliders && !g_rockConfig.rockDebugShowHandAxes && !drawGrabPivots && !drawPalmVectors) {
            debug::ClearFrame();
            return;
        }

        debug::Install();

        debug::BodyOverlayFrame frame{};
        frame.world = hknp;
        frame.drawRockBodies = g_rockConfig.rockDebugShowColliders;
        frame.drawTargetBodies = g_rockConfig.rockDebugShowTargetColliders;
        frame.drawAxes = g_rockConfig.rockDebugShowHandAxes;
        frame.drawMarkers = drawGrabPivots || drawPalmVectors;
        const bool rightDisabled = s_rightHandDisabled.load(std::memory_order_acquire);
        const bool leftDisabled = s_leftHandDisabled.load(std::memory_order_acquire);

        auto addBody = [&](RE::hknpBodyId bodyId, debug::BodyOverlayRole role) {
            if (bodyId.value == INVALID_BODY_ID || frame.count >= frame.entries.size()) {
                return;
            }

            for (std::uint32_t i = 0; i < frame.count; i++) {
                if (frame.entries[i].bodyId.value == bodyId.value && frame.entries[i].role == role) {
                    return;
                }
            }

            frame.entries[frame.count++] = debug::BodyOverlayEntry{ bodyId, role };
        };

        auto addAxisTransform = [&](const RE::NiTransform& transform, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
            if (!frame.drawAxes || frame.axisCount >= frame.axisEntries.size()) {
                return;
            }

            auto& entry = frame.axisEntries[frame.axisCount++];
            entry.source = debug::AxisOverlaySource::Transform;
            entry.role = role;
            entry.transform = transform;
            entry.translationStart = translationStart;
            entry.drawTranslationLine = drawTranslationLine;
        };

        auto addAxisBody = [&](RE::hknpBodyId bodyId, debug::AxisOverlayRole role, const RE::NiPoint3& translationStart, bool drawTranslationLine) {
            if (!frame.drawAxes || bodyId.value == INVALID_BODY_ID || frame.axisCount >= frame.axisEntries.size()) {
                return;
            }

            auto& entry = frame.axisEntries[frame.axisCount++];
            entry.source = debug::AxisOverlaySource::Body;
            entry.role = role;
            entry.bodyId = bodyId;
            entry.translationStart = translationStart;
            entry.drawTranslationLine = drawTranslationLine;
        };

        auto addMarker = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& position, const RE::NiPoint3& lineEnd, float size, bool drawPoint, bool drawLine) {
            if (!frame.drawMarkers || frame.markerCount >= frame.markerEntries.size()) {
                return;
            }

            auto& entry = frame.markerEntries[frame.markerCount++];
            entry.role = role;
            entry.position = position;
            entry.lineEnd = lineEnd;
            entry.size = size;
            entry.drawPoint = drawPoint;
            entry.drawLine = drawLine;
        };

        auto addMarkerPoint = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& position, float size) {
            addMarker(role, position, position, size, true, false);
        };

        auto addMarkerRay = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& start, const RE::NiPoint3& end, float startSize) {
            addMarker(role, start, end, startSize, true, true);
        };

        auto addMarkerLine = [&](debug::MarkerOverlayRole role, const RE::NiPoint3& start, const RE::NiPoint3& end) {
            addMarker(role, start, end, 0.0f, false, true);
        };

        if (frame.drawAxes) {
            if (!rightDisabled) {
                const RE::NiTransform rawHand = getInteractionHandTransform(false);
                const RE::NiTransform colliderHand = computeHandCollisionTransformFromHandBasis(rawHand, false);
                addAxisTransform(rawHand, debug::AxisOverlayRole::RightHandRaw, rawHand.translate, false);
                addAxisTransform(colliderHand, debug::AxisOverlayRole::RightHandCollider, rawHand.translate, true);
                addAxisBody(_rightHand.getCollisionBodyId(), debug::AxisOverlayRole::RightHandBody, rawHand.translate, true);
            }

            if (!leftDisabled) {
                const RE::NiTransform rawHand = getInteractionHandTransform(true);
                const RE::NiTransform colliderHand = computeHandCollisionTransformFromHandBasis(rawHand, true);
                addAxisTransform(rawHand, debug::AxisOverlayRole::LeftHandRaw, rawHand.translate, false);
                addAxisTransform(colliderHand, debug::AxisOverlayRole::LeftHandCollider, rawHand.translate, true);
                addAxisBody(_leftHand.getCollisionBodyId(), debug::AxisOverlayRole::LeftHandBody, rawHand.translate, true);
            }
        }

        if (drawPalmVectors) {
            auto addPalmVectorDebug = [&](bool isLeft) {
                if ((isLeft && leftDisabled) || (!isLeft && rightDisabled)) {
                    return;
                }

                const RE::NiTransform rawHand = getInteractionHandTransform(isLeft);
                const RE::NiPoint3 grabAnchor = computeGrabPivotAPositionFromHandBasis(rawHand, isLeft);
                const RE::NiPoint3 palmNormal = computePalmNormalFromHandBasis(rawHand, isLeft);
                const RE::NiPoint3 pointing = computePointingVectorFromHandBasis(rawHand, isLeft);
                const float palmNormalLength = (std::max)(5.0f, g_rockConfig.rockNearDetectionRange);
                const float pointingLength = (std::min)(90.0f, (std::max)(20.0f, g_rockConfig.rockFarDetectionRange));

                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabAnchor : debug::MarkerOverlayRole::RightGrabAnchor, grabAnchor, 2.0f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPalmNormal : debug::MarkerOverlayRole::RightPalmNormal, grabAnchor,
                    grabAnchor + palmNormal * palmNormalLength, 1.6f);
                addMarkerRay(isLeft ? debug::MarkerOverlayRole::LeftPointing : debug::MarkerOverlayRole::RightPointing, grabAnchor,
                    grabAnchor + pointing * pointingLength, 1.2f);
            };

            addPalmVectorDebug(false);
            addPalmVectorDebug(true);
        }

        if (drawGrabPivots) {
            auto addGrabPivotDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                GrabPivotDebugSnapshot snapshot{};
                if (!hand.getGrabPivotDebugSnapshot(hknp, snapshot)) {
                    return;
                }

                const bool isLeft = hand.isLeft();
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotA : debug::MarkerOverlayRole::RightGrabPivotA, snapshot.handPivotWorld, 3.0f);
                addMarkerPoint(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotB : debug::MarkerOverlayRole::RightGrabPivotB, snapshot.objectPivotWorld, 3.0f);
                addMarkerLine(isLeft ? debug::MarkerOverlayRole::LeftGrabPivotError : debug::MarkerOverlayRole::RightGrabPivotError, snapshot.handPivotWorld,
                    snapshot.objectPivotWorld);
            };

            addGrabPivotDebug(_rightHand);
            addGrabPivotDebug(_leftHand);
        }

        if (frame.drawRockBodies) {
            if (debug_overlay_policy::shouldDrawHandBody(frame.drawRockBodies, g_rockConfig.rockDebugDrawHandColliders)) {
                addBody(_rightHand.getCollisionBodyId(), debug::BodyOverlayRole::RightHand);
                addBody(_leftHand.getCollisionBodyId(), debug::BodyOverlayRole::LeftHand);
            }

            const auto weaponCount = _weaponCollision.getWeaponBodyCount();
            for (std::uint32_t i = 0; i < weaponCount; i++) {
                if (debug_overlay_policy::shouldDrawWeaponBody(frame.drawRockBodies, g_rockConfig.rockDebugDrawWeaponColliders, i, g_rockConfig.rockDebugMaxWeaponBodiesDrawn)) {
                    addBody(RE::hknpBodyId{ _weaponCollision.getWeaponBodyIdAtomic(i) }, debug::BodyOverlayRole::Weapon);
                }
            }
        }

        if (frame.drawTargetBodies) {
            auto addHandTarget = [&](const Hand& hand) {
                if (hand.isHolding()) {
                    addBody(hand.getSavedObjectState().bodyId, debug::BodyOverlayRole::Target);
                    addAxisBody(hand.getSavedObjectState().bodyId, debug::AxisOverlayRole::TargetBody, getInteractionHandTransform(hand.isLeft()).translate, true);
                    return;
                }
                if (hand.hasSelection()) {
                    addBody(hand.getSelection().bodyId, debug::BodyOverlayRole::Target);
                    addAxisBody(hand.getSelection().bodyId, debug::AxisOverlayRole::TargetBody, getInteractionHandTransform(hand.isLeft()).translate, true);
                }
            };

            addHandTarget(_rightHand);
            addHandTarget(_leftHand);
        }

        debug::PublishFrame(frame);
    }

    void PhysicsInteraction::updateSelection(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady())
            return;

        const RE::NiTransform rightTransform = getInteractionHandTransform(false);
        const RE::NiTransform leftTransform = getInteractionHandTransform(true);
        auto rightGrabAnchor = computeGrabPivotAPositionFromHandBasis(rightTransform, false);
        auto rightPalmNormal = computePalmNormalFromHandBasis(rightTransform, false);
        auto rightPointing = computePointingVectorFromHandBasis(rightTransform, false);
        auto leftGrabAnchor = computeGrabPivotAPositionFromHandBasis(leftTransform, true);
        auto leftPalmNormal = computePalmNormalFromHandBasis(leftTransform, true);
        auto leftPointing = computePointingVectorFromHandBasis(leftTransform, true);

        auto* rightHeldRef = _rightHand.hasSelection() ? _rightHand.getSelection().refr : nullptr;
        auto* leftHeldRef = _leftHand.hasSelection() ? _leftHand.getSelection().refr : nullptr;

        if (!s_rightHandDisabled.load(std::memory_order_acquire)) {
            _rightHand.updateSelection(bhk, hknp, rightGrabAnchor, rightPalmNormal, rightPointing, g_rockConfig.rockNearDetectionRange, g_rockConfig.rockFarDetectionRange,
                leftHeldRef);
        }

        if (!s_leftHandDisabled.load(std::memory_order_acquire)) {
            _leftHand.updateSelection(bhk, hknp, leftGrabAnchor, leftPalmNormal, leftPointing, g_rockConfig.rockNearDetectionRange, g_rockConfig.rockFarDetectionRange, rightHeldRef);
        }
    }

    void PhysicsInteraction::updateGrabInput(RE::hknpWorld* hknp)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady())
            return;

        int grabButton = g_rockConfig.rockGrabButtonID;

        auto processHand = [&](Hand& hand, bool isLeft) {
            if (isLeft && s_leftHandDisabled.load(std::memory_order_acquire))
                return;
            if (!isLeft && s_rightHandDisabled.load(std::memory_order_acquire))
                return;

            vrcf::Hand vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;

            if (hand.isHolding()) {
                if (vrcf::VRControllers.isReleased(vrHand, grabButton)) {
                    auto* heldRef = hand.getHeldRef();
                    auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                    hand.releaseGrabbedObject(hknp);
                    if (heldRef)
                        releaseObject(heldRef);
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                } else {
                    auto transform = getInteractionHandTransform(isLeft);
                    hand.updateHeldObject(hknp, transform, _deltaTime, g_rockConfig.rockGrabForceFadeInTime, g_rockConfig.rockGrabTauMin, g_rockConfig.rockGrabTauMax, 2.0f, 5.0f,
                        g_rockConfig.rockGrabCloseThreshold, g_rockConfig.rockGrabFarThreshold);

                    RE::NiTransform adjustedHand;
                    if (hand.getAdjustedHandTransform(adjustedHand)) {
                        auto* handNode = getInteractionHandNode(isLeft);

                        if (handNode) {
                            handNode->world.translate = adjustedHand.translate;
                            handNode->world.rotate = adjustedHand.rotate;
                            f4vr::updateTransformsDown(handNode, false);
                        }
                    }
                }
            } else if (hand.getState() == HandState::SelectedClose && hand.hasSelection()) {
                if (vrcf::VRControllers.isPressed(vrHand, grabButton)) {
                    if (hand.getSelection().isFarSelection && hand.getSelection().distance > 50.0f) {
                        ROCK_LOG_INFO(Hand, "{} hand: far grab blocked (dist={:.1f}) — too far", hand.handName(), hand.getSelection().distance);
                        return;
                    }

                    auto* selRef = hand.getSelection().refr;
                    if (selRef) {
                        auto* baseObj = selRef->GetObjectReference();
                        if (baseObj) {
                            const char* typeStr = baseObj->GetFormTypeString();

                            if (typeStr) {
                                std::string_view t(typeStr);
                                if (t == "DOOR" || t == "CONT" || t == "TERM" || t == "FURN") {
                                    return;
                                }

                                if (t == "ACTI") {
                                    auto& sel = hand.getSelection();
                                    if (sel.bodyId.value != 0x7FFF'FFFF) {
                                        auto* motion = hknp->GetBodyMotion(sel.bodyId);
                                        if (motion) {
                                            auto motionProps = *reinterpret_cast<std::uint16_t*>(reinterpret_cast<char*>(motion) + offsets::kMotion_PropertiesId);
                                            if (motionProps == 2 || motionProps == 0) {
                                                return;
                                            }
                                        }
                                    }
                                }
                            }

                            if (std::string_view(baseObj->GetFormTypeString()) == "NPC_") {
                                if (!selRef->IsDead(false))
                                    return;
                            }
                        }
                    }

                    auto transform = getInteractionHandTransform(isLeft);

                    bool grabbed = hand.grabSelectedObject(hknp, transform, g_rockConfig.rockGrabLinearTau, g_rockConfig.rockGrabLinearDamping,
                        g_rockConfig.rockGrabConstraintMaxForce, g_rockConfig.rockGrabLinearProportionalRecovery, g_rockConfig.rockGrabLinearConstantRecovery);

                    if (grabbed) {
                        auto* heldRef = hand.getHeldRef();
                        claimObject(heldRef);
                        dispatchPhysicsMessage(kPhysMsg_OnGrab, isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
                    }
                }
            }
        };

        processHand(_rightHand, false);
        processHand(_leftHand, true);
    }

    void PhysicsInteraction::resolveContacts(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        auto rightContactBody = _lastContactBodyRight.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (rightContactBody != 0xFFFFFFFF) {
            resolveAndLogContact("Right", bhk, hknp, RE::hknpBodyId{ rightContactBody });
        }

        auto leftContactBody = _lastContactBodyLeft.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (leftContactBody != 0xFFFFFFFF) {
            resolveAndLogContact("Left", bhk, hknp, RE::hknpBodyId{ leftContactBody });
        }
    }

    void PhysicsInteraction::resolveAndLogContact(const char* handName, RE::bhkWorld* bhk, RE::hknpWorld* hknp, RE::hknpBodyId bodyId)
    {
        if (!bhk || !hknp)
            return;

        auto& body = hknp->GetBody(bodyId);
        auto layer = body.collisionFilterInfo & 0x7F;

        auto* ref = resolveBodyToRef(bhk, hknp, bodyId);
        if (ref) {
            auto* baseObj = ref->GetObjectReference();
            const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";
            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const char* nameStr = objName.empty() ? "(unnamed)" : objName.data();

            ROCK_LOG_INFO(Hand, "{} hand TOUCHED [{}] '{}' formID={:08X} body={} layer={}", handName, typeName, nameStr, ref->GetFormID(), bodyId.value, layer);

            bool isLeft = (std::string_view(handName) == "Left");
            auto& hand = isLeft ? _leftHand : _rightHand;
            hand.setTouchState(ref, ref->GetFormID(), layer);
            dispatchPhysicsMessage(kPhysMsg_OnTouch, isLeft, ref, ref->GetFormID(), layer);
        } else {
            ROCK_LOG_DEBUG(Hand, "{} hand touched body={} layer={} (unresolved)", handName, bodyId.value, layer);
        }
    }

    void PhysicsInteraction::subscribeContactEvents(RE::hknpWorld* world)
    {
        void* signal = world->GetEventSignal(RE::hknpEventType::kContact);
        if (!signal) {
            ROCK_LOG_ERROR(Init, "Failed to get contact event signal");
            return;
        }

        struct CallbackInfo
        {
            void* fn;
            std::uint64_t ctx;
        };

        static CallbackInfo cbInfo;
        cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
        cbInfo.ctx = 0;

        typedef void subscribe_ext_t(void* signal, void* userData, void* callbackInfo);
        static REL::Relocation<subscribe_ext_t> subscribeExt{ REL::Offset(offsets::kFunc_SubscribeContactEvent) };
        subscribeExt(signal, static_cast<void*>(this), &cbInfo);

        ROCK_LOG_INFO(Init, "Subscribed to contact events");
    }

    void PhysicsInteraction::onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData)
    {
        __try {
            (void)userData;
            (void)worldPtrHolder;
            if (!s_hooksEnabled.load(std::memory_order_acquire))
                return;
            auto* self = s_instance.load(std::memory_order_acquire);
            if (self && self->_initialized.load(std::memory_order_acquire)) {
                self->handleContactEvent(contactEventData);
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehLogCounter = 0;
            if (sehLogCounter++ % 100 == 0) {
                logger::error(
                    "[ROCK::Contact] SEH exception caught on physics thread (count={}) — "
                    "likely stale world during cell transition",
                    sehLogCounter);
            }
            s_hooksEnabled.store(false, std::memory_order_release);
        }
    }

    void PhysicsInteraction::handleContactEvent(void* contactEventData)
    {
        if (!contactEventData)
            return;

        auto* data = reinterpret_cast<std::uint8_t*>(contactEventData);
        std::uint32_t bodyIdA = *reinterpret_cast<std::uint32_t*>(data + 0x08);
        std::uint32_t bodyIdB = *reinterpret_cast<std::uint32_t*>(data + 0x0C);

        const auto rightId = _rightHand.getCollisionBodyId().value;
        const auto leftId = _leftHand.getCollisionBodyId().value;

        if (_rightHand.isHoldingAtomic()) {
            if (_rightHand.isHeldBodyId(bodyIdA) || _rightHand.isHeldBodyId(bodyIdB)) {
                std::uint32_t heldId = _rightHand.isHeldBodyId(bodyIdA) ? bodyIdA : bodyIdB;
                std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
                if (other != rightId && other != leftId) {
                    _rightHand.notifyHeldBodyContact();
                }
            }
        }
        if (_leftHand.isHoldingAtomic()) {
            if (_leftHand.isHeldBodyId(bodyIdA) || _leftHand.isHeldBodyId(bodyIdB)) {
                std::uint32_t heldId = _leftHand.isHeldBodyId(bodyIdA) ? bodyIdA : bodyIdB;
                std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
                if (other != rightId && other != leftId) {
                    _leftHand.notifyHeldBodyContact();
                }
            }
        }

        bool isRight = (bodyIdA == rightId || bodyIdB == rightId);
        bool isLeft = (bodyIdA == leftId || bodyIdB == leftId);

        if (!isRight && !isLeft) {
            return;
        }

        {
            if (_weaponCollision.getWeaponBodyIdAtomic() != 0x7FFF'FFFF) {
                bool offhandIsLeft = !f4vr::isLeftHandedMode();
                std::uint32_t offhandId = offhandIsLeft ? leftId : rightId;
                bool offhandInvolved = (bodyIdA == offhandId || bodyIdB == offhandId);
                bool weaponInvolved = _weaponCollision.isWeaponBodyIdAtomic(bodyIdA) || _weaponCollision.isWeaponBodyIdAtomic(bodyIdB);
                if (offhandInvolved && weaponInvolved) {
                    _offhandTouchingWeapon.store(true, std::memory_order_release);
                }
            }
        }

        std::uint32_t otherBody = isRight ? (bodyIdA == rightId ? bodyIdB : bodyIdA) : (bodyIdA == leftId ? bodyIdB : bodyIdA);

        if (isRight) {
            _lastContactBodyRight.store(otherBody, std::memory_order_release);
        } else {
            _lastContactBodyLeft.store(otherBody, std::memory_order_release);
        }

        int logCount = _contactLogCounter.fetch_add(1, std::memory_order_relaxed);
        if (logCount % 30 == 0) {
            ROCK_LOG_DEBUG(Hand, "Contact: {} hand (body {}) hit body {}", isRight ? "Right" : "Left", isRight ? rightId : leftId, otherBody);
        }
    }

    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_ownedObjectsMutex);
        return _ownedObjects.contains(ref->GetFormID());
    }

    void PhysicsInteraction::claimObject(RE::TESObjectREFR* ref)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        _ownedObjects.insert(formID);
        ROCK_LOG_DEBUG(Hand, "Claimed object: formID={:08X}", formID);
    }

    void PhysicsInteraction::releaseObject(RE::TESObjectREFR* ref)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        if (_ownedObjects.erase(formID)) {
            ROCK_LOG_DEBUG(Hand, "Released object: formID={:08X}", formID);
        }
    }

    void PhysicsInteraction::releaseAllObjects()
    {
        std::scoped_lock lock(_ownedObjectsMutex);
        if (!_ownedObjects.empty()) {
            ROCK_LOG_DEBUG(Hand, "Releasing all {} owned objects", _ownedObjects.size());
            _ownedObjects.clear();
        }

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_Physics", false);
        }
    }

    void PhysicsInteraction::forceDropHeldObject(bool isLeft)
    {
        auto& hand = isLeft ? _leftHand : _rightHand;
        if (!hand.isHolding())
            return;

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no bhkWorld available");
            return;
        }
        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            ROCK_LOG_WARN(Hand, "forceDropHeldObject: no hknpWorld available");
            return;
        }

        auto* heldRef = hand.getHeldRef();
        ROCK_LOG_INFO(Hand, "forceDropHeldObject: {} hand dropping {}", isLeft ? "Left" : "Right", heldRef ? heldRef->GetFormID() : 0);

        hand.releaseGrabbedObject(hknp);
        if (heldRef)
            releaseObject(heldRef);
    }

    RE::bhkWorld* PhysicsInteraction::getPlayerBhkWorld() const
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player)
            return nullptr;

        auto* cell = player->GetParentCell();
        if (!cell)
            return nullptr;

        return cell->GetbhkWorld();
    }

    RE::hknpWorld* PhysicsInteraction::getHknpWorld(RE::bhkWorld* bhk)
    {
        if (!bhk)
            return nullptr;

        constexpr std::uintptr_t kBhkWorld_HknpWorldPtr = 0x60;
        return *reinterpret_cast<RE::hknpWorld**>(reinterpret_cast<uintptr_t>(bhk) + kBhkWorld_HknpWorldPtr);
    }
}
