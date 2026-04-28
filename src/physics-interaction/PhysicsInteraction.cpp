#include "PhysicsInteraction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

#include "BodyCollisionControl.h"
#include "CollisionLayerPolicy.h"
#include "HavokOffsets.h"
#include "DebugBodyOverlay.h"
#include "DebugOverlayPolicy.h"
#include "GrabInteractionPolicy.h"
#include "HeldObjectPhysicsMath.h"
#include "HandCollisionSuppressionMath.h"
#include "ObjectPhysicsBodySet.h"
#include "PalmTransform.h"
#include "PhysicsHooks.h"
#include "PhysicsRecursiveWrappers.h"
#include "PhysicsUtils.h"
#include "PushAssist.h"
#include "SelectionStatePolicy.h"
#include "WeaponTwoHandedGripMath.h"
#include "WeaponAuthorityLifecyclePolicy.h"
#include "WeaponMuzzleAuthorityMath.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

#include "ROCKMain.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "api/FRIKApi.h"
#include "f4vr/MiscStructs.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "f4sevr/Forms.h"
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

        const char* reloadStageName(WeaponVanillaReloadStage stage)
        {
            switch (stage) {
            case WeaponVanillaReloadStage::Idle:
                return "Idle";
            case WeaponVanillaReloadStage::ReloadRequested:
                return "ReloadRequested";
            case WeaponVanillaReloadStage::VanillaReloadStarted:
                return "VanillaReloadStarted";
            case WeaponVanillaReloadStage::AmmoDetachWindow:
                return "AmmoDetachWindow";
            case WeaponVanillaReloadStage::AmmoCommitted:
                return "AmmoCommitted";
            case WeaponVanillaReloadStage::ActionWindow:
                return "ActionWindow";
            case WeaponVanillaReloadStage::Completing:
                return "Completing";
            case WeaponVanillaReloadStage::Complete:
                return "Complete";
            case WeaponVanillaReloadStage::Canceled:
                return "Canceled";
            case WeaponVanillaReloadStage::UnsafeUnknown:
                return "UnsafeUnknown";
            }
            return "Unknown";
        }

        const char* reloadSourceName(WeaponReloadStageSource source)
        {
            switch (source) {
            case WeaponReloadStageSource::None:
                return "None";
            case WeaponReloadStageSource::NativeReloadEvent:
                return "NativeReloadEvent";
            case WeaponReloadStageSource::NativeAmmoCountEvent:
                return "NativeAmmoCountEvent";
            case WeaponReloadStageSource::PollingFallback:
                return "PollingFallback";
            case WeaponReloadStageSource::ConfigFallback:
                return "ConfigFallback";
            }
            return "Unknown";
        }

        const char* reloadRuntimeStateName(WeaponReloadState state)
        {
            switch (state) {
            case WeaponReloadState::Idle:
                return "Idle";
            case WeaponReloadState::ReloadRequested:
                return "ReloadRequested";
            case WeaponReloadState::WeaponOpened:
                return "WeaponOpened";
            case WeaponReloadState::WeaponUnloaded:
                return "WeaponUnloaded";
            case WeaponReloadState::PouchAvailable:
                return "PouchAvailable";
            case WeaponReloadState::AmmoHeld:
                return "AmmoHeld";
            case WeaponReloadState::AmmoAligned:
                return "AmmoAligned";
            case WeaponReloadState::AmmoInserted:
                return "AmmoInserted";
            case WeaponReloadState::ActionRequired:
                return "ActionRequired";
            case WeaponReloadState::ActionManipulating:
                return "ActionManipulating";
            case WeaponReloadState::Completing:
                return "Completing";
            case WeaponReloadState::Canceled:
                return "Canceled";
            }
            return "Unknown";
        }

        const char* weaponDiagnosticNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }

            const char* name = node->name.c_str();
            return name ? name : "";
        }

        const char* pushAssistSkipReasonName(push_assist::PushAssistSkipReason reason)
        {
            switch (reason) {
            case push_assist::PushAssistSkipReason::None:
                return "none";
            case push_assist::PushAssistSkipReason::Disabled:
                return "disabled";
            case push_assist::PushAssistSkipReason::Cooldown:
                return "cooldown";
            case push_assist::PushAssistSkipReason::BelowMinSpeed:
                return "below-min-speed";
            case push_assist::PushAssistSkipReason::InvalidImpulse:
                return "invalid-impulse";
            }
            return "unknown";
        }

        WeaponInteractionDebugInfo makeWeaponInteractionDebugInfo(
            const WeaponCollision& weaponCollision,
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& contact)
        {
            WeaponInteractionDebugInfo info{};
            info.weaponNodeName = weaponDiagnosticNodeName(weaponNode);

            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            if (weaponForm) {
                info.weaponFormId = weaponForm->formID;
                if (const char* fullName = weaponForm->GetFullName()) {
                    info.weaponName = fullName;
                }
            }

            if (contact.valid) {
                WeaponInteractionDebugInfo sourceInfo{};
                if (weaponCollision.tryGetWeaponContactDebugInfo(contact.bodyId, sourceInfo)) {
                    info.sourceName = sourceInfo.sourceName;
                    info.sourceRootName = sourceInfo.sourceRootName;
                }
            }

            return info;
        }

        f4vr::MuzzleFlash* getEquippedMuzzleFlashNodes()
        {
            /*
             * Ported from FRIK's WeaponPositionAdjuster muzzle fix because ROCK
             * is now the final weapon visual owner during mesh/hand authority.
             * FRIK still performs its own fix earlier in the frame, but any
             * later ROCK weapon write must re-own the fire node from the current
             * projectile node so the origin remains at the barrel tip.
             */
            const auto equipWeaponData = f4vr::getEquippedWeaponData();
            if (!equipWeaponData) {
                return nullptr;
            }

            const auto vfunc = reinterpret_cast<std::uint64_t*>(equipWeaponData);
            if ((*vfunc & 0xFFFF) != (f4vr::EquippedWeaponData_vfunc.get() & 0xFFFF)) {
                return nullptr;
            }

            const auto muzzle = reinterpret_cast<f4vr::MuzzleFlash*>(equipWeaponData->unk28);
            if (!muzzle || !muzzle->fireNode || !muzzle->projectileNode) {
                return nullptr;
            }

            return muzzle;
        }

        void applyFinalWeaponMuzzleAuthority()
        {
            auto* muzzle = getEquippedMuzzleFlashNodes();
            if (!muzzle) {
                return;
            }

            muzzle->fireNode->local = weapon_muzzle_authority_math::fireNodeLocalFromProjectileWorld(muzzle->projectileNode->world);
            f4vr::updateTransformsDown(muzzle->fireNode, true);
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
            ROCK_LOG_SAMPLE_DEBUG(Init, g_rockConfig.rockLogSampleMilliseconds, "No bhkWorld available for offset validation (will retry)");
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
                ROCK_LOG_DEBUG(Hand, "{} parity: raw(pos={:.3f}, rot={:.3f}deg) basis(collider={:.3f}, palmPos={:.3f}, palmNormal={:.3f}deg, pointing={:.3f}deg)", summaryHandLabel,
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
            ROCK_LOG_DEBUG(Hand, "Raw hand parity summary: R(pos={:.3f}, rot={:.3f}deg) L(pos={:.3f}, rot={:.3f}deg)", right.lastPositionDelta, right.lastRotationDeltaDegrees,
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

        if (!installNativeMeleeSuppressionHooks() && g_rockConfig.rockNativeMeleeSuppressionEnabled) {
            ROCK_LOG_CRITICAL(Init, "Native melee suppression requested but hook installation failed; ROCK will continue without melee suppression");
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
        _hasHeldPlayerSpacePosition = false;
        _heldObjectPlayerSpaceFrame = {};
        _heldPlayerSpaceLogCounter = 0;
        _deltaLogCounter = 0;
        _contactLogCounter = 0;
        _dynamicPushElapsedSeconds = 0.0f;
        _dynamicPushCooldownUntil.clear();
        _lastContactBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);

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
        _dynamicPushElapsedSeconds += _deltaTime;
        if (_dynamicPushCooldownUntil.size() > 512) {
            for (auto it = _dynamicPushCooldownUntil.begin(); it != _dynamicPushCooldownUntil.end();) {
                if (it->second <= _dynamicPushElapsedSeconds) {
                    it = _dynamicPushCooldownUntil.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (!frik::api::FRIKApi::inst->isSkeletonReady()) {
            if (_initialized) {
                ROCK_LOG_WARN(Update, "Skeleton no longer ready — shutting down");
                shutdown();
            }
            return;
        }

        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                frik::api::FRIKApi::inst->isAnyMenuOpen(),
                false,
                false)) {
            if (_initialized) {
                resetWeaponReloadStateForInterruption("menu");
                _primaryGripAuthority.reset();
                _twoHandedGrip.reset();
                auto* bhkMenu = getPlayerBhkWorld();
                if (bhkMenu) {
                    auto* hknpMenu = getHknpWorld(bhkMenu);
                    if (hknpMenu) {
                        restoreLeftHandCollisionAfterWeaponSupport(hknpMenu);
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
                } else {
                    hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
                    _leftWeaponSupportBroadPhaseSuppressed = false;
                }
            }
            debug::ClearFrame();
            return;
        }

        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                false,
                !g_rockConfig.rockEnabled,
                false)) {
            resetWeaponReloadStateForInterruption("disabled");
            _primaryGripAuthority.reset();
            if (_initialized) {
                shutdown();
            }
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

        if (_collisionLayerRegistered && (_expectedHandLayerMask != 0 || _expectedWeaponLayerMask != 0)) {
            auto modifierMgr = *reinterpret_cast<std::uintptr_t*>(reinterpret_cast<std::uintptr_t>(hknp) + offsets::kHknpWorld_ModifierManager);
            if (modifierMgr) {
                auto* filterPtr = *reinterpret_cast<void**>(modifierMgr + offsets::kModifierMgr_FilterPtr);
                if (filterPtr) {
                    auto* matrix = reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);
                    const auto currentHandMask = matrix[ROCK_HAND_LAYER];
                    const auto currentWeaponMask = matrix[ROCK_WEAPON_LAYER];
                    const bool handMaskDrifted =
                        _expectedHandLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentHandMask, _expectedHandLayerMask);
                    const bool weaponMaskDrifted =
                        _expectedWeaponLayerMask != 0 && !collision_layer_policy::configuredLayerMaskMatches(currentWeaponMask, _expectedWeaponLayerMask);
                    if (handMaskDrifted || weaponMaskDrifted) {
                        ROCK_LOG_WARN(Config,
                            "ROCK configured layer mask drift detected; hand expected=0x{:016X} current=0x{:016X}, weapon expected=0x{:016X} current=0x{:016X}; re-registering",
                            collision_layer_policy::configuredMask(_expectedHandLayerMask),
                            collision_layer_policy::configuredMask(currentHandMask),
                            collision_layer_policy::configuredMask(_expectedWeaponLayerMask),
                            collision_layer_policy::configuredMask(currentWeaponMask));
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
            RE::NiNode* weaponNode = f4vr::getWeaponNode();

            auto dominantHandBodyId = _rightHand.getCollisionBodyId();

            if (g_rockConfig.rockDebugVerboseLogging) {
                if (++_wpnNodeLogCounter >= 90) {
                    _wpnNodeLogCounter = 0;
                    if (weaponNode) {
                        ROCK_LOG_DEBUG(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}", weaponNode->name.c_str(), weaponNode->world.translate.x,
                            weaponNode->world.translate.y, weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
                    } else {
                    }
                }
            }
            _weaponCollision.update(hknp, weaponNode, dominantHandBodyId, _deltaTime);
        }

        {
            WeaponInteractionContact leftWeaponContact{};
            auto leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::None;
            RE::NiNode* weaponNode = f4vr::getWeaponNode();

            auto publishLeftWeaponProbeContact = [&](WeaponInteractionContact& contact) {
                _leftWeaponContactPartKind.store(static_cast<std::uint32_t>(contact.partKind), std::memory_order_release);
                _leftWeaponContactReloadRole.store(static_cast<std::uint32_t>(contact.reloadRole), std::memory_order_release);
                _leftWeaponContactSupportRole.store(static_cast<std::uint32_t>(contact.supportGripRole), std::memory_order_release);
                _leftWeaponContactSocketRole.store(static_cast<std::uint32_t>(contact.socketRole), std::memory_order_release);
                _leftWeaponContactActionRole.store(static_cast<std::uint32_t>(contact.actionRole), std::memory_order_release);
                _leftWeaponContactGripPose.store(static_cast<std::uint32_t>(contact.fallbackGripPose), std::memory_order_release);
                contact.sequence = _leftWeaponContactSequence.fetch_add(1, std::memory_order_acq_rel) + 1;
                _leftWeaponContactMissedFrames.store(0, std::memory_order_release);
            };

            const std::uint32_t leftWeaponBodyId = _leftWeaponContactBodyId.exchange(INVALID_CONTACT_BODY_ID, std::memory_order_acquire);
            if (leftWeaponBodyId != INVALID_CONTACT_BODY_ID) {
                _leftWeaponContactMissedFrames.store(0, std::memory_order_release);
                leftWeaponContact.valid = true;
                leftWeaponContact.bodyId = leftWeaponBodyId;
                leftWeaponContact.partKind = static_cast<WeaponPartKind>(_leftWeaponContactPartKind.load(std::memory_order_acquire));
                leftWeaponContact.reloadRole = static_cast<WeaponReloadRole>(_leftWeaponContactReloadRole.load(std::memory_order_acquire));
                leftWeaponContact.supportGripRole = static_cast<WeaponSupportGripRole>(_leftWeaponContactSupportRole.load(std::memory_order_acquire));
                leftWeaponContact.socketRole = static_cast<WeaponSocketRole>(_leftWeaponContactSocketRole.load(std::memory_order_acquire));
                leftWeaponContact.actionRole = static_cast<WeaponActionRole>(_leftWeaponContactActionRole.load(std::memory_order_acquire));
                leftWeaponContact.fallbackGripPose = static_cast<WeaponGripPoseId>(_leftWeaponContactGripPose.load(std::memory_order_acquire));
                leftWeaponContact.sequence = _leftWeaponContactSequence.load(std::memory_order_acquire);
                leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::Contact;
            } else if (weaponNode) {
                const RE::NiTransform leftHandTransform = getInteractionHandTransform(true);
                const RE::NiPoint3 leftProbePoint = computeGrabPivotAPositionFromHandBasis(leftHandTransform, true);
                if (_weaponCollision.tryFindInteractionContactNearPoint(weaponNode, leftProbePoint, g_rockConfig.rockWeaponInteractionProbeRadius, leftWeaponContact)) {
                    publishLeftWeaponProbeContact(leftWeaponContact);
                    leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::Probe;
                    if (g_rockConfig.rockDebugVerboseLogging && ++_weaponInteractionProbeLogCounter >= 90) {
                        _weaponInteractionProbeLogCounter = 0;
                        ROCK_LOG_DEBUG(Weapon, "WeaponInteractionProbe: bodyId={} partKind={} supportRole={} reloadRole={} actionRole={} radius={:.1f}",
                            leftWeaponContact.bodyId,
                            static_cast<int>(leftWeaponContact.partKind),
                            static_cast<int>(leftWeaponContact.supportGripRole),
                            static_cast<int>(leftWeaponContact.reloadRole),
                            static_cast<int>(leftWeaponContact.actionRole),
                            g_rockConfig.rockWeaponInteractionProbeRadius);
                    }
                } else {
                    const auto missedFrames = _leftWeaponContactMissedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                    if (missedFrames > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                        clearLeftWeaponContact();
                    }
                }
            } else {
                const auto missedFrames = _leftWeaponContactMissedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                if (missedFrames > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                    clearLeftWeaponContact();
                }
            }

            bool gripPressed = vrcf::VRControllers.isPressHeldDown(vrcf::Hand::Left, g_rockConfig.rockGrabButtonID);

            updateWeaponReloadState(weaponNode != nullptr);

            const WeaponInteractionDecision leftWeaponDecision = routeWeaponInteraction(leftWeaponContact, _weaponReloadCoordinator.runtime);
            const auto weaponNotificationKey = weapon_debug_notification_policy::makeWeaponNotificationKey(
                leftWeaponContact,
                leftWeaponDecision,
                _weaponReloadCoordinator.runtime,
                leftWeaponContactSource);

            const bool leftHandHoldingObject = _leftHand.isHolding();
            _twoHandedGrip.update(weaponNode, leftWeaponContact, gripPressed, leftHandHoldingObject, _deltaTime, _weaponReloadCoordinator.runtime);
            const bool weaponSupportGripActive = _twoHandedGrip.isGripping();
            if (g_rockConfig.rockDebugShowWeaponNotifications) {
                const auto gripNotificationEvent =
                    weapon_debug_notification_policy::observeWeaponSupportGrip(_weaponDebugNotificationState, weaponSupportGripActive);
                if (gripNotificationEvent != weapon_debug_notification_policy::WeaponGripNotificationEvent::None) {
                    if (gripNotificationEvent == weapon_debug_notification_policy::WeaponGripNotificationEvent::Started) {
                        const auto weaponDebugInfo = makeWeaponInteractionDebugInfo(_weaponCollision, weaponNode, leftWeaponContact);
                        f4vr::showNotification(
                            weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey, weaponDebugInfo));
                        ROCK_LOG_INFO(Weapon,
                            "WeaponGripDiagnostics: weapon='{}' formID={:08X} node='{}' root='{}' nif='{}' part={} route={} pose={} reload={} body={} source={}",
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponName),
                            weaponDebugInfo.weaponFormId,
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponNodeName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceRootName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceName),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.partKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.interactionKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.gripPose),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.reloadState),
                            weaponNotificationKey.bodyId,
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.source));
                    } else {
                        f4vr::showNotification(weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey));
                    }
                }
            } else {
                _weaponDebugNotificationState.supportGripActive = weaponSupportGripActive;
            }

            if (weaponSupportGripActive) {
                _primaryGripAuthority.reset();
                suppressLeftHandCollisionForWeaponSupport(hknp);
            } else {
                restoreLeftHandCollisionAfterWeaponSupport(hknp);
                _primaryGripAuthority.update(weaponNode, false, _weaponReloadCoordinator.runtime, g_rockConfig.rockOneHandedMeshPrimaryGripAuthorityEnabled);
            }

            _weaponCollision.updateBodiesFromCurrentSourceTransforms(hknp, weaponNode, _deltaTime);
            if (f4vr::isNodeVisible(weaponNode)) {
                applyFinalWeaponMuzzleAuthority();
            }
        }

        updateSelection(bhk, hknp);

        _heldObjectPlayerSpaceFrame = sampleHeldObjectPlayerSpaceFrame();

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

    void PhysicsInteraction::updateWeaponReloadState(bool weaponEquipped)
    {
        if (!g_rockConfig.rockReloadUseVanillaStageObserver) {
            if (_weaponReloadEventBridge.isInstalled()) {
                _weaponReloadEventBridge.uninstall();
            }
            _weaponReloadEventBridge.reset();
            resetWeaponReloadStateForInterruption("reload observer disabled");
            return;
        }

        _weaponReloadEventBridge.install();

        const auto snapshot = _weaponReloadEventBridge.consumeSnapshot(weaponEquipped);
        const auto observed = advanceWeaponReloadStageObserver(_weaponReloadObserver, snapshot);
        _lastWeaponReloadObservation = observed;

        constexpr bool kPhysicalReloadCompletionSupported = false;
        const bool requirePhysicalCompletion = shouldRequireWeaponReloadPhysicalCompletion(
            g_rockConfig.rockReloadRequirePhysicalCompletion, g_rockConfig.rockReloadAllowStageFallbacks, kPhysicalReloadCompletionSupported);
        const bool physicalAmmoInserted = !requirePhysicalCompletion && observed.stage == WeaponVanillaReloadStage::AmmoCommitted;
        auto coordinated = advanceWeaponReloadCoordinator(_weaponReloadCoordinator, observed, physicalAmmoInserted, requirePhysicalCompletion);

        const bool nativeEventReceived = snapshot.reloadEventReceived || snapshot.ammoEventReceived;
        if (!coordinated.runtime.isReloadActive() || nativeEventReceived || coordinated.stateChanged) {
            _reloadObserverStaleFrames = 0;
        } else {
            ++_reloadObserverStaleFrames;
        }

        const std::uint32_t staleTimeoutFrames = static_cast<std::uint32_t>((std::max)(0, g_rockConfig.rockReloadObserverStaleFrameTimeout));
        if (shouldFallbackCompleteStaleReload(
                coordinated.runtime.isReloadActive(),
                requirePhysicalCompletion,
                g_rockConfig.rockReloadAllowStageFallbacks,
                _reloadObserverStaleFrames,
                staleTimeoutFrames)) {
            ROCK_LOG_WARN(Weapon,
                "ReloadObserver stale fallback: stage={} runtime={} frames={} timeout={} — restoring support grip until physical reload completion is implemented",
                reloadStageName(coordinated.vanillaStage),
                reloadRuntimeStateName(coordinated.runtime.state),
                _reloadObserverStaleFrames,
                staleTimeoutFrames);

            _weaponReloadObserver = {};
            _weaponReloadCoordinator = {};
            _reloadObserverStaleFrames = 0;
            coordinated.runtime = _weaponReloadCoordinator.runtime;
            coordinated.vanillaStage = WeaponVanillaReloadStage::Idle;
            coordinated.source = WeaponReloadStageSource::PollingFallback;
            coordinated.stateChanged = true;
            coordinated.vanillaCompletionNeedsPhysicalGate = false;
        }

        _observedWeaponReloadStage.store(static_cast<std::uint32_t>(coordinated.vanillaStage), std::memory_order_release);
        _weaponReloadStageSource.store(static_cast<std::uint32_t>(coordinated.source), std::memory_order_release);
        _activeWeaponReloadState.store(static_cast<std::uint32_t>(coordinated.runtime.state), std::memory_order_release);

        const bool shouldLogTransition = observed.stageChanged || coordinated.stateChanged;
        const bool shouldLogDebug = g_rockConfig.rockReloadDebugStageLogging && (++_reloadStateLogCounter >= 60);
        if (shouldLogTransition || shouldLogDebug) {
            _reloadStateLogCounter = 0;
            ROCK_LOG_DEBUG(Weapon,
                "ReloadObserver: equipped={} vanillaStage={} source={} runtime={} supportAllowed={} clip={} reserve={} seq={} physicalGateNeeded={} physicalGateSupported={} fallbackAllowed={}",
                weaponEquipped,
                reloadStageName(coordinated.vanillaStage),
                reloadSourceName(coordinated.source),
                reloadRuntimeStateName(coordinated.runtime.state),
                coordinated.runtime.supportGripAllowed,
                observed.clipAmmo,
                observed.reserveAmmo,
                observed.sequence,
                coordinated.vanillaCompletionNeedsPhysicalGate,
                kPhysicalReloadCompletionSupported,
                g_rockConfig.rockReloadAllowStageFallbacks);
        }
    }

    void PhysicsInteraction::resetWeaponReloadStateForInterruption(const char* reason)
    {
        const bool wasActive = _weaponReloadCoordinator.runtime.isReloadActive();
        _weaponReloadObserver = {};
        _weaponReloadCoordinator = {};
        _lastWeaponReloadObservation = {};
        _activeWeaponReloadState.store(static_cast<std::uint32_t>(WeaponReloadState::Idle), std::memory_order_release);
        _observedWeaponReloadStage.store(static_cast<std::uint32_t>(WeaponVanillaReloadStage::Idle), std::memory_order_release);
        _weaponReloadStageSource.store(static_cast<std::uint32_t>(WeaponReloadStageSource::None), std::memory_order_release);
        _reloadStateLogCounter = 0;
        _reloadObserverStaleFrames = 0;
        _weaponInteractionProbeLogCounter = 0;

        if (wasActive || g_rockConfig.rockReloadDebugStageLogging) {
            ROCK_LOG_INFO(Weapon, "ReloadObserver reset ({})", reason ? reason : "unknown");
        }
    }

    void PhysicsInteraction::clearLeftWeaponContact()
    {
        _leftWeaponContactBodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        _leftWeaponContactPartKind.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        _leftWeaponContactReloadRole.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        _leftWeaponContactSupportRole.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        _leftWeaponContactSocketRole.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        _leftWeaponContactActionRole.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        _leftWeaponContactGripPose.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        _leftWeaponContactMissedFrames.store(WEAPON_CONTACT_TIMEOUT_FRAMES + 1, std::memory_order_release);
    }

    void PhysicsInteraction::suppressLeftHandCollisionForWeaponSupport(RE::hknpWorld* world)
    {
        /*
         * Layer 43 vs 44 is intentionally allowed so the offhand can physically
         * touch the equipped weapon before support grip starts. Once support
         * grip owns the transform, the left hand body becomes a driver and must
         * stop solving against the weapon package, matching the HIGGS pattern
         * used for held-object hand collision suppression.
         */
        if (!world || !_leftHand.hasCollisionBody()) {
            return;
        }

        const auto handBodyId = _leftHand.getCollisionBodyId();
        if (handBodyId.value == INVALID_CONTACT_BODY_ID) {
            return;
        }

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, handBodyId, currentFilter)) {
            return;
        }

        const bool firstSuppression = !_leftWeaponSupportCollisionSuppression.active || _leftWeaponSupportCollisionSuppression.bodyId != handBodyId.value;
        const auto disabledFilter = hand_collision_suppression_math::beginSuppression(_leftWeaponSupportCollisionSuppression, handBodyId.value, currentFilter);
        if (disabledFilter != currentFilter) {
            body_collision::setFilterInfo(world, handBodyId, disabledFilter);
        }

        const bool broadPhaseSet = body_collision::setBroadPhaseEnabled(world, handBodyId, false);
        _leftWeaponSupportBroadPhaseSuppressed = _leftWeaponSupportBroadPhaseSuppressed || broadPhaseSet;

        if (firstSuppression || disabledFilter != currentFilter) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: left hand collision suppressed bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} broadphase={}",
                handBodyId.value,
                currentFilter,
                disabledFilter,
                _leftWeaponSupportCollisionSuppression.wasNoCollideBeforeSuppression ? "yes" : "no",
                broadPhaseSet ? "disabled" : "unchanged");
        }
    }

    void PhysicsInteraction::restoreLeftHandCollisionAfterWeaponSupport(RE::hknpWorld* world)
    {
        if (!_leftWeaponSupportCollisionSuppression.active) {
            return;
        }

        if (!world || !_leftHand.hasCollisionBody()) {
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            _leftWeaponSupportBroadPhaseSuppressed = false;
            return;
        }

        const auto handBodyId = _leftHand.getCollisionBodyId();
        if (handBodyId.value == INVALID_CONTACT_BODY_ID || handBodyId.value != _leftWeaponSupportCollisionSuppression.bodyId) {
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            _leftWeaponSupportBroadPhaseSuppressed = false;
            return;
        }

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, handBodyId, currentFilter)) {
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            _leftWeaponSupportBroadPhaseSuppressed = false;
            return;
        }

        const auto restoredFilter = hand_collision_suppression_math::restoreFilter(_leftWeaponSupportCollisionSuppression, currentFilter);
        if (restoredFilter != currentFilter) {
            body_collision::setFilterInfo(world, handBodyId, restoredFilter);
        }

        const bool broadPhaseRestored = _leftWeaponSupportBroadPhaseSuppressed && body_collision::setBroadPhaseEnabled(world, handBodyId, true);
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: left hand collision restored bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} broadphase={}",
            handBodyId.value,
            currentFilter,
            restoredFilter,
            _leftWeaponSupportCollisionSuppression.wasNoCollideBeforeSuppression ? "yes" : "no",
            broadPhaseRestored ? "enabled" : "unchanged");

        hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
        _leftWeaponSupportBroadPhaseSuppressed = false;
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
            restoreLeftHandCollisionAfterWeaponSupport(hknp);
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
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            _leftWeaponSupportBroadPhaseSuppressed = false;
        }

        debug::ClearFrame();
        _primaryGripAuthority.reset();
        _twoHandedGrip.reset();
        _weaponCollision.shutdown();
        _weaponReloadEventBridge.uninstall();
        _weaponReloadEventBridge.reset();
        resetWeaponReloadStateForInterruption("shutdown");
        clearLeftWeaponContact();
        releaseAllObjects();
        _rightHand.reset();
        _leftHand.reset();

        _cachedBhkWorld = nullptr;
        _collisionLayerRegistered = false;
        _initialized = false;
        _hasPrevPositions = false;
        _hasHeldPlayerSpacePosition = false;
        _heldObjectPlayerSpaceFrame = {};
        _heldPlayerSpaceLogCounter = 0;
        _handBoneCache.reset();
        _handCacheResolveLogCounter = 0;
        _paritySummaryCounter = 0;
        _parityEnabledLogged = false;
        _runtimeScaleLogged = false;
        _rawHandParityStates = {};
        _dynamicPushCooldownUntil.clear();
        _lastContactBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);

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
            ROCK_LOG_DEBUG(Config, "Filter source: world={:p}, global={:p}, same={}", filterPtr, (void*)globalFilter, filterPtr == globalFilter);
        }

        auto* matrix = reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);

        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_HAND_LAYER, matrix[ROCK_HAND_LAYER]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_WEAPON_LAYER, matrix[ROCK_WEAPON_LAYER]);

        collision_layer_policy::applyRockHandLayerPolicy(matrix, g_rockConfig.rockCollideWithCharControllers, true);
        collision_layer_policy::applyRockWeaponLayerPolicy(
            matrix, g_rockConfig.rockWeaponCollisionBlocksProjectiles, g_rockConfig.rockWeaponCollisionBlocksSpells);

        _expectedHandLayerMask = collision_layer_policy::buildRockHandExpectedMask(g_rockConfig.rockCollideWithCharControllers, true);
        _expectedWeaponLayerMask =
            collision_layer_policy::buildRockWeaponExpectedMask(g_rockConfig.rockWeaponCollisionBlocksProjectiles, g_rockConfig.rockWeaponCollisionBlocksSpells);
        _collisionLayerRegistered = true;

        ROCK_LOG_INFO(Config, "Registered ROCK collision layers: hand={} mask=0x{:016X}, weapon={} mask=0x{:016X}, projectiles={}, spells={}",
            ROCK_HAND_LAYER,
            matrix[ROCK_HAND_LAYER],
            ROCK_WEAPON_LAYER,
            matrix[ROCK_WEAPON_LAYER],
            g_rockConfig.rockWeaponCollisionBlocksProjectiles ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksSpells ? "enabled" : "disabled");
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
        const bool drawFingerProbes = g_rockConfig.rockDebugShowGrabFingerProbes;
        const bool drawPalmVectors = g_rockConfig.rockDebugShowPalmVectors;
        const bool drawWeaponAuthorityDebug = _twoHandedGrip.isGripping() && (g_rockConfig.rockDebugShowHandAxes || drawGrabPivots);
        if (!g_rockConfig.rockDebugShowColliders && !g_rockConfig.rockDebugShowTargetColliders && !g_rockConfig.rockDebugShowHandAxes && !drawGrabPivots && !drawFingerProbes &&
            !drawPalmVectors && !drawWeaponAuthorityDebug) {
            debug::ClearFrame();
            return;
        }

        debug::Install();

        debug::BodyOverlayFrame frame{};
        frame.world = hknp;
        frame.drawRockBodies = g_rockConfig.rockDebugShowColliders;
        frame.drawTargetBodies = g_rockConfig.rockDebugShowTargetColliders;
        frame.drawAxes = g_rockConfig.rockDebugShowHandAxes;
        frame.drawMarkers = drawGrabPivots || drawFingerProbes || drawPalmVectors || drawWeaponAuthorityDebug;
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

        auto pointDistance = [](const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) {
            const RE::NiPoint3 delta = lhs - rhs;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
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

        if (drawFingerProbes) {
            auto addFingerProbeDebug = [&](const Hand& hand) {
                if ((hand.isLeft() && leftDisabled) || (!hand.isLeft() && rightDisabled)) {
                    return;
                }

                std::array<RE::NiPoint3, 5> starts{};
                std::array<RE::NiPoint3, 5> ends{};
                if (!hand.getGrabFingerProbeDebug(starts, ends)) {
                    return;
                }

                const auto role = hand.isLeft() ? debug::MarkerOverlayRole::LeftGrabFingerProbe : debug::MarkerOverlayRole::RightGrabFingerProbe;
                for (std::size_t i = 0; i < starts.size(); ++i) {
                    addMarkerLine(role, starts[i], ends[i]);
                }
            };

            addFingerProbeDebug(_rightHand);
            addFingerProbeDebug(_leftHand);
        }

        if (drawWeaponAuthorityDebug) {
            TwoHandedGripDebugSnapshot snapshot{};
            if (_twoHandedGrip.getDebugAuthoritySnapshot(snapshot)) {
                addAxisTransform(snapshot.weaponWorld, debug::AxisOverlayRole::WeaponAuthority, snapshot.weaponWorld.translate, false);
                addAxisTransform(snapshot.rightRequestedHandWorld, debug::AxisOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, true);
                addAxisTransform(snapshot.leftRequestedHandWorld, debug::AxisOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, true);
                addMarkerPoint(debug::MarkerOverlayRole::RightWeaponPrimaryGrip, snapshot.rightGripWorld, 3.0f);
                addMarkerPoint(debug::MarkerOverlayRole::LeftWeaponSupportGrip, snapshot.leftGripWorld, 3.0f);

                if (frik::api::FRIKApi::inst) {
                    const RE::NiTransform appliedRight = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right);
                    const RE::NiTransform appliedLeft = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left);
                    addAxisTransform(appliedRight, debug::AxisOverlayRole::RightFrikAppliedHand, snapshot.rightRequestedHandWorld.translate, true);
                    addAxisTransform(appliedLeft, debug::AxisOverlayRole::LeftFrikAppliedHand, snapshot.leftRequestedHandWorld.translate, true);
                    addMarkerLine(debug::MarkerOverlayRole::RightWeaponAuthorityMismatch, snapshot.rightRequestedHandWorld.translate, appliedRight.translate);
                    addMarkerLine(debug::MarkerOverlayRole::LeftWeaponAuthorityMismatch, snapshot.leftRequestedHandWorld.translate, appliedLeft.translate);

                    static std::uint32_t authorityMismatchLogCounter = 0;
                    if (++authorityMismatchLogCounter >= 120) {
                        authorityMismatchLogCounter = 0;
                        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip authority mismatch: right={:.2f}gu left={:.2f}gu",
                            pointDistance(snapshot.rightRequestedHandWorld.translate, appliedRight.translate),
                            pointDistance(snapshot.leftRequestedHandWorld.translate, appliedLeft.translate));
                    }
                }
            }
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

    HeldObjectPlayerSpaceFrame PhysicsInteraction::sampleHeldObjectPlayerSpaceFrame()
    {
        HeldObjectPlayerSpaceFrame frame{};
        auto* api = frik::api::FRIKApi::inst;
        if (!api) {
            _hasHeldPlayerSpacePosition = false;
            return frame;
        }

        const RE::NiPoint3 smoothPos = api->getSmoothedPlayerPosition();
        if (!g_rockConfig.rockGrabPlayerSpaceCompensation) {
            _prevHeldPlayerSpacePosition = smoothPos;
            _hasHeldPlayerSpacePosition = true;
            return frame;
        }

        frame.enabled = true;
        if (_hasHeldPlayerSpacePosition) {
            frame.deltaGameUnits = smoothPos - _prevHeldPlayerSpacePosition;
            frame.velocityHavok = held_object_physics_math::gameUnitsDeltaToHavokVelocity(frame.deltaGameUnits, _deltaTime);
            frame.warp = held_object_physics_math::shouldWarpPlayerSpaceDelta(frame.deltaGameUnits, g_rockConfig.rockGrabPlayerSpaceWarpDistance);
        }

        _prevHeldPlayerSpacePosition = smoothPos;
        _hasHeldPlayerSpacePosition = true;

        if (g_rockConfig.rockDebugGrabFrameLogging && (_rightHand.isHolding() || _leftHand.isHolding())) {
            ++_heldPlayerSpaceLogCounter;
            if (_heldPlayerSpaceLogCounter >= 45 || frame.warp) {
                _heldPlayerSpaceLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Held player-space: enabled={} warp={} delta=({:.2f},{:.2f},{:.2f}) velHk=({:.3f},{:.3f},{:.3f})",
                    frame.enabled ? "yes" : "no", frame.warp ? "yes" : "no", frame.deltaGameUnits.x, frame.deltaGameUnits.y, frame.deltaGameUnits.z,
                    frame.velocityHavok.x, frame.velocityHavok.y, frame.velocityHavok.z);
            }
        }

        return frame;
    }

    void PhysicsInteraction::updateGrabInput(RE::hknpWorld* hknp)
    {
        if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady())
            return;

        int grabButton = g_rockConfig.rockGrabButtonID;
        const bool rightHandWeaponEquipped = frik::api::FRIKApi::inst->isWeaponDrawn() && f4vr::getWeaponNode();
        const bool equippedWeaponSupportGripActive = _twoHandedGrip.isGripping();

        auto releaseSuppressedHeldObject = [&](Hand& hand, bool isLeft, const char* reason) {
            auto* heldRef = hand.getHeldRef();
            auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
            hand.releaseGrabbedObject(hknp);
            if (heldRef) {
                releaseObject(heldRef);
            }
            ROCK_LOG_DEBUG(Hand, "{} hand: released held object because normal grab input is suppressed ({})", hand.handName(), reason ? reason : "unknown");
            dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
        };

        auto processHand = [&](Hand& hand, bool isLeft) {
            if (isLeft && s_leftHandDisabled.load(std::memory_order_acquire))
                return;
            if (!isLeft && s_rightHandDisabled.load(std::memory_order_acquire))
                return;
            if (!weapon_two_handed_grip_math::canProcessNormalGrabInput(isLeft, equippedWeaponSupportGripActive, rightHandWeaponEquipped)) {
                if (hand.isHolding()) {
                    releaseSuppressedHeldObject(hand, isLeft, isLeft ? "equipped weapon support grip active" : "right-hand weapon equipped");
                } else if (hand.getState() == HandState::Pulled || hand.getState() == HandState::SelectionLocked) {
                    auto* selectedRef = hand.getSelection().refr;
                    hand.clearSelectionState(true);
                    releaseObject(selectedRef);
                    ROCK_LOG_DEBUG(Hand, "{} hand: cleared pull/locked selection because normal grab input is suppressed", hand.handName());
                }
                return;
            }

            vrcf::Hand vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;
            auto selectedObjectInteractionBlocked = [&]() {
                const auto& sel = hand.getSelection();
                auto* selRef = sel.refr;
                if (!selRef) {
                    return false;
                }

                auto* baseObj = selRef->GetObjectReference();
                if (!baseObj) {
                    return false;
                }

                const char* typeStr = baseObj->GetFormTypeString();
                const std::string_view formType = typeStr ? std::string_view(typeStr) : std::string_view{};

                bool hasMotionProps = false;
                std::uint16_t motionProps = 0;
                if (formType == "ACTI" && sel.bodyId.value != 0x7FFF'FFFF && hknp) {
                    if (auto* motion = hknp->GetBodyMotion(sel.bodyId)) {
                        motionProps = *reinterpret_cast<std::uint16_t*>(reinterpret_cast<char*>(motion) + offsets::kMotion_PropertiesId);
                        hasMotionProps = true;
                    }
                }

                const bool isLiveNpc = formType == "NPC_" && !selRef->IsDead(false);
                const bool blocked = grab_interaction_policy::shouldBlockSelectedObjectInteraction(formType, isLiveNpc, hasMotionProps, motionProps);
                if (blocked) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand: selected object interaction blocked formType={} formID={:08X} far={} motionProps={} hasMotionProps={}",
                        hand.handName(),
                        formType.empty() ? "???" : typeStr,
                        selRef->GetFormID(),
                        sel.isFarSelection ? "yes" : "no",
                        motionProps,
                        hasMotionProps ? "yes" : "no");
                }
                return blocked;
            };

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
                    hand.updateHeldObject(hknp, transform, _heldObjectPlayerSpaceFrame, _deltaTime, g_rockConfig.rockGrabForceFadeInTime, g_rockConfig.rockGrabTauMin);

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
            } else if (selection_state_policy::canProcessSelectedState(hand.getState()) && hand.hasSelection()) {
                if (vrcf::VRControllers.isPressed(vrHand, grabButton)) {
                    if (!grab_interaction_policy::canAttemptSelectedObjectGrab(
                            hand.getSelection().isFarSelection, hand.getSelection().distance, g_rockConfig.rockFarDetectionRange)) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand: far grab blocked (dist={:.1f}, configuredFarRange={:.1f})",
                            hand.handName(),
                            hand.getSelection().distance,
                            g_rockConfig.rockFarDetectionRange);
                        return;
                    }

                    if (selectedObjectInteractionBlocked()) {
                        return;
                    }

                    if (hand.getSelection().isFarSelection) {
                        auto* selectedRef = hand.getSelection().refr;
                        auto transform = getInteractionHandTransform(isLeft);
                        if (hand.lockFarSelection() && hand.startDynamicPull(hknp, transform)) {
                            claimObject(selectedRef);
                        } else {
                            releaseObject(selectedRef);
                        }
                        return;
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
            } else if (hand.getState() == HandState::SelectionLocked) {
                if (vrcf::VRControllers.isReleased(vrHand, grabButton)) {
                    auto* selectedRef = hand.getSelection().refr;
                    ROCK_LOG_DEBUG(Hand, "{} hand released locked far selection", hand.handName());
                    hand.clearSelectionState(true);
                    releaseObject(selectedRef);
                }
            } else if (hand.getState() == HandState::Pulled) {
                auto* pulledRef = hand.getSelection().refr;
                if (vrcf::VRControllers.isReleased(vrHand, grabButton)) {
                    ROCK_LOG_DEBUG(Hand, "{} hand released dynamic pull", hand.handName());
                    hand.clearSelectionState(true);
                    releaseObject(pulledRef);
                    return;
                }

                auto transform = getInteractionHandTransform(isLeft);
                const bool readyToGrab = hand.updateDynamicPull(hknp, transform, _deltaTime);
                if (!hand.hasSelection() || hand.getState() == HandState::Idle) {
                    releaseObject(pulledRef);
                    return;
                }

                if (readyToGrab) {
                    bool grabbed = hand.grabSelectedObject(hknp, transform, g_rockConfig.rockGrabLinearTau, g_rockConfig.rockGrabLinearDamping,
                        g_rockConfig.rockGrabConstraintMaxForce, g_rockConfig.rockGrabLinearProportionalRecovery, g_rockConfig.rockGrabLinearConstantRecovery);

                    if (grabbed) {
                        auto* heldRef = hand.getHeldRef();
                        claimObject(heldRef);
                        dispatchPhysicsMessage(kPhysMsg_OnGrab, isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
                    } else {
                        hand.clearSelectionState(true);
                        releaseObject(pulledRef);
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
            applyDynamicPushAssist("Right", bhk, hknp, _rightHand.getCollisionBodyId().value, rightContactBody, false);
        }

        auto leftContactBody = _lastContactBodyLeft.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (leftContactBody != 0xFFFFFFFF) {
            resolveAndLogContact("Left", bhk, hknp, RE::hknpBodyId{ leftContactBody });
            applyDynamicPushAssist("Left", bhk, hknp, _leftHand.getCollisionBodyId().value, leftContactBody, false);
        }

        auto weaponContactBody = _lastContactBodyWeapon.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        auto weaponSourceBody = _lastContactSourceWeapon.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (weaponContactBody != 0xFFFFFFFF && weaponSourceBody != 0xFFFFFFFF) {
            applyDynamicPushAssist("Weapon", bhk, hknp, weaponSourceBody, weaponContactBody, true);
        }
    }

    void PhysicsInteraction::applyDynamicPushAssist(const char* sourceName,
        RE::bhkWorld* bhk,
        RE::hknpWorld* hknp,
        std::uint32_t sourceBodyId,
        std::uint32_t targetBodyId,
        bool sourceIsWeapon)
    {
        if (!bhk || !hknp || sourceBodyId == 0xFFFFFFFF || targetBodyId == 0xFFFFFFFF ||
            sourceBodyId == object_physics_body_set::INVALID_BODY_ID || targetBodyId == object_physics_body_set::INVALID_BODY_ID || sourceBodyId == targetBodyId) {
            return;
        }

        auto* targetRef = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ targetBodyId });
        if (!targetRef || targetRef->IsDeleted() || targetRef->IsDisabled()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds, "{} dynamic push skipped: target body {} has no valid ref", sourceName, targetBodyId);
            return;
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::PassivePush;
        scanOptions.rightHandBodyId = _rightHand.getCollisionBodyId().value;
        scanOptions.leftHandBodyId = _leftHand.getCollisionBodyId().value;
        scanOptions.sourceBodyId = sourceBodyId;
        scanOptions.sourceWeaponBodyId = sourceIsWeapon ? sourceBodyId : object_physics_body_set::INVALID_BODY_ID;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;
        if (!sourceIsWeapon) {
            if (sourceBodyId == _rightHand.getCollisionBodyId().value) {
                scanOptions.heldBySameHand = &_rightHand.getHeldBodyIds();
            } else if (sourceBodyId == _leftHand.getCollisionBodyId().value) {
                scanOptions.heldBySameHand = &_leftHand.getHeldBodyIds();
            }
        }

        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhk, hknp, targetRef, scanOptions);
        const auto* targetRecord = bodySet.findRecord(targetBodyId);
        if (!targetRecord) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} not found in ref tree formID={:08X} visitedNodes={} collisionObjects={}",
                sourceName,
                targetBodyId,
                targetRef->GetFormID(),
                bodySet.diagnostics.visitedNodes,
                bodySet.diagnostics.collisionObjects);
            return;
        }
        if (!targetRecord->accepted) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} rejected reason={} layer={} motionId={} motionType={}",
                sourceName,
                targetBodyId,
                physics_body_classifier::rejectReasonName(targetRecord->rejectReason),
                targetRecord->collisionLayer,
                targetRecord->motionId,
                static_cast<int>(targetRecord->motionType));
            return;
        }

        const auto uniqueMotionBodyIds = bodySet.uniqueAcceptedMotionBodyIds();
        if (uniqueMotionBodyIds.empty()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: accepted target body {} produced no unique motion bodies",
                sourceName,
                targetBodyId);
            return;
        }

        auto* sourceMotion = hknp->GetBodyMotion(RE::hknpBodyId{ sourceBodyId });
        if (!sourceMotion) {
            return;
        }

        const RE::NiPoint3 sourceVelocityHavok{ sourceMotion->linearVelocity.x, sourceMotion->linearVelocity.y, sourceMotion->linearVelocity.z };
        const std::uint64_t cooldownKey = (static_cast<std::uint64_t>(sourceBodyId) << 32) | targetBodyId;
        float cooldownRemaining = 0.0f;
        if (const auto it = _dynamicPushCooldownUntil.find(cooldownKey); it != _dynamicPushCooldownUntil.end() && it->second > _dynamicPushElapsedSeconds) {
            cooldownRemaining = it->second - _dynamicPushElapsedSeconds;
        }

        const push_assist::PushAssistInput<RE::NiPoint3> pushInput{
            .enabled = g_rockConfig.rockDynamicPushAssistEnabled,
            .sourceVelocity = sourceVelocityHavok,
            .minSpeed = g_rockConfig.rockDynamicPushMinSpeed,
            .maxImpulse = g_rockConfig.rockDynamicPushMaxImpulse,
            .layerMultiplier = 1.0f,
            .cooldownRemainingSeconds = cooldownRemaining,
        };
        const auto push = push_assist::computePushImpulse(pushInput);
        if (!push.apply) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: reason={} speed=({:.3f},{:.3f},{:.3f}) targetBody={} layer={} acceptedBodies={}",
                sourceName,
                pushAssistSkipReasonName(push.skipReason),
                sourceVelocityHavok.x,
                sourceVelocityHavok.y,
                sourceVelocityHavok.z,
                targetBodyId,
                targetRecord->collisionLayer,
                bodySet.acceptedCount());
            return;
        }

        std::uint32_t appliedCount = 0;
        for (const auto bodyId : uniqueMotionBodyIds) {
            physics_recursive_wrappers::activateBody(hknp, bodyId);
            if (push_assist::applyLinearVelocityDeltaDeferred(hknp, bodyId, push.impulse)) {
                ++appliedCount;
            }
        }

        if (appliedCount > 0) {
            _dynamicPushCooldownUntil[cooldownKey] =
                _dynamicPushElapsedSeconds + (std::max)(0.0f, g_rockConfig.rockDynamicPushCooldownSeconds);
            auto* baseObj = targetRef->GetObjectReference();
            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const char* nameStr = objName.empty() ? "(unnamed)" : objName.data();
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push applied: '{}' formID={:08X} targetBody={} layer={} acceptedBodies={} uniqueMotions={} deltaVel=({:.3f},{:.3f},{:.3f})",
                sourceName,
                nameStr,
                targetRef->GetFormID(),
                targetBodyId,
                targetRecord->collisionLayer,
                bodySet.acceptedCount(),
                appliedCount,
                push.impulse.x,
                push.impulse.y,
                push.impulse.z);
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

            ROCK_LOG_DEBUG(Hand, "{} hand touched [{}] '{}' formID={:08X} body={} layer={}", handName, typeName, nameStr, ref->GetFormID(), bodyId.value, layer);

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

        const bool bodyAIsWeapon = _weaponCollision.isWeaponBodyIdAtomic(bodyIdA);
        const bool bodyBIsWeapon = _weaponCollision.isWeaponBodyIdAtomic(bodyIdB);
        if (bodyAIsWeapon != bodyBIsWeapon) {
            const std::uint32_t sourceWeaponBody = bodyAIsWeapon ? bodyIdA : bodyIdB;
            const std::uint32_t otherBody = bodyAIsWeapon ? bodyIdB : bodyIdA;
            if (otherBody != rightId && otherBody != leftId && !_weaponCollision.isWeaponBodyIdAtomic(otherBody)) {
                _lastContactSourceWeapon.store(sourceWeaponBody, std::memory_order_release);
                _lastContactBodyWeapon.store(otherBody, std::memory_order_release);
            }
        }

        if (!isRight && !isLeft) {
            return;
        }

        if (isLeft) {
            const std::uint32_t otherForLeft = (bodyIdA == leftId) ? bodyIdB : bodyIdA;
            WeaponInteractionContact weaponContact{};
            if (_weaponCollision.tryGetWeaponContactAtomic(otherForLeft, weaponContact)) {
                _leftWeaponContactPartKind.store(static_cast<std::uint32_t>(weaponContact.partKind), std::memory_order_release);
                _leftWeaponContactReloadRole.store(static_cast<std::uint32_t>(weaponContact.reloadRole), std::memory_order_release);
                _leftWeaponContactSupportRole.store(static_cast<std::uint32_t>(weaponContact.supportGripRole), std::memory_order_release);
                _leftWeaponContactSocketRole.store(static_cast<std::uint32_t>(weaponContact.socketRole), std::memory_order_release);
                _leftWeaponContactActionRole.store(static_cast<std::uint32_t>(weaponContact.actionRole), std::memory_order_release);
                _leftWeaponContactGripPose.store(static_cast<std::uint32_t>(weaponContact.fallbackGripPose), std::memory_order_release);
                _leftWeaponContactSequence.fetch_add(1, std::memory_order_acq_rel);
                _leftWeaponContactMissedFrames.store(0, std::memory_order_release);
                _leftWeaponContactBodyId.store(otherForLeft, std::memory_order_release);
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
