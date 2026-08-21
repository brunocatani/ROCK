/*
 * SHARED DETAIL HELPERS for the PhysicsInteraction translation units.
 *
 * Everything here is in rock::physics_interaction_detail and is declared in
 * PhysicsInteractionInternal.h. A helper belongs here only when two or more
 * PhysicsInteraction TUs call it. A helper used by one TU stays in that TU's
 * anonymous namespace: internal linkage is still the default.
 */

#include "physics-interaction/core/PhysicsInteractionInternal.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string_view>

#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"

#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

namespace rock::physics_interaction_detail
{
    namespace
    {
        enum class GrabButtonReadMode
        {
            ConsumeAll,
            PeekHeld,
            PeekPressed,
        };

        enum class PalmClockLogMode : std::uint8_t
        {
            Disabled,
            Sampled,
            Trace,
        };

        template <class Probe>
        Probe makeHeldObjectProbe(
            RE::hknpWorld* world,
            const Hand& hand,
            const HandFrameInput& handInput)
        {
            Probe probe{};
            if (hand.tryGetHeldObjectGrabPivotWorld(world, probe.pointGame)) {
                return probe;
            }

            // Use the body frame when the live grab pivot is unavailable.
            const auto& savedState = hand.getSavedObjectState();
            RE::NiTransform heldBodyWorld{};
            if (savedState.isValid() &&
                world &&
                tryGetBodyWorldTransform(world, savedState.bodyId, heldBodyWorld)) {
                probe.pointGame = heldBodyWorld.translate;
            } else {
                // Fail closed to the hand anchor for incomplete body state.
                probe.pointGame = handInput.grabAnchorWorld;
            }
            return probe;
        }

        [[nodiscard]] GrabButtonState readGrabButtonStateImpl(
            bool isLeft,
            int buttonId,
            GrabButtonReadMode mode)
        {
            if (!input_remap_policy::isAllowedGrabButtonId(buttonId)) {
                return {};
            }

            // Only the full-frame reader consumes remapped edge state.
            const auto rawState = mode == GrabButtonReadMode::ConsumeAll ?
                input_remap_runtime::consumeRawButtonState(isLeft, buttonId) :
                input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return GrabButtonState{
                    .held = rawState.held,
                    .pressed = rawState.pressed,
                    .released = rawState.released,
                };
            }

            const auto vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;
            switch (mode) {
            case GrabButtonReadMode::PeekHeld:
                return GrabButtonState{
                    .held = vrcf::VRControllers.isPressHeldDown(vrHand, buttonId),
                };
            case GrabButtonReadMode::PeekPressed:
                return GrabButtonState{
                    .pressed = vrcf::VRControllers.isPressed(vrHand, buttonId),
                };
            case GrabButtonReadMode::ConsumeAll:
            default:
                return GrabButtonState{
                    .held = vrcf::VRControllers.isPressHeldDown(vrHand, buttonId),
                    .pressed = vrcf::VRControllers.isPressed(vrHand, buttonId),
                    .released = vrcf::VRControllers.isReleased(vrHand, buttonId),
                };
            }
        }

        [[nodiscard]] const char* physicsStepPhaseName(
            havok_physics_timing::PhysicsStepPhase phase)
        {
            switch (phase) {
            case havok_physics_timing::PhysicsStepPhase::WholePreStep:
                return "whole-pre";
            case havok_physics_timing::PhysicsStepPhase::SubstepPreCollide:
                return "substep-pre-collide";
            case havok_physics_timing::PhysicsStepPhase::BetweenCollideAndSolveFinish:
                return "between-finish-before-solve";
            case havok_physics_timing::PhysicsStepPhase::SubstepPostSolve:
                return "substep-post-solve";
            }
            return "unknown";
        }

        [[nodiscard]] PalmClockLogMode palmClockLogMode()
        {
            if (g_rockConfig.rockDebugGrabTimelineTrace) {
                return PalmClockLogMode::Trace;
            }
            if (g_rockConfig.rockDebugGrabFrameLogging ||
                g_rockConfig.rockDebugVerboseLogging) {
                return PalmClockLogMode::Sampled;
            }
            return PalmClockLogMode::Disabled;
        }

        [[nodiscard]] bool palmClockTraceFrameSelected(std::uint64_t gameFrameIndex)
        {
            const auto interval = static_cast<std::uint64_t>(
                (std::max)(1, g_rockConfig.rockDebugGrabTimelineTraceIntervalFrames));
            return gameFrameIndex <= 3 || (gameFrameIndex % interval) == 0;
        }

        [[nodiscard]] RE::EquippedWeaponData* getValidatedEquippedWeaponDataImpl()
        {
            auto* equipWeaponData = f4vr::getEquippedWeaponData();
            if (!equipWeaponData) {
                return nullptr;
            }

            const auto vtable =
                *reinterpret_cast<const std::uintptr_t*>(equipWeaponData);
            if (vtable != f4vr::EquippedWeaponData_vtable.address()) {
                return nullptr;
            }
            return equipWeaponData;
        }

        [[nodiscard]] RE::NiNode* resolveEquippedWeaponInteractionNodeDirect()
        {
            auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
            return firstPersonSkeleton ?
                f4vr::findNode(firstPersonSkeleton, "Weapon") :
                nullptr;
        }
    }

    bool providerHandAnimationAuthorityActive()
    {
        const auto animationAuthorityFlags =
            provider::currentNativeAnimationAuthorityFlagsV1();
        return (animationAuthorityFlags &
                   (authored_weapon_grip_capture_policy::kArms |
                       authored_weapon_grip_capture_policy::kHands)) != 0;
    }

    std::uint32_t claimOwnerCount(std::uint32_t ownerMask)
    {
        std::uint32_t count = 0;
        while (ownerMask != 0) {
            count += ownerMask & 1u;
            ownerMask >>= 1u;
        }
        return count;
    }

    TransformDelta measureTransformDelta(
        const RE::NiTransform& a,
        const RE::NiTransform& b)
    {
        const float dx = a.translate.x - b.translate.x;
        const float dy = a.translate.y - b.translate.y;
        const float dz = a.translate.z - b.translate.z;

        const auto qa = niRotToHkQuat(a.rotate);
        const auto qb = niRotToHkQuat(b.rotate);
        const float dot = std::clamp(
            std::fabs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w),
            0.0f,
            1.0f);
        const float angleRadians = 2.0f * std::acos(dot);
        return TransformDelta{
            .position = std::sqrt(dx * dx + dy * dy + dz * dz),
            .rotationDegrees = angleRadians * (180.0f / std::numbers::pi_v<float>),
        };
    }

    float measurePointDelta(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    FarSelectionHmdConeGate makeFarSelectionHmdConeGate(
        const PhysicsFrameContext& frame)
    {
        FarSelectionHmdConeGate gate{};
        gate.enabled = g_rockConfig.rockFarSelectionHmdConeEnabled;
        gate.hasHmdFrame = frame.hasHmdFrame;
        gate.hmdPositionWorld = frame.hmdPositionWorld;
        gate.hmdForwardWorld = frame.hmdForwardWorld;
        gate.minDot = selection_query_policy::farSelectionHmdConeMinDot(
            g_rockConfig.rockFarSelectionHmdConeHalfAngleDegrees);
        return gate;
    }

    shoulder_stash::DetectorConfig makeShoulderStashDetectorConfig()
    {
        shoulder_stash::DetectorConfig config{};
        config.enabled = g_rockConfig.rockShoulderStashEnabled;
        config.useBodyZoneColliders = g_rockConfig.rockShoulderStashUseBodyZoneColliders;
        config.useHmdBackVolume = g_rockConfig.rockShoulderStashUseHmdBackVolume;
        config.enterPaddingGameUnits = g_rockConfig.rockShoulderStashEnterPaddingGameUnits;
        config.exitPaddingGameUnits = g_rockConfig.rockShoulderStashExitPaddingGameUnits;
        config.minDwellSeconds = g_rockConfig.rockShoulderStashMinDwellSeconds;
        config.maxSpeedGameUnitsPerSecond = g_rockConfig.rockShoulderStashMaxSpeedGameUnitsPerSecond;
        config.recentContactFrames = g_rockConfig.rockShoulderStashRecentContactFrames;
        config.sustainedContactMissFrames = g_rockConfig.rockShoulderStashSustainedContactMissFrames;
        config.hmdBackRightOffsetGameUnits = g_rockConfig.rockShoulderStashHmdBackRightOffsetGameUnits;
        config.hmdBackLeftOffsetGameUnits = g_rockConfig.rockShoulderStashHmdBackLeftOffsetGameUnits;
        config.hmdBackRadiusGameUnits = g_rockConfig.rockShoulderStashHmdBackRadiusGameUnits;
        config.hmdBackEnterPaddingGameUnits = g_rockConfig.rockShoulderStashHmdBackEnterPaddingGameUnits;
        config.hmdBackExitPaddingGameUnits = g_rockConfig.rockShoulderStashHmdBackExitPaddingGameUnits;
        config.hmdBackMinBehindGameUnits = g_rockConfig.rockShoulderStashHmdBackMinBehindGameUnits;
        return config;
    }

    shoulder_stash::DetectorConfig makeEquippedWeaponStashDetectorConfig(bool enabled)
    {
        // Equipped weapons use the loose-object back gesture without body contacts.
        shoulder_stash::DetectorConfig config = makeShoulderStashDetectorConfig();
        config.enabled = enabled;
        config.useBodyZoneColliders = false;
        config.useHmdBackVolume = true;
        return config;
    }

    shoulder_stash::Probe makeShoulderStashObjectProbe(
        RE::hknpWorld* world,
        const Hand& hand,
        const HandFrameInput& handInput)
    {
        return makeHeldObjectProbe<shoulder_stash::Probe>(world, hand, handInput);
    }

    shoulder_stash::Probe makeShoulderStashHmdProbe(const HandFrameInput& handInput)
    {
        shoulder_stash::Probe probe{};
        probe.pointGame = handInput.rawHandWorld.translate;
        return probe;
    }

    bool shouldEmitShoulderStashCandidatePulse(
        const shoulder_stash::Decision& decision,
        shoulder_stash::RuntimeState& state,
        float elapsedSeconds)
    {
        const bool pulseDue = elapsedSeconds >= state.nextCandidatePulseTimeSeconds;
        if (!decision.enteredCandidate && !decision.changedCandidate && !pulseDue) {
            return false;
        }

        // Reserve the next pulse before the caller emits feedback.
        state.nextCandidatePulseTimeSeconds =
            elapsedSeconds +
            (std::max)(
                0.02f,
                g_rockConfig.rockShoulderStashCandidateHapticIntervalSeconds);
        return true;
    }

    void showShoulderStashCollectedNotification(
        const shoulder_stash::TransferResult& transferResult,
        std::uint32_t fallbackFormID)
    {
        if (!g_rockConfig.rockShoulderStashShowCollectedNotifications) {
            return;
        }

        f4vr::showNotification(
            shoulder_stash_notification_policy::formatCollectedNotification(
                shoulderStashItemName(transferResult.baseForm),
                transferResult.count,
                transferResult.formID != 0 ? transferResult.formID : fallbackFormID));
    }

    std::string_view shoulderStashItemName(RE::TESBoundObject* baseForm)
    {
        if (!baseForm) {
            return {};
        }
        return RE::TESFullName::GetFullName(*baseForm, false);
    }

    mouth_consume::DetectorConfig makeMouthConsumeDetectorConfig()
    {
        mouth_consume::DetectorConfig config{};
        config.enabled = g_rockConfig.rockMouthConsumeEnabled;
        config.hmdMouthOffsetGameUnits = g_rockConfig.rockMouthConsumeHmdOffsetGameUnits;
        config.mouthRadiusGameUnits = g_rockConfig.rockMouthConsumeRadiusGameUnits;
        config.enterPaddingGameUnits = g_rockConfig.rockMouthConsumeEnterPaddingGameUnits;
        config.exitPaddingGameUnits = g_rockConfig.rockMouthConsumeExitPaddingGameUnits;
        config.minDwellSeconds = g_rockConfig.rockMouthConsumeMinDwellSeconds;
        config.maxSpeedGameUnitsPerSecond = g_rockConfig.rockMouthConsumeMaxSpeedGameUnitsPerSecond;
        return config;
    }

    mouth_consume::Probe makeMouthConsumeObjectProbe(
        RE::hknpWorld* world,
        const Hand& hand,
        const HandFrameInput& handInput)
    {
        return makeHeldObjectProbe<mouth_consume::Probe>(world, hand, handInput);
    }

    mouth_consume::Probe makeMouthConsumeHandProbe(const HandFrameInput& handInput)
    {
        mouth_consume::Probe probe{};
        probe.pointGame = handInput.grabAnchorWorld;
        return probe;
    }

    bool isInvalidGrabBodyId(std::uint32_t bodyId)
    {
        return bodyId == kInvalidAtomicBodyId ||
               bodyId == ROCK_GRAB_EVENT_INVALID_BODY_ID ||
               bodyId == object_physics_body_set::INVALID_BODY_ID;
    }

    std::uint64_t packHeldImpactPair(
        std::uint32_t heldBodyId,
        std::uint32_t otherBodyId)
    {
        if (isInvalidGrabBodyId(heldBodyId) || isInvalidGrabBodyId(otherBodyId)) {
            return kInvalidHeldImpactPair;
        }
        return (static_cast<std::uint64_t>(heldBodyId) << 32) |
               static_cast<std::uint64_t>(otherBodyId);
    }

    bool unpackHeldImpactPair(
        std::uint64_t packedPair,
        std::uint32_t& heldBodyId,
        std::uint32_t& otherBodyId)
    {
        if (packedPair == kInvalidHeldImpactPair) {
            return false;
        }
        heldBodyId = static_cast<std::uint32_t>(packedPair >> 32);
        otherBodyId = static_cast<std::uint32_t>(packedPair & 0xFFFF'FFFFu);
        return !isInvalidGrabBodyId(heldBodyId) && !isInvalidGrabBodyId(otherBodyId);
    }

    float readGrabEventBodyMass(RE::hknpWorld* world, std::uint32_t bodyId)
    {
        if (!world || isInvalidGrabBodyId(bodyId)) {
            return 0.0f;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
        if (!motion) {
            return 0.0f;
        }

        const auto packedInvMass =
            static_cast<std::int16_t>(motion->packedInverseInertia[3]);
        if (packedInvMass == 0) {
            return 0.0f;
        }
        return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
    }

    GrabButtonState readGrabButtonState(bool isLeft, int buttonId)
    {
        return readGrabButtonStateImpl(
            isLeft,
            buttonId,
            GrabButtonReadMode::ConsumeAll);
    }

    bool readGrabButtonHeld(bool isLeft, int buttonId)
    {
        return readGrabButtonStateImpl(
            isLeft,
            buttonId,
            GrabButtonReadMode::PeekHeld)
            .held;
    }

    bool readGrabButtonPressedEdge(bool isLeft, int buttonId)
    {
        return readGrabButtonStateImpl(
            isLeft,
            buttonId,
            GrabButtonReadMode::PeekPressed)
            .pressed;
    }

    bool readHeldWeaponEquipTriggerPressedEdge(bool isLeft)
    {
        constexpr int buttonId = input_remap_policy::kOpenVrSteamVrTriggerButtonId;
        const auto rawState =
            input_remap_runtime::consumeRawButtonState(isLeft, buttonId);
        if (rawState.available) {
            return rawState.pressed;
        }

        return vrcf::VRControllers.isPressed(
            isLeft ? vrcf::Hand::Left : vrcf::Hand::Right,
            buttonId);
    }

    // This diagnostic is const-only over Hand on both caller threads.
    void logPalmClockSampleForHand(
        const char* stage,
        const Hand& hand,
        RE::hknpWorld* world,
        const RE::NiTransform* rawHandWorld,
        std::uint64_t gameFrameIndex,
        float gameDeltaSeconds,
        const havok_physics_timing::PhysicsTimingSample* timing)
    {
        const PalmClockLogMode mode = palmClockLogMode();
        if (mode == PalmClockLogMode::Disabled) {
            return;
        }
        if (mode == PalmClockLogMode::Trace &&
            !palmClockTraceFrameSelected(gameFrameIndex)) {
            return;
        }
        if (!hand.isHoldingAtomic() && !g_rockConfig.rockDebugVerboseLogging) {
            return;
        }

        RE::NiTransform palmTargetWorld{};
        const bool targetOk = hand.tryGetPalmAnchorTarget(palmTargetWorld);
        Hand::LivePalmAnchorReference livePalm{};
        const bool liveOk = hand.tryResolveLivePalmAnchorReference(world, livePalm);
        if (!targetOk && !liveOk && !rawHandWorld) {
            return;
        }

        const bool rawOk = rawHandWorld != nullptr;
        const TransformDelta rawToTarget = rawOk && targetOk ?
            measureTransformDelta(*rawHandWorld, palmTargetWorld) :
            TransformDelta{ -1.0f, -1.0f };
        const TransformDelta rawToLive = rawOk && liveOk ?
            measureTransformDelta(*rawHandWorld, livePalm.world) :
            TransformDelta{ -1.0f, -1.0f };
        const TransformDelta targetToLive = targetOk && liveOk ?
            measureTransformDelta(palmTargetWorld, livePalm.world) :
            TransformDelta{ -1.0f, -1.0f };

        const float rawDt = timing ? timing->rawDeltaSeconds : -1.0f;
        const float subDt = timing ? timing->substepDeltaSeconds : -1.0f;
        const float driveDt = timing ?
            havok_physics_timing::driveDeltaSeconds(*timing) :
            -1.0f;
        const float progress = timing ? timing->substepProgress : -1.0f;
        const std::uint32_t substepIndex = timing ? timing->substepIndex + 1 : 0;
        const std::uint32_t substepCount = timing ? timing->substepCount : 0;
        const char* physicsPhase = timing ?
            physicsStepPhaseName(timing->phase) :
            "game-frame";

        const char* rawState = rawOk ? "ok" : "none";
        const char* targetState = targetOk ? "ok" : "none";
        const char* liveState = liveOk ? "ok" : "none";
        const char* palmSource = liveOk ?
            body_frame::bodyFrameSourceCode(livePalm.source) :
            "none";
        const std::uint32_t palmMotion = liveOk ?
            livePalm.motionIndex :
            body_frame::kFreeMotionIndex;
        const RE::NiPoint3 rawPosition = rawOk ?
            rawHandWorld->translate :
            RE::NiPoint3{};
        const RE::NiPoint3 targetPosition = targetOk ?
            palmTargetWorld.translate :
            RE::NiPoint3{};
        const RE::NiPoint3 livePosition = liveOk ?
            livePalm.world.translate :
            RE::NiPoint3{};

        const auto emit = [&]() {
            if (mode == PalmClockLogMode::Trace) {
                ROCK_LOG_INFO(Hand,
                    "PALM_CLOCK stage={} hand={} frame={} holding={} raw={} target={} live={} body={} proxyBody={} gameDt={:.6f} physicsPhase={} rawDt={:.6f} subDt={:.6f} driveDt={:.6f} substep={}/{} progress={:.3f} rawToTarget={:.3f}gu/{:.3f}deg rawToLive={:.3f}gu/{:.3f}deg targetToLive={:.3f}gu/{:.3f}deg rawPos=({:.2f},{:.2f},{:.2f}) targetPos=({:.2f},{:.2f},{:.2f}) livePos=({:.2f},{:.2f},{:.2f}) liveSource={} liveMotion={}",
                    stage ? stage : "unknown", hand.handName(), gameFrameIndex,
                    hand.isHoldingAtomic() ? "yes" : "no", rawState, targetState,
                    liveState, hand.getCollisionBodyId().value,
                    hand.getGrabAuthorityProxyBodyId().value, gameDeltaSeconds,
                    physicsPhase, rawDt, subDt, driveDt, substepIndex, substepCount,
                    progress, rawToTarget.position, rawToTarget.rotationDegrees,
                    rawToLive.position, rawToLive.rotationDegrees,
                    targetToLive.position, targetToLive.rotationDegrees,
                    rawPosition.x, rawPosition.y, rawPosition.z,
                    targetPosition.x, targetPosition.y, targetPosition.z,
                    livePosition.x, livePosition.y, livePosition.z,
                    palmSource, palmMotion);
                return;
            }

            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "PALM_CLOCK stage={} hand={} frame={} holding={} raw={} target={} live={} body={} proxyBody={} gameDt={:.6f} physicsPhase={} rawDt={:.6f} subDt={:.6f} driveDt={:.6f} substep={}/{} progress={:.3f} rawToTarget={:.3f}gu/{:.3f}deg rawToLive={:.3f}gu/{:.3f}deg targetToLive={:.3f}gu/{:.3f}deg rawPos=({:.2f},{:.2f},{:.2f}) targetPos=({:.2f},{:.2f},{:.2f}) livePos=({:.2f},{:.2f},{:.2f}) liveSource={} liveMotion={}",
                stage ? stage : "unknown", hand.handName(), gameFrameIndex,
                hand.isHoldingAtomic() ? "yes" : "no", rawState, targetState,
                liveState, hand.getCollisionBodyId().value,
                hand.getGrabAuthorityProxyBodyId().value, gameDeltaSeconds,
                physicsPhase, rawDt, subDt, driveDt, substepIndex, substepCount,
                progress, rawToTarget.position, rawToTarget.rotationDegrees,
                rawToLive.position, rawToLive.rotationDegrees,
                targetToLive.position, targetToLive.rotationDegrees,
                rawPosition.x, rawPosition.y, rawPosition.z,
                targetPosition.x, targetPosition.y, targetPosition.z,
                livePosition.x, livePosition.y, livePosition.z,
                palmSource, palmMotion);
        };
        emit();
    }

    RE::TESObjectWEAP* currentEquippedWeaponForm()
    {
        auto* equipData = f4vr::getEquippedWeaponItem();
        auto* weaponForm = equipData ? equipData->item.object : nullptr;
        if (!weaponForm || weaponForm->formType != RE::ENUM_FORM_ID::kWEAP) {
            return nullptr;
        }
        return weaponForm->As<RE::TESObjectWEAP>();
    }

    RE::TBO_InstanceData* currentEquippedWeaponInstanceData(
        const RE::TESObjectWEAP* expectedWeapon)
    {
        auto* equipData = f4vr::getEquippedWeaponItem();
        if (!expectedWeapon || !equipData) {
            return nullptr;
        }

        auto* weaponForm = equipData->item.object;
        auto* equippedWeapon = weaponForm ?
            weaponForm->As<RE::TESObjectWEAP>() :
            nullptr;
        return equippedWeapon == expectedWeapon ?
            equipData->item.instanceData.get() :
            nullptr;
    }

    std::uint32_t currentEquippedWeaponFormId()
    {
        const auto* weapon = currentEquippedWeaponForm();
        return weapon ? weapon->formID : 0;
    }

    void fillProviderTransform(
        const RE::NiTransform& source,
        ::rock::provider::RockProviderTransform& target)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotate[static_cast<std::size_t>(row * 3 + column)] =
                    source.rotate.entry[row][column];
            }
        }
        target.translate[0] = source.translate.x;
        target.translate[1] = source.translate.y;
        target.translate[2] = source.translate.z;
        target.scale = source.scale;
    }

    RE::NiTransform providerTransformToNi(
        const ::rock::provider::RockProviderTransform& source)
    {
        RE::NiTransform result{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.rotate.entry[row][column] =
                    source.rotate[static_cast<std::size_t>(row * 3 + column)];
            }
        }
        result.translate.x = source.translate[0];
        result.translate.y = source.translate[1];
        result.translate.z = source.translate[2];
        result.scale = source.scale;
        return result;
    }

    std::uint32_t providerHandStateFlags(const Hand& hand, bool isLeft)
    {
        std::uint32_t flags = 0;
        if (hand.isTouching()) {
            flags |= static_cast<std::uint32_t>(
                ::rock::provider::RockProviderHandStateFlag::Touching);
        }
        if (hand.isHolding()) {
            flags |= static_cast<std::uint32_t>(
                ::rock::provider::RockProviderHandStateFlag::Holding);
        }
        const bool disabled = isLeft ?
            PhysicsInteraction::s_leftHandDisabled.load(std::memory_order_acquire) :
            PhysicsInteraction::s_rightHandDisabled.load(std::memory_order_acquire);
        if (disabled) {
            flags |= static_cast<std::uint32_t>(
                ::rock::provider::RockProviderHandStateFlag::PhysicsDisabled);
        }
        return flags;
    }

    void copyProviderString(
        char* target,
        std::size_t targetSize,
        const std::string& source)
    {
        if (!target || targetSize == 0) {
            return;
        }
        std::snprintf(target, targetSize, "%s", source.c_str());
        target[targetSize - 1] = '\0';
    }

    void copyProviderString(char* target, std::size_t targetSize, const char* source)
    {
        if (!target || targetSize == 0) {
            return;
        }
        std::snprintf(target, targetSize, "%s", source ? source : "");
        target[targetSize - 1] = '\0';
    }

    ::rock::provider::RockProviderPoint3 makeProviderPoint(
        const WeaponEvidencePoint3& point)
    {
        return ::rock::provider::RockProviderPoint3{
            .x = point.x,
            .y = point.y,
            .z = point.z,
        };
    }

    ::rock::provider::RockProviderPoint3 makeProviderPoint(const RE::NiPoint3& point)
    {
        return ::rock::provider::RockProviderPoint3{
            .x = point.x,
            .y = point.y,
            .z = point.z,
        };
    }

    ::rock::provider::RockProviderBodyContactTargetKind providerBodyContactTargetKind(
        contact_pipeline_policy::ContactEndpointKind kind)
    {
        using contact_pipeline_policy::ContactEndpointKind;
        using ::rock::provider::RockProviderBodyContactTargetKind;
        switch (kind) {
        case ContactEndpointKind::RightHand:
        case ContactEndpointKind::LeftHand:
            return RockProviderBodyContactTargetKind::Hand;
        case ContactEndpointKind::Weapon:
            return RockProviderBodyContactTargetKind::Weapon;
        case ContactEndpointKind::RightHeldObject:
        case ContactEndpointKind::LeftHeldObject:
            return RockProviderBodyContactTargetKind::HeldObject;
        case ContactEndpointKind::Body:
            return RockProviderBodyContactTargetKind::Body;
        case ContactEndpointKind::External:
            return RockProviderBodyContactTargetKind::External;
        case ContactEndpointKind::WorldSurface:
            return RockProviderBodyContactTargetKind::WorldSurface;
        case ContactEndpointKind::DynamicProp:
            return RockProviderBodyContactTargetKind::DynamicProp;
        case ContactEndpointKind::Actor:
            return RockProviderBodyContactTargetKind::Actor;
        case ContactEndpointKind::QueryOnly:
            return RockProviderBodyContactTargetKind::QueryOnly;
        case ContactEndpointKind::Unknown:
            break;
        }
        return RockProviderBodyContactTargetKind::Unknown;
    }

    RE::NiAVObject* getEquippedProjectileNode()
    {
        // The native fire node is the stable equipped projectile authority.
        auto* equipWeaponData = getValidatedEquippedWeaponData();
        return equipWeaponData ? equipWeaponData->fireNode : nullptr;
    }

    RE::EquippedWeaponData* getValidatedEquippedWeaponData()
    {
        return getValidatedEquippedWeaponDataImpl();
    }

    RE::NiNode* resolveEquippedWeaponInteractionNode()
    {
        // All equipped-weapon interaction uses the first-person Weapon root.
        if (!runtime_state::currentFrame().weaponDrawn) {
            return nullptr;
        }
        return resolveEquippedWeaponInteractionNodeDirect();
    }
}
