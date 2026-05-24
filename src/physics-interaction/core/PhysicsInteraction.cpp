#include "physics-interaction/core/PhysicsInteraction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/ContactSignalSubscriptionPolicy.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/grab/HeldPlayerSpaceRegistry.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/debug/PhysicsWorldOriginDiagnostics.h"
#include "physics-interaction/collision/PushAssist.h"
#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/PhysicsBodyFrame.h"

#include "RE/Bethesda/ActorValueInfo.h"
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
#include "f4vr/MiscStructs.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "f4sevr/Forms.h"
#include "vrcf/VRControllersManager.h"
#include <windows.h>

namespace rock
{
    namespace
    {
        constexpr float kRawParityWarnPosition = 0.10f;
        constexpr float kRawParityWarnRotationDegrees = 0.5f;
        constexpr float kRawParityFailPosition = 0.50f;
        constexpr float kRawParityFailRotationDegrees = 2.0f;
        constexpr int kRawParityWarnFrames = 2;
        constexpr int kRawParityFailFrames = 10;

        constexpr std::uint32_t claimOwnerBit(PhysicsObjectClaimOwner owner)
        {
            return 1u << static_cast<std::uint32_t>(owner);
        }

        constexpr PhysicsObjectClaimOwner claimOwnerForHand(bool isLeft)
        {
            return isLeft ? PhysicsObjectClaimOwner::LeftHand : PhysicsObjectClaimOwner::RightHand;
        }

        FarSelectionHmdConeGate makeFarSelectionHmdConeGate(const PhysicsFrameContext& frame)
        {
            FarSelectionHmdConeGate gate{};
            gate.enabled = g_rockConfig.rockFarSelectionHmdConeEnabled;
            gate.hasHmdFrame = frame.hasHmdFrame;
            gate.hmdPositionWorld = frame.hmdPositionWorld;
            gate.hmdForwardWorld = frame.hmdForwardWorld;
            gate.minDot = selection_query_policy::farSelectionHmdConeMinDot(g_rockConfig.rockFarSelectionHmdConeHalfAngleDegrees);
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

        shoulder_stash::Probe makeShoulderStashObjectProbe(RE::hknpWorld* world, const Hand& hand, const HandFrameInput& handInput)
        {
            shoulder_stash::Probe probe{};
            if (hand.tryGetHeldObjectGrabPivotWorld(world, probe.pointGame)) {
                return probe;
            }

            const auto& savedState = hand.getSavedObjectState();
            RE::NiTransform heldBodyWorld{};
            if (savedState.isValid() && world && tryGetBodyWorldTransform(world, savedState.bodyId, heldBodyWorld)) {
                probe.pointGame = heldBodyWorld.translate;
            } else {
                probe.pointGame = handInput.grabAnchorWorld;
            }
            return probe;
        }

        shoulder_stash::Probe makeShoulderStashHmdProbe(const HandFrameInput& handInput)
        {
            shoulder_stash::Probe probe{};
            probe.pointGame = handInput.rawHandWorld.translate;
            return probe;
        }

        std::string_view shoulderStashItemName(RE::TESBoundObject* baseForm)
        {
            if (!baseForm) {
                return {};
            }

            return RE::TESFullName::GetFullName(*baseForm, false);
        }

        void showShoulderStashCollectedNotification(const shoulder_stash::TransferResult& transferResult, std::uint32_t fallbackFormID)
        {
            if (!g_rockConfig.rockShoulderStashShowCollectedNotifications) {
                return;
            }

            f4vr::showNotification(shoulder_stash_notification_policy::formatCollectedNotification(
                shoulderStashItemName(transferResult.baseForm),
                transferResult.count,
                transferResult.formID != 0 ? transferResult.formID : fallbackFormID));
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

        constexpr std::uint32_t kInvalidAtomicBodyId = 0xFFFF'FFFFu;
        constexpr std::uint64_t kInvalidHeldImpactPair = 0xFFFF'FFFF'FFFF'FFFFull;

        bool isInvalidGrabBodyId(std::uint32_t bodyId)
        {
            return bodyId == kInvalidAtomicBodyId ||
                   bodyId == ROCK_GRAB_EVENT_INVALID_BODY_ID ||
                   bodyId == object_physics_body_set::INVALID_BODY_ID;
        }

        std::uint64_t packHeldImpactPair(std::uint32_t heldBodyId, std::uint32_t otherBodyId)
        {
            if (isInvalidGrabBodyId(heldBodyId) || isInvalidGrabBodyId(otherBodyId)) {
                return kInvalidHeldImpactPair;
            }
            return (static_cast<std::uint64_t>(heldBodyId) << 32) | static_cast<std::uint64_t>(otherBodyId);
        }

        bool unpackHeldImpactPair(std::uint64_t packedPair, std::uint32_t& heldBodyId, std::uint32_t& otherBodyId)
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

            const auto packedInvMass = static_cast<std::uint16_t>(motion->packedInverseInertia[3]);
            const std::uint32_t asUint = static_cast<std::uint32_t>(packedInvMass) << 16;
            float inverseMass = 0.0f;
            std::memcpy(&inverseMass, &asUint, sizeof(float));
            if (!std::isfinite(inverseMass) || inverseMass <= 0.0001f) {
                return 0.0f;
            }
            return 1.0f / inverseMass;
        }

        bool applyPlayerSpeedReduction(float previousReduction, float targetReduction)
        {
            previousReduction = held_mass_movement::sanitizeReduction(previousReduction);
            targetReduction = held_mass_movement::sanitizeReduction(targetReduction);
            if (std::fabs(previousReduction - targetReduction) <= 0.001f &&
                (targetReduction > 0.0f || previousReduction <= 0.0f)) {
                return true;
            }

            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* actorValues = RE::ActorValue::GetSingleton();
            if (!player || !actorValues || !actorValues->speedMult || !actorValues->carryWeight) {
                return false;
            }

            if (previousReduction > 0.0f) {
                player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->speedMult, previousReduction);
            }
            if (targetReduction > 0.0f) {
                player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->speedMult, -targetReduction);
            }

            player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->carryWeight, 0.1f);
            player->ModActorValue(RE::ACTOR_VALUE_MODIFIER::kTemporary, *actorValues->carryWeight, -0.1f);
            return true;
        }

        std::uint32_t fillGrabEventBodyKinematics(RE::hknpWorld* world, std::uint32_t bodyId, GrabEventData& eventData)
        {
            if (!world || isInvalidGrabBodyId(bodyId)) {
                return 0;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
            if (!motion) {
                return 0;
            }

            std::uint32_t flags = 0;
            const float scale = havokToGameScale();
            eventData.positionGame[0] = motion->position.x * scale;
            eventData.positionGame[1] = motion->position.y * scale;
            eventData.positionGame[2] = motion->position.z * scale;
            flags |= ROCK_GRAB_EVENT_FLAG_POSITION_VALID;

            eventData.velocityGame[0] = motion->linearVelocity.x * scale;
            eventData.velocityGame[1] = motion->linearVelocity.y * scale;
            eventData.velocityGame[2] = motion->linearVelocity.z * scale;
            flags |= ROCK_GRAB_EVENT_FLAG_VELOCITY_VALID;

            const float speedHavok = std::sqrt(
                motion->linearVelocity.x * motion->linearVelocity.x +
                motion->linearVelocity.y * motion->linearVelocity.y +
                motion->linearVelocity.z * motion->linearVelocity.z);
            if (std::isfinite(speedHavok)) {
                eventData.speedGameUnitsPerSecond = speedHavok * scale;
                flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
            }

            eventData.mass = readGrabEventBodyMass(world, bodyId);
            if (eventData.mass > 0.0f) {
                flags |= ROCK_GRAB_EVENT_FLAG_MASS_VALID;
            }

            return flags;
        }
        constexpr int kRawParitySummaryFrames = 300;
        constexpr int kRawParityLagFrames = 5;
        constexpr float kRawParityLagSlack = 0.05f;
        DirectSkeletonBoneReader s_directSkeletonBoneReader;
        std::uint32_t s_directSkeletonBoneLogCounter = 0;
        bool s_worldOriginDiagnosticsEnabledLogged = false;

        struct ContactEventCallbackInfo
        {
            void* fn = nullptr;
            std::uint64_t ctx = 0;
        };

        struct ContactEventSubscriptionBridge
        {
            struct NativeSlot
            {
                RE::hknpWorld* world = nullptr;
                void* signal = nullptr;
                std::uint32_t epoch = 0;
            };

            static constexpr std::size_t kMaxRetainedNativeSlots = 64;

            std::atomic<PhysicsInteraction*> instance{ nullptr };
            std::atomic<RE::hknpWorld*> world{ nullptr };
            std::atomic<void*> signal{ nullptr };
            std::atomic<std::uint32_t> subscriptionEpoch{ 0 };
            std::mutex retainedSlotMutex;
            std::array<NativeSlot, kMaxRetainedNativeSlots> retainedSlots{};
            std::size_t retainedSlotCount = 0;

            [[nodiscard]] bool hasRetainedNativeSlot(RE::hknpWorld* requestedWorld, void* requestedSignal)
            {
                if (!requestedWorld || !requestedSignal) {
                    return false;
                }

                std::scoped_lock lock(retainedSlotMutex);
                for (std::size_t i = 0; i < retainedSlotCount; ++i) {
                    const auto& slot = retainedSlots[i];
                    if (slot.world == requestedWorld && slot.signal == requestedSignal) {
                        return true;
                    }
                }
                return false;
            }

            bool rememberRetainedNativeSlot(RE::hknpWorld* subscribedWorld, void* subscribedSignal, std::uint32_t epoch)
            {
                if (!subscribedWorld || !subscribedSignal) {
                    return false;
                }

                std::scoped_lock lock(retainedSlotMutex);
                for (std::size_t i = 0; i < retainedSlotCount; ++i) {
                    auto& slot = retainedSlots[i];
                    if (slot.world == subscribedWorld && slot.signal == subscribedSignal) {
                        slot.epoch = epoch;
                        return true;
                    }
                }

                if (retainedSlotCount >= retainedSlots.size()) {
                    return false;
                }

                retainedSlots[retainedSlotCount++] = NativeSlot{
                    .world = subscribedWorld,
                    .signal = subscribedSignal,
                    .epoch = epoch,
                };
                return true;
            }
        };

        ContactEventSubscriptionBridge s_contactEventBridge;

        struct GrabButtonState
        {
            bool held{ false };
            bool pressed{ false };
            bool released{ false };
            bool syntheticPressed{ false };
        };

        GrabButtonState readGrabButtonState(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isValidButtonId(buttonId)) {
                return {};
            }

            const auto rawState = input_remap_runtime::consumeRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return GrabButtonState{ .held = rawState.held, .pressed = rawState.pressed, .released = rawState.released };
            }

            const auto vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;
            return GrabButtonState{
                .held = vrcf::VRControllers.isPressHeldDown(vrHand, buttonId),
                .pressed = vrcf::VRControllers.isPressed(vrHand, buttonId),
                .released = vrcf::VRControllers.isReleased(vrHand, buttonId),
            };
        }

        bool readGrabButtonHeld(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isValidButtonId(buttonId)) {
                return false;
            }

            const auto rawState = input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return rawState.held;
            }

            return vrcf::VRControllers.isPressHeldDown(isLeft ? vrcf::Hand::Left : vrcf::Hand::Right, buttonId);
        }

        bool readGrabButtonPressedEdge(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isValidButtonId(buttonId)) {
                return false;
            }

            const auto rawState = input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return rawState.pressed;
            }

            return vrcf::VRControllers.isPressed(isLeft ? vrcf::Hand::Left : vrcf::Hand::Right, buttonId);
        }

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

        bool startsWith(std::string_view value, std::string_view prefix)
        {
            return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
        }

        debug::SkeletonOverlayRole skeletonOverlayRoleForBone(std::string_view name)
        {
            if (startsWith(name, "RArm_Finger")) {
                return debug::SkeletonOverlayRole::RightFinger;
            }
            if (startsWith(name, "LArm_Finger")) {
                return debug::SkeletonOverlayRole::LeftFinger;
            }
            if (startsWith(name, "RArm_")) {
                return debug::SkeletonOverlayRole::RightArm;
            }
            if (startsWith(name, "LArm_")) {
                return debug::SkeletonOverlayRole::LeftArm;
            }
            if (startsWith(name, "RLeg_")) {
                return debug::SkeletonOverlayRole::RightLeg;
            }
            if (startsWith(name, "LLeg_")) {
                return debug::SkeletonOverlayRole::LeftLeg;
            }
            if (name == "Head" || name == "Neck") {
                return debug::SkeletonOverlayRole::Head;
            }
            return debug::SkeletonOverlayRole::Core;
        }

        std::string_view trimView(std::string_view value)
        {
            while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
                value.remove_prefix(1);
            }
            while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
                value.remove_suffix(1);
            }
            return value;
        }

        bool skeletonLogFilterMatches(std::string_view filter, std::string_view boneName)
        {
            filter = trimView(filter);
            if (filter.empty()) {
                return false;
            }

            while (!filter.empty()) {
                const std::size_t comma = filter.find(',');
                const std::string_view token = trimView(filter.substr(0, comma));
                if (token == boneName) {
                    return true;
                }
                if (comma == std::string_view::npos) {
                    break;
                }
                filter.remove_prefix(comma + 1);
            }
            return false;
        }

        std::uint32_t currentEquippedWeaponFormId()
        {
            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            return weaponForm ? weaponForm->formID : 0;
        }

        void fillProviderTransform(const RE::NiTransform& source, ::rock::provider::RockProviderTransform& target)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    target.rotate[static_cast<std::size_t>(row * 3 + column)] = source.rotate.entry[row][column];
                }
            }
            target.translate[0] = source.translate.x;
            target.translate[1] = source.translate.y;
            target.translate[2] = source.translate.z;
            target.scale = source.scale;
        }

        std::uint32_t providerHandStateFlags(const Hand& hand, bool isLeft)
        {
            std::uint32_t flags = 0;
            if (hand.isTouching()) {
                flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderHandStateFlag::Touching);
            }
            if (hand.isHolding()) {
                flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderHandStateFlag::Holding);
            }
            if (isLeft ? PhysicsInteraction::s_leftHandDisabled.load(std::memory_order_acquire) :
                         PhysicsInteraction::s_rightHandDisabled.load(std::memory_order_acquire)) {
                flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderHandStateFlag::PhysicsDisabled);
            }
            return flags;
        }

        void copyProviderString(char* target, std::size_t targetSize, const std::string& source)
        {
            if (!target || targetSize == 0) {
                return;
            }

            std::snprintf(target, targetSize, "%s", source.c_str());
            target[targetSize - 1] = '\0';
        }

        ::rock::provider::RockProviderPoint3 makeProviderPoint(const WeaponEvidencePoint3& point)
        {
            return ::rock::provider::RockProviderPoint3{ .x = point.x, .y = point.y, .z = point.z };
        }

        ::rock::provider::RockProviderPoint3 makeProviderPoint(const RE::NiPoint3& point)
        {
            return ::rock::provider::RockProviderPoint3{ .x = point.x, .y = point.y, .z = point.z };
        }

        ::rock::provider::RockProviderBodyContactTargetKind providerBodyContactTargetKind(contact_pipeline_policy::ContactEndpointKind kind)
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

        const char* weaponDiagnosticNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }

            const char* name = node->name.c_str();
            return name ? name : "";
        }

        const char* weaponSupportClassName(weapon_support_authority_policy::WeaponSupportWeaponClass weaponClass)
        {
            using weapon_support_authority_policy::WeaponSupportWeaponClass;

            switch (weaponClass) {
            case WeaponSupportWeaponClass::Unknown:
                return "Unknown";
            case WeaponSupportWeaponClass::Sidearm:
                return "Sidearm";
            case WeaponSupportWeaponClass::LongGun:
                return "LongGun";
            }
            return "Unknown";
        }

        const char* weaponSupportAuthorityModeName(weapon_support_authority_policy::WeaponSupportAuthorityMode mode)
        {
            using weapon_support_authority_policy::WeaponSupportAuthorityMode;

            switch (mode) {
            case WeaponSupportAuthorityMode::FullTwoHandedSolver:
                return "FullTwoHandedSolver";
            case WeaponSupportAuthorityMode::VisualOnlySupport:
                return "VisualOnlySupport";
            }
            return "Unknown";
        }

        struct WeaponSupportKeywordCache
        {
            bool initialized{ false };
            RE::BGSKeyword* pistolGrip{ nullptr };
            RE::BGSKeyword* rifleAssaultGrip{ nullptr };
            RE::BGSKeyword* rifleStraightGrip{ nullptr };
            RE::BGSKeyword* shoulderFiredGrip{ nullptr };
        };

        RE::BGSKeyword* resolveWeaponSupportKeyword(const char* editorID)
        {
            return editorID ? RE::TESForm::GetFormByEditorID<RE::BGSKeyword>(RE::BSFixedString(editorID)) : nullptr;
        }

        const WeaponSupportKeywordCache& weaponSupportKeywordCache()
        {
            static WeaponSupportKeywordCache cache{};
            if (!cache.initialized) {
                cache.initialized = true;
                cache.pistolGrip = resolveWeaponSupportKeyword("AnimsGripPistol");
                cache.rifleAssaultGrip = resolveWeaponSupportKeyword("AnimsGripRifleAssault");
                cache.rifleStraightGrip = resolveWeaponSupportKeyword("AnimsGripRifleStraight");
                cache.shoulderFiredGrip = resolveWeaponSupportKeyword("AnimsGripShoulderFired");
                if (!cache.pistolGrip || !cache.rifleAssaultGrip || !cache.rifleStraightGrip || !cache.shoulderFiredGrip) {
                    ROCK_LOG_WARN(
                        Weapon,
                        "Weapon support grip keyword lookup incomplete: AnimsGripPistol={} AnimsGripRifleAssault={} AnimsGripRifleStraight={} AnimsGripShoulderFired={}",
                        static_cast<const void*>(cache.pistolGrip),
                        static_cast<const void*>(cache.rifleAssaultGrip),
                        static_cast<const void*>(cache.rifleStraightGrip),
                        static_cast<const void*>(cache.shoulderFiredGrip));
                }
            }
            return cache;
        }

        bool keywordFormHasKeyword(const RE::BGSKeywordForm* keywordForm, const RE::BGSKeyword* keyword)
        {
            return keywordForm && keyword && keywordForm->HasKeyword(keyword, nullptr);
        }

        bool equippedWeaponHasKeyword(const RE::TESObjectWEAP* weapon, const RE::TBO_InstanceData* instanceData, const RE::BGSKeyword* keyword)
        {
            return weapon && keyword && weapon->HasKeyword(keyword, instanceData);
        }

        bool equippedInstanceHasKeyword(const RE::TBO_InstanceData* instanceData, const RE::BGSKeyword* keyword)
        {
            const auto* keywordData = instanceData ? instanceData->GetKeywordData() : nullptr;
            return keywordFormHasKeyword(keywordData, keyword);
        }

        bool equippedWeaponHasAnyKeyword(
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData,
            const RE::BGSKeyword* keywordA,
            const RE::BGSKeyword* keywordB,
            const RE::BGSKeyword* keywordC)
        {
            return equippedWeaponHasKeyword(weapon, instanceData, keywordA) ||
                   equippedWeaponHasKeyword(weapon, instanceData, keywordB) ||
                   equippedWeaponHasKeyword(weapon, instanceData, keywordC);
        }

        bool equippedInstanceHasAnyKeyword(
            const RE::TBO_InstanceData* instanceData,
            const RE::BGSKeyword* keywordA,
            const RE::BGSKeyword* keywordB,
            const RE::BGSKeyword* keywordC)
        {
            return equippedInstanceHasKeyword(instanceData, keywordA) ||
                   equippedInstanceHasKeyword(instanceData, keywordB) ||
                   equippedInstanceHasKeyword(instanceData, keywordC);
        }

        void populateEquippedWeaponKeywordIdentity(
            weapon_support_authority_policy::EquippedWeaponIdentity& identity,
            const RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* instanceData)
        {
            const auto& keywords = weaponSupportKeywordCache();
            identity.hasPistolGripKeyword = equippedWeaponHasKeyword(weapon, instanceData, keywords.pistolGrip);
            identity.hasInstancePistolGripKeyword = equippedInstanceHasKeyword(instanceData, keywords.pistolGrip);
            identity.hasLongGunGripKeyword = equippedWeaponHasAnyKeyword(
                weapon,
                instanceData,
                keywords.rifleAssaultGrip,
                keywords.rifleStraightGrip,
                keywords.shoulderFiredGrip);
            identity.hasInstanceLongGunGripKeyword = equippedInstanceHasAnyKeyword(
                instanceData,
                keywords.rifleAssaultGrip,
                keywords.rifleStraightGrip,
                keywords.shoulderFiredGrip);
        }

        const RE::TESObjectWEAP* asEquippedWeaponForm(const F4SEVR::TESForm* form)
        {
            if (!form || form->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
                return nullptr;
            }

            const auto* reForm = reinterpret_cast<const RE::TESForm*>(form);
            return reForm->As<RE::TESObjectWEAP>();
        }

        weapon_support_authority_policy::EquippedWeaponIdentity makeEquippedWeaponSupportIdentity(RE::NiNode* weaponNode)
        {
            weapon_support_authority_policy::EquippedWeaponIdentity identity{};
            identity.nodeName = weaponDiagnosticNodeName(weaponNode);

            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            auto* weaponForm = equipData ? equipData->item : nullptr;
            if (weaponForm) {
                identity.formID = weaponForm->formID;
                if (const char* fullName = weaponForm->GetFullName()) {
                    identity.displayName = fullName;
                }
                populateEquippedWeaponKeywordIdentity(identity, asEquippedWeaponForm(weaponForm), equipData ? equipData->instanceData : nullptr);
            }

            return identity;
        }

        weapon_support_authority_policy::WeaponSupportAuthorityMode resolveEquippedWeaponSupportAuthorityMode(RE::NiNode* weaponNode)
        {
            using namespace weapon_support_authority_policy;

            if (!g_rockConfig.rockVisualOnlySidearmSupportGripEnabled) {
                return WeaponSupportAuthorityMode::FullTwoHandedSolver;
            }

            const auto identity = makeEquippedWeaponSupportIdentity(weaponNode);
            const auto weaponClass = classifyEquippedWeaponForSupportGrip(identity);
            const auto authorityMode = resolveSupportAuthorityMode(true, weaponClass);
            ROCK_LOG_SAMPLE_DEBUG(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Weapon support classification: formID={:08X} name='{}' node='{}' pistolGrip={} instancePistolGrip={} longGunGrip={} instanceLongGunGrip={} class={} mode={}",
                identity.formID,
                identity.displayName,
                identity.nodeName,
                identity.hasPistolGripKeyword,
                identity.hasInstancePistolGripKeyword,
                identity.hasLongGunGripKeyword,
                identity.hasInstanceLongGunGripKeyword,
                weaponSupportClassName(weaponClass),
                weaponSupportAuthorityModeName(authorityMode));
            return authorityMode;
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
                    info.interactionRootName = sourceInfo.interactionRootName;
                    info.sourceRootName = sourceInfo.sourceRootName;
                }
                if (info.interactionRootName.empty()) {
                    info.interactionRootName = weaponDiagnosticNodeName(contact.interactionRoot);
                }
                if (info.sourceRootName.empty()) {
                    info.sourceRootName = weaponDiagnosticNodeName(contact.sourceRoot);
                }
            }

            return info;
        }

        f4vr::MuzzleFlash* getEquippedMuzzleFlashNodes()
        {
            /*
             * ROCK is the final weapon visual owner during mesh/hand authority.
             * Any ROCK weapon write after the normal first-person weapon update
             * must re-own the fire node from the current projectile node so the
             * muzzle origin remains at the barrel tip.
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

        RE::NiNode* resolveEquippedWeaponInteractionNode()
        {
            /*
             * Weapon interaction uses the same first-person weapon root for all
             * equipped weapons. Generated collision scans every known package
             * candidate internally, so this handoff should not branch by weapon
             * type or create a separate melee-owned update path.
             */
            if (!runtime_state::currentFrame().weaponDrawn) {
                return nullptr;
            }

            return f4vr::getWeaponNode();
        }
    }

    PhysicsInteraction::PhysicsInteraction(std::uint32_t skeletonGeneration, std::uint32_t providerGeneration)
    {
        s_instance.store(this, std::memory_order_release);
        _lifecycleState.skeletonGeneration = skeletonGeneration == 0 ? 1 : skeletonGeneration;
        _lifecycleState.providerGeneration = providerGeneration == 0 ? 1 : providerGeneration;
        _skeletonGenerationAtomic.store(_lifecycleState.skeletonGeneration, std::memory_order_release);
        _providerGenerationAtomic.store(_lifecycleState.providerGeneration, std::memory_order_release);
        _generatedBodyStepDrive.setDriveCallbacks(
            nullptr,
            &PhysicsInteraction::onGeneratedColliderPhysicsSubstep,
            &PhysicsInteraction::onCustomGrabAuthorityBetweenStep,
            &PhysicsInteraction::onCustomGrabAuthorityAfterSolve,
            this);

        installBumpHook();
        installNativeGrabHook();
        /*
         * PAPER is the reload owner. ROCK keeps hand, weapon, and contact
         * provider hooks active, but it must not install the native clip-write
         * gate because that would create two authorities for one ammo mutation
         * path. PAPER will reinstall verified native reload hooks after the
         * required Ghidra audit records the FO4VR addresses.
         */
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

    bool PhysicsInteraction::tryGetRootFlattenedHandTransform(bool isLeft, RE::NiTransform& outTransform) const
    {
        outTransform = {};
        if (!_handBoneCache.isReady()) {
            return false;
        }

        outTransform = _handBoneCache.getWorldTransform(isLeft);
        return true;
    }

    void PhysicsInteraction::noteSkeletonLifecycle(std::uint32_t skeletonGeneration, ::rock::provider::RockProviderLifecycleReason reason)
    {
        physics_lifecycle::noteSkeletonGeneration(_lifecycleState, skeletonGeneration, reason);
        physics_lifecycle::noteReason(_lifecycleState, reason);
        markGeneratedBodiesInvalidated();
        _lifecycleState.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        _lifecycleState.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::GeneratedBodiesValid);
        _lifecycleState.flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::LoadingOrWorldTransition);
        _lifecycleFlagsAtomic.store(_lifecycleState.flags, std::memory_order_release);
        _skeletonGenerationAtomic.store(_lifecycleState.skeletonGeneration, std::memory_order_release);
        _stableFrameCountAtomic.store(_lifecycleState.stableFrameCount, std::memory_order_release);
        _lastLifecycleReasonAtomic.store(static_cast<std::uint32_t>(_lifecycleState.lastReason), std::memory_order_release);
        _lifecycleHknpWorldAtomic.store(nullptr, std::memory_order_release);
    }

    void PhysicsInteraction::noteProviderLifecycle(std::uint32_t providerGeneration, ::rock::provider::RockProviderLifecycleReason reason)
    {
        physics_lifecycle::noteProviderGeneration(_lifecycleState, providerGeneration, reason);
        physics_lifecycle::noteReason(_lifecycleState, reason);
        markGeneratedBodiesInvalidated();
        _lifecycleState.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        _lifecycleFlagsAtomic.store(_lifecycleState.flags, std::memory_order_release);
        _providerGenerationAtomic.store(_lifecycleState.providerGeneration, std::memory_order_release);
        _stableFrameCountAtomic.store(_lifecycleState.stableFrameCount, std::memory_order_release);
        _lastLifecycleReasonAtomic.store(static_cast<std::uint32_t>(_lifecycleState.lastReason), std::memory_order_release);
    }

    bool PhysicsInteraction::generatedBodiesExistForConfig() const
    {
        return _rightHand.hasCollisionBody() && _leftHand.hasCollisionBody();
    }

    bool PhysicsInteraction::generatedBodiesMatchLifecycle(RE::bhkWorld* bhk, RE::hknpWorld* hknp) const
    {
        return generatedBodiesExistForConfig() &&
               _generatedBodiesBhkWorld == bhk &&
               _generatedBodiesHknpWorld == hknp &&
               _generatedBodiesWorldGeneration != 0 &&
               _generatedBodiesWorldGeneration == _lifecycleState.worldGeneration &&
               _generatedBodiesSkeletonGeneration == _lifecycleState.skeletonGeneration &&
               _generatedBodiesProviderGeneration == _lifecycleState.providerGeneration;
    }

    void PhysicsInteraction::markGeneratedBodiesRebuilt(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        if (!bhk || !hknp || !generatedBodiesExistForConfig()) {
            markGeneratedBodiesInvalidated();
            return;
        }

        _generatedBodiesBhkWorld = bhk;
        _generatedBodiesHknpWorld = hknp;
        _generatedBodiesWorldGeneration = _lifecycleState.worldGeneration;
        _generatedBodiesSkeletonGeneration = _lifecycleState.skeletonGeneration;
        _generatedBodiesProviderGeneration = _lifecycleState.providerGeneration;
    }

    void PhysicsInteraction::markGeneratedBodiesInvalidated()
    {
        _generatedBodiesBhkWorld = nullptr;
        _generatedBodiesHknpWorld = nullptr;
        _generatedBodiesWorldGeneration = 0;
        _generatedBodiesSkeletonGeneration = 0;
        _generatedBodiesProviderGeneration = 0;
        _lifecycleState.generatedBodiesValid = false;
        _lifecycleState.generatedBodiesWorldGeneration = 0;
        _lifecycleState.generatedBodiesSkeletonGeneration = 0;
        _lifecycleState.generatedBodiesProviderGeneration = 0;
        _lifecycleState.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::GeneratedBodiesValid);
        _lifecycleState.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        _lifecycleFlagsAtomic.store(_lifecycleState.flags, std::memory_order_release);
        _stableFrameCountAtomic.store(_lifecycleState.stableFrameCount, std::memory_order_release);
        _lifecycleHknpWorldAtomic.store(nullptr, std::memory_order_release);
        _generatedBodyStepDrive.reset();
        _shoulderStashStates = {};
    }

    bool PhysicsInteraction::rebuildGeneratedBodiesForLifecycle(RE::bhkWorld* bhk, RE::hknpWorld* hknp, const char* reason)
    {
        if (!bhk || !hknp) {
            markGeneratedBodiesInvalidated();
            return false;
        }

        ROCK_LOG_INFO(Init,
            "Rebuilding ROCK generated bodies for lifecycle reason={} worldGen={} skeletonGen={} providerGen={}",
            reason ? reason : "unknown",
            _lifecycleState.worldGeneration,
            _lifecycleState.skeletonGeneration,
            _lifecycleState.providerGeneration);

        destroyHandCollisions(bhk);
        destroyBodyBoneCollisions(bhk);

        if (!createHandCollisions(hknp, bhk)) {
            ROCK_LOG_WARN(Init, "Generated body lifecycle rebuild failed while creating hand colliders");
            markGeneratedBodiesInvalidated();
            _handColliderCreateRetryFrames = 120;
            return false;
        }

        if (g_rockConfig.rockBodyBoneCollidersEnabled && !createBodyBoneCollisions(hknp, bhk)) {
            ROCK_LOG_WARN(Init, "Generated body lifecycle rebuild continuing without body bone colliders; runtime update will retry");
            _bodyBoneColliderCreateRetryFrames = 120;
        }

        _rightHand.updateCollisionTransform(hknp, 0.011f);
        _leftHand.updateCollisionTransform(hknp, 0.011f);
        _bodyBoneColliders.update(hknp, 0.011f);
        _bodyContactRuntime.reset();
        markGeneratedBodiesRebuilt(bhk, hknp);
        return generatedBodiesMatchLifecycle(bhk, hknp);
    }

    void PhysicsInteraction::observeLifecycleFrame(RE::bhkWorld* bhk, RE::hknpWorld* hknp, ::rock::provider::RockProviderLifecycleReason reasonHint)
    {
        const auto& runtime = runtime_state::currentFrame();
        physics_lifecycle::FrameInputs inputs{};
        inputs.bhkWorld = reinterpret_cast<std::uintptr_t>(bhk);
        inputs.hknpWorld = reinterpret_cast<std::uintptr_t>(hknp);
        inputs.skeletonGeneration = _lifecycleState.skeletonGeneration;
        inputs.providerGeneration = _lifecycleState.providerGeneration;
        inputs.providerReady = _initialized.load(std::memory_order_acquire) && runtime.visualAuthorityAvailable;
        inputs.skeletonReady = runtime.localSkeletonReady;
        inputs.menuBlocking = runtime.localMenuBlocking;
        inputs.configBlocking = runtime.compatibilityConfigBlocking;
        inputs.generatedBodiesValid = generatedBodiesExistForConfig();
        inputs.generatedBodiesWorldGeneration = _generatedBodiesWorldGeneration;
        inputs.generatedBodiesSkeletonGeneration = _generatedBodiesSkeletonGeneration;
        inputs.generatedBodiesProviderGeneration = _generatedBodiesProviderGeneration;
        inputs.reasonHint = reasonHint;

        physics_lifecycle::observeFrame(_lifecycleState, inputs);
        _cachedBhkWorld = bhk;
        _cachedHknpWorld = hknp;
        _lifecycleFlagsAtomic.store(_lifecycleState.flags, std::memory_order_release);
        _lastLifecycleReasonAtomic.store(static_cast<std::uint32_t>(_lifecycleState.lastReason), std::memory_order_release);
        _worldGenerationAtomic.store(_lifecycleState.worldGeneration, std::memory_order_release);
        _skeletonGenerationAtomic.store(_lifecycleState.skeletonGeneration, std::memory_order_release);
        _providerGenerationAtomic.store(_lifecycleState.providerGeneration, std::memory_order_release);
        _stableFrameCountAtomic.store(_lifecycleState.stableFrameCount, std::memory_order_release);
        _lifecycleHknpWorldAtomic.store(hknp, std::memory_order_release);
    }

    bool PhysicsInteraction::physicsWritesAllowedForWorld(RE::hknpWorld* world) const
    {
        if (!world || world != _lifecycleHknpWorldAtomic.load(std::memory_order_acquire)) {
            return false;
        }

        return ::rock::provider::hasLifecycleFlag(
            _lifecycleFlagsAtomic.load(std::memory_order_acquire),
            ::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
    }

#include "physics-interaction/core/PhysicsInteractionProvider.inl"
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

        if (!havok_runtime::getBodyArray(hknp)) {
            ROCK_LOG_ERROR(Init, "hknpWorld body array pointer returned null");
            return false;
        }

        ROCK_LOG_INFO(Init, "Critical offset validation passed");
        return true;
    }

    bool PhysicsInteraction::refreshHandBoneCache()
    {
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
        const bool cacheReady = _handBoneCache.isReady();
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, cacheReady ? _handBoneCache.getWorldTransform(isLeft) : RE::NiTransform());
        if (frame.valid) {
            return frame.transform;
        }

        return RE::NiTransform();
    }

    RE::NiNode* PhysicsInteraction::getInteractionHandNode(bool isLeft) const
    {
        const bool cacheReady = _handBoneCache.isReady();
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, cacheReady ? _handBoneCache.getWorldTransform(isLeft) : RE::NiTransform());
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

        if (!frik_visual_authority::isAvailable() || !_handBoneCache.isReady()) {
            return;
        }

        if (!_parityEnabledLogged) {
            ROCK_LOG_INFO(Init, "Hand-transform parity enabled (root flattened cache vs FRIK API, pre-write sampling)");
            _parityEnabledLogged = true;
        }

        const bool playerMoving = runtime_state::currentFrame().playerSpace.moving;
        const bool emitSummary = (++_paritySummaryCounter >= kRawParitySummaryFrames);

        auto sampleHand = [&](bool isLeft) {
            auto& state = _rawHandParityStates[isLeft ? 1 : 0];
            const auto handEnum = handFromBool(isLeft);
            const auto localTransform = _handBoneCache.getWorldTransform(isLeft);
            const auto apiTransform = frik_visual_authority::getHandWorldTransform(handEnum);
            const auto delta = measureTransformDelta(localTransform, apiTransform);
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
                ROCK_LOG_DEBUG(Hand, "{} parity: raw(pos={:.3f}, rot={:.3f}deg) basis(palmPos={:.3f}, palmNormal={:.3f}deg, pointing={:.3f}deg)", summaryHandLabel,
                    delta.position, delta.rotationDegrees, measurePointDelta(localPalmPosition, apiPalmPosition), measureDirectionDeltaDegrees(localPalmNormal, apiPalmNormal),
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

        const bool nativeMeleeSuppressionHooksInstalled = installNativeMeleeSuppressionHooks();
        if (!nativeMeleeSuppressionHooksInstalled && g_rockConfig.rockNativeMeleeSuppressionEnabled) {
            ROCK_LOG_CRITICAL(Init, "Native melee suppression requested but hook installation failed; ROCK will continue without melee suppression");
        } else if (nativeMeleeSuppressionHooksInstalled) {
            enforceNativeMeleeRuntimeSuppression(true);
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

        physics_scale::refreshAndLogIfChanged();

        _cachedBhkWorld = bhk;
        _cachedHknpWorld = hknp;
        if (!refreshHandBoneCache()) {
            ROCK_LOG_WARN(Init, "HandBoneCache not ready during init; runtime remains on pre-00 transform paths");
        }

        registerCollisionLayer(hknp);
        if (!_collisionLayerRegistered) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: collision layer registration failed");
            _cachedBhkWorld = nullptr;
            _cachedHknpWorld = nullptr;
            return;
        }

        if (!createHandCollisions(hknp, bhk)) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: hand collision body creation failed");
            _cachedBhkWorld = nullptr;
            _cachedHknpWorld = nullptr;
            return;
        }

        if (g_rockConfig.rockBodyBoneCollidersEnabled && !createBodyBoneCollisions(hknp, bhk)) {
            ROCK_LOG_WARN(Init, "Body bone colliders were not available during init; runtime update will retry");
        }

        _handContactActivity.reset();
        _bodyContactRuntime.reset();
        subscribeContactEvents(hknp);

        _weaponCollision.init(hknp, bhk);

        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_Physics", true)) {
            ROCK_LOG_INFO(Init, "FRIK offhand grip permanently suppressed");
        }

        {
            _rightHand.updateCollisionTransform(hknp, 0.011f);
            _leftHand.updateCollisionTransform(hknp, 0.011f);
            _bodyBoneColliders.update(hknp, 0.011f);
            ROCK_LOG_INFO(Init, "Initial bone-derived hand collider transforms updated");
        }

        _hasPrevPositions = false;
        _hasHeldPlayerSpacePosition = false;
        _heldObjectPlayerSpaceFrame = {};
        _heldPlayerSpaceLogCounter = 0;
        _deltaLogCounter = 0;
        _contactLogCounter = 0;
        _softContactRuntime.reset();
        _nativeContactEvidence.reset();
        _bodyContactRuntime.reset();
        _dynamicPushElapsedSeconds = 0.0f;
        _dynamicPushCooldownUntil.clear();
        _heldImpactHapticCooldownUntil.clear();
        _grabEventFrameCounter = 0;
        _grabInputIntentStates = {};
        _lastContactBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastHeldImpactPairRight.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _lastHeldImpactPairLeft.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _handContactActivity.reset();

        _initialized = true;
        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::None);
        markGeneratedBodiesRebuilt(bhk, hknp);
        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesRebuilt);

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsInit, false);

        ROCK_LOG_INFO(Init, "ROCK physics module initialized — bhkWorld={}, hknpWorld={}, R_body={}, L_body={}", static_cast<const void*>(bhk), static_cast<const void*>(hknp),
            _rightHand.getCollisionBodyId().value, _leftHand.getCollisionBodyId().value);
    }

#include "physics-interaction/core/PhysicsInteractionFrame.inl"

    void PhysicsInteraction::update()
    {
        const auto& runtime = runtime_state::currentFrame();
        if (!runtime.visualAuthorityAvailable) {
            restoreHeldMassMovementSlowdown("frik-unavailable");
            return;
        }

        vrcf::VRControllers.update(f4vr::isLeftHandedMode());

        _deltaTime = runtime.deltaSeconds;

        if (_deltaTime <= 0.0f || _deltaTime > 0.1f) {
            _deltaTime = 1.0f / 90.0f;
        }
        advanceNativeMeleeFrameClock();
        enforceNativeMeleeRuntimeSuppression();
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

        if (!runtime.localSkeletonReady) {
            if (_initialized) {
                ROCK_LOG_WARN(Update, "Local skeleton no longer ready — shutting down");
                shutdown();
            }
            return;
        }

        const bool menuBlocking = runtime.localMenuBlocking;
        auto updateWeaponVisualGraphRefresh = [&](bool graphMenuBlocking, RE::NiAVObject* weaponNode) {
            return _weaponVisualGraphRefresh.update(WeaponVisualGraphRefreshCoordinator::UpdateInput{
                .enabled = g_rockConfig.rockEnabled && g_rockConfig.rockWeaponCollisionEnabled,
                .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
                .skeletonReady = runtime.localSkeletonReady,
                .menuBlocking = graphMenuBlocking,
                .weaponDrawn = runtime.weaponDrawn,
                .weaponNode = weaponNode,
            });
        };
        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                menuBlocking,
                false,
                false)) {
            updateWeaponVisualGraphRefresh(true, nullptr);
            if (_initialized) {
                _twoHandedGrip.reset();
                auto* bhkMenu = getPlayerBhkWorld();
                if (bhkMenu) {
                    auto* hknpMenu = getHknpWorld(bhkMenu);
                    if (hknpMenu) {
                        restoreRightHandCollisionAfterDominantWeapon(hknpMenu);
                        restoreLeftHandCollisionAfterWeaponSupport(hknpMenu);
                        _softContactRuntime.reset();
                        _nativeContactEvidence.reset();
                        if (_rightHand.isHolding()) {
                            auto* r = _rightHand.getHeldRef();
                            _rightHand.releaseGrabbedObject(hknpMenu, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(_rightHand, false));
                            if (r)
                                releaseObject(r, PhysicsObjectClaimOwner::RightHand);
                        }
                        if (_leftHand.isHolding()) {
                            auto* r = _leftHand.getHeldRef();
                            _leftHand.releaseGrabbedObject(hknpMenu, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(_leftHand, true));
                            if (r)
                                releaseObject(r, PhysicsObjectClaimOwner::LeftHand);
                        }
                    }
                } else {
                    _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
                    _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
                    hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
                    hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
                    _softContactRuntime.reset();
                    _nativeContactEvidence.reset();
                }
            }
            debug::ClearFrame();
            auto* snapshotBhk = getPlayerBhkWorld();
            auto* snapshotHknp = snapshotBhk ? getHknpWorld(snapshotBhk) : nullptr;
            observeLifecycleFrame(snapshotBhk, snapshotHknp, ::rock::provider::RockProviderLifecycleReason::MenuBlocked);
            restoreHeldMassMovementSlowdown("menu-blocked");
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                false,
                !g_rockConfig.rockEnabled,
                false)) {
            _weaponVisualGraphRefresh.reset();
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
            _cachedHknpWorld = nullptr;
            observeLifecycleFrame(bhk, nullptr, ::rock::provider::RockProviderLifecycleReason::WorldUnavailable);
            _softContactRuntime.reset();
            _nativeContactEvidence.reset();
            debug::ClearFrame();
            restoreHeldMassMovementSlowdown("world-unavailable");
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }
        _cachedHknpWorld = hknp;

        if (physics_scale::refreshAndLogIfChanged()) {
            ROCK_LOG_WARN(Config, "Authoritative Havok scale changed; invalidating ROCK-generated collision bodies");
            /*
             * A live scale change is a physics-frame convention change, not a
             * cosmetic setting reload. Existing constraints and ROCK-owned shapes
             * were authored with the previous conversion, so active interactions
             * must yield before generated bodies are destroyed and rebuilt.
             */
            auto releaseHeldForScaleChange = [&](Hand& hand, bool isLeft) {
                if (!hand.isHolding()) {
                    return;
                }

                auto* heldRef = hand.getHeldRef();
                hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, makeGrabReleaseContext(hand, isLeft));
                if (heldRef) {
                    releaseObject(heldRef, claimOwnerForHand(isLeft));
                }
            };
            releaseHeldForScaleChange(_rightHand, false);
            releaseHeldForScaleChange(_leftHand, true);

            restoreRightHandCollisionAfterDominantWeapon(hknp);
            restoreLeftHandCollisionAfterWeaponSupport(hknp);
            _twoHandedGrip.reset();
            _softContactRuntime.reset();
            _nativeContactEvidence.reset();
            _bodyContactRuntime.reset();
            clearLeftWeaponContact();

            destroyHandCollisions(bhk);
            destroyBodyBoneCollisions(bhk);
            _weaponCollision.invalidateForScaleChange(hknp);
            markGeneratedBodiesInvalidated();
            hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
            _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            restoreNativePlayerCollisionSuppression(hknp, "scale-change");
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        refreshHandBoneCache();
        sampleHandTransformParity();
        const auto frame = buildFrameContext(bhk, hknp, _deltaTime);

        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::None);
        if (!generatedBodiesMatchLifecycle(bhk, hknp)) {
            if (rebuildGeneratedBodiesForLifecycle(bhk, hknp, "epoch-mismatch")) {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesRebuilt);
            } else {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesInvalidated);
                ROCK_LOG_SAMPLE_DEBUG(Update,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "ROCK lifecycle generated-body rebuild pending: flags=0x{:08X} reason={} worldGen={} skeletonGen={} providerGen={} stableFrames={}",
                    _lifecycleFlagsAtomic.load(std::memory_order_acquire),
                    _lastLifecycleReasonAtomic.load(std::memory_order_acquire),
                    _worldGenerationAtomic.load(std::memory_order_acquire),
                    _skeletonGenerationAtomic.load(std::memory_order_acquire),
                    _providerGenerationAtomic.load(std::memory_order_acquire),
                    _stableFrameCountAtomic.load(std::memory_order_acquire));
                debug::ClearFrame();
                ::rock::provider::dispatchFrameCallbacks(*this);
                return;
            }
        }

        if (!physicsWritesAllowedForWorld(hknp)) {
            ROCK_LOG_SAMPLE_DEBUG(Update,
                g_rockConfig.rockLogSampleMilliseconds,
                "ROCK lifecycle gate closed frame: flags=0x{:08X} reason={} worldGen={} skeletonGen={} providerGen={} stableFrames={}",
                _lifecycleFlagsAtomic.load(std::memory_order_acquire),
                _lastLifecycleReasonAtomic.load(std::memory_order_acquire),
                _worldGenerationAtomic.load(std::memory_order_acquire),
                _skeletonGenerationAtomic.load(std::memory_order_acquire),
                _providerGenerationAtomic.load(std::memory_order_acquire),
                _stableFrameCountAtomic.load(std::memory_order_acquire));
            debug::ClearFrame();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        if (_collisionLayerRegistered &&
            (_expectedHandLayerMask != 0 || _expectedWeaponLayerMask != 0 || _expectedReloadLayerMask != 0 || _expectedBodyLayerMask != 0 ||
                _nativeCharacterControllerLayerPolicyCaptured)) {
            const auto desiredHandMask = collision_layer_policy::buildRockHandExpectedMask(true, g_rockConfig.rockHandCollisionStaticWorldEnabled);
            const auto desiredWeaponMask = collision_layer_policy::buildRockWeaponExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockWeaponCollisionStaticWorldEnabled,
                true);
            const auto desiredReloadMask = collision_layer_policy::buildRockReloadExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockHandCollisionStaticWorldEnabled);
            const auto desiredBodyMask = collision_layer_policy::buildRockBodyExpectedMask(g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled);
            const bool desiredNativeControllerPolicyEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
            const bool nativeControllerPolicyModeChanged =
                _nativeCharacterControllerLayerPolicyCaptured &&
                _nativeCharacterControllerLayerPolicyEnabled != desiredNativeControllerPolicyEnabled;
            if (!collision_layer_policy::matrixLayerMaskMatches(_expectedHandLayerMask, desiredHandMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedWeaponLayerMask, desiredWeaponMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedReloadLayerMask, desiredReloadMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedBodyLayerMask, desiredBodyMask) ||
                nativeControllerPolicyModeChanged) {
                ROCK_LOG_INFO(Config, "ROCK collision layer config changed; re-registering matrix policy");
                _collisionLayerRegistered = false;
                registerCollisionLayer(hknp);
            }

            if (auto* matrix = havok_runtime::getCollisionFilterMatrix(hknp)) {
                const auto currentHandMask = matrix[collision_layer_policy::ROCK_LAYER_HAND];
                const auto currentWeaponMask = matrix[collision_layer_policy::ROCK_LAYER_WEAPON];
                const auto currentReloadMask = matrix[collision_layer_policy::ROCK_LAYER_RELOAD];
                const auto currentBodyMask = matrix[collision_layer_policy::ROCK_LAYER_BODY];
                const bool handMaskDrifted = _expectedHandLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentHandMask, _expectedHandLayerMask);
                const bool weaponMaskDrifted = _expectedWeaponLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentWeaponMask, _expectedWeaponLayerMask);
                const bool reloadMaskDrifted = _expectedReloadLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentReloadMask, _expectedReloadLayerMask);
                const bool bodyMaskDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::bodyManagedLayerMaskMatches(currentBodyMask, _expectedBodyLayerMask);
                const bool actorToolPairsDrifted =
                    _expectedHandLayerMask != 0 && _expectedWeaponLayerMask != 0 &&
                    !collision_layer_policy::rockToolActorPairsMatch(matrix, _expectedHandLayerMask, _expectedWeaponLayerMask);
                const bool bodyPairsDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::rockBodyManagedPairsMatch(matrix, _expectedBodyLayerMask);
                const bool nativeControllerObjectPairsDrifted =
                    _nativeCharacterControllerLayerPolicyCaptured &&
                    !collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix, _expectedNativeCharacterControllerLayerMask);
                if (handMaskDrifted || weaponMaskDrifted || reloadMaskDrifted || bodyMaskDrifted || actorToolPairsDrifted || bodyPairsDrifted ||
                    nativeControllerObjectPairsDrifted) {
                    const auto currentNativeCharacterControllerMask =
                        _nativeCharacterControllerLayerPolicyCaptured ? matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER] : 0;
                    ROCK_LOG_WARN(Config,
                        "ROCK configured layer mask drift detected; hand expected=0x{:016X} current=0x{:016X}, weapon expected=0x{:016X} current=0x{:016X}, reload expected=0x{:016X} current=0x{:016X}, body expected=0x{:016X} current=0x{:016X}, nativeController expected=0x{:016X} current=0x{:016X}, actorToolPairs={}, bodyManagedPairs={}, nativeControllerObjects={}; re-registering",
                        collision_layer_policy::matrixAddressableMask(_expectedHandLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentHandMask),
                        collision_layer_policy::matrixAddressableMask(_expectedWeaponLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentWeaponMask),
                        collision_layer_policy::matrixAddressableMask(_expectedReloadLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentReloadMask),
                        collision_layer_policy::matrixAddressableMask(_expectedBodyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentBodyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedNativeCharacterControllerLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentNativeCharacterControllerMask),
                        actorToolPairsDrifted ? "drifted" : "ok",
                        bodyPairsDrifted ? "drifted" : "ok",
                        nativeControllerObjectPairsDrifted ? "drifted" : "ok");
                    _collisionLayerRegistered = false;
                    registerCollisionLayer(hknp);
                }
            }
        }

        RE::NiNode* weaponNode = resolveEquippedWeaponInteractionNode();
        const auto weaponGraphRefresh = updateWeaponVisualGraphRefresh(false, weaponNode);
        if (weaponGraphRefresh.refreshApplied) {
            _weaponCollision.invalidateForVisualGraphRefresh(hknp, "equipped-weapon-3d-update-flag");
            clearLeftWeaponContact();
        }
        if (weaponGraphRefresh.deferWeaponCollision) {
            weaponNode = nullptr;
        }
        const bool rightHandWeaponEquipped = weaponNode != nullptr;
        bool leftSupportGripActive = false;
        if (rightHandWeaponEquipped) {
            suppressRightHandCollisionForDominantWeapon(hknp);
        } else {
            restoreRightHandCollisionAfterDominantWeapon(hknp);
        }

        updateHandCollisions(frame);
        updateBodyBoneCollisions(frame);
        updateNativePlayerCollisionSuppression(bhk, hknp);

        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollision);
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
            _weaponCollision.update(hknp, weaponNode, dominantHandBodyId, frame.deltaSeconds);
        }

        _generatedBodyStepDrive.registerForNextStep(bhk, hknp);

        {
            WeaponInteractionContact leftWeaponContact{};
            auto leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::None;

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
                if (!_weaponCollision.tryGetWeaponContactAtomic(leftWeaponBodyId, leftWeaponContact)) {
                    clearLeftWeaponContact();
                    leftWeaponContact = {};
                    leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::None;
                } else {
                    leftWeaponContact.sequence = _leftWeaponContactSequence.load(std::memory_order_acquire);
                    leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::Contact;
                }
            } else if (weaponNode) {
                const RE::NiPoint3 leftProbePoint = frame.left.grabAnchorWorld;
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

            const bool gripPressed = readGrabButtonHeld(true, g_rockConfig.rockGrabButtonID);
            const bool gripConfirmPressed = readGrabButtonPressedEdge(true, g_rockConfig.rockGrabButtonID);
            (void)gripConfirmPressed;

            WeaponInteractionRuntimeState providerInteractionState{};
            const auto offhandReservation = offhand_interaction_reservation::fromProvider(::rock::provider::currentOffhandReservation());
            if (!offhand_interaction_reservation::allowsSupportGrip(offhandReservation)) {
                providerInteractionState.supportGripAllowed = false;
            }

            const WeaponInteractionDecision leftWeaponDecision = routeWeaponInteraction(leftWeaponContact, providerInteractionState);
            const std::uint64_t currentWeaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();
            const auto weaponNotificationKey = weapon_debug_notification_policy::makeWeaponNotificationKey(
                leftWeaponContact,
                leftWeaponDecision,
                leftWeaponContactSource);

            const bool leftHandHoldingObject = _leftHand.isHolding();
            const auto supportAuthorityMode = resolveEquippedWeaponSupportAuthorityMode(weaponNode);

            _twoHandedGrip.update(
                weaponNode,
                leftWeaponContact,
                gripPressed,
                leftHandHoldingObject,
                frame.deltaSeconds,
                currentWeaponGenerationKey,
                providerInteractionState,
                supportAuthorityMode);
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
                            "WeaponGripDiagnostics: weapon='{}' formID={:08X} node='{}' driveRoot='{}' sourceRoot='{}' nif='{}' part={} route={} pose={} body={} source={}",
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponName),
                            weaponDebugInfo.weaponFormId,
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.weaponNodeName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.interactionRootName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceRootName),
                            weapon_debug_notification_policy::debugTextOrUnknown(weaponDebugInfo.sourceName),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.partKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.interactionKind),
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.gripPose),
                            weaponNotificationKey.bodyId,
                            weapon_debug_notification_policy::nameOf(weaponNotificationKey.source));
                    } else {
                        f4vr::showNotification(weapon_debug_notification_policy::formatWeaponGripNotification(gripNotificationEvent, weaponNotificationKey));
                    }
                }
            } else {
                _weaponDebugNotificationState.supportGripActive = weaponSupportGripActive;
            }
            leftSupportGripActive = weaponSupportGripActive;

            if (weaponSupportGripActive) {
                suppressLeftHandCollisionForWeaponSupport(hknp);
            } else {
                restoreLeftHandCollisionAfterWeaponSupport(hknp);
            }

            if (weaponNode) {
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollision);
                _weaponCollision.updateBodiesFromCurrentSourceTransforms(hknp, weaponNode, frame.deltaSeconds);
            }
            if (f4vr::isNodeVisible(weaponNode)) {
                applyFinalWeaponMuzzleAuthority();
            }
        }

        updateSelection(frame);

        _heldObjectPlayerSpaceFrame = sampleHeldObjectPlayerSpaceFrame(frame.deltaSeconds);
        /*
         * ROCK applies player/room-space compensation before held-object grab
         * constraints are updated. That keeps the constraint target from solving
         * against a stale body velocity and removes the apparent held-object
         * teleport/stutter caused by compensating after the grab loop has already
         * written the frame target.
         */
        applyHeldPlayerSpaceVelocity(hknp);

        updateGrabInput(frame);
        updateHeldMassMovementSlowdown(hknp, frame.deltaSeconds);
        synchronizeContactEvidenceOwnership(rightHandWeaponEquipped, leftSupportGripActive);

        /*
         * Soft contact is intentionally evaluated after normal grab input. A
         * free-hand touch is visual-only; a grab, pull, support grip, or weapon
         * owner is transform authority. Running this after updateGrabInput lets
         * the solver see the final owner state for the frame, while explicit
         * pre-grab clears below prevent stale lower-priority FRIK hand targets
         * from surviving into grab-frame capture.
         */
        contact_evidence::NativeContactEvidenceSnapshot nativeContactEvidence{};
        _nativeContactEvidence.snapshot(nativeContactEvidence, _handContactActivity.currentFrame());
        _softContactRuntime.update(
            frame,
            _rightHand,
            _leftHand,
            _weaponCollision,
            weaponNode,
            rightHandWeaponEquipped,
            leftSupportGripActive,
            nativeContactEvidence);

        publishDebugBodyOverlay(frame);

        resolveContacts(frame);

        bool wasTouchingR = _rightHand.isTouching();
        bool wasTouchingL = _leftHand.isTouching();
        _rightHand.tickTouchState();
        _leftHand.tickTouchState();
        _rightHand.tickSemanticContactState();
        _leftHand.tickSemanticContactState();
        _handContactActivity.advanceFrame();
        if (wasTouchingR && !_rightHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, false, _rightHand.getLastTouchedRef(), _rightHand.getLastTouchedFormID(), _rightHand.getLastTouchedLayer());
        }
        if (wasTouchingL && !_leftHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, true, _leftHand.getLastTouchedRef(), _leftHand.getLastTouchedFormID(), _leftHand.getLastTouchedLayer());
        }

        _deltaLogCounter++;
        if (g_rockConfig.rockDebugVerboseLogging && _deltaLogCounter >= 90) {
            _deltaLogCounter = 0;

            const auto& playerSpace = runtime_state::currentFrame().playerSpace;
            if (playerSpace.valid) {
                const auto smoothPos = playerSpace.world.translate;
                const bool moving = playerSpace.moving;

                if (_hasPrevPositions && moving) {
                    const auto smoothDelta = smoothPos - _prevSmoothedPos;

                    ROCK_LOG_DEBUG(Update, "PlayerSpace: smoothDelta=({:.2f},{:.2f},{:.2f}) moving={}", smoothDelta.x, smoothDelta.y, smoothDelta.z, moving);
                }

                _prevSmoothedPos = smoothPos;
                _hasPrevPositions = true;
            }
        }

        ::rock::provider::dispatchFrameCallbacks(*this);
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

    bool PhysicsInteraction::isHandContactEvidenceSuppressed(bool isLeft) const
    {
        /*
         * Native hknp contact callbacks can run on the physics boundary while
         * game-frame ownership is changing. Use only atomic state here: the
         * physics thread needs ROCK's "hand collision disabled while owned"
         * answer without reading Hand::_state directly.
         */
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        return hand.hasContactEvidenceSuppressedAtomic() ||
               (!isLeft && _rightDominantWeaponCollisionSuppressed.load(std::memory_order_acquire)) ||
               (isLeft && _leftWeaponSupportCollisionSuppressed.load(std::memory_order_acquire));
    }

    void PhysicsInteraction::clearContactEvidenceForHand(bool isLeft, const char* reason)
    {
        if (isLeft) {
            _leftHand.clearSemanticContactEvidence();
        } else {
            _rightHand.clearSemanticContactEvidence();
        }

        const std::uint32_t invalidatedNative = _nativeContactEvidence.invalidateHand(isLeft);
        if (invalidatedNative > 0) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} hand contact evidence invalidated for stronger owner ({}) nativeRecords={}",
                isLeft ? "Left" : "Right",
                reason ? reason : "unknown",
                invalidatedNative);
        }
    }

    void PhysicsInteraction::synchronizeContactEvidenceOwnership(bool rightHandWeaponEquipped, bool leftSupportGripActive)
    {
        /*
         * ROCK disables generated hand collision when a grab or two-hand/tool
         * owner has the hand. Callback records can still arrive from the hknp
         * step boundary, so producer caches must be invalidated at the same
         * authority transition before the visual solver snapshots them.
         */
        if (_rightHand.hasContactEvidenceSuppressedAtomic() || rightHandWeaponEquipped ||
            _rightDominantWeaponCollisionSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(false, rightHandWeaponEquipped ? "right-hand-equipped-weapon" : "right-hand-grab-owner");
        }

        if (_leftHand.hasContactEvidenceSuppressedAtomic() || leftSupportGripActive ||
            _leftWeaponSupportCollisionSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(true, leftSupportGripActive ? "equipped-weapon-support-grip" : "left-hand-grab-owner");
        }
    }

    void PhysicsInteraction::suppressRightHandCollisionForDominantWeapon(RE::hknpWorld* world)
    {
        /*
         * The equipped gun already owns the dominant-hand pose and weapon aim.
         * Letting the generated right-hand bodies keep colliding while that
         * authority is active creates a second physical owner: stale hand
         * contacts can push props, feed semantic touch, or leak native evidence
         * even though visual soft contact is suppressed. ROCK treats owned tool
         * states as collision-filter ownership, so it uses the shared suppression
         * lease here instead of a visual-only gate.
         */
        _rightDominantWeaponCollisionSuppressed.store(true, std::memory_order_release);

        if (!world || !_rightHand.hasCollisionBody()) {
            return;
        }

        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                return;
            }

            const auto suppression = hand_collision_suppression_math::beginSuppression(_rightDominantWeaponCollisionSuppression, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Weapon, "DominantWeapon: right hand suppression set full; bodyId={} left active", bodyId);
                return;
            }

            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::WeaponDominantHand,
                "dominant-weapon-hand");

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "DominantWeapon: right hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    bodyId,
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = _rightHand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_rightHand.getHandColliderBodyIdAtomic(i));
            }
        } else {
            suppressBody(_rightHand.getCollisionBodyId().value);
        }
    }

    void PhysicsInteraction::restoreRightHandCollisionAfterDominantWeapon(RE::hknpWorld* world)
    {
        if (!hand_collision_suppression_math::hasActive(_rightDominantWeaponCollisionSuppression)) {
            _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon, "DominantWeapon: cannot restore right hand collision yet (world=null); preserving suppression leases");
            return;
        }

        bool restoreDeferred = false;
        for (const auto& entry : _rightDominantWeaponCollisionSuppression.entries) {
            if (!entry.active || entry.bodyId == INVALID_CONTACT_BODY_ID) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,
                entry.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::WeaponDominantHand,
                "dominant-weapon-hand");
            if (releaseResult.readFailed) {
                restoreDeferred = true;
                continue;
            }

            ROCK_LOG_DEBUG(Weapon,
                "DominantWeapon: right hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }

        if (restoreDeferred) {
            ROCK_LOG_WARN(Weapon, "DominantWeapon: right hand collision restore deferred; suppression leases preserved");
            return;
        }

        hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
        _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::suppressLeftHandCollisionForWeaponSupport(RE::hknpWorld* world)
    {
        /*
         * Layer 43 vs 44 is intentionally allowed so the offhand can physically
         * touch the equipped weapon before support grip starts. Once support
         * grip owns the transform, the left hand body becomes a driver and must
         * stop solving against the weapon package just like held-object hand
         * collision suppression.
         */
        _leftWeaponSupportCollisionSuppressed.store(true, std::memory_order_release);

        if (!world || !_leftHand.hasCollisionBody()) {
            return;
        }

        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                return;
            }

            const auto suppression = hand_collision_suppression_math::beginSuppression(_leftWeaponSupportCollisionSuppression, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left hand support suppression set full; bodyId={} left active", bodyId);
                return;
            }

            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::WeaponSupportHand,
                "weapon-support-hand");

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: left hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    bodyId,
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = _leftHand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_leftHand.getHandColliderBodyIdAtomic(i));
            }
        } else {
            suppressBody(_leftHand.getCollisionBodyId().value);
        }
    }

    void PhysicsInteraction::restoreLeftHandCollisionAfterWeaponSupport(RE::hknpWorld* world)
    {
        if (!hand_collision_suppression_math::hasActive(_leftWeaponSupportCollisionSuppression)) {
            _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: cannot restore left hand support collision yet (world=null); preserving suppression leases");
            return;
        }

        bool restoreDeferred = false;
        for (const auto& entry : _leftWeaponSupportCollisionSuppression.entries) {
            if (!entry.active || entry.bodyId == INVALID_CONTACT_BODY_ID) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,
                entry.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::WeaponSupportHand,
                "weapon-support-hand");
            if (releaseResult.readFailed) {
                restoreDeferred = true;
                continue;
            }

            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: left hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }

        if (restoreDeferred) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left hand support collision restore deferred; suppression leases preserved");
            return;
        }

        hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
        _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::shutdown(::rock::provider::RockProviderLifecycleReason reason)
    {
        if (!_initialized) {
            return;
        }

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsShutdown, false);

        ROCK_LOG_INFO(Init, "Shutting down ROCK physics module...");
        restoreHeldMassMovementSlowdown("shutdown");

        auto* currentBhk = getPlayerBhkWorld();
        const bool worldValid = _cachedBhkWorld && currentBhk == _cachedBhkWorld;

        if (worldValid) {
            auto* hknp = getHknpWorld(_cachedBhkWorld);
            unsubscribeContactEvents(hknp);
            restoreNativePlayerCollisionSuppression(hknp, "shutdown");
            restoreRightHandCollisionAfterDominantWeapon(hknp);
            restoreLeftHandCollisionAfterWeaponSupport(hknp);
            if (_rightHand.isHolding()) {
                auto* r = _rightHand.getHeldRef();
                _rightHand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, makeGrabReleaseContext(_rightHand, false));
                if (r)
                    releaseObject(r, PhysicsObjectClaimOwner::RightHand);
            }
            if (_leftHand.isHolding()) {
                auto* r = _leftHand.getHeldRef();
                _leftHand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, makeGrabReleaseContext(_leftHand, true));
                if (r)
                    releaseObject(r, PhysicsObjectClaimOwner::LeftHand);
            }
            _weaponCollision.destroyWeaponBody(hknp);
            destroyBodyBoneCollisions(_cachedBhkWorld);
            destroyHandCollisions(_cachedBhkWorld);
        } else {
            unsubscribeContactEvents(nullptr);
            ROCK_LOG_INFO(Init, "World stale or null — skipping Havok body destruction");
            _rightHand.abandonHavokStateAfterWorldLoss();
            _leftHand.abandonHavokStateAfterWorldLoss();
            _bodyBoneColliders.reset();
            _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
            _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            _nativePlayerCollisionSuppressedBodyCount = 0;
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            _nativePlayerCollisionSuppressionOverflowLogged = false;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        debug::ClearFrame();
        _twoHandedGrip.reset();
        _softContactRuntime.reset();
        _bodyContactRuntime.reset();
        _shoulderStashStates = {};
        _weaponCollision.shutdown();
        _weaponVisualGraphRefresh.reset();
        _bodyBoneColliders.reset();
        _generatedBodyStepDrive.reset();
        markGeneratedBodiesInvalidated();
        clearNativeMeleePhysicalSwingLeases();
        collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        ::rock::provider::clearExternalBodiesForProviderLoss();
        clearLeftWeaponContact();
        releaseAllObjects();
        _rightHand.reset();
        _leftHand.reset();

        _cachedBhkWorld = nullptr;
        _cachedHknpWorld = nullptr;
        _collisionLayerRegistered = false;
        _expectedHandLayerMask = 0;
        _expectedWeaponLayerMask = 0;
        _expectedReloadLayerMask = 0;
        _expectedBodyLayerMask = 0;
        _originalNativeCharacterControllerLayerMask = 0;
        _expectedNativeCharacterControllerLayerMask = 0;
        _nativeCharacterControllerLayerPolicyCaptured = false;
        _nativeCharacterControllerLayerPolicyEnabled = false;
        _initialized = false;
        observeLifecycleFrame(nullptr, nullptr, reason);
        _hasPrevPositions = false;
        _hasHeldPlayerSpacePosition = false;
        _heldObjectPlayerSpaceFrame = {};
        _heldPlayerSpaceLogCounter = 0;
        _heldMassMovementLogCounter = 0;
        _handBoneCache.reset();
        _handCacheResolveLogCounter = 0;
        _paritySummaryCounter = 0;
        _parityEnabledLogged = false;
        _runtimeScaleLogged = false;
        _rawHandParityStates = {};
        _dynamicPushCooldownUntil.clear();
        _heldImpactHapticCooldownUntil.clear();
        _grabEventFrameCounter = 0;
        _grabInputIntentStates = {};
        _bodyBoneColliderCreateRetryFrames = 0;
        _handColliderCreateRetryFrames = 0;
        _lastContactBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceRight.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceLeft.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastContactSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _lastHeldImpactPairRight.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _lastHeldImpactPairLeft.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _handContactActivity.reset();
        _nativeContactEvidence.reset();
        _bodyContactRuntime.reset();
        hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
        hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
        _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
        _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
        _nativePlayerCollisionSuppressedBodyCount = 0;
        _nativePlayerCollisionSuppressionRefreshFrames = 0;
        _nativePlayerCollisionSuppressionOverflowLogged = false;

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

    void PhysicsInteraction::pruneHeldImpactHapticCooldowns()
    {
        if (_heldImpactHapticCooldownUntil.size() < 128) {
            return;
        }

        for (auto it = _heldImpactHapticCooldownUntil.begin(); it != _heldImpactHapticCooldownUntil.end();) {
            if (it->second <= _dynamicPushElapsedSeconds) {
                it = _heldImpactHapticCooldownUntil.erase(it);
            } else {
                ++it;
            }
        }
    }

    void PhysicsInteraction::handleGrabEventHaptics(const GrabEventData& eventData)
    {
        auto triggerHaptic = [](bool isLeft, float durationSeconds, float intensity) {
            const float duration = std::clamp(std::isfinite(durationSeconds) ? durationSeconds : 0.0f, 0.0f, 0.2f);
            const float clampedIntensity = std::clamp(std::isfinite(intensity) ? intensity : 0.0f, 0.0f, 1.0f);
            if (duration <= 0.0f || clampedIntensity <= 0.0f) {
                return;
            }

            f4cf::vrcf::VRControllers.triggerHaptic(
                isLeft ? f4cf::vrcf::Hand::Left : f4cf::vrcf::Hand::Right,
                duration,
                clampedIntensity);
        };

        if ((eventData.flags & ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC) != 0) {
            return;
        }

        switch (eventData.type) {
        case GrabEventType::SelectionLocked:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                triggerHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockSelectionLockHapticIntensity);
            }
            return;
        case GrabEventType::SelectionUnlocked:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                triggerHaptic(
                    eventData.isLeft, g_rockConfig.rockSelectionLockReleaseHapticDurationSeconds, g_rockConfig.rockSelectionLockReleaseHapticIntensity);
            }
            return;
        case GrabEventType::PullStarted:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                triggerHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockPullStartHapticIntensity);
            }
            return;
        case GrabEventType::PullCatchSucceeded:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                triggerHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockPullCatchHapticIntensity);
            }
            return;
        case GrabEventType::StashCandidate:
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                const float confidence =
                    (eventData.flags & ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID) != 0 ? eventData.intensityHint : 1.0f;
                triggerHaptic(eventData.isLeft,
                    g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
                    shoulder_stash_haptic_policy::computeCandidatePulseIntensity(confidence,
                        shoulder_stash_haptic_policy::CandidatePulseConfig{
                            .enabled = true,
                            .baseIntensity = g_rockConfig.rockShoulderStashCandidateHapticBaseIntensity,
                            .maxIntensity = g_rockConfig.rockShoulderStashCandidateHapticIntensity,
                        }));
            }
            return;
        case GrabEventType::Stashed:
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                triggerHaptic(eventData.isLeft,
                    g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.rockShoulderStashCommitHapticIntensity);
            }
            return;
        case GrabEventType::GrabCommitted:
            triggerHaptic(eventData.isLeft,
                g_rockConfig.rockGrabHapticDurationSeconds,
                grab_haptic_policy::computeMassPulseIntensity(eventData.mass,
                    grab_haptic_policy::MassPulseConfig{
                        .enabled = g_rockConfig.rockGrabHapticsEnabled,
                        .baseIntensity = g_rockConfig.rockGrabHapticBaseIntensity,
                        .maxIntensity = g_rockConfig.rockGrabHapticMaxIntensity,
                        .massScale = g_rockConfig.rockGrabHapticMassScale,
                        .massExponent = g_rockConfig.rockGrabHapticMassExponent,
                    }));
            return;
        case GrabEventType::HeldImpact: {
            pruneHeldImpactHapticCooldowns();
            const std::uint64_t cooldownKey =
                (static_cast<std::uint64_t>(eventData.isLeft ? 1u : 0u) << 63) |
                (static_cast<std::uint64_t>(eventData.primaryBodyId) << 32) |
                static_cast<std::uint64_t>(eventData.secondaryBodyId);
            if (const auto it = _heldImpactHapticCooldownUntil.find(cooldownKey);
                it != _heldImpactHapticCooldownUntil.end() && it->second > _dynamicPushElapsedSeconds) {
                return;
            }

            const bool damped = (eventData.flags & ROCK_GRAB_EVENT_FLAG_HELD_IMPACT_DAMPED) != 0;
            const float intensity = grab_haptic_policy::computeImpactPulseIntensity(eventData.mass,
                eventData.speedGameUnitsPerSecond,
                damped,
                grab_haptic_policy::ImpactPulseConfig{
                    .enabled = g_rockConfig.rockHeldImpactHapticsEnabled,
                    .baseIntensity = g_rockConfig.rockHeldImpactHapticBaseIntensity,
                    .maxIntensity = g_rockConfig.rockHeldImpactHapticMaxIntensity,
                    .speedScale = g_rockConfig.rockHeldImpactHapticSpeedScale,
                    .massScale = g_rockConfig.rockHeldImpactHapticMassScale,
                    .massExponent = g_rockConfig.rockHeldImpactHapticMassExponent,
                    .minSpeedGameUnitsPerSecond = g_rockConfig.rockHeldImpactHapticMinSpeedGameUnits,
                    .dampedMultiplier = g_rockConfig.rockHeldImpactHapticDampedMultiplier,
                });
            if (intensity <= 0.0f) {
                return;
            }

            _heldImpactHapticCooldownUntil[cooldownKey] =
                _dynamicPushElapsedSeconds + (std::max)(0.0f, g_rockConfig.rockHeldImpactHapticCooldownSeconds);
            triggerHaptic(eventData.isLeft, g_rockConfig.rockHeldImpactHapticDurationSeconds, intensity);
            return;
        }
        default:
            return;
        }
    }

    void PhysicsInteraction::dispatchGrabEvent(GrabEventData eventData)
    {
        eventData.size = sizeof(GrabEventData);
        eventData.version = ROCK_GRAB_EVENT_VERSION;
        if (eventData.refr && eventData.formID == 0) {
            eventData.formID = eventData.refr->GetFormID();
        }
        eventData.frameIndex = ++_grabEventFrameCounter;

        handleGrabEventHaptics(eventData);

        if (auto* m = ::rock::getROCKMessaging()) {
            m->Dispatch(kPhysMsg_OnGrabEvent, &eventData, sizeof(eventData), nullptr);
        }
    }

    void PhysicsInteraction::dispatchSimpleGrabEvent(
        GrabEventType type,
        bool isLeft,
        RE::TESObjectREFR* refr,
        std::uint32_t primaryBodyId,
        std::uint32_t flags)
    {
        GrabEventData eventData{};
        eventData.type = type;
        switch (type) {
        case GrabEventType::SelectionLocked:
        case GrabEventType::SelectionUnlocked:
            eventData.sourceKind = GrabEventSourceKind::Hand;
            break;
        case GrabEventType::PullStarted:
        case GrabEventType::PullArrived:
        case GrabEventType::PullCatchAttempt:
        case GrabEventType::PullCatchSucceeded:
            eventData.sourceKind = GrabEventSourceKind::PulledObject;
            break;
        default:
            eventData.sourceKind = GrabEventSourceKind::HeldObject;
            break;
        }
        eventData.isLeft = isLeft;
        eventData.refr = refr;
        eventData.formID = refr ? refr->GetFormID() : 0;
        eventData.primaryBodyId = primaryBodyId;
        eventData.flags = flags;
        dispatchGrabEvent(eventData);
    }

    void PhysicsInteraction::dispatchGrabCommittedEvent(bool isLeft, RE::TESObjectREFR* refr, std::uint32_t primaryBodyId, RE::hknpWorld* world)
    {
        GrabEventData eventData{};
        eventData.type = GrabEventType::GrabCommitted;
        eventData.sourceKind = GrabEventSourceKind::HeldObject;
        eventData.isLeft = isLeft;
        eventData.refr = refr;
        eventData.formID = refr ? refr->GetFormID() : 0;
        eventData.primaryBodyId = primaryBodyId;
        eventData.flags |= fillGrabEventBodyKinematics(world, primaryBodyId, eventData);
        dispatchGrabEvent(eventData);
    }

    void PhysicsInteraction::dispatchHeldImpactGrabEvent(
        bool isLeft,
        RE::TESObjectREFR* refr,
        std::uint32_t heldBodyId,
        std::uint32_t otherBodyId,
        float mass,
        float speedGameUnitsPerSecond)
    {
        GrabEventData eventData{};
        eventData.type = GrabEventType::HeldImpact;
        eventData.sourceKind = GrabEventSourceKind::HeldObject;
        eventData.isLeft = isLeft;
        eventData.refr = refr;
        eventData.formID = refr ? refr->GetFormID() : 0;
        eventData.primaryBodyId = heldBodyId;
        eventData.secondaryBodyId = otherBodyId;
        eventData.mass = mass;
        eventData.speedGameUnitsPerSecond = speedGameUnitsPerSecond;
        if (std::isfinite(mass) && mass > 0.0f) {
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_MASS_VALID;
        }
        if (std::isfinite(speedGameUnitsPerSecond) && speedGameUnitsPerSecond > 0.0f) {
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
        }
        if (isLeft ? _leftHand.isHeldBodyColliding() : _rightHand.isHeldBodyColliding()) {
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_HELD_IMPACT_DAMPED;
        }
        dispatchGrabEvent(eventData);
    }

    void PhysicsInteraction::registerCollisionLayer(RE::hknpWorld* world)
    {
        if (!world) {
            ROCK_LOG_ERROR(Config, "registerCollisionLayer: world is null");
            return;
        }

        bool usedFilterFallback = false;
        auto* matrix = havok_runtime::getCollisionFilterMatrix(world, &usedFilterFallback);
        if (!matrix) {
            ROCK_LOG_ERROR(Config, "Both world filter and global singleton are null — cannot configure layer");
            return;
        }
        ROCK_LOG_DEBUG(Config, "Filter source: matrix={:p}, usedFallback={}", static_cast<const void*>(matrix), usedFilterFallback ? "yes" : "no");

        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_HAND, matrix[collision_layer_policy::ROCK_LAYER_HAND]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_WEAPON, matrix[collision_layer_policy::ROCK_LAYER_WEAPON]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_RELOAD, matrix[collision_layer_policy::ROCK_LAYER_RELOAD]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_BODY, matrix[collision_layer_policy::ROCK_LAYER_BODY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::FO4_LAYER_CHARCONTROLLER, matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER]);

        if (!_nativeCharacterControllerLayerPolicyCaptured) {
            _originalNativeCharacterControllerLayerMask = matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER];
            _nativeCharacterControllerLayerPolicyCaptured = true;
        }

        collision_layer_policy::applyRockGeneratedLayerPolicies(
            matrix,
            g_rockConfig.rockHandCollisionStaticWorldEnabled,
            g_rockConfig.rockWeaponCollisionStaticWorldEnabled,
            g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled,
            g_rockConfig.rockWeaponCollisionBlocksProjectiles,
            g_rockConfig.rockWeaponCollisionBlocksSpells);
        collision_layer_policy::applyNativeCharacterControllerObjectSuppressionPolicy(
            matrix,
            g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled,
            _originalNativeCharacterControllerLayerMask);

        _expectedHandLayerMask = collision_layer_policy::buildRockHandExpectedMask(true, g_rockConfig.rockHandCollisionStaticWorldEnabled);
        _expectedWeaponLayerMask =
            collision_layer_policy::buildRockWeaponExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockWeaponCollisionStaticWorldEnabled,
                true);
        _expectedReloadLayerMask =
            collision_layer_policy::buildRockReloadExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockHandCollisionStaticWorldEnabled);
        _expectedBodyLayerMask = collision_layer_policy::buildRockBodyExpectedMask(g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled);
        _expectedNativeCharacterControllerLayerMask =
            collision_layer_policy::nativeCharacterControllerExpectedMask(
                _originalNativeCharacterControllerLayerMask,
                g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled);
        _nativeCharacterControllerLayerPolicyEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
        _collisionLayerRegistered = true;

        const bool nativeControllerObjectPairsMatch =
            collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix, _expectedNativeCharacterControllerLayerMask);
        const char* nativeControllerObjectStatus =
            _nativeCharacterControllerLayerPolicyEnabled ?
                (nativeControllerObjectPairsMatch ? "blocked" : "bad") :
                (nativeControllerObjectPairsMatch ? "restored" : "bad");

        ROCK_LOG_INFO(Config,
            "Registered ROCK collision layers: hand={} mask=0x{:016X}, weapon={} mask=0x{:016X}, reload={} mask=0x{:016X}, body={} mask=0x{:016X}, actorPairs(biped={},deadbip={},bipedNoCC={}), bodyPairs(hand={},weapon={},self={},static={},animstatic={},clutter={},query={},charController={}), handStaticWorld={}, weaponStaticWorld={}, bodyStaticWorld={}, projectiles={}, spells={}, nativeBubbleObjects={}",
            collision_layer_policy::ROCK_LAYER_HAND,
            matrix[collision_layer_policy::ROCK_LAYER_HAND],
            collision_layer_policy::ROCK_LAYER_WEAPON,
            matrix[collision_layer_policy::ROCK_LAYER_WEAPON],
            collision_layer_policy::ROCK_LAYER_RELOAD,
            matrix[collision_layer_policy::ROCK_LAYER_RELOAD],
            collision_layer_policy::ROCK_LAYER_BODY,
            matrix[collision_layer_policy::ROCK_LAYER_BODY],
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_BIPED,
                collision_layer_policy::maskEnablesLayer(_expectedHandLayerMask, collision_layer_policy::FO4_LAYER_BIPED)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_BIPED,
                        collision_layer_policy::maskEnablesLayer(_expectedWeaponLayerMask, collision_layer_policy::FO4_LAYER_BIPED)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_DEADBIP,
                collision_layer_policy::maskEnablesLayer(_expectedHandLayerMask, collision_layer_policy::FO4_LAYER_DEADBIP)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_DEADBIP,
                        collision_layer_policy::maskEnablesLayer(_expectedWeaponLayerMask, collision_layer_policy::FO4_LAYER_DEADBIP)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_BIPED_NO_CC,
                collision_layer_policy::maskEnablesLayer(_expectedHandLayerMask, collision_layer_policy::FO4_LAYER_BIPED_NO_CC)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_BIPED_NO_CC,
                        collision_layer_policy::maskEnablesLayer(_expectedWeaponLayerMask, collision_layer_policy::FO4_LAYER_BIPED_NO_CC)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::ROCK_LAYER_HAND)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_WEAPON,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::ROCK_LAYER_WEAPON)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::ROCK_LAYER_BODY)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_STATIC,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::FO4_LAYER_STATIC)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_ANIMSTATIC,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::FO4_LAYER_ANIMSTATIC)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_CLUTTER,
                collision_layer_policy::maskEnablesLayer(_expectedBodyLayerMask, collision_layer_policy::FO4_LAYER_CLUTTER)) ? "ok" : "bad",
            !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::ROCK_LAYER_BODY, collision_layer_policy::FO4_LAYER_ITEMPICK) &&
                    !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::FO4_LAYER_ITEMPICK, collision_layer_policy::ROCK_LAYER_BODY) ?
                "ok" :
                "bad",
            !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::ROCK_LAYER_BODY, collision_layer_policy::FO4_LAYER_CHARCONTROLLER) &&
                    !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::ROCK_LAYER_BODY) ?
                "ok" :
                "bad",
            g_rockConfig.rockHandCollisionStaticWorldEnabled ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionStaticWorldEnabled ? "enabled" : "disabled",
            g_rockConfig.rockBodyBoneCollisionStaticWorldEnabled ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksProjectiles ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksSpells ? "enabled" : "disabled",
            nativeControllerObjectStatus);
    }

    bool PhysicsInteraction::createHandCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!runtime_state::isLocalSkeletonReady()) {
            ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
            return false;
        }

        const bool rightOk = _rightHand.createCollision(world, bhkWorld);

        const bool leftOk = _leftHand.createCollision(world, bhkWorld);

        if (!rightOk || !leftOk) {
            ROCK_LOG_ERROR(Hand, "Hand collision creation failed (rightOk={}, leftOk={})", rightOk, leftOk);
            if (rightOk)
                _rightHand.destroyCollision(bhkWorld);
            if (leftOk)
                _leftHand.destroyCollision(bhkWorld);
            return false;
        }

        ROCK_LOG_INFO(Hand,
            "Bone-derived hand collision created: rightBodies={} leftBodies={} mode={} requireAnchor={} requireAllFingerBones={}",
            _rightHand.getHandColliderBodyCount(),
            _leftHand.getHandColliderBodyCount(),
            g_rockConfig.rockHandColliderRuntimeMode,
            g_rockConfig.rockHandBoneCollidersRequirePalmAnchor ? "true" : "false",
            g_rockConfig.rockHandBoneCollidersRequireAllFingerBones ? "true" : "false");

        _handColliderCreateRetryFrames = 0;
        return true;
    }

    void PhysicsInteraction::destroyHandCollisions(void* bhkWorld)
    {
        _rightHand.destroyCollision(bhkWorld);
        _leftHand.destroyCollision(bhkWorld);
        _handColliderCreateRetryFrames = 0;
    }

    void PhysicsInteraction::updateHandCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::HandColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* world = frame.hknpWorld;

        if (!_rightHand.hasCollisionBody() || !_leftHand.hasCollisionBody()) {
            if (_handColliderCreateRetryFrames > 0) {
                --_handColliderCreateRetryFrames;
                return;
            }

            ROCK_LOG_WARN(Hand,
                "Hand collider runtime missing generated bodies; recreating rightBody={} leftBody={}",
                _rightHand.hasCollisionBody() ? "yes" : "no",
                _leftHand.hasCollisionBody() ? "yes" : "no");
            destroyHandCollisions(frame.bhkWorld);
            if (!createHandCollisions(frame.hknpWorld, frame.bhkWorld)) {
                _handColliderCreateRetryFrames = 120;
            }
            return;
        }

        _rightHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);
        _leftHand.updateDelayedGrabHandCollisionRestore(world, frame.deltaSeconds);

        if (!frame.right.disabled) {
            _rightHand.updateCollisionTransform(world, frame.deltaSeconds);
        }
        if (!frame.left.disabled) {
            _leftHand.updateCollisionTransform(world, frame.deltaSeconds);
        }
    }

    bool PhysicsInteraction::createBodyBoneCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!g_rockConfig.rockBodyBoneCollidersEnabled) {
            _bodyBoneColliders.destroy(bhkWorld);
            return true;
        }

        if (!runtime_state::isLocalSkeletonReady()) {
            ROCK_LOG_WARN(Body, "Cannot create body bone colliders: skeleton not ready");
            return false;
        }

        if (!_bodyBoneColliders.create(world, bhkWorld)) {
            return false;
        }

        _bodyBoneColliderCreateRetryFrames = 0;
        _bodyContactRuntime.reset();
        ROCK_LOG_INFO(Body,
            "Full body bone collider set created: bodies={} enabled=true",
            _bodyBoneColliders.getBodyCount());
        return true;
    }

    void PhysicsInteraction::destroyBodyBoneCollisions(void* bhkWorld)
    {
        _bodyBoneColliders.destroy(bhkWorld);
        _bodyContactRuntime.reset();
        _bodyBoneColliderCreateRetryFrames = 0;
    }

    void PhysicsInteraction::updateBodyBoneCollisions(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::BodyColliderUpdate);

        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        if (!g_rockConfig.rockBodyBoneCollidersEnabled) {
            if (_bodyBoneColliders.hasBodies()) {
                ROCK_LOG_INFO(Body, "Body bone collider config disabled; destroying generated body set");
                destroyBodyBoneCollisions(frame.bhkWorld);
                _bodyContactRuntime.reset();
            }
            return;
        }

        if (!_bodyBoneColliders.hasBodies()) {
            if (_bodyBoneColliderCreateRetryFrames > 0) {
                --_bodyBoneColliderCreateRetryFrames;
                return;
            }

            if (!createBodyBoneCollisions(frame.hknpWorld, frame.bhkWorld)) {
                _bodyBoneColliderCreateRetryFrames = 120;
            }
            return;
        }

        _bodyBoneColliders.update(frame.hknpWorld, frame.deltaSeconds);
    }

    bool PhysicsInteraction::shouldSuppressNativePlayerCollisionBody(RE::bhkWorld* bhk, RE::hknpWorld* hknp, std::uint32_t bodyId) const
    {
        if (!bhk || !hknp || !contact_pipeline_policy::isValidBodyId(bodyId)) {
            return false;
        }

        if (bodyId == _rightHand.getCollisionBodyId().value ||
            bodyId == _leftHand.getCollisionBodyId().value ||
            _rightHand.isHandColliderBodyId(bodyId) ||
            _leftHand.isHandColliderBodyId(bodyId) ||
            _rightHand.isHeldBodyId(bodyId) ||
            _leftHand.isHeldBodyId(bodyId) ||
            _weaponCollision.isWeaponBodyIdAtomic(bodyId) ||
            _bodyBoneColliders.isColliderBodyIdAtomic(bodyId) ||
            ::rock::provider::isExternalBodyId(bodyId)) {
            return false;
        }

        std::uint32_t filterInfo = 0;
        if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ bodyId }, filterInfo)) {
            return false;
        }

        const std::uint32_t layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
        if (!collision_layer_policy::isNativePlayerCollisionSuppressionLayer(layer)) {
            return false;
        }

        auto* resolvedRef = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ bodyId });
        auto* player = RE::PlayerCharacter::GetSingleton();
        return !resolvedRef || resolvedRef == player;
    }

    void PhysicsInteraction::restoreNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* reason)
    {
        if (_nativePlayerCollisionSuppressedBodyCount == 0) {
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            return;
        }

        std::array<std::uint32_t, kNativePlayerCollisionSuppressionBodyCapacity> pending{};
        std::uint32_t pendingCount = 0;

        auto keepPending = [&](std::uint32_t bodyId) {
            if (pendingCount < pending.size()) {
                pending[pendingCount++] = bodyId;
            }
        };

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodyIds.size(); ++i) {
            const auto bodyId = _nativePlayerCollisionSuppressedBodyIds[i];
            if (!contact_pipeline_policy::isValidBodyId(bodyId)) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                hknp,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                reason ? reason : "native-player-body");
            if (releaseResult.readFailed) {
                keepPending(bodyId);
            }
        }

        _nativePlayerCollisionSuppressedBodyIds = pending;
        _nativePlayerCollisionSuppressedBodyCount = pendingCount;
        _nativePlayerCollisionSuppressionRefreshFrames = pendingCount == 0 ? 0 : 30;
    }

    void PhysicsInteraction::refreshNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* context)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled || !hknp || _nativePlayerCollisionSuppressedBodyCount == 0) {
            return;
        }

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodyIds.size(); ++i) {
            const auto bodyId = _nativePlayerCollisionSuppressedBodyIds[i];
            if (!contact_pipeline_policy::isValidBodyId(bodyId)) {
                continue;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ bodyId }, currentFilter)) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "Native player collision suppression refresh skipped: bodyId={} context={} cannot read filter",
                    bodyId,
                    context ? context : "");
                continue;
            }

            const std::uint32_t refreshedFilter = currentFilter | collision_suppression_registry::kSuppressionNoCollideBit;
            if (refreshedFilter != currentFilter) {
                body_collision::setFilterInfo(hknp, RE::hknpBodyId{ bodyId }, refreshedFilter);
            }
        }
    }

    void PhysicsInteraction::updateNativePlayerCollisionSuppression(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled) {
            restoreNativePlayerCollisionSuppression(hknp, "native-player-filter-disabled");
            return;
        }

        if (!bhk || !hknp) {
            return;
        }

        refreshNativePlayerCollisionSuppression(hknp, "native-player-body-frame-refresh");

        if (_nativePlayerCollisionSuppressionRefreshFrames > 0) {
            --_nativePlayerCollisionSuppressionRefreshFrames;
            return;
        }
        _nativePlayerCollisionSuppressionRefreshFrames = 30;

        struct NativePlayerBodyScanContext
        {
            PhysicsInteraction* self = nullptr;
            RE::bhkWorld* bhk = nullptr;
            RE::hknpWorld* hknp = nullptr;
            std::array<std::uint32_t, PhysicsInteraction::kNativePlayerCollisionSuppressionBodyCapacity> bodyIds{};
            std::uint32_t bodyCount = 0;
            bool overflow = false;

            bool contains(std::uint32_t bodyId) const
            {
                for (std::uint32_t i = 0; i < bodyCount && i < bodyIds.size(); ++i) {
                    if (bodyIds[i] == bodyId) {
                        return true;
                    }
                }
                return false;
            }

            void append(std::uint32_t bodyId)
            {
                if (!self || !self->shouldSuppressNativePlayerCollisionBody(bhk, hknp, bodyId) || contains(bodyId)) {
                    return;
                }
                if (bodyCount >= bodyIds.size()) {
                    overflow = true;
                    return;
                }
                bodyIds[bodyCount++] = bodyId;
            }
        } scanContext{ this, bhk, hknp };

        auto visitBody = [](std::uint32_t bodyId, void* userData) {
            auto* context = static_cast<NativePlayerBodyScanContext*>(userData);
            if (!context) {
                return false;
            }
            context->append(bodyId);
            return true;
        };

        auto scanCollisionObject = [&](RE::NiCollisionObject* collisionObject) {
            havok_runtime::forEachPhysicsSystemBodyIdDetailed(collisionObject, hknp, 256, visitBody, &scanContext);
        };

        auto scanNode = [&](auto&& self, RE::NiAVObject* node, int depth) -> void {
            if (!node || depth <= 0) {
                return;
            }

            scanCollisionObject(node->collisionObject.get());
            if (auto* niNode = node->IsNode()) {
                auto& children = niNode->GetRuntimeData().children;
                for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                    if (auto* child = children[i].get()) {
                        self(self, child, depth - 1);
                    }
                }
            }
        };

        if (auto* player = RE::PlayerCharacter::GetSingleton()) {
            if (player->currentProcess && player->currentProcess->middleHigh && player->currentProcess->middleHigh->poseBound) {
                scanCollisionObject(player->currentProcess->middleHigh->poseBound.get());
            }
        }
        scanNode(scanNode, f4cf::f4vr::getFirstPersonSkeleton(), 64);
        if (auto* player = f4cf::f4vr::getPlayer(); player && player->unkF0) {
            scanNode(scanNode, player->unkF0->rootNode, 64);
        }

        if (scanContext.overflow && !_nativePlayerCollisionSuppressionOverflowLogged) {
            _nativePlayerCollisionSuppressionOverflowLogged = true;
            ROCK_LOG_WARN(Hand,
                "Native player collision suppression body capacity exceeded; keeping first {} bodies",
                kNativePlayerCollisionSuppressionBodyCapacity);
        } else if (!scanContext.overflow) {
            _nativePlayerCollisionSuppressionOverflowLogged = false;
        }

        std::array<std::uint32_t, kNativePlayerCollisionSuppressionBodyCapacity> next{};
        std::uint32_t nextCount = 0;
        auto nextContains = [&](std::uint32_t bodyId) {
            for (std::uint32_t i = 0; i < nextCount && i < next.size(); ++i) {
                if (next[i] == bodyId) {
                    return true;
                }
            }
            return false;
        };
        auto appendNext = [&](std::uint32_t bodyId) {
            if (!contact_pipeline_policy::isValidBodyId(bodyId) || nextContains(bodyId) || nextCount >= next.size()) {
                return;
            }
            next[nextCount++] = bodyId;
        };

        for (std::uint32_t i = 0; i < scanContext.bodyCount && i < scanContext.bodyIds.size(); ++i) {
            const auto bodyId = scanContext.bodyIds[i];
            const auto acquireResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                hknp,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                "native-player-body");
            if (acquireResult.valid) {
                appendNext(bodyId);
            }
        }

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodyIds.size(); ++i) {
            const auto bodyId = _nativePlayerCollisionSuppressedBodyIds[i];
            if (scanContext.contains(bodyId)) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                hknp,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                "native-player-body-stale");
            if (releaseResult.readFailed) {
                appendNext(bodyId);
            }
        }

        _nativePlayerCollisionSuppressedBodyIds = next;
        _nativePlayerCollisionSuppressedBodyCount = nextCount;
    }

    void PhysicsInteraction::onGeneratedColliderPhysicsSubstep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        /*
         * This pre-collide callback runs inside the same native Havok step path
         * used for generated body writes. It only reasserts bit 14 on cached
         * native player bodies; registry ownership is not changed from here.
         */
        self->refreshNativePlayerCollisionSuppression(world, "native-player-body-pre-collide");
        self->driveGeneratedCollidersFromPhysicsSubstep(world, timing);
    }

    void PhysicsInteraction::onCustomGrabAuthorityBetweenStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->driveCustomGrabAuthorityFromBetweenStep(world, timing);
    }

    void PhysicsInteraction::onCustomGrabAuthorityAfterSolve(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->observeCustomGrabAuthorityAfterSolve(world, timing);
    }

    void PhysicsInteraction::driveGeneratedCollidersFromPhysicsSubstep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GeneratedColliderPhysicsFlush);

        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        _rightHand.flushPendingCollisionPhysicsDrive(world, timing);
        _leftHand.flushPendingCollisionPhysicsDrive(world, timing);
        _bodyBoneColliders.flushPendingPhysicsDrive(world, timing);
        _weaponCollision.flushPendingPhysicsDrive(world, timing);
    }

    void PhysicsInteraction::driveCustomGrabAuthorityFromBetweenStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        _rightHand.flushPendingCustomGrabAuthority(world, timing);
        _leftHand.flushPendingCustomGrabAuthority(world, timing);
    }

    void PhysicsInteraction::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        _rightHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _leftHand.observeCustomGrabAuthorityAfterSolve(world, timing);
    }

#include "physics-interaction/core/PhysicsInteractionDebugOverlay.inl"
    void PhysicsInteraction::updateSelection(const PhysicsFrameContext& frame)
    {
        if (!runtime_state::isLocalSkeletonReady())
            return;

        auto selectionContextForOtherHand = [](const Hand& hand) {
            OtherHandSelectionContext context{};
            if (hand.isHolding() && hand.getHeldRef()) {
                context.shareableHeldRef = hand.getHeldRef();
                return context;
            }
            if (hand.hasActivePullCatchIntent()) {
                context.exclusiveRef = hand.getPullCatchIntentRef();
                return context;
            }
            if (hand.hasSelection() && selection_state_policy::hasExclusiveObjectSelection(hand.getState())) {
                context.exclusiveRef = hand.getSelection().refr;
            }
            return context;
        };

        const auto rightHandContext = selectionContextForOtherHand(_rightHand);
        const auto leftHandContext = selectionContextForOtherHand(_leftHand);
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);

        if (!frame.right.disabled) {
            _rightHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.right.grabAnchorWorld,
                frame.right.palmNormalWorld,
                frame.right.pointingWorld,
                frame.right.pinchPocketWorld,
                frame.right.pinchDirectionWorld,
                frame.right.hasPinchPocketWorld,
                farHmdConeGate,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                leftHandContext);
        }

        if (!frame.left.disabled) {
            _leftHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.left.grabAnchorWorld,
                frame.left.palmNormalWorld,
                frame.left.pointingWorld,
                frame.left.pinchPocketWorld,
                frame.left.pinchDirectionWorld,
                frame.left.hasPinchPocketWorld,
                farHmdConeGate,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                rightHandContext);
        }
    }

    GrabReleaseContext PhysicsInteraction::makeGrabReleaseContext(const Hand& hand, bool isLeft) const
    {
        const Hand& peer = isLeft ? _rightHand : _leftHand;
        auto* heldRef = hand.getHeldRef();
        const bool peerStillHoldingSameObject = heldRef && peer.isHolding() && peer.getHeldRef() == heldRef;
        return GrabReleaseContext{
            .finalObjectRelease = !peerStillHoldingSameObject,
            .peerHandStillHolding = peerStillHoldingSameObject,
            .reason = peerStillHoldingSameObject ? "peer-hand-still-holding-object" : "last-hand-release",
        };
    }

    GrabSharedObjectContext PhysicsInteraction::makeGrabSharedObjectContext(const Hand& hand, bool isLeft) const
    {
        const Hand& peer = isLeft ? _rightHand : _leftHand;
        auto* selectedRef = hand.hasSelection() ? hand.getSelection().refr : nullptr;
        if (!selectedRef || !peer.isHolding() || peer.getHeldRef() != selectedRef) {
            return {};
        }

        return GrabSharedObjectContext{
            .joiningPeerHeldObject = true,
            .peerSavedObjectState = &peer.getSavedObjectState(),
            .peerActiveGrabLifecycle = &peer.getActiveGrabLifecycle(),
            .peerHeldBodyIds = &peer.getHeldBodyIds(),
        };
    }

    HeldObjectPlayerSpaceFrame PhysicsInteraction::sampleHeldObjectPlayerSpaceFrame(float deltaSeconds)
    {
        HeldObjectPlayerSpaceFrame frame{};
        const auto& playerSpace = runtime_state::currentFrame().playerSpace;
        if (!playerSpace.valid) {
            _hasHeldPlayerSpacePosition = false;
            _hasHeldPlayerSpaceTransform = false;
            return frame;
        }

        const RE::NiPoint3 smoothPos = playerSpace.world.translate;
        const RE::NiTransform playerSpaceWorld = playerSpace.world;
        const char* playerSpaceSource = playerSpace.source;

        if (!g_rockConfig.rockGrabPlayerSpaceCompensation) {
            _prevHeldPlayerSpacePosition = smoothPos;
            _prevHeldPlayerSpaceTransform = playerSpaceWorld;
            _hasHeldPlayerSpacePosition = true;
            _hasHeldPlayerSpaceTransform = true;
            return frame;
        }

        frame.enabled = true;
        frame.currentPlayerSpaceWorld = playerSpaceWorld;
        frame.source = playerSpaceSource;
        if (_hasHeldPlayerSpacePosition) {
            frame.deltaGameUnits = smoothPos - _prevHeldPlayerSpacePosition;
            frame.velocityHavok = held_object_physics_math::gameUnitsDeltaToHavokVelocity(frame.deltaGameUnits, deltaSeconds, physics_scale::havokToGame());
            frame.warpByDistance = held_object_physics_math::shouldWarpPlayerSpaceDelta(frame.deltaGameUnits, g_rockConfig.rockGrabPlayerSpaceWarpDistance);
        }
        if (_hasHeldPlayerSpaceTransform) {
            frame.previousPlayerSpaceWorld = _prevHeldPlayerSpaceTransform;
            frame.hasWarpTransforms = true;
            frame.rotationDeltaDegrees =
                held_player_space_math::rotationDeltaDegrees(_prevHeldPlayerSpaceTransform.rotate, playerSpaceWorld.rotate);
            frame.warpByRotation = held_player_space_math::shouldWarpPlayerSpaceRotation(
                _prevHeldPlayerSpaceTransform.rotate,
                playerSpaceWorld.rotate,
                g_rockConfig.rockGrabPlayerSpaceWarpMinRotationDegrees);
        }
        frame.warp = frame.warpByDistance || frame.warpByRotation;

        _prevHeldPlayerSpacePosition = smoothPos;
        _prevHeldPlayerSpaceTransform = playerSpaceWorld;
        _hasHeldPlayerSpacePosition = true;
        _hasHeldPlayerSpaceTransform = true;

        if (g_rockConfig.rockDebugGrabFrameLogging && (_rightHand.isHolding() || _leftHand.isHolding())) {
            ++_heldPlayerSpaceLogCounter;
            if (_heldPlayerSpaceLogCounter >= 45 || frame.warp) {
                _heldPlayerSpaceLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Held player-space: source={} enabled={} beforeHeld=yes warp={} distWarp={} rotWarp={} rotDelta={:.2f}deg "
                    "delta=({:.2f},{:.2f},{:.2f}) velHk=({:.3f},{:.3f},{:.3f})",
                    frame.source, frame.enabled ? "yes" : "no", frame.warp ? "yes" : "no", frame.warpByDistance ? "yes" : "no",
                    frame.warpByRotation ? "yes" : "no", frame.rotationDeltaDegrees, frame.deltaGameUnits.x, frame.deltaGameUnits.y, frame.deltaGameUnits.z,
                    frame.velocityHavok.x, frame.velocityHavok.y, frame.velocityHavok.z);
            }
        }

        return frame;
    }

    void PhysicsInteraction::applyHeldPlayerSpaceVelocity(RE::hknpWorld* hknp)
    {
        /*
         * Held object motion has one velocity authority while the grab is active:
         * the grab constraint targets plus this single player-space compensation
         * pass. Per-hand held loops only sample local velocity for throw history,
         * avoiding two hands or connected bodies writing the same Havok motion
         * more than once.
         */
        if (!hknp) {
            _lastCentralHeldPlayerSpaceVelocityHavok = {};
            return;
        }

        std::vector<std::uint32_t> bodyIds;
        bodyIds.reserve(_rightHand.getHeldBodyIds().size() + _leftHand.getHeldBodyIds().size() + 2);
        auto appendHandBodies = [&](const Hand& hand) {
            if (!hand.isHolding()) {
                return;
            }

            const auto& savedState = hand.getSavedObjectState();
            if (savedState.bodyId.value != INVALID_BODY_ID) {
                bodyIds.push_back(savedState.bodyId.value);
            }
            for (const auto bodyId : hand.getHeldBodyIds()) {
                if (bodyId != INVALID_BODY_ID) {
                    bodyIds.push_back(bodyId);
                }
            }
        };

        appendHandBodies(_rightHand);
        appendHandBodies(_leftHand);

        const float keep = g_rockConfig.rockGrabResidualVelocityDamping ?
                               held_object_damping_math::velocityKeepFactor(g_rockConfig.rockGrabVelocityDamping) :
                               1.0f;
        const bool runtimeTransformWarp = held_player_space_math::shouldApplyRuntimeTransformWarp(
            g_rockConfig.rockGrabPlayerSpaceTransformWarpEnabled,
            _heldObjectPlayerSpaceFrame.warp,
            _heldObjectPlayerSpaceFrame.hasWarpTransforms);

        const auto result = held_player_space_registry::applyCentralPlayerSpaceVelocity(
            hknp,
            bodyIds,
            _heldObjectPlayerSpaceFrame.velocityHavok,
            _lastCentralHeldPlayerSpaceVelocityHavok,
            keep,
            _heldObjectPlayerSpaceFrame.enabled,
            runtimeTransformWarp,
            runtimeTransformWarp ? &_heldObjectPlayerSpaceFrame.previousPlayerSpaceWorld : nullptr,
            runtimeTransformWarp ? &_heldObjectPlayerSpaceFrame.currentPlayerSpaceWorld : nullptr);

        if (held_player_space_registry::shouldCarryPreviousPlayerVelocity(
                _heldObjectPlayerSpaceFrame.enabled,
                runtimeTransformWarp,
                result.motionsWritten)) {
            _lastCentralHeldPlayerSpaceVelocityHavok = _heldObjectPlayerSpaceFrame.velocityHavok;
        } else {
            _lastCentralHeldPlayerSpaceVelocityHavok = {};
        }

        if (g_rockConfig.rockDebugGrabFrameLogging && !bodyIds.empty()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "Held player-space central writer: beforeHeld=yes diagWarp={} runtimeWarp={} distWarp={} rotWarp={} bodies={} registered={} motionsWritten={} transformsWarped={} duplicateMotions={} writerMask=0x{:02X}",
                _heldObjectPlayerSpaceFrame.warp ? "yes" : "no",
                runtimeTransformWarp ? "yes" : "no",
                _heldObjectPlayerSpaceFrame.warpByDistance ? "yes" : "no",
                _heldObjectPlayerSpaceFrame.warpByRotation ? "yes" : "no",
                bodyIds.size(),
                result.registeredBodies,
                result.motionsWritten,
                result.transformsWarped,
                result.duplicateMotionSkips,
                result.writerMask);
        }
    }

    void PhysicsInteraction::restoreHeldMassMovementSlowdown(const char* reason)
    {
        if (_heldMassMovementSpeedReduction <= 0.0f) {
            return;
        }

        const float previousReduction = _heldMassMovementSpeedReduction;
        if (applyPlayerSpeedReduction(previousReduction, 0.0f)) {
            _heldMassMovementSpeedReduction = 0.0f;
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
            _heldMassMovementLogCounter = 0;
            ROCK_LOG_DEBUG(Hand,
                "Held mass movement slowdown restored: previousReduction={:.2f} reason={}",
                previousReduction,
                reason ? reason : "restore");
        } else {
            ROCK_LOG_SAMPLE_WARN(Hand,
                300,
                "Held mass movement slowdown restore delayed: previousReduction={:.2f} reason={}",
                previousReduction,
                reason ? reason : "restore");
        }
    }

    void PhysicsInteraction::updateHeldMassMovementSlowdown(RE::hknpWorld* hknp, float deltaSeconds)
    {
        if (!g_rockConfig.rockGrabHeldMassMovementSlowdownEnabled) {
            restoreHeldMassMovementSlowdown("disabled");
            return;
        }

        float heldMass = 0.0f;
        if (hknp) {
            constexpr std::size_t kMaxMovementMassMotionSlots = 160;
            std::array<std::uint32_t, kMaxMovementMassMotionSlots> sampledMotionSlots{};
            std::size_t sampledMotionSlotCount = 0;

            auto motionAlreadySampled = [&](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                    if (sampledMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto sampleBody = [&](std::uint32_t bodyId) {
                if (isInvalidGrabBodyId(bodyId)) {
                    return;
                }

                auto* body = havok_runtime::getBody(hknp, RE::hknpBodyId{ bodyId });
                if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex) || motionAlreadySampled(body->motionIndex)) {
                    return;
                }
                if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                    return;
                }

                const float mass = readGrabEventBodyMass(hknp, bodyId);
                if (!std::isfinite(mass) || mass <= 0.0f) {
                    return;
                }

                sampledMotionSlots[sampledMotionSlotCount++] = body->motionIndex;
                heldMass += mass;
            };

            auto sampleHand = [&](const Hand& hand) {
                if (!hand.isHolding()) {
                    return;
                }

                const auto& savedState = hand.getSavedObjectState();
                sampleBody(savedState.bodyId.value);
                for (const auto bodyId : hand.getHeldBodyIds()) {
                    sampleBody(bodyId);
                }
            };

            sampleHand(_rightHand);
            sampleHand(_leftHand);
        }

        const held_mass_movement::Config movementConfig{
            .enabled = g_rockConfig.rockGrabHeldMassMovementSlowdownEnabled,
            .massProportion = g_rockConfig.rockGrabHeldMassMovementMassProportion,
            .massExponent = g_rockConfig.rockGrabHeldMassMovementMassExponent,
            .maxReduction = g_rockConfig.rockGrabHeldMassMovementMaxReduction,
            .fadeOutSeconds = g_rockConfig.rockGrabHeldMassMovementFadeOutSeconds,
        };
        const float heldMassReduction = held_mass_movement::computeHeldMassReduction(heldMass, movementConfig);
        float targetReduction = heldMassReduction;
        if (heldMassReduction > 0.0f) {
            _heldMassMovementFadeStartReduction = heldMassReduction;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        } else if (_heldMassMovementSpeedReduction > 0.0f) {
            if (_heldMassMovementFadeStartReduction <= 0.0f) {
                _heldMassMovementFadeStartReduction = _heldMassMovementSpeedReduction;
                _heldMassMovementFadeElapsedSeconds = 0.0f;
            }
            _heldMassMovementFadeElapsedSeconds += std::isfinite(deltaSeconds) ? (std::max)(0.0f, deltaSeconds) : 0.0f;
            targetReduction = held_mass_movement::computeFadeOutReduction(
                _heldMassMovementFadeStartReduction,
                _heldMassMovementFadeElapsedSeconds,
                movementConfig.fadeOutSeconds);
        } else {
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        }

        if (std::fabs(targetReduction - _heldMassMovementSpeedReduction) <= 0.001f &&
            (targetReduction > 0.0f || _heldMassMovementSpeedReduction <= 0.0f)) {
            return;
        }

        const float previousReduction = _heldMassMovementSpeedReduction;
        if (!applyPlayerSpeedReduction(previousReduction, targetReduction)) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                300,
                "Held mass movement slowdown skipped: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                heldMass,
                previousReduction,
                targetReduction);
            return;
        }

        _heldMassMovementSpeedReduction = targetReduction;
        if (targetReduction <= 0.0f) {
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        }
        if (g_rockConfig.rockDebugGrabFrameLogging) {
            ++_heldMassMovementLogCounter;
            if (_heldMassMovementLogCounter >= 90 || heldMass <= 0.0f || previousReduction <= 0.0f) {
                _heldMassMovementLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Held mass movement slowdown: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                    heldMass,
                    previousReduction,
                    targetReduction);
            }
        }
    }

    void PhysicsInteraction::updateGrabInput(const PhysicsFrameContext& frame)
    {
        auto clearShoulderStashForHand = [&](Hand& hand, bool isLeft) {
            shoulder_stash::resetRuntime(_shoulderStashStates[isLeft ? 1u : 0u]);
            hand.cancelStashCandidate();
        };

        if (!runtime_state::isLocalSkeletonReady()) {
            clearShoulderStashForHand(_rightHand, false);
            clearShoulderStashForHand(_leftHand, true);
            return;
        }

        auto* hknp = frame.hknpWorld;
        int grabButton = g_rockConfig.rockGrabButtonID;
        const bool rightHandWeaponEquipped = resolveEquippedWeaponInteractionNode() != nullptr;
        const bool equippedWeaponSupportGripActive = _twoHandedGrip.isGripping();
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);

        auto releaseSuppressedHeldObject = [&](Hand& hand, bool isLeft, const char* reason) {
            auto* heldRef = hand.getHeldRef();
            auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
            hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(hand, isLeft));
            if (heldRef) {
                releaseObject(heldRef, claimOwnerForHand(isLeft));
            }
            ROCK_LOG_DEBUG(Hand, "{} hand: released held object because normal grab input is suppressed ({})", hand.handName(), reason ? reason : "unknown");
            dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
            dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
        };

        auto processHand = [&](Hand& hand, bool isLeft) {
            const auto& handInput = isLeft ? frame.left : frame.right;
            auto& inputIntentState = _grabInputIntentStates[isLeft ? 1u : 0u];
            auto& shoulderStashState = _shoulderStashStates[isLeft ? 1u : 0u];
            if (handInput.disabled) {
                clearShoulderStashForHand(hand, isLeft);
                return;
            }
            if (!weapon_two_handed_grip_math::canProcessNormalGrabInput(isLeft, equippedWeaponSupportGripActive, rightHandWeaponEquipped)) {
                grab_input_intent_policy::reset(inputIntentState);
                clearShoulderStashForHand(hand, isLeft);
                _softContactRuntime.clearHandForStrongerOwner(
                    isLeft,
                    isLeft ? "equipped-weapon-support-grip" : "right-hand-equipped-weapon");
                if (hand.isHolding()) {
                    releaseSuppressedHeldObject(hand, isLeft, isLeft ? "equipped weapon support grip active" : "right-hand weapon equipped");
                } else if (hand.hasActivePullCatchIntent()) {
                    auto* pullCatchRef = hand.getPullCatchIntentRef();
                    hand.clearSelectionState(true);
                    releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                    ROCK_LOG_DEBUG(Hand, "{} hand: cleared pull catch because normal grab input is suppressed", hand.handName());
                } else if (hand.hasPendingActorEquipmentDropHandoff()) {
                    hand.clearSelectionState(true);
                    ROCK_LOG_DEBUG(Hand, "{} hand: cleared actor-equipment drop handoff because normal grab input is suppressed", hand.handName());
                } else if (hand.getState() == HandState::Pulled || hand.getState() == HandState::SelectionLocked) {
                    auto* selectedRef = hand.getSelection().refr;
                    if (hand.getState() == HandState::SelectionLocked) {
                        dispatchSimpleGrabEvent(GrabEventType::SelectionUnlocked, isLeft, selectedRef, hand.getSelection().bodyId.value);
                    }
                    hand.clearSelectionState(true);
                    releaseObject(selectedRef, claimOwnerForHand(isLeft));
                    ROCK_LOG_DEBUG(Hand, "{} hand: cleared pull/locked selection because normal grab input is suppressed", hand.handName());
                }
                return;
            }

            auto grabInput = readGrabButtonState(isLeft, grabButton);
            if (grabInput.pressed &&
                selection_state_policy::canProcessSelectedState(hand.getState()) &&
                hand.hasSelection() &&
                hand.getSelection().isFarSelection &&
                !hand.hasPendingActorEquipmentDropHandoff() &&
                !hand.hasPendingPullCatchCommit()) {
                float hmdConeDot = -1.0f;
                if (!selectedObjectPassesFarHmdCone(hknp, hand.getSelection(), farHmdConeGate, &hmdConeDot)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand: far grab press ignored outside HMD cone formID={:08X} hmdDot={:.3f} minDot={:.3f}",
                        hand.handName(),
                        hand.getSelection().refr ? hand.getSelection().refr->GetFormID() : 0,
                        hmdConeDot,
                        farHmdConeGate.minDot);
                    hand.clearSelectionState(true);
                    grab_input_intent_policy::reset(inputIntentState);
                    return;
                }
            }

            if (hand.hasArrivedPullCatchIntent() && !hand.hasPendingPullCatchCommit()) {
                auto* pullCatchRef = hand.getPullCatchIntentRef();
                if (g_rockConfig.rockPullCatchWideReacquireEnabled &&
                    hand.reacquirePullCatchCloseSelection(frame.bhkWorld,
                        frame.hknpWorld,
                        handInput.grabAnchorWorld,
                        handInput.palmNormalWorld,
                        g_rockConfig.rockPullCatchWideReacquireRadiusGameUnits,
                        g_rockConfig.rockPullCatchWideReacquireMaxBodyDistanceGameUnits)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand restored stale pull catch commit with target-specific wide close reacquire",
                        hand.handName());
                } else {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand cancelled stale pull catch commit because selected close ref/body no longer matches the pull owner",
                        hand.handName());
                    hand.clearSelectionState(true);
                    releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                    return;
                }
            }

            const Hand& peerForInputIntent = isLeft ? _rightHand : _leftHand;
            const bool peerHeldCloseCandidate =
                !hand.isHolding() &&
                peerForInputIntent.isHolding() &&
                peerForInputIntent.getHeldRef() &&
                (!hand.hasSelection() || hand.getSelection().refr == peerForInputIntent.getHeldRef());
            const bool selectedPressCandidate =
                !hand.isHolding() &&
                hand.hasSelection() &&
                selection_state_policy::canProcessSelectedState(hand.getState());
            const bool pullCatchPressCandidate = !hand.isHolding() && hand.hasPendingPullCatchCommit();
            const auto intentDecision = grab_input_intent_policy::update(
                inputIntentState,
                grab_input_intent_policy::RawButtonState{
                    .held = grabInput.held,
                    .pressed = grabInput.pressed,
                    .released = grabInput.released,
                },
                selectedPressCandidate || pullCatchPressCandidate || peerHeldCloseCandidate,
                hand.isHolding(),
                frame.deltaSeconds,
                grab_input_intent_policy::Config{
                    .enabled = g_rockConfig.rockGrabInputIntentStateEnabled,
                    .leewaySeconds = g_rockConfig.rockGrabInputLeewaySeconds,
                    .forceSeconds = g_rockConfig.rockGrabInputForceSeconds,
                });
            grabInput.held = intentDecision.held;
            grabInput.pressed = intentDecision.pressed;
            grabInput.released = intentDecision.released;
            grabInput.syntheticPressed = intentDecision.syntheticPressed;
            if (intentDecision.syntheticPressed) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} hand delivered latched grab input intent state={} reason={}",
                    hand.handName(),
                    grab_input_intent_policy::stateName(intentDecision.state),
                    intentDecision.reason);
            }

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
                    hasMotionProps = havok_runtime::tryReadBodyMotionPropertiesId(hknp, sel.bodyId, motionProps);
                }

                const bool isLiveNpc = formType == "NPC_" && !selRef->IsDead(false);
                const bool blocked = grab_interaction_policy::shouldBlockSelectedObjectInteractionForTarget(sel.targetKind, formType, isLiveNpc, hasMotionProps, motionProps);
                if (blocked) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand: selected object interaction blocked targetKind={} formType={} formID={:08X} far={} motionProps={} hasMotionProps={}",
                        hand.handName(),
                        grab_target::name(sel.targetKind),
                        formType.empty() ? "???" : typeStr,
                        selRef->GetFormID(),
                        sel.isFarSelection ? "yes" : "no",
                        motionProps,
                        hasMotionProps ? "yes" : "no");
                }
                return blocked;
            };
            auto attemptPeerHeldCloseJoinSelection = [&]() {
                if (!grabInput.pressed || hand.isHolding()) {
                    return false;
                }

                const Hand& peer = isLeft ? _rightHand : _leftHand;
                if (!peer.isHolding() || !peer.getHeldRef()) {
                    return false;
                }

                if (hand.hasSelection() && !hand.getSelection().isFarSelection && hand.getSelection().refr != peer.getHeldRef()) {
                    return false;
                }

                const bool hadPeerHeldCloseSelection =
                    hand.hasSelection() && hand.getSelection().refr == peer.getHeldRef() && !hand.getSelection().isFarSelection;
                const bool refreshedPeerHeldSelection = hand.acquirePeerHeldCloseSelection(frame.bhkWorld,
                    frame.hknpWorld,
                    peer.getSavedObjectState(),
                    peer.getHeldBodyIds(),
                    handInput.grabAnchorWorld,
                    handInput.palmNormalWorld,
                    g_rockConfig.rockNearDetectionRange);
                if (!refreshedPeerHeldSelection && hadPeerHeldCloseSelection) {
                    hand.clearSelectionState(false);
                }
                return refreshedPeerHeldSelection;
            };
            auto attemptSelectedGrab = [&]() {
                const auto& transform = handInput.rawHandWorld;

                const auto sharedContext = makeGrabSharedObjectContext(hand, isLeft);
                const bool grabbedFromPullCatchCommit = hand.hasPendingPullCatchCommit();
                bool grabbed = hand.grabSelectedObject(hknp,
                    transform,
                    g_rockConfig.rockGrabLinearTau,
                    g_rockConfig.rockGrabLinearDamping,
                    g_rockConfig.rockGrabConstraintMaxForce,
                    g_rockConfig.rockGrabLinearProportionalRecovery,
                    g_rockConfig.rockGrabLinearConstantRecovery,
                    &_bodyBoneColliders,
                    sharedContext);

                if (grabbed) {
                    if (sharedContext.joiningPeerHeldObject) {
                        Hand& peer = isLeft ? _rightHand : _leftHand;
                        const auto& peerInput = isLeft ? frame.right : frame.left;
                        if (!peer.promoteHeldObjectToConstraintDrive(frame.bhkWorld,
                                hknp,
                                peerInput.rawHandWorld,
                                g_rockConfig.rockGrabLinearTau,
                                g_rockConfig.rockGrabLinearDamping,
                                g_rockConfig.rockGrabConstraintMaxForce,
                                g_rockConfig.rockGrabLinearProportionalRecovery,
                                g_rockConfig.rockGrabLinearConstantRecovery,
                                "peer-hand-joined-loose-object")) {
                            auto* joinedRef = hand.getHeldRef();
                            ROCK_LOG_WARN(Hand,
                                "{} hand: rolling back shared grab because peer hand could not promote to constraint drive formID={:08X}",
                                hand.handName(),
                                joinedRef ? joinedRef->GetFormID() : 0);
                            hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, makeGrabReleaseContext(hand, isLeft));
                            return false;
                        }
                    }
                    auto* heldRef = hand.getHeldRef();
                    claimObject(heldRef, claimOwnerForHand(isLeft));
                    dispatchPhysicsMessage(kPhysMsg_OnGrab, isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
                    dispatchGrabCommittedEvent(isLeft, heldRef, hand.getSavedObjectState().bodyId.value, hknp);
                    if (grabbedFromPullCatchCommit) {
                        dispatchSimpleGrabEvent(GrabEventType::PullCatchSucceeded, isLeft, heldRef, hand.getSavedObjectState().bodyId.value);
                    }
                }
                return grabbed;
            };
            auto dispatchHeldObjectEventByFormID =
                [&](GrabEventType type, RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t primaryBodyId) {
                    GrabEventData eventData{};
                    eventData.type = type;
                    eventData.sourceKind = GrabEventSourceKind::HeldObject;
                    eventData.isLeft = isLeft;
                    eventData.refr = refr;
                    eventData.formID = formID != 0 ? formID : (refr ? refr->GetFormID() : 0);
                    eventData.primaryBodyId = primaryBodyId;
                    dispatchGrabEvent(eventData);
                };
            auto dispatchShoulderStashEvent = [&](GrabEventType type,
                                                  RE::TESObjectREFR* refr,
                                                  std::uint32_t formID,
                                                  std::uint32_t primaryBodyId,
                                                  const shoulder_stash::Decision& decision) {
                GrabEventData eventData{};
                eventData.type = type;
                eventData.sourceKind = GrabEventSourceKind::HeldObject;
                eventData.isLeft = isLeft;
                eventData.refr = refr;
                eventData.formID = formID != 0 ? formID : (refr ? refr->GetFormID() : 0);
                eventData.primaryBodyId = primaryBodyId;
                eventData.secondaryBodyId = decision.shoulderBodyId;
                eventData.positionGame[0] = decision.nearestPointGame.x;
                eventData.positionGame[1] = decision.nearestPointGame.y;
                eventData.positionGame[2] = decision.nearestPointGame.z;
                eventData.flags |= ROCK_GRAB_EVENT_FLAG_POSITION_VALID;
                if (std::isfinite(decision.speedGameUnitsPerSecond)) {
                    eventData.speedGameUnitsPerSecond = decision.speedGameUnitsPerSecond;
                    eventData.flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
                }
                eventData.intensityHint = std::clamp(std::isfinite(decision.confidence) ? decision.confidence : 0.0f, 0.0f, 1.0f);
                eventData.flags |= ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID;
                dispatchGrabEvent(eventData);
            };

            if (hand.isHolding()) {
                _softContactRuntime.clearHandForStrongerOwner(isLeft, "held-object");
                const Hand& peer = isLeft ? _rightHand : _leftHand;
                auto* heldRefForStash = hand.getHeldRef();
                const bool peerHoldingSameObject =
                    heldRefForStash && peer.isHolding() && peer.getHeldRef() == heldRefForStash;
                const auto stashEligibility = shoulder_stash::evaluateEligibility(shoulder_stash::EligibilityInput{
                    .enabled = g_rockConfig.rockShoulderStashEnabled,
                    .peerHoldingSameObject = peerHoldingSameObject,
                    .heldRef = heldRefForStash,
                    .savedState = &hand.getSavedObjectState(),
                });

                shoulder_stash::Decision stashDecision{};
                if (stashEligibility.eligible) {
                    stashDecision = shoulder_stash::evaluate(shoulder_stash::DetectorInput{
                            .world = hknp,
                            .bodyColliders = &_bodyBoneColliders,
                            .bodyContacts = &_bodyContactRuntime,
                            .heldBodyIds = &hand.getHeldBodyIds(),
                            .contactFrame = _handContactActivity.currentFrame(),
                            .isLeftHand = isLeft,
                            .probe = makeShoulderStashObjectProbe(hknp, hand, handInput),
                            .hmdProbe = makeShoulderStashHmdProbe(handInput),
                            .hasHmdProbe = true,
                            .hasHmdFrame = frame.hasHmdFrame,
                            .hmdPositionWorld = frame.hmdPositionWorld,
                            .hmdForwardWorld = frame.hmdForwardWorld,
                            .deltaSeconds = frame.deltaSeconds,
                            .config = makeShoulderStashDetectorConfig(),
                        },
                        shoulderStashState);
                } else {
                    clearShoulderStashForHand(hand, isLeft);
                }

                if (stashEligibility.eligible && stashDecision.candidate) {
                    if (hand.getState() == HandState::HeldBody) {
                        hand.beginStashCandidate();
                    }
                    if (hand.getState() == HandState::StashCandidate) {
                        const bool pulseDue = _dynamicPushElapsedSeconds >= shoulderStashState.nextCandidatePulseTimeSeconds;
                        if (stashDecision.enteredCandidate || stashDecision.changedCandidate || pulseDue) {
                            dispatchShoulderStashEvent(
                                GrabEventType::StashCandidate,
                                heldRefForStash,
                                heldRefForStash ? heldRefForStash->GetFormID() : 0,
                                hand.getSavedObjectState().bodyId.value,
                                stashDecision);
                            shoulderStashState.nextCandidatePulseTimeSeconds =
                                _dynamicPushElapsedSeconds + (std::max)(0.02f, g_rockConfig.rockShoulderStashCandidateHapticIntervalSeconds);
                        }
                    }
                } else {
                    hand.cancelStashCandidate();
                }

                if (grabInput.released) {
                    hand.captureHeldReleaseMotion(hknp, handInput.rawHandWorld, _heldObjectPlayerSpaceFrame, frame.deltaSeconds);
                    auto* heldRef = hand.getHeldRef();
                    std::uint32_t heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                    if (stashEligibility.eligible && stashDecision.confirmedForCommit && hand.getState() == HandState::StashCandidate) {
                        /*
                         * Shoulder stash uses a two-phase release because native
                         * pickup can still refuse the reference. ROCK detaches the
                         * grab as a pending transfer, captures the physical release
                         * velocity, then applies that velocity only when transfer
                         * fails so successful stash stays quiet and failed stash is
                         * an honest drop.
                         */
                        auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                        releaseContext.disposition = GrabReleaseDisposition::PendingInventoryTransfer;
                        releaseContext.reason = "shoulder-stash-pending-transfer";
                        const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                        const auto releaseOutcome = hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                        if (heldRef) {
                            releaseObject(heldRef, claimOwnerForHand(isLeft));
                        }
                        const auto transferResult = shoulder_stash::transferToPlayerInventory(shoulder_stash::TransferInput{
                            .heldRef = heldRef,
                            .skipActivateBooks = g_rockConfig.rockShoulderStashSkipActivateBooks,
                            .skipActivateNotes = g_rockConfig.rockShoulderStashSkipActivateNotes,
                            .playPickupSounds = true,
                        });
                        if (heldFormID == 0 && transferResult.formID != 0) {
                            heldFormID = transferResult.formID;
                        }

                        auto* postTransferRef = transferResult.attempted ? nullptr : heldRef;
                        dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, postTransferRef, heldFormID, 0);
                        if (transferResult.success) {
                            dispatchShoulderStashEvent(GrabEventType::Stashed, nullptr, heldFormID, primaryBodyId, stashDecision);
                            showShoulderStashCollectedNotification(transferResult, heldFormID);
                        } else {
                            hand.applyReleaseVelocitySnapshot(hknp, releaseOutcome.velocity);
                            dispatchHeldObjectEventByFormID(GrabEventType::Released, postTransferRef, heldFormID, primaryBodyId);
                        }
                        ROCK_LOG_INFO(Hand,
                            "{} hand shoulder stash release formID={:08X} success={} transferReason={} stashSource={} zone={} count={} confidence={:.2f} speed={:.1f}",
                            hand.handName(),
                            heldFormID,
                            transferResult.success ? "yes" : "no",
                            shoulder_stash::transferReasonName(transferResult.reason),
                            shoulder_stash::evidenceSourceName(stashDecision.source),
                            body_zone::bodyZoneName(stashDecision.zone),
                            transferResult.count,
                            stashDecision.confidence,
                            stashDecision.speedGameUnitsPerSecond);
                        clearShoulderStashForHand(hand, isLeft);
                        return;
                    }
                    hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(hand, isLeft));
                    if (heldRef)
                        releaseObject(heldRef, claimOwnerForHand(isLeft));
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                    dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                    clearShoulderStashForHand(hand, isLeft);
                } else {
                    const auto& transform = handInput.rawHandWorld;
                    auto* heldRef = hand.getHeldRef();
                    auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                    hand.updateHeldObject(hknp,
                        transform,
                        _heldObjectPlayerSpaceFrame,
                        frame.deltaSeconds,
                        g_rockConfig.rockGrabForceFadeInTime,
                        g_rockConfig.rockGrabTauMin,
                        &_bodyBoneColliders,
                        makeGrabReleaseContext(hand, isLeft));
                    if (heldRef && !hand.isHolding()) {
                        releaseObject(heldRef, claimOwnerForHand(isLeft));
                        dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                        dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                        clearShoulderStashForHand(hand, isLeft);
                    }
                }
            } else {
                clearShoulderStashForHand(hand, isLeft);
                if (grabInput.pressed) {
                    attemptPeerHeldCloseJoinSelection();
                }
            }

            auto actorEquipmentHandoffMaxSeconds = []() -> float {
                return (std::isfinite(g_rockConfig.rockPullCatchRetryMaxTimeSeconds) && g_rockConfig.rockPullCatchRetryMaxTimeSeconds > 0.0f) ?
                           g_rockConfig.rockPullCatchRetryMaxTimeSeconds :
                           0.65f;
            };

            if (!hand.isHolding() && selection_state_policy::canProcessSelectedState(hand.getState()) && hand.hasSelection()) {
                const bool pullCatchCommitPending = hand.hasPendingPullCatchCommit();
                auto* pullCatchRef = pullCatchCommitPending ? hand.getPullCatchIntentRef() : nullptr;
                bool actorEquipmentDropHandoffReady = false;
                if (pullCatchCommitPending) {
                    if (grabInput.released || !grabInput.held) {
                        ROCK_LOG_DEBUG(Hand, "{} hand cancelled pull catch commit because grip was released", hand.handName());
                        hand.clearSelectionState(true);
                        releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                        return;
                    }
                    if (!hand.advancePullCatchCommit(frame.deltaSeconds, g_rockConfig.rockPullCatchRetryMaxTimeSeconds)) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand cancelled pull catch commit because retry window expired ({:.3f}s)",
                            hand.handName(),
                            g_rockConfig.rockPullCatchRetryMaxTimeSeconds);
                        hand.clearSelectionState(true);
                        releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                        return;
                    }
                }

                if (hand.hasPendingActorEquipmentDropHandoff()) {
                    if (grabInput.released || !grabInput.held) {
                        ROCK_LOG_DEBUG(Hand, "{} hand cancelled actor-equipment drop handoff because grip was released", hand.handName());
                        hand.clearSelectionState(true);
                        return;
                    }

                    const auto handoffStatus = hand.advanceActorEquipmentDropHandoff(
                        frame.bhkWorld,
                        hknp,
                        frame.deltaSeconds,
                        actorEquipmentHandoffMaxSeconds());
                    switch (handoffStatus) {
                    case Hand::ActorEquipmentDropHandoffStatus::Ready:
                        actorEquipmentDropHandoffReady = true;
                        break;
                    case Hand::ActorEquipmentDropHandoffStatus::Pending:
                        return;
                    case Hand::ActorEquipmentDropHandoffStatus::None:
                        break;
                    case Hand::ActorEquipmentDropHandoffStatus::InvalidSelection:
                    case Hand::ActorEquipmentDropHandoffStatus::MissingDroppedReference:
                    case Hand::ActorEquipmentDropHandoffStatus::TimedOut:
                    default:
                        ROCK_LOG_WARN(Hand,
                            "{} hand actor-equipment drop handoff failed status={} actorSelection={:08X}",
                            hand.handName(),
                            static_cast<int>(handoffStatus),
                            hand.getSelection().refr ? hand.getSelection().refr->GetFormID() : 0);
                        hand.clearSelectionState(true);
                        return;
                    }
                }

                if (grabInput.pressed || (pullCatchCommitPending && grabInput.held) || (actorEquipmentDropHandoffReady && grabInput.held)) {
                    if (!grab_interaction_policy::canAttemptSelectedObjectGrab(
                            hand.getSelection().isFarSelection, hand.getSelection().distance, g_rockConfig.rockFarDetectionRange)) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand: far grab blocked (dist={:.1f}, configuredFarRange={:.1f})",
                            hand.handName(),
                            hand.getSelection().distance,
                            g_rockConfig.rockFarDetectionRange);
                        return;
                    }

                    if (hand.getSelection().isFarSelection) {
                        float hmdConeDot = -1.0f;
                        if (!selectedObjectPassesFarHmdCone(hknp, hand.getSelection(), farHmdConeGate, &hmdConeDot)) {
                            ROCK_LOG_DEBUG(Hand,
                                "{} hand: far grab blocked outside HMD cone formID={:08X} hmdDot={:.3f} minDot={:.3f}",
                                hand.handName(),
                                hand.getSelection().refr ? hand.getSelection().refr->GetFormID() : 0,
                                hmdConeDot,
                                farHmdConeGate.minDot);
                            hand.clearSelectionState(true);
                            return;
                        }
                    }

                    if (!pullCatchCommitPending &&
                        hand.getSelection().isFarSelection &&
                        hand.getSelection().targetKind == grab_target::Kind::ActorEquipment) {
                        const auto actorSelection = hand.getSelection();
                        const auto dropResult = actor_equipment_grab::dropFarActorEquipmentSelection(
                            actorSelection.refr,
                            actorSelection.actorEquipment,
                            actor_equipment_grab::kDefaultAttachedDropZOffsetGameUnits);
                        if (dropResult.status != actor_equipment_grab::DropStatus::Success || !dropResult.droppedRef) {
                            ROCK_LOG_WARN(Hand,
                                "{} hand actor-equipment far pull failed before drop handoff: status={} actor={:08X} item={:08X}",
                                hand.handName(),
                                actor_equipment_grab::dropStatusName(dropResult.status),
                                dropResult.actorFormId,
                                dropResult.itemFormId);
                            hand.clearSelectionState(true);
                            return;
                        }

                        if (!hand.beginActorEquipmentDropHandoff(
                                dropResult,
                                actorSelection.hasHitPoint ? actorSelection.hitPointWorld : actorSelection.actorEquipment.hitPointWorld)) {
                            ROCK_LOG_WARN(Hand,
                                "{} hand actor-equipment far pull failed to arm drop handoff: dropped={:08X} actor={:08X} item={:08X}",
                                hand.handName(),
                                dropResult.droppedFormId,
                                dropResult.actorFormId,
                                dropResult.itemFormId);
                            hand.clearSelectionState(true);
                            return;
                        }

                        const auto handoffStatus = hand.advanceActorEquipmentDropHandoff(
                            frame.bhkWorld,
                            hknp,
                            0.0f,
                            actorEquipmentHandoffMaxSeconds());
                        if (handoffStatus == Hand::ActorEquipmentDropHandoffStatus::Ready) {
                            actorEquipmentDropHandoffReady = true;
                        } else if (handoffStatus == Hand::ActorEquipmentDropHandoffStatus::Pending) {
                            return;
                        } else {
                            ROCK_LOG_WARN(Hand,
                                "{} hand actor-equipment far pull failed after arming handoff: status={} dropped={:08X} actor={:08X} item={:08X}",
                                hand.handName(),
                                static_cast<int>(handoffStatus),
                                dropResult.droppedFormId,
                                dropResult.actorFormId,
                                dropResult.itemFormId);
                            hand.clearSelectionState(true);
                            return;
                        }
                    }

                    if (selectedObjectInteractionBlocked()) {
                        if (pullCatchCommitPending) {
                            hand.clearSelectionState(true);
                            releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                        }
                        return;
                    }

                    if (hand.getSelection().isFarSelection) {
                        if (pullCatchCommitPending) {
                            ROCK_LOG_WARN(Hand,
                                "{} hand cancelled pull catch commit because pending catch unexpectedly resolved to far selection",
                                hand.handName());
                            hand.clearSelectionState(true);
                            releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                            return;
                        }
                        auto* selectedRef = hand.getSelection().refr;
                        const auto selectedBodyId = hand.getSelection().bodyId.value;
                        const auto& transform = handInput.rawHandWorld;
                        _softContactRuntime.clearHandForStrongerOwner(isLeft, "dynamic-pull-start");
                        /*
                         * Far-pull startup publishes the lock before dynamic body conversion because
                         * startDynamicPull owns failure cleanup and may clear selection internally.
                         * Deferring SelectionLocked fixed haptic overwrite but exposed impossible
                         * lock/unlock ordering to API consumers, so the event stays ordered and only
                         * the selection haptic is suppressed on this pull-start path.
                         */
                        const bool lockedSelection = hand.lockFarSelection();
                        if (lockedSelection) {
                            dispatchSimpleGrabEvent(
                                GrabEventType::SelectionLocked,
                                isLeft,
                                selectedRef,
                                selectedBodyId,
                                ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC);
                        }
                        const bool pullStarted = lockedSelection && hand.startDynamicPull(hknp, transform);
                        if (pullStarted) {
                            claimObject(selectedRef, claimOwnerForHand(isLeft));
                            dispatchSimpleGrabEvent(GrabEventType::PullStarted, isLeft, selectedRef, selectedBodyId);
                        } else {
                            if (lockedSelection) {
                                dispatchSimpleGrabEvent(GrabEventType::SelectionUnlocked, isLeft, selectedRef, selectedBodyId);
                            }
                            releaseObject(selectedRef, claimOwnerForHand(isLeft));
                        }
                        return;
                    }

                    if (pullCatchCommitPending) {
                        _softContactRuntime.clearHandForStrongerOwner(isLeft, "dynamic-pull-grab-capture-retry");
                    } else {
                        _softContactRuntime.clearHandForStrongerOwner(isLeft, "normal-grab-capture");
                    }
                    if (pullCatchCommitPending) {
                        dispatchSimpleGrabEvent(GrabEventType::PullCatchAttempt, isLeft, pullCatchRef, hand.getSelection().bodyId.value);
                    }
                    const bool grabbed = attemptSelectedGrab();
                    if (!grabbed && pullCatchCommitPending) {
                        hand.notePullCatchCommitAttemptFailed();
                        ROCK_LOG_SAMPLE_DEBUG(Hand,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "{} hand retaining pull catch commit after grab attempt failed; grip still held",
                            hand.handName());
                    }
                }
            } else if (hand.getState() == HandState::SelectionLocked) {
                if (grabInput.released) {
                    auto* selectedRef = hand.getSelection().refr;
                    ROCK_LOG_DEBUG(Hand, "{} hand released locked far selection", hand.handName());
                    dispatchSimpleGrabEvent(GrabEventType::SelectionUnlocked, isLeft, selectedRef, hand.getSelection().bodyId.value);
                    hand.clearSelectionState(true);
                    releaseObject(selectedRef, claimOwnerForHand(isLeft));
                }
            } else if (hand.getState() == HandState::Pulled) {
                auto* pulledRef = hand.getSelection().refr;
                if (grabInput.released) {
                    ROCK_LOG_DEBUG(Hand, "{} hand released dynamic pull", hand.handName());
                    hand.clearSelectionState(true);
                    releaseObject(pulledRef, claimOwnerForHand(isLeft));
                    return;
                }

                const auto& transform = handInput.rawHandWorld;
                const bool readyToGrab = hand.updateDynamicPull(hknp, transform, frame.deltaSeconds);
                if (!hand.hasSelection() || hand.getState() == HandState::Idle) {
                    releaseObject(pulledRef, claimOwnerForHand(isLeft));
                    return;
                }

                if (readyToGrab) {
                    dispatchSimpleGrabEvent(GrabEventType::PullArrived, isLeft, pulledRef, hand.getSelection().bodyId.value);
                    if (selectedObjectInteractionBlocked()) {
                        hand.clearSelectionState(true);
                        releaseObject(pulledRef, claimOwnerForHand(isLeft));
                        return;
                    }

                    _softContactRuntime.clearHandForStrongerOwner(isLeft, "dynamic-pull-grab-capture");
                    dispatchSimpleGrabEvent(GrabEventType::PullCatchAttempt, isLeft, pulledRef, hand.getSelection().bodyId.value);
                    const bool grabbed = attemptSelectedGrab();
                    if (!grabbed && (!hand.hasSelection() || !hand.hasPendingPullCatchCommit())) {
                        hand.clearSelectionState(true);
                        releaseObject(pulledRef, claimOwnerForHand(isLeft));
                    } else if (!grabbed) {
                        hand.notePullCatchCommitAttemptFailed();
                        ROCK_LOG_SAMPLE_DEBUG(Hand,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "{} hand pull arrived but grab commit did not accept yet; retaining catch intent while grip is held",
                            hand.handName());
                    }
                }
            }
        };

        processHand(_rightHand, false);
        processHand(_leftHand, true);
    }

#include "physics-interaction/core/PhysicsInteractionContacts.inl"
    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_ownedObjectsMutex);
        const auto it = _ownedObjects.find(ref->GetFormID());
        return it != _ownedObjects.end() && it->second != 0;
    }

    bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner) const
    {
        if (!ref)
            return false;
        std::scoped_lock lock(_ownedObjectsMutex);
        const auto it = _ownedObjects.find(ref->GetFormID());
        return it != _ownedObjects.end() && (it->second & claimOwnerBit(owner)) != 0;
    }

    void PhysicsInteraction::claimObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        auto& ownerMask = _ownedObjects[formID];
        const auto previousMask = ownerMask;
        ownerMask |= claimOwnerBit(owner);
        ROCK_LOG_DEBUG(Hand,
            "Claimed object: formID={:08X} owner={} mask=0x{:02X}->0x{:02X} owners={}",
            formID,
            static_cast<std::uint32_t>(owner),
            previousMask,
            ownerMask,
            claimOwnerCount(ownerMask));
    }

    void PhysicsInteraction::releaseObject(RE::TESObjectREFR* ref, PhysicsObjectClaimOwner owner)
    {
        if (!ref)
            return;
        auto formID = ref->GetFormID();
        std::scoped_lock lock(_ownedObjectsMutex);
        auto it = _ownedObjects.find(formID);
        if (it == _ownedObjects.end()) {
            return;
        }

        const auto previousMask = it->second;
        it->second &= ~claimOwnerBit(owner);
        if (it->second != 0) {
            ROCK_LOG_DEBUG(Hand,
                "Released object claim: formID={:08X} owner={} mask=0x{:02X}->0x{:02X} ownersRemaining={}",
                formID,
                static_cast<std::uint32_t>(owner),
                previousMask,
                it->second,
                claimOwnerCount(it->second));
            return;
        }

        _ownedObjects.erase(it);
        ROCK_LOG_DEBUG(Hand,
            "Released object: formID={:08X} owner={} mask=0x{:02X}->0x00",
            formID,
            static_cast<std::uint32_t>(owner),
            previousMask);
    }

    void PhysicsInteraction::releaseAllObjects()
    {
        std::scoped_lock lock(_ownedObjectsMutex);
        if (!_ownedObjects.empty()) {
            ROCK_LOG_DEBUG(Hand, "Releasing all {} owned objects", _ownedObjects.size());
            _ownedObjects.clear();
        }

        (void)frik_visual_authority::blockOffHandWeaponGripping("ROCK_Physics", false);
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

        hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(hand, isLeft));
        if (heldRef)
            releaseObject(heldRef, claimOwnerForHand(isLeft));
        dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
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

        return havok_runtime::getHknpWorldFromBhk(bhk);
    }
}
