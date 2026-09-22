#pragma once

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/native/NativeImpactAudio.h"


#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"
#include "api/ProviderStatePolicy.h"
#include "api/ROCKProviderApiInternal.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/ContactSignalSubscriptionPolicy.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"
#include "physics-interaction/consume/MouthConsumePolicy.h"
#include "physics-interaction/consume/MouthConsumeTransfer.h"
#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/debug/DebugBodyOverlay.h"
#include "physics-interaction/debug/DebugVisualizationPolicy.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/hand/DynamicHandCollisionPolicy.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/ReferenceInteraction.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/HeldScenePresentation.h"
#include "physics-interaction/native/HeldWeaponInstantTransition.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/NativeEquippedWeaponDraw.h"
#include "physics-interaction/weapon/WeaponTransitionAnimationAcceleration.h"
#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/timing/RockGameTiming.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/PhysicsRayCast.h"
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
#include "physics-interaction/TransformMath.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/ActorValueInfo.h"
#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/UI.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

#include "ROCKMain.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/Fo4VrActorStatePolicy.h"
#include "rock_support/VRControllers.h"
#include <windows.h>

/*
 * Shared internals of the PhysicsInteraction implementation, split across
 * the interaction/ translation units. These helpers lived in the anonymous
 * namespace of the former single PhysicsInteraction.cpp; the using-directive
 * below preserves unqualified name lookup for the moved method bodies.
 * Mutable former file-scope state is declared inline so exactly one
 * instance exists across the split, matching the single-TU behavior.
 * Include only from PhysicsInteraction implementation files.
 */

namespace rock
{
    namespace physics_interaction_internal
    {
        inline constexpr float kRawParityWarnPosition = 0.10f;
        inline constexpr float kRawParityWarnRotationDegrees = 0.5f;
        inline constexpr float kRawParityFailPosition = 0.50f;
        inline constexpr float kRawParityFailRotationDegrees = 2.0f;
        inline constexpr int kRawParityWarnFrames = 2;
        inline constexpr int kRawParityFailFrames = 10;
        inline constexpr std::array<std::string_view, 5> kWeaponCollisionWorkbenchExitMenuNames{
            "ExamineMenu",
            "PowerArmorModMenu",
            "RobotModMenu",
            "Crafting Menu",
            "CraftingMenu",
        };
        inline constexpr float kNearbyCarCollisionRadiusGameUnits = 4096.0f;

        template <std::size_t Count>
        void releaseStaleGeneratedHandSuppressionLeases(
            RE::hknpWorld* world,
            const Hand& hand,
            collision_suppression_registry::SuppressionLeaseSet<Count>& suppressionSet,
            const char* context)
        {
            if (!world) {
                return;
            }

            const auto handContainsBody = [&](std::uint32_t bodyId) {
                if (bodyId == collision_suppression_registry::kInvalidBodyId) {
                    return false;
                }

                const std::uint32_t colliderCount =
                    hand.getHandColliderBodyCount();
                if (colliderCount > 0) {
                    for (std::uint32_t index = 0;
                         index < colliderCount;
                         ++index) {
                        if (hand.getHandColliderBodyIdAtomic(index) == bodyId) {
                            return true;
                        }
                    }
                    return false;
                }

                return hand.hasCollisionBody() &&
                       hand.getCollisionBodyId().value == bodyId;
            };

            suppressionSet.releaseWhere(
                world,
                context,
                [&handContainsBody](std::uint32_t bodyId) {
                    return !handContainsBody(bodyId);
                },
                [](std::uint32_t, const auto&) {});
        }

        inline std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkRegistered{ false };
        inline std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged{ false };

        [[nodiscard]] inline bool isWeaponCollisionWorkbenchExitMenu(const RE::BSFixedString& menuName)
        {
            for (const auto name : kWeaponCollisionWorkbenchExitMenuNames) {
                if (menuName == name) {
                    return true;
                }
            }

            return false;
        }

        class WeaponCollisionWorkbenchExitMenuSink final : public RE::BSTEventSink<RE::MenuOpenCloseEvent>
        {
        public:
            RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent& event, RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override
            {
                if (event.opening || !event.menuName.c_str() || !isWeaponCollisionWorkbenchExitMenu(event.menuName)) {
                    return RE::BSEventNotifyControl::kContinue;
                }

                auto* interaction = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
                if (interaction && interaction->isInitialized()) {
                    interaction->requestWeaponCollisionRebuildAfterWorkbenchExit(event.menuName.c_str());
                }

                return RE::BSEventNotifyControl::kContinue;
            }
        };

        inline WeaponCollisionWorkbenchExitMenuSink s_weaponCollisionWorkbenchExitMenuSink;

        inline bool tryReadNativeScopeRequestState(bool& outActive)
        {
            performance_profiler::ScopedTimer gripStageTimer(performance_profiler::Scope::EquippedNativeScopeRead);
            using GetScopeRequestState = bool (*)(const void*);
            static REL::Relocation<GetScopeRequestState> getScopeRequestState{ REL::Offset(offsets::kFunc_NativeScopeRequestStateGet) };
            static REL::Relocation<std::uintptr_t> rendererState{ REL::Offset(offsets::kData_NativeScopeRendererState) };
            if (!getScopeRequestState.address() || !rendererState.address()) {
                return false;
            }
            outActive = getScopeRequestState(reinterpret_cast<const void*>(rendererState.address()));
            return true;
        }

        inline bool ensureWeaponCollisionWorkbenchExitMenuSinkRegistered()
        {
            bool expected = false;
            if (!s_weaponCollisionWorkbenchExitMenuSinkRegistered.compare_exchange_strong(
                    expected,
                    true,
                    std::memory_order_acq_rel,
                    std::memory_order_acquire)) {
                return true;
            }

            auto* ui = RE::UI::GetSingleton();
            if (!ui) {
                s_weaponCollisionWorkbenchExitMenuSinkRegistered.store(false, std::memory_order_release);
                if (!s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged.exchange(true, std::memory_order_acq_rel)) {
                    ROCK_LOG_WARN(Weapon, "UI singleton unavailable; weapon collision workbench-exit menu sink will retry");
                }
                return false;
            }

            ui->RegisterSink<RE::MenuOpenCloseEvent>(&s_weaponCollisionWorkbenchExitMenuSink);
            s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged.store(false, std::memory_order_release);
            ROCK_LOG_INFO(Weapon,
                "Registered weapon collision workbench-exit menu sink for {} menu names",
                kWeaponCollisionWorkbenchExitMenuNames.size());
            return true;
        }

        constexpr std::uint32_t claimOwnerBit(PhysicsObjectClaimOwner owner)
        {
            return 1u << static_cast<std::uint32_t>(owner);
        }

        constexpr PhysicsObjectClaimOwner claimOwnerForHand(bool isLeft)
        {
            return isLeft ? PhysicsObjectClaimOwner::LeftHand : PhysicsObjectClaimOwner::RightHand;
        }

        inline void clearEquippedWeaponFiringGripInputState()
        {
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
            input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
            input_remap_runtime::setEquippedWeaponLeftHandFiringActive(false);
        }

        [[nodiscard]] inline float pointLength(const RE::NiPoint3& value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        }

        inline FarSelectionHmdConeGate makeFarSelectionHmdConeGate(const PhysicsFrameContext& frame)
        {
            FarSelectionHmdConeGate gate{};
            gate.enabled = selection_query_policy::kFarSelectionHmdConeEnabled;
            gate.hasHmdFrame = frame.hasHmdFrame;
            gate.hmdPositionWorld = frame.hmdPositionWorld;
            gate.hmdForwardWorld = frame.hmdForwardWorld;
            gate.minDot = selection_query_policy::farSelectionHmdConeMinDot(selection_query_policy::kFarSelectionHmdConeHalfAngleDegrees);
            return gate;
        }

        inline shoulder_stash::DetectorConfig makeShoulderStashDetectorConfig()
        {
            shoulder_stash::DetectorConfig config{};
            config.enabled = g_rockConfig.rockShoulderStashEnabled;
            config.useBodyZoneColliders = g_rockConfig.rockShoulderStashUseBodyZoneColliders;
            config.useHmdBackVolume = g_rockConfig.rockShoulderStashUseHmdBackVolume;
            config.enterPaddingGameUnits = g_rockConfig.rockShoulderStashEnterPaddingGameUnits;
            config.exitPaddingGameUnits = g_rockConfig.rockShoulderStashExitPaddingGameUnits;
            config.minDwellSeconds = g_rockConfig.rockShoulderStashMinDwellSeconds;
            config.maxSpeedGameUnitsPerSecond = g_rockConfig.rockShoulderStashMaxSpeedGameUnitsPerSecond;
            config.recentContactSeconds = g_rockConfig.rockShoulderStashRecentContactSeconds;
            config.sustainedContactMissSeconds = g_rockConfig.rockShoulderStashSustainedContactMissSeconds;
            config.hmdBackRightOffsetGameUnits = g_rockConfig.rockShoulderStashHmdBackRightOffsetGameUnits;
            config.hmdBackLeftOffsetGameUnits = g_rockConfig.rockShoulderStashHmdBackLeftOffsetGameUnits;
            config.hmdBackRadiusGameUnits = g_rockConfig.rockShoulderStashHmdBackRadiusGameUnits;
            config.hmdBackEnterPaddingGameUnits = g_rockConfig.rockShoulderStashHmdBackEnterPaddingGameUnits;
            config.hmdBackExitPaddingGameUnits = g_rockConfig.rockShoulderStashHmdBackExitPaddingGameUnits;
            config.hmdBackMinBehindGameUnits = g_rockConfig.rockShoulderStashHmdBackMinBehindGameUnits;
            return config;
        }

        /*
         * Equipped-weapon stash reuses the loose-object back volume so the
         * player learns one gesture. Body-zone collider/contact evidence is
         * disabled for this variant: the equipped weapon has no held-body
         * contact identity, so the HMD-relative back volume (forced on) is the
         * gesture authority for hand-carried weapons.
         */
        inline shoulder_stash::DetectorConfig makeEquippedWeaponStashDetectorConfig(bool enabled)
        {
            shoulder_stash::DetectorConfig config = makeShoulderStashDetectorConfig();
            config.enabled = enabled;
            config.useBodyZoneColliders = false;
            config.useHmdBackVolume = true;
            return config;
        }

        inline shoulder_stash::Probe makeShoulderStashObjectProbe(RE::hknpWorld* world, const Hand& hand, const HandFrameInput& handInput)
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

        inline shoulder_stash::Probe makeShoulderStashHmdProbe(const HandFrameInput& handInput)
        {
            shoulder_stash::Probe probe{};
            probe.pointGame = handInput.rawHandWorld.translate;
            return probe;
        }

        inline mouth_consume::DetectorConfig makeMouthConsumeDetectorConfig()
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

        inline mouth_consume::Probe makeMouthConsumeObjectProbe(RE::hknpWorld* world, const Hand& hand, const HandFrameInput& handInput)
        {
            mouth_consume::Probe probe{};
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

        inline mouth_consume::Probe makeMouthConsumeHandProbe(const HandFrameInput& handInput)
        {
            mouth_consume::Probe probe{};
            probe.pointGame = handInput.grabAnchorWorld;
            return probe;
        }

        inline std::string_view shoulderStashItemName(RE::TESBoundObject* baseForm)
        {
            if (!baseForm) {
                return {};
            }

            return RE::TESFullName::GetFullName(*baseForm, false);
        }

        inline void showShoulderStashCollectedNotification(const shoulder_stash::TransferResult& transferResult, std::uint32_t fallbackFormID)
        {
            if (!g_rockConfig.rockShoulderStashShowCollectedNotifications) {
                return;
            }

            f4vr::showNotification(shoulder_stash_notification_policy::formatCollectedNotification(
                shoulderStashItemName(transferResult.baseForm),
                transferResult.count,
                transferResult.formID != 0 ? transferResult.formID : fallbackFormID));
        }

        inline std::uint32_t claimOwnerCount(std::uint32_t ownerMask)
        {
            std::uint32_t count = 0;
            while (ownerMask != 0) {
                count += ownerMask & 1u;
                ownerMask >>= 1u;
            }
            return count;
        }

        inline constexpr std::uint32_t kInvalidAtomicBodyId = 0xFFFF'FFFFu;
        inline constexpr std::uint64_t kInvalidHeldImpactPair = 0xFFFF'FFFF'FFFF'FFFFull;

        inline bool isInvalidGrabBodyId(std::uint32_t bodyId)
        {
            return bodyId == kInvalidAtomicBodyId ||
                   bodyId == ROCK_GRAB_EVENT_INVALID_BODY_ID ||
                   bodyId == object_physics_body_set::INVALID_BODY_ID;
        }

        inline std::uint64_t packHeldImpactPair(std::uint32_t heldBodyId, std::uint32_t otherBodyId)
        {
            if (isInvalidGrabBodyId(heldBodyId) || isInvalidGrabBodyId(otherBodyId)) {
                return kInvalidHeldImpactPair;
            }
            return (static_cast<std::uint64_t>(heldBodyId) << 32) | static_cast<std::uint64_t>(otherBodyId);
        }

        inline bool unpackHeldImpactPair(std::uint64_t packedPair, std::uint32_t& heldBodyId, std::uint32_t& otherBodyId)
        {
            if (packedPair == kInvalidHeldImpactPair) {
                return false;
            }
            heldBodyId = static_cast<std::uint32_t>(packedPair >> 32);
            otherBodyId = static_cast<std::uint32_t>(packedPair & 0xFFFF'FFFFu);
            return !isInvalidGrabBodyId(heldBodyId) && !isInvalidGrabBodyId(otherBodyId);
        }

        inline float readGrabEventBodyMass(RE::hknpWorld* world, std::uint32_t bodyId)
        {
            if (!world || isInvalidGrabBodyId(bodyId)) {
                return 0.0f;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
            if (!motion) {
                return 0.0f;
            }

            const auto packedInvMass = static_cast<std::int16_t>(motion->packedInverseInertia[3]);
            if (packedInvMass == 0) {
                return 0.0f;
            }
            return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
        }

        inline bool applyPlayerSpeedReduction(float previousReduction, float targetReduction)
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

        inline std::uint32_t fillGrabEventBodyKinematics(RE::hknpWorld* world, std::uint32_t bodyId, GrabEventData& eventData)
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
        inline constexpr int kRawParitySummaryFrames = 300;
        inline constexpr int kRawParityLagFrames = 5;
        inline constexpr float kRawParityLagSlack = 0.05f;
        inline DirectSkeletonBoneReader s_directSkeletonBoneReader;
        inline std::uint32_t s_directSkeletonBoneLogCounter = 0;
        inline bool s_worldOriginDiagnosticsEnabledLogged = false;

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

        inline ContactEventSubscriptionBridge s_contactEventBridge;
        inline ContactEventSubscriptionBridge s_manifoldProcessedEventBridge;

        struct GrabButtonState
        {
            bool held{ false };
            bool pressed{ false };
            bool released{ false };
            bool syntheticPressed{ false };
        };

        inline GrabButtonState readGrabButtonState(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isAllowedGrabButtonId(buttonId)) {
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

        inline GrabButtonState peekGrabButtonState(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isAllowedGrabButtonId(buttonId)) {
                return {};
            }

            const auto rawState = input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return GrabButtonState{
                    .held = rawState.held,
                    .pressed = rawState.pressed,
                    .released = rawState.released,
                };
            }

            const auto vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;
            return GrabButtonState{
                .held = vrcf::VRControllers.isPressHeldDown(vrHand, buttonId),
                .pressed = vrcf::VRControllers.isPressed(vrHand, buttonId),
                .released = vrcf::VRControllers.isReleased(vrHand, buttonId),
            };
        }

        [[nodiscard]] constexpr
            equipped_weapon_shoulder::NativePresentation
            shoulderNativePresentation(
                const std::uint32_t nativeWeaponState) noexcept
        {
            using Presentation =
                equipped_weapon_shoulder::NativePresentation;
            switch (nativeWeaponState) {
            case 0:
                return Presentation::StableSheathed;
            case 1:
                return Presentation::WantDraw;
            case 2:
                return Presentation::Drawing;
            case 3:
                return Presentation::StableDrawn;
            case 4:
                return Presentation::WantSheathe;
            case 5:
                return Presentation::Sheathing;
            default:
                return Presentation::Invalid;
            }
        }

        inline bool readGrabButtonHeld(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isAllowedGrabButtonId(buttonId)) {
                return false;
            }

            const auto rawState = input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return rawState.held;
            }

            return vrcf::VRControllers.isPressHeldDown(isLeft ? vrcf::Hand::Left : vrcf::Hand::Right, buttonId);
        }

        inline bool readGrabButtonPressedEdge(bool isLeft, int buttonId)
        {
            if (!input_remap_policy::isAllowedGrabButtonId(buttonId)) {
                return false;
            }

            const auto rawState = input_remap_runtime::peekRawButtonState(isLeft, buttonId);
            if (rawState.available) {
                return rawState.pressed;
            }

            return vrcf::VRControllers.isPressed(isLeft ? vrcf::Hand::Left : vrcf::Hand::Right, buttonId);
        }

        inline bool readHeldWeaponEquipTriggerPressedEdge(bool isLeft)
        {
            constexpr int buttonId = input_remap_policy::kOpenVrSteamVrTriggerButtonId;
            const auto rawState = input_remap_runtime::consumeRawButtonState(isLeft, buttonId);
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

        inline TransformDelta measureTransformDelta(const RE::NiTransform& a, const RE::NiTransform& b)
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

        inline float measurePointDelta(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        enum class PalmClockLogMode : std::uint8_t
        {
            Disabled,
            Sampled,
            Trace,
        };

        inline const char* physicsStepPhaseName(havok_physics_timing::PhysicsStepPhase phase)
        {
            switch (phase) {
            case havok_physics_timing::PhysicsStepPhase::WholePreStep:
                return "whole-pre";
            case havok_physics_timing::PhysicsStepPhase::SubstepPreCollide:
                return "substep-pre-collide";
            case havok_physics_timing::PhysicsStepPhase::BetweenCollideAndSolve:
                return "between-collide-solve";
            case havok_physics_timing::PhysicsStepPhase::SubstepPostSolve:
                return "substep-post-solve";
            }
            return "unknown";
        }

        inline PalmClockLogMode palmClockLogMode()
        {
            if (g_rockConfig.rockDebugGrabTimelineTrace) {
                return PalmClockLogMode::Trace;
            }
            if (g_rockConfig.rockDebugGrabFrameLogging || g_rockConfig.rockDebugVerboseLogging) {
                return PalmClockLogMode::Sampled;
            }
            return PalmClockLogMode::Disabled;
        }

        inline bool palmClockTraceFrameSelected(std::uint64_t gameFrameIndex)
        {
            const auto interval = static_cast<std::uint64_t>((std::max)(1, g_rockConfig.rockDebugGrabTimelineTraceIntervalFrames));
            return gameFrameIndex <= 3 || (gameFrameIndex % interval) == 0;
        }

        /*
         * Palm clock diagnostics compare the game-frame queued skeleton target to
         * the live palm body at each authority boundary. This keeps rate-mismatch
         * evidence in one log row without changing grab authority.
         */
        inline void logPalmClockSampleForHand(
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
            if (mode == PalmClockLogMode::Trace && !palmClockTraceFrameSelected(gameFrameIndex)) {
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
            const TransformDelta rawToTarget = (rawOk && targetOk) ? measureTransformDelta(*rawHandWorld, palmTargetWorld) : TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToLive = (rawOk && liveOk) ? measureTransformDelta(*rawHandWorld, livePalm.world) : TransformDelta{ -1.0f, -1.0f };
            const TransformDelta targetToLive = (targetOk && liveOk) ? measureTransformDelta(palmTargetWorld, livePalm.world) : TransformDelta{ -1.0f, -1.0f };

            const float rawDt = timing ? timing->rawDeltaSeconds : -1.0f;
            const float subDt = timing ? timing->substepDeltaSeconds : -1.0f;
            float driveDt = -1.0f;
            if (timing) {
                (void)havok_physics_timing::tryGetDriveDeltaSeconds(*timing, driveDt);
            }
            const float progress = timing ? timing->substepProgress : -1.0f;
            const std::uint32_t substepIndex = timing ? timing->substepIndex + 1 : 0;
            const std::uint32_t substepCount = timing ? timing->substepCount : 0;
            const char* physicsPhase = timing ? physicsStepPhaseName(timing->phase) : "game-frame";

            auto emit = [&]() {
                const char* rawState = rawOk ? "ok" : "none";
                const char* targetState = targetOk ? "ok" : "none";
                const char* liveState = liveOk ? "ok" : "none";
                const char* palmSource = liveOk ? body_frame::bodyFrameSourceCode(livePalm.source) : "none";
                const std::uint32_t palmMotion = liveOk ? livePalm.motionIndex : body_frame::kFreeMotionIndex;
                const RE::NiPoint3 rawPosition = rawOk ? rawHandWorld->translate : RE::NiPoint3{};
                const RE::NiPoint3 targetPosition = targetOk ? palmTargetWorld.translate : RE::NiPoint3{};
                const RE::NiPoint3 livePosition = liveOk ? livePalm.world.translate : RE::NiPoint3{};

                if (mode == PalmClockLogMode::Trace) {
                    ROCK_LOG_INFO(Hand,
                        "PALM_CLOCK stage={} hand={} frame={} holding={} raw={} target={} live={} body={} proxyBody={} gameDt={:.6f} physicsPhase={} rawDt={:.6f} subDt={:.6f} driveDt={:.6f} substep={}/{} progress={:.3f} rawToTarget={:.3f}gu/{:.3f}deg rawToLive={:.3f}gu/{:.3f}deg targetToLive={:.3f}gu/{:.3f}deg rawPos=({:.2f},{:.2f},{:.2f}) targetPos=({:.2f},{:.2f},{:.2f}) livePos=({:.2f},{:.2f},{:.2f}) liveSource={} liveMotion={}",
                        stage ? stage : "unknown",
                        hand.handName(),
                        gameFrameIndex,
                        hand.isHoldingAtomic() ? "yes" : "no",
                        rawState,
                        targetState,
                        liveState,
                        hand.getCollisionBodyId().value,
                        hand.getGrabAuthorityProxyBodyId().value,
                        gameDeltaSeconds,
                        physicsPhase,
                        rawDt,
                        subDt,
                        driveDt,
                        substepIndex,
                        substepCount,
                        progress,
                        rawToTarget.position,
                        rawToTarget.rotationDegrees,
                        rawToLive.position,
                        rawToLive.rotationDegrees,
                        targetToLive.position,
                        targetToLive.rotationDegrees,
                        rawPosition.x,
                        rawPosition.y,
                        rawPosition.z,
                        targetPosition.x,
                        targetPosition.y,
                        targetPosition.z,
                        livePosition.x,
                        livePosition.y,
                        livePosition.z,
                        palmSource,
                        palmMotion);
                } else {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "PALM_CLOCK stage={} hand={} frame={} holding={} raw={} target={} live={} body={} proxyBody={} gameDt={:.6f} physicsPhase={} rawDt={:.6f} subDt={:.6f} driveDt={:.6f} substep={}/{} progress={:.3f} rawToTarget={:.3f}gu/{:.3f}deg rawToLive={:.3f}gu/{:.3f}deg targetToLive={:.3f}gu/{:.3f}deg rawPos=({:.2f},{:.2f},{:.2f}) targetPos=({:.2f},{:.2f},{:.2f}) livePos=({:.2f},{:.2f},{:.2f}) liveSource={} liveMotion={}",
                        stage ? stage : "unknown",
                        hand.handName(),
                        gameFrameIndex,
                        hand.isHoldingAtomic() ? "yes" : "no",
                        rawState,
                        targetState,
                        liveState,
                        hand.getCollisionBodyId().value,
                        hand.getGrabAuthorityProxyBodyId().value,
                        gameDeltaSeconds,
                        physicsPhase,
                        rawDt,
                        subDt,
                        driveDt,
                        substepIndex,
                        substepCount,
                        progress,
                        rawToTarget.position,
                        rawToTarget.rotationDegrees,
                        rawToLive.position,
                        rawToLive.rotationDegrees,
                        targetToLive.position,
                        targetToLive.rotationDegrees,
                        rawPosition.x,
                        rawPosition.y,
                        rawPosition.z,
                        targetPosition.x,
                        targetPosition.y,
                        targetPosition.z,
                        livePosition.x,
                        livePosition.y,
                        livePosition.z,
                        palmSource,
                        palmMotion);
                }
            };

            emit();
        }

        inline float measureDirectionDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f);
            return std::acos(dot) * (180.0f / std::numbers::pi_v<float>);
        }

        inline bool startsWith(std::string_view value, std::string_view prefix)
        {
            return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
        }

        inline debug::SkeletonOverlayRole skeletonOverlayRoleForBone(std::string_view name)
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

        inline std::string_view trimView(std::string_view value)
        {
            while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
                value.remove_prefix(1);
            }
            while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {
                value.remove_suffix(1);
            }
            return value;
        }

        inline bool skeletonLogFilterMatches(std::string_view filter, std::string_view boneName)
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

        inline RE::TESObjectWEAP* currentEquippedWeaponForm()
        {
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            if (!weaponForm || weaponForm->formType != RE::ENUM_FORM_ID::kWEAP) {
                return nullptr;
            }

            return weaponForm->As<RE::TESObjectWEAP>();
        }

        inline RE::TBO_InstanceData* currentEquippedWeaponInstanceData(const RE::TESObjectWEAP* expectedWeapon)
        {
            auto* equipData = f4vr::getEquippedWeaponItem();
            if (!expectedWeapon || !equipData) {
                return nullptr;
            }

            auto* weaponForm = equipData->item.object;
            auto* equippedWeapon = weaponForm ? weaponForm->As<RE::TESObjectWEAP>() : nullptr;
            return equippedWeapon == expectedWeapon ? equipData->item.instanceData.get() : nullptr;
        }

        inline std::uint32_t currentEquippedWeaponFormId()
        {
            const auto* weapon = currentEquippedWeaponForm();
            return weapon ? weapon->formID : 0;
        }

        inline bool currentEquippedWeaponOccupiesHand()
        {
            return fo4vr_actor_state_policy::equippedWeaponOccupiesHand(
                currentEquippedWeaponFormId() != 0,
                f4vr::getNativeWeaponState(f4vr::getPlayer()));
        }

        inline void fillProviderTransform(const RE::NiTransform& source, ::rock::provider::RockProviderTransform& target)
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

        inline RE::NiTransform providerTransformToNi(const ::rock::provider::RockProviderTransform& source)
        {
            RE::NiTransform result{};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    result.rotate.entry[row][column] = source.rotate[static_cast<std::size_t>(row * 3 + column)];
                }
            }
            result.translate.x = source.translate[0];
            result.translate.y = source.translate[1];
            result.translate.z = source.translate[2];
            result.scale = source.scale;
            return result;
        }

        inline bool finiteNiTransform(const RE::NiTransform& transform)
        {
            return left_carry_readiness::finiteTransform(transform);
        }

        inline std::string_view providerFixedStringView(const char* value, std::size_t capacity)
        {
            if (!value) {
                return {};
            }
            for (std::size_t i = 0; i < capacity; ++i) {
                if (value[i] == '\0') {
                    return std::string_view(value, i);
                }
            }
            return std::string_view(value, capacity);
        }

        inline bool nodeNameEquals(RE::NiAVObject* node, std::string_view name)
        {
            if (!node || name.empty()) {
                return false;
            }
            const char* nodeName = node->name.c_str();
            return nodeName && std::string_view(nodeName) == name;
        }

        inline RE::NiAVObject* findWeaponNodeBySourceName(RE::NiAVObject* root, std::string_view sourceName, int maxDepth = 32)
        {
            if (!root || sourceName.empty() || maxDepth < 0) {
                return nullptr;
            }
            if (nodeNameEquals(root, sourceName)) {
                return root;
            }
            auto* node = root->IsNode();
            if (!node) {
                return nullptr;
            }
            auto& children = node->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < children.size(); ++i) {
                if (auto* found = findWeaponNodeBySourceName(children[i].get(), sourceName, maxDepth - 1)) {
                    return found;
                }
            }
            return nullptr;
        }

        inline std::uint32_t providerColliderFlags(RE::hknpWorld* world,
            std::uint32_t bodyId, bool bodiesCurrent, bool lifecycleAllowed)
        {
            std::uint32_t filter = 0;
            const bool known = bodiesCurrent && world &&
                body_collision::tryReadFilterInfo(world, RE::hknpBodyId{bodyId}, filter);
            return provider_state_policy::colliderFlags(lifecycleAllowed, known,
                (filter & collision_suppression_registry::kSuppressionNoCollideBit) != 0);
        }

        inline void copyProviderString(char* target, std::size_t targetSize, const std::string& source)
        {
            if (!target || targetSize == 0) {
                return;
            }

            std::snprintf(target, targetSize, "%s", source.c_str());
            target[targetSize - 1] = '\0';
        }

        inline void copyProviderString(char* target, std::size_t targetSize, const char* source)
        {
            if (!target || targetSize == 0) {
                return;
            }
            std::snprintf(target, targetSize, "%s", source ? source : "");
            target[targetSize - 1] = '\0';
        }

        inline ::rock::provider::RockProviderWeaponPartTargetQueryV1 makeProviderWeaponPartTargetQuery(
            const WeaponInteractionContact& contact,
            const WeaponCollision& weaponCollision)
        {
            ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
            query.weaponGenerationKey = contact.weaponGenerationKey;
            query.bodyId = contact.bodyId;
            query.partKind = static_cast<std::uint32_t>(contact.partKind);
            query.reloadRole = static_cast<std::uint32_t>(contact.reloadRole);
            query.supportRole = static_cast<std::uint32_t>(contact.supportGripRole);
            query.socketRole = static_cast<std::uint32_t>(contact.socketRole);
            query.actionRole = static_cast<std::uint32_t>(contact.actionRole);
            query.sourceRoot = reinterpret_cast<std::uintptr_t>(contact.sourceRoot);
            const auto evidence = weaponCollision.getProfileEvidenceDescriptors();
            const auto* descriptor = evidence.find(contact.bodyId);
            if (descriptor &&
                descriptor->weaponGenerationKey == contact.weaponGenerationKey) {
                query.sourceRoot = descriptor->sourceRootAddress;
                copyProviderString(query.sourceName, sizeof(query.sourceName), descriptor->sourceName);
            }
            return query;
        }

        inline WeaponProviderPartAuthority makeWeaponProviderPartAuthority(
            const ::rock::provider::RockProviderWeaponPartTargetQueryV1& query,
            const ::rock::provider::RockProviderWeaponPartTargetResolutionV1& resolution)
        {
            WeaponProviderPartAuthority authority{};
            authority.active = resolution.matched != 0;
            authority.ownerToken = resolution.ownerToken;
            authority.weaponGenerationKey = query.weaponGenerationKey;
            authority.bodyId = query.bodyId;
            authority.sourceRoot = query.sourceRoot;
            authority.partKind = query.partKind;
            authority.reloadRole = query.reloadRole;
            authority.supportRole = query.supportRole;
            authority.socketRole = query.socketRole;
            authority.actionRole = query.actionRole;
            authority.groupId = resolution.groupId;
            authority.grabMode = static_cast<std::uint32_t>(resolution.grabMode);
            static_assert(WeaponProviderPartAuthority{}.sourceName.size() == ::rock::provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME);
            std::memcpy(authority.sourceName.data(), query.sourceName, authority.sourceName.size());
            authority.sourceName[authority.sourceName.size() - 1] = '\0';
            return authority;
        }

        inline ::rock::provider::RockProviderPoint3 makeProviderPoint(const WeaponEvidencePoint3& point)
        {
            return ::rock::provider::RockProviderPoint3{ .x = point.x, .y = point.y, .z = point.z };
        }

        inline ::rock::provider::RockProviderPoint3 makeProviderPoint(const RE::NiPoint3& point)
        {
            return ::rock::provider::RockProviderPoint3{ .x = point.x, .y = point.y, .z = point.z };
        }

        inline ::rock::provider::RockProviderBodyContactTargetKind providerBodyContactTargetKind(contact_pipeline_policy::ContactEndpointKind kind)
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

        inline const char* weaponDiagnosticNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }

            const char* name = node->name.c_str();
            return name ? name : "";
        }

        inline const char* pushAssistSkipReasonName(push_assist::PushAssistSkipReason reason)
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

        inline WeaponInteractionDebugInfo makeWeaponInteractionDebugInfo(
            const WeaponCollision& weaponCollision,
            RE::NiNode* weaponNode,
            const WeaponInteractionContact& contact)
        {
            WeaponInteractionDebugInfo info{};
            info.weaponNodeName = weaponDiagnosticNodeName(weaponNode);

            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            if (weaponForm) {
                info.weaponFormId = weaponForm->formID;
                const auto fullName = RE::TESFullName::GetFullName(*weaponForm);
                if (!fullName.empty()) {
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

        inline RE::EquippedWeaponData* getValidatedEquippedWeaponData()
        {
            auto* equipWeaponData = f4vr::getEquippedWeaponData();
            if (!equipWeaponData) {
                return nullptr;
            }

            const auto vtable =
                *reinterpret_cast<const std::uintptr_t*>(
                    equipWeaponData);
            if (vtable !=
                f4vr::EquippedWeaponData_vtable.address()) {
                return nullptr;
            }
            return equipWeaponData;
        }

        inline RE::NiAVObject* getEquippedProjectileNode()
        {
            /*
             * FO4VR 1.2.72 initializes EquippedWeaponData::fireNode (+0x30)
             * during equip through its native ProjectileNode/P-ProjectileNode
             * resolver and refreshes it with equipped-item updates. The
             * separate muzzleFlash pointer (+0x28) is presentation state and
             * may remain null until the weapon has fired, so provider data
             * must never depend on it.
             */
            auto* equipWeaponData =
                getValidatedEquippedWeaponData();
            return equipWeaponData ?
                equipWeaponData->fireNode :
                nullptr;
        }

        inline f4vr::MuzzleFlash* getEquippedMuzzleFlashNodes()
        {
            /*
             * ROCK is the final weapon visual owner during mesh/hand authority.
             * Any ROCK weapon write after the normal first-person weapon update
             * must re-own the fire node from the current projectile node so the
             * muzzle origin remains at the barrel tip.
             */
            const auto equipWeaponData =
                getValidatedEquippedWeaponData();
            if (!equipWeaponData) {
                return nullptr;
            }

            const auto muzzle = reinterpret_cast<f4vr::MuzzleFlash*>(equipWeaponData->muzzleFlash);
            if (!muzzle || !muzzle->fireNode || !muzzle->projectileNode) {
                return nullptr;
            }

            return muzzle;
        }

        inline void applyFinalWeaponMuzzleAuthority()
        {
            auto* muzzle = getEquippedMuzzleFlashNodes();
            if (!muzzle) {
                return;
            }

            muzzle->fireNode->local = weapon_muzzle_authority_math::fireNodeLocalFromProjectileWorld(muzzle->projectileNode->world);
            f4vr::updateTransformsDown(muzzle->fireNode, true);
        }

        inline RE::NiNode* resolveEquippedWeaponInteractionNodeDirect()
        {
            auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
            return firstPersonSkeleton ? f4vr::findNode(firstPersonSkeleton, "Weapon") : nullptr;
        }

        inline RE::NiNode* resolveEquippedWeaponInteractionNode()
        {
            /*
             * Weapon interaction uses the same first-person weapon root for all
             * equipped weapons. Generated collision scans every known package
             * candidate internally, so this handoff should not branch by weapon
             * type or create a separate melee-owned update path.
             */
            if (!runtime_state::currentFrame().weaponDrawn || !currentEquippedWeaponOccupiesHand()) {
                return nullptr;
            }

            return resolveEquippedWeaponInteractionNodeDirect();
        }
    }

    // Implementation TUs resolve the helper names unqualified, exactly as
    // they did inside the original anonymous namespace.
    using namespace physics_interaction_internal;

    struct PhysicsInteraction::EquippedWeaponFrameResult
    {
        bool rightHandWeaponAuthorityActive = false;
        bool leftSupportGripActive = false;
        bool rightPartGripActive = false;
    };
}
