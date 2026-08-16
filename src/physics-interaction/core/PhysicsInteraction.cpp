#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"

#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"
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
#include "physics-interaction/debug/DebugOverlayPolicy.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/CustomOGA.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
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
#include "physics-interaction/weapon/PipboyEquipRuntime.h"
#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
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
#include "rock_support/Fo4VrActorStatePolicy.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"
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
        constexpr std::uint16_t
            kEquippedWeaponHandAssignmentMaximumResolveFrames = 180;
        constexpr std::array<std::string_view, 5> kWeaponCollisionWorkbenchExitMenuNames{
            "ExamineMenu",
            "PowerArmorModMenu",
            "RobotModMenu",
            "Crafting Menu",
            "CraftingMenu",
        };
        constexpr float kNearbyCarCollisionRadiusGameUnits = 4096.0f;

        std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkRegistered{ false };
        std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged{ false };

        [[nodiscard]] bool nativeReloadHandAuthorityActive()
        {
            const auto animationAuthorityFlags =
                provider::currentNativeAnimationAuthorityFlagsV1();
            const bool providerOwnsArmsOrHands =
                (animationAuthorityFlags &
                    (authored_weapon_grip_capture_policy::kArms |
                        authored_weapon_grip_capture_policy::kHands)) != 0;
            return providerOwnsArmsOrHands ||
                   fo4vr_actor_state_policy::isNativeReloading(
                       f4vr::getNativeGunState(f4vr::getPlayer()));
        }

        [[nodiscard]] bool isWeaponCollisionWorkbenchExitMenu(const RE::BSFixedString& menuName)
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

        WeaponCollisionWorkbenchExitMenuSink s_weaponCollisionWorkbenchExitMenuSink;

        bool tryReadNativeScopeRequestState(bool& outActive)
        {
            using GetScopeRequestState = bool (*)(const void*);
            static REL::Relocation<GetScopeRequestState> getScopeRequestState{ REL::Offset(offsets::kFunc_NativeScopeRequestStateGet) };
            static REL::Relocation<std::uintptr_t> rendererState{ REL::Offset(offsets::kData_NativeScopeRendererState) };
            if (!getScopeRequestState.address() || !rendererState.address()) {
                return false;
            }
            outActive = getScopeRequestState(reinterpret_cast<const void*>(rendererState.address()));
            return true;
        }

        bool ensureWeaponCollisionWorkbenchExitMenuSinkRegistered()
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

        void clearEquippedWeaponFiringGripInputState()
        {
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
            input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
        }

        [[nodiscard]] float pointLength(const RE::NiPoint3& value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
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

        /*
         * Equipped-weapon stash reuses the loose-object back volume so the
         * player learns one gesture. Body-zone collider/contact evidence is
         * disabled for this variant: the equipped weapon has no held-body
         * contact identity, so the HMD-relative back volume (forced on) is the
         * gesture authority for hand-carried weapons.
         */
        shoulder_stash::DetectorConfig makeEquippedWeaponStashDetectorConfig(bool enabled)
        {
            shoulder_stash::DetectorConfig config = makeShoulderStashDetectorConfig();
            config.enabled = enabled;
            config.useBodyZoneColliders = false;
            config.useHmdBackVolume = true;
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

        mouth_consume::Probe makeMouthConsumeObjectProbe(RE::hknpWorld* world, const Hand& hand, const HandFrameInput& handInput)
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

        mouth_consume::Probe makeMouthConsumeHandProbe(const HandFrameInput& handInput)
        {
            mouth_consume::Probe probe{};
            probe.pointGame = handInput.grabAnchorWorld;
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

            const auto packedInvMass = static_cast<std::int16_t>(motion->packedInverseInertia[3]);
            if (packedInvMass == 0) {
                return 0.0f;
            }
            return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
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
        ContactEventSubscriptionBridge s_manifoldProcessedEventBridge;

        struct GrabButtonState
        {
            bool held{ false };
            bool pressed{ false };
            bool released{ false };
            bool syntheticPressed{ false };
        };

        GrabButtonState readGrabButtonState(bool isLeft, int buttonId)
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

        bool readGrabButtonHeld(bool isLeft, int buttonId)
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

        bool readGrabButtonPressedEdge(bool isLeft, int buttonId)
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

        bool readHeldWeaponEquipTriggerPressedEdge(bool isLeft)
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

        enum class PalmClockLogMode : std::uint8_t
        {
            Disabled,
            Sampled,
            Trace,
        };

        const char* physicsStepPhaseName(havok_physics_timing::PhysicsStepPhase phase)
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

        PalmClockLogMode palmClockLogMode()
        {
            if (g_rockConfig.rockDebugGrabTimelineTrace) {
                return PalmClockLogMode::Trace;
            }
            if (g_rockConfig.rockDebugGrabFrameLogging || g_rockConfig.rockDebugVerboseLogging) {
                return PalmClockLogMode::Sampled;
            }
            return PalmClockLogMode::Disabled;
        }

        bool palmClockTraceFrameSelected(std::uint64_t gameFrameIndex)
        {
            const auto interval = static_cast<std::uint64_t>((std::max)(1, g_rockConfig.rockDebugGrabTimelineTraceIntervalFrames));
            return gameFrameIndex <= 3 || (gameFrameIndex % interval) == 0;
        }

        /*
         * Palm clock diagnostics compare the game-frame queued skeleton target to
         * the live palm body at each authority boundary. This keeps rate-mismatch
         * evidence in one log row without changing grab authority.
         */
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
            const float driveDt = timing ? havok_physics_timing::driveDeltaSeconds(*timing) : -1.0f;
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

        float measureDirectionDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f);
            return std::acos(dot) * (180.0f / std::numbers::pi_v<float>);
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

        RE::TBO_InstanceData* currentEquippedWeaponInstanceData(const RE::TESObjectWEAP* expectedWeapon)
        {
            auto* equipData = f4vr::getEquippedWeaponItem();
            if (!expectedWeapon || !equipData) {
                return nullptr;
            }

            auto* weaponForm = equipData->item.object;
            auto* equippedWeapon = weaponForm ? weaponForm->As<RE::TESObjectWEAP>() : nullptr;
            return equippedWeapon == expectedWeapon ? equipData->item.instanceData.get() : nullptr;
        }

        std::uint32_t currentEquippedWeaponFormId()
        {
            const auto* weapon = currentEquippedWeaponForm();
            return weapon ? weapon->formID : 0;
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

        RE::NiTransform providerTransformToNi(const ::rock::provider::RockProviderTransform& source)
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

        bool approximatelySameWeaponLocalOffset(
            const RE::NiTransform& live,
            const RE::NiTransform& expected)
        {
            constexpr float kMaximumTranslationError = 0.05f;
            constexpr float kMaximumRotationElementError = 0.001f;
            constexpr float kMaximumScaleError = 0.001f;
            if (!finiteNiTransform(live) || !finiteNiTransform(expected)) {
                return false;
            }

            const float dx = live.translate.x - expected.translate.x;
            const float dy = live.translate.y - expected.translate.y;
            const float dz = live.translate.z - expected.translate.z;
            if (dx * dx + dy * dy + dz * dz > kMaximumTranslationError * kMaximumTranslationError ||
                std::abs(live.scale - expected.scale) > kMaximumScaleError) {
                return false;
            }

            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (std::abs(live.rotate.entry[row][column] - expected.rotate.entry[row][column]) >
                        kMaximumRotationElementError) {
                        return false;
                    }
                }
            }
            return true;
        }

        std::string_view providerFixedStringView(const char* value, std::size_t capacity)
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

        bool nodeNameEquals(RE::NiAVObject* node, std::string_view name)
        {
            if (!node || name.empty()) {
                return false;
            }
            const char* nodeName = node->name.c_str();
            return nodeName && std::string_view(nodeName) == name;
        }

        RE::NiAVObject* findWeaponNodeBySourceName(RE::NiAVObject* root, std::string_view sourceName, int maxDepth = 32)
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

        void copyProviderString(char* target, std::size_t targetSize, const char* source)
        {
            if (!target || targetSize == 0) {
                return;
            }
            std::snprintf(target, targetSize, "%s", source ? source : "");
            target[targetSize - 1] = '\0';
        }

        ::rock::provider::RockProviderWeaponPartTargetQueryV1 makeProviderWeaponPartTargetQuery(
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
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* sourceNode = nullptr;
            if (weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(contact.bodyId, descriptor, sourceNode) &&
                descriptor.weaponGenerationKey == contact.weaponGenerationKey) {
                query.sourceRoot = descriptor.sourceRootAddress;
                copyProviderString(query.sourceName, sizeof(query.sourceName), descriptor.sourceName);
            }
            return query;
        }

        WeaponProviderPartAuthority makeWeaponProviderPartAuthority(
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

        RE::EquippedWeaponData* getValidatedEquippedWeaponData()
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

        RE::NiAVObject* getEquippedProjectileNode()
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

        f4vr::MuzzleFlash* getEquippedMuzzleFlashNodes()
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

        void applyFinalWeaponMuzzleAuthority()
        {
            auto* muzzle = getEquippedMuzzleFlashNodes();
            if (!muzzle) {
                return;
            }

            muzzle->fireNode->local = weapon_muzzle_authority_math::fireNodeLocalFromProjectileWorld(muzzle->projectileNode->world);
            f4vr::updateTransformsDown(muzzle->fireNode, true);
        }

        RE::NiNode* resolveEquippedWeaponInteractionNodeDirect()
        {
            auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
            return firstPersonSkeleton ? f4vr::findNode(firstPersonSkeleton, "Weapon") : nullptr;
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

            return resolveEquippedWeaponInteractionNodeDirect();
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
            &PhysicsInteraction::onCustomGrabAuthorityAfterCharacterMovement,
            &PhysicsInteraction::onCustomGrabAuthorityAfterSolve,
            this);
        auto* generatedBodyCallbackGate = &_generatedBodyStepDrive.callbackGate();
        _rightHand.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _leftHand.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _touchGrabRuntime.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _touchGrabRuntime.setDynamicHandCollisionRuntime(&_dynamicHandCollision);
        _bodyBoneColliders.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _dynamicHandCollision.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _weaponCollision.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _dynamicWeaponCollision.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _twoHandedGrip.setWeaponVisualIntentObserver(
            &_dynamicWeaponCollision,
            &DynamicWeaponCollisionRuntime::observeWeaponVisualIntent);
        clearLooseGrenadeImpactWatches();

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

        _authoredPrimaryFiringGrip.reset("physics-destroyed", _twoHandedGrip);

        if (_initialized) {
            shutdown();
        }
        ROCK_LOG_INFO(Init, "ROCK Physics Module — destroyed");
    }

    void PhysicsInteraction::requestWeaponCollisionRebuildAfterWorkbenchExit(const char* sourceMenuName)
    {
        if (!_initialized.load(std::memory_order_acquire)) {
            return;
        }

        _weaponCollision.requestWorkbenchExitRebuild();
        _equippedWeaponTransition.requestCurrentWeaponReconcile(
            EquippedWeaponTransitionCoordinator::Source::WorkbenchExit);
        ROCK_LOG_DEBUG(Weapon,
            "Weapon collision workbench-exit rebuild gate armed by {} close",
            sourceMenuName ? sourceMenuName : "<unknown>");
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
        _authoredPrimaryFiringGrip.reset("skeleton-lifecycle", _twoHandedGrip);
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
        _authoredPrimaryFiringGrip.reset("provider-lifecycle", _twoHandedGrip);
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
        _collisionGenerationAtomic.fetch_add(1, std::memory_order_acq_rel);
        refreshGeneratedBodyContactRegistry();
    }

    void PhysicsInteraction::markGeneratedBodiesInvalidated()
    {
        const auto collisionGeneration =
            _collisionGenerationAtomic.fetch_add(
                1,
                std::memory_order_acq_rel) +
            1;
        auto* currentBhkWorld = getPlayerBhkWorld();
        auto* currentHknpWorld =
            currentBhkWorld ?
            getHknpWorld(currentBhkWorld) :
            nullptr;
        /*
         * Touch constraints reference ROCK's generated hand bodies. Retire
         * them while the matching world is still authoritative; if the world
         * has already changed, the runtime abandons stale Havok IDs without
         * dereferencing them.
         */
        _touchGrabRuntime.releaseAll(
            currentBhkWorld,
            currentHknpWorld,
            provider::RockProviderTouchGrabReleaseReasonV1::
                GenerationChanged,
            collisionGeneration);
        // Close callback entry and drain any native step already traversing
        // ROCK-owned body banks before clearing registry or wrapper state.
        _generatedBodyStepDrive.reset();
        clearGeneratedBodyContactRegistry();
        const bool generatedWorldStillLive =
            currentBhkWorld &&
            currentBhkWorld == _generatedBodiesBhkWorld &&
            currentHknpWorld &&
            currentHknpWorld == _generatedBodiesHknpWorld;
        if (generatedWorldStillLive) {
            _dynamicHandCollision.retireAll(_generatedBodiesBhkWorld);
            _dynamicWeaponCollision.retireAll(_generatedBodiesBhkWorld);
        } else {
            _dynamicHandCollision.reset();
            _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
        }
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
        _completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _equippedWeaponDropMomentumHandoffs = {};
        _shoulderStashStates = {};
        _mouthConsumeStates = {};
        _feedbackHaptics.reset();
    }

    void PhysicsInteraction::clearGeneratedBodyContactRegistry()
    {
        _generatedBodyContactRegistry.clear();
    }

    void PhysicsInteraction::refreshGeneratedBodyContactRegistry()
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GeneratedBodyContactRegistry);

        using generated_body_contact_registry::Entry;
        using generated_body_contact_registry::GeneratedBodyKind;
        using generated_body_contact_registry::kFlagPowerArmor;
        using generated_body_contact_registry::kFlagPrimaryAnchor;
        using generated_body_contact_registry::kFlagSampledVelocity;

        std::array<Entry, kGeneratedBodyContactRegistryCapacity> entries{};
        std::size_t entryCount = 0;

        auto addEntry = [&](const Entry& entry) {
            if (entryCount < entries.size()) {
                entries[entryCount++] = entry;
            }
        };

        auto addHandEntries = [&](const Hand& hand, bool isLeft) {
            const std::uint32_t count = (std::min)(hand.getHandColliderBodyCount(), static_cast<std::uint32_t>(hand_collider_semantics::kHandColliderBodyCountPerHand));
            for (std::uint32_t i = 0; i < count; ++i) {
                const std::uint32_t bodyId = hand.getHandColliderBodyIdAtomic(i);
                HandColliderBodyMetadata metadata{};
                if (!hand.tryGetHandColliderMetadata(bodyId, metadata) || !metadata.valid) {
                    continue;
                }

                Entry entry{};
                entry.bodyId = metadata.bodyId;
                entry.kind = isLeft ? GeneratedBodyKind::LeftHand : GeneratedBodyKind::RightHand;
                entry.role = static_cast<std::uint32_t>(metadata.role);
                entry.partKind = static_cast<std::uint32_t>(metadata.finger);
                entry.subRole = static_cast<std::uint32_t>(metadata.segment);
                if (metadata.primaryPalmAnchor) {
                    entry.flags |= kFlagPrimaryAnchor;
                }
                if (metadata.hasSampledLinearVelocityHavok &&
                    std::isfinite(metadata.sampledLinearVelocityHavok[0]) &&
                    std::isfinite(metadata.sampledLinearVelocityHavok[1]) &&
                    std::isfinite(metadata.sampledLinearVelocityHavok[2])) {
                    entry.flags |= kFlagSampledVelocity;
                    entry.sampledVelocityHavokX = metadata.sampledLinearVelocityHavok[0];
                    entry.sampledVelocityHavokY = metadata.sampledLinearVelocityHavok[1];
                    entry.sampledVelocityHavokZ = metadata.sampledLinearVelocityHavok[2];
                }
                addEntry(entry);
            }
        };

        addHandEntries(_rightHand, false);
        addHandEntries(_leftHand, true);

        const auto weaponSnapshot = _weaponCollision.getWeaponBodySnapshotAtomic();
        for (std::uint32_t i = 0; i < weaponSnapshot.count && i < MAX_WEAPON_COLLISION_BODIES; ++i) {
            WeaponInteractionContact contact{};
            if (!_weaponCollision.tryGetWeaponContactAtomic(weaponSnapshot.bodyIds[i], contact) || !contact.valid) {
                continue;
            }

            Entry entry{};
            entry.bodyId = contact.bodyId;
            entry.kind = GeneratedBodyKind::Weapon;
            entry.role = static_cast<std::uint32_t>(contact.reloadRole);
            entry.partKind = static_cast<std::uint32_t>(contact.partKind);
            entry.subRole = static_cast<std::uint32_t>(contact.supportGripRole);
            entry.socketRole = static_cast<std::uint32_t>(contact.socketRole);
            entry.actionRole = static_cast<std::uint32_t>(contact.actionRole);
            entry.gripPose = static_cast<std::uint32_t>(contact.fallbackGripPose);
            entry.generationKey = contact.weaponGenerationKey;

            float sampledVelocityHavok[4]{};
            if (_weaponCollision.tryGetWeaponBodySampledVelocityAtomic(contact.bodyId, sampledVelocityHavok) &&
                std::isfinite(sampledVelocityHavok[0]) &&
                std::isfinite(sampledVelocityHavok[1]) &&
                std::isfinite(sampledVelocityHavok[2])) {
                entry.flags |= kFlagSampledVelocity;
                entry.sampledVelocityHavokX = sampledVelocityHavok[0];
                entry.sampledVelocityHavokY = sampledVelocityHavok[1];
                entry.sampledVelocityHavokZ = sampledVelocityHavok[2];
            }
            addEntry(entry);
        }

        const std::uint32_t bodyCount = (std::min)(_bodyBoneColliders.getBodyCount(), static_cast<std::uint32_t>(kBodyBoneColliderBodyCount));
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const std::uint32_t bodyId = _bodyBoneColliders.getBodyIdAtomic(i);
            BodyBoneColliderMetadata metadata{};
            if (!_bodyBoneColliders.tryGetBodyMetadataAtomic(bodyId, metadata) || !metadata.valid) {
                continue;
            }

            Entry entry{};
            entry.bodyId = metadata.bodyId;
            entry.kind = GeneratedBodyKind::Body;
            entry.role = static_cast<std::uint32_t>(metadata.role);
            entry.zone = static_cast<std::uint32_t>(metadata.zone);
            entry.side = static_cast<std::uint32_t>(metadata.side);
            entry.descriptorIndex = metadata.descriptorIndex;
            entry.lengthGameUnits = metadata.lengthGameUnits;
            entry.radiusGameUnits = metadata.radiusGameUnits;
            if (metadata.inPowerArmor) {
                entry.flags |= kFlagPowerArmor;
            }
            addEntry(entry);
        }

        _generatedBodyContactRegistry.publish(entries.data(), entryCount);
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

        _dynamicWeaponCollision.retireAll(bhk);
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

        _rightHand.updateCollisionTransform(hknp, getInteractionHandTransform(false), 0.011f);
        _leftHand.updateCollisionTransform(hknp, getInteractionHandTransform(true), 0.011f);
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

    HandFrame PhysicsInteraction::getInteractionHandFrame(bool isLeft)
    {
        const bool cacheReady = _handBoneCache.isReady();
        auto* const driverNode =
            isLeft ?
            f4vr::getLeftHandNode() :
            f4vr::getRightHandNode();
        const std::size_t handIndex = isLeft ? 0u : 1u;
        if (_currentPreFrikSchedulerSequence == 0 ||
            _persistentFrikHandInputIsolationSequence[handIndex] !=
                _currentPreFrikSchedulerSequence) {
            _persistentFrikHandInputIsolationSequence[handIndex] =
                _currentPreFrikSchedulerSequence;
            const bool sampledPersistentWorldAuthority =
                frik_visual_authority::hasPublishedExternalHandWorldTransform(
                    frik_visual_authority::handFromBool(isLeft));
            if (_persistentFrikHandInputIsolationActive[handIndex] !=
                sampledPersistentWorldAuthority) {
                _persistentFrikHandInputIsolationActive[handIndex] =
                    sampledPersistentWorldAuthority;
                ROCK_LOG_INFO(Hand,
                    "{} collision-isolated controller input {} for persistent FRIK V2 hand authority",
                    isLeft ? "Left" : "Right",
                    sampledPersistentWorldAuthority ? "engaged" : "released");
            }
        }
        const bool persistentWorldAuthorityPublished =
            _persistentFrikHandInputIsolationActive[handIndex];
        const HandFrame frame = _handFrameResolver.resolve(
            isLeft,
            cacheReady,
            cacheReady ?
                _handBoneCache.getWorldTransform(isLeft) :
                RE::NiTransform{},
            cacheReady ? _handBoneCache.getSkeleton() : nullptr,
            cacheReady ? _handBoneCache.getBoneTree() : nullptr,
            persistentWorldAuthorityPublished,
            driverNode != nullptr,
            driverNode ? driverNode->world : RE::NiTransform{});
        const auto physicalHand =
            frik_visual_authority::handFromBool(isLeft);
        const bool handWorldPublicationReady =
            _handFrameResolver.hasControllerReconstructionCalibration(
                isLeft,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree()) &&
            (!persistentWorldAuthorityPublished || frame.valid);
        const bool handWorldPublicationReadinessChanged =
            frik_visual_authority::isExternalHandWorldPublicationReady(
                physicalHand) != handWorldPublicationReady;
        frik_visual_authority::setExternalHandWorldPublicationReady(
            physicalHand,
            handWorldPublicationReady);
        if (handWorldPublicationReadinessChanged) {
            ROCK_LOG_INFO(Hand,
                "{} persistent FRIK V2 hand publication {} after controller calibration",
                isLeft ? "Left" : "Right",
                handWorldPublicationReady ? "enabled" : "suspended");
        }
        if (persistentWorldAuthorityPublished && !frame.valid) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} persistent FRIK V2 hand authority failed closed because controller reconstruction is unavailable cacheReady={} driverReady={}",
                isLeft ? "Left" : "Right",
                cacheReady ? "yes" : "no",
                driverNode ? "yes" : "no");
        }
        return frame;
    }

    RE::NiTransform PhysicsInteraction::getInteractionHandTransform(bool isLeft)
    {
        const auto frame = getInteractionHandFrame(isLeft);
        if (frame.valid) {
            return frame.transform;
        }

        return RE::NiTransform();
    }

    void PhysicsInteraction::refreshExternalHandWorldTransformsBeforeFrik(
        const std::uint64_t schedulerSequence)
    {
        _currentPreFrikSchedulerSequence = schedulerSequence;
        if (!_initialized.load(std::memory_order_acquire)) {
            return;
        }

        const auto captureWand = [](RE::NiNode* node,
                                    RE::NiTransform& outWorld) {
            outWorld = {};
            if (!node || !finiteNiTransform(node->world)) {
                return false;
            }
            outWorld = node->world;
            return true;
        };

        RE::NiTransform rightWandWorld{};
        RE::NiTransform leftWandWorld{};
        const bool rightWandValid = captureWand(
            f4vr::getRightHandNode(),
            rightWandWorld);
        const bool leftWandValid = captureWand(
            f4vr::getLeftHandNode(),
            leftWandWorld);

        RE::NiTransform rightRawHandWorld{};
        RE::NiTransform leftRawHandWorld{};
        const bool rightRawHandValid =
            rightWandValid &&
            _handFrameResolver.tryReconstructCalibratedHand(
                false,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree(),
                rightWandWorld,
                rightRawHandWorld);
        const bool leftRawHandValid =
            leftWandValid &&
            _handFrameResolver.tryReconstructCalibratedHand(
                true,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree(),
                leftWandWorld,
                leftRawHandWorld);

        _rightHand.refreshGrabVisualAuthorityBeforeFrik(
            schedulerSequence,
            rightRawHandValid,
            rightRawHandWorld);
        _leftHand.refreshGrabVisualAuthorityBeforeFrik(
            schedulerSequence,
            leftRawHandValid,
            leftRawHandWorld);
        _equippedWeaponTransition.refreshHandVisualAuthorityBeforeFrik(
            schedulerSequence);

        _dynamicHandCollision.refreshContactVisualAuthorityBeforeFrik(
            schedulerSequence,
            rightRawHandValid,
            rightRawHandWorld,
            leftRawHandValid,
            leftRawHandWorld,
            g_rockConfig.
                rockHandCollisionDynamicDivergenceTeleportGameUnits);

        // Deferred FRIK hand requests must be transported by the physical
        // controller-derived hand, not by weapon-offset nodes. Hunting/combat
        // rifle firing animations rotate those weapon nodes toward their
        // native angled pose; feeding that rotation back here makes ROCK's
        // authored weapon alignment latch to the native angle.
        const EquippedWeaponScopeHandDriverFrame leftWeaponHandDriver =
            { leftRawHandValid, leftRawHandWorld };
        const EquippedWeaponScopeHandDriverFrame rightWeaponHandDriver =
            { rightRawHandValid, rightRawHandWorld };

        const bool firingHandIsLeft = _twoHandedGrip.isFiringHandLeft();
        _twoHandedGrip.setNativeReloadHandAuthorityActive(
            nativeReloadHandAuthorityActive());
        _twoHandedGrip.refreshRetainedHandVisualAuthoritiesBeforeFrik(
            leftWeaponHandDriver,
            rightWeaponHandDriver,
            _weaponCollision.getCurrentWeaponGenerationKey(),
            firingHandIsLeft,
            schedulerSequence);
        _twoHandedGrip.refreshWeaponCollisionHandAuthorityBeforeFrik(
            leftWeaponHandDriver,
            rightWeaponHandDriver,
            _weaponCollision.getCurrentWeaponGenerationKey(),
            firingHandIsLeft,
            schedulerSequence);
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
            const auto localPalmPosition = computeGrabLegacyPalmPivotAWorldFromHandBasis(localTransform, isLeft);
            const auto apiPalmPosition = computeGrabLegacyPalmPivotAWorldFromHandBasis(apiTransform, isLeft);
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

    void PhysicsInteraction::logColliderClockTrace(
        const PhysicsFrameContext& frame,
        const RE::NiNode* weaponNode)
    {
        constexpr std::uint32_t kTraceFramesPerEpisode = 360;

        const bool colliderClockDebugActive =
            g_rockConfig.rockDebugShowColliders ||
            g_rockConfig.rockDebugShowTargetColliders ||
            g_rockConfig.rockDebugDrawHandColliders ||
            g_rockConfig.rockDebugDrawHandBoneColliders ||
            g_rockConfig.rockDebugDrawDynamicHandColliders ||
            g_rockConfig.rockDebugDrawWeaponColliders ||
            g_rockConfig.rockDebugDrawDynamicWeaponColliders;
        if (!colliderClockDebugActive) {
            _colliderClockLastLoggedFrame = 0;
            _colliderClockHasLoggedFrame = false;
            _colliderClockSession = 0;
            _colliderClockFramesRemaining = 0;
            _colliderClockPreviousPlayerMoving = false;
            _colliderClockPreviousContactActive = false;
            _colliderClockPreviousGrabActive = false;
            _grabLocomotionClockStates = {};
            return;
        }

        DynamicWeaponCollisionRuntime::DebugSnapshot weapon{};
        const bool weaponValid = _dynamicWeaponCollision.getDebugSnapshot(weapon);
        dynamic_hand_collision_telemetry::Snapshot hands{};
        const bool handsValid = _dynamicHandCollision.getTelemetrySnapshot(hands);
        const bool contactActive =
            (weaponValid && weapon.contactActive) ||
            (handsValid && (hands.hands[0].anyContact || hands.hands[1].anyContact));
        const auto& runtime = runtime_state::currentFrame();
        const bool playerMoving = runtime.playerSpace.valid && runtime.playerSpace.moving;
        const bool movementStarted =
            playerMoving && !_colliderClockPreviousPlayerMoving;
        const bool contactStarted =
            contactActive && !_colliderClockPreviousContactActive;
        const bool grabActive = _rightHand.isHolding() || _leftHand.isHolding();
        const bool grabStarted = grabActive && !_colliderClockPreviousGrabActive;
        _colliderClockPreviousPlayerMoving = playerMoving;
        _colliderClockPreviousContactActive = contactActive;
        _colliderClockPreviousGrabActive = grabActive;

        if (movementStarted || contactStarted || grabStarted) {
            ++_colliderClockSession;
            _colliderClockFramesRemaining = kTraceFramesPerEpisode;
            const char* trigger = movementStarted ? "movement" : (contactStarted ? "contact" : "grab");
            ROCK_LOG_INFO(
                Physics,
                "COLLIDER_CLOCK begin session={} trigger={} budgetFrames={} gameFrame={}",
                _colliderClockSession,
                trigger,
                kTraceFramesPerEpisode,
                frame.gameFrameIndex);
        }
        if (_colliderClockFramesRemaining == 0 ||
            (_colliderClockHasLoggedFrame &&
                _colliderClockLastLoggedFrame == frame.gameFrameIndex)) {
            return;
        }
        _colliderClockLastLoggedFrame = frame.gameFrameIndex;
        _colliderClockHasLoggedFrame = true;
        --_colliderClockFramesRemaining;

        const auto currentFrikHand = [](const bool isLeft) {
            TrackedNodeFrame sample{};
            if (!frik_visual_authority::isAvailable()) {
                return sample;
            }
            const auto world = frik_visual_authority::getHandWorldTransform(
                frik_visual_authority::handFromBool(isLeft));
            if (finiteNiTransform(world)) {
                sample.world = world;
                sample.valid = true;
            }
            return sample;
        };
        const TrackedNodeFrame rightFrik = currentFrikHand(false);
        const TrackedNodeFrame leftFrik = currentFrikHand(true);
        const bool rightExternal =
            frik_visual_authority::hasPublishedExternalHandWorldTransform(
                frik_visual_authority::Hand::Right);
        const bool leftExternal =
            frik_visual_authority::hasPublishedExternalHandWorldTransform(
                frik_visual_authority::Hand::Left);
        const auto rightController =
            vrcf::VRControllers.getPollSnapshot(vrcf::Hand::Right);
        const auto leftController =
            vrcf::VRControllers.getPollSnapshot(vrcf::Hand::Left);

        ROCK_LOG_INFO(
            Input,
            "COLLIDER_CLOCK input session={} frame={} controllerR(valid/packet/previous/changed/buttons)={}/{}/{}/{}/0x{:016X} controllerL(valid/packet/previous/changed/buttons)={}/{}/{}/{}/0x{:016X} axesR[0..4]=({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f}) axesL[0..4]=({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})/({:.3f},{:.3f})",
            _colliderClockSession,
            frame.gameFrameIndex,
            rightController.valid,
            rightController.packetNumber,
            rightController.previousPacketNumber,
            rightController.packetChanged,
            rightController.buttonsPressed,
            leftController.valid,
            leftController.packetNumber,
            leftController.previousPacketNumber,
            leftController.packetChanged,
            leftController.buttonsPressed,
            rightController.axisX[0],
            rightController.axisY[0],
            rightController.axisX[1],
            rightController.axisY[1],
            rightController.axisX[2],
            rightController.axisY[2],
            rightController.axisX[3],
            rightController.axisY[3],
            rightController.axisX[4],
            rightController.axisY[4],
            leftController.axisX[0],
            leftController.axisY[0],
            leftController.axisX[1],
            leftController.axisY[1],
            leftController.axisX[2],
            leftController.axisY[2],
            leftController.axisX[3],
            leftController.axisY[3],
            leftController.axisX[4],
            leftController.axisY[4]);

        ROCK_LOG_INFO(
            Physics,
            "COLLIDER_CLOCK frame session={} frame={} gameDt={:.6f} player(valid/moving/source)={}/{}/{} playerDelta=({:.3f},{:.3f},{:.3f}) playerWorld=({:.3f},{:.3f},{:.3f}) nodes(wandR/wandL/driverR/driverL)={}/{}/{}/{} wandR=({:.3f},{:.3f},{:.3f}) wandL=({:.3f},{:.3f},{:.3f}) driverR=({:.3f},{:.3f},{:.3f}) driverL=({:.3f},{:.3f},{:.3f})",
            _colliderClockSession,
            frame.gameFrameIndex,
            frame.deltaSeconds,
            runtime.playerSpace.valid,
            playerMoving,
            runtime.playerSpace.source ? runtime.playerSpace.source : "none",
            runtime.playerSpace.deltaGameUnits.x,
            runtime.playerSpace.deltaGameUnits.y,
            runtime.playerSpace.deltaGameUnits.z,
            runtime.playerSpace.world.translate.x,
            runtime.playerSpace.world.translate.y,
            runtime.playerSpace.world.translate.z,
            frame.rightWand.valid,
            frame.leftWand.valid,
            frame.rightWeaponDriver.valid,
            frame.leftWeaponDriver.valid,
            frame.rightWand.world.translate.x,
            frame.rightWand.world.translate.y,
            frame.rightWand.world.translate.z,
            frame.leftWand.world.translate.x,
            frame.leftWand.world.translate.y,
            frame.leftWand.world.translate.z,
            frame.rightWeaponDriver.world.translate.x,
            frame.rightWeaponDriver.world.translate.y,
            frame.rightWeaponDriver.world.translate.z,
            frame.leftWeaponDriver.world.translate.x,
            frame.leftWeaponDriver.world.translate.y,
            frame.leftWeaponDriver.world.translate.z);

        const bool weaponPresentedValid =
            weaponNode && finiteNiTransform(weaponNode->world);
        const RE::NiTransform weaponPresented =
            weaponPresentedValid ? weaponNode->world : RE::NiTransform{};
        const TransformDelta weaponRequestedToLive = weaponValid ?
            measureTransformDelta(
                weapon.requestedWeaponWorld,
                weapon.liveWeaponWorld) :
            TransformDelta{ -1.0f, -1.0f };
        const TransformDelta weaponResolvedToPresented =
            weaponValid && weaponPresentedValid ?
            measureTransformDelta(
                weapon.resolvedWeaponWorld,
                weaponPresented) :
            TransformDelta{ -1.0f, -1.0f };
        const std::uint64_t weaponSourceAgeFrames =
            weaponValid && frame.gameFrameIndex >= weapon.sourceGameFrameIndex ?
            frame.gameFrameIndex - weapon.sourceGameFrameIndex :
            0;
        ROCK_LOG_INFO(
            Weapon,
            "COLLIDER_CLOCK weapon session={} frame={} valid={} proxy={} contact={} visualCorrection={} source(frame/age/queue/solve)={}/{}/{}/{} physics(raw/sub/rem/accum/index/count/progress)=({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) requested=({:.3f},{:.3f},{:.3f}) live=({:.3f},{:.3f},{:.3f}) resolved=({:.3f},{:.3f},{:.3f}) presented(valid/pos)={}/({:.3f},{:.3f},{:.3f}) requestedToLive=({:.3f}gu,{:.3f}deg) resolvedToPresented=({:.3f}gu,{:.3f}deg)",
            _colliderClockSession,
            frame.gameFrameIndex,
            weaponValid,
            weapon.valid,
            weapon.contactActive,
            weapon.visualCorrectionActive,
            weapon.sourceGameFrameIndex,
            weaponSourceAgeFrames,
            weapon.sourceQueueSequence,
            weapon.solveSequence,
            weapon.physicsRawDeltaSeconds,
            weapon.physicsSubstepDeltaSeconds,
            weapon.physicsRemainderDeltaSeconds,
            weapon.physicsAccumulatedDeltaSeconds,
            weapon.physicsSubstepIndex,
            weapon.physicsSubstepCount,
            weapon.physicsSubstepProgress,
            weapon.requestedWeaponWorld.translate.x,
            weapon.requestedWeaponWorld.translate.y,
            weapon.requestedWeaponWorld.translate.z,
            weapon.liveWeaponWorld.translate.x,
            weapon.liveWeaponWorld.translate.y,
            weapon.liveWeaponWorld.translate.z,
            weapon.resolvedWeaponWorld.translate.x,
            weapon.resolvedWeaponWorld.translate.y,
            weapon.resolvedWeaponWorld.translate.z,
            weaponPresentedValid,
            weaponPresented.translate.x,
            weaponPresented.translate.y,
            weaponPresented.translate.z,
            weaponRequestedToLive.position,
            weaponRequestedToLive.rotationDegrees,
            weaponResolvedToPresented.position,
            weaponResolvedToPresented.rotationDegrees);

        const auto logHand = [&](const bool isLeft) {
            const std::size_t handIndex = isLeft ? 1u : 0u;
            const auto& handInput = isLeft ? frame.left : frame.right;
            const auto& wand = isLeft ? frame.leftWand : frame.rightWand;
            const auto& driver = isLeft ? frame.leftWeaponDriver : frame.rightWeaponDriver;
            const auto& frik = isLeft ? leftFrik : rightFrik;
            const bool external = isLeft ? leftExternal : rightExternal;
            const auto& hand = hands.hands[handIndex];
            const auto& palm = hand.twins[dynamic_hand_collision_telemetry::kPalmSlot];
            const TransformDelta rawToWand =
                !handInput.disabled && wand.valid ?
                measureTransformDelta(handInput.rawHandWorld, wand.world) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta wandToDriver =
                wand.valid && driver.valid ?
                measureTransformDelta(wand.world, driver.world) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToFrik =
                !handInput.disabled && frik.valid ?
                measureTransformDelta(handInput.rawHandWorld, frik.world) :
                TransformDelta{ -1.0f, -1.0f };
            const std::uint64_t sourceAgeFrames =
                palm.physicsSampleValid &&
                    frame.gameFrameIndex >= palm.sourceGameFrameIndex ?
                frame.gameFrameIndex - palm.sourceGameFrameIndex :
                0;

            ROCK_LOG_INFO(
                Hand,
                "COLLIDER_CLOCK hand session={} frame={} side={} snapshot={} disabled={} contact={} visual(active/external)={}/{} source(frame/age/queue/solve/seqlock)={}/{}/{}/{}/{} physics(raw/sub/rem/accum/index/count/progress)=({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) raw=({:.3f},{:.3f},{:.3f}) palm(published/requested/commanded/live)=({:.3f},{:.3f},{:.3f})/({:.3f},{:.3f},{:.3f})/({:.3f},{:.3f},{:.3f})/({:.3f},{:.3f},{:.3f}) frik(valid/pos)={}/({:.3f},{:.3f},{:.3f}) appliedDeviation=({:.3f},{:.3f},{:.3f}) deltas(rawToWand/wandToDriver/rawToFrik/requestedToLive)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/{:.3f}gu",
                _colliderClockSession,
                frame.gameFrameIndex,
                isLeft ? "L" : "R",
                handsValid,
                handInput.disabled,
                palm.contactActive,
                hand.visualActive,
                external,
                palm.sourceGameFrameIndex,
                sourceAgeFrames,
                palm.sourceQueueSequence,
                palm.solveSequence,
                palm.physicsSampleSequence,
                palm.physicsRawDeltaSeconds,
                palm.physicsDeltaSeconds,
                palm.physicsRemainderDeltaSeconds,
                palm.physicsAccumulatedDeltaSeconds,
                palm.physicsSubstepIndex,
                palm.physicsSubstepCount,
                palm.physicsSubstepProgress,
                handInput.rawHandWorld.translate.x,
                handInput.rawHandWorld.translate.y,
                handInput.rawHandWorld.translate.z,
                palm.publishedTargetWorld.translate.x,
                palm.publishedTargetWorld.translate.y,
                palm.publishedTargetWorld.translate.z,
                palm.requestedTargetWorldGame.x,
                palm.requestedTargetWorldGame.y,
                palm.requestedTargetWorldGame.z,
                palm.commandedTargetWorldGame.x,
                palm.commandedTargetWorldGame.y,
                palm.commandedTargetWorldGame.z,
                palm.liveBodyWorldGame.x,
                palm.liveBodyWorldGame.y,
                palm.liveBodyWorldGame.z,
                frik.valid,
                frik.world.translate.x,
                frik.world.translate.y,
                frik.world.translate.z,
                hand.appliedVisualDeviationWorldGame.x,
                hand.appliedVisualDeviationWorldGame.y,
                hand.appliedVisualDeviationWorldGame.z,
                rawToWand.position,
                rawToWand.rotationDegrees,
                wandToDriver.position,
                wandToDriver.rotationDegrees,
                rawToFrik.position,
                rawToFrik.rotationDegrees,
                palm.requestedGapGameUnits);
        };
        logHand(false);
        logHand(true);

        const auto logGrab = [&](const bool isLeft) {
            const std::size_t handIndex = isLeft ? 1u : 0u;
            Hand& heldHand = isLeft ? _leftHand : _rightHand;
            const auto& handInput = isLeft ? frame.left : frame.right;
            const auto& frik = isLeft ? leftFrik : rightFrik;
            auto& previous = _grabLocomotionClockStates[handIndex];
            if (!frame.hknpWorld || handInput.disabled || !heldHand.isHolding()) {
                previous = {};
                return;
            }

            GrabOverlayPointProbeSample applied{};
            const bool appliedValid =
                heldHand.tryGetGrabOverlayPointProbeSample(frame.hknpWorld, applied);
            grab_transform_telemetry::RuntimeSample sample{};
            const bool sampleValid = heldHand.getGrabTransformTelemetrySnapshot(
                frame.hknpWorld,
                handInput.rawHandWorld,
                sample);
            const bool consecutive =
                previous.active &&
                previous.session == _colliderClockSession &&
                frame.gameFrameIndex == previous.frame + 1;
            const auto stepDelta = [&](const bool currentValid,
                                       const bool previousValid,
                                       const RE::NiTransform& current,
                                       const RE::NiTransform& prior) {
                return consecutive && currentValid && previousValid ?
                    measureTransformDelta(prior, current) :
                    TransformDelta{ -1.0f, -1.0f };
            };

            const TransformDelta rawStep =
                consecutive ?
                measureTransformDelta(previous.rawHandWorld, handInput.rawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta targetStep = stepDelta(
                appliedValid,
                previous.hasAppliedProxyTarget,
                applied.appliedProxyTargetWorld,
                previous.appliedProxyTargetWorld);
            const TransformDelta proxyStep = stepDelta(
                sampleValid && sample.hasProxyReadback,
                previous.hasProxyReadback,
                sample.proxyReadbackWorld,
                previous.proxyReadbackWorld);
            const TransformDelta bodyStep = stepDelta(
                sampleValid && sample.hasHeldBodyWorld,
                previous.hasHeldBody,
                sample.heldBodyWorld,
                previous.heldBodyWorld);
            const TransformDelta nodeStep = stepDelta(
                sampleValid && sample.hasHeldNodeWorld,
                previous.hasHeldNode,
                sample.heldNodeWorld,
                previous.heldNodeWorld);
            const TransformDelta visualStep = stepDelta(
                sampleValid && sample.hasHeldRelativeHandTarget,
                previous.hasHeldRelativeHandTarget,
                sample.heldRelativeHandTargetWorld,
                previous.heldRelativeHandTargetWorld);
            const TransformDelta frikStep = stepDelta(
                frik.valid,
                previous.hasFrikHand,
                frik.world,
                previous.frikHandWorld);

            const TransformDelta rawToApplied = appliedValid ?
                measureTransformDelta(
                    handInput.rawHandWorld,
                    applied.appliedRawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToQueued = appliedValid ?
                measureTransformDelta(
                    handInput.rawHandWorld,
                    applied.queuedRawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta queuedToApplied = appliedValid ?
                measureTransformDelta(
                    applied.queuedRawHandWorld,
                    applied.appliedRawHandWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta targetToProxy =
                appliedValid && sampleValid && sample.hasProxyReadback ?
                measureTransformDelta(
                    applied.appliedProxyTargetWorld,
                    sample.proxyReadbackWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta desiredToBody =
                sampleValid && sample.hasGrabStartFrames && sample.hasHeldBodyWorld ?
                measureTransformDelta(
                    sample.currentRawDesiredBodyWorld,
                    sample.heldBodyWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta bodyDerivedToNode =
                sampleValid && sample.hasHeldBodyDerivedNodeWorld && sample.hasHeldNodeWorld ?
                measureTransformDelta(
                    sample.heldBodyDerivedNodeWorld,
                    sample.heldNodeWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta visualToFrik =
                sampleValid && sample.hasHeldRelativeHandTarget && frik.valid ?
                measureTransformDelta(
                    sample.heldRelativeHandTargetWorld,
                    frik.world) :
                TransformDelta{ -1.0f, -1.0f };
            const TransformDelta rawToVisual =
                sampleValid && sample.hasHeldRelativeHandTarget ?
                measureTransformDelta(
                    handInput.rawHandWorld,
                    sample.heldRelativeHandTargetWorld) :
                TransformDelta{ -1.0f, -1.0f };
            const std::uint64_t sourceAgeFrames =
                appliedValid && applied.sourceGameFrameIndex != 0 &&
                    frame.gameFrameIndex >= applied.sourceGameFrameIndex ?
                frame.gameFrameIndex - applied.sourceGameFrameIndex :
                0;

            ROCK_LOG_INFO(
                Hand,
                "GRAB_LOCOMOTION drive session={} frame={} side={} valid(applied/sample/proxy/body)={}/{}/{}/{} form=0x{:08X} bodies(proxy/held)={}/{} source(frame/age/queue/pending/flush/after)={}/{}/{}/{}/{}/{} flushPhysics(valid/raw/sub/rem/accum/index/count/progress)={}/({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) afterPhysics(valid/raw/sub/rem/accum/index/count/progress)={}/({:.6f}/{:.6f}/{:.6f}/{:.6f}/{}/{}/{:.3f}) consumptionRebase(status/shift/controller/vtable/scaleRev)={}/({:.3f},{:.3f},{:.3f})/{:016X}:{:016X}/{:016X}:{:016X}/{}:{} raw=({:.3f},{:.3f},{:.3f}) queuedRaw=({:.3f},{:.3f},{:.3f}) appliedRaw=({:.3f},{:.3f},{:.3f}) target=({:.3f},{:.3f},{:.3f}) proxy=({:.3f},{:.3f},{:.3f}) desiredBody=({:.3f},{:.3f},{:.3f}) body=({:.3f},{:.3f},{:.3f}) gaps(rawToQueued/queuedToApplied/rawToApplied/targetToProxy/desiredToBody)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg) steps(raw/target/proxy/body)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)",
                _colliderClockSession,
                frame.gameFrameIndex,
                isLeft ? "L" : "R",
                appliedValid,
                sampleValid,
                sample.hasProxyReadback,
                sample.hasHeldBodyWorld,
                sample.heldFormId,
                applied.proxyBodyId.value,
                applied.objectBodyId.value,
                applied.sourceGameFrameIndex,
                sourceAgeFrames,
                applied.sourceQueueSequence,
                applied.pendingQueueSequence,
                applied.flushSequence,
                applied.afterSolveSequence,
                applied.flushTiming.valid,
                applied.flushTiming.rawDeltaSeconds,
                applied.flushTiming.substepDeltaSeconds,
                applied.flushTiming.remainderDeltaSeconds,
                applied.flushTiming.accumulatedDeltaSeconds,
                applied.flushTiming.substepIndex,
                applied.flushTiming.substepCount,
                applied.flushTiming.substepProgress,
                applied.afterSolveTiming.valid,
                applied.afterSolveTiming.rawDeltaSeconds,
                applied.afterSolveTiming.substepDeltaSeconds,
                applied.afterSolveTiming.remainderDeltaSeconds,
                applied.afterSolveTiming.accumulatedDeltaSeconds,
                applied.afterSolveTiming.substepIndex,
                applied.afterSolveTiming.substepCount,
                applied.afterSolveTiming.substepProgress,
                grab_authority_source_clock::consumptionFrameRebaseStatusName(applied.consumptionFrameRebaseStatus),
                applied.consumptionFrameShiftGame.x,
                applied.consumptionFrameShiftGame.y,
                applied.consumptionFrameShiftGame.z,
                applied.sourceControllerIdentity,
                applied.consumptionControllerIdentity,
                applied.sourceControllerVtable,
                applied.consumptionControllerVtable,
                applied.sourcePhysicsScaleRevision,
                applied.consumptionPhysicsScaleRevision,
                handInput.rawHandWorld.translate.x,
                handInput.rawHandWorld.translate.y,
                handInput.rawHandWorld.translate.z,
                applied.queuedRawHandWorld.translate.x,
                applied.queuedRawHandWorld.translate.y,
                applied.queuedRawHandWorld.translate.z,
                applied.appliedRawHandWorld.translate.x,
                applied.appliedRawHandWorld.translate.y,
                applied.appliedRawHandWorld.translate.z,
                applied.appliedProxyTargetWorld.translate.x,
                applied.appliedProxyTargetWorld.translate.y,
                applied.appliedProxyTargetWorld.translate.z,
                sample.proxyReadbackWorld.translate.x,
                sample.proxyReadbackWorld.translate.y,
                sample.proxyReadbackWorld.translate.z,
                sample.currentRawDesiredBodyWorld.translate.x,
                sample.currentRawDesiredBodyWorld.translate.y,
                sample.currentRawDesiredBodyWorld.translate.z,
                sample.heldBodyWorld.translate.x,
                sample.heldBodyWorld.translate.y,
                sample.heldBodyWorld.translate.z,
                rawToQueued.position,
                rawToQueued.rotationDegrees,
                queuedToApplied.position,
                queuedToApplied.rotationDegrees,
                rawToApplied.position,
                rawToApplied.rotationDegrees,
                targetToProxy.position,
                targetToProxy.rotationDegrees,
                desiredToBody.position,
                desiredToBody.rotationDegrees,
                rawStep.position,
                rawStep.rotationDegrees,
                targetStep.position,
                targetStep.rotationDegrees,
                proxyStep.position,
                proxyStep.rotationDegrees,
                bodyStep.position,
                bodyStep.rotationDegrees);

            ROCK_LOG_INFO(
                Hand,
                "GRAB_LOCOMOTION visual session={} frame={} side={} valid(bodyDerived/node/heldHand/frik)={}/{}/{}/{} bodyDerived=({:.3f},{:.3f},{:.3f}) node=({:.3f},{:.3f},{:.3f}) heldHand=({:.3f},{:.3f},{:.3f}) frik=({:.3f},{:.3f},{:.3f}) gaps(bodyDerivedToNode/heldHandToFrik/rawToHeldHand)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg) steps(node/heldHand/frik)=({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)/({:.3f}gu,{:.3f}deg)",
                _colliderClockSession,
                frame.gameFrameIndex,
                isLeft ? "L" : "R",
                sample.hasHeldBodyDerivedNodeWorld,
                sample.hasHeldNodeWorld,
                sample.hasHeldRelativeHandTarget,
                frik.valid,
                sample.heldBodyDerivedNodeWorld.translate.x,
                sample.heldBodyDerivedNodeWorld.translate.y,
                sample.heldBodyDerivedNodeWorld.translate.z,
                sample.heldNodeWorld.translate.x,
                sample.heldNodeWorld.translate.y,
                sample.heldNodeWorld.translate.z,
                sample.heldRelativeHandTargetWorld.translate.x,
                sample.heldRelativeHandTargetWorld.translate.y,
                sample.heldRelativeHandTargetWorld.translate.z,
                frik.world.translate.x,
                frik.world.translate.y,
                frik.world.translate.z,
                bodyDerivedToNode.position,
                bodyDerivedToNode.rotationDegrees,
                visualToFrik.position,
                visualToFrik.rotationDegrees,
                rawToVisual.position,
                rawToVisual.rotationDegrees,
                nodeStep.position,
                nodeStep.rotationDegrees,
                visualStep.position,
                visualStep.rotationDegrees,
                frikStep.position,
                frikStep.rotationDegrees);

            previous.rawHandWorld = handInput.rawHandWorld;
            previous.appliedProxyTargetWorld = applied.appliedProxyTargetWorld;
            previous.proxyReadbackWorld = sample.proxyReadbackWorld;
            previous.heldBodyWorld = sample.heldBodyWorld;
            previous.heldNodeWorld = sample.heldNodeWorld;
            previous.heldRelativeHandTargetWorld = sample.heldRelativeHandTargetWorld;
            previous.frikHandWorld = frik.world;
            previous.session = _colliderClockSession;
            previous.frame = frame.gameFrameIndex;
            previous.active = true;
            previous.hasAppliedProxyTarget = appliedValid;
            previous.hasProxyReadback = sampleValid && sample.hasProxyReadback;
            previous.hasHeldBody = sampleValid && sample.hasHeldBodyWorld;
            previous.hasHeldNode = sampleValid && sample.hasHeldNodeWorld;
            previous.hasHeldRelativeHandTarget =
                sampleValid && sample.hasHeldRelativeHandTarget;
            previous.hasFrikHand = frik.valid;
        };
        logGrab(false);
        logGrab(true);

        if (_colliderClockFramesRemaining == 0) {
            ROCK_LOG_INFO(
                Physics,
                "COLLIDER_CLOCK end session={} reason=budget gameFrame={}",
                _colliderClockSession,
                frame.gameFrameIndex);
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
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_Physics", true)) {
            ROCK_LOG_INFO(Init, "FRIK offhand grip permanently suppressed");
        }

        {
            _rightHand.updateCollisionTransform(hknp, getInteractionHandTransform(false), 0.011f);
            _leftHand.updateCollisionTransform(hknp, getInteractionHandTransform(true), 0.011f);
            _bodyBoneColliders.update(hknp, 0.011f);
            ROCK_LOG_INFO(Init, "Initial bone-derived hand collider transforms updated");
        }

        _rightHand.preloadSelectionBeam();
        _leftHand.preloadSelectionBeam();

        _hasPrevPositions = false;
        _deltaLogCounter = 0;
        _contactLogCounter = 0;
        _bodyContactRuntime.reset();
        _dynamicPushElapsedSeconds = 0.0f;
        _dynamicPushCooldownUntil.clear();
        _heldImpactHapticCooldownUntil.clear();
        _grabEventFrameCounter = 0;
        _shoulderStashStates = {};
        _mouthConsumeStates = {};
        _feedbackHaptics.reset();
        _grabInputIntentStates = {};
        _peerHeldJoinRetryStates = {};
        _heldWeaponTriggerEquipIntents = {};
        _forceGrabCommittedThisFrame = {};
        _equippedWeaponShoulderSheath = {};
        _equippedWeaponSheathRetrievalStates = {};
        _equippedWeaponSheathCommittedThisFrame = {};
        _equippedWeaponUnsheathCommittedThisFrame = {};
        _bareFistGuardState = {};
        _completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _equippedWeaponDropMomentumHandoffs = {};
        clearLooseGrenadeRuntimeState();
        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        _equippedWeaponHandAssignment = {};
        _lastPipboyWeaponSelectionSequence = 0;
        _equippedWeaponHandlingSettings = {};
        _fixedFiringHandIsLeft = false;
        _dynamicWeaponRightHandInteractionEnabled = false;
        _dynamicWeaponLeftHandInteractionEnabled = true;
        _equippedWeaponHandlingModeInitialized = false;
        _equippedWeaponHandlingModeReconcilePending = false;
        _fixedLeftCarry = {};
        equipped_weapon_handling_runtime::reset();
        clearEquippedWeaponPostDropCollisionSuppressionState();
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

    void PhysicsInteraction::synchronizeNativeScopePresentationAfterFrikUpdate()
    {
        if (!_initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* weaponNode = f4vr::getWeaponNode();
        _twoHandedGrip.synchronizeNativeScopePresentationAfterFrikUpdate(weaponNode, _weaponCollision.getCurrentWeaponGenerationKey());
    }

    void PhysicsInteraction::finalizeGunstockPresentationAfterNativeAnimation()
    {
        if (!_initialized.load(std::memory_order_acquire) ||
            !g_rockConfig.rockGunstockModeEnabled ||
            !runtime_state::isLocalSkeletonReady() ||
            (provider::currentNativeAnimationAuthorityFlagsV1() &
                authored_weapon_grip_capture_policy::kWeapon) == 0) {
            return;
        }

        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        (void)_twoHandedGrip.
            finalizeGunstockPresentationAfterNativeWeaponAnimation(
                weaponNode,
                _weaponCollision.getCurrentWeaponGenerationKey());
    }

    bool PhysicsInteraction::tryGetManualScopeDirectTransitionTarget(
        std::uint64_t& outWeaponGenerationKey,
        std::uint32_t& outNativeOverlayIndex) const
    {
        outWeaponGenerationKey = 0;
        outNativeOverlayIndex = 0;
        if (!_initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return false;
        }
        const auto snapshot = _weaponCollision.getNativeScopeSightAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity publishedIdentity{
            .weaponGenerationKey = snapshot.weaponGenerationKey,
            .equippedWeaponOwnershipKey = snapshot.equippedWeaponOwnershipKey,
            .weaponFormID = snapshot.weaponFormID,
        };
        const native_scope_sight_anchor_policy::PublicationIdentity currentIdentity{
            .weaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey(),
            .equippedWeaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey(),
            .weaponFormID = _weaponCollision.getCurrentObservedEquippedWeaponFormID(),
        };
        const NativeScopeResolvedAnchorSnapshot resolvedAnchor =
            _twoHandedGrip.getNativeScopeResolvedAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity
            resolvedIdentity{
                .weaponGenerationKey = resolvedAnchor.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    resolvedAnchor.equippedWeaponOwnershipKey,
                .weaponFormID = resolvedAnchor.weaponFormID,
            };
        if (!resolvedAnchor.valid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(
                resolvedIdentity,
                currentIdentity) ||
            !snapshot.manualDirectTransitionRequired || !snapshot.nativeScopeOverlayValid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(publishedIdentity, currentIdentity)) {
            return false;
        }
        outWeaponGenerationKey = snapshot.weaponGenerationKey;
        outNativeOverlayIndex = snapshot.nativeScopeOverlayIndex;
        return true;
    }

    void PhysicsInteraction::updateEquippedWeaponTransition()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeGunState =
            f4vr::getNativeGunState(player);
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(player);
        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool currentIdentityCaptured =
            native_equipped_weapon_draw::captureCurrentIdentity(
                currentIdentity);
        weapon_transition_animation_acceleration::service(
            weapon_transition_animation_acceleration::ServiceInput{
                .player = player,
                .identity = currentIdentityCaptured ?
                    weapon_transition_animation_acceleration::Identity{
                        .formID = currentIdentity.formID,
                        .instanceData = currentIdentity.instanceData,
                        .equipIndex = currentIdentity.equipIndex,
                    } :
                    weapon_transition_animation_acceleration::Identity{},
                .nativeWeaponState = nativeWeaponState,
                .runtimeAllowed =
                    runtime.visualAuthorityAvailable &&
                    runtime.localSkeletonReady &&
                    !runtime.localMenuBlocking &&
                    !runtime.compatibilityConfigBlocking,
            });
        const bool nativeWeaponAnimationActive =
            provider::currentNativeAnimationAuthorityFlagsV1() != 0 ||
            nativeGunState ==
                static_cast<std::uint32_t>(RE::GUN_STATE::kReloading);
        _equippedWeaponTransition.update(
            EquippedWeaponTransitionCoordinator::FrameInput{
                .deltaSeconds = runtime.deltaSeconds,
                .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
                .localSkeletonReady = runtime.localSkeletonReady,
                .menuBlocking = runtime.localMenuBlocking,
                .compatibilityBlocking = runtime.compatibilityConfigBlocking,
                .nativeWeaponState = nativeWeaponState,
                .intentionalShoulderSheathActive =
                    _equippedWeaponShoulderSheath.active,
                .shoulderSheathFormID =
                    _equippedWeaponShoulderSheath.weaponFormID,
                .shoulderSheathInstanceData =
                    _equippedWeaponShoulderSheath.weaponInstanceData,
                .shoulderSheathEquipIndex =
                    _equippedWeaponShoulderSheath.equipIndex,
                .nativeWeaponAnimationActive = nativeWeaponAnimationActive,
                .sourceSchedulerSequence =
                    _currentPreFrikSchedulerSequence,
            });
    }

    void PhysicsInteraction::update()
    {
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        _equippedWeaponSheathCommittedThisFrame = {};
        _equippedWeaponUnsheathCommittedThisFrame = {};
        const auto& runtime = runtime_state::currentFrame();
        // A publication is valid only when this invocation reaches the single
        // completed-frame handoff below. Early returns must never let the main
        // loop freeze a previous frame's Havok state again.
        _pendingDebugOverlayFrame = {};
        const auto retireDynamicWeaponForInterruptedFrame = [this]() {
            if (!_initialized.load(std::memory_order_acquire)) {
                return;
            }
            auto* currentBhk = getPlayerBhkWorld();
            auto* currentHknp = currentBhk ? getHknpWorld(currentBhk) : nullptr;
            if (currentBhk && currentBhk == _cachedBhkWorld &&
                currentHknp && currentHknp == _cachedHknpWorld) {
                _dynamicWeaponCollision.retireAll(currentBhk);
            } else {
                _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
            }
        };
        refreshEquippedWeaponHandlingSettings();
        if (!runtime.visualAuthorityAvailable) {
            retireDynamicWeaponForInterruptedFrame();
            restoreHeldMassMovementSlowdown("frik-unavailable");
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            return;
        }

        // ROCK always binds raw controller identity physically: right is the
        // primary wand and left is the secondary wand. Weapon handedness is a
        // separate ROCK role and never remaps buttons/controllers.
        vrcf::VRControllers.update(false);

        // Before any early return below: a skipped consume would let a stale
        // accept-button press replay as a reload frames later (see the API doc).
        input_remap_runtime::updateFiringHandReloadInput(runtime.deltaSeconds);

        _deltaTime = runtime.deltaSeconds;

        if (_deltaTime <= 0.0f || _deltaTime > 0.1f) {
            _deltaTime = 1.0f / 90.0f;
        }
        enforceNativeGrabHapticRuntimeSuppression();
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
        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                menuBlocking,
                false,
                false)) {
            retireDynamicWeaponForInterruptedFrame();
            _equippedWeaponMenuReconcilePending = true;
            if (_initialized) {
                _twoHandedGrip.reset();
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                clearEquippedWeaponFiringGripInputState();
                auto* bhkMenu = getPlayerBhkWorld();
                if (bhkMenu) {
                    auto* hknpMenu = getHknpWorld(bhkMenu);
                    if (hknpMenu) {
                        restoreRightHandCollisionAfterDominantWeapon(hknpMenu);
                        restoreHandCollisionAfterWeaponSupport(hknpMenu, true);
                        restoreHandCollisionAfterWeaponSupport(hknpMenu, false);
                        restoreHandCollisionAfterEquippedWeaponDrop(hknpMenu, false);
                        restoreHandCollisionAfterEquippedWeaponDrop(hknpMenu, true);
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
                    _rightWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
                    hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
                    hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
                    hand_collision_suppression_math::clear(_rightWeaponSupportCollisionSuppression);
                    clearEquippedWeaponPostDropCollisionSuppressionState();
                }
            }
            debug::ClearFrame();
            clearEquippedWeaponFiringGripInputState();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            auto* snapshotBhk = getPlayerBhkWorld();
            auto* snapshotHknp = snapshotBhk ? getHknpWorld(snapshotBhk) : nullptr;
            if (snapshotBhk && snapshotHknp) {
                _dynamicWorldCarCollision.restoreAll(snapshotBhk, snapshotHknp, "menu-blocked");
            } else {
                _dynamicWorldCarCollision.abandon();
            }
            observeLifecycleFrame(snapshotBhk, snapshotHknp, ::rock::provider::RockProviderLifecycleReason::MenuBlocked);
            restoreHeldMassMovementSlowdown("menu-blocked");
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                false,
                !g_rockConfig.rockEnabled,
                false)) {
            if (_initialized) {
                shutdown();
            }
            debug::ClearFrame();
            return;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            _dynamicWorldCarCollision.abandon();
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
            _dynamicWorldCarCollision.abandon();
            _cachedHknpWorld = nullptr;
            observeLifecycleFrame(bhk, nullptr, ::rock::provider::RockProviderLifecycleReason::WorldUnavailable);
            _twoHandedGrip.reset();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            debug::ClearFrame();
            restoreHeldMassMovementSlowdown("world-unavailable");
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
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
            restoreHandCollisionAfterWeaponSupport(hknp, true);
            restoreHandCollisionAfterWeaponSupport(hknp, false);
            restoreHandCollisionAfterEquippedWeaponDrop(hknp, false);
            restoreHandCollisionAfterEquippedWeaponDrop(hknp, true);
            _twoHandedGrip.reset();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _bodyContactRuntime.reset();
            clearLeftWeaponContact();
            clearRightWeaponContact();

            destroyHandCollisions(bhk);
            destroyBodyBoneCollisions(bhk);
            _weaponCollision.invalidateForScaleChange(hknp);
            markGeneratedBodiesInvalidated();
            hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            hand_collision_suppression_math::clear(_rightWeaponSupportCollisionSuppression);
            _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
            _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            _rightWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            clearEquippedWeaponPostDropCollisionSuppressionState();
            restoreNativePlayerCollisionSuppression(hknp, "scale-change");
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        refreshHandBoneCache();
        sampleHandTransformParity();
        const auto frame = buildFrameContext(bhk, hknp, _deltaTime);
        _palmClockGameFrameIndex.store(runtime.frameIndex, std::memory_order_release);
        _palmClockGameDeltaSeconds.store(frame.deltaSeconds, std::memory_order_release);
        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::None);
        if (!generatedBodiesMatchLifecycle(bhk, hknp)) {
            const bool rebuilt =
                !frame.reloadBoundaryActive &&
                rebuildGeneratedBodiesForLifecycle(bhk, hknp, "epoch-mismatch");
            if (rebuilt) {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesRebuilt);
            } else {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesInvalidated);
                ROCK_LOG_SAMPLE_DEBUG(Update,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "ROCK lifecycle generated-body rebuild pending: animationBoundary={} flags=0x{:08X} reason={} worldGen={} skeletonGen={} providerGen={} stableFrames={}",
                    frame.reloadBoundaryActive ? "yes" : "no",
                    _lifecycleFlagsAtomic.load(std::memory_order_acquire),
                    _lastLifecycleReasonAtomic.load(std::memory_order_acquire),
                    _worldGenerationAtomic.load(std::memory_order_acquire),
                    _skeletonGenerationAtomic.load(std::memory_order_acquire),
                    _providerGenerationAtomic.load(std::memory_order_acquire),
                    _stableFrameCountAtomic.load(std::memory_order_acquire));
                debug::ClearFrame();
                _twoHandedGrip.reset();
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                clearEquippedWeaponFiringGripInputState();
                _shoulderStashStates = {};
                _mouthConsumeStates = {};
                _feedbackHaptics.reset();
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
            _twoHandedGrip.reset();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        const bool forceBareFistRecheck = _equippedWeaponMenuReconcilePending;
        if (_equippedWeaponMenuReconcilePending) {
            const bool firingHandIsLeft = _twoHandedGrip.isFiringGripOccupied() ?
                _twoHandedGrip.isFiringHandLeft() :
                _fixedFiringHandIsLeft;
            const bool primaryGrabHeld = input_remap_runtime::isRawButtonPhysicallyHeld(
                firingHandIsLeft,
                g_rockConfig.rockGrabButtonID);
            _pendingEquippedWeaponPrimaryOnlyGripStart = PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = _equippedWeaponHandlingSettings.primaryDetachEnabled &&
                    primaryGrabHeld,
                .isLeft = firingHandIsLeft,
            };
            _equippedWeaponMenuReconcilePending = false;
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon ownership reconciled after menu: primaryGrabHeld={} pendingPrimaryOnlyStart={}",
                primaryGrabHeld ? "yes" : "no",
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending ? "yes" : "no");
        }
        enforceNoBareFistState(forceBareFistRecheck);

        if (_collisionLayerRegistered &&
            (_expectedHandLayerMask != 0 || _expectedWeaponLayerMask != 0 || _expectedReloadLayerMask != 0 || _expectedBodyLayerMask != 0 ||
                _expectedDynamicHandProxyLayerMask != 0 || _expectedDynamicWeaponProxyLayerMask != 0 ||
                _expectedDynamicWorldCarClutterLayerMask != 0 || _expectedDynamicWorldCarLargeClutterLayerMask != 0 ||
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
            const auto desiredDynamicRightHandProxyMask =
                collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                    false,
                    g_rockConfig.rockHandDynamicInteractionsEnabled,
                    _dynamicWeaponRightHandInteractionEnabled);
            const auto desiredDynamicLeftHandProxyMask =
                collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                    true,
                    g_rockConfig.rockHandDynamicInteractionsEnabled,
                    _dynamicWeaponLeftHandInteractionEnabled);
            const auto desiredDynamicWeaponProxyMask =
                collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask(
                    g_rockConfig.rockHandDynamicInteractionsEnabled,
                    _dynamicWeaponRightHandInteractionEnabled,
                    _dynamicWeaponLeftHandInteractionEnabled);
            const bool desiredNativeControllerPolicyEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
            const bool nativeControllerPolicyModeChanged =
                _nativeCharacterControllerLayerPolicyCaptured &&
                _nativeCharacterControllerLayerPolicyEnabled != desiredNativeControllerPolicyEnabled;
            if (!collision_layer_policy::matrixLayerMaskMatches(_expectedHandLayerMask, desiredHandMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedWeaponLayerMask, desiredWeaponMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedReloadLayerMask, desiredReloadMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedBodyLayerMask, desiredBodyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicHandProxyLayerMask,
                    desiredDynamicRightHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicLeftHandProxyLayerMask,
                    desiredDynamicLeftHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicWeaponProxyLayerMask,
                    desiredDynamicWeaponProxyMask) ||
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
                const auto currentDynamicHandProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY];
                const auto currentDynamicLeftHandProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY];
                const auto currentDynamicWeaponProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY];
                const auto currentDynamicWorldCarClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER];
                const auto currentDynamicWorldCarLargeClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER];
                const bool handMaskDrifted = _expectedHandLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentHandMask, _expectedHandLayerMask);
                const bool weaponMaskDrifted = _expectedWeaponLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentWeaponMask, _expectedWeaponLayerMask);
                const bool reloadMaskDrifted = _expectedReloadLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentReloadMask, _expectedReloadLayerMask);
                const bool bodyMaskDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::bodyManagedLayerMaskMatches(currentBodyMask, _expectedBodyLayerMask);
                const bool dynamicHandProxyMaskDrifted = _expectedDynamicHandProxyLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicHandProxyMask, _expectedDynamicHandProxyLayerMask);
                const bool dynamicLeftHandProxyMaskDrifted =
                    _expectedDynamicLeftHandProxyLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(
                        currentDynamicLeftHandProxyMask,
                        _expectedDynamicLeftHandProxyLayerMask);
                const bool dynamicWeaponProxyMaskDrifted = _expectedDynamicWeaponProxyLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWeaponProxyMask, _expectedDynamicWeaponProxyLayerMask);
                const bool dynamicWorldCarClutterMaskDrifted = _expectedDynamicWorldCarClutterLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarClutterMask, _expectedDynamicWorldCarClutterLayerMask);
                const bool dynamicWorldCarLargeClutterMaskDrifted = _expectedDynamicWorldCarLargeClutterLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarLargeClutterMask, _expectedDynamicWorldCarLargeClutterLayerMask);
                const bool actorToolPairsDrifted =
                    _expectedHandLayerMask != 0 && _expectedWeaponLayerMask != 0 &&
                    !collision_layer_policy::rockToolActorPairsMatch(matrix, _expectedHandLayerMask, _expectedWeaponLayerMask);
                const bool bodyPairsDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::rockBodyManagedPairsMatch(matrix, _expectedBodyLayerMask);
                const bool nativeControllerObjectPairsDrifted =
                    _nativeCharacterControllerLayerPolicyCaptured &&
                    !collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix, _expectedNativeCharacterControllerLayerMask);
                if (handMaskDrifted || weaponMaskDrifted || reloadMaskDrifted || bodyMaskDrifted || dynamicHandProxyMaskDrifted || dynamicLeftHandProxyMaskDrifted || dynamicWeaponProxyMaskDrifted ||
                    dynamicWorldCarClutterMaskDrifted || dynamicWorldCarLargeClutterMaskDrifted || actorToolPairsDrifted || bodyPairsDrifted ||
                    nativeControllerObjectPairsDrifted) {
                    const auto currentNativeCharacterControllerMask =
                        _nativeCharacterControllerLayerPolicyCaptured ? matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER] : 0;
                    ROCK_LOG_WARN(Config,
                        "ROCK configured layer mask drift detected; hand expected=0x{:016X} current=0x{:016X}, weapon expected=0x{:016X} current=0x{:016X}, reload expected=0x{:016X} current=0x{:016X}, body expected=0x{:016X} current=0x{:016X}, dynamicRightHandProxy expected=0x{:016X} current=0x{:016X}, dynamicLeftHandProxy expected=0x{:016X} current=0x{:016X}, dynamicWeaponProxy expected=0x{:016X} current=0x{:016X}, carClutter expected=0x{:016X} current=0x{:016X}, carLarge expected=0x{:016X} current=0x{:016X}, nativeController expected=0x{:016X} current=0x{:016X}, actorToolPairs={}, bodyManagedPairs={}, nativeControllerObjects={}; re-registering",
                        collision_layer_policy::matrixAddressableMask(_expectedHandLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentHandMask),
                        collision_layer_policy::matrixAddressableMask(_expectedWeaponLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentWeaponMask),
                        collision_layer_policy::matrixAddressableMask(_expectedReloadLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentReloadMask),
                        collision_layer_policy::matrixAddressableMask(_expectedBodyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentBodyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicHandProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicLeftHandProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicLeftHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWeaponProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWeaponProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWorldCarClutterLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarClutterMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWorldCarLargeClutterLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarLargeClutterMask),
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
        /*
         * FRIK re-attaches the weapon node to the firing hand every frame
         * before ROCK runs, even in part-carry. Republish ROCK's solved carry
         * transform first so weapon-part probes, firing-grip zone checks, and
         * grip capture frames all read the weapon where the player sees it —
         * the same frame the generated colliders follow.
         */
        (void)_twoHandedGrip.republishPartCarryWeaponTransform(weaponNode);
        const bool rightHandWeaponEquipped = weaponNode != nullptr;
        const bool retainedWeaponCollisionActive =
            _weaponCollision.hasWeaponBody() && _weaponCollision.getCurrentWeaponGenerationKey() != 0;
        /*
         * Reload can temporarily remove the first-person weapon node while ROCK
         * deliberately retains the generated weapon body set. Keep the dominant
         * hand under weapon authority until those retained bodies are gone.
         */
        bool rightHandWeaponAuthorityActive = rightHandWeaponEquipped || retainedWeaponCollisionActive;
        /*
         * A visible part-carry (no hand at the firing grip) frees the right hand
         * even while generated weapon bodies exist: the free hand needs live
         * colliders for offhand-parity interaction, the same layer 43 vs 44
         * coexistence the left hand already has. Reload-retained bodies with no
         * visible weapon node keep the dominant-hand suppression because the
         * part-carry state cannot survive a missing weapon node anyway.
         */
        if (rightHandWeaponEquipped && _twoHandedGrip.isPartCarryActive()) {
            rightHandWeaponAuthorityActive = false;
        }
        /*
         * Left-firing carry frees the right hand the same way: the LEFT hand
         * owns the firing grip and the weapon transform, so the right hand has
         * support/free parity (its own part grip is leased separately below).
         */
        if (rightHandWeaponEquipped && _twoHandedGrip.isFiringHandLeft() && _twoHandedGrip.isFiringGripOccupied()) {
            rightHandWeaponAuthorityActive = false;
        }
        const bool rightHandWeaponAuthorityActiveBeforeGrip = rightHandWeaponAuthorityActive;
        bool leftSupportGripActive = false;
        bool rightPartGripActive = _twoHandedGrip.isHandPartGripping(false);
        if (rightHandWeaponAuthorityActive) {
            suppressRightHandCollisionForDominantWeapon(hknp);
        } else {
            restoreRightHandCollisionAfterDominantWeapon(hknp);
        }
        // A part-gripping free hand is a transform driver like the support hand
        // and must not also solve contacts against the weapon package.
        if (rightPartGripActive) {
            suppressHandCollisionForWeaponSupport(hknp, false);
        } else {
            restoreHandCollisionAfterWeaponSupport(hknp, false);
        }

        updateHandCollisions(frame);
        logPalmClockSampleForHand("game-after-hand-collider-queue",
            _rightHand,
            hknp,
            frame.right.disabled ? nullptr : &frame.right.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        logPalmClockSampleForHand("game-after-hand-collider-queue",
            _leftHand,
            hknp,
            frame.left.disabled ? nullptr : &frame.left.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        updateBodyBoneCollisions(frame);
        updateNativePlayerCollisionSuppression(bhk, hknp);

        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollision);

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
            _weaponCollision.update(hknp, weaponNode, frame.deltaSeconds, runtime.weaponDrawn);
        }

        const std::uint64_t currentWeaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();
        const std::uint64_t currentEquippedWeaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        _twoHandedGrip.beginWeaponCollisionPresentationFrame(
            currentWeaponGenerationKey);
        const bool suppressDefaultNativeWeaponIntent =
            _twoHandedGrip.previousWeaponCollisionPresentationWasLive();
        _dynamicWeaponCollision.beginFrame(
            runtime.frameIndex,
            hknp,
            bhk,
            weaponNode,
            currentWeaponGenerationKey,
            g_rockConfig.rockWeaponCollisionEnabled &&
                g_rockConfig.rockWeaponCollisionDynamicBoxEnabled &&
                runtime.weaponDrawn &&
                !frame.menuBlocked &&
                physicsWritesAllowedForWorld(frame.hknpWorld),
            suppressDefaultNativeWeaponIntent);
        reconcileEquippedWeaponHandlingMode();
        serviceEquippedWeaponHandAssignment(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            input_remap_runtime::isMenuInputActive(),
            _equippedWeaponHandlingSettings);
        serviceFixedWeaponHand(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            input_remap_runtime::isMenuInputActive());

        {
            WeaponInteractionContact leftWeaponContact{};
            WeaponInteractionContact rightWeaponContact{};
            auto leftWeaponContactSource = weapon_debug_notification_policy::WeaponContactSource::None;

            auto publishWeaponInteractionContact = [&](bool isLeft, WeaponInteractionContact& contact) {
                auto& partKind = isLeft ? _leftWeaponContactPartKind : _rightWeaponContactPartKind;
                auto& reloadRole = isLeft ? _leftWeaponContactReloadRole : _rightWeaponContactReloadRole;
                auto& supportRole = isLeft ? _leftWeaponContactSupportRole : _rightWeaponContactSupportRole;
                auto& socketRole = isLeft ? _leftWeaponContactSocketRole : _rightWeaponContactSocketRole;
                auto& actionRole = isLeft ? _leftWeaponContactActionRole : _rightWeaponContactActionRole;
                auto& gripPose = isLeft ? _leftWeaponContactGripPose : _rightWeaponContactGripPose;
                auto& sequence = isLeft ? _leftWeaponContactSequence : _rightWeaponContactSequence;
                auto& missedFrames = isLeft ? _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;

                partKind.store(static_cast<std::uint32_t>(contact.partKind), std::memory_order_release);
                reloadRole.store(static_cast<std::uint32_t>(contact.reloadRole), std::memory_order_release);
                supportRole.store(static_cast<std::uint32_t>(contact.supportGripRole), std::memory_order_release);
                socketRole.store(static_cast<std::uint32_t>(contact.socketRole), std::memory_order_release);
                actionRole.store(static_cast<std::uint32_t>(contact.actionRole), std::memory_order_release);
                gripPose.store(static_cast<std::uint32_t>(contact.fallbackGripPose), std::memory_order_release);
                contact.sequence = sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
                missedFrames.store(0, std::memory_order_release);
            };

            auto clearWeaponContactForHand = [&](bool isLeft) {
                if (isLeft) {
                    clearLeftWeaponContact();
                } else {
                    clearRightWeaponContact();
                }
            };

            auto consumeWeaponContactForHand = [&](bool isLeft, const HandFrameInput& handInput, bool probeAllowed, WeaponInteractionContact& outContact) {
                auto& bodyIdAtomic = isLeft ? _leftWeaponContactBodyId : _rightWeaponContactBodyId;
                auto& missedFrames = isLeft ? _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;
                auto& acquisitionState = _weaponInteractionAcquisitionStates[isLeft ? 0u : 1u];

                // Drain the physics-thread notification, but do not use an
                // arbitrary finger/body callback as palm-touch provenance.
                // Touch is the deterministic overlap below for both physical
                // hands and for either firing/support role.
                (void)bodyIdAtomic.exchange(INVALID_CONTACT_BODY_ID, std::memory_order_acquire);

                const RE::NiPoint3 legacyPalmPivotWorld =
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(
                        handInput.rawHandWorld,
                        isLeft);
                const bool touchObserved = weaponNode &&
                    _weaponCollision.tryFindInteractionContactNearPoint(
                        weaponNode,
                        legacyPalmPivotWorld,
                        g_rockConfig.rockWeaponInteractionTouchRadius,
                        outContact);
                if (touchObserved) {
                    publishWeaponInteractionContact(isLeft, outContact);
                } else if (weaponNode && probeAllowed) {
                    if (_weaponCollision.tryFindInteractionContactNearPoint(
                            weaponNode,
                            handInput.grabAnchorWorld,
                            g_rockConfig.rockWeaponInteractionProbeRadius,
                            outContact)) {
                        publishWeaponInteractionContact(isLeft, outContact);
                        if (g_rockConfig.rockDebugVerboseLogging && ++_weaponInteractionProbeLogCounter >= 90) {
                            _weaponInteractionProbeLogCounter = 0;
                            ROCK_LOG_DEBUG(Weapon,
                                "WeaponInteractionProbe: hand={} bodyId={} partKind={} supportRole={} reloadRole={} actionRole={} radius={:.1f}",
                                isLeft ? "left" : "right",
                                outContact.bodyId,
                                static_cast<int>(outContact.partKind),
                                static_cast<int>(outContact.supportGripRole),
                                static_cast<int>(outContact.reloadRole),
                                static_cast<int>(outContact.actionRole),
                                g_rockConfig.rockWeaponInteractionProbeRadius);
                        }
                    } else {
                        const auto missed = missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                        if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                            clearWeaponContactForHand(isLeft);
                        }
                    }
                } else {
                    const auto missed = missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                    if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                        clearWeaponContactForHand(isLeft);
                    }
                }

                outContact.acquisitionSource = weapon_interaction_acquisition_policy::resolve(
                    acquisitionState,
                    touchObserved,
                    outContact.valid);
                switch (outContact.acquisitionSource) {
                case WeaponInteractionAcquisitionSource::PhysicalContact:
                    return weapon_debug_notification_policy::WeaponContactSource::Contact;
                case WeaponInteractionAcquisitionSource::ProximityProbe:
                    return weapon_debug_notification_policy::WeaponContactSource::Probe;
                case WeaponInteractionAcquisitionSource::None:
                default:
                    return weapon_debug_notification_policy::WeaponContactSource::None;
                }
            };

            // A loose-weapon equip carries the originating physical hand into
            // the first equipped frame; use it immediately so input/contact
            // routing never spends a frame under the default right-hand role.
            if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending) {
                _pendingEquippedWeaponPrimaryOnlyGripStart.remainingSeconds -=
                    (std::max)(0.0f, frame.deltaSeconds);
                if (_pendingEquippedWeaponPrimaryOnlyGripStart.remainingSeconds <= 0.0f) {
                    ROCK_LOG_WARN(Weapon,
                        "Held weapon manual ownership handoff expired targetForm={:08X} targetInstance={:#x}",
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponInstanceData);
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                }
            }
            auto* observedEquippedWeapon = currentEquippedWeaponForm();
            const std::uint32_t observedEquippedWeaponFormID =
                observedEquippedWeapon ? observedEquippedWeapon->formID : 0;
            const auto observedEquippedWeaponInstanceData =
                reinterpret_cast<std::uintptr_t>(
                    currentEquippedWeaponInstanceData(observedEquippedWeapon));
            const bool equippedWeaponShoulderStashActive =
                equipped_weapon_drop_policy::equippedWeaponShoulderStashAvailable(
                    _equippedWeaponHandlingSettings.equippedWeaponShoulderStashEnabled);
            const bool inputBlockingMenuActive =
                input_remap_runtime::isMenuInputActive();
            serviceEquippedWeaponShoulderSheathRetrieval(
                frame,
                equippedWeaponShoulderStashActive,
                inputBlockingMenuActive,
                observedEquippedWeaponFormID,
                observedEquippedWeaponInstanceData);
            const bool pendingPrimaryStartMatchesCurrentWeapon =
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                (_pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID == 0 ||
                    equipped_weapon_transition_policy::matchesExpectedIdentity(
                        observedEquippedWeaponFormID,
                        observedEquippedWeaponInstanceData,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponFormID,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.targetWeaponInstanceData,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.previousWeaponFormID,
                        _pendingEquippedWeaponPrimaryOnlyGripStart.previousWeaponInstanceData));
            const bool firingHandIsLeft = pendingPrimaryStartMatchesCurrentWeapon ?
                _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft :
                _twoHandedGrip.isFiringHandLeft();
            const bool supportHandIsLeft = !firingHandIsLeft;

            /*
             * While the LEFT hand carries the weapon, the node still sits at
             * FRIK's offhand glue pose here; the ranked part probes below
             * convert real palm points into node-local space, so glue space
             * made a forend grab select the scope's sight body ~10gu away
             * (fallback wrap pose, grab churn). Publish the canonical carry
             * pose first so both hands probe the weapon where it actually is.
             */
            (void)_twoHandedGrip.publishLeftFiringFeedForwardWeaponPose(weaponNode);

            leftWeaponContactSource = consumeWeaponContactForHand(true, frame.left, weaponNode != nullptr, leftWeaponContact);
            // The free firing hand needs weapon-part probes for part grips and
            // for the reattach squeeze's proximity check, exactly like the
            // offhand; while the LEFT hand fires, the right hand is the
            // support/free hand and probes unconditionally.
            const bool rightWeaponContactProbeAllowed = weaponNode != nullptr &&
                (_twoHandedGrip.isPartCarryActive() || firingHandIsLeft);
            (void)consumeWeaponContactForHand(false, frame.right, rightWeaponContactProbeAllowed, rightWeaponContact);

            const bool gripPressed = readGrabButtonHeld(true, g_rockConfig.rockGrabButtonID);
            const bool rightGripHeld = readGrabButtonHeld(false, g_rockConfig.rockGrabButtonID);
            const bool gripConfirmPressed = readGrabButtonPressedEdge(true, g_rockConfig.rockGrabButtonID);
            (void)gripConfirmPressed;

            WeaponInteractionRuntimeState providerInteractionState{};

            ::rock::provider::RockProviderWeaponPartTargetResolutionV1 weaponPartResolution{};
            const auto weaponPartQuery = makeProviderWeaponPartTargetQuery(leftWeaponContact, _weaponCollision);
            const bool weaponPartResolved = leftWeaponContact.valid &&
                ::rock::provider::resolveWeaponPartTargetV1(weaponPartQuery, weaponPartResolution);
            const bool weaponPartWhitelistActive = weaponPartResolved && weaponPartResolution.whitelistActive != 0;
            const bool weaponPartMatched = weaponPartResolved && weaponPartResolution.matched != 0;
            if (weaponPartWhitelistActive && !weaponPartMatched) {
                providerInteractionState.supportGripAllowed = false;
            } else if (weaponPartMatched) {
                providerInteractionState.providerPartAuthority = makeWeaponProviderPartAuthority(weaponPartQuery, weaponPartResolution);
            }

            WeaponInteractionRuntimeState rightHandInteractionState{};
            ::rock::provider::RockProviderWeaponPartTargetResolutionV1 rightWeaponPartResolution{};
            const auto rightWeaponPartQuery = makeProviderWeaponPartTargetQuery(rightWeaponContact, _weaponCollision);
            const bool rightWeaponPartResolved = rightWeaponContact.valid &&
                ::rock::provider::resolveWeaponPartTargetV1(rightWeaponPartQuery, rightWeaponPartResolution);
            const bool rightWeaponPartWhitelistActive = rightWeaponPartResolved && rightWeaponPartResolution.whitelistActive != 0;
            const bool rightWeaponPartMatched = rightWeaponPartResolved && rightWeaponPartResolution.matched != 0;
            if (rightWeaponPartWhitelistActive && !rightWeaponPartMatched) {
                rightHandInteractionState.supportGripAllowed = false;
            } else if (rightWeaponPartMatched) {
                rightHandInteractionState.providerPartAuthority = makeWeaponProviderPartAuthority(rightWeaponPartQuery, rightWeaponPartResolution);
            }

            /*
             * The offhand reservation is a SUPPORT-ROLE gate, not a physical
             * left-hand gate: it constrains whichever hand currently plays the
             * support role. Part grips by the free firing hand stay gated by
             * the provider part whitelist alone so PAPER reload sessions still
             * constrain which parts the free hand may take.
             */
            const auto offhandReservation = offhand_interaction_reservation::fromProvider(::rock::provider::currentOffhandReservation());
            if (!offhand_interaction_reservation::allowsSupportGrip(offhandReservation)) {
                (supportHandIsLeft ? providerInteractionState : rightHandInteractionState).supportGripAllowed = false;
            }

            const WeaponInteractionDecision leftWeaponDecision = routeWeaponInteraction(leftWeaponContact, providerInteractionState);
            const auto weaponNotificationKey = weapon_debug_notification_policy::makeWeaponNotificationKey(
                leftWeaponContact,
                leftWeaponDecision,
                leftWeaponContactSource);

            const bool leftHandHoldingObject = _leftHand.isHolding();
            auto supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
            bool supportAuthorityProviderOverride = false;
            // The grab-mode override follows the SUPPORT-ROLE hand's provider
            // resolution: that is the hand whose grip the mode describes.
            const bool supportWeaponPartMatched = supportHandIsLeft ? weaponPartMatched : rightWeaponPartMatched;
            const auto& supportWeaponPartResolution = supportHandIsLeft ? weaponPartResolution : rightWeaponPartResolution;
            if (supportWeaponPartMatched) {
                if (supportWeaponPartResolution.grabMode == ::rock::provider::RockProviderWeaponPartGrabModeV1::FullTwoHandAuthority) {
                    supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
                    supportAuthorityProviderOverride = true;
                } else if (supportWeaponPartResolution.grabMode == ::rock::provider::RockProviderWeaponPartGrabModeV1::AttachOnly) {
                    supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport;
                    supportAuthorityProviderOverride = true;
                }
            }
            const bool firingGripProximityAuthorityEnabled = weapon_support_authority_policy::canApplyFiringGripProximityAuthority(
                supportAuthorityProviderOverride);
            EquippedWeaponPrimaryGripInput primaryGripInput{};
            GrabButtonState primaryGrabState{};
            bool primaryGrabStateRead = false;
            _firingHandGrabButtonFrameState = {};
            auto readPrimaryGrabState = [&]() -> const GrabButtonState& {
                if (!primaryGrabStateRead) {
                    primaryGrabState = readGrabButtonState(firingHandIsLeft, g_rockConfig.rockGrabButtonID);
                    // Menu rearm intentionally masks gameplay edges, but
                    // firing-grip ownership still follows the physical hand
                    // state after the menu closes.
                    primaryGrabState.held = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, g_rockConfig.rockGrabButtonID);
                    primaryGrabStateRead = true;
                    // Publish the consumed snapshot so the normal grab pipeline
                    // sees the same edges instead of re-consuming cleared ones.
                    _firingHandGrabButtonFrameState = SharedGrabButtonFrameState{
                        .valid = true,
                        .isLeft = firingHandIsLeft,
                        .held = primaryGrabState.held,
                        .pressed = primaryGrabState.pressed,
                        .released = primaryGrabState.released,
                    };
                }
                return primaryGrabState;
            };
            const bool primaryPoseBlockerAvailable = frik_visual_authority::canBlockPrimaryHandWeaponPose();
            const bool ambidextrousHandoffAvailable =
                _equippedWeaponHandlingSettings.ambidextrousHandoffEnabled &&
                TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
            const bool firingGripOwnershipFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
                !_equippedWeaponShoulderSheath.active &&
                    _equippedWeaponHandlingSettings.firingGripOwnershipEnabled,
                primaryPoseBlockerAvailable,
                weaponNode != nullptr,
                currentEquippedWeaponOwnershipKey);
            const bool primaryDetachFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
                !_equippedWeaponShoulderSheath.active &&
                    _equippedWeaponHandlingSettings.primaryDetachEnabled,
                primaryPoseBlockerAvailable,
                weaponNode != nullptr,
                currentEquippedWeaponOwnershipKey);
            if (inputBlockingMenuActive) {
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            } else if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                !equipped_weapon_manual_ownership_policy::shouldKeepPendingPrimaryOnlyStart(
                    equipped_weapon_manual_ownership_policy::PendingPrimaryOnlyStartInput{
                        .pending = _pendingEquippedWeaponPrimaryOnlyGripStart.pending,
                        .gripHeld = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, g_rockConfig.rockGrabButtonID),
                        .committedTransfer =
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                committedTransfer,
                        .ownershipModeEnabled = _equippedWeaponHandlingSettings.firingGripOwnershipEnabled,
                        .primaryPoseBlockerAvailable = primaryPoseBlockerAvailable,
                    })) {
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            }
            const input_remap_policy::EquippedWeaponFiringGripInputGate firingGripInputGate{
                .featureAvailable = firingGripOwnershipFeatureAvailable,
                .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
                .menuInputActive = inputBlockingMenuActive,
            };
            if (input_remap_policy::shouldConsumeEquippedWeaponFiringGripInput(firingGripInputGate)) {
                const auto& primaryState = readPrimaryGrabState();
                if (input_remap_policy::shouldUseEquippedWeaponFiringGripInput(firingGripInputGate)) {
                    primaryGripInput = EquippedWeaponPrimaryGripInput{
                        .held = primaryState.held,
                        .pressed = primaryState.pressed,
                        .released = primaryState.released,
                    };
                }
            }

            bool primaryOnlyGripStartedThisFrame = false;
            if (firingGripOwnershipFeatureAvailable && !inputBlockingMenuActive && !_twoHandedGrip.isManualOwnershipActive()) {
                const auto& primaryState = readPrimaryGrabState();
                if (_pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                    !primaryState.held &&
                    !_pendingEquippedWeaponPrimaryOnlyGripStart.
                        committedTransfer) {
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                }

                const bool pendingPrimaryOnlyStartRequested =
                    equipped_weapon_manual_ownership_policy::
                        shouldStartPendingPrimaryOnlyGrip(
                            pendingPrimaryStartMatchesCurrentWeapon,
                            primaryState.held,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.
                                committedTransfer);
                const bool primaryOnlyStartRequested =
                    weaponNode != nullptr &&
                    currentEquippedWeaponOwnershipKey != 0 &&
                    ((primaryDetachFeatureAvailable && primaryState.held &&
                         primaryState.pressed) ||
                        pendingPrimaryOnlyStartRequested);
                if (pendingPrimaryStartMatchesCurrentWeapon &&
                    _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft &&
                    (!_pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal ||
                        !_pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal)) {
                    _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal =
                        _twoHandedGrip.tryBuildCurrentLeftFiringGripCapture(
                            weaponNode,
                            currentWeaponGenerationKey,
                            currentEquippedWeaponOwnershipKey,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.firingHandWeaponLocal,
                            _pendingEquippedWeaponPrimaryOnlyGripStart.firingGripWeaponLocal);
                    _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal =
                        _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal;
                }
                const RE::NiTransform* capturedFiringHandWeaponLocal =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                        _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringHandWeaponLocal ?
                    &_pendingEquippedWeaponPrimaryOnlyGripStart.firingHandWeaponLocal :
                    nullptr;
                const RE::NiPoint3* capturedFiringGripWeaponLocal =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                        _pendingEquippedWeaponPrimaryOnlyGripStart.hasFiringGripWeaponLocal ?
                    &_pendingEquippedWeaponPrimaryOnlyGripStart.firingGripWeaponLocal :
                    nullptr;
                const bool committedTransfer =
                    pendingPrimaryStartMatchesCurrentWeapon &&
                    _pendingEquippedWeaponPrimaryOnlyGripStart.
                        committedTransfer;
                if (primaryOnlyStartRequested &&
                    _twoHandedGrip.beginPrimaryOnlyGrip(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        firingHandIsLeft,
                        capturedFiringHandWeaponLocal,
                        capturedFiringGripWeaponLocal,
                        committedTransfer)) {
                    primaryOnlyGripStartedThisFrame = true;
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                    primaryGripInput = EquippedWeaponPrimaryGripInput{
                        .held = primaryState.held,
                        .pressed = primaryState.pressed,
                        .released = primaryState.released,
                    };
                }
            } else if (inputBlockingMenuActive) {
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            }

            std::array<const RE::NiAVObject*, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> drivenSourceNodes{};
            std::size_t drivenSourceNodeCount = 0;
            if (weaponNode) {
                drivenSourceNodeCount = applyProviderWeaponPartDrives(
                    weaponNode,
                    currentWeaponGenerationKey,
                    frame,
                    drivenSourceNodes);
            } else {
                _providerWeaponPartDriveResultCount = 0;
            }

            /*
             * Firing-grip reattach is the squeeze gesture (grab held with the
             * palm on the grip); distance is evaluated by TwoHandedGrip. This
             * only gates whether each free hand may be captured at all -
             * either hand can take the firing grip when ambidextrous takeover
             * is available.
             */
            bool leftReattachEligible = false;
            bool rightReattachEligible = false;
            if (_twoHandedGrip.isPartCarryActive() && primaryDetachFeatureAvailable) {
                leftReattachEligible = weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                    weapon_two_handed_grip_math::FiringGripReattachInput{
                        .partCarryActive = true,
                        .menuInputActive = inputBlockingMenuActive,
                        .handHoldingObject = _leftHand.isHolding(),
                    });
                rightReattachEligible = weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                    weapon_two_handed_grip_math::FiringGripReattachInput{
                        .partCarryActive = true,
                        .menuInputActive = inputBlockingMenuActive,
                        .handHoldingObject = _rightHand.isHolding(),
                    });
            }

            /*
             * Equipped-weapon shoulder stash: evaluated before update() so the
             * release frame has a fresh in-zone decision. Provider-owned
             * physical detach follows the single manual carry hand; ROCK's
             * native path follows the current firing hand without granting
             * world-drop authority. The idle hand is always reset so stale
             * dwell can never confirm a later release.
             */
            std::array<shoulder_stash::Decision, 2> equippedWeaponStashCommitDecisions{};
            bool nativeShoulderSheathRequested = false;
            auto nativeShoulderSheathSourceHand =
                equipped_weapon_drop_policy::SourceHand::None;
            {
                // Carry-authority grips remain authoritative for provider
                // detach mode. Without that mode, the natively attached
                // firing hand owns ROCK's dedicated shoulder release gesture.
                const auto manualStashCarryHand =
                    equipped_weapon_drop_policy::resolveEquippedWeaponStashCarryHand(
                        _twoHandedGrip.isPrimaryOnlyActive(),
                        _twoHandedGrip.isPartCarryActive(),
                        _twoHandedGrip.isHandPartCarryGripping(true),
                        _twoHandedGrip.isHandPartCarryGripping(false),
                        _twoHandedGrip.isFiringHandLeft());
                const bool nativeShoulderGestureAvailable =
                    equippedWeaponShoulderStashActive &&
                    !_equippedWeaponHandlingSettings.primaryDetachEnabled;
                const auto nativeShoulderGestureHand = firingHandIsLeft ?
                    equipped_weapon_drop_policy::SourceHand::Left :
                    equipped_weapon_drop_policy::SourceHand::Right;
                auto stashCarryHand =
                    equipped_weapon_drop_policy::SourceHand::None;
                if (equippedWeaponShoulderStashActive) {
                    stashCarryHand = manualStashCarryHand !=
                            equipped_weapon_drop_policy::SourceHand::None ?
                        manualStashCarryHand :
                        nativeShoulderGestureAvailable ?
                        nativeShoulderGestureHand :
                        equipped_weapon_drop_policy::SourceHand::None;
                }
                const bool stashCarryEligible = !inputBlockingMenuActive &&
                                                stashCarryHand != equipped_weapon_drop_policy::SourceHand::None;
                for (const bool stashHandIsLeft : { true, false }) {
                    const std::size_t stashHandIndex = stashHandIsLeft ? 1u : 0u;
                    auto& stashState = _equippedWeaponStashStates[stashHandIndex];
                    auto& commitLease = _equippedWeaponStashCommitLeases[stashHandIndex];
                    if (!stashCarryEligible || equipped_weapon_drop_policy::isLeft(stashCarryHand) != stashHandIsLeft) {
                        shoulder_stash::resetRuntime(stashState);
                        commitLease = {};
                        continue;
                    }

                    const HandFrameInput& carryInput = stashHandIsLeft ? frame.left : frame.right;
                    const auto stashConfig = makeEquippedWeaponStashDetectorConfig(equippedWeaponShoulderStashActive);
                    shoulder_stash::DetectorInput stashInput{
                            .isLeftHand = stashHandIsLeft,
                            .probe = shoulder_stash::Probe{ .pointGame = carryInput.grabAnchorWorld },
                            .hmdProbe = makeShoulderStashHmdProbe(carryInput),
                            .hasHmdProbe = true,
                            .hasHmdFrame = frame.hasHmdFrame,
                            .hmdPositionWorld = frame.hmdPositionWorld,
                            .hmdForwardWorld = frame.hmdForwardWorld,
                            .deltaSeconds = frame.deltaSeconds,
                            .config = stashConfig,
                        };
                    const shoulder_stash::RuntimeState stashStateBeforeEvaluation = stashState;
                    const auto stashDecision = shoulder_stash::evaluate(stashInput, stashState);
                    equippedWeaponStashCommitDecisions[stashHandIndex] = stashDecision;

                    const bool gripPhysicallyHeld =
                        input_remap_runtime::isRawButtonPhysicallyHeld(stashHandIsLeft, g_rockConfig.rockGrabButtonID);
                    if (gripPhysicallyHeld) {
                        commitLease = {};
                    } else if (!stashDecision.confirmedForCommit) {
                        const bool speedLimitExceeded =
                            shoulder_stash::exceedsShoulderStashSpeedLimit(
                                stashDecision.speedGameUnitsPerSecond,
                                stashConfig.maxSpeedGameUnitsPerSecond);
                        const bool canArmFastReleaseLease = stashStateBeforeEvaluation.confirmed && speedLimitExceeded;
                        if (commitLease.active || canArmFastReleaseLease) {
                            /*
                             * The normal detector remains the speed authority.
                             * A speed-unlimited copy is used only to prove that
                             * the already-dwelled hand stayed in the same back
                             * volume during the two-frame physical release
                             * debounce; it cannot acquire a new stash candidate.
                             */
                            auto spatialInput = stashInput;
                            spatialInput.config.maxSpeedGameUnitsPerSecond = 0.0f;
                            auto spatialState = commitLease.active ? commitLease.spatialState : stashStateBeforeEvaluation;
                            const auto spatialDecision = shoulder_stash::evaluate(spatialInput, spatialState);
                            const auto expectedZone = commitLease.active ? commitLease.zone : stashStateBeforeEvaluation.zone;
                            const auto expectedSource = commitLease.active ? commitLease.source : stashStateBeforeEvaluation.source;
                            const bool sameSpatialCandidate =
                                spatialDecision.candidate &&
                                spatialDecision.zone == expectedZone &&
                                spatialDecision.source == expectedSource;

                            if (!commitLease.active &&
                                shoulder_stash::shouldArmEquippedWeaponFastReleaseCommitLease(
                                    stashStateBeforeEvaluation.confirmed,
                                    speedLimitExceeded,
                                    gripPhysicallyHeld,
                                    sameSpatialCandidate)) {
                                commitLease.active = true;
                                commitLease.ownershipKey = currentEquippedWeaponOwnershipKey;
                                commitLease.remainingOpenFrames =
                                    equipped_weapon_manual_ownership_policy::kPrimaryReleaseConfirmFrames;
                                commitLease.zone = spatialDecision.zone;
                                commitLease.source = spatialDecision.source;
                            }

                            if (shoulder_stash::equippedWeaponFastReleaseCommitLeaseIsUsable(
                                    commitLease.active,
                                    commitLease.ownershipKey,
                                    currentEquippedWeaponOwnershipKey,
                                    commitLease.remainingOpenFrames,
                                    gripPhysicallyHeld,
                                    sameSpatialCandidate)) {
                                commitLease.spatialState = spatialState;
                                equippedWeaponStashCommitDecisions[stashHandIndex] = spatialDecision;
                                equippedWeaponStashCommitDecisions[stashHandIndex].confirmedForCommit = true;
                                --commitLease.remainingOpenFrames;
                            } else {
                                commitLease = {};
                            }
                        }
                    } else {
                        commitLease = {};
                    }

                    if (nativeShoulderGestureAvailable &&
                        stashHandIsLeft == firingHandIsLeft &&
                        equippedWeaponStashCommitDecisions[stashHandIndex].
                            confirmedForCommit) {
                        Hand& stashHand = stashHandIsLeft ?
                            _leftHand : _rightHand;
                        const bool stashHandEmpty =
                            !stashHand.isHolding() &&
                            !_touchGrabRuntime.isHandActive(
                                stashHandIsLeft) &&
                            !_pendingForceGrabCommits[stashHandIndex].active &&
                            !stashHand.hasActivePullCatchIntent() &&
                            !stashHand.
                                hasPendingActorEquipmentDropHandoff();
                        const auto& primaryState = readPrimaryGrabState();
                        nativeShoulderSheathRequested =
                            equipped_weapon_drop_policy::
                                canCommitNativeShoulderSheath(
                                    equipped_weapon_drop_policy::
                                        NativeShoulderSheathInput{
                                            .handlingEnabled =
                                                equippedWeaponShoulderStashActive,
                                            .primaryDetachEnabled =
                                                _equippedWeaponHandlingSettings.
                                                    primaryDetachEnabled,
                                            .weaponAvailable =
                                                weaponNode != nullptr &&
                                                currentEquippedWeaponOwnershipKey != 0,
                                            .menuInputActive =
                                                inputBlockingMenuActive,
                                            .handDisabled =
                                                carryInput.disabled,
                                            .handEmpty = stashHandEmpty,
                                            .detectorConfirmed = true,
                                            .gripReleased =
                                                primaryState.released,
                                        });
                        if (nativeShoulderSheathRequested) {
                            nativeShoulderSheathSourceHand =
                                nativeShoulderGestureHand;
                        }
                    }

                    if (stashDecision.candidate && g_rockConfig.rockShoulderStashHapticsEnabled) {
                        const bool pulseDue = _dynamicPushElapsedSeconds >= stashState.nextCandidatePulseTimeSeconds;
                        if (stashDecision.enteredCandidate || stashDecision.changedCandidate || pulseDue) {
                            (void)_feedbackHaptics.queue(
                                stashHandIsLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                                g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
                                shoulder_stash_haptic_policy::computeCandidatePulseIntensity(stashDecision.confidence,
                                    shoulder_stash_haptic_policy::CandidatePulseConfig{
                                        .enabled = true,
                                        .baseIntensity = g_rockConfig.rockShoulderStashCandidateHapticBaseIntensity,
                                        .maxIntensity = g_rockConfig.rockShoulderStashCandidateHapticIntensity,
                                    }));
                            stashState.nextCandidatePulseTimeSeconds =
                                _dynamicPushElapsedSeconds + (std::max)(0.02f, g_rockConfig.rockShoulderStashCandidateHapticIntervalSeconds);
                        }
                    }
                }
            }

            const auto captureScopeHandDriverFrame = [](RE::NiNode* driverNode) {
                EquippedWeaponScopeHandDriverFrame result{};
                if (driverNode && finiteNiTransform(driverNode->world)) {
                    result.valid = true;
                    result.world = driverNode->world;
                }
                return result;
            };
            auto* playerNodes = f4vr::getPlayerNodes();
            const auto scopeHandDriverNode = [playerNodes](bool isLeft) -> RE::NiNode* {
                if (!playerNodes) {
                    return nullptr;
                }
                return isLeft ?
                    playerNodes->SecondaryMeleeWeaponOffsetNode2 :
                    playerNodes->primaryWeaponOffsetNOde;
            };
            const EquippedWeaponScopeHandDriverFrame leftHandDriverFrame{
                !frame.left.disabled && finiteNiTransform(frame.left.rawHandWorld),
                frame.left.rawHandWorld,
            };
            const EquippedWeaponScopeHandDriverFrame rightHandDriverFrame{
                !frame.right.disabled && finiteNiTransform(frame.right.rawHandWorld),
                frame.right.rawHandWorld,
            };
            const EquippedWeaponScopeHandDriverFrame leftScopeHandDriverFrame = captureScopeHandDriverFrame(scopeHandDriverNode(true));
            const EquippedWeaponScopeHandDriverFrame rightScopeHandDriverFrame = captureScopeHandDriverFrame(scopeHandDriverNode(false));
            bool nativeScopeRequestActive = false;
            const bool nativeScopeRequestStateValid =
                tryReadNativeScopeRequestState(nativeScopeRequestActive);
            const bool manualScopeActivationRequested =
                input_remap_runtime::isManualScopeActivationRequested();
            const bool gunstockObservationActive =
                g_rockConfig.rockGunstockModeEnabled ||
                g_rockConfig.rockDebugDrawGunstockAlignment;
            RE::NiAVObject* gunstockProjectileNode =
                gunstockObservationActive ?
                getEquippedProjectileNode() :
                nullptr;
            /*
             * FO4VR 1.2.72 binary verification (2026-08-06): native
             * TESObjectWEAP paths at 0x14033FF00 and 0x140334260 read the type
             * byte at object +0x2CF; the native type-label table's index 9 is
             * referenced by CombatBehaviorTreeGun. Pair that kGun witness with
             * the collision observer's form boundary before it may establish
             * generation-latched gunstock eligibility.
             */
            const bool gunstockGunTypeObserved =
                gunstockObservationActive &&
                observedEquippedWeapon &&
                currentWeaponGenerationKey != 0 &&
                _weaponCollision.getCurrentObservedEquippedWeaponFormID() ==
                    observedEquippedWeapon->formID &&
                observedEquippedWeapon->weaponData.type ==
                    RE::WEAPON_TYPE::kGun;

            const EquippedWeaponGripFrameInput gripFrameInput{
                .leftGripHeld = gripPressed,
                .rightGripHeld = rightGripHeld,
                .leftHandHoldingObject = leftHandHoldingObject,
                .rightHandHoldingObject = _rightHand.isHolding(),
                .leftReattachEligible = leftReattachEligible,
                .rightReattachEligible = rightReattachEligible,
                .scopeMenuOpen = runtime.localScopeMenuOpen,
                .manualScopeActivationRequested = manualScopeActivationRequested,
                .nativeScopeRequestStateValid = nativeScopeRequestStateValid,
                .nativeScopeRequestActive = nativeScopeRequestActive,
                .nativeReloadHandAuthorityActive =
                    frame.reloadBoundaryActive,
                .gunstockPresentationBlocked =
                    frame.menuBlocked || frame.reloadBoundaryActive,
                .leftHandDriverFrame = leftHandDriverFrame,
                .rightHandDriverFrame = rightHandDriverFrame,
                .leftScopeHandDriverFrame = leftScopeHandDriverFrame,
                .rightScopeHandDriverFrame = rightScopeHandDriverFrame,
                .primaryGripInput = primaryGripInput,
            };
            auto effectiveHandlingSettings = _equippedWeaponHandlingSettings;
            effectiveHandlingSettings.firingGripOwnershipEnabled =
                firingGripOwnershipFeatureAvailable;
            effectiveHandlingSettings.ambidextrousHandoffEnabled =
                ambidextrousHandoffAvailable;
            effectiveHandlingSettings.primaryDetachEnabled =
                primaryDetachFeatureAvailable;
            _twoHandedGrip.update(
                weaponNode,
                gunstockProjectileNode,
                gunstockGunTypeObserved,
                leftWeaponContact,
                rightWeaponContact,
                gripFrameInput,
                frame.deltaSeconds,
                _currentPreFrikSchedulerSequence,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                _weaponCollision,
                providerInteractionState,
                rightHandInteractionState,
                supportAuthorityMode,
                firingGripProximityAuthorityEnabled,
                effectiveHandlingSettings);
            synchronizeDynamicWeaponHandCollisionRoles(hknp);
            const bool gunstockNeutralSampleBlocked =
                g_rockConfig.rockGunstockModeEnabled &&
                (input_remap_runtime::isRawButtonPhysicallyHeld(
                     _twoHandedGrip.isFiringHandLeft(),
                     input_remap_policy::
                         kOpenVrSteamVrTriggerButtonId) ||
                    frame.reloadBoundaryActive);
            const bool gunstockPresentationBlocked =
                frame.menuBlocked || frame.reloadBoundaryActive;
            _twoHandedGrip.prepareGunstockAlignmentDebugSnapshot(
                weaponNode,
                gunstockProjectileNode,
                _weaponCollision.
                    getCurrentObservedEquippedWeaponFormID(),
                currentWeaponGenerationKey,
                gunstockNeutralSampleBlocked,
                gunstockPresentationBlocked);
            (void)_twoHandedGrip.applyGunstockAlignment(
                weaponNode,
                gunstockProjectileNode,
                currentWeaponGenerationKey,
                gunstockNeutralSampleBlocked,
                gunstockPresentationBlocked);
            reconcileEquippedWeaponHandAssignmentAfterGrip();
            if (_twoHandedGrip.hasVisualAuthorityForHand(false)) {
                _rightHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
            }
            if (_twoHandedGrip.hasVisualAuthorityForHand(true)) {
                _leftHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
            }
            if (primaryOnlyGripStartedThisFrame) {
                ROCK_LOG_DEBUG(Weapon, "Equipped weapon firing-grip ownership started from grip input or held-weapon equip");
            }
            const auto gripHapticEvents = _twoHandedGrip.consumeHapticEvents();
            const auto queueGripHaptic = [this](bool isLeft, float intensity) {
                (void)_feedbackHaptics.queue(
                    isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    _equippedWeaponHandlingSettings.weaponGripHapticDurationSeconds,
                    intensity);
            };
            if (_equippedWeaponHandlingSettings.externalAuthorityActive) {
                if (gripHapticEvents.firingGripAttached) {
                    queueGripHaptic(
                        gripHapticEvents.firingGripAttachedHandIsLeft,
                        _equippedWeaponHandlingSettings.firingGripAttachHapticIntensity);
                }
                if (gripHapticEvents.firingGripDetached) {
                    queueGripHaptic(
                        gripHapticEvents.firingGripDetachedHandIsLeft,
                        _equippedWeaponHandlingSettings.firingGripDetachHapticIntensity);
                }
                if (gripHapticEvents.leftPartGripCaptured) {
                    queueGripHaptic(
                        true,
                        _equippedWeaponHandlingSettings.supportGripHapticIntensity);
                }
                if (gripHapticEvents.rightPartGripCaptured) {
                    queueGripHaptic(
                        false,
                        _equippedWeaponHandlingSettings.supportGripHapticIntensity);
                }
            }
            /*
             * Continuous hover feedback while the open firing palm sits inside
             * the reattach radius during part carry: re-queued every frame so
             * the vibration holds until the squeeze reattaches (which flips
             * the state and hands off to the firingGripAttached pulse above).
             */
            if (_equippedWeaponHandlingSettings.gripZoneHoverHapticsEnabled &&
                _twoHandedGrip.isFiringGripReattachHoverInsideRadius()) {
                (void)_feedbackHaptics.queue(
                    _twoHandedGrip.isFiringGripReattachHoverHandLeft() ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                    _equippedWeaponHandlingSettings.gripZoneHoverHapticIntensity);
            }
            bool nativeShoulderSheathSelected = false;
            if (nativeShoulderSheathRequested &&
                nativeShoulderSheathSourceHand !=
                    equipped_weapon_drop_policy::SourceHand::None) {
                nativeShoulderSheathSelected = true;
                const bool sheathHandIsLeft =
                    equipped_weapon_drop_policy::isLeft(
                        nativeShoulderSheathSourceHand);
                const std::size_t sheathHandIndex =
                    sheathHandIsLeft ? 1u : 0u;
                _equippedWeaponSheathCommittedThisFrame[sheathHandIndex] = true;
                (void)submitEquippedWeaponShoulderSheath(
                    observedEquippedWeaponFormID,
                    observedEquippedWeaponInstanceData,
                    nativeShoulderSheathSourceHand,
                    equippedWeaponStashCommitDecisions[sheathHandIndex],
                    weaponNode,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey);
            }
            const auto equippedWeaponDropRequest = _twoHandedGrip.consumeEquippedWeaponDropRequest();
            if (equippedWeaponDropRequest.requested) {
                const auto sourceHand = equippedWeaponDropRequest.sourceHand;
                const bool sourceHandKnown = sourceHand == equipped_weapon_drop_policy::SourceHand::Right ||
                                              sourceHand == equipped_weapon_drop_policy::SourceHand::Left;
                const RE::NiPoint3 dropLoc = sourceHandKnown ?
                                                (equipped_weapon_drop_policy::isLeft(sourceHand) ? frame.left.grabAnchorWorld : frame.right.grabAnchorWorld) :
                                                (weaponNode ? weaponNode->world.translate : frame.right.grabAnchorWorld);
                if (inputBlockingMenuActive) {
                    ROCK_LOG_INFO(Weapon,
                        "Equipped weapon manual release suppressed because an input-blocking menu is active sourceHand={} releaseLoc=({:.1f},{:.1f},{:.1f})",
                        equipped_weapon_drop_policy::sourceHandName(sourceHand),
                        dropLoc.x,
                        dropLoc.y,
                        dropLoc.z);
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                    clearEquippedWeaponFiringGripInputState();
                } else {
                    const std::size_t sourceHandIndex =
                        sourceHandKnown &&
                            equipped_weapon_drop_policy::isLeft(sourceHand) ?
                        1u : 0u;
                    const bool manualStashCommitSelected =
                        sourceHandKnown &&
                        equippedWeaponShoulderStashActive &&
                        equippedWeaponStashCommitDecisions[sourceHandIndex].
                            confirmedForCommit;
                    const bool stashCommitSelected =
                        nativeShoulderSheathSelected ||
                        manualStashCommitSelected;
                    if (manualStashCommitSelected &&
                        !nativeShoulderSheathSelected) {
                        /*
                         * Native sheathe is terminal for this release gesture. The
                         * exact equipped stack remains equipped and no world
                         * reference is created. Once this action is selected, a
                         * failed native transition must never become a world drop.
                         */
                        (void)submitEquippedWeaponShoulderSheath(
                            observedEquippedWeaponFormID,
                            observedEquippedWeaponInstanceData,
                            sourceHand,
                            equippedWeaponStashCommitDecisions[
                                sourceHandIndex],
                            weaponNode,
                            currentWeaponGenerationKey,
                            currentEquippedWeaponOwnershipKey);
                    }
                    const bool physicalDropRequested =
                        equipped_weapon_drop_policy::shouldAttemptPhysicalDrop(stashCommitSelected);
                    const bool dropHandoffAvailable = hasAvailableEquippedWeaponDropHandoff();
                    if (physicalDropRequested && !dropHandoffAvailable) {
                        ROCK_LOG_WARN(Weapon,
                            "Equipped weapon physical drop blocked because all native handoffs are active: capacity={}",
                            _equippedWeaponDropMomentumHandoffs.size());
                        f4vr::showNotification("ROCK: Cannot drop weapon - drop handoff queue is full.");
                    }
                    if (physicalDropRequested && dropHandoffAvailable) {
                        /*
                         * Seamless drop: spawn the world ref at the weapon's last
                         * visually-published pose (equipped and dropped weapons
                         * share the same nif) and hand the captured release
                         * momentum to the spawned physics bodies once they
                         * resolve. The previous-frame capture is preferred over
                         * the live node because the release transition restores
                         * the weapon node to the FRIK hand baseline before this
                         * code runs.
                         */
                        RE::NiPoint3 releaseLoc = dropLoc;
                        RE::NiPoint3 releaseRot{};
                        RE::NiTransform releaseWeaponWorld{};
                        bool hasReleaseRot = false;
                        if (_equippedWeaponReleaseCapture.hasWeaponWorld &&
                            finiteNiTransform(_equippedWeaponReleaseCapture.weaponWorld)) {
                            releaseWeaponWorld = _equippedWeaponReleaseCapture.weaponWorld;
                            releaseLoc = _equippedWeaponReleaseCapture.weaponWorld.translate;
                            releaseRot = grab_node_info_math::nifskopeMatrixToEulerRadians<RE::NiMatrix3, RE::NiPoint3>(_equippedWeaponReleaseCapture.weaponWorld.rotate);
                            hasReleaseRot = true;
                        } else if (weaponNode && finiteNiTransform(weaponNode->world)) {
                            releaseWeaponWorld = weaponNode->world;
                            releaseLoc = weaponNode->world.translate;
                            releaseRot = grab_node_info_math::nifskopeMatrixToEulerRadians<RE::NiMatrix3, RE::NiPoint3>(weaponNode->world.rotate);
                            hasReleaseRot = true;
                        }
                        const std::size_t releaseHandIndex = equipped_weapon_drop_policy::isLeft(sourceHand) ? 1u : 0u;
                        const auto& releaseHandInput = releaseHandIndex == 1u ? frame.left : frame.right;
                        const RE::NiPoint3 releaseGripWorld = _equippedWeaponReleaseCapture.hasPreviousHandWorld[releaseHandIndex] ?
                                                                 _equippedWeaponReleaseCapture.previousHandWorld[releaseHandIndex].translate :
                                                                 releaseHandInput.grabAnchorWorld;
                        // Consume the equipped body's generated points before
                        // the drop transaction retires that bank. They only
                        // bound long-object angular release speed; the frozen
                        // transform is the native body's placement authority.
                        const auto releaseGeometry = hasReleaseRot ?
                                                         _weaponCollision.getCurrentWeaponReleaseGeometry(releaseGripWorld, releaseWeaponWorld) :
                                                         WeaponCollision::ReleaseGeometrySnapshot{};
                        if (!releaseGeometry.hasCapturedWeaponWorld) {
                            ROCK_LOG_WARN(Weapon,
                                "Equipped weapon physical drop blocked because no finite frozen release pose is available: sourceHand={}",
                                equipped_weapon_drop_policy::sourceHandName(sourceHand));
                            f4vr::showNotification("ROCK: Cannot drop weapon - release pose is not ready.");
                        } else {
                            const auto dropResult = weapon_equip_transfer::dropEquippedWeaponFromPlayer(weapon_equip_transfer::EquippedDropInput{
                                .dropLoc = releaseLoc,
                                .dropRot = releaseRot,
                                .hasDropLoc = true,
                                .hasDropRot = true,
                            });
                            const bool dropCommitted = equipped_weapon_drop_policy::physicalDropCommitted(
                                equipped_weapon_drop_policy::PhysicalDropCommitInput{
                                    .dropSucceeded = dropResult.success,
                                    .droppedReferenceUnavailable =
                                        dropResult.reason == weapon_equip_transfer::DropReason::DroppedReferenceUnavailable,
                                });
                            if (dropCommitted) {
                                enforceNoBareFistState(true);
                                /*
                                 * RemoveItem creates the native layer-5 weapon at
                                 * the last layer-44 equipped-collider pose. Retire
                                 * ROCK's generated representation in this same
                                 * transaction so no physics step can solve the two
                                 * coincident weapon body sets before the native
                                 * handoff takes ownership.
                                 */
                                _weaponCollision.destroyWeaponBody(hknp);
                            }
                            if (dropCommitted && dropResult.handle) {
                                armEquippedWeaponDropMomentumHandoff(
                                    dropResult.handle,
                                    dropResult.droppedFormID,
                                    sourceHand,
                                    releaseGeometry);
                            }
                            if (dropCommitted) {
                                ROCK_LOG_INFO(Weapon,
                                    "Equipped weapon manual release committed formID={:08X} dropped={:08X} reference={} sourceHand={} dropLoc=({:.1f},{:.1f},{:.1f}) lever={:.1f}gu stack={} instanceMatch={}",
                                    dropResult.formID,
                                    dropResult.droppedFormID,
                                    dropResult.success ? "ready" : "pending",
                                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                                    releaseLoc.x,
                                    releaseLoc.y,
                                    releaseLoc.z,
                                    releaseGeometry.leverGameUnits,
                                    dropResult.stackID,
                                    dropResult.matchedInstanceData ? "yes" : "no");
                            } else {
                                ROCK_LOG_WARN(Weapon,
                                    "Equipped weapon manual release drop failed formID={:08X} reason={} sourceHand={} attempted={} stack={} instanceMatch={}",
                                    dropResult.formID,
                                    weapon_equip_transfer::dropReasonName(dropResult.reason),
                                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                                    dropResult.attempted ? "yes" : "no",
                                    dropResult.stackID,
                                    dropResult.matchedInstanceData ? "yes" : "no");
                            }
                            if (sourceHandKnown && dropCommitted) {
                                suppressHandCollisionAfterEquippedWeaponDrop(hknp, sourceHand);
                            }
                        }
                    }
                    _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                    clearEquippedWeaponFiringGripInputState();
                }
            }
            updateEquippedWeaponReleaseCapture(frame, weaponNode);
            const bool weaponSupportGripActive = _twoHandedGrip.isHandPartGripping(true);
            const input_remap_policy::EquippedWeaponFiringGripInputGate updatedFiringGripInputGate{
                .featureAvailable = firingGripOwnershipFeatureAvailable,
                .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
                .menuInputActive = inputBlockingMenuActive,
            };
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(
                input_remap_policy::shouldUseEquippedWeaponFiringGripInput(updatedFiringGripInputGate));
            input_remap_runtime::setEquippedWeaponPrimaryDetached(_twoHandedGrip.isPartCarryActive());
            /*
             * Left-hand fire publication: while the LEFT hand occupies the
             * firing grip, the OpenVR-level trigger remap presents the left
             * trigger to the game as the primary (right) wand's trigger.
             */
            const bool leftHandFiringActiveAfterGrip = _twoHandedGrip.isFiringHandLeft() && _twoHandedGrip.isFiringGripOccupied();
            input_remap_runtime::setEquippedWeaponLeftHandFiringActive(leftHandFiringActiveAfterGrip);
            ::rock::provider::setEquippedWeaponFiringHandIsLeft(_twoHandedGrip.isFiringHandLeft());

            bool rightHandWeaponAuthorityActiveAfterGrip = rightHandWeaponEquipped || retainedWeaponCollisionActive;
            // A visible part-carry frees the right hand even while weapon bodies exist (see the pre-grip gate).
            if (rightHandWeaponEquipped && _twoHandedGrip.isPartCarryActive()) {
                rightHandWeaponAuthorityActiveAfterGrip = false;
            }
            // Left-firing carry frees the right hand the same way (see the pre-grip gate).
            if (rightHandWeaponEquipped && leftHandFiringActiveAfterGrip) {
                rightHandWeaponAuthorityActiveAfterGrip = false;
            }
            if (rightHandWeaponAuthorityActiveAfterGrip != rightHandWeaponAuthorityActiveBeforeGrip) {
                if (rightHandWeaponAuthorityActiveAfterGrip) {
                    suppressRightHandCollisionForDominantWeapon(hknp);
                } else {
                    restoreRightHandCollisionAfterDominantWeapon(hknp);
                }
            }
            rightHandWeaponAuthorityActive = rightHandWeaponAuthorityActiveAfterGrip;
            const bool rightPartGripActiveAfterGrip = _twoHandedGrip.isHandPartGripping(false);
            if (rightPartGripActiveAfterGrip != rightPartGripActive) {
                if (rightPartGripActiveAfterGrip) {
                    suppressHandCollisionForWeaponSupport(hknp, false);
                } else {
                    restoreHandCollisionAfterWeaponSupport(hknp, false);
                }
            }
            rightPartGripActive = rightPartGripActiveAfterGrip;

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

            /*
             * A LEFT hand occupying the firing grip is weapon-engaged exactly
             * like a support hand from the collision standpoint: its generated
             * colliders must not become a second physical owner while the
             * weapon rides the hand. Reuses the per-hand support lease.
             */
            if (weaponSupportGripActive || leftHandFiringActiveAfterGrip) {
                suppressHandCollisionForWeaponSupport(hknp, true);
            } else {
                restoreHandCollisionAfterWeaponSupport(hknp, true);
            }

            const auto dynamicWeaponFrame =
                _dynamicWeaponCollision.finishFrame(
                    frame,
                    physicsWritesAllowedForWorld(frame.hknpWorld),
                    weaponNode,
                    currentWeaponGenerationKey,
                    _weaponCollision);
            if (dynamicWeaponFrame.contactEpisodeStarted &&
                g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
                auto* otherRef = resolveBodyToRef(
                    frame.bhkWorld,
                    frame.hknpWorld,
                    RE::hknpBodyId{ dynamicWeaponFrame.otherBodyId });
                const auto* otherBase = otherRef ? otherRef->GetObjectReference() : nullptr;
                const auto otherNameView = otherBase ?
                    RE::TESFullName::GetFullName(*otherBase, false) :
                    std::string_view{};
                const std::string otherName = otherNameView.empty() ?
                    std::string("(unresolved)") :
                    std::string(otherNameView);
                const char* otherType = otherBase ?
                    otherBase->GetFormTypeString() :
                    "unresolved";

                WeaponCollision::WeaponSurfaceProximityWitness partWitness{};
                WeaponInteractionDebugInfo partInfo{};
                constexpr float kContactPartSearchRadiusGameUnits = 24.0f;
                const bool partWitnessValid =
                    dynamicWeaponFrame.rawContactPointValid &&
                    _weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(
                        weaponNode,
                        dynamicWeaponFrame.rawContactPointGame,
                        kContactPartSearchRadiusGameUnits,
                        partWitness);
                const bool partInfoValid =
                    partWitnessValid &&
                    _weaponCollision.tryGetWeaponContactDebugInfo(
                        partWitness.bodyId,
                        partInfo);

                ROCK_LOG_INFO(
                    Weapon,
                    "DWC contact witness: episode={} solveAge={} generation={:016X} weaponForm={:08X} other(body/layer/motion/ref/form/type/name)=({}/{}/{}/{:p}/{:08X}/{}/{}) native(collObj/owner)=({:p}/{:p}) raw(valid/proxyWasA/points/index/weight)={}/{}/{}/{}/{:.3f} pointGame=({:.2f},{:.2f},{:.2f}) normalRaw=({:.3f},{:.3f},{:.3f}) nearestPart(valid/body/source/distance/current)={}/{}/{}/{:.3f}/{}",
                    dynamicWeaponFrame.contactEpisode,
                    dynamicWeaponFrame.contactSolveAge,
                    currentWeaponGenerationKey,
                    _weaponCollision.getCurrentObservedEquippedWeaponFormID(),
                    dynamicWeaponFrame.otherBodyId,
                    dynamicWeaponFrame.otherLayer,
                    dynamicWeaponFrame.otherMotionIndex,
                    static_cast<void*>(otherRef),
                    otherRef ? otherRef->GetFormID() : 0,
                    otherType,
                    otherName,
                    reinterpret_cast<void*>(dynamicWeaponFrame.otherCollisionObject),
                    reinterpret_cast<void*>(dynamicWeaponFrame.otherOwnerNode),
                    dynamicWeaponFrame.rawContactPointValid,
                    dynamicWeaponFrame.rawContactProxyWasBodyA,
                    dynamicWeaponFrame.rawContactPointCount,
                    dynamicWeaponFrame.rawContactPointIndex,
                    dynamicWeaponFrame.rawContactPointWeightSum,
                    dynamicWeaponFrame.rawContactPointGame.x,
                    dynamicWeaponFrame.rawContactPointGame.y,
                    dynamicWeaponFrame.rawContactPointGame.z,
                    dynamicWeaponFrame.rawContactNormalHavok.x,
                    dynamicWeaponFrame.rawContactNormalHavok.y,
                    dynamicWeaponFrame.rawContactNormalHavok.z,
                    partWitnessValid,
                    partWitnessValid ? partWitness.bodyId : 0x7FFF'FFFFu,
                    partInfoValid ? partInfo.sourceName : std::string("(unresolved)"),
                    partWitnessValid ? partWitness.distanceGameUnits : -1.0f,
                    partWitnessValid && partWitness.sourceNodeCurrent);
                ROCK_LOG_INFO(
                    Weapon,
                    "DWC contact transforms: episode={} requestedBody=({:.2f},{:.2f},{:.2f}) liveBody=({:.2f},{:.2f},{:.2f}) otherReadable={} otherBody=({:.2f},{:.2f},{:.2f}) correction=({:.2f}gu,{:.2f}deg)",
                    dynamicWeaponFrame.contactEpisode,
                    dynamicWeaponFrame.requestedContactBodyWorld.translate.x,
                    dynamicWeaponFrame.requestedContactBodyWorld.translate.y,
                    dynamicWeaponFrame.requestedContactBodyWorld.translate.z,
                    dynamicWeaponFrame.liveContactBodyWorld.translate.x,
                    dynamicWeaponFrame.liveContactBodyWorld.translate.y,
                    dynamicWeaponFrame.liveContactBodyWorld.translate.z,
                    dynamicWeaponFrame.otherBodyWorldValid,
                    dynamicWeaponFrame.otherBodyWorld.translate.x,
                    dynamicWeaponFrame.otherBodyWorld.translate.y,
                    dynamicWeaponFrame.otherBodyWorld.translate.z,
                    dynamicWeaponFrame.translationCorrectionGameUnits,
                    dynamicWeaponFrame.rotationCorrectionDegrees);
            }
            if (dynamicWeaponFrame.publishVisualAuthority) {
                const bool visualPublishSucceeded = _twoHandedGrip.applyWeaponCollisionResolvedAuthority(
                    weaponNode,
                    dynamicWeaponFrame.requestedWeaponWorld,
                    dynamicWeaponFrame.resolvedWeaponWorld,
                    currentWeaponGenerationKey);
                const float immediateTranslationError =
                    visualPublishSucceeded && weaponNode ?
                        dynamic_weapon_collision_policy::translationDeltaGameUnits(
                            weaponNode->world,
                            dynamicWeaponFrame.resolvedWeaponWorld) :
                        -1.0f;
                const float immediateRotationError =
                    visualPublishSucceeded && weaponNode ?
                        dynamic_weapon_collision_policy::rotationDeltaDegrees(
                            weaponNode->world,
                            dynamicWeaponFrame.resolvedWeaponWorld) :
                        -1.0f;
                if (g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
                    ROCK_LOG_SAMPLE_INFO(
                        Weapon,
                        500,
                        "DWC visual publication: bodyActive={} publishSucceeded={} immediateError=({:.3f}gu,{:.3f}deg) requestedCorrection=({:.3f}gu,{:.3f}deg)",
                        dynamicWeaponFrame.proxyActive,
                        visualPublishSucceeded,
                        immediateTranslationError,
                        immediateRotationError,
                        dynamicWeaponFrame.translationCorrectionGameUnits,
                        dynamicWeaponFrame.rotationCorrectionDegrees);
                }
            }
            _twoHandedGrip.finishWeaponCollisionPresentationFrame(
                dynamicWeaponFrame.publishVisualAuthority);
            (void)_twoHandedGrip.applyFiringWeaponRecoilPresentation(
                weaponNode,
                currentWeaponGenerationKey);
            if (weaponNode) {
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollisionTransforms);
                _weaponCollision.updateBodiesFromCurrentSourceTransforms(
                    hknp,
                    weaponNode,
                    frame.deltaSeconds,
                    drivenSourceNodes.data(),
                    drivenSourceNodeCount);
            }
            if (f4vr::isNodeVisible(weaponNode)) {
                applyFinalWeaponMuzzleAuthority();
            }
            _twoHandedGrip.finalizeGunstockAlignmentDebugSnapshot(
                weaponNode,
                gunstockProjectileNode,
                currentWeaponGenerationKey);
        }
        refreshGeneratedBodyContactRegistry();
        updateSelection(frame);

        updateGrabInput(frame);
        auto selectedCloseCarTarget = [&](const Hand& hand, const HandFrameInput& handInput) {
            DynamicWorldCarTarget target{};
            if (handInput.disabled || hand.isHolding() || !hand.hasSelection()) {
                return target;
            }
            const auto& selection = hand.getSelection();
            if (selection.isFarSelection || !selection.refr || !fo4vr::isExplodableCar(selection.refr->GetObjectReference())) {
                return target;
            }
            target.ref = selection.refr;
            target.seedBodyId = selection.bodyId.value;
            return target;
        };
        _dynamicWorldCarCollision.update(
            frame.bhkWorld,
            frame.hknpWorld,
            std::array<DynamicWorldCarTarget, 2>{
                selectedCloseCarTarget(_rightHand, frame.right),
                selectedCloseCarTarget(_leftHand, frame.left),
            });
        updateHeldMassMovementSlowdown(hknp, frame.deltaSeconds);
        synchronizeContactEvidenceOwnership(rightHandWeaponAuthorityActive, leftSupportGripActive, rightPartGripActive);

        /*
         * Dynamic hand collision runs after normal grab input so the final
         * grab, pull, support-grip, or weapon owner for this frame can gate its
         * lower-priority visual authority without delaying proxy tracking.
         */
        _dynamicHandCollision.updateFrame(
            frame,
            physicsWritesAllowedForWorld(frame.hknpWorld),
            _rightHand,
            _leftHand,
            _bodyBoneColliders,
            rightHandWeaponAuthorityActive || rightPartGripActive,
            leftSupportGripActive ||
                (_twoHandedGrip.isFiringHandLeft() &&
                    _twoHandedGrip.isFiringGripOccupied()),
            _dynamicWeaponCollision.proxyBodyIdForDebug().value,
            _rightHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(false),
            _leftHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(true));
        logColliderClockTrace(frame, weaponNode);
        const auto dynamicHandHapticEvents = _dynamicHandCollision.consumeHapticEvents();
        for (const auto& pulse : dynamicHandHapticEvents.hands) {
            if (!pulse.fire) {
                continue;
            }
            TouchGrabRuntime::HandReport touchGrabReport{};
            const bool surfaceGrabOwnsFeedback =
                g_rockConfig.rockSurfaceGrabHapticsEnabled &&
                _touchGrabRuntime.getHandReport(
                    pulse.isLeft,
                    touchGrabReport) &&
                touchGrabReport.kind ==
                    provider::RockProviderTouchGrabKindV1::FixedAnchor;
            if (surfaceGrabOwnsFeedback) {
                continue;
            }
            (void)_feedbackHaptics.queue(
                pulse.isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockHandCollisionDynamicHapticDurationSeconds,
                pulse.intensity);
        }
        updateFeedbackHaptics(frame.deltaSeconds);

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
        // Publish callback ownership only after every main-thread collider
        // mutation and target update for this frame has committed.
        _generatedBodyStepDrive.registerForNextStep(bhk, hknp);

        // Defer immutable overlay construction until the outer frame hook has
        // also completed native-animation finalization and every provider
        // animation phase. The context is consumed once in that same hook.
        _pendingDebugOverlayFrame = PendingDebugOverlayFrame{
            .context = frame,
            .valid = true,
        };
    }

    void PhysicsInteraction::publishDebugOverlayAfterFrameCallbacks()
    {
        if (!_pendingDebugOverlayFrame.valid) {
            return;
        }

        const auto frame = _pendingDebugOverlayFrame.context;
        _pendingDebugOverlayFrame = {};

        if (!_initialized.load(std::memory_order_acquire) ||
            !frame.worldReady || !frame.bhkWorld || !frame.hknpWorld ||
            frame.gameFrameIndex != runtime_state::currentFrame().frameIndex ||
            frame.bhkWorld != _cachedBhkWorld ||
            frame.hknpWorld != _cachedHknpWorld) {
            debug::ClearFrame();
            return;
        }

        auto* currentBhk = getPlayerBhkWorld();
        auto* currentHknp = currentBhk ? getHknpWorld(currentBhk) : nullptr;
        if (currentBhk != frame.bhkWorld || currentHknp != frame.hknpWorld) {
            debug::ClearFrame();
            return;
        }

        publishDebugBodyOverlay(frame);
    }

    void PhysicsInteraction::updateAuthoredPrimaryFiringGrip()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        // The authored/native canonical weapon frame is always ROCK's
        // physical-right primary controller, independent of FO4VR settings.
        const bool leftHandHoldingObject = _leftHand.isHolding();
        const bool rightHandHoldingObject = _rightHand.isHolding();
        _twoHandedGrip.setGrabbedObjectHandPoseOwnership(
            leftHandHoldingObject,
            rightHandHoldingObject);
        const auto nativeAuthorityFlags =
            provider::currentNativeAnimationAuthorityFlagsV1();
        auto* equippedWeapon = currentEquippedWeaponForm();
        const std::uint64_t weaponGenerationKey =
            weaponNode ? _weaponCollision.getCurrentWeaponGenerationKey() : 0;
        std::uint64_t weaponOwnershipKey =
            weaponNode ? _weaponCollision.getCurrentEquippedWeaponOwnershipKey() : 0;
        if (weaponNode && weaponOwnershipKey == 0) {
            // Keep authored grip alignment independent of generated weapon
            // collision.
            // The richer stack/instance key wins when available; the equipped
            // form remains a stable freshness boundary when collision is off.
            weaponOwnershipKey = currentEquippedWeaponFormId();
        }

        const bool equippedGenerationMatchesForm =
            weaponNode &&
            equippedWeapon &&
            weaponGenerationKey != 0 &&
            weaponOwnershipKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() == equippedWeapon->formID;

        native_idle_grip_preharvest::observeEquippedWeapon(
            equippedGenerationMatchesForm ? equippedWeapon : nullptr,
            equippedGenerationMatchesForm ? weaponNode : nullptr,
            equippedGenerationMatchesForm ? currentEquippedWeaponInstanceData(equippedWeapon) : nullptr,
            equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0);

        const bool rockFiringHandIsLeft =
            _twoHandedGrip.isFiringHandLeft();
        RE::NiTransform controllerHandWorld{};
        RE::NiNode* const controllerWand = rockFiringHandIsLeft ?
            f4vr::getLeftHandNode() :
            f4vr::getRightHandNode();
        const bool controllerHandWorldValid =
            controllerWand &&
            finiteNiTransform(controllerWand->world) &&
            _handFrameResolver.tryReconstructCalibratedHand(
                rockFiringHandIsLeft,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree(),
                controllerWand->world,
                controllerHandWorld);

        _authoredPrimaryFiringGrip.update(AuthoredPrimaryFiringGripFrameInput{
            .weaponNode = weaponNode,
            .weapon = equippedWeapon,
            .weaponOwnershipKey = weaponOwnershipKey,
            .weaponGenerationKey = weaponGenerationKey,
            .weaponInstanceContentKey = equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0,
            .weaponInstanceContentKnown = equippedGenerationMatchesForm,
            .controllerHandWorld = controllerHandWorld,
            .controllerHandWorldValid = controllerHandWorldValid,
            .runtimeInitialized = _initialized.load(std::memory_order_acquire),
            .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
            .localSkeletonReady = runtime.localSkeletonReady,
            .menuBlocking = runtime.localMenuBlocking,
            .compatibilityBlocking = runtime.compatibilityConfigBlocking,
            .weaponDrawn = runtime.weaponDrawn,
            .weaponVisible = weaponNode && f4vr::isNodeVisible(weaponNode),
            // Arms/hands-only manual cycling must retain ROCK's authored
            // weapon-to-controller alignment. Only a native Weapon transform
            // lease (the full reload path) suspends that owner.
            .nativeReloadAuthorityActive =
                (nativeAuthorityFlags &
                    authored_weapon_grip_capture_policy::kWeapon) != 0,
            .conflictingWeaponTransformAuthorityActive =
                _twoHandedGrip.blocksAuthoredPrimaryGripWeaponAlignment(),
            .weaponVisualReturnActive = _twoHandedGrip.isWeaponVisualReturnActive(),
            .primaryHandHoldingObject = rightHandHoldingObject,
            .rockFiringHandIsLeft = rockFiringHandIsLeft,
            .inPowerArmor = f4vr::isInPowerArmor(),
        }, _twoHandedGrip);

        if (_equippedWeaponTransition.isHandPoseHandoffActive()) {
            const bool handoffHandIsLeft = _equippedWeaponTransition.handPoseHandoffIsLeft();
            if (nativeAuthorityFlags != 0 ||
                runtime.localMenuBlocking ||
                runtime.compatibilityConfigBlocking) {
                _equippedWeaponTransition.completeHandPoseHandoff("authored-pose-unavailable");
            } else if (equippedWeapon && equippedWeapon->formID != _equippedWeaponTransition.bridgeWeaponBaseFormID()) {
                _equippedWeaponTransition.completeHandPoseHandoff("equipped-weapon-changed");
            } else if (_twoHandedGrip.hasPublishedAuthoredPrimaryFiringGripFingerPose(handoffHandIsLeft)) {
                _equippedWeaponTransition.completeHandPoseHandoff("equipped-authored-pose-acquired");
            }
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
        _weaponInteractionAcquisitionStates[0] = {};
    }

    void PhysicsInteraction::clearRightWeaponContact()
    {
        _rightWeaponContactBodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        _rightWeaponContactPartKind.store(static_cast<std::uint32_t>(WeaponPartKind::Other), std::memory_order_release);
        _rightWeaponContactReloadRole.store(static_cast<std::uint32_t>(WeaponReloadRole::None), std::memory_order_release);
        _rightWeaponContactSupportRole.store(static_cast<std::uint32_t>(WeaponSupportGripRole::None), std::memory_order_release);
        _rightWeaponContactSocketRole.store(static_cast<std::uint32_t>(WeaponSocketRole::None), std::memory_order_release);
        _rightWeaponContactActionRole.store(static_cast<std::uint32_t>(WeaponActionRole::None), std::memory_order_release);
        _rightWeaponContactGripPose.store(static_cast<std::uint32_t>(WeaponGripPoseId::None), std::memory_order_release);
        _rightWeaponContactMissedFrames.store(WEAPON_CONTACT_TIMEOUT_FRAMES + 1, std::memory_order_release);
        _weaponInteractionAcquisitionStates[1] = {};
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
               (!isLeft && _rightWeaponSupportCollisionSuppressed.load(std::memory_order_acquire)) ||
               (isLeft && _leftWeaponSupportCollisionSuppressed.load(std::memory_order_acquire));
    }

    void PhysicsInteraction::clearContactEvidenceForHand(bool isLeft)
    {
        if (isLeft) {
            _leftHand.clearSemanticContactEvidence();
        } else {
            _rightHand.clearSemanticContactEvidence();
        }
    }

    void PhysicsInteraction::synchronizeContactEvidenceOwnership(bool rightHandWeaponAuthorityActive, bool leftSupportGripActive, bool rightPartGripActive)
    {
        /*
         * ROCK disables generated hand collision when a grab or two-hand/tool
         * owner has the hand. Clear semantic contact state at the same
         * authority transition so callbacks cannot leave a stale touch owner.
         */
        if (_rightHand.hasContactEvidenceSuppressedAtomic() || rightHandWeaponAuthorityActive || rightPartGripActive ||
            _rightDominantWeaponCollisionSuppressed.load(std::memory_order_acquire) ||
            _rightWeaponSupportCollisionSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(false);
        }

        if (_leftHand.hasContactEvidenceSuppressedAtomic() || leftSupportGripActive ||
            _leftWeaponSupportCollisionSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(true);
        }
    }

    void PhysicsInteraction::suppressRightHandCollisionForDominantWeapon(RE::hknpWorld* world)
    {
        /*
         * The equipped gun already owns the dominant-hand pose and weapon aim.
         * Letting the generated right-hand bodies keep colliding while that
         * authority is active creates a second physical owner: stale hand
         * contacts can push props or feed semantic touch. ROCK treats owned
         * tool states as collision-filter ownership, so it uses the shared
         * suppression lease here instead of a visual-only gate.
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

    void PhysicsInteraction::suppressHandCollisionForWeaponSupport(RE::hknpWorld* world, bool isLeft)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::SupportGripSuppression);

        /*
         * Layer 43 vs 44 is intentionally allowed so a free hand can physically
         * touch the equipped weapon before a part grip starts. Once the part
         * grip owns the transform, that hand body becomes a driver and must
         * stop solving against the weapon package just like held-object hand
         * collision suppression. This applies to the offhand support grip and
         * to the detached firing hand's part grips symmetrically.
         */
        Hand& hand = isLeft ? _leftHand : _rightHand;
        auto& suppressionSet = isLeft ? _leftWeaponSupportCollisionSuppression : _rightWeaponSupportCollisionSuppression;
        auto& suppressedFlag = isLeft ? _leftWeaponSupportCollisionSuppressed : _rightWeaponSupportCollisionSuppressed;
        suppressedFlag.store(true, std::memory_order_release);

        auto bodyAlreadySuppressed = [&](std::uint32_t bodyId) {
            return bodyId == INVALID_CONTACT_BODY_ID ||
                   hand_collision_suppression_math::findSuppressionState(suppressionSet, bodyId) != nullptr;
        };

        auto currentHandBodiesAlreadySuppressed = [&]() {
            if (!hand.hasCollisionBody()) {
                return false;
            }

            bool sawValidBody = false;
            const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
            if (colliderCount > 0) {
                for (std::uint32_t i = 0; i < colliderCount; ++i) {
                    const std::uint32_t bodyId = hand.getHandColliderBodyIdAtomic(i);
                    if (bodyId == INVALID_CONTACT_BODY_ID) {
                        continue;
                    }
                    sawValidBody = true;
                    if (!bodyAlreadySuppressed(bodyId)) {
                        return false;
                    }
                }
                return sawValidBody;
            }

            const std::uint32_t bodyId = hand.getCollisionBodyId().value;
            return bodyId != INVALID_CONTACT_BODY_ID && bodyAlreadySuppressed(bodyId);
        };

        if (currentHandBodiesAlreadySuppressed()) {
            return;
        }

        if (!world || !hand.hasCollisionBody()) {
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

            const auto suppression = hand_collision_suppression_math::beginSuppression(suppressionSet, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: {} hand support suppression set full; bodyId={} left active", isLeft ? "left" : "right", bodyId);
                return;
            }

            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::WeaponSupportHand,
                "weapon-support-hand");

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: {} hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    isLeft ? "left" : "right",
                    bodyId,
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(hand.getHandColliderBodyIdAtomic(i));
            }
        } else {
            suppressBody(hand.getCollisionBodyId().value);
        }
    }

    void PhysicsInteraction::restoreHandCollisionAfterWeaponSupport(RE::hknpWorld* world, bool isLeft)
    {
        auto& suppressionSet = isLeft ? _leftWeaponSupportCollisionSuppression : _rightWeaponSupportCollisionSuppression;
        auto& suppressedFlag = isLeft ? _leftWeaponSupportCollisionSuppressed : _rightWeaponSupportCollisionSuppressed;
        if (!hand_collision_suppression_math::hasActive(suppressionSet)) {
            suppressedFlag.store(false, std::memory_order_release);
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: cannot restore {} hand support collision yet (world=null); preserving suppression leases", isLeft ? "left" : "right");
            return;
        }

        bool restoreDeferred = false;
        for (const auto& entry : suppressionSet.entries) {
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
                "TwoHandedGrip: {} hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                isLeft ? "left" : "right",
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }

        if (restoreDeferred) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: {} hand support collision restore deferred; suppression leases preserved", isLeft ? "left" : "right");
            return;
        }

        hand_collision_suppression_math::clear(suppressionSet);
        suppressedFlag.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::suppressHandCollisionAfterEquippedWeaponDrop(
        RE::hknpWorld* world,
        equipped_weapon_drop_policy::SourceHand sourceHand)
    {
        if (sourceHand != equipped_weapon_drop_policy::SourceHand::Right &&
            sourceHand != equipped_weapon_drop_policy::SourceHand::Left) {
            return;
        }

        const bool isLeft = equipped_weapon_drop_policy::isLeft(sourceHand);
        auto& hand = isLeft ? _leftHand : _rightHand;
        auto& suppressionSet = isLeft ? _leftEquippedWeaponDropCollisionSuppression : _rightEquippedWeaponDropCollisionSuppression;
        auto& suppressed = isLeft ? _leftEquippedWeaponDropCollisionSuppressed : _rightEquippedWeaponDropCollisionSuppressed;
        auto& delayedRestore = isLeft ? _leftEquippedWeaponDropDelayedRestore : _rightEquippedWeaponDropDelayedRestore;
        hand_collision_suppression_math::clear(delayedRestore);

        if (!world || !hand.hasCollisionBody()) {
            return;
        }

        auto suppressBody = [&](std::uint32_t bodyId, const char* context) {
            if (bodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                return;
            }

            const auto suppression = hand_collision_suppression_math::beginSuppression(suppressionSet, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Weapon,
                    "EquippedWeaponDrop: {} hand post-drop suppression set full; bodyId={} context={} left active",
                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                    bodyId,
                    context ? context : "unknown");
                return;
            }

            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::EquippedWeaponDropHand,
                context);

            if (registryResult.valid && (registryResult.firstLeaseForBody || registryResult.filterChanged)) {
                ROCK_LOG_DEBUG(Weapon,
                    "EquippedWeaponDrop: {} hand post-drop collision lease acquired bodyId={} context={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    equipped_weapon_drop_policy::sourceHandName(sourceHand),
                    bodyId,
                    context ? context : "unknown",
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = hand.getHandColliderBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(hand.getHandColliderBodyIdAtomic(i), "equipped-weapon-drop-hand-suite");
            }
        } else {
            suppressBody(hand.getCollisionBodyId().value, "equipped-weapon-drop-hand-anchor");
        }

        std::array<std::uint32_t, kGrabCollisionSuppressionArmBodyCountPerHand> armBodyIds{};
        const auto armBodyCount = _bodyBoneColliders.copyGrabSuppressionArmBodyIdsAtomic(isLeft, armBodyIds.data(), armBodyIds.size());
        for (std::uint32_t i = 0; i < armBodyCount && i < armBodyIds.size(); ++i) {
            suppressBody(armBodyIds[i], "equipped-weapon-drop-arm-chain");
        }

        const bool hasSuppression = hand_collision_suppression_math::hasActive(suppressionSet);
        suppressed.store(hasSuppression, std::memory_order_release);
        if (!hasSuppression) {
            return;
        }

        if (hand_collision_suppression_math::beginDelayedRestore(
                delayedRestore,
                suppressionSet,
                g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds)) {
            ROCK_LOG_DEBUG(Weapon,
                "EquippedWeaponDrop: {} hand post-drop collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                delayedRestore.bodyCount,
                delayedRestore.bodyId,
                delayedRestore.remainingSeconds);
        } else {
            restoreHandCollisionAfterEquippedWeaponDrop(world, isLeft);
        }
    }

    void PhysicsInteraction::restoreHandCollisionAfterEquippedWeaponDrop(RE::hknpWorld* world, bool isLeft)
    {
        auto& suppressionSet = isLeft ? _leftEquippedWeaponDropCollisionSuppression : _rightEquippedWeaponDropCollisionSuppression;
        auto& suppressed = isLeft ? _leftEquippedWeaponDropCollisionSuppressed : _rightEquippedWeaponDropCollisionSuppressed;
        auto& delayedRestore = isLeft ? _leftEquippedWeaponDropDelayedRestore : _rightEquippedWeaponDropDelayedRestore;

        if (!hand_collision_suppression_math::hasActive(suppressionSet)) {
            hand_collision_suppression_math::clear(delayedRestore);
            suppressed.store(false, std::memory_order_release);
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Weapon,
                "EquippedWeaponDrop: cannot restore {} hand post-drop collision yet (world=null); preserving suppression leases",
                isLeft ? "left" : "right");
            return;
        }

        bool restoreDeferred = false;
        for (const auto& entry : suppressionSet.entries) {
            if (!entry.active || entry.bodyId == INVALID_CONTACT_BODY_ID) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,
                entry.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::EquippedWeaponDropHand,
                "equipped-weapon-drop-hand");
            if (releaseResult.readFailed) {
                restoreDeferred = true;
                continue;
            }

            ROCK_LOG_DEBUG(Weapon,
                "EquippedWeaponDrop: {} hand post-drop collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                isLeft ? "left" : "right",
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }

        if (restoreDeferred) {
            ROCK_LOG_WARN(Weapon,
                "EquippedWeaponDrop: {} hand post-drop collision restore deferred; suppression leases preserved",
                isLeft ? "left" : "right");
            return;
        }

        hand_collision_suppression_math::clear(suppressionSet);
        hand_collision_suppression_math::clear(delayedRestore);
        suppressed.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::updateEquippedWeaponPostDropCollisionSuppression(RE::hknpWorld* world, float deltaSeconds)
    {
        auto updateHand = [&](bool isLeft) {
            auto& suppressionSet = isLeft ? _leftEquippedWeaponDropCollisionSuppression : _rightEquippedWeaponDropCollisionSuppression;
            auto& suppressed = isLeft ? _leftEquippedWeaponDropCollisionSuppressed : _rightEquippedWeaponDropCollisionSuppressed;
            auto& delayedRestore = isLeft ? _leftEquippedWeaponDropDelayedRestore : _rightEquippedWeaponDropDelayedRestore;

            if (delayedRestore.pending && !hand_collision_suppression_math::advanceDelayedRestore(delayedRestore, suppressionSet, deltaSeconds)) {
                return;
            }

            if (hand_collision_suppression_math::hasActive(suppressionSet)) {
                restoreHandCollisionAfterEquippedWeaponDrop(world, isLeft);
                return;
            }

            hand_collision_suppression_math::clear(delayedRestore);
            suppressed.store(false, std::memory_order_release);
        };

        updateHand(false);
        updateHand(true);
    }

    void PhysicsInteraction::clearEquippedWeaponPostDropCollisionSuppressionState()
    {
        hand_collision_suppression_math::clear(_rightEquippedWeaponDropCollisionSuppression);
        hand_collision_suppression_math::clear(_leftEquippedWeaponDropCollisionSuppression);
        hand_collision_suppression_math::clear(_rightEquippedWeaponDropDelayedRestore);
        hand_collision_suppression_math::clear(_leftEquippedWeaponDropDelayedRestore);
        _rightEquippedWeaponDropCollisionSuppressed.store(false, std::memory_order_release);
        _leftEquippedWeaponDropCollisionSuppressed.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::refreshEquippedWeaponHandlingSettings()
    {
        ::rock::provider::RockProviderEquippedWeaponHandlingRequestV1 request{};
        const bool externalAuthorityActive =
            ::rock::provider::getEquippedWeaponHandlingAuthorityV1(request);
        const RockEquippedWeaponHandlingBaseline rockBaseline{
            .ambidextrousHandoffEnabled =
                g_rockConfig.rockAmbidextrousFiringGripEnabled,
            .equippedWeaponShoulderStashEnabled =
                g_rockConfig.rockEquippedWeaponShoulderStashEnabled,
            .firingGripProximitySupportRadiusGameUnits =
                g_rockConfig.rockFiringGripProximitySupportRadius,
            .firingGripPromotionRadiusGameUnits =
                g_rockConfig.rockFiringGripPromotionRadius,
            .leftFiringAimYawDegrees =
                g_rockConfig.rockLeftFiringAimYawDegrees,
            .leftFiringAimPitchDegrees =
                g_rockConfig.rockLeftFiringAimPitchDegrees,
            .leftFiringAimOffsetXGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetXGameUnits,
            .leftFiringAimOffsetYGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetYGameUnits,
            .leftFiringAimOffsetZGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetZGameUnits,
        };
        auto settings = makeEquippedWeaponHandlingSettings(
            rockBaseline,
            externalAuthorityActive ? &request : nullptr);

        const bool fixedFiringHandIsLeft = g_rockConfig.rockLeftHandedMode;
        if (fixedFiringHandIsLeft) {
            // The addon does not own ROCK's fixed-hand preference. Persistent
            // left carry needs firing-grip ownership even when an active addon
            // request explicitly disables dynamic ambidextrous handoff.
            settings.firingGripOwnershipEnabled = true;
        }

        if (_equippedWeaponHandlingModeInitialized) {
            if (requiresEquippedWeaponHandlingModeReconcile(
                    _equippedWeaponHandlingSettings,
                    settings,
                    fixedFiringHandIsLeft != _fixedFiringHandIsLeft)) {
                _equippedWeaponHandlingModeReconcilePending = true;
            }
        }

        _equippedWeaponHandlingSettings = settings;
        _fixedFiringHandIsLeft = fixedFiringHandIsLeft;
        _equippedWeaponHandlingModeInitialized = true;
        equipped_weapon_handling_runtime::publish(settings);

        const auto pipboyMode = pipboy_equip_policy::resolveEquipMode(
            settings.externalAuthorityActive &&
                settings.pipboyTriggerHandEquipEnabled,
            fixedFiringHandIsLeft);
        pipboy_equip_runtime::setEquipMode(pipboyMode);
    }

    void PhysicsInteraction::reconcileEquippedWeaponHandlingMode()
    {
        if (!_equippedWeaponHandlingModeReconcilePending) {
            return;
        }

        if (_equippedWeaponHandAssignment.pending ||
            _equippedWeaponHandAssignment.active) {
            clearEquippedWeaponHandAssignment(
                "equipped-weapon-handling-mode-changed",
                true);
        } else {
            pipboy_equip_runtime::AssignmentSnapshot persisted{};
            if (pipboy_equip_runtime::getAssignment(persisted) &&
                persisted.active) {
                pipboy_equip_runtime::clearWeaponAssignment();
            }
            _twoHandedGrip.restoreNativeRightEquippedCarry(
                "equipped-weapon-handling-mode-changed");
        }
        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        _fixedLeftCarry = {};
        _equippedWeaponHandlingModeReconcilePending = false;
    }

    bool PhysicsInteraction::submitEquippedWeaponShoulderSheath(
        const std::uint32_t observedWeaponFormID,
        const std::uintptr_t observedWeaponInstanceData,
        const equipped_weapon_drop_policy::SourceHand sourceHand,
        const shoulder_stash::Decision& stashDecision,
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        const bool stashHandIsLeft =
            equipped_weapon_drop_policy::isLeft(sourceHand);
        const std::size_t stashHandIndex = stashHandIsLeft ? 1u : 0u;
        native_equipped_weapon_draw::Identity sheathIdentity{};
        const bool identityCaptured =
            native_equipped_weapon_draw::captureCurrentIdentity(
                sheathIdentity);
        const bool identityMatchesObserved =
            identityCaptured &&
            sheathIdentity.formID == observedWeaponFormID &&
            sheathIdentity.instanceData == observedWeaponInstanceData;
        RE::NiTransform leftFiringHandWeaponLocal{};
        RE::NiPoint3 leftFiringGripWeaponLocal{};
        const bool hasLeftFiringGripTransfer =
            identityMatchesObserved &&
            _twoHandedGrip.tryCaptureLeftFiringGripTransfer(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                leftFiringHandWeaponLocal,
                leftFiringGripWeaponLocal);
        native_equipped_weapon_draw::Result sheathResult{};
        sheathResult.result = identityCaptured ?
            native_equipped_weapon_draw::SubmitResult::IdentityChanged :
            native_equipped_weapon_draw::SubmitResult::MissingEquippedWeapon;
        if (identityMatchesObserved) {
            sheathResult = native_equipped_weapon_draw::
                submitSheatheExactCurrent(sheathIdentity);
        }
        const bool sheathAccepted =
            (sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::Submitted ||
                sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::
                        AlreadySheathingOrSheathed) &&
            held_weapon_equip_state_policy::
                isShoulderStashedPresentationState(
                    sheathResult.stateAfter);
        if (sheathAccepted) {
            clearEquippedWeaponHandAssignment(
                "shoulder-weapon-sheathed",
                true);
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            _fixedLeftCarry = {};
            _equippedWeaponShoulderSheath =
                EquippedWeaponShoulderSheathState{
                    .active = true,
                    .stashedByLeftHand = stashHandIsLeft,
                    .weaponFormID = sheathIdentity.formID,
                    .weaponInstanceData = sheathIdentity.instanceData,
                    .equipIndex = sheathIdentity.equipIndex,
                    .zone = stashDecision.zone,
                    .hasLeftFiringGripTransfer =
                        hasLeftFiringGripTransfer,
                    .leftFiringHandWeaponLocal =
                        leftFiringHandWeaponLocal,
                    .leftFiringGripWeaponLocal =
                        leftFiringGripWeaponLocal,
                };
            _equippedWeaponSheathRetrievalStates = {};
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon shoulder sheathed formID={:08X} instance={:#x} equipIndex={} sourceHand={} zone={} confidence={:.2f} leftTransfer={} state={}({})->{}({}) result={}",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                sheathIdentity.equipIndex,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                body_zone::bodyZoneName(stashDecision.zone),
                stashDecision.confidence,
                hasLeftFiringGripTransfer ? "captured" : "unavailable",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                (void)_feedbackHaptics.queue(
                    stashHandIsLeft ? feedback_haptics::FeedbackHand::Left :
                                      feedback_haptics::FeedbackHand::Right,
                    g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.rockShoulderStashCommitHapticIntensity);
            }
            if (g_rockConfig.rockShoulderStashShowCollectedNotifications) {
                f4vr::showNotification(
                    shoulder_stash_notification_policy::
                        formatStowedNotification(
                            shoulderStashItemName(
                                currentEquippedWeaponForm()),
                            sheathIdentity.formID));
            }
        } else {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon shoulder sheathe failed formID={:08X} instance={:#x} sourceHand={} identityMatch={} state={}({})->{}({}) result={} -- weapon stays equipped and physical drop is suppressed",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                identityMatchesObserved ? "yes" : "no",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
        }

        shoulder_stash::resetRuntime(
            _equippedWeaponStashStates[stashHandIndex]);
        _equippedWeaponStashCommitLeases[stashHandIndex] = {};
        return sheathAccepted;
    }

    void PhysicsInteraction::clearEquippedWeaponShoulderSheath(
        const char* reason)
    {
        if (_equippedWeaponShoulderSheath.active) {
            ROCK_LOG_INFO(
                Weapon,
                "Equipped weapon shoulder sheath cleared reason={} formID={:08X} instance={:#x} zone={}",
                reason ? reason : "unknown",
                _equippedWeaponShoulderSheath.weaponFormID,
                _equippedWeaponShoulderSheath.weaponInstanceData,
                body_zone::bodyZoneName(
                    _equippedWeaponShoulderSheath.zone));
        }
        _equippedWeaponShoulderSheath = {};
        _equippedWeaponSheathRetrievalStates = {};
    }

    void PhysicsInteraction::serviceEquippedWeaponShoulderSheathRetrieval(
        const PhysicsFrameContext& frame,
        const bool handlingEnabled,
        const bool menuInputActive,
        const std::uint32_t observedWeaponFormID,
        const std::uintptr_t observedWeaponInstanceData)
    {
        if (!_equippedWeaponShoulderSheath.active) {
            _equippedWeaponSheathRetrievalStates = {};
            return;
        }
        if (!handlingEnabled) {
            clearEquippedWeaponShoulderSheath(
                "shoulder-stash-authority-lost");
            return;
        }

        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool capturedCurrentIdentity =
            native_equipped_weapon_draw::captureCurrentIdentity(
                currentIdentity);
        const bool identityMatches = capturedCurrentIdentity &&
            currentIdentity.formID ==
                _equippedWeaponShoulderSheath.weaponFormID &&
            currentIdentity.instanceData ==
                _equippedWeaponShoulderSheath.weaponInstanceData &&
            currentIdentity.equipIndex ==
                _equippedWeaponShoulderSheath.equipIndex &&
            observedWeaponFormID == currentIdentity.formID &&
            observedWeaponInstanceData == currentIdentity.instanceData;
        if (!identityMatches) {
            clearEquippedWeaponShoulderSheath(
                "equipped-weapon-identity-changed");
            return;
        }

        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(player);
        if (!held_weapon_equip_state_policy::isValidNativeWeaponState(
                nativeWeaponState)) {
            clearEquippedWeaponShoulderSheath(
                "invalid-native-weapon-state");
            return;
        }

        const auto commitRetrieval = [this, &currentIdentity](
                                         const equipped_weapon_drop_policy::
                                             SourceHand retrievalHand,
                                         const float confidence,
                                         const native_equipped_weapon_draw::
                                             Result& drawResult) {
            const bool retrieveWithLeftHand =
                equipped_weapon_drop_policy::isLeft(retrievalHand);
            const std::size_t handIndex =
                retrieveWithLeftHand ? 1u : 0u;
            _pendingEquippedWeaponPrimaryOnlyGripStart =
                PendingEquippedWeaponPrimaryOnlyGripStart{
                    .pending = true,
                    .isLeft = retrieveWithLeftHand,
                    .targetWeaponFormID = currentIdentity.formID,
                    .targetWeaponInstanceData = currentIdentity.instanceData,
                    .remainingSeconds = 10.0f,
                    .committedTransfer = true,
                    .hasFiringHandWeaponLocal = retrieveWithLeftHand &&
                        _equippedWeaponShoulderSheath.
                            hasLeftFiringGripTransfer,
                    .firingHandWeaponLocal =
                        _equippedWeaponShoulderSheath.
                            leftFiringHandWeaponLocal,
                    .hasFiringGripWeaponLocal = retrieveWithLeftHand &&
                        _equippedWeaponShoulderSheath.
                            hasLeftFiringGripTransfer,
                    .firingGripWeaponLocal =
                        _equippedWeaponShoulderSheath.
                            leftFiringGripWeaponLocal,
                };
            _equippedWeaponUnsheathCommittedThisFrame[handIndex] = true;
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                (void)_feedbackHaptics.queue(
                    retrieveWithLeftHand ?
                        feedback_haptics::FeedbackHand::Left :
                        feedback_haptics::FeedbackHand::Right,
                    g_rockConfig.
                        rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.
                        rockShoulderStashCommitHapticIntensity);
            }
            ROCK_LOG_INFO(
                Weapon,
                "Equipped weapon shoulder unsheath acknowledged formID={:08X} hand={} zone={} confidence={:.2f} requests={} state={}({})->{}({}) result={}",
                currentIdentity.formID,
                equipped_weapon_drop_policy::sourceHandName(retrievalHand),
                body_zone::bodyZoneName(
                    _equippedWeaponShoulderSheath.zone),
                confidence,
                _equippedWeaponShoulderSheath.drawWait.requestCount,
                drawResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateBefore),
                drawResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    drawResult.result));
            clearEquippedWeaponShoulderSheath(
                "physical-hand-unsheath-acknowledged");
        };

        const auto restartRetrievalGesture = [this, &currentIdentity](
                                                 const char* reason) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon shoulder unsheath unacknowledged formID={:08X} hand={} requests={} elapsed={:.3f}s reason={}; preserving shoulder stash for another gesture",
                currentIdentity.formID,
                equipped_weapon_drop_policy::sourceHandName(
                    _equippedWeaponShoulderSheath.retrievalHand),
                _equippedWeaponShoulderSheath.drawWait.requestCount,
                _equippedWeaponShoulderSheath.drawWait.elapsedSeconds,
                reason ? reason : "unknown");
            _equippedWeaponShoulderSheath.retrievalHand =
                equipped_weapon_drop_policy::SourceHand::None;
            _equippedWeaponShoulderSheath.retrievalConfidence = 0.0f;
            _equippedWeaponShoulderSheath.drawWait = {};
            _equippedWeaponSheathRetrievalStates = {};
        };

        if (menuInputActive) {
            _equippedWeaponSheathRetrievalStates = {};
            return;
        }

        if (_equippedWeaponShoulderSheath.retrievalHand !=
            equipped_weapon_drop_policy::SourceHand::None) {
            const auto action =
                equipped_weapon_drop_policy::advanceShoulderDrawWait(
                    _equippedWeaponShoulderSheath.drawWait,
                    nativeWeaponState,
                    frame.deltaSeconds);
            if (action == equipped_weapon_drop_policy::
                              ShoulderDrawAction::CommitRetrieval) {
                native_equipped_weapon_draw::Result acknowledged{};
                acknowledged.stateBefore = nativeWeaponState;
                acknowledged.stateAfter = nativeWeaponState;
                acknowledged.result = native_equipped_weapon_draw::
                    SubmitResult::AlreadyDrawingOrDrawn;
                commitRetrieval(
                    _equippedWeaponShoulderSheath.retrievalHand,
                    _equippedWeaponShoulderSheath.retrievalConfidence,
                    acknowledged);
                return;
            }
            if (action == equipped_weapon_drop_policy::
                              ShoulderDrawAction::RestartGesture) {
                restartRetrievalGesture("native-draw-timeout");
                return;
            }
            if (action == equipped_weapon_drop_policy::
                              ShoulderDrawAction::SubmitDraw) {
                const auto drawResult =
                    native_equipped_weapon_draw::submitExactCurrent(
                        currentIdentity);
                const bool drawAccepted =
                    drawResult.result == native_equipped_weapon_draw::
                                             SubmitResult::Submitted ||
                    drawResult.result == native_equipped_weapon_draw::
                                             SubmitResult::
                                                 AlreadyDrawingOrDrawn;
                if (!drawAccepted) {
                    restartRetrievalGesture(
                        native_equipped_weapon_draw::submitResultName(
                            drawResult.result));
                    return;
                }
                if (drawResult.result == native_equipped_weapon_draw::
                                             SubmitResult::
                                                 AlreadyDrawingOrDrawn ||
                    equipped_weapon_drop_policy::
                        nativeStateAcknowledgesShoulderDraw(
                            drawResult.stateAfter)) {
                    commitRetrieval(
                        _equippedWeaponShoulderSheath.retrievalHand,
                        _equippedWeaponShoulderSheath.
                            retrievalConfidence,
                        drawResult);
                    return;
                }
                ROCK_LOG_SAMPLE_INFO(
                    Weapon,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "Equipped weapon shoulder unsheath awaiting native acknowledgement formID={:08X} hand={} requests={} elapsed={:.3f}s state={}({})->{}({})",
                    currentIdentity.formID,
                    equipped_weapon_drop_policy::sourceHandName(
                        _equippedWeaponShoulderSheath.retrievalHand),
                    _equippedWeaponShoulderSheath.drawWait.requestCount,
                    _equippedWeaponShoulderSheath.drawWait.elapsedSeconds,
                    drawResult.stateBefore,
                    held_weapon_equip_state_policy::nativeWeaponStateName(
                        drawResult.stateBefore),
                    drawResult.stateAfter,
                    held_weapon_equip_state_policy::nativeWeaponStateName(
                        drawResult.stateAfter));
            }
            return;
        }

        const bool nativePresentationRetrievable =
            held_weapon_equip_state_policy::
                isShoulderStashedPresentationState(nativeWeaponState);
        if (!nativePresentationRetrievable) {
            clearEquippedWeaponShoulderSheath(
                "native-weapon-no-longer-sheathed");
            return;
        }

        std::array<shoulder_stash::Decision, 2> decisions{};
        std::array<equipped_weapon_drop_policy::ShoulderRetrievalCandidate, 2>
            candidates{};
        const auto detectorConfig =
            makeEquippedWeaponStashDetectorConfig(true);
        for (const bool isLeft : { false, true }) {
            const std::size_t handIndex = isLeft ? 1u : 0u;
            Hand& hand = isLeft ? _leftHand : _rightHand;
            const HandFrameInput& handInput = isLeft ? frame.left : frame.right;
            auto& detectorState =
                _equippedWeaponSheathRetrievalStates[handIndex];
            const bool handEmpty = !hand.isHolding() &&
                !_touchGrabRuntime.isHandActive(isLeft) &&
                !_pendingForceGrabCommits[handIndex].active &&
                !hand.hasActivePullCatchIntent() &&
                !hand.hasPendingActorEquipmentDropHandoff() &&
                !_twoHandedGrip.isHandPartGripping(isLeft) &&
                !(_twoHandedGrip.isFiringGripOccupied() &&
                    _twoHandedGrip.isFiringHandLeft() == isLeft);
            const bool handAllowedByHandlingMode =
                _equippedWeaponHandlingSettings.
                    ambidextrousHandoffEnabled ||
                isLeft == _fixedFiringHandIsLeft;
            const bool handCanOwnFiringGrip =
                handAllowedByHandlingMode &&
                TwoHandedGrip::canBeginPrimaryOnlyGripForHand(isLeft);
            if (handInput.disabled || !handEmpty ||
                !handCanOwnFiringGrip) {
                shoulder_stash::resetRuntime(detectorState);
                continue;
            }

            const auto decision = shoulder_stash::evaluate(
                shoulder_stash::DetectorInput{
                    .isLeftHand = isLeft,
                    .probe = shoulder_stash::Probe{
                        .pointGame = handInput.grabAnchorWorld,
                    },
                    .hmdProbe = makeShoulderStashHmdProbe(handInput),
                    .hasHmdProbe = true,
                    .hasHmdFrame = frame.hasHmdFrame,
                    .hmdPositionWorld = frame.hmdPositionWorld,
                    .hmdForwardWorld = frame.hmdForwardWorld,
                    .deltaSeconds = frame.deltaSeconds,
                    .config = detectorConfig,
                },
                detectorState);
            decisions[handIndex] = decision;
            const bool sameShoulderZone =
                decision.zone == _equippedWeaponShoulderSheath.zone;
            const bool gripPhysicallyHeld =
                input_remap_runtime::isRawButtonPhysicallyHeld(
                    isLeft,
                    g_rockConfig.rockGrabButtonID);
            candidates[handIndex] =
                equipped_weapon_drop_policy::ShoulderRetrievalCandidate{
                    .eligible = equipped_weapon_drop_policy::
                        canRetrieveShoulderStashedWeapon(
                            equipped_weapon_drop_policy::
                                ShoulderRetrievalInput{
                                    .stashActive = true,
                                    .handlingEnabled = handlingEnabled,
                                    .identityMatches = identityMatches,
                                    .nativePresentationRetrievable =
                                        nativePresentationRetrievable,
                                    .menuInputActive = menuInputActive,
                                    .handDisabled = handInput.disabled,
                                    .handEmpty = handEmpty,
                                    .handCanOwnFiringGrip =
                                        handCanOwnFiringGrip,
                                    .detectorConfirmed =
                                        decision.confirmedForCommit,
                                    .sameShoulderZone = sameShoulderZone,
                                    .gripPhysicallyHeld =
                                        gripPhysicallyHeld,
                                }),
                    .confidence = decision.confidence,
                };

            if (decision.candidate && sameShoulderZone &&
                g_rockConfig.rockShoulderStashHapticsEnabled) {
                const bool pulseDue = _dynamicPushElapsedSeconds >=
                    detectorState.nextCandidatePulseTimeSeconds;
                if (decision.enteredCandidate ||
                    decision.changedCandidate || pulseDue) {
                    (void)_feedbackHaptics.queue(
                        isLeft ? feedback_haptics::FeedbackHand::Left :
                                 feedback_haptics::FeedbackHand::Right,
                        g_rockConfig.
                            rockShoulderStashCandidateHapticDurationSeconds,
                        shoulder_stash_haptic_policy::
                            computeCandidatePulseIntensity(
                                decision.confidence,
                                shoulder_stash_haptic_policy::
                                    CandidatePulseConfig{
                                        .enabled = true,
                                        .baseIntensity = g_rockConfig.
                                            rockShoulderStashCandidateHapticBaseIntensity,
                                        .maxIntensity = g_rockConfig.
                                            rockShoulderStashCandidateHapticIntensity,
                                    }));
                    detectorState.nextCandidatePulseTimeSeconds =
                        _dynamicPushElapsedSeconds +
                        (std::max)(0.02f,
                            g_rockConfig.
                                rockShoulderStashCandidateHapticIntervalSeconds);
                }
            }
        }

        const auto retrievalHand =
            equipped_weapon_drop_policy::selectShoulderRetrievalHand(
                candidates[0],
                candidates[1],
                _equippedWeaponShoulderSheath.stashedByLeftHand);
        if (retrievalHand ==
            equipped_weapon_drop_policy::SourceHand::None) {
            return;
        }

        const bool retrieveWithLeftHand =
            equipped_weapon_drop_policy::isLeft(retrievalHand);
        const std::size_t handIndex = retrieveWithLeftHand ? 1u : 0u;
        const auto drawResult =
            native_equipped_weapon_draw::submitExactCurrent(
                currentIdentity);
        const bool drawAccepted =
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::Submitted ||
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::
                    AlreadyDrawingOrDrawn;
        if (!drawAccepted) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon shoulder unsheath failed formID={:08X} hand={} zone={} state={}({})->{}({}) result={}",
                currentIdentity.formID,
                equipped_weapon_drop_policy::sourceHandName(retrievalHand),
                body_zone::bodyZoneName(
                    _equippedWeaponShoulderSheath.zone),
                drawResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateBefore),
                drawResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    drawResult.result));
            shoulder_stash::resetRuntime(
                _equippedWeaponSheathRetrievalStates[handIndex]);
            return;
        }

        if (drawResult.result == native_equipped_weapon_draw::SubmitResult::
                                     AlreadyDrawingOrDrawn ||
            equipped_weapon_drop_policy::nativeStateAcknowledgesShoulderDraw(
                drawResult.stateAfter)) {
            commitRetrieval(
                retrievalHand,
                decisions[handIndex].confidence,
                drawResult);
            return;
        }

        _equippedWeaponShoulderSheath.retrievalHand = retrievalHand;
        _equippedWeaponShoulderSheath.retrievalConfidence =
            decisions[handIndex].confidence;
        equipped_weapon_drop_policy::beginShoulderDrawWait(
            _equippedWeaponShoulderSheath.drawWait);
        shoulder_stash::resetRuntime(
            _equippedWeaponSheathRetrievalStates[handIndex]);
        ROCK_LOG_INFO(
            Weapon,
            "Equipped weapon shoulder unsheath submitted formID={:08X} hand={} zone={} confidence={:.2f} state={}({})->{}({}); retaining transaction until native acknowledgement",
            currentIdentity.formID,
            equipped_weapon_drop_policy::sourceHandName(retrievalHand),
            body_zone::bodyZoneName(
                _equippedWeaponShoulderSheath.zone),
            decisions[handIndex].confidence,
            drawResult.stateBefore,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateBefore),
            drawResult.stateAfter,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateAfter));
    }

    void PhysicsInteraction::serviceFixedWeaponHand(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool menuInputActive)
    {
        constexpr std::uint16_t kMaximumResolveFrames = 180;

        if (!_fixedFiringHandIsLeft || !weaponNode ||
            currentWeaponGenerationKey == 0 ||
            currentEquippedWeaponOwnershipKey == 0) {
            _fixedLeftCarry = {};
            return;
        }
        if (_equippedWeaponShoulderSheath.active ||
            _pendingEquippedWeaponPrimaryOnlyGripStart.pending) {
            _fixedLeftCarry = {};
            return;
        }

        // An addon-owned Pip-Boy selection is an explicit dynamic side choice.
        // Likewise, a live manual handoff is preserved regardless of whether
        // its effective ambidextrous policy comes from ROCK or the addon. The
        // fixed hand remains the fallback/default rather than fighting the
        // player's deliberate switch.
        if (_equippedWeaponHandAssignment.pending ||
            _equippedWeaponHandAssignment.active) {
            _fixedLeftCarry = {};
            return;
        }
        if (_equippedWeaponHandlingSettings.ambidextrousHandoffEnabled &&
            _twoHandedGrip.isManualOwnershipActive()) {
            _fixedLeftCarry = {};
            return;
        }

        if (_twoHandedGrip.isManualOwnershipActive()) {
            if (_twoHandedGrip.isFiringHandLeft()) {
                _fixedLeftCarry = {};
                return;
            }
            _twoHandedGrip.restoreNativeRightEquippedCarry(
                "fixed-left-hand-enforcement");
        }

        if (!TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true)) {
            if (!_fixedLeftCarry.infrastructureWarningLogged) {
                _fixedLeftCarry.infrastructureWarningLogged = true;
                ROCK_LOG_WARN(
                    Weapon,
                    "Fixed left weapon hand unavailable because the required hFRIK ownership blockers are missing; retaining physical right-hand carry");
            }
            return;
        }
        if (menuInputActive || !f4vr::isNodeVisible(weaponNode)) {
            return;
        }

        auto& state = _fixedLeftCarry;
        if (state.weaponGenerationKey != currentWeaponGenerationKey ||
            state.weaponOwnershipKey != currentEquippedWeaponOwnershipKey) {
            state = FixedLeftCarryState{
                .weaponGenerationKey = currentWeaponGenerationKey,
                .weaponOwnershipKey = currentEquippedWeaponOwnershipKey,
                .remainingResolveFrames = kMaximumResolveFrames,
            };
        }

        const bool liveOffsetFinite = finiteNiTransform(weaponNode->local);
        bool liveOffsetMatches =
            state.nativeOffsetSampleValid && liveOffsetFinite &&
            approximatelySameWeaponLocalOffset(
                weaponNode->local,
                state.nativeOffsetSample);
        if (liveOffsetFinite && !liveOffsetMatches) {
            state.nativeOffsetSample = weaponNode->local;
            state.nativeOffsetSampleValid = true;
            state.matchingNativeOffsetFrames = 0;
            liveOffsetMatches = true;
        }
        const bool nativeOffsetReady =
            pipboy_equip_policy::advanceNativeOffsetReadiness(
                state.nativeOffsetSampleValid,
                liveOffsetMatches,
                state.matchingNativeOffsetFrames);
        if (nativeOffsetReady &&
            _twoHandedGrip.beginPersistentEquippedCarry(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            ROCK_LOG_INFO(
                Weapon,
                "ROCK fixed left-hand equipped-weapon carry active generation={:016X} ownership={:016X}",
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey);
            state = {};
            return;
        }

        if (state.remainingResolveFrames > 0) {
            --state.remainingResolveFrames;
        }
        if (state.remainingResolveFrames == 0) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Fixed left weapon hand is waiting for a generation-bound native carry calibration generation={:016X} ownership={:016X}",
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey);
            state.remainingResolveFrames = kMaximumResolveFrames;
        }
    }

    ::rock::provider::RockProviderResultV1
    PhysicsInteraction::requestProviderEquippedWeaponHandV1(
        const std::uint64_t ownerToken,
        const ::rock::provider::RockProviderEquippedWeaponHandRequestV1& request)
    {
        using Result = ::rock::provider::RockProviderResultV1;
        using Hand = ::rock::provider::RockProviderHand;

        if (!_initialized) {
            return Result::NotReady;
        }
        if (request.hand != Hand::Right && request.hand != Hand::Left) {
            return Result::HandUnavailable;
        }

        const auto* equippedWeapon = currentEquippedWeaponForm();
        if (!equippedWeapon) {
            return Result::TargetUnavailable;
        }
        if (request.weaponFormId != 0 &&
            request.weaponFormId != equippedWeapon->formID) {
            return Result::TargetUnavailable;
        }

        const auto currentWeaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        if (request.weaponGenerationKey != 0 &&
            request.weaponGenerationKey != currentWeaponGenerationKey) {
            return Result::TargetUnavailable;
        }

        const bool requestedLeft = request.hand == Hand::Left;
        if (requestedLeft &&
            !TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true)) {
            return Result::HandUnavailable;
        }

        auto& existing = _equippedWeaponHandAssignment;
        if (existing.source ==
                EquippedWeaponHandAssignmentSource::Provider &&
            existing.ownerToken == ownerToken &&
            existing.formId == equippedWeapon->formID &&
            existing.assignedLeft == requestedLeft &&
            existing.requestedWeaponGenerationKey ==
                request.weaponGenerationKey) {
            return existing.active &&
                           existing.effectiveLeft == requestedLeft ?
                Result::Ok :
                Result::RequestQueued;
        }

        clearEquippedWeaponHandAssignment(
            "provider-hand-request-replaced",
            true);
        _equippedWeaponHandAssignment =
            EquippedWeaponHandAssignmentState{
                .source =
                    EquippedWeaponHandAssignmentSource::Provider,
                .pending = true,
                .active = false,
                .assignedLeft = requestedLeft,
                .effectiveLeft = false,
                .remainingResolveFrames =
                    kEquippedWeaponHandAssignmentMaximumResolveFrames,
                .formId = equippedWeapon->formID,
                .ownerToken = ownerToken,
                .requestedWeaponGenerationKey =
                    request.weaponGenerationKey,
            };
        ROCK_LOG_INFO(
            Weapon,
            "Provider requested equipped weapon hand={} owner={:016X} form={:08X} generation={:016X}",
            requestedLeft ? "left" : "right",
            ownerToken,
            equippedWeapon->formID,
            request.weaponGenerationKey);
        return Result::RequestQueued;
    }

    void PhysicsInteraction::clearEquippedWeaponHandAssignment(
        const char* reason,
        const bool clearUiAssignment)
    {
        const auto& assignment = _equippedWeaponHandAssignment;
        if (assignment.pending || assignment.active) {
            ROCK_LOG_INFO(
                Weapon,
                "{} weapon hand assignment cleared reason={} handle={} stack={} form={:08X} owner={:016X}",
                assignment.source ==
                        EquippedWeaponHandAssignmentSource::Provider ?
                    "Provider" :
                    "Pip-Boy",
                reason ? reason : "unknown",
                assignment.handleId,
                assignment.stackId,
                assignment.formId,
                assignment.ownerToken);
        }
        _twoHandedGrip.restoreNativeRightEquippedCarry(reason);
        if (clearUiAssignment &&
            assignment.source ==
                EquippedWeaponHandAssignmentSource::Pipboy) {
            pipboy_equip_runtime::clearWeaponAssignment(
                assignment.handleId,
                assignment.stackId);
        }
        _equippedWeaponHandAssignment = {};
    }

    void PhysicsInteraction::serviceEquippedWeaponHandAssignment(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool menuInputActive,
        const EquippedWeaponHandlingSettings& handlingSettings)
    {
        if (_equippedWeaponShoulderSheath.active) {
            return;
        }
        const auto equipMode = pipboy_equip_policy::resolveEquipMode(
            handlingSettings.externalAuthorityActive &&
                handlingSettings.pipboyTriggerHandEquipEnabled,
            _fixedFiringHandIsLeft);
        const bool pipboyAssignmentManaged =
            pipboy_equip_policy::managesHandAssignment(equipMode);
        const bool leftCarryAvailable =
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        pipboy_equip_runtime::setLeftHandEquipAvailable(
            pipboyAssignmentManaged && leftCarryAvailable);

        if (!pipboyAssignmentManaged) {
            pipboy_equip_runtime::SelectionEvent discardedEvent{};
            (void)pipboy_equip_runtime::consumeSelectionEvent(
                _lastPipboyWeaponSelectionSequence,
                discardedEvent);
            if (_equippedWeaponHandAssignment.source ==
                    EquippedWeaponHandAssignmentSource::Pipboy &&
                (_equippedWeaponHandAssignment.pending ||
                    _equippedWeaponHandAssignment.active)) {
                clearEquippedWeaponHandAssignment(
                    "native-right-preference",
                    true);
            } else {
                pipboy_equip_runtime::AssignmentSnapshot persisted{};
                if (pipboy_equip_runtime::getAssignment(persisted) &&
                    persisted.active) {
                    pipboy_equip_runtime::clearWeaponAssignment();
                }
            }
        } else {
            pipboy_equip_runtime::SelectionEvent event{};
            if (pipboy_equip_runtime::consumeSelectionEvent(
                    _lastPipboyWeaponSelectionSequence,
                    event)) {
                const bool matchesCurrent =
                    _equippedWeaponHandAssignment.source ==
                        EquippedWeaponHandAssignmentSource::Pipboy &&
                    (_equippedWeaponHandAssignment.pending ||
                        _equippedWeaponHandAssignment.active) &&
                    _equippedWeaponHandAssignment.handleId ==
                        event.handleId &&
                    _equippedWeaponHandAssignment.stackId ==
                        event.stackId;
                if (!event.equipped) {
                    if (matchesCurrent) {
                        clearEquippedWeaponHandAssignment(
                            "selected-stack-unequipped",
                            false);
                    }
                } else {
                    clearEquippedWeaponHandAssignment(
                        "new-pipboy-selection",
                        false);
                    _equippedWeaponHandAssignment =
                        EquippedWeaponHandAssignmentState{
                            .source =
                                EquippedWeaponHandAssignmentSource::
                                    Pipboy,
                            .pending = true,
                            .active = false,
                            .assignedLeft =
                                event.requestedHand ==
                                pipboy_equip_policy::Hand::Left,
                            .effectiveLeft = false,
                            .remainingResolveFrames =
                                kEquippedWeaponHandAssignmentMaximumResolveFrames,
                            .handleId = event.handleId,
                            .stackId = event.stackId,
                            .formId = event.formId,
                        };
                }
            }
        }

        // PhysicsInteraction may be recreated across an hFRIK/skeleton
        // lifecycle while the inventory assignment remains valid. Rehydrate
        // from the hook-owned value snapshot instead of losing the selected
        // hand or retaining any engine pointer across the lifecycle.
        if (pipboyAssignmentManaged &&
            !_equippedWeaponHandAssignment.pending &&
            !_equippedWeaponHandAssignment.active) {
            pipboy_equip_runtime::AssignmentSnapshot persisted{};
            if (pipboy_equip_runtime::getAssignment(persisted) &&
                persisted.active) {
                const bool left = persisted.hand == pipboy_equip_policy::Hand::Left;
                _equippedWeaponHandAssignment =
                    EquippedWeaponHandAssignmentState{
                        .source =
                            EquippedWeaponHandAssignmentSource::Pipboy,
                        .pending = left,
                        .active = !left,
                        .assignedLeft = left,
                        .effectiveLeft = left,
                        .remainingResolveFrames =
                            kEquippedWeaponHandAssignmentMaximumResolveFrames,
                        .handleId = persisted.handleId,
                        .stackId = persisted.stackId,
                        .formId = persisted.formId,
                    };
            }
        }

        auto& assignment = _equippedWeaponHandAssignment;
        if (!assignment.pending && !assignment.active) {
            return;
        }

        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        if (assignment.source ==
            EquippedWeaponHandAssignmentSource::Pipboy) {
            pipboy_equip_runtime::StackSnapshot stack{};
            if (!pipboy_equip_runtime::inspectStack(
                    assignment.handleId,
                    assignment.stackId,
                    stack) ||
                !stack.resolved || !stack.weapon || !stack.equipped ||
                stack.formId != assignment.formId) {
                clearEquippedWeaponHandAssignment(
                    "selected-stack-no-longer-equipped",
                    true);
                return;
            }
        } else if (assignment.source ==
                   EquippedWeaponHandAssignmentSource::Provider) {
            std::uint32_t requiredFlags =
                static_cast<std::uint32_t>(
                    ::rock::provider::
                        RockProviderEquippedWeaponHandlingFlagV1::
                            FiringGripOwnership);
            if (assignment.assignedLeft) {
                requiredFlags |= static_cast<std::uint32_t>(
                    ::rock::provider::
                        RockProviderEquippedWeaponHandlingFlagV1::
                            AmbidextrousHandoff);
            }
            if (!::rock::provider::
                    ownsEquippedWeaponHandlingAuthorityV1(
                        assignment.ownerToken,
                        requiredFlags)) {
                clearEquippedWeaponHandAssignment(
                    "provider-hand-authority-lost",
                    false);
                return;
            }
            const auto* equippedWeapon = currentEquippedWeaponForm();
            if (!equippedWeapon ||
                equippedWeapon->formID != assignment.formId ||
                (assignment.requestedWeaponGenerationKey != 0 &&
                    currentWeaponGenerationKey != 0 &&
                    currentWeaponGenerationKey !=
                        assignment.requestedWeaponGenerationKey)) {
                clearEquippedWeaponHandAssignment(
                    "provider-hand-target-changed",
                    false);
                return;
            }
        } else {
            clearEquippedWeaponHandAssignment(
                "invalid-hand-assignment-source",
                false);
            return;
        }

        if (pipboy_equip_policy::shouldReacquirePersistentLeftCarry(
                assignment.active,
                assignment.assignedLeft,
                assignment.effectiveLeft,
                _twoHandedGrip.isPersistentEquippedCarryActive(),
                _twoHandedGrip.isManualOwnershipActive())) {
            // Menu/lifecycle gates intentionally reset TwoHandedGrip. Preserve
            // the exact hand assignment and reacquire after native right carry
            // has produced a fresh canonical frame.
            assignment.active = false;
            assignment.pending = true;
            assignment.ownershipKey = 0;
            assignment.remainingResolveFrames =
                kEquippedWeaponHandAssignmentMaximumResolveFrames;
            assignment.nativeOffsetGenerationKey = 0;
            assignment.nativeOffsetSampleValid = false;
            assignment.nativeOffsetReadinessLogged = false;
            assignment.matchingNativeOffsetFrames = 0;
        }

        if (!assignment.pending) {
            if (!assignment.effectiveLeft) {
                _twoHandedGrip.clearPersistentEquippedCarry("right-hand-assignment");
            }
            return;
        }

        const auto commitRight = [&](const char* reason) {
            _twoHandedGrip.restoreNativeRightEquippedCarry(reason);
            assignment.pending = false;
            assignment.active = true;
            assignment.effectiveLeft = false;
            assignment.ownershipKey = currentEquippedWeaponOwnershipKey;
            if (assignment.source ==
                EquippedWeaponHandAssignmentSource::Pipboy) {
                pipboy_equip_runtime::publishWeaponAssignment(
                    assignment.handleId,
                    assignment.stackId,
                    assignment.formId,
                    pipboy_equip_policy::Hand::Right);
            }
            ROCK_LOG_INFO(
                Weapon,
                "{} weapon assigned to native right hand reason={} form={:08X}",
                assignment.source ==
                        EquippedWeaponHandAssignmentSource::Provider ?
                    "Provider" :
                    "Pip-Boy",
                reason ? reason : "unknown",
                assignment.formId);
        };

        if (!assignment.assignedLeft || !leftCarryAvailable) {
            if (assignment.assignedLeft &&
                assignment.source ==
                    EquippedWeaponHandAssignmentSource::Provider) {
                clearEquippedWeaponHandAssignment(
                    "provider-left-carry-unavailable",
                    false);
                return;
            }
            commitRight(
                assignment.assignedLeft ?
                    "left-carry-unavailable" :
                    "right-hand-assignment");
            return;
        }
        if (menuInputActive) {
            return;
        }

        const auto* equippedWeapon = currentEquippedWeaponForm();
        const bool identityReady =
            weaponNode &&
            f4vr::isNodeVisible(weaponNode) &&
            currentWeaponGenerationKey != 0 &&
            currentEquippedWeaponOwnershipKey != 0 &&
            equippedWeapon &&
            equippedWeapon->formID == assignment.formId;

        bool nativeOffsetReady = false;
        if (identityReady) {
            if (assignment.nativeOffsetGenerationKey != currentWeaponGenerationKey) {
                assignment.nativeOffsetGenerationKey = currentWeaponGenerationKey;
                assignment.nativeOffsetSampleValid = false;
                assignment.nativeOffsetReadinessLogged = false;
                assignment.matchingNativeOffsetFrames = 0;
            }

            const bool liveOffsetFinite = finiteNiTransform(weaponNode->local);
            bool liveOffsetMatches =
                assignment.nativeOffsetSampleValid &&
                liveOffsetFinite &&
                approximatelySameWeaponLocalOffset(weaponNode->local, assignment.nativeOffsetSample);
            if (liveOffsetFinite && !liveOffsetMatches) {
                // hFRIK owns the native-right offset, including custom,
                // no-custom, melee, PA, and in-session configuration values.
                // Rebase until that live authority remains stable instead of
                // duplicating or overriding its placement rules in ROCK.
                assignment.nativeOffsetSample = weaponNode->local;
                assignment.nativeOffsetSampleValid = true;
                assignment.matchingNativeOffsetFrames = 0;
                liveOffsetMatches = true;
            }

            const auto previousMatchingFrames = assignment.matchingNativeOffsetFrames;
            nativeOffsetReady = pipboy_equip_policy::advanceNativeOffsetReadiness(
                assignment.nativeOffsetSampleValid,
                liveOffsetMatches,
                assignment.matchingNativeOffsetFrames);
            if (!assignment.nativeOffsetReadinessLogged &&
                previousMatchingFrames == 0 && assignment.matchingNativeOffsetFrames == 1) {
                assignment.nativeOffsetReadinessLogged = true;
                ROCK_LOG_INFO(
                    Weapon,
                    "{} left-hand assignment observed visible native-right offset; reserving canonical refresh form={:08X}",
                    assignment.source ==
                            EquippedWeaponHandAssignmentSource::Provider ?
                        "Provider" :
                        "Pip-Boy",
                    assignment.formId);
            }
        } else {
            assignment.matchingNativeOffsetFrames = 0;
        }

        if (identityReady && nativeOffsetReady &&
            _twoHandedGrip.beginPersistentEquippedCarry(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            assignment.pending = false;
            assignment.active = true;
            assignment.effectiveLeft = true;
            assignment.ownershipKey = currentEquippedWeaponOwnershipKey;
            if (assignment.source ==
                EquippedWeaponHandAssignmentSource::Pipboy) {
                pipboy_equip_runtime::publishWeaponAssignment(
                    assignment.handleId,
                    assignment.stackId,
                    assignment.formId,
                    pipboy_equip_policy::Hand::Left);
            }
            ROCK_LOG_INFO(
                Weapon,
                "{} weapon assigned to left hand form={:08X}",
                assignment.source ==
                        EquippedWeaponHandAssignmentSource::Provider ?
                    "Provider" :
                    "Pip-Boy",
                assignment.formId);
            return;
        }

        if (assignment.remainingResolveFrames > 0) {
            --assignment.remainingResolveFrames;
        }
        if (assignment.remainingResolveFrames == 0) {
            if (assignment.source ==
                EquippedWeaponHandAssignmentSource::Provider) {
                ROCK_LOG_WARN(
                    Weapon,
                    "Provider left-hand assignment timed out waiting for a generation-bound canonical carry form={:08X}",
                    assignment.formId);
                clearEquippedWeaponHandAssignment(
                    "provider-left-carry-resolve-timeout",
                    false);
            } else {
                ROCK_LOG_WARN(
                    Weapon,
                    "Pip-Boy left-hand assignment timed out waiting for a generation-bound canonical carry; falling back to right form={:08X}",
                    assignment.formId);
                commitRight("left-carry-resolve-timeout");
            }
        }
    }

    void PhysicsInteraction::reconcileEquippedWeaponHandAssignmentAfterGrip()
    {
        auto& assignment = _equippedWeaponHandAssignment;
        if (!assignment.active) {
            return;
        }
        const bool currentLeft = _twoHandedGrip.isFiringGripOccupied() && _twoHandedGrip.isFiringHandLeft();
        if (currentLeft == assignment.effectiveLeft) {
            return;
        }
        // A deliberate physical handover becomes the durable assignment.
        // Lifecycle reacquisition must restore the current side, not the side
        // originally requested by an older Pip-Boy transaction.
        assignment.assignedLeft = currentLeft;
        assignment.effectiveLeft = currentLeft;
        if (assignment.source ==
            EquippedWeaponHandAssignmentSource::Pipboy) {
            pipboy_equip_runtime::publishWeaponAssignment(
                assignment.handleId,
                assignment.stackId,
                assignment.formId,
                currentLeft ?
                    pipboy_equip_policy::Hand::Left :
                    pipboy_equip_policy::Hand::Right);
        }
        ROCK_LOG_INFO(
            Weapon,
            "{} weapon hand assignment followed firing-grip handoff hand={} form={:08X}",
            assignment.source ==
                    EquippedWeaponHandAssignmentSource::Provider ?
                "Provider" :
                "Pip-Boy",
            currentLeft ? "left" : "right",
            assignment.formId);
    }

    void PhysicsInteraction::shutdown(::rock::provider::RockProviderLifecycleReason reason)
    {
        _pendingDebugOverlayFrame = {};
        weapon_transition_animation_acceleration::cancel("physics-shutdown");
        debug::ShutdownShapePipeline();
        equipped_weapon_handling_runtime::reset();
        _equippedWeaponHandlingSettings = {};
        _fixedFiringHandIsLeft = false;
        _equippedWeaponHandlingModeInitialized = false;
        _equippedWeaponHandlingModeReconcilePending = false;
        _fixedLeftCarry = {};
        pipboy_equip_runtime::setLeftHandEquipAvailable(false);
        _authoredPrimaryFiringGrip.reset("physics-shutdown", _twoHandedGrip);
        if (!_initialized) {
            _equippedWeaponShoulderSheath = {};
            _equippedWeaponSheathRetrievalStates = {};
            _equippedWeaponSheathCommittedThisFrame = {};
            _equippedWeaponUnsheathCommittedThisFrame = {};
            return;
        }

        // No generated-body owner may be torn down while a native listener is
        // still executing or eligible to enter its ROCK callback.
        _generatedBodyStepDrive.reset();

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsShutdown, false);

        ROCK_LOG_INFO(Init, "Shutting down ROCK physics module...");
        restoreHeldMassMovementSlowdown("shutdown");

        auto* currentBhk = getPlayerBhkWorld();
        auto* currentHknp = currentBhk ? getHknpWorld(currentBhk) : nullptr;
        const bool worldValid =
            _cachedBhkWorld &&
            currentBhk == _cachedBhkWorld &&
            _cachedHknpWorld &&
            currentHknp == _cachedHknpWorld;

        if (worldValid) {
            auto* hknp = getHknpWorld(_cachedBhkWorld);
            _dynamicWorldCarCollision.restoreAll(_cachedBhkWorld, hknp, "shutdown");
            _touchGrabRuntime.releaseAll(
                _cachedBhkWorld,
                hknp,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    GenerationChanged,
                _collisionGenerationAtomic.load(
                    std::memory_order_acquire));
            unsubscribeContactEvents(hknp);
            restoreNativePlayerCollisionSuppression(hknp, "shutdown");
            restoreRightHandCollisionAfterDominantWeapon(hknp);
            restoreHandCollisionAfterWeaponSupport(hknp, true);
            restoreHandCollisionAfterWeaponSupport(hknp, false);
            restoreHandCollisionAfterEquippedWeaponDrop(hknp, false);
            restoreHandCollisionAfterEquippedWeaponDrop(hknp, true);
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
            _dynamicWeaponCollision.retireAll(_cachedBhkWorld);
            _weaponCollision.destroyWeaponBody(hknp);
            destroyBodyBoneCollisions(_cachedBhkWorld);
            destroyHandCollisions(_cachedBhkWorld);
        } else {
            _dynamicWorldCarCollision.abandon();
            _touchGrabRuntime.abandonAll(
                provider::RockProviderTouchGrabReleaseReasonV1::
                    WorldLost);
            unsubscribeContactEvents(nullptr);
            ROCK_LOG_INFO(Init, "World stale or null — skipping Havok body destruction");
            _rightHand.abandonHavokStateAfterWorldLoss();
            _leftHand.abandonHavokStateAfterWorldLoss();
            _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
            _weaponCollision.abandonHavokStateAfterWorldLoss();
            _equippedWeaponTransition.abandonSceneGraph();
            _bodyBoneColliders.reset();
            _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
            _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            _rightWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
            hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
            hand_collision_suppression_math::clear(_rightWeaponSupportCollisionSuppression);
            clearEquippedWeaponPostDropCollisionSuppressionState();
            _nativePlayerCollisionSuppressedBodies = {};
            _nativePlayerCollisionSuppressedBodyCount = 0;
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            _nativePlayerCollisionSuppressionOverflowLogged = false;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        clearEquippedWeaponHandAssignment("physics-shutdown", false);
        clearEquippedWeaponShoulderSheath("physics-shutdown");
        _twoHandedGrip.reset();
        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin::ProviderForceGrabCommand);
        clearLooseGrenadeRuntimeState();
        clearEquippedWeaponFiringGripInputState();
        _bodyContactRuntime.reset();
        _shoulderStashStates = {};
        _mouthConsumeStates = {};
        _feedbackHaptics.reset();
        _equippedWeaponTransition.shutdown();
        _weaponCollision.shutdown();
        _bodyBoneColliders.reset();
        _generatedBodyStepDrive.reset();
        _completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _equippedWeaponDropMomentumHandoffs = {};
        markGeneratedBodiesInvalidated();
        clearNativeMeleePhysicalSwingLeases();
        collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        ::rock::provider::clearExternalBodiesForProviderLoss();
        clearLeftWeaponContact();
        clearRightWeaponContact();
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
        _expectedDynamicHandProxyLayerMask = 0;
        _expectedDynamicLeftHandProxyLayerMask = 0;
        _expectedDynamicWeaponProxyLayerMask = 0;
        _expectedDynamicWorldCarClutterLayerMask = 0;
        _expectedDynamicWorldCarLargeClutterLayerMask = 0;
        _originalNativeCharacterControllerLayerMask = 0;
        _expectedNativeCharacterControllerLayerMask = 0;
        _nativeCharacterControllerLayerPolicyCaptured = false;
        _nativeCharacterControllerLayerPolicyEnabled = false;
        _initialized = false;
        observeLifecycleFrame(nullptr, nullptr, reason);
        _hasPrevPositions = false;
        _heldMassMovementLogCounter = 0;
        _handBoneCache.reset();
        _handFrameResolver.reset();
        _currentPreFrikSchedulerSequence = 0;
        collision_isolated_hand_frame_runtime::reset();
        _persistentFrikHandInputIsolationActive = {};
        _persistentFrikHandInputIsolationSequence = {};
        _handCacheResolveLogCounter = 0;
        _paritySummaryCounter = 0;
        _parityEnabledLogged = false;
        _runtimeScaleLogged = false;
        _rawHandParityStates = {};
        _dynamicPushCooldownUntil.clear();
        _heldImpactHapticCooldownUntil.clear();
        _grabEventFrameCounter = 0;
        _mouthConsumeStates = {};
        _feedbackHaptics.reset();
        _grabInputIntentStates = {};
        _peerHeldJoinRetryStates = {};
        _heldWeaponTriggerEquipIntents = {};
        _forceGrabCommittedThisFrame = {};
        _equippedWeaponSheathCommittedThisFrame = {};
        _equippedWeaponUnsheathCommittedThisFrame = {};
        _bareFistGuardState = {};
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
        _bodyContactRuntime.reset();
        hand_collision_suppression_math::clear(_rightDominantWeaponCollisionSuppression);
        hand_collision_suppression_math::clear(_leftWeaponSupportCollisionSuppression);
        hand_collision_suppression_math::clear(_rightWeaponSupportCollisionSuppression);
        _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
        _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
        _rightWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
        clearEquippedWeaponPostDropCollisionSuppressionState();
        _nativePlayerCollisionSuppressedBodies = {};
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
        auto queueHaptic = [this](bool isLeft, float durationSeconds, float intensity) {
            (void)_feedbackHaptics.queue(
                isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                durationSeconds,
                intensity);
        };

        if ((eventData.flags & ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC) != 0) {
            return;
        }

        switch (eventData.type) {
        case GrabEventType::SelectionLocked:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockSelectionLockHapticIntensity);
            }
            return;
        case GrabEventType::SelectionUnlocked:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(
                    eventData.isLeft, g_rockConfig.rockSelectionLockReleaseHapticDurationSeconds, g_rockConfig.rockSelectionLockReleaseHapticIntensity);
            }
            return;
        case GrabEventType::PullStarted:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockPullStartHapticIntensity);
            }
            return;
        case GrabEventType::PullCatchSucceeded:
            if (g_rockConfig.rockGrabHapticsEnabled) {
                queueHaptic(eventData.isLeft, g_rockConfig.rockGrabHapticDurationSeconds, g_rockConfig.rockPullCatchHapticIntensity);
            }
            return;
        case GrabEventType::StashCandidate:
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                const float confidence =
                    (eventData.flags & ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID) != 0 ? eventData.intensityHint : 1.0f;
                queueHaptic(eventData.isLeft,
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
                queueHaptic(eventData.isLeft,
                    g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.rockShoulderStashCommitHapticIntensity);
            }
            return;
        case GrabEventType::ConsumeCandidate:
            if (g_rockConfig.rockMouthConsumeHapticsEnabled) {
                const float confidence =
                    (eventData.flags & ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID) != 0 ? eventData.intensityHint : 1.0f;
                queueHaptic(eventData.isLeft,
                    g_rockConfig.rockMouthConsumeCandidateHapticDurationSeconds,
                    mouth_consume_haptic_policy::computeCandidatePulseIntensity(confidence,
                        mouth_consume_haptic_policy::CandidatePulseConfig{
                            .enabled = true,
                            .baseIntensity = g_rockConfig.rockMouthConsumeCandidateHapticBaseIntensity,
                            .maxIntensity = g_rockConfig.rockMouthConsumeCandidateHapticIntensity,
                        }));
            }
            return;
        case GrabEventType::Consumed:
            if (g_rockConfig.rockMouthConsumeHapticsEnabled) {
                queueHaptic(eventData.isLeft,
                    g_rockConfig.rockMouthConsumeCommitHapticDurationSeconds,
                    g_rockConfig.rockMouthConsumeCommitHapticIntensity);
            }
            return;
        case GrabEventType::GrabCommitted:
            queueHaptic(eventData.isLeft,
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
            queueHaptic(eventData.isLeft, g_rockConfig.rockHeldImpactHapticDurationSeconds, intensity);
            return;
        }
        default:
            return;
        }
    }

    void PhysicsInteraction::updateFeedbackHaptics(float deltaSeconds)
    {
        std::array<feedback_haptics::HapticOutput, 2> outputs{};
        const auto outputCount = _feedbackHaptics.update(deltaSeconds, outputs.data(), outputs.size());
        for (std::size_t i = 0; i < outputCount; ++i) {
            const auto& output = outputs[i];
            if (!output.active || output.intensity <= 0.0f || output.pulseDurationSeconds <= 0.0f) {
                continue;
            }

            vrcf::VRControllers.triggerHaptic(
                output.hand == feedback_haptics::FeedbackHand::Left ? vrcf::Hand::Left : vrcf::Hand::Right,
                output.pulseDurationSeconds,
                output.intensity);
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

    void PhysicsInteraction::synchronizeDynamicWeaponHandCollisionRoles(
        RE::hknpWorld* world)
    {
        const auto attachedHands =
            _twoHandedGrip.weaponCollisionAttachedHands();
        const bool rightHandInteractionEnabled =
            !attachedHands.right;
        const bool leftHandInteractionEnabled =
            !attachedHands.left;
        if (_dynamicWeaponRightHandInteractionEnabled ==
                rightHandInteractionEnabled &&
            _dynamicWeaponLeftHandInteractionEnabled ==
                leftHandInteractionEnabled) {
            return;
        }

        _dynamicWeaponRightHandInteractionEnabled =
            rightHandInteractionEnabled;
        _dynamicWeaponLeftHandInteractionEnabled =
            leftHandInteractionEnabled;

        ROCK_LOG_DEBUG(Weapon,
            "Dynamic weapon hand collision roles changed: right={} left={}",
            rightHandInteractionEnabled ? "free" : "attached",
            leftHandInteractionEnabled ? "free" : "attached");
        _collisionLayerRegistered = false;
        registerCollisionLayer(world);
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
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER]);
        ROCK_LOG_DEBUG(Config, "Layer {} pre-set mask=0x{:016X}", collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER, matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER]);
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
            g_rockConfig.rockWeaponCollisionBlocksSpells,
            g_rockConfig.rockHandDynamicInteractionsEnabled,
            _dynamicWeaponRightHandInteractionEnabled,
            _dynamicWeaponLeftHandInteractionEnabled);
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
        _expectedDynamicHandProxyLayerMask =
            collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                false,
                g_rockConfig.rockHandDynamicInteractionsEnabled,
                _dynamicWeaponRightHandInteractionEnabled);
        _expectedDynamicLeftHandProxyLayerMask =
            collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                true,
                g_rockConfig.rockHandDynamicInteractionsEnabled,
                _dynamicWeaponLeftHandInteractionEnabled);
        _expectedDynamicWeaponProxyLayerMask =
            collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask(
                g_rockConfig.rockHandDynamicInteractionsEnabled,
                _dynamicWeaponRightHandInteractionEnabled,
                _dynamicWeaponLeftHandInteractionEnabled);
        _expectedDynamicWorldCarClutterLayerMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER];
        _expectedDynamicWorldCarLargeClutterLayerMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER];
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
                (nativeControllerObjectPairsMatch ? "suppressed" : "bad") :
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
        ROCK_LOG_INFO(
            Config,
            "Registered dynamic weapon proxy layer={} mask=0x{:016X} hands(right={},left={})",
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY,
            collision_layer_policy::matrixAddressableMask(
                _expectedDynamicWeaponProxyLayerMask),
            _dynamicWeaponRightHandInteractionEnabled ? "free" : "attached",
            _dynamicWeaponLeftHandInteractionEnabled ? "free" : "attached");
    }

    bool PhysicsInteraction::createHandCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!runtime_state::isLocalSkeletonReady()) {
            ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
            return false;
        }

        const HandFrame rightHandFrame = getInteractionHandFrame(false);
        const HandFrame leftHandFrame = getInteractionHandFrame(true);
        if (!rightHandFrame.valid || !leftHandFrame.valid) {
            ROCK_LOG_WARN(Hand,
                "Cannot create hand collisions: collision-isolated hand frame unavailable right={} left={}",
                rightHandFrame.valid ? "ready" : "missing",
                leftHandFrame.valid ? "ready" : "missing");
            return false;
        }

        const bool rightOk = _rightHand.createCollision(
            world,
            bhkWorld,
            rightHandFrame.transform);

        const bool leftOk = _leftHand.createCollision(
            world,
            bhkWorld,
            leftHandFrame.transform);

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
        auto* typedBhkWorld =
            static_cast<RE::bhkWorld*>(bhkWorld);
        auto* hknpWorld =
            typedBhkWorld ?
            getHknpWorld(typedBhkWorld) :
            nullptr;
        _touchGrabRuntime.releaseAll(
            typedBhkWorld,
            hknpWorld,
            provider::RockProviderTouchGrabReleaseReasonV1::
                GenerationChanged,
            _collisionGenerationAtomic.load(
                std::memory_order_acquire));
        clearGeneratedBodyContactRegistry();
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
            if (frame.reloadBoundaryActive) {
                return;
            }
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
        updateEquippedWeaponPostDropCollisionSuppression(world, frame.deltaSeconds);

        if (!frame.right.disabled) {
            _rightHand.updateCollisionTransform(world, frame.right.rawHandWorld, frame.deltaSeconds);
        }
        if (!frame.left.disabled) {
            _leftHand.updateCollisionTransform(world, frame.left.rawHandWorld, frame.deltaSeconds);
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
            "Body bone collider set created: bodies={} legsAndFeet={}",
            _bodyBoneColliders.getBodyCount(),
            g_rockConfig.rockBodyBoneLegAndFootCollidersEnabled ? "enabled" : "disabled");
        return true;
    }

    void PhysicsInteraction::destroyBodyBoneCollisions(void* bhkWorld)
    {
        clearGeneratedBodyContactRegistry();
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
            if (frame.reloadBoundaryActive) {
                return;
            }
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

        auto cacheMutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        std::array<NativePlayerCollisionSuppressedBody, kNativePlayerCollisionSuppressionBodyCapacity> pending{};
        std::uint32_t pendingCount = 0;

        auto keepPending = [&](const NativePlayerCollisionSuppressedBody& body) {
            if (pendingCount < pending.size()) {
                pending[pendingCount++] = body;
            }
        };

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto& body = _nativePlayerCollisionSuppressedBodies[i];
            if (!contact_pipeline_policy::isValidBodyId(body.bodyId)) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                hknp,
                body.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                reason ? reason : "native-player-body");
            if (releaseResult.readFailed) {
                keepPending(body);
            }
        }

        _nativePlayerCollisionSuppressedBodies = pending;
        _nativePlayerCollisionSuppressedBodyCount = pendingCount;
        _nativePlayerCollisionSuppressionRefreshFrames = pendingCount == 0 ? 0 : 30;
    }

    void PhysicsInteraction::refreshNativePlayerCollisionSuppression(RE::hknpWorld* hknp, const char* context)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled || !hknp || _nativePlayerCollisionSuppressedBodyCount == 0) {
            return;
        }

        std::uint32_t retainedCount = 0;
        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto body = _nativePlayerCollisionSuppressedBodies[i];
            if (!contact_pipeline_policy::isValidBodyId(body.bodyId)) {
                continue;
            }

            const auto refreshResult = collision_suppression_registry::globalCollisionSuppressionRegistry().refresh(
                hknp,
                body.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                context ? context : "native-player-body-refresh");
            if (refreshResult.readFailed || (refreshResult.valid && !refreshResult.staleLeaseDiscarded)) {
                _nativePlayerCollisionSuppressedBodies[retainedCount++] = body;
            }
        }

        for (std::uint32_t i = retainedCount; i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            _nativePlayerCollisionSuppressedBodies[i] = {};
        }
        _nativePlayerCollisionSuppressedBodyCount = retainedCount;
    }

    void PhysicsInteraction::refreshNativePlayerCollisionSuppressionFromPhysicsSubstep(RE::hknpWorld* hknp, const char* context)
    {
        if (!g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled || !hknp || _nativePlayerCollisionSuppressedBodyCount == 0) {
            return;
        }

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto& leasedBody = _nativePlayerCollisionSuppressedBodies[i];
            if (!contact_pipeline_policy::isValidBodyId(leasedBody.bodyId)) {
                continue;
            }

            const auto snapshot = havok_runtime::snapshotBody(hknp, RE::hknpBodyId{ leasedBody.bodyId });
            if (!snapshot.valid) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "Native player collision suppression physics refresh skipped: bodyId={} context={} cannot snapshot body",
                    leasedBody.bodyId,
                    context ? context : "");
                continue;
            }

            if (snapshot.motionIndex != leasedBody.motionIndex ||
                snapshot.collisionObject != leasedBody.collisionObject ||
                snapshot.ownerNode != leasedBody.ownerNode) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "Native player collision suppression physics refresh rejected recycled body: bodyId={} context={} oldMotion={} newMotion={} oldOwner={} newOwner={} oldCollision={} newCollision={}",
                    leasedBody.bodyId,
                    context ? context : "",
                    leasedBody.motionIndex,
                    snapshot.motionIndex,
                    static_cast<const void*>(leasedBody.ownerNode),
                    static_cast<const void*>(snapshot.ownerNode),
                    static_cast<const void*>(leasedBody.collisionObject),
                    static_cast<const void*>(snapshot.collisionObject));
                continue;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ leasedBody.bodyId }, currentFilter)) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "Native player collision suppression physics refresh skipped: bodyId={} context={} cannot read filter",
                    leasedBody.bodyId,
                    context ? context : "");
                continue;
            }

            const std::uint32_t refreshedFilter = currentFilter | collision_suppression_registry::kSuppressionNoCollideBit;
            if (refreshedFilter != currentFilter) {
                body_collision::setFilterInfo(hknp, RE::hknpBodyId{ leasedBody.bodyId }, refreshedFilter);
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

        auto cacheMutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
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
            RE::NiPoint3 playerPositionGameUnits{};
            bool playerPositionValid = false;
            RE::TESObjectREFR* rightHeldRef = nullptr;
            RE::TESObjectREFR* leftHeldRef = nullptr;
            std::array<DynamicWorldCarTarget, DynamicWorldCarCollisionRuntime::kMaxTrackedTargets> nearbyCars{};
            std::uint32_t nearbyCarCount = 0;
            bool nearbyCarOverflow = false;

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
                appendNearbyCar(bodyId);
                if (!self || !self->shouldSuppressNativePlayerCollisionBody(bhk, hknp, bodyId) || contains(bodyId)) {
                    return;
                }
                if (bodyCount >= bodyIds.size()) {
                    overflow = true;
                    return;
                }
                bodyIds[bodyCount++] = bodyId;
            }

            void appendNearbyCar(std::uint32_t bodyId)
            {
                if (!self || !bhk || !hknp || !playerPositionValid ||
                    !contact_pipeline_policy::isValidBodyId(bodyId)) {
                    return;
                }

                std::uint32_t filterInfo = 0;
                if (!body_collision::tryReadFilterInfo(hknp, RE::hknpBodyId{ bodyId }, filterInfo)) {
                    return;
                }
                const auto layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
                if (layer != collision_layer_policy::FO4_LAYER_CLUTTER &&
                    layer != collision_layer_policy::FO4_LAYER_CLUTTER_LARGE &&
                    !collision_layer_policy::isDynamicWorldCarLayer(layer)) {
                    return;
                }

                RE::NiTransform bodyWorld{};
                if (!havok_runtime::tryGetBodyWorldTransform(hknp, RE::hknpBodyId{ bodyId }, bodyWorld)) {
                    return;
                }
                const float dx = bodyWorld.translate.x - playerPositionGameUnits.x;
                const float dy = bodyWorld.translate.y - playerPositionGameUnits.y;
                const float dz = bodyWorld.translate.z - playerPositionGameUnits.z;
                const float distanceSquared = dx * dx + dy * dy + dz * dz;
                constexpr float radiusSquared =
                    kNearbyCarCollisionRadiusGameUnits * kNearbyCarCollisionRadiusGameUnits;
                if (!std::isfinite(distanceSquared) || distanceSquared > radiusSquared) {
                    return;
                }

                auto* ref = resolveBodyToRef(bhk, hknp, RE::hknpBodyId{ bodyId });
                if (!ref || ref == rightHeldRef || ref == leftHeldRef || ref->IsDeleted() || ref->IsDisabled() ||
                    !fo4vr::isExplodableCar(ref->GetObjectReference())) {
                    return;
                }
                for (std::uint32_t index = 0; index < nearbyCarCount && index < nearbyCars.size(); ++index) {
                    if (nearbyCars[index].ref == ref) {
                        return;
                    }
                }
                if (nearbyCarCount >= nearbyCars.size()) {
                    nearbyCarOverflow = true;
                    return;
                }
                nearbyCars[nearbyCarCount++] = DynamicWorldCarTarget{
                    .ref = ref,
                    .seedBodyId = bodyId,
                };
            }
        } scanContext{ this, bhk, hknp };

        scanContext.playerPositionValid =
            character_controller_runtime::tryGetPlayerActorPositionGameUnits(scanContext.playerPositionGameUnits);
        scanContext.rightHeldRef = _rightHand.isHolding() ? _rightHand.getHeldRef() : nullptr;
        scanContext.leftHeldRef = _leftHand.isHolding() ? _leftHand.getHeldRef() : nullptr;

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
        scanNode(scanNode, f4vr::getFirstPersonSkeleton(), 64);
        scanNode(scanNode, f4vr::getWorldRootNode(), 64);

        if (scanContext.playerPositionValid) {
            _dynamicWorldCarCollision.synchronizeNearbyTargets(
                bhk,
                hknp,
                std::span<const DynamicWorldCarTarget>{ scanContext.nearbyCars.data(), scanContext.nearbyCarCount });
        }

        if (scanContext.nearbyCarOverflow) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "Nearby car collision target capacity exceeded; keeping first {} cars within {:.0f} game units",
                scanContext.nearbyCars.size(),
                kNearbyCarCollisionRadiusGameUnits);
        }

        if (scanContext.overflow && !_nativePlayerCollisionSuppressionOverflowLogged) {
            _nativePlayerCollisionSuppressionOverflowLogged = true;
            ROCK_LOG_WARN(Hand,
                "Native player collision suppression body capacity exceeded; keeping first {} bodies",
                kNativePlayerCollisionSuppressionBodyCapacity);
        } else if (!scanContext.overflow) {
            _nativePlayerCollisionSuppressionOverflowLogged = false;
        }

        std::array<NativePlayerCollisionSuppressedBody, kNativePlayerCollisionSuppressionBodyCapacity> next{};
        std::uint32_t nextCount = 0;
        auto nextContains = [&](std::uint32_t bodyId) {
            for (std::uint32_t i = 0; i < nextCount && i < next.size(); ++i) {
                if (next[i].bodyId == bodyId) {
                    return true;
                }
            }
            return false;
        };
        auto appendNext = [&](const NativePlayerCollisionSuppressedBody& body) {
            if (!contact_pipeline_policy::isValidBodyId(body.bodyId) || nextContains(body.bodyId) || nextCount >= next.size()) {
                return;
            }
            next[nextCount++] = body;
        };

        for (std::uint32_t i = 0; i < scanContext.bodyCount && i < scanContext.bodyIds.size(); ++i) {
            const auto bodyId = scanContext.bodyIds[i];
            const auto acquireResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                hknp,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                "native-player-body");
            if (acquireResult.valid && acquireResult.leaseIdentityValid) {
                appendNext({
                    bodyId,
                    acquireResult.leaseMotionIndex,
                    acquireResult.leaseCollisionObject,
                    acquireResult.leaseOwnerNode,
                });
            } else if (acquireResult.valid) {
                collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                    hknp,
                    bodyId,
                    collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                    "native-player-body-missing-identity");
                ROCK_LOG_WARN(Hand,
                    "Native player collision suppression acquisition rejected: bodyId={} lease identity unavailable",
                    bodyId);
            }
        }

        for (std::uint32_t i = 0; i < _nativePlayerCollisionSuppressedBodyCount && i < _nativePlayerCollisionSuppressedBodies.size(); ++i) {
            const auto& body = _nativePlayerCollisionSuppressedBodies[i];
            if (nextContains(body.bodyId)) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                hknp,
                body.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::NativePlayerBody,
                "native-player-body-stale");
            if (releaseResult.readFailed) {
                appendNext(body);
            }
        }

        _nativePlayerCollisionSuppressedBodies = next;
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
         * used for generated body writes. It reasserts bit 14 only while the
         * cached native identity still owns the lease. This callback only reads
         * the cache; game-frame refresh owns stale-lease eviction under the
         * callback quiescence gate.
         */
        self->refreshNativePlayerCollisionSuppressionFromPhysicsSubstep(world, "native-player-body-pre-collide");
        self->driveGeneratedCollidersFromPhysicsSubstep(world, timing);
    }

    void PhysicsInteraction::onCustomGrabAuthorityAfterCharacterMovement(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->driveCustomGrabAuthorityAfterCharacterMovement(world, timing);
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
        _dynamicWeaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicHandCollision.flushPendingPhysicsDrive(world, timing);
        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-collider-drive", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-collider-drive", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
    }

    void PhysicsInteraction::driveCustomGrabAuthorityAfterCharacterMovement(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }
        if (!_rightHand.isHoldingAtomic() && !_leftHand.isHoldingAtomic()) {
            return;
        }

        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-character-move-before-grab-flush", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-character-move-before-grab-flush", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);

        // bhkWorld executes every listener's between-collide-and-solve graph
        // before invoking the +0x30 finish slot that owns this callback. The
        // character-manager movement task has therefore updated the live root,
        // while hknp solve has not started. One read is shared by both hands so
        // two-hand grabs receive an identical consumption-frame measurement.
        const auto controller = character_controller_runtime::samplePlayerCharacterControllerPositionHavok();
        const auto scale = physics_scale::current();
        const grab_authority_source_clock::ControllerRootFrameSample consumptionControllerRoot{
            .positionHavok = controller.positionHavok,
            .controllerIdentity = controller.controllerIdentity,
            .controllerVtable = controller.controllerVtable,
            .physicsScaleRevision = scale.revision,
            .valid = controller.valid,
        };
        _rightHand.flushPendingCustomGrabAuthority(world, timing, consumptionControllerRoot, scale.havokToGame);
        _leftHand.flushPendingCustomGrabAuthority(world, timing, consumptionControllerRoot, scale.havokToGame);
    }

    void PhysicsInteraction::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        const auto completedSolveSequence =
            _completedPhysicsSolveSequence.fetch_add(
                1,
                std::memory_order_release) +
            1;
        _rightHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _leftHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _dynamicWeaponCollision.samplePostSolve(
            world,
            completedSolveSequence,
            timing);
        _dynamicHandCollision.samplePostSolveDeviations(
            world,
            completedSolveSequence,
            timing);
        /*
         * driveToKeyFrame programs velocity for the substep that follows the
         * pre-collide callback; it does not make that callback's body matrix a
         * solved pose. Capture only after the final solve, when every generated
         * body has actually consumed the current game-frame target. Publishing
         * from pre-collide exposed the previous solved transform as if it were
         * current and created an exact one-physics-step presentation delay.
         * Intermediate substeps are intentionally not admitted to Submit.
         */
        if (timing.substepCount > 0 &&
            timing.substepIndex + 1 >= timing.substepCount) {
            debug::CaptureAppliedGeneratedBodyTransformsFromPhysicsStep(world);
        }
        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-solve", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-solve", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        serviceRetiredGrabConstraintPayloads();
        _weaponCollision.serviceRetiredWeaponBodies();
        // Frees hand/body bone-collider and grab-authority-proxy collision objects
        // that were world-removed on the main thread, only after the broadphase has
        // been rebuilt by this step. Runs here so all deferred collider teardown
        // shares the same post-solve grace cadence as weapon bodies and constraints.
        BethesdaPhysicsBody::serviceRetiredDeferredPayloads();
    }

    void PhysicsInteraction::updateSelection(const PhysicsFrameContext& frame)
    {
        if (!runtime_state::isLocalSkeletonReady()) {
            _rightHand.stopSelectionBeam();
            _leftHand.stopSelectionBeam();
            return;
        }

        const auto rightPendingTargetPtr = _pendingForceGrabCommits[0].targetHandle.get();
        const auto leftPendingTargetPtr = _pendingForceGrabCommits[1].targetHandle.get();

        auto selectionContextForOtherHand = [](const Hand& hand, RE::TESObjectREFR* pendingTarget) {
            OtherHandSelectionContext context{};
            if (pendingTarget) {
                context.exclusiveRef = pendingTarget;
                return context;
            }
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

        const auto rightHandContext = selectionContextForOtherHand(
            _rightHand,
            _pendingForceGrabCommits[0].active ? rightPendingTargetPtr.get() : nullptr);
        const auto leftHandContext = selectionContextForOtherHand(
            _leftHand,
            _pendingForceGrabCommits[1].active ? leftPendingTargetPtr.get() : nullptr);
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);

        if (_pendingForceGrabCommits[0].active) {
            if (_rightHand.hasSelection()) {
                _rightHand.clearSelectionState(false);
            }
            _rightHand.stopSelectionBeam();
        } else if (!frame.right.disabled) {
            _rightHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.right.grabAnchorWorld,
                frame.right.closeSelectionDirectionWorld,
                frame.right.farSelectionDirectionWorld,
                frame.right.pinchPocketWorld,
                frame.right.pinchDirectionWorld,
                frame.right.hasPinchPocketWorld,
                farHmdConeGate,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                leftHandContext);
            _rightHand.updateSelectionBeam(frame.hknpWorld, frame.right.grabAnchorWorld);
        } else {
            _rightHand.stopSelectionBeam();
        }

        if (_pendingForceGrabCommits[1].active) {
            if (_leftHand.hasSelection()) {
                _leftHand.clearSelectionState(false);
            }
            _leftHand.stopSelectionBeam();
        } else if (!frame.left.disabled) {
            _leftHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.left.grabAnchorWorld,
                frame.left.closeSelectionDirectionWorld,
                frame.left.farSelectionDirectionWorld,
                frame.left.pinchPocketWorld,
                frame.left.pinchDirectionWorld,
                frame.left.hasPinchPocketWorld,
                farHmdConeGate,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                rightHandContext);
            _leftHand.updateSelectionBeam(frame.hknpWorld, frame.left.grabAnchorWorld);
        } else {
            _leftHand.stopSelectionBeam();
        }
    }

    void PhysicsInteraction::prepareDynamicWorldCarCollisionForGrab(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* ref)
    {
        if (!ref) {
            return;
        }
        auto* baseObject = ref->GetObjectReference();
        const bool targetIsCar = fo4vr::isExplodableCar(baseObject);
        if (!targetIsCar) {
            return;
        }
        const auto decision = car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = targetIsCar,
            .playerInPowerArmor = fo4vr::isInPowerArmor(),
        });
        if (decision.allowed) {
            _dynamicWorldCarCollision.restoreReference(bhkWorld, hknpWorld, ref, "grab-commit");
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

    void PhysicsInteraction::enforceNoBareFistState(bool forceRecheck)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* legacyPlayer = f4vr::getPlayer();
        if (!player || !legacyPlayer) {
            _bareFistGuardState = {};
            return;
        }

        const bool weaponDrawn = f4vr::IsWeaponDrawn();
        const std::uint32_t equippedWeaponFormId = currentEquippedWeaponFormId();
        const bool actorUsingMelee = weaponDrawn && f4vr::CombatUtilities_IsActorUsingMelee(legacyPlayer);
        if (!bare_fist_guard_policy::shouldRefreshWitness(
                _bareFistGuardState,
                forceRecheck,
                weaponDrawn,
                equippedWeaponFormId,
                actorUsingMelee)) {
            return;
        }

        // Inventory stack scanning is transition-only; never run it every
        // frame while a legitimate melee weapon remains drawn.
        const bool realMeleeWeaponEquipped = actorUsingMelee && f4vr::isMeleeWeaponEquipped();

        _bareFistGuardState = bare_fist_guard_policy::RecheckState{
            .initialized = true,
            .weaponDrawn = weaponDrawn,
            .equippedWeaponFormId = equippedWeaponFormId,
            .actorUsingMelee = actorUsingMelee,
            .realMeleeWeaponEquipped = realMeleeWeaponEquipped,
        };
        if (!weaponDrawn) {
            return;
        }

        if (!bare_fist_guard_policy::shouldHolster(bare_fist_guard_policy::Witness{
                .rockEnabled = g_rockConfig.rockEnabled,
                .weaponDrawn = weaponDrawn,
                .actorUsingMelee = actorUsingMelee,
                .realMeleeWeaponEquipped = realMeleeWeaponEquipped,
            })) {
            return;
        }

        /*
         * FO4 represents an unarmed fallback as drawn melee even though no
         * inventory weapon owns the hand. ROCK has no manual unequip action,
         * so close that state centrally without filtering real hand-to-hand
         * weapons such as knuckles or power fists.
         */
        player->DrawWeaponMagicHands(false);
        _bareFistGuardState.weaponDrawn = false;
        ROCK_LOG_SAMPLE_INFO(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "Bare-fist draw state blocked: holstering unarmed fallback (equippedForm={:08X})",
            equippedWeaponFormId);
    }

    void PhysicsInteraction::clearLooseGrenadeImpactWatches()
    {
        for (auto& bodyId : _armedLooseGrenadeImpactBodyIds) {
            bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        }
        _pendingLooseGrenadeImpactPair.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
    }

    void PhysicsInteraction::clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin origin)
    {
        for (auto& commit : _pendingForceGrabCommits) {
            if (commit.active && commit.origin == origin) {
                if (origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand &&
                    provider::isInteractionCommandActiveV1(
                        commit.providerResultTemplate.ownerToken,
                        commit.providerResultTemplate.commandId)) {
                    commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Cancelled;
                    commit.providerResultTemplate.failure = provider::RockProviderInteractionFailureV1::ProviderNotReady;
                    provider::completeInteractionCommandV1(commit.providerResultTemplate);
                } else if (origin == PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw) {
                    const auto targetRefPtr = commit.targetHandle.get();
                    auto* targetRef = targetRefPtr.get();
                    if (targetRef && !loose_grenade_runtime::returnDroppedReferenceToInventory(targetRef)) {
                        ROCK_LOG_WARN(Hand,
                            "Loose grenade shutdown cleanup left the physical drop in world: ref={:08X} request={}",
                            targetRef->GetFormID(),
                            commit.grenadeRequestId);
                    }
                }
                commit = {};
            }
        }
    }

    void PhysicsInteraction::clearLooseGrenadeRuntimeState()
    {
        clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw);
        _armedLooseGrenadeFuses = {};
        clearLooseGrenadeImpactWatches();
    }

    std::uint32_t PhysicsInteraction::forceGrabHandBlockerMask(
        const Hand& hand,
        bool isLeft,
        bool handDisabled,
        bool includePendingCommit) const
    {
        const auto state = hand.getState();
        const bool openInteractionState =
            state == HandState::Idle || state == HandState::SelectedClose || state == HandState::SelectedFar;
        // Equip data remains authoritative while menus temporarily hide or
        // detach the weapon's 3D node.
        const bool equippedWeaponPresent = currentEquippedWeaponFormId() != 0;
        const bool equippedWeaponOccupiesHand = force_grab_policy::equippedWeaponOccupiesHand(
            isLeft,
            equippedWeaponPresent,
            _twoHandedGrip.isPartCarryActive(),
            _twoHandedGrip.isFiringHandLeft(),
            _twoHandedGrip.isHandPartGripping(isLeft));

        return force_grab_policy::blockerMask(force_grab_policy::HandAvailabilityInput{
            .disabled = handDisabled,
            .openInteractionState = openInteractionState,
            .holding = hand.isHolding(),
            .activePullCatch = hand.hasActivePullCatchIntent(),
            .actorEquipmentHandoff = hand.hasPendingActorEquipmentDropHandoff(),
            .pendingForceGrab = includePendingCommit && _pendingForceGrabCommits[isLeft ? 1u : 0u].active,
            .equippedWeaponOccupiesHand = equippedWeaponOccupiesHand,
            .touchGrabActive = _touchGrabRuntime.isHandActive(isLeft),
        });
    }

    bool PhysicsInteraction::canHandAcceptForceGrab(const Hand& hand, bool isLeft, bool handDisabled) const
    {
        return forceGrabHandBlockerMask(hand, isLeft, handDisabled, false) == 0;
    }

    bool PhysicsInteraction::handHoldsLooseGrenade(const Hand& hand) const
    {
        return hand.isHolding() && loose_grenade_runtime::isGrenadeRef(hand.getHeldRef());
    }

    bool PhysicsInteraction::hasActiveLooseGrenadeCommit() const
    {
        for (const auto& commit : _pendingForceGrabCommits) {
            if (commit.active && commit.targetIsLooseGrenade) {
                return true;
            }
        }
        return false;
    }

    bool PhysicsInteraction::isPendingForceGrabTarget(RE::TESObjectREFR* ref) const
    {
        if (!ref) {
            return false;
        }
        for (const auto& commit : _pendingForceGrabCommits) {
            if (!commit.active) {
                continue;
            }
            const auto targetRefPtr = commit.targetHandle.get();
            if (targetRefPtr.get() == ref) {
                return true;
            }
        }
        return false;
    }

    void PhysicsInteraction::pruneInactiveProviderForceGrabCommits()
    {
        for (auto& commit : _pendingForceGrabCommits) {
            if (commit.active &&
                commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand &&
                !provider::isInteractionCommandActiveV1(
                    commit.providerResultTemplate.ownerToken,
                    commit.providerResultTemplate.commandId)) {
                commit = {};
            }
        }
    }

    void PhysicsInteraction::serviceLooseGrenadeQuickDraw(const PhysicsFrameContext& frame)
    {
        constexpr float kLooseGrenadeQuickDrawMaxDistanceGame = 96.0f;

        const auto buttonState = input_remap_runtime::consumeRawButtonState(
            false,
            input_remap_policy::kOpenVrGrenadeQuickDrawButtonId);
        if (!g_rockConfig.rockEnabled || !buttonState.available ||
            !buttonState.pressed ||
            input_remap_runtime::isConfiguratorChordInputReserved(false)) {
            return;
        }

        if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
            ROCK_LOG_WARN(Hand, "Ignored grenade quick draw because the physics world is unavailable");
            return;
        }

        pruneInactiveProviderForceGrabCommits();

        if (handHoldsLooseGrenade(_rightHand) || handHoldsLooseGrenade(_leftHand) ||
            hasActiveLooseGrenadeCommit()) {
            ROCK_LOG_INFO(Hand, "Ignored grenade quick draw because a loose grenade is already held or attaching");
            return;
        }

        loose_grenade_runtime::EquippedGrenadeSelection equippedSelection{};
        const auto equippedStatus =
            loose_grenade_runtime::resolveEquippedGrenadeSelection(equippedSelection);
        if (equippedStatus != loose_grenade_runtime::EquippedGrenadeSelectionStatus::Selected) {
            f4vr::showNotification(
                equippedStatus == loose_grenade_runtime::EquippedGrenadeSelectionStatus::NoneEquipped ?
                    "ROCK: No grenade is selected." :
                    "ROCK: The selected grenade cannot be drawn.");
            ROCK_LOG_WARN(Hand,
                "Grenade quick draw could not resolve one native equipped stack: status={}",
                loose_grenade_runtime::selectionStatusName(equippedStatus));
            return;
        }

        const std::uint32_t rightBlockers = forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true);
        const std::uint32_t leftBlockers = forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true);
        const auto handSelection = force_grab_policy::selectGrenadeHand(
            false,
            rightBlockers == 0,
            leftBlockers == 0);
        if (handSelection.failure == force_grab_policy::GrenadeSelectionFailure::HandsBlocked) {
            f4vr::showNotification("ROCK: Cannot draw grenade - both hands are occupied.");
            ROCK_LOG_WARN(Hand,
                "Blocked grenade quick draw before inventory removal: request={} rightBlockers=0x{:02X} leftBlockers=0x{:02X}",
                equippedSelection.requestId,
                rightBlockers,
                leftBlockers);
            return;
        }

        const bool isLeft = handSelection.hand == force_grab_policy::HandChoice::Left;
        auto& commit = _pendingForceGrabCommits[isLeft ? 1u : 0u];
        const auto& handInput = isLeft ? frame.left : frame.right;

        /*
         * Spawn pose is irrelevant: the force-grab commit snaps the
         * grenade to a canonical attach pose, so the drop only needs a
         * location with enough clearance that the spawned body does not
         * start intersecting the hand collider and get ejected before
         * the grab commits.
         */
        constexpr float kLooseGrenadeSpawnHandClearanceGameUnits = 3.0f;
        RE::NiPoint3 dropLocation = handInput.grabAnchorWorld;
        dropLocation.z -= kLooseGrenadeSpawnHandClearanceGameUnits;

        const auto dropResult = loose_grenade_runtime::dropEquippedGrenadeSelectionToWorld(
            equippedSelection,
            dropLocation);
        if (!dropResult.success) {
            f4vr::showNotification("ROCK: The selected grenade could not be drawn.");
            ROCK_LOG_WARN(Hand,
                "Grenade quick-draw inventory drop failed: weapon={:08X} request={} stack={} reason={}",
                equippedSelection.weapon ? equippedSelection.weapon->GetFormID() : 0,
                equippedSelection.requestId,
                equippedSelection.stackId,
                dropResult.reason ? dropResult.reason : "unknown");
            return;
        }

        commit = PendingForceGrabCommit{
            .active = true,
            .isLeft = isLeft,
            .origin = PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw,
            .phase = PendingForceGrabCommitPhase::WaitingForReference,
            .targetHandle = dropResult.handle,
            .targetIsLooseGrenade = true,
            .preferredBodyId = INVALID_BODY_ID,
            .maxDistanceGame = kLooseGrenadeQuickDrawMaxDistanceGame,
            .grenadeRequestId = equippedSelection.requestId,
            .grenadeRuntime = equippedSelection.runtime,
        };
        ROCK_LOG_INFO(Hand,
            "Grenade quick draw created ref={:08X} weapon={:08X} stack={} request={} hand={}",
            dropResult.droppedRef ? dropResult.droppedRef->GetFormID() : 0,
            equippedSelection.weapon ? equippedSelection.weapon->GetFormID() : 0,
            dropResult.stackId,
            equippedSelection.requestId,
            isLeft ? "left" : "right");
    }

    void PhysicsInteraction::servicePendingForceGrabCommits(const PhysicsFrameContext& frame)
    {
        if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
            return;
        }

        for (auto& commit : _pendingForceGrabCommits) {
            if (!commit.active) {
                continue;
            }

            Hand& hand = commit.isLeft ? _leftHand : _rightHand;
            const auto& handInput = commit.isLeft ? frame.left : frame.right;

            auto abandon = [&](const char* reason,
                               provider::RockProviderInteractionFailureV1 providerFailure,
                               RE::TESObjectREFR* targetRef) {
                if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand) {
                    commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Rejected;
                    commit.providerResultTemplate.failure = providerFailure;
                    provider::completeInteractionCommandV1(commit.providerResultTemplate);
                } else {
                    const bool returnedToInventory = targetRef && loose_grenade_runtime::returnDroppedReferenceToInventory(targetRef);
                    if (targetRef) {
                        f4vr::showNotification(returnedToInventory ?
                                "ROCK: Grenade attach failed; returned to inventory." :
                                "ROCK: Grenade attach failed; it remains at your hand.");
                    }
                    ROCK_LOG_WARN(Hand,
                        "Loose grenade force-grab cleanup: ref={:08X} request={} returnedToInventory={}",
                        targetRef ? targetRef->GetFormID() : 0,
                        commit.grenadeRequestId,
                        returnedToInventory ? "yes" : "no");
                }
                ROCK_LOG_WARN(Hand,
                    "Pending force-grab commit abandoned ({}): hand={} origin={}",
                    reason,
                    commit.isLeft ? "left" : "right",
                    static_cast<int>(commit.origin));
                commit = {};
            };

            if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand &&
                !provider::isInteractionCommandActiveV1(
                    commit.providerResultTemplate.ownerToken,
                    commit.providerResultTemplate.commandId)) {
                ROCK_LOG_INFO(Hand,
                    "Pending provider force-grab commit cancelled because its command is no longer active: command={} hand={}",
                    commit.providerResultTemplate.commandId,
                    commit.isLeft ? "left" : "right");
                commit = {};
                continue;
            }

            commit.elapsedTotalSeconds += (std::max)(0.0f, frame.deltaSeconds);
            const bool timedOut = commit.elapsedTotalSeconds >= commit.maxTotalSeconds;

            auto targetRefPtr = commit.targetHandle.get();
            auto* targetRef = targetRefPtr.get();
            if (!targetRef) {
                if (timedOut) {
                    abandon("target ref did not resolve", provider::RockProviderInteractionFailureV1::TargetUnavailable, nullptr);
                }
                continue;
            }
            if (targetRef->IsDeleted() || targetRef->IsDisabled()) {
                abandon("target ref disappeared", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                continue;
            }

            if (commit.phase == PendingForceGrabCommitPhase::WaitingForReference) {
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
            }
            if (!canHandAcceptForceGrab(hand, commit.isLeft, handInput.disabled)) {
                if (timedOut) {
                    abandon("hand busy", provider::RockProviderInteractionFailureV1::HandBusy, targetRef);
                }
                continue;
            }

            if (commit.phase == PendingForceGrabCommitPhase::WaitingForSettle) {
                commit.elapsedSettleSeconds += (std::max)(0.0f, frame.deltaSeconds);
                if (commit.elapsedSettleSeconds < g_rockConfig.rockForceGrabAttachSettleSeconds) {
                    continue;
                }
            }

            if (hand.hasSelection()) {
                hand.clearSelectionState(false);
            }
            const RE::NiPoint3 sourcePoint = commit.hasSourcePointOverride ? commit.sourcePointOverride : handInput.grabAnchorWorld;
            if (!hand.acquireForceGrabLooseSelection(frame.bhkWorld,
                    frame.hknpWorld,
                    targetRef,
                    sourcePoint,
                    commit.preferredBodyId,
                    commit.maxDistanceGame)) {
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
                if (timedOut) {
                    abandon("failed to resolve physics body", provider::RockProviderInteractionFailureV1::TargetBodyMissing, targetRef);
                }
                continue;
            }

            if (hand.getSelection().refr != targetRef) {
                hand.clearSelectionState(false);
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
                if (timedOut) {
                    abandon("selection did not retain exact target", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                }
                continue;
            }
            if (commit.preferredBodyId != INVALID_BODY_ID && hand.getSelection().bodyId.value != commit.preferredBodyId) {
                const std::uint32_t resolvedBodyId = hand.getSelection().bodyId.value;
                hand.clearSelectionState(false);
                if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand) {
                    commit.providerResultTemplate.targetBodyId = resolvedBodyId;
                }
                abandon("resolved body does not match requested body", provider::RockProviderInteractionFailureV1::TargetBodyMissing, targetRef);
                continue;
            }

            /*
             * Acquire and commit the exact handle target in one update. Never
             * persist a ready phase that organic selection can replace before
             * the retry. Saved/calibrated offsets remain applied by the normal
             * grabSelectedObject commit path through forcedArrival.
             */
            commit.phase = PendingForceGrabCommitPhase::AcquireAndCommitExactTarget;

            const auto sharedContext = makeGrabSharedObjectContext(hand, commit.isLeft);
            prepareDynamicWorldCarCollisionForGrab(frame.bhkWorld, frame.hknpWorld, targetRef);
            const bool grabbed = hand.grabSelectedObject(frame.hknpWorld,
                handInput.rawHandWorld,
                g_rockConfig.rockGrabLinearTau,
                g_rockConfig.rockGrabLinearDamping,
                g_rockConfig.rockGrabConstraintMaxForce,
                g_rockConfig.rockGrabLinearProportionalRecovery,
                g_rockConfig.rockGrabLinearConstantRecovery,
                &_bodyBoneColliders,
                sharedContext);
            if (!grabbed) {
                hand.clearSelectionState(false);
                commit.phase = PendingForceGrabCommitPhase::WaitingForSettle;
                if (timedOut) {
                    abandon("failed to commit grab", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                }
                continue;
            }

            auto* heldRef = hand.getHeldRef();
            const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
            const bool exactBody = commit.preferredBodyId == INVALID_BODY_ID || primaryBodyId == commit.preferredBodyId;
            if (heldRef != targetRef || !exactBody) {
                hand.releaseGrabbedObject(
                    frame.hknpWorld,
                    GrabReleaseCollisionRestoreMode::Immediate,
                    makeGrabReleaseContext(hand, commit.isLeft));
                abandon("grab postcondition did not match exact target", provider::RockProviderInteractionFailureV1::TargetUnavailable, targetRef);
                continue;
            }
            if (commit.origin == PendingForceGrabCommitOrigin::ProviderForceGrabCommand) {
                commit.providerResultTemplate.targetBodyId = primaryBodyId;
                commit.providerResultTemplate.state = provider::RockProviderInteractionCommandStateV1::Succeeded;
                commit.providerResultTemplate.failure = provider::RockProviderInteractionFailureV1::None;
                if (!provider::completeInteractionCommandV1(commit.providerResultTemplate)) {
                    /*
                     * Owner/provider loss can race the final main-thread
                     * commit. Do not publish or retain a grab whose command
                     * reservation was cancelled before the terminal result.
                     */
                    hand.releaseGrabbedObject(
                        frame.hknpWorld,
                        GrabReleaseCollisionRestoreMode::Immediate,
                        makeGrabReleaseContext(hand, commit.isLeft));
                    ROCK_LOG_INFO(Hand,
                        "Provider force-grab rolled back because command ownership ended during commit: command={} hand={}",
                        commit.providerResultTemplate.commandId,
                        commit.isLeft ? "left" : "right");
                    commit = {};
                    continue;
                }
            }

            claimObject(heldRef, claimOwnerForHand(commit.isLeft));
            dispatchPhysicsMessage(kPhysMsg_OnGrab, commit.isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
            dispatchGrabCommittedEvent(commit.isLeft, heldRef, primaryBodyId, frame.hknpWorld);
            input_remap_runtime::setHandHeldWeapon(commit.isLeft, (commit.isLeft ? _leftHand : _rightHand).isHoldingLooseWeapon());

            if (commit.origin == PendingForceGrabCommitOrigin::LooseGrenadeQuickDraw) {
                ROCK_LOG_INFO(Hand,
                    "Grenade quick draw force-grabbed: ref={:08X} body={} request={}",
                    heldRef ? heldRef->GetFormID() : 0,
                    primaryBodyId,
                    commit.grenadeRequestId);
            }
            _forceGrabCommittedThisFrame[commit.isLeft ? 1u : 0u] = true;
            commit = {};
        }
    }

    void PhysicsInteraction::saveGrabOffsetForHand(Hand& hand, bool isLeft, RE::hknpWorld* hknpWorld)
    {
        if (!hknpWorld) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, no hknpWorld this frame ({} hand)", isLeft ? "left" : "right");
            return;
        }
        if (!hand.isHolding()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, {} hand is not holding anything", isLeft ? "left" : "right");
            return;
        }

        auto* heldRef = hand.getHeldRef();
        auto* rootNode = heldRef ? heldRef->Get3D() : nullptr;
        auto* baseForm = heldRef ? heldRef->GetObjectReference() : nullptr;
        if (!rootNode || !baseForm) {
            ROCK_LOG_WARN(Hand,
                "Saved grab offset: aborted, held ref missing 3D root or base form ({} hand, refr={:08X})",
                isLeft ? "left" : "right",
                heldRef ? heldRef->GetFormID() : 0);
            return;
        }

        const auto* weaponForm = baseForm->As<RE::TESObjectWEAP>();
        const bool throwableWeapon = weaponForm &&
            (weaponForm->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                weaponForm->weaponData.type == RE::WEAPON_TYPE::kMine);
        if (!saved_grab_offset::participatesInSavedGrabOffsets(weaponForm != nullptr, throwableWeapon)) {
            ROCK_LOG_INFO(Hand,
                "Saved grab offset: skipped for {} hand, '{}' ({:08X}) is a weapon and weapons seat through FRIK weapon offsets only",
                isLeft ? "left" : "right",
                heldRef->GetDisplayFullName() ? heldRef->GetDisplayFullName() : "",
                baseForm->GetFormID());
            return;
        }

        const auto formRef = saved_grab_offset::formRefFromRuntimeId(baseForm->GetFormID());
        if (formRef.empty()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, could not resolve load-order-independent identity for base form {:08X}",
                baseForm->GetFormID());
            return;
        }

        RE::NiTransform proxyWorld{};
        if (!hand.tryComputeGrabProxyLocalPalmPocketFrameWorld(hknpWorld, proxyWorld)) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: could not resolve live proxy frame for {} hand", isLeft ? "left" : "right");
            return;
        }

        const RE::NiTransform objectProxyLocal = grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorld, rootNode->world);

        saved_grab_offset::SavedGrabOffsetFile file{};
        std::string loadError;
        if (!saved_grab_offset::load(formRef, file, &loadError) && !loadError.empty()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: existing file for {:08X} unreadable ({}), overwriting", baseForm->GetFormID(), loadError);
        }
        file.object = formRef;
        file.objectName = heldRef->GetDisplayFullName() ? heldRef->GetDisplayFullName() : std::string{};
        file.formatVersion = saved_grab_offset::kFormatVersion;

        auto& handOffset = isLeft ? file.left : file.right;
        handOffset.present = true;
        handOffset.translateGame[0] = objectProxyLocal.translate.x;
        handOffset.translateGame[1] = objectProxyLocal.translate.y;
        handOffset.translateGame[2] = objectProxyLocal.translate.z;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                handOffset.rotate[row * 3 + column] = objectProxyLocal.rotate.entry[row][column];
            }
        }

        /*
         * Finger pose is only captured from a live organic mesh-curl grab
         * (see Hand::tryGetLiveGrabFingerPoseSnapshot). If this hold never
         * ran the mesh solve - e.g. re-saving position only while the object
         * is already attached via a previously-saved offset (pull-catch/
         * force-grab) - there is no fresh finger data this frame, so any
         * finger pose already on disk for this hand is left untouched
         * instead of being cleared.
         */
        Hand::GrabFingerPoseSnapshot fingerSnapshot{};
        if (hand.tryGetLiveGrabFingerPoseSnapshot(fingerSnapshot)) {
            handOffset.hasFingerPose = true;
            handOffset.fingerValues[0] = fingerSnapshot.values[0];
            handOffset.fingerValues[1] = fingerSnapshot.values[1];
            handOffset.fingerValues[2] = fingerSnapshot.values[2];
            handOffset.fingerValues[3] = fingerSnapshot.values[3];
            handOffset.fingerValues[4] = fingerSnapshot.values[4];
            handOffset.hasFingerJointValues = fingerSnapshot.hasJointValues;
            if (fingerSnapshot.hasJointValues) {
                for (std::size_t i = 0; i < fingerSnapshot.jointValues.size(); ++i) {
                    handOffset.fingerJointValues[i] = fingerSnapshot.jointValues[i];
                }
            }
        }

        saved_grab_offset::save(file);

        /*
         * Ground-truth capture companion. The offset records WHERE the object
         * ends up; this records what it was posed against - the mesh ROCK
         * scored, the hand colliders it was posed relative to, the physics,
         * the contacts the pose actually makes, and the seat ROCK itself
         * committed before the user corrected it. That is what makes a saved
         * pose replayable and scorable offline instead of only reproducible
         * in-game. Physics and identity are filled here because this scope
         * owns the body-mass reader and the form identity.
         */
        saved_grab_capture::SavedGrabCaptureFile capture{};
        if (hand.tryBuildSavedGrabCapture(hknpWorld, proxyWorld, capture.capture)) {
            capture.object = formRef;
            capture.objectName = file.objectName;
            capture.hand = isLeft ? "left" : "right";
            capture.rockVersion = std::string(Version::NAME);

            const std::time_t capturedAt = std::time(nullptr);
            std::tm capturedUtc{};
            if (gmtime_s(&capturedUtc, &capturedAt) == 0) {
                char timeText[32]{};
                if (std::strftime(timeText, sizeof(timeText), "%Y-%m-%dT%H:%M:%SZ", &capturedUtc) > 0) {
                    capture.capturedUtc = timeText;
                }
            }

            const std::uint32_t heldBodyId = hand.getSavedObjectState().bodyId.value;
            capture.capture.physics.bodyId = heldBodyId;
            const float heldMass = readGrabEventBodyMass(hknpWorld, heldBodyId);
            if (std::isfinite(heldMass) && heldMass > 0.0f) {
                capture.capture.physics.mass = heldMass;
                capture.capture.physics.valid = true;
            }

            saved_grab_offset::saveCapture(capture);
            ROCK_LOG_INFO(Hand,
                "Saved grab capture for {:08X} ({} hand): triangles={} colliders={} mass={:.2f} com={} shape={} seatReasons=[align={} roll={} depth={} backstop={}]",
                baseForm->GetFormID(),
                isLeft ? "left" : "right",
                capture.capture.mesh.triangleCount,
                capture.capture.fingerSegments.size() + (capture.capture.palm.valid ? 1u : 0u),
                capture.capture.physics.mass,
                capture.capture.physics.hasCenterOfMass ? (capture.capture.physics.comTrusted ? "trusted" : "UNTRUSTED") : "none",
                capture.capture.seat.shapeClass,
                capture.capture.seat.alignmentReason,
                capture.capture.seat.rollReason,
                capture.capture.seat.depthReason,
                capture.capture.seat.penetrationBackstopReason);
        } else {
            ROCK_LOG_WARN(Hand,
                "Saved grab offset: ground-truth capture unavailable for {} hand ({:08X}); the offset itself was still saved",
                isLeft ? "left" : "right",
                baseForm->GetFormID());
        }

        ROCK_LOG_INFO(Hand,
            "Saved grab offset for {:08X} ({} hand, finger pose {})",
            baseForm->GetFormID(),
            isLeft ? "left" : "right",
            handOffset.hasFingerPose ? "captured" : "unchanged");

        const char* itemName = heldRef->GetDisplayFullName();
        f4vr::showNotification(std::string("Saved grab offset: ") + (itemName && *itemName ? itemName : "item"));
    }

    void PhysicsInteraction::updateSavedGrabOffsetGesture(const PhysicsFrameContext& frame)
    {
        /*
         * The actual press detection (developer mode, Activate/WandAccept
         * edge, which hand is engaged) lives in InputRemapRuntime, which
         * already tracks per-hand held-object state and native-event
         * dispatch; this just consumes the resulting per-hand request.
         */
        if (input_remap_runtime::consumePendingSavedGrabOffsetRequest(false)) {
            saveGrabOffsetForHand(_rightHand, false, frame.hknpWorld);
        }
        if (input_remap_runtime::consumePendingSavedGrabOffsetRequest(true)) {
            saveGrabOffsetForHand(_leftHand, true, frame.hknpWorld);
        }
    }

    void PhysicsInteraction::updateEquippedWeaponReleaseCapture(const PhysicsFrameContext& frame, RE::NiNode* weaponNode)
    {
        auto& capture = _equippedWeaponReleaseCapture;
        if (!_twoHandedGrip.isManualOwnershipActive()) {
            capture = {};
            return;
        }

        /*
         * Prefer the transform ROCK published this frame (part-carry and
         * two-handed solves own the weapon node); the live node world is the
         * FRIK/game-final pose otherwise (primary-only carry).
         */
        RE::NiTransform solvedWeaponWorld{};
        if (_twoHandedGrip.getSolvedWeaponTransform(solvedWeaponWorld) && finiteNiTransform(solvedWeaponWorld)) {
            capture.weaponWorld = solvedWeaponWorld;
            capture.hasWeaponWorld = true;
        } else if (weaponNode && finiteNiTransform(weaponNode->world)) {
            capture.weaponWorld = weaponNode->world;
            capture.hasWeaponWorld = true;
        }

        const bool usableDeltaTime = std::isfinite(frame.deltaSeconds) && frame.deltaSeconds > 0.000001f;

        for (std::size_t handIndex = 0; handIndex < 2; ++handIndex) {
            const auto& handInput = handIndex == 1 ? frame.left : frame.right;
            auto& history = capture.handHistories[handIndex];
            if (!finiteNiTransform(handInput.rawHandWorld)) {
                continue;
            }
            if (capture.hasPreviousHandWorld[handIndex] && usableDeltaTime) {
                const RE::NiPoint3 deltaGameUnits = handInput.rawHandWorld.translate - capture.previousHandWorld[handIndex].translate;
                const RE::NiPoint3 rawHandVelocityHavok = held_object_physics_math::gameUnitsDeltaToHavokVelocity(
                    deltaGameUnits,
                    frame.deltaSeconds,
                    physics_scale::havokToGame());
                const RE::NiPoint3 angularVelocity = held_object_physics_math::angularVelocityFromRotationDelta<RE::NiMatrix3, RE::NiPoint3>(
                    capture.previousHandWorld[handIndex].rotate,
                    handInput.rawHandWorld.rotate,
                    frame.deltaSeconds);
                history.push(rawHandVelocityHavok, angularVelocity);
            }
            capture.previousHandWorld[handIndex] = handInput.rawHandWorld;
            capture.hasPreviousHandWorld[handIndex] = true;
        }
    }

    bool PhysicsInteraction::hasAvailableEquippedWeaponDropHandoff() const
    {
        return std::any_of(
            _equippedWeaponDropMomentumHandoffs.begin(),
            _equippedWeaponDropMomentumHandoffs.end(),
            [](const EquippedWeaponDropMomentumHandoff& handoff) { return !handoff.active; });
    }

    void PhysicsInteraction::armEquippedWeaponDropMomentumHandoff(
        const RE::ObjectRefHandle& handle,
        std::uint32_t droppedFormId,
        equipped_weapon_drop_policy::SourceHand sourceHand,
        const WeaponCollision::ReleaseGeometrySnapshot& releaseGeometry)
    {
        if (!handle) {
            return;
        }
        if (!releaseGeometry.hasCapturedWeaponWorld || !finiteNiTransform(releaseGeometry.capturedWeaponWorld)) {
            ROCK_LOG_ERROR(Weapon,
                "Equipped weapon drop handoff rejected because the frozen release pose is invalid: dropped={:08X}",
                droppedFormId);
            return;
        }
        EquippedWeaponDropMomentumHandoff* handoff = nullptr;
        for (auto& candidate : _equippedWeaponDropMomentumHandoffs) {
            if (!candidate.active) {
                handoff = &candidate;
                break;
            }
        }
        if (!handoff) {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon drop handoff capacity exhausted after admission: dropped={:08X} capacity={}",
                droppedFormId,
                _equippedWeaponDropMomentumHandoffs.size());
            return;
        }

        // Unknown source (SourceHand::None) falls back to the right hand.
        const auto& history = _equippedWeaponReleaseCapture.handHistories[equipped_weapon_drop_policy::isLeft(sourceHand) ? 1u : 0u];
        // Histories are world-space hand velocities; no player-space addend exists anymore.
        const auto release = equipped_weapon_drop_momentum::composeReleaseVelocity(
            history,
            equipped_weapon_drop_momentum::ReleaseVelocitySettings{
                .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                .throwMultiplier = g_rockConfig.rockThrowVelocityMultiplier,
                .maxLinearVelocityHavok = g_rockConfig.rockGrabThrowMaxVelocityHavok,
                .angularVelocityScale = g_rockConfig.rockGrabThrowAngularVelocityScale,
                .maxAngularVelocityRadiansPerSecond = g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                .longObjectAngularScalingEnabled = g_rockConfig.rockGrabLongObjectAngularScalingEnabled,
                .longObjectLeverGameUnits = releaseGeometry.leverGameUnits,
                .longObjectReferenceLeverGameUnits = g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
                .longObjectMinAngularScale = g_rockConfig.rockGrabLongObjectMinAngularScale,
            });

        *handoff = EquippedWeaponDropMomentumHandoff{
            .active = true,
            .hasReleaseVelocity = release.hasData,
            .handle = handle,
            .droppedFormId = droppedFormId,
            .linearVelocityHavok = release.linearVelocityHavok,
            .angularVelocityRadiansPerSecond = release.angularVelocityRadiansPerSecond,
            .hasReleaseWeaponWorld = releaseGeometry.hasCapturedWeaponWorld,
            .releaseWeaponWorld = releaseGeometry.capturedWeaponWorld,
            .progressSolveSequence = _completedPhysicsSolveSequence.load(std::memory_order_acquire),
        };
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon drop handoff armed: dropped={:08X} sourceHand={} velocity={} lever={:.1f}gu angularScale={:.3f} angularCap={:.3f} "
            "linear=({:.3f},{:.3f},{:.3f}) angular=({:.3f},{:.3f},{:.3f})",
            droppedFormId,
            equipped_weapon_drop_policy::sourceHandName(sourceHand),
            release.hasData ? "captured" : "none",
            releaseGeometry.leverGameUnits,
            release.longObjectAngularScale,
            release.angularVelocityCapRadiansPerSecond,
            release.linearVelocityHavok.x,
            release.linearVelocityHavok.y,
            release.linearVelocityHavok.z,
            release.angularVelocityRadiansPerSecond.x,
            release.angularVelocityRadiansPerSecond.y,
            release.angularVelocityRadiansPerSecond.z);
    }

    void PhysicsInteraction::serviceEquippedWeaponDropMomentumHandoff(const PhysicsFrameContext& frame)
    {
        for (auto& handoff : _equippedWeaponDropMomentumHandoffs) {
            if (handoff.active) {
                serviceEquippedWeaponDropMomentumTransaction(handoff, frame);
            }
        }
    }

    void PhysicsInteraction::serviceEquippedWeaponDropMomentumTransaction(
        EquippedWeaponDropMomentumHandoff& handoff,
        const PhysicsFrameContext& frame)
    {
        // This bound applies only while an asynchronously published drop has
        // made no state progress. Paused physics does not consume the budget.
        constexpr std::uint64_t kPublicationStallSolveSteps = 180;

        if (!handoff.active || !frame.worldReady || !frame.hknpWorld) {
            return;
        }

        handoff.elapsedSeconds += (std::max)(0.0f, frame.deltaSeconds);
        const std::uint64_t completedSolveSequence =
            _completedPhysicsSolveSequence.load(std::memory_order_acquire);
        const auto publicationStalled = [&]() {
            return equipped_weapon_drop_momentum::publicationProgressStalled(
                handoff.progressSolveSequence,
                completedSolveSequence,
                kPublicationStallSolveSteps);
        };
        const auto endHandoff = [&](const char* reason, bool warn) {
            if (warn) {
                ROCK_LOG_WARN(Weapon,
                    "Equipped weapon drop handoff ended: dropped={:08X} reason={} stage={} elapsed={:.3f}s solveProgress={}->{} restarts={}",
                    handoff.droppedFormId,
                    reason ? reason : "unknown",
                    static_cast<std::uint32_t>(handoff.stage),
                    handoff.elapsedSeconds,
                    handoff.progressSolveSequence,
                    completedSolveSequence,
                    handoff.identityRestartCount);
            } else {
                ROCK_LOG_DEBUG(Weapon,
                    "Equipped weapon drop handoff ended: dropped={:08X} reason={} stage={} elapsed={:.3f}s",
                    handoff.droppedFormId,
                    reason ? reason : "unknown",
                    static_cast<std::uint32_t>(handoff.stage),
                    handoff.elapsedSeconds);
            }
            handoff = {};
        };

        const auto droppedRefPtr = handoff.handle.get();
        auto* droppedRef = droppedRefPtr.get();
        if (!droppedRef) {
            if (handoff.referenceResolvedOnce) {
                endHandoff("reference-unloaded", false);
            } else if (!handoff.handle || publicationStalled()) {
                endHandoff("reference-publication-stalled", true);
            }
            return;
        }
        if (!handoff.referenceResolvedOnce) {
            handoff.referenceResolvedOnce = true;
            handoff.progressSolveSequence = completedSolveSequence;
        }
        if (droppedRef->IsDeleted() || droppedRef->IsDisabled()) {
            endHandoff("reference-left-world", false);
            return;
        }
        if (handoff.droppedFormId == 0) {
            handoff.droppedFormId = droppedRef->GetFormID();
        }

        auto* scanWorld = frame.bhkWorld;
        if (!scanWorld) {
            auto* cell = droppedRef->GetParentCell();
            scanWorld = cell ? cell->GetbhkWorld() : nullptr;
        }
        auto* droppedRoot = droppedRef->Get3D();
        if (!scanWorld || !droppedRoot) {
            if (handoff.threeDResolvedOnce) {
                endHandoff("physics-3d-unloaded", false);
            } else if (publicationStalled()) {
                endHandoff("physics-3d-publication-stalled", true);
            }
            return;
        }
        if (!handoff.threeDResolvedOnce) {
            handoff.threeDResolvedOnce = true;
            handoff.progressSolveSequence = completedSolveSequence;
        }
        if (!handoff.hasReleaseWeaponWorld ||
            !finiteNiTransform(handoff.releaseWeaponWorld) ||
            !finiteNiTransform(droppedRoot->world)) {
            endHandoff("invalid-release-or-root-pose", true);
            return;
        }

        if (handoff.stage == EquippedWeaponDropHandoffStage::ResolvingBodies) {
            const bool collisionPrepared =
                physics_recursive_wrappers::enableCollisionRecursive(droppedRoot, true, true, true);
            if (!collisionPrepared) {
                if (publicationStalled()) {
                    endHandoff("collision-enable-stalled", true);
                }
                return;
            }
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.targetKind = grab_target::Kind::LooseObject;
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowUnresolvedRefBodies = true;
        scanOptions.allowWeaponRefExpansion = true;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        // The general exact-reference scanner owns temporary containers. It is
        // deliberately confined to this short publication/one-solve handoff;
        // no scan remains active during normal weapon flight.
        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(
            scanWorld,
            frame.hknpWorld,
            droppedRef,
            scanOptions);
        const bool bodyScanComplete =
            bodySet.diagnostics.scanFailures == 0 &&
            bodySet.diagnostics.invalidPhysicsSystems == 0 &&
            bodySet.diagnostics.depthLimitSkips == 0;
        if (!bodyScanComplete) {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon drop exact-reference scan incomplete; retaining transaction: dropped={:08X} stage={} failures={} invalidSystems={} depthSkips={}",
                handoff.droppedFormId,
                static_cast<std::uint32_t>(handoff.stage),
                bodySet.diagnostics.scanFailures,
                bodySet.diagnostics.invalidPhysicsSystems,
                bodySet.diagnostics.depthLimitSkips);
            if (publicationStalled()) {
                endHandoff("exact-reference-scan-stalled", true);
            }
            return;
        }

        using BodyRecord = object_physics_body_set::ObjectPhysicsBodyRecord;
        std::array<const BodyRecord*, kEquippedWeaponDropBodySnapshotCapacity> acceptedRecords{};
        std::array<const BodyRecord*, kEquippedWeaponDropBodySnapshotCapacity> uniqueMotionRecords{};
        std::size_t acceptedRecordCount = 0;
        std::size_t uniqueMotionRecordCount = 0;
        bool bodyCollectionOverflow = false;
        for (const auto& record : bodySet.records) {
            if (!record.accepted || record.bodyId == object_physics_body_set::INVALID_BODY_ID) {
                continue;
            }
            if (acceptedRecordCount >= acceptedRecords.size()) {
                bodyCollectionOverflow = true;
                break;
            }
            acceptedRecords[acceptedRecordCount++] = &record;

            bool motionSeen = false;
            for (std::size_t i = 0; i < uniqueMotionRecordCount; ++i) {
                if (uniqueMotionRecords[i]->motionId == record.motionId) {
                    motionSeen = true;
                    break;
                }
            }
            if (!motionSeen) {
                if (uniqueMotionRecordCount >= uniqueMotionRecords.size()) {
                    bodyCollectionOverflow = true;
                    break;
                }
                uniqueMotionRecords[uniqueMotionRecordCount++] = &record;
            }
        }
        if (bodyCollectionOverflow) {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon drop native body set exceeds fixed transaction capacity: dropped={:08X} capacity={}",
                handoff.droppedFormId,
                acceptedRecords.size());
            if (publicationStalled()) {
                endHandoff("native-body-capacity-stalled", true);
            }
            return;
        }
        if (acceptedRecordCount == 0 || uniqueMotionRecordCount == 0) {
            if (publicationStalled()) {
                endHandoff("native-body-publication-stalled", true);
            }
            return;
        }

        auto tryReadBodyIdentity = [&](std::uint32_t bodyId,
                                       equipped_weapon_drop_momentum::BodyIdentityKey& outIdentity) {
            const auto live = havok_runtime::snapshotBody(
                frame.hknpWorld,
                RE::hknpBodyId{ bodyId });
            if (!live.valid || !live.body || !live.motion || !live.body->shape ||
                !live.collisionObject || !live.ownerNode ||
                live.body->bodyId.value != bodyId) {
                return false;
            }
            auto* physicsSystem =
                havok_runtime::getPhysicsSystemFromCollisionObject(live.collisionObject);
            auto* physicsSystemInstance =
                havok_runtime::getPhysicsSystemInstance(physicsSystem);
            if (!physicsSystemInstance) {
                return false;
            }
            outIdentity = equipped_weapon_drop_momentum::BodyIdentityKey{
                .bodyId = bodyId,
                .motionId = live.motionIndex,
                .motionFirstBodyId = live.motion->firstBodyId,
                .shapeIdentity = reinterpret_cast<std::uintptr_t>(live.body->shape),
                .owningNodeIdentity = reinterpret_cast<std::uintptr_t>(live.ownerNode),
                .collisionObjectIdentity = reinterpret_cast<std::uintptr_t>(live.collisionObject),
                .physicsSystemInstanceIdentity =
                    reinterpret_cast<std::uintptr_t>(physicsSystemInstance),
            };
            return true;
        };
        auto tryBuildBodyIdentity = [&](const BodyRecord& record,
                                        equipped_weapon_drop_momentum::BodyIdentityKey& outIdentity) {
            if (!tryReadBodyIdentity(record.bodyId, outIdentity)) {
                return false;
            }
            return outIdentity.motionId == record.motionId &&
                   outIdentity.collisionObjectIdentity ==
                       reinterpret_cast<std::uintptr_t>(record.collisionObject) &&
                   outIdentity.owningNodeIdentity ==
                       reinterpret_cast<std::uintptr_t>(record.owningNode);
        };
        auto currentBodySetMatches = [&]() {
            if (handoff.bodySnapshotCount == 0 ||
                handoff.bodySnapshotCount != acceptedRecordCount) {
                return false;
            }
            for (std::size_t recordIndex = 0;
                 recordIndex < acceptedRecordCount;
                 ++recordIndex) {
                equipped_weapon_drop_momentum::BodyIdentityKey currentIdentity{};
                if (!acceptedRecords[recordIndex] ||
                    !tryBuildBodyIdentity(*acceptedRecords[recordIndex], currentIdentity)) {
                    return false;
                }
                bool found = false;
                for (std::size_t snapshotIndex = 0;
                     snapshotIndex < handoff.bodySnapshotCount;
                     ++snapshotIndex) {
                    const auto& expected = handoff.bodySnapshots[snapshotIndex];
                    if (expected.valid &&
                        equipped_weapon_drop_momentum::sameBodyIdentity(
                            expected.identity,
                            currentIdentity)) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    return false;
                }
            }
            return true;
        };

        if (handoff.stage == EquippedWeaponDropHandoffStage::ResolvingBodies) {
            std::array<EquippedWeaponDropBodySnapshot, kEquippedWeaponDropBodySnapshotCapacity>
                capturedIdentities{};
            for (std::size_t i = 0; i < acceptedRecordCount; ++i) {
                equipped_weapon_drop_momentum::BodyIdentityKey identity{};
                if (!acceptedRecords[i] ||
                    !tryBuildBodyIdentity(*acceptedRecords[i], identity)) {
                    if (publicationStalled()) {
                        endHandoff("native-identity-read-stalled", true);
                    }
                    return;
                }
                capturedIdentities[i] = EquippedWeaponDropBodySnapshot{
                    .valid = true,
                    .identity = identity,
                };
            }

            const auto currentRootInverse =
                transform_math::invertTransform(droppedRoot->world);
            if (!finiteNiTransform(currentRootInverse)) {
                if (publicationStalled()) {
                    endHandoff("native-root-inverse-stalled", true);
                }
                return;
            }
            std::array<std::uint32_t, kEquippedWeaponDropBodySnapshotCapacity>
                motionBodyIds{};
            std::array<RE::NiTransform, kEquippedWeaponDropBodySnapshotCapacity>
                originalMotionTransforms{};
            std::array<RE::NiTransform, kEquippedWeaponDropBodySnapshotCapacity>
                releaseMotionTargets{};
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const auto* record = uniqueMotionRecords[motionIndex];
                RE::NiTransform currentBodyWorld{};
                if (!record ||
                    !havok_runtime::tryGetBodyArrayWorldTransform(
                        frame.hknpWorld,
                        RE::hknpBodyId{ record->bodyId },
                        currentBodyWorld) ||
                    !finiteNiTransform(currentBodyWorld)) {
                    if (publicationStalled()) {
                        endHandoff("native-body-transform-read-stalled", true);
                    }
                    return;
                }
                const auto rootToBody = transform_math::composeTransforms(
                    currentRootInverse,
                    currentBodyWorld);
                const auto targetBodyWorld = transform_math::composeTransforms(
                    handoff.releaseWeaponWorld,
                    rootToBody);
                if (!finiteNiTransform(rootToBody) ||
                    !finiteNiTransform(targetBodyWorld)) {
                    if (publicationStalled()) {
                        endHandoff("native-release-target-stalled", true);
                    }
                    return;
                }
                motionBodyIds[motionIndex] = record->bodyId;
                originalMotionTransforms[motionIndex] = currentBodyWorld;
                releaseMotionTargets[motionIndex] = targetBodyWorld;
            }

            const RE::hkVector4f zeroVelocity{};
            std::size_t queuedMotions = 0;
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const bool transformQueued =
                    havok_runtime::setBodyTransformDeferred(
                        frame.hknpWorld,
                        motionBodyIds[motionIndex],
                        releaseMotionTargets[motionIndex],
                        1);
                const bool velocityQueued =
                    transformQueued &&
                    havok_runtime::setBodyVelocityDeferred(
                        frame.hknpWorld,
                        motionBodyIds[motionIndex],
                        zeroVelocity,
                        zeroVelocity);
                const bool activated =
                    velocityQueued &&
                    havok_runtime::activateBody(
                        frame.hknpWorld,
                        motionBodyIds[motionIndex]);
                if (!activated) {
                    const std::size_t rollbackCount =
                        queuedMotions + (transformQueued ? 1u : 0u);
                    for (std::size_t rollbackIndex = 0;
                         rollbackIndex < rollbackCount;
                         ++rollbackIndex) {
                        (void)havok_runtime::setBodyTransformDeferred(
                            frame.hknpWorld,
                            motionBodyIds[rollbackIndex],
                            originalMotionTransforms[rollbackIndex],
                            1);
                        (void)havok_runtime::setBodyVelocityDeferred(
                            frame.hknpWorld,
                            motionBodyIds[rollbackIndex],
                            zeroVelocity,
                            zeroVelocity);
                        (void)havok_runtime::activateBody(
                            frame.hknpWorld,
                            motionBodyIds[rollbackIndex]);
                    }
                    ROCK_LOG_SAMPLE_WARN(Weapon,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Equipped weapon drop release-pose placement write failed and queued writes were rolled back; retaining transaction: dropped={:08X} queued={} expected={}",
                        handoff.droppedFormId,
                        queuedMotions,
                        uniqueMotionRecordCount);
                    if (publicationStalled()) {
                        endHandoff("native-release-placement-write-stalled", true);
                    }
                    return;
                }
                ++queuedMotions;
            }

            handoff.bodySnapshots = capturedIdentities;
            handoff.bodySnapshotCount = acceptedRecordCount;
            handoff.bodyDiscoverySolveSequence = completedSolveSequence;
            handoff.progressSolveSequence = completedSolveSequence;
            handoff.stage = EquippedWeaponDropHandoffStage::WaitingForSettleStep;
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon drop native bodies placed at frozen release pose; waiting one solve: dropped={:08X} scanned={} bodies={} motions={} collisionEnabled=yes solveSeq={} restarts={}",
                handoff.droppedFormId,
                bodySet.records.size(),
                acceptedRecordCount,
                uniqueMotionRecordCount,
                handoff.bodyDiscoverySolveSequence,
                handoff.identityRestartCount);
            return;
        }

        if (handoff.stage != EquippedWeaponDropHandoffStage::WaitingForSettleStep ||
            !equipped_weapon_drop_momentum::completedSettleStep(
                handoff.bodyDiscoverySolveSequence,
                completedSolveSequence)) {
            return;
        }

        if (!currentBodySetMatches()) {
            ++handoff.identityRestartCount;
            handoff.stage = EquippedWeaponDropHandoffStage::ResolvingBodies;
            handoff.bodySnapshots = {};
            handoff.bodySnapshotCount = 0;
            handoff.progressSolveSequence = completedSolveSequence;
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon drop native generation changed before momentum; restarting placement: dropped={:08X} restart={}",
                handoff.droppedFormId,
                handoff.identityRestartCount);
            return;
        }

        const RE::hkVector4f linearVelocity{
            handoff.linearVelocityHavok.x,
            handoff.linearVelocityHavok.y,
            handoff.linearVelocityHavok.z,
            0.0f,
        };
        const RE::hkVector4f angularVelocity{
            handoff.angularVelocityRadiansPerSecond.x,
            handoff.angularVelocityRadiansPerSecond.y,
            handoff.angularVelocityRadiansPerSecond.z,
            0.0f,
        };
        const RE::hkVector4f zeroVelocity{};
        std::size_t velocityWrites = 0;
        if (handoff.hasReleaseVelocity) {
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const auto* record = uniqueMotionRecords[motionIndex];
                if (!record ||
                    !havok_runtime::setBodyVelocityDeferred(
                        frame.hknpWorld,
                        record->bodyId,
                        linearVelocity,
                        angularVelocity)) {
                    break;
                }
                ++velocityWrites;
            }
        } else {
            velocityWrites = uniqueMotionRecordCount;
        }

        std::size_t completedMotions = 0;
        if (velocityWrites == uniqueMotionRecordCount) {
            for (std::size_t motionIndex = 0;
                 motionIndex < uniqueMotionRecordCount;
                 ++motionIndex) {
                const auto* record = uniqueMotionRecords[motionIndex];
                if (!record ||
                    !havok_runtime::activateBody(frame.hknpWorld, record->bodyId)) {
                    break;
                }
                ++completedMotions;
            }
        }
        if (completedMotions != uniqueMotionRecordCount) {
            if (handoff.hasReleaseVelocity) {
                for (std::size_t motionIndex = 0;
                     motionIndex < velocityWrites;
                     ++motionIndex) {
                    const auto* record = uniqueMotionRecords[motionIndex];
                    if (!record) {
                        continue;
                    }
                    (void)havok_runtime::setBodyVelocityDeferred(
                        frame.hknpWorld,
                        record->bodyId,
                        zeroVelocity,
                        zeroVelocity);
                    (void)havok_runtime::activateBody(
                        frame.hknpWorld,
                        record->bodyId);
                }
            }
            ROCK_LOG_SAMPLE_WARN(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon drop final momentum write incomplete and partial writes were zeroed; retaining transaction: dropped={:08X} velocityWrites={} activated={} expected={}",
                handoff.droppedFormId,
                velocityWrites,
                completedMotions,
                uniqueMotionRecordCount);
            if (publicationStalled()) {
                endHandoff("final-momentum-write-stalled", true);
            }
            return;
        }

        ROCK_LOG_INFO(Weapon,
            "Equipped weapon drop handoff complete after native contact solve: dropped={:08X} bodies={} motions={} velocity={} elapsed={:.3f}s linear=({:.3f},{:.3f},{:.3f}) angular=({:.3f},{:.3f},{:.3f})",
            handoff.droppedFormId,
            acceptedRecordCount,
            uniqueMotionRecordCount,
            handoff.hasReleaseVelocity ? "applied" : "none",
            handoff.elapsedSeconds,
            handoff.linearVelocityHavok.x,
            handoff.linearVelocityHavok.y,
            handoff.linearVelocityHavok.z,
            handoff.angularVelocityRadiansPerSecond.x,
            handoff.angularVelocityRadiansPerSecond.y,
            handoff.angularVelocityRadiansPerSecond.z);
        handoff = {};
    }

    bool PhysicsInteraction::armHeldLooseGrenade(Hand& hand, const PhysicsFrameContext& frame)
    {
        auto* heldRef = hand.getHeldRef();
        if (!heldRef || !loose_grenade_runtime::isGrenadeRef(heldRef)) {
            return false;
        }

        for (const auto& fuse : _armedLooseGrenadeFuses) {
            const auto fuseRefPtr = fuse.handle.get();
            if (fuse.active && fuseRefPtr.get() == heldRef) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} hand loose grenade trigger ignored because fuse is already active: ref={:08X} remaining={:.3f}s",
                    hand.handName(),
                    heldRef->GetFormID(),
                    fuse.remainingSeconds);
                return true;
            }
        }

        loose_grenade_runtime::GrenadeRuntimeData runtime{};
        if (!loose_grenade_runtime::resolveGrenadeRuntimeDataForReference(heldRef, runtime)) {
            ROCK_LOG_WARN(Hand,
                "{} hand loose grenade trigger could not arm because projectile/explosion/fuse data was missing: ref={:08X}",
                hand.handName(),
                heldRef->GetFormID());
            return true;
        }

        std::uint32_t impactBodyId = INVALID_CONTACT_BODY_ID;
        if (runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Impact) {
            const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
            if (!isInvalidGrabBodyId(primaryBodyId)) {
                impactBodyId = primaryBodyId;
            } else {
                for (const auto bodyId : hand.getHeldBodyIds()) {
                    if (!isInvalidGrabBodyId(bodyId)) {
                        impactBodyId = bodyId;
                        break;
                    }
                }
            }

            if (impactBodyId == INVALID_CONTACT_BODY_ID) {
                ROCK_LOG_WARN(Hand,
                    "{} hand loose Molotov trigger could not arm impact detonation because no held body id was available: ref={:08X}",
                    hand.handName(),
                    heldRef->GetFormID());
                return true;
            }
        }

        for (std::size_t slotIndex = 0; slotIndex < _armedLooseGrenadeFuses.size(); ++slotIndex) {
            auto& fuse = _armedLooseGrenadeFuses[slotIndex];
            if (fuse.active) {
                continue;
            }

            fuse = ArmedLooseGrenadeFuseState{
                .active = true,
                .handle = heldRef->GetHandle(),
                .refFormID = heldRef->GetFormID(),
                .runtime = runtime,
                .remainingSeconds = runtime.fuseSeconds,
                .impactBodyId = impactBodyId,
            };
            _armedLooseGrenadeImpactBodyIds[slotIndex].store(
                runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Impact ? impactBodyId : INVALID_CONTACT_BODY_ID,
                std::memory_order_release);
            ROCK_LOG_INFO(Hand,
                "{} hand armed loose grenade: ref={:08X} projectile={:08X} explosion={:08X} mode={} fuse={:.3f}s impactBody={} frameDt={:.4f}",
                hand.handName(),
                heldRef->GetFormID(),
                runtime.projectile ? runtime.projectile->GetFormID() : 0,
                runtime.explosion ? runtime.explosion->GetFormID() : 0,
                loose_grenade_runtime::detonationModeName(runtime.detonationMode),
                runtime.fuseSeconds,
                impactBodyId,
                frame.deltaSeconds);
            const bool feedbackPlayed = loose_grenade_runtime::playPinPulledFeedbackAtReference(heldRef);
            ROCK_LOG_DEBUG(Hand,
                "{} hand loose grenade pin-pull feedback: ref={:08X} played={}",
                hand.handName(),
                heldRef->GetFormID(),
                feedbackPlayed ? "yes" : "no");
            return true;
        }

        ROCK_LOG_WARN(Hand,
            "{} hand loose grenade trigger could not arm because armed grenade capacity is full: ref={:08X}",
            hand.handName(),
            heldRef->GetFormID());
        return true;
    }

    void PhysicsInteraction::updateLooseGrenadeFuses(const PhysicsFrameContext& frame)
    {
        const float deltaSeconds = (std::max)(0.0f, frame.deltaSeconds);
        if (deltaSeconds <= 0.0f) {
            return;
        }

        std::uint32_t pendingImpactBodyId = INVALID_CONTACT_BODY_ID;
        std::uint32_t pendingImpactOtherBodyId = INVALID_CONTACT_BODY_ID;
        const bool pendingImpact =
            unpackHeldImpactPair(
                _pendingLooseGrenadeImpactPair.exchange(INVALID_HELD_IMPACT_PAIR, std::memory_order_acq_rel),
                pendingImpactBodyId,
                pendingImpactOtherBodyId);

        auto releaseHandIfHolding = [&](Hand& hand, bool isLeft, RE::TESObjectREFR* ref, std::uint32_t formID) {
            if (!ref || !hand.isHolding() || hand.getHeldRef() != ref) {
                return;
            }

            const auto releaseContext = makeGrabReleaseContext(hand, isLeft);
            static_cast<void>(hand.releaseGrabbedObject(frame.hknpWorld, GrabReleaseCollisionRestoreMode::Immediate, releaseContext));
            releaseObject(ref, claimOwnerForHand(isLeft));
            dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, ref, formID, 0);
            dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, ref);
        };

        auto detonateLooseGrenade = [&](ArmedLooseGrenadeFuseState& fuse,
                                        std::size_t slotIndex,
                                        RE::TESObjectREFR* ref,
                                        const char* reason) {
            const std::uint32_t formID = ref ? ref->GetFormID() : fuse.refFormID;
            releaseHandIfHolding(_rightHand, false, ref, formID);
            releaseHandIfHolding(_leftHand, true, ref, formID);

            const bool explosionCreated = loose_grenade_runtime::createExplosionAtReference(ref, fuse.runtime.explosion);
            if (explosionCreated) {
                loose_grenade_runtime::disableAndDeleteReference(ref);
                ROCK_LOG_INFO(Hand,
                    "Loose grenade detonated: ref={:08X} explosion={:08X} mode={} reason={} impactBody={} otherBody={}",
                    formID,
                    fuse.runtime.explosion ? fuse.runtime.explosion->GetFormID() : 0,
                    loose_grenade_runtime::detonationModeName(fuse.runtime.detonationMode),
                    reason ? reason : "unknown",
                    fuse.impactBodyId,
                    pendingImpactOtherBodyId);
            } else {
                ROCK_LOG_WARN(Hand,
                    "Loose grenade detonation failed; leaving ref loose: ref={:08X} explosion={:08X} mode={} reason={}",
                    formID,
                    fuse.runtime.explosion ? fuse.runtime.explosion->GetFormID() : 0,
                    loose_grenade_runtime::detonationModeName(fuse.runtime.detonationMode),
                    reason ? reason : "unknown");
            }
            _armedLooseGrenadeImpactBodyIds[slotIndex].store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
            fuse = {};
        };

        for (std::size_t slotIndex = 0; slotIndex < _armedLooseGrenadeFuses.size(); ++slotIndex) {
            auto& fuse = _armedLooseGrenadeFuses[slotIndex];
            if (!fuse.active) {
                continue;
            }

            auto refPtr = fuse.handle.get();
            auto* ref = refPtr.get();
            if (!ref || ref->IsDeleted() || ref->IsDisabled()) {
                ROCK_LOG_DEBUG(Hand,
                    "Loose grenade fuse cleared because ref is gone: ref={:08X} remaining={:.3f}s",
                    fuse.refFormID,
                    fuse.remainingSeconds);
                _armedLooseGrenadeImpactBodyIds[slotIndex].store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
                fuse = {};
                continue;
            }

            if (fuse.runtime.detonationMode == loose_grenade_runtime::GrenadeDetonationMode::Impact) {
                if (!pendingImpact || pendingImpactBodyId != fuse.impactBodyId) {
                    continue;
                }
                if ((_rightHand.isHolding() && _rightHand.getHeldRef() == ref) ||
                    (_leftHand.isHolding() && _leftHand.getHeldRef() == ref)) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Loose Molotov impact ignored while still held: ref={:08X} impactBody={} otherBody={}",
                        fuse.refFormID,
                        pendingImpactBodyId,
                        pendingImpactOtherBodyId);
                    continue;
                }

                detonateLooseGrenade(fuse, slotIndex, ref, "impact");
                continue;
            }

            fuse.remainingSeconds -= deltaSeconds;
            if (fuse.remainingSeconds > 0.0f) {
                continue;
            }

            detonateLooseGrenade(fuse, slotIndex, ref, "timed-fuse");
        }
    }

    void PhysicsInteraction::processProviderInteractionCommands(const PhysicsFrameContext& frame)
    {
        using namespace provider;

        /*
         * Unregister/provider-loss clears API reservations immediately. Prune
         * their deferred runtime slots before admitting a replacement command
         * so a cancelled owner cannot cause a one-frame false HandBusy.
         */
        pruneInactiveProviderForceGrabCommits();

        QueuedInteractionCommandV1 command{};
        std::uint32_t processed = 0;
        while (processed++ < ROCK_PROVIDER_MAX_INTERACTION_COMMANDS_V1 && provider::dequeueInteractionCommandV1(command)) {
            pruneInactiveProviderForceGrabCommits();
            if (command.kind == RockProviderInteractionCommandKindV1::ForceGrab &&
                !provider::isInteractionCommandActiveV1(command.ownerToken, command.commandId)) {
                continue;
            }
            const auto requestHand = [&]() {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.hand;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.hand;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.hand;
                default:
                    return RockProviderHand::None;
                }
            }();
            const auto requestTargetFormId = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.targetFormId;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.targetFormId;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.targetFormId;
                default:
                    return 0;
                }
            }();
            const auto requestTargetBodyId = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.targetBodyId;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.targetBodyId;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.targetBodyId;
                default:
                    return INVALID_BODY_ID;
                }
            }();
            const auto requestWorldGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.worldGeneration;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.worldGeneration;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.worldGeneration;
                default:
                    return 0;
                }
            }();
            const auto requestSkeletonGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.skeletonGeneration;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.skeletonGeneration;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.skeletonGeneration;
                default:
                    return 0;
                }
            }();
            const auto requestProviderGeneration = [&]() -> std::uint32_t {
                switch (command.kind) {
                case RockProviderInteractionCommandKindV1::ForceGrab:
                    return command.forceGrab.providerGeneration;
                case RockProviderInteractionCommandKindV1::ForceRelease:
                    return command.forceRelease.providerGeneration;
                case RockProviderInteractionCommandKindV1::ThrownDrop:
                    return command.thrownDrop.providerGeneration;
                default:
                    return 0;
                }
            }();

            RockProviderInteractionCommandResultV1 result{};
            result.size = sizeof(RockProviderInteractionCommandResultV1);
            result.version = ROCK_PROVIDER_API_VERSION;
            result.ownerToken = command.ownerToken;
            result.commandId = command.commandId;
            result.kind = command.kind;
            result.state = RockProviderInteractionCommandStateV1::Rejected;
            result.failure = RockProviderInteractionFailureV1::InvalidRequest;
            result.hand = requestHand;
            result.targetFormId = requestTargetFormId;
            result.targetBodyId = requestTargetBodyId;
            result.frameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
            result.worldGeneration = _worldGenerationAtomic.load(std::memory_order_acquire);
            result.skeletonGeneration = _skeletonGenerationAtomic.load(std::memory_order_acquire);
            result.providerGeneration = _providerGenerationAtomic.load(std::memory_order_acquire);

            auto complete = [&](RockProviderInteractionCommandStateV1 state, RockProviderInteractionFailureV1 failure) {
                result.state = state;
                result.failure = failure;
                return provider::completeInteractionCommandV1(result);
            };

            const bool isForceGrabCommand = command.kind == RockProviderInteractionCommandKindV1::ForceGrab;
            const bool isForceReleaseCommand = command.kind == RockProviderInteractionCommandKindV1::ForceRelease;
            const bool isThrownDropCommand = command.kind == RockProviderInteractionCommandKindV1::ThrownDrop;
            if (!isForceGrabCommand && !isForceReleaseCommand && !isThrownDropCommand) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::InvalidRequest);
                continue;
            }

            if (!frame.worldReady || !frame.bhkWorld || !frame.hknpWorld) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::ProviderNotReady);
                continue;
            }
            if (!physicsWritesAllowedForWorld(frame.hknpWorld)) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::PhysicsWritesBlocked);
                continue;
            }
            if (requestWorldGeneration != 0 && requestWorldGeneration != result.worldGeneration) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::StaleWorldGeneration);
                continue;
            }
            if (requestSkeletonGeneration != 0 && requestSkeletonGeneration != result.skeletonGeneration) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::StaleSkeletonGeneration);
                continue;
            }
            if (requestProviderGeneration != 0 && requestProviderGeneration != result.providerGeneration) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::StaleProviderGeneration);
                continue;
            }

            const bool isLeft = requestHand == RockProviderHand::Left;
            if (requestHand != RockProviderHand::Left && requestHand != RockProviderHand::Right) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandInvalid);
                continue;
            }
            Hand& hand = isLeft ? _leftHand : _rightHand;
            const auto& handInput = isLeft ? frame.left : frame.right;

            auto hasTargetIdentity = [&]() {
                return requestTargetFormId != 0 || requestTargetBodyId != INVALID_BODY_ID;
            };
            auto heldObjectMatchesRequest = [&](std::uint32_t heldFormId, std::uint32_t primaryBodyId) {
                if (!hasTargetIdentity()) {
                    return true;
                }
                if (requestTargetFormId != 0 && heldFormId != requestTargetFormId) {
                    return false;
                }
                if (requestTargetBodyId != INVALID_BODY_ID && primaryBodyId != requestTargetBodyId && !hand.isHeldBodyId(requestTargetBodyId)) {
                    return false;
                }
                return true;
            };
            auto clearProviderReleaseInputState = [&]() {
                grab_input_intent_policy::reset(_grabInputIntentStates[isLeft ? 1u : 0u]);
                peer_held_join_retry_policy::reset(_peerHeldJoinRetryStates[isLeft ? 1u : 0u]);
                shoulder_stash::resetRuntime(_shoulderStashStates[isLeft ? 1u : 0u]);
                mouth_consume::resetRuntime(_mouthConsumeStates[isLeft ? 1u : 0u]);
                hand.cancelStashCandidate();
                hand.cancelConsumeCandidate();
                input_remap_runtime::setHandHeldWeapon(isLeft, hand.isHoldingLooseWeapon());
            };

            if (isForceReleaseCommand || isThrownDropCommand) {
                if (!hand.isHolding()) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandNotHolding);
                    continue;
                }

                auto* heldRef = hand.getHeldRef();
                const std::uint32_t heldFormId = heldRef ? heldRef->GetFormID() : 0u;
                const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                result.targetFormId = heldFormId;
                result.targetBodyId = primaryBodyId;
                if (!heldObjectMatchesRequest(heldFormId, primaryBodyId)) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HeldObjectMismatch);
                    continue;
                }

                const Hand& peer = isLeft ? _rightHand : _leftHand;
                if (isThrownDropCommand && heldRef && peer.isHolding() && peer.getHeldRef() == heldRef) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                    continue;
                }

                GrabReleaseOutcome::VelocitySnapshot requestedVelocity{};
                const bool forceReleaseUsesVelocity = isForceReleaseCommand &&
                    (command.forceRelease.flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::UseVelocityHavok)) != 0;
                const bool thrownDropUsesVelocity = isThrownDropCommand &&
                    (command.thrownDrop.flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::UseVelocityHavok)) != 0;
                const bool applyRequestedVelocity = forceReleaseUsesVelocity || thrownDropUsesVelocity;
                if (applyRequestedVelocity) {
                    const float* linearVelocityHavok = forceReleaseUsesVelocity ?
                        command.forceRelease.linearVelocityHavok :
                        command.thrownDrop.linearVelocityHavok;
                    const float* angularVelocityRadiansPerSecond = forceReleaseUsesVelocity ?
                        command.forceRelease.angularVelocityRadiansPerSecond :
                        command.thrownDrop.angularVelocityRadiansPerSecond;

                    requestedVelocity.available = true;
                    requestedVelocity.primaryBodyId = RE::hknpBodyId{ primaryBodyId };
                    requestedVelocity.linearVelocityHavok = RE::NiPoint3{
                        linearVelocityHavok[0],
                        linearVelocityHavok[1],
                        linearVelocityHavok[2],
                    };
                    requestedVelocity.angularVelocityRadiansPerSecond = RE::NiPoint3{
                        angularVelocityRadiansPerSecond[0],
                        angularVelocityRadiansPerSecond[1],
                        angularVelocityRadiansPerSecond[2],
                    };
                    requestedVelocity.overrideAngularVelocity = true;
                    for (const auto bodyId : hand.getHeldBodyIds()) {
                        if (requestedVelocity.bodyCount >= requestedVelocity.bodyIds.size()) {
                            break;
                        }
                        requestedVelocity.bodyIds[requestedVelocity.bodyCount++] = bodyId;
                    }
                }

                if (isThrownDropCommand && !applyRequestedVelocity) {
                    hand.captureHeldReleaseMotion(frame.hknpWorld, handInput.rawHandWorld, frame.deltaSeconds);
                }

                const std::uint32_t flags = isThrownDropCommand ? command.thrownDrop.flags : command.forceRelease.flags;
                const bool immediateCollisionRestore = isThrownDropCommand ?
                    (flags & static_cast<std::uint32_t>(RockProviderThrownDropFlagV1::ImmediateCollisionRestore)) != 0 :
                    (flags & static_cast<std::uint32_t>(RockProviderForceReleaseFlagV1::ImmediateCollisionRestore)) != 0;
                auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                releaseContext.disposition = GrabReleaseDisposition::PhysicalDrop;
                releaseContext.applyCapturedReleaseVelocity = isThrownDropCommand && !applyRequestedVelocity;
                releaseContext.reason = isThrownDropCommand ? "provider-thrown-drop" : "provider-force-release";
                const auto releaseOutcome = hand.releaseGrabbedObject(frame.hknpWorld,
                    immediateCollisionRestore ? GrabReleaseCollisionRestoreMode::Immediate : GrabReleaseCollisionRestoreMode::Delayed,
                    releaseContext);
                if (!releaseOutcome.released) {
                    complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandNotHolding);
                    continue;
                }

                if (heldRef) {
                    releaseObject(heldRef, claimOwnerForHand(isLeft));
                }
                if (applyRequestedVelocity && releaseContext.finalObjectRelease) {
                    hand.applyReleaseVelocitySnapshot(frame.hknpWorld, requestedVelocity);
                }
                dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormId, 0);
                dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                clearProviderReleaseInputState();
                complete(RockProviderInteractionCommandStateV1::Succeeded, RockProviderInteractionFailureV1::None);
                continue;
            }

            if (handInput.disabled) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandDisabled);
                continue;
            }
            if (forceGrabHandBlockerMask(hand, isLeft, false, true) != 0) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            if (command.forceGrab.targetFormId == 0) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::InvalidRequest);
                continue;
            }

            auto* targetRef = RE::TESForm::GetFormByID<RE::TESObjectREFR>(command.forceGrab.targetFormId);
            if (!targetRef) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetMissing);
                continue;
            }
            result.targetFormId = targetRef->GetFormID();
            if (targetRef->IsDeleted() || targetRef->IsDisabled()) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetUnavailable);
                continue;
            }
            if (command.forceGrab.targetFormId != 0 && targetRef->GetFormID() != command.forceGrab.targetFormId) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetUnavailable);
                continue;
            }
            if (physicsModOwnsObject(targetRef)) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::TargetAlreadyOwned);
                continue;
            }

            const bool targetIsLooseGrenade = loose_grenade_runtime::isGrenadeRef(targetRef);
            if (targetIsLooseGrenade &&
                (handHoldsLooseGrenade(_rightHand) ||
                    handHoldsLooseGrenade(_leftHand) ||
                    hasActiveLooseGrenadeCommit())) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            auto& commit = _pendingForceGrabCommits[isLeft ? 1u : 0u];
            if (commit.active) {
                complete(RockProviderInteractionCommandStateV1::Rejected, RockProviderInteractionFailureV1::HandBusy);
                continue;
            }

            const bool hasSourcePointOverride =
                (command.forceGrab.flags & static_cast<std::uint32_t>(RockProviderForceGrabFlagV1::UsePreferredGrabPointGame)) != 0;

            commit = PendingForceGrabCommit{
                .active = true,
                .isLeft = isLeft,
                .origin = PendingForceGrabCommitOrigin::ProviderForceGrabCommand,
                .phase = PendingForceGrabCommitPhase::WaitingForSettle,
                .targetHandle = targetRef->GetHandle(),
                .targetIsLooseGrenade = targetIsLooseGrenade,
                .preferredBodyId = command.forceGrab.targetBodyId,
                .maxDistanceGame = command.forceGrab.maxDistanceGame,
                .hasSourcePointOverride = hasSourcePointOverride,
                .sourcePointOverride = hasSourcePointOverride ?
                    RE::NiPoint3{
                        command.forceGrab.preferredGrabPointGame[0],
                        command.forceGrab.preferredGrabPointGame[1],
                        command.forceGrab.preferredGrabPointGame[2],
                    } :
                    RE::NiPoint3{},
                .providerResultTemplate = result,
            };
            if (!complete(RockProviderInteractionCommandStateV1::Queued, RockProviderInteractionFailureV1::None)) {
                commit = {};
                continue;
            }
        }
    }

    std::size_t PhysicsInteraction::applyProviderWeaponPartDrives(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        const PhysicsFrameContext& frame,
        std::array<const RE::NiAVObject*, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>& outDrivenSourceNodes)
    {
        outDrivenSourceNodes = {};
        _providerWeaponPartDriveResultCount = 0;
        if (!weaponNode || currentWeaponGenerationKey == 0 || !frame.worldReady) {
            if (weaponNode && currentWeaponGenerationKey != 0) {
                restoreExpiredProviderWeaponPartDriveNodes(weaponNode, currentWeaponGenerationKey);
            }
            return 0;
        }

        if (_providerWeaponPartDriveGenerationKey != 0 && _providerWeaponPartDriveGenerationKey != currentWeaponGenerationKey) {
            _providerWeaponPartDriveNodeStates = {};
            _providerWeaponPartDriveGenerationKey = 0;
        }
        for (auto& state : _providerWeaponPartDriveNodeStates) {
            state.activeThisFrame = false;
        }

        std::array<::rock::provider::RockProviderWeaponPartDriveTargetV1, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> driveTargets{};
        std::array<std::uint64_t,
            ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1>
            driveOwners{};
        const std::uint32_t driveCount = ::rock::provider::copyWeaponPartDriveTargetsV1(
            driveTargets.data(),
            static_cast<std::uint32_t>(driveTargets.size()),
            driveOwners.data());
        if (driveCount == 0) {
            restoreExpiredProviderWeaponPartDriveNodes(weaponNode, currentWeaponGenerationKey);
            return 0;
        }

        struct AppliedNode
        {
            RE::NiAVObject* node{ nullptr };
            std::uint32_t priority{ 0 };
            std::uint32_t resultIndex{ 0 };
        };

        std::array<AppliedNode, ::rock::provider::ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1> appliedNodes{};
        std::size_t appliedNodeCount = 0;
        const auto evidenceDescriptors = _weaponCollision.getProfileEvidenceDescriptors();

        auto resolveDriveNode = [&](const ::rock::provider::RockProviderWeaponPartDriveTargetV1& drive) -> RE::NiAVObject* {
            if (drive.weaponGenerationKey != 0 && drive.weaponGenerationKey != currentWeaponGenerationKey) {
                return nullptr;
            }

            RE::NiAVObject* candidate = nullptr;
            auto acceptResolvedNode = [&](RE::NiAVObject* node) {
                if (!node || !actor_equipment_grab::nodeContainsNode(weaponNode, node, 64)) {
                    return false;
                }
                if (candidate && candidate != node) {
                    return false;
                }
                candidate = node;
                return true;
            };

            if ((drive.flags & static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchSourceRoot)) != 0 && drive.sourceRoot != 0) {
                auto* node = reinterpret_cast<RE::NiAVObject*>(drive.sourceRoot);
                if (!acceptResolvedNode(node)) {
                    return nullptr;
                }
            }

            if ((drive.flags & static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchBodyId)) != 0 && drive.bodyId != INVALID_CONTACT_BODY_ID) {
                WeaponCollisionProfileEvidenceDescriptor descriptor{};
                RE::NiAVObject* sourceNode = nullptr;
                if (_weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(drive.bodyId, descriptor, sourceNode) &&
                    sourceNode &&
                    descriptor.weaponGenerationKey == currentWeaponGenerationKey &&
                    acceptResolvedNode(sourceNode)) {
                } else {
                    return nullptr;
                }
            }

            const auto sourceName = providerFixedStringView(drive.sourceName, ::rock::provider::ROCK_PROVIDER_MAX_EVIDENCE_NAME);
            if ((drive.flags & static_cast<std::uint32_t>(::rock::provider::RockProviderWeaponPartTargetFlagV1::MatchSourceName)) != 0 && !sourceName.empty()) {
                RE::NiAVObject* matchedSourceNode = nullptr;
                for (const auto& descriptor : evidenceDescriptors) {
                    if (!descriptor.valid || descriptor.weaponGenerationKey != currentWeaponGenerationKey || descriptor.sourceName != sourceName) {
                        continue;
                    }
                    matchedSourceNode = reinterpret_cast<RE::NiAVObject*>(descriptor.sourceRootAddress);
                    break;
                }
                if (!matchedSourceNode) {
                    matchedSourceNode = findWeaponNodeBySourceName(weaponNode, sourceName, 32);
                }
                if (!acceptResolvedNode(matchedSourceNode)) {
                    return nullptr;
                }
            }

            return candidate;
        };

        auto shouldApplyPriority = [&](
                                       RE::NiAVObject* node,
                                       std::uint32_t priority,
                                       const std::uint32_t resultIndex) {
            for (std::size_t i = 0; i < appliedNodeCount; ++i) {
                if (appliedNodes[i].node != node) {
                    continue;
                }
                if (priority < appliedNodes[i].priority) {
                    return false;
                }
                _providerWeaponPartDriveResults[
                    appliedNodes[i].resultIndex].result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::LostPriority;
                appliedNodes[i].priority = priority;
                appliedNodes[i].resultIndex = resultIndex;
                return true;
            }
            if (appliedNodeCount < appliedNodes.size()) {
                appliedNodes[appliedNodeCount++] = AppliedNode{
                    .node = node,
                    .priority = priority,
                    .resultIndex = resultIndex,
                };
                return true;
            }
            return false;
        };

        auto markDrivenNode = [&](
                                  RE::NiAVObject* node,
                                  const ::rock::provider::RockProviderWeaponPartDriveApplicationResultV1& result) {
            if (!node) {
                return false;
            }
            if (_providerWeaponPartDriveGenerationKey == 0) {
                _providerWeaponPartDriveGenerationKey = currentWeaponGenerationKey;
            }
            for (auto& state : _providerWeaponPartDriveNodeStates) {
                if (state.node == node) {
                    state.activeThisFrame = true;
                    state.ownerToken = result.ownerToken;
                    state.bodyId = result.bodyId;
                    state.groupId = result.groupId;
                    state.priority = result.priority;
                    std::memcpy(
                        state.sourceName.data(),
                        result.sourceName,
                        state.sourceName.size());
                    return true;
                }
            }
            for (auto& state : _providerWeaponPartDriveNodeStates) {
                if (!state.node) {
                    state.node = node;
                    state.baselineLocal = node->local;
                    state.ownerToken = result.ownerToken;
                    state.bodyId = result.bodyId;
                    state.groupId = result.groupId;
                    state.priority = result.priority;
                    std::memcpy(
                        state.sourceName.data(),
                        result.sourceName,
                        state.sourceName.size());
                    state.activeThisFrame = true;
                    return true;
                }
            }
            return false;
        };

        std::size_t drivenSourceNodeCount = 0;
        for (std::uint32_t i = 0; i < driveCount && i < driveTargets.size(); ++i) {
            const auto& drive = driveTargets[i];
            auto& applicationResult = _providerWeaponPartDriveResults[
                _providerWeaponPartDriveResultCount++];
            applicationResult = {};
            applicationResult.frameIndex =
                _palmClockGameFrameIndex.load(std::memory_order_acquire);
            applicationResult.ownerToken = driveOwners[i];
            applicationResult.weaponGenerationKey =
                currentWeaponGenerationKey;
            applicationResult.bodyId = drive.bodyId;
            applicationResult.groupId = drive.groupId;
            applicationResult.priority = drive.priority;
            std::memcpy(
                applicationResult.sourceName,
                drive.sourceName,
                sizeof(applicationResult.sourceName));
            applicationResult.sourceName[
                sizeof(applicationResult.sourceName) - 1] = '\0';
            if (drive.weaponGenerationKey != 0 &&
                drive.weaponGenerationKey != currentWeaponGenerationKey) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::StaleGeneration;
                continue;
            }
            auto* sourceNode = resolveDriveNode(drive);
            if (!sourceNode) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::Unresolved;
                continue;
            }
            if (!sourceNode->parent) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::MissingParent;
                continue;
            }
            const RE::NiTransform requestedLocal = providerTransformToNi(drive.targetTransform);
            if (!finiteNiTransform(requestedLocal)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::InvalidTransform;
                continue;
            }

            RE::NiTransform desiredWorld{};
            switch (drive.driveSpace) {
            case ::rock::provider::RockProviderWeaponPartDriveSpaceV1::SourceParentLocal:
                desiredWorld = transform_math::composeTransforms(sourceNode->parent->world, requestedLocal);
                break;
            case ::rock::provider::RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal:
            default:
                desiredWorld = transform_math::composeTransforms(weaponNode->world, requestedLocal);
                break;
            }
            if (!finiteNiTransform(desiredWorld)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::InvalidTransform;
                continue;
            }

            const RE::NiTransform requestedNodeLocal = transform_math::composeTransforms(transform_math::invertTransform(sourceNode->parent->world), desiredWorld);
            if (!finiteNiTransform(requestedNodeLocal)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::InvalidTransform;
                continue;
            }
            if (!shouldApplyPriority(
                    sourceNode,
                    drive.priority,
                    _providerWeaponPartDriveResultCount - 1)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::LostPriority;
                continue;
            }
            if (!markDrivenNode(sourceNode, applicationResult)) {
                applicationResult.result =
                    ::rock::provider::RockProviderWeaponPartDriveApplicationV1::CapacityRejected;
                continue;
            }
            sourceNode->local = requestedNodeLocal;
            f4vr::updateTransformsDown(sourceNode, true);
            applicationResult.result =
                ::rock::provider::RockProviderWeaponPartDriveApplicationV1::Applied;
            fillProviderTransform(
                requestedNodeLocal,
                applicationResult.appliedSourceParentLocal);

            if (drivenSourceNodeCount < outDrivenSourceNodes.size()) {
                outDrivenSourceNodes[drivenSourceNodeCount++] = sourceNode;
            }
        }

        restoreExpiredProviderWeaponPartDriveNodes(weaponNode, currentWeaponGenerationKey);
        return drivenSourceNodeCount;
    }

    void PhysicsInteraction::restoreExpiredProviderWeaponPartDriveNodes(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey)
    {
        if (_providerWeaponPartDriveGenerationKey == 0) {
            return;
        }
        if (!weaponNode || currentWeaponGenerationKey == 0 || _providerWeaponPartDriveGenerationKey != currentWeaponGenerationKey) {
            _providerWeaponPartDriveNodeStates = {};
            _providerWeaponPartDriveGenerationKey = 0;
            return;
        }

        bool anyActive = false;
        for (auto& state : _providerWeaponPartDriveNodeStates) {
            if (!state.node) {
                continue;
            }
            if (state.activeThisFrame) {
                anyActive = true;
                continue;
            }
            if (actor_equipment_grab::nodeContainsNode(weaponNode, state.node, 64)) {
                state.node->local = state.baselineLocal;
                f4vr::updateTransformsDown(state.node, true);
                if (_providerWeaponPartDriveResultCount <
                    _providerWeaponPartDriveResults.size()) {
                    auto& result = _providerWeaponPartDriveResults[
                        _providerWeaponPartDriveResultCount++];
                    result = {};
                    result.frameIndex =
                        _palmClockGameFrameIndex.load(
                            std::memory_order_acquire);
                    result.ownerToken = state.ownerToken;
                    result.weaponGenerationKey =
                        currentWeaponGenerationKey;
                    result.bodyId = state.bodyId;
                    result.groupId = state.groupId;
                    result.priority = state.priority;
                    result.result =
                        ::rock::provider::RockProviderWeaponPartDriveApplicationV1::Restored;
                    fillProviderTransform(
                        state.baselineLocal,
                        result.appliedSourceParentLocal);
                    std::memcpy(
                        result.sourceName,
                        state.sourceName.data(),
                        sizeof(result.sourceName));
                    result.sourceName[sizeof(result.sourceName) - 1] = '\0';
                }
            }
            state = {};
        }
        if (!anyActive) {
            _providerWeaponPartDriveGenerationKey = 0;
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
        _forceGrabCommittedThisFrame = {};

        auto clearShoulderStashForHand = [&](Hand& hand, bool isLeft) {
            shoulder_stash::resetRuntime(_shoulderStashStates[isLeft ? 1u : 0u]);
            hand.cancelStashCandidate();
        };

        auto clearMouthConsumeForHand = [&](Hand& hand, bool isLeft) {
            mouth_consume::resetRuntime(_mouthConsumeStates[isLeft ? 1u : 0u]);
            hand.cancelConsumeCandidate();
        };

        auto clearGameplayCandidatesForHand = [&](Hand& hand, bool isLeft) {
            clearShoulderStashForHand(hand, isLeft);
            clearMouthConsumeForHand(hand, isLeft);
        };

        if (!runtime_state::isLocalSkeletonReady()) {
            _touchGrabRuntime.releaseAll(
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                _collisionGenerationAtomic.load(
                    std::memory_order_acquire));
            _rightHand.cancelGrabVisualReturn("skeleton-not-ready");
            _leftHand.cancelGrabVisualReturn("skeleton-not-ready");
            provider::clearInteractionCommandsForProviderLossV1(provider::RockProviderInteractionFailureV1::ProviderNotReady);
            clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin::ProviderForceGrabCommand);
            input_remap_runtime::setHandHeldWeapon(false, false);
            input_remap_runtime::setHandHeldWeapon(true, false);
            input_remap_runtime::setHandInteractionEngaged(false, false);
            input_remap_runtime::setHandInteractionEngaged(true, false);
            input_remap_runtime::setHeldObjectFormId(false, 0u);
            input_remap_runtime::setHeldObjectFormId(true, 0u);
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
            input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
            input_remap_runtime::setProviderOpenVrGameInputSuppressed(false, false);
            input_remap_runtime::setProviderOpenVrGameInputSuppressed(true, false);
            _heldWeaponTriggerEquipIntents = {};
            clearGameplayCandidatesForHand(_rightHand, false);
            clearGameplayCandidatesForHand(_leftHand, true);
            return;
        }

        auto* hknp = frame.hknpWorld;
        const auto worldGeneration =
            _worldGenerationAtomic.load(
                std::memory_order_acquire);
        const auto skeletonGeneration =
            _skeletonGenerationAtomic.load(
                std::memory_order_acquire);
        const auto providerGeneration =
            _providerGenerationAtomic.load(
                std::memory_order_acquire);
        const auto collisionGeneration =
            _collisionGenerationAtomic.load(
                std::memory_order_acquire);
        _touchGrabRuntime.setGlobalSurfaceGrabEnabled(
            g_rockConfig.rockGlobalSurfaceGrabEnabled);
        _touchGrabRuntime.service(
            frame.bhkWorld,
            frame.hknpWorld,
            frame.deltaSeconds,
            worldGeneration,
            skeletonGeneration,
            providerGeneration,
            collisionGeneration);
        if (frame.menuBlocked) {
            _touchGrabRuntime.releaseAll(
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                collisionGeneration);
        }
        int grabButton = g_rockConfig.rockGrabButtonID;
        const bool rightHandWeaponEquipped = resolveEquippedWeaponInteractionNode() != nullptr;
        const bool ambidextrousHandoffAvailable =
            _equippedWeaponHandlingSettings.ambidextrousHandoffEnabled &&
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        const equipped_weapon_manual_ownership_policy::FiringGripModeAvailability firingGripModes{
            .primaryDetachEnabled = _equippedWeaponHandlingSettings.primaryDetachEnabled,
            .ambidextrousHandoffAvailable = ambidextrousHandoffAvailable,
        };
        const bool gripZoneSettleEquipEnabled =
            equipped_weapon_manual_ownership_policy::canSettleEquipInGripZone(
                _equippedWeaponHandlingSettings.gripZoneEquipEnabled);
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);
        auto publishHandInputOwnership = [&](const Hand& hand, const bool isLeft) {
            auto* heldRef = hand.isHolding() ? hand.getHeldRef() : nullptr;
            const bool pendingEquippedGripOwnership =
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft == isLeft;
            input_remap_runtime::setHandHeldWeapon(isLeft, hand.isHoldingLooseWeapon());
            // Engaged = holding a ROCK object or gripping the equipped weapon (support/two-hand, part carry while primary detached, attach-only glue).
            input_remap_runtime::setHandInteractionEngaged(
                isLeft,
                hand.isHolding() ||
                    _touchGrabRuntime.isHandActive(isLeft) ||
                    _twoHandedGrip.isHandPartGripping(isLeft) ||
                    pendingEquippedGripOwnership);
            input_remap_runtime::setHeldObjectFormId(isLeft, heldRef ? heldRef->GetFormID() : 0u);
        };
        publishHandInputOwnership(_rightHand, false);
        publishHandInputOwnership(_leftHand, true);
        processProviderInteractionCommands(frame);
        serviceLooseGrenadeQuickDraw(frame);
        servicePendingForceGrabCommits(frame);
        updateSavedGrabOffsetGesture(frame);
        serviceEquippedWeaponDropMomentumHandoff(frame);
        updateLooseGrenadeFuses(frame);
        publishHandInputOwnership(_rightHand, false);
        publishHandInputOwnership(_leftHand, true);

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
            auto& peerHeldJoinRetryState = _peerHeldJoinRetryStates[isLeft ? 1u : 0u];
            auto& triggerEquipIntent = _heldWeaponTriggerEquipIntents[isLeft ? 1u : 0u];
            auto& shoulderStashState = _shoulderStashStates[isLeft ? 1u : 0u];
            auto& mouthConsumeState = _mouthConsumeStates[isLeft ? 1u : 0u];
            auto& inputSuppressionState = _providerHandInputSuppressionStates[isLeft ? 1u : 0u];
            const bool heldWeaponAtFrameStart = hand.isHoldingLooseWeapon();
            const auto providerHand = isLeft ? provider::RockProviderHand::Left : provider::RockProviderHand::Right;
            const std::uint32_t providerInputSuppressionFlags = provider::currentHandInputSuppressionFlagsV1(providerHand);
            auto providerSuppresses = [&](provider::RockProviderHandInputSuppressionFlagV1 flag) {
                return provider::hasHandInputSuppressionFlagV1(providerInputSuppressionFlags, flag);
            };
            const bool providerSuppressesNormalGrabPress =
                providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressNormalGrabPress);
            const bool providerSuppressesGrabRelease =
                providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressGrabRelease);
            const bool providerSuppressesHeldWeaponTriggerEquip =
                providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressHeldWeaponTriggerEquip);
            const bool providerSuppressesGameplayCandidates =
                providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressGameplayCandidates);
            const bool providerSuppressesOpenVrGameInput =
                providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput);
            input_remap_runtime::setProviderOpenVrGameInputSuppressed(isLeft, providerSuppressesOpenVrGameInput);
            auto cancelPeerHeldJoinRetry = [&](const char* reason, bool logCancellation) {
                if (!peerHeldJoinRetryState.active) {
                    return;
                }
                const auto peerFormId = peerHeldJoinRetryState.peerFormId;
                const auto attempts = peerHeldJoinRetryState.attempts;
                const char* lastRefusal = peerHeldJoinRetryState.lastRefusalReason ? peerHeldJoinRetryState.lastRefusalReason : "none";
                peer_held_join_retry_policy::reset(peerHeldJoinRetryState);
                if (logCancellation) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand peer-held join retry cancelled: reason={} peerFormID={:08X} attempts={} lastRefusal={}",
                        hand.handName(),
                        reason ? reason : "unknown",
                        peerFormId,
                        attempts,
                        lastRefusal);
                }
            };

            const auto handIndex = isLeft ? 1u : 0u;
            if (_equippedWeaponSheathCommittedThisFrame[handIndex] ||
                _equippedWeaponUnsheathCommittedThisFrame[handIndex]) {
                // Consume the shoulder gesture once. The equipped-weapon path
                // already used this edge to sheath, or used the squeeze to
                // draw and claim this hand.
                if (_firingHandGrabButtonFrameState.valid &&
                    _firingHandGrabButtonFrameState.isLeft == isLeft) {
                    _firingHandGrabButtonFrameState.valid = false;
                } else {
                    static_cast<void>(
                        readGrabButtonState(isLeft, grabButton));
                }
                inputSuppressionState.deferredGrabRelease = false;
                grab_input_intent_policy::reset(inputIntentState);
                cancelPeerHeldJoinRetry(
                    "equipped-weapon-shoulder-gesture-this-frame",
                    true);
                clearGameplayCandidatesForHand(hand, isLeft);
                if (hand.hasSelection()) {
                    hand.clearSelectionState(false);
                }
                return;
            }
            if (_forceGrabCommittedThisFrame[handIndex]) {
                /*
                 * Consume, but do not apply, the physical button edges from
                 * before this programmatic attachment. Otherwise a stale
                 * release from the Pip-Boy/API frame can drop the object in
                 * the same update that reported a successful force-grab.
                 */
                if (_firingHandGrabButtonFrameState.valid && _firingHandGrabButtonFrameState.isLeft == isLeft) {
                    _firingHandGrabButtonFrameState.valid = false;
                } else {
                    static_cast<void>(readGrabButtonState(isLeft, grabButton));
                }
                inputSuppressionState.deferredGrabRelease = false;
                grab_input_intent_policy::reset(inputIntentState);
                cancelPeerHeldJoinRetry("force-grab-committed-this-frame", true);
                clearGameplayCandidatesForHand(hand, isLeft);
                return;
            }
            if (_pendingForceGrabCommits[handIndex].active) {
                grab_input_intent_policy::reset(inputIntentState);
                cancelPeerHeldJoinRetry("pending-force-grab-reservation", true);
                clearGameplayCandidatesForHand(hand, isLeft);
                if (hand.hasSelection()) {
                    hand.clearSelectionState(false);
                }
                return;
            }

            const auto& peerCommit = _pendingForceGrabCommits[isLeft ? 0u : 1u];
            if (peerCommit.active && hand.hasSelection()) {
                const auto peerTargetPtr = peerCommit.targetHandle.get();
                if (peerTargetPtr && hand.getSelection().refr == peerTargetPtr.get()) {
                    hand.clearSelectionState(false);
                }
            }
            if (handInput.disabled) {
                _touchGrabRuntime.releaseHand(
                    isLeft,
                    frame.bhkWorld,
                    frame.hknpWorld,
                    provider::RockProviderTouchGrabReleaseReasonV1::
                        HandUnavailable,
                    collisionGeneration);
                cancelPeerHeldJoinRetry("hand-input-disabled", false);
                clearGameplayCandidatesForHand(hand, isLeft);
                return;
            }
            if (providerSuppressesGameplayCandidates) {
                clearGameplayCandidatesForHand(hand, isLeft);
            }
            const bool providerHoldsCurrentGrabState =
                providerSuppressesGrabRelease &&
                (hand.isHolding() ||
                    _touchGrabRuntime.isHandActive(isLeft) ||
                    hand.getState() == HandState::SelectionLocked ||
                    hand.getState() == HandState::Pulled);
            const bool providerBlocksNewGrabPress =
                providerSuppressesNormalGrabPress &&
                !hand.isHolding() &&
                !_touchGrabRuntime.isHandActive(isLeft) &&
                hand.getState() != HandState::SelectionLocked &&
                hand.getState() != HandState::Pulled;
            if (providerHoldsCurrentGrabState || providerBlocksNewGrabPress) {
                grab_input_intent_policy::reset(inputIntentState);
                cancelPeerHeldJoinRetry("provider-hand-input-suppressed", true);
                if (providerHoldsCurrentGrabState &&
                    !readGrabButtonHeld(isLeft, grabButton)) {
                    inputSuppressionState.deferredGrabRelease = true;
                }
                return;
            }

            const bool heldWeaponEquipTriggerPressedEdge =
                !providerSuppressesHeldWeaponTriggerEquip && readHeldWeaponEquipTriggerPressedEdge(isLeft);
            const bool handIsFiringHand = isLeft == _twoHandedGrip.isFiringHandLeft();
            if (!weapon_two_handed_grip_math::canProcessNormalGrabInput(
                    handIsFiringHand,
                    rightHandWeaponEquipped,
                    _twoHandedGrip.isHandPartGripping(isLeft),
                    _twoHandedGrip.isPartCarryActive() && !_twoHandedGrip.isHandPartGripping(isLeft))) {
                grab_input_intent_policy::reset(inputIntentState);
                cancelPeerHeldJoinRetry("normal-grab-suppressed", true);
                clearGameplayCandidatesForHand(hand, isLeft);
                if (_touchGrabRuntime.isHandActive(isLeft)) {
                    _touchGrabRuntime.releaseHand(
                        isLeft,
                        frame.bhkWorld,
                        frame.hknpWorld,
                        provider::RockProviderTouchGrabReleaseReasonV1::
                            HandUnavailable,
                        collisionGeneration);
                }
                if (hand.isHolding()) {
                    releaseSuppressedHeldObject(hand, isLeft, handIsFiringHand ? "firing-hand weapon equipped" : "equipped weapon support grip active");
                } else if (hand.hasActivePullCatchIntent()) {
                    auto* pullCatchRef = hand.getPullCatchIntentRef();
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-normal-grab-suppressed");
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
                    } else {
                        hand.finishPullPrepAsPhysicalDropIfActive("pull-normal-grab-suppressed");
                    }
                    hand.clearSelectionState(true);
                    releaseObject(selectedRef, claimOwnerForHand(isLeft));
                    ROCK_LOG_DEBUG(Hand, "{} hand: cleared pull/locked selection because normal grab input is suppressed", hand.handName());
                }
                return;
            }

            /*
             * The equipped-weapon manual ownership path consumes the firing
             * hand's grab edges earlier this frame. Reuse that single consumed
             * snapshot for the same physical hand; re-reading would see
             * cleared edges and starve free-hand world grabs of press/release
             * input.
             */
            GrabButtonState grabInput{};
            if (_firingHandGrabButtonFrameState.valid && _firingHandGrabButtonFrameState.isLeft == isLeft) {
                grabInput = GrabButtonState{
                    .held = _firingHandGrabButtonFrameState.held,
                    .pressed = _firingHandGrabButtonFrameState.pressed,
                    .released = _firingHandGrabButtonFrameState.released,
                };
                _firingHandGrabButtonFrameState.valid = false;
            } else {
                grabInput = readGrabButtonState(isLeft, grabButton);
            }
            if (inputSuppressionState.deferredGrabRelease) {
                if (grabInput.held) {
                    inputSuppressionState.deferredGrabRelease = false;
                } else {
                    if (!grabInput.released &&
                        (hand.isHolding() ||
                            _touchGrabRuntime.isHandActive(isLeft) ||
                            hand.getState() == HandState::SelectionLocked ||
                            hand.getState() == HandState::Pulled)) {
                        grabInput.released = true;
                    }
                    inputSuppressionState.deferredGrabRelease = false;
                }
            }
            const auto rawGrabInput = grabInput;

            /*
             * Provider-registered touch targets consume the same physical
             * grip edge as ordinary grabs, but live in a separate runtime so
             * ROCK's loose-object selection policy continues to reject static
             * and keyframed bodies. An explicit body registration always wins
             * over a wildcard fixed-surface registration when one press has
             * contact evidence for both.
             */
            if (_touchGrabRuntime.isHandActive(isLeft)) {
                if (rawGrabInput.released ||
                    !rawGrabInput.held) {
                    _touchGrabRuntime.releaseHand(
                        isLeft,
                        frame.bhkWorld,
                        frame.hknpWorld,
                        provider::RockProviderTouchGrabReleaseReasonV1::
                            GripReleased,
                        collisionGeneration);
                }
                grab_input_intent_policy::reset(inputIntentState);
                cancelPeerHeldJoinRetry(
                    "touch-grab-active",
                    true);
                clearGameplayCandidatesForHand(hand, isLeft);
                hand.cancelGrabVisualReturn(
                    "touch-grab-active");
                return;
            }

            const auto handState = hand.getState();
            const bool touchGrabStateAvailable =
                handState == HandState::Idle ||
                handState == HandState::SelectedClose ||
                handState == HandState::SelectedFar;
            const bool touchGrabPhysicsWritesAllowed =
                physicsWritesAllowedForWorld(frame.hknpWorld);
            const bool canTryTouchGrab =
                rawGrabInput.pressed &&
                rawGrabInput.held &&
                frame.worldReady &&
                !frame.menuBlocked &&
                !input_remap_runtime::isMenuInputActive() &&
                !hand.isHolding() &&
                touchGrabStateAvailable &&
                !hand.hasActivePullCatchIntent() &&
                !hand.hasPendingActorEquipmentDropHandoff() &&
                !_pendingForceGrabCommits[handIndex].active &&
                touchGrabPhysicsWritesAllowed;
            if (rawGrabInput.pressed && !canTryTouchGrab) {
                ROCK_LOG_INFO(
                    Hand,
                    "Touch grab edge gated: hand={} held={} worldReady={} menuBlocked={} menuInput={} holding={} handState={} stateAvailable={} pullCatch={} actorDrop={} forceCommit={} physicsWrites={}",
                    isLeft ? "left" : "right",
                    rawGrabInput.held,
                    frame.worldReady,
                    frame.menuBlocked,
                    input_remap_runtime::isMenuInputActive(),
                    hand.isHolding(),
                    static_cast<std::uint32_t>(handState),
                    touchGrabStateAvailable,
                    hand.hasActivePullCatchIntent(),
                    hand.hasPendingActorEquipmentDropHandoff(),
                    _pendingForceGrabCommits[handIndex].active,
                    touchGrabPhysicsWritesAllowed);
            }
            if (canTryTouchGrab) {
                _touchGrabRuntime.beginAttemptDiagnostics();
                constexpr std::uint32_t
                    kTouchGrabContactFreshnessFrames = 4;
                const auto contacts =
                    hand.collectFreshSemanticContacts(
                        kTouchGrabContactFreshnessFrames);
                const auto surfaceContacts =
                    _dynamicHandCollision.collectFreshSurfaceContacts(
                        isLeft,
                        kTouchGrabContactFreshnessFrames);
                const auto tryTargetClass =
                    [&](const TouchGrabRuntime::TargetClass
                            targetClass) {
                        const auto tryContacts =
                            [&](const hand_semantic_contact_state::
                                    SemanticContactCollection& candidates,
                                const TouchGrabRuntime::ContactSource source) {
                                for (std::size_t index = 0;
                                     index < candidates.count;
                                     ++index) {
                                    if (_touchGrabRuntime.tryAcquire(
                                            isLeft,
                                            candidates.records[index],
                                            frame.bhkWorld,
                                            frame.hknpWorld,
                                            worldGeneration,
                                            skeletonGeneration,
                                            providerGeneration,
                                            collisionGeneration,
                                            targetClass,
                                            source)) {
                                        return true;
                                    }
                                }
                                return false;
                            };
                        if (tryContacts(
                                contacts,
                                TouchGrabRuntime::ContactSource::
                                    SemanticHand)) {
                            return true;
                        }
                        return tryContacts(
                            surfaceContacts,
                            TouchGrabRuntime::ContactSource::
                                DynamicSurface);
                    };
                const bool touchGrabAcquired =
                    tryTargetClass(
                        TouchGrabRuntime::TargetClass::
                            Explicit) ||
                    tryTargetClass(
                        TouchGrabRuntime::TargetClass::
                            Wildcard);
                if (touchGrabAcquired) {
                    TouchGrabRuntime::HandReport touchGrabReport{};
                    if (g_rockConfig.rockSurfaceGrabHapticsEnabled &&
                        _touchGrabRuntime.getHandReport(
                            isLeft,
                            touchGrabReport) &&
                        touchGrabReport.kind ==
                            provider::RockProviderTouchGrabKindV1::
                                FixedAnchor) {
                        (void)_feedbackHaptics.queue(
                            isLeft ?
                                feedback_haptics::FeedbackHand::Left :
                                feedback_haptics::FeedbackHand::Right,
                            g_rockConfig.
                                rockSurfaceGrabHapticDurationSeconds,
                            g_rockConfig.
                                rockSurfaceGrabHapticIntensity);
                    }
                    if (hand.hasSelection()) {
                        hand.clearSelectionState(false);
                    }
                    grab_input_intent_policy::reset(
                        inputIntentState);
                    cancelPeerHeldJoinRetry(
                        "touch-grab-acquired",
                        true);
                    clearGameplayCandidatesForHand(
                        hand,
                        isLeft);
                    _twoHandedGrip.cancelHandVisualReturn(
                        isLeft,
                        "touch-grab-acquired");
                    hand.cancelGrabVisualReturn(
                        "touch-grab-acquired");
                    return;
                }
                const auto attempt =
                    _touchGrabRuntime.getAttemptReport();
                ROCK_LOG_INFO(
                    Hand,
                    "Touch grab attempt rejected: hand={} semanticContacts={} surfaceContacts={} failure={} targetClass={} source={} body={} layer={} motionClass={} motionProperties={} motionIndex={} latchFailure={} meshFailure={}",
                    isLeft ? "left" : "right",
                    contacts.count,
                    surfaceContacts.count,
                    static_cast<std::uint32_t>(attempt.failure),
                    static_cast<std::uint32_t>(attempt.targetClass),
                    static_cast<std::uint32_t>(attempt.contactSource),
                    attempt.bodyId,
                    attempt.collisionLayer,
                    static_cast<std::uint32_t>(attempt.motionClass),
                    attempt.motionPropertiesId,
                    attempt.motionIndex,
                    attempt.surfaceLatchFailure,
                    attempt.surfaceMeshFailure);
            }

            if (triggerEquipIntent.pending) {
                triggerEquipIntent.remainingSeconds -= (std::max)(0.0f, frame.deltaSeconds);
                if (triggerEquipIntent.remainingSeconds <= 0.0f) {
                    triggerEquipIntent = {};
                }
            }
            /*
             * Input and grab commit are sampled in the same frame but the
             * held-weapon equip block runs before the selected-object commit.
             * Retain a same-hand trigger edge only for the selected weapon and
             * replay it after that exact ref becomes held. Without this, a
             * quick grip+trigger gesture lost the left trigger edge forever;
             * the right hand appeared more reliable only because its legacy
             * settle auto-equip could mask the loss.
             */
            if (heldWeaponEquipTriggerPressedEdge && !heldWeaponAtFrameStart && rawGrabInput.held && hand.hasSelection()) {
                auto* selectedRef = hand.getSelection().refr;
                auto* selectedBase = selectedRef ? selectedRef->GetObjectReference() : nullptr;
                const auto* selectedWeapon = selectedBase ? selectedBase->As<RE::TESObjectWEAP>() : nullptr;
                const bool throwable = selectedWeapon &&
                    (selectedWeapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                        selectedWeapon->weaponData.type == RE::WEAPON_TYPE::kMine);
                if (selectedWeapon && !throwable) {
                    triggerEquipIntent = HeldWeaponTriggerEquipIntent{
                        .pending = true,
                        .formID = selectedRef->GetFormID(),
                        .remainingSeconds = 0.35f,
                    };
                }
            }
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
                        handInput.closeSelectionDirectionWorld,
                        g_rockConfig.rockPullCatchWideReacquireRadiusGameUnits,
                        g_rockConfig.rockPullCatchWideReacquireMaxBodyDistanceGameUnits)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand restored stale pull catch commit with target-specific wide close reacquire",
                        hand.handName());
                } else {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand cancelled stale pull catch commit because selected close ref/body no longer matches the pull owner",
                        hand.handName());
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-stale-reacquire-failed");
                    hand.clearSelectionState(true);
                    releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                    return;
                }
            }

            const Hand& peerForInputIntent = isLeft ? _rightHand : _leftHand;
            auto* peerHeldRefForInput = peerForInputIntent.getHeldRef();
            const auto& peerSavedObjectStateForInput = peerForInputIntent.getSavedObjectState();
            const bool peerHoldingLooseObject =
                !hand.isHolding() &&
                peerForInputIntent.isHolding() &&
                peerHeldRefForInput &&
                peerSavedObjectStateForInput.isValid() &&
                peerSavedObjectStateForInput.targetKind == grab_target::Kind::LooseObject;
            const bool peerHeldCloseSelectionReady =
                peerHoldingLooseObject &&
                hand.hasSelection() &&
                !hand.getSelection().isFarSelection &&
                hand.getSelection().refr == peerHeldRefForInput;
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
                selectedPressCandidate || pullCatchPressCandidate || peerHeldCloseSelectionReady,
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

            auto attemptPeerHeldCloseJoinSelection = [&](const char** outRefusalReason = nullptr) {
                auto setRefusal = [&](const char* reason) {
                    if (outRefusalReason) {
                        *outRefusalReason = reason ? reason : "unknown";
                    }
                    return false;
                };

                if (hand.isHolding()) {
                    return setRefusal("hand-already-holding");
                }

                if (outRefusalReason) {
                    *outRefusalReason = "not-evaluated";
                }

                const Hand& peer = isLeft ? _rightHand : _leftHand;
                if (!peer.isHolding() || !peer.getHeldRef()) {
                    return setRefusal("peer-not-holding");
                }
                if (!peer.getSavedObjectState().isValid() || peer.getSavedObjectState().targetKind != grab_target::Kind::LooseObject) {
                    return setRefusal("peer-target-not-loose-object");
                }

                if (hand.hasSelection() && hand.getSelection().refr != peer.getHeldRef()) {
                    return setRefusal(hand.getSelection().isFarSelection ? "unrelated-far-selection" : "unrelated-close-selection");
                }

                const bool hadPeerHeldCloseSelection =
                    hand.hasSelection() && hand.getSelection().refr == peer.getHeldRef() && !hand.getSelection().isFarSelection;
                const bool refreshedPeerHeldSelection = hand.acquirePeerHeldCloseSelection(frame.bhkWorld,
                    frame.hknpWorld,
                    peer.getSavedObjectState(),
                    peer.getHeldBodyIds(),
                    handInput.grabAnchorWorld,
                    handInput.closeSelectionDirectionWorld,
                    g_rockConfig.rockNearDetectionRange,
                    outRefusalReason);
                if (!refreshedPeerHeldSelection && hadPeerHeldCloseSelection) {
                    hand.clearSelectionState(false);
                }
                return refreshedPeerHeldSelection;
            };

            const bool unrelatedSelectionForPeerJoin =
                peerHoldingLooseObject &&
                hand.hasSelection() &&
                hand.getSelection().refr != peerHeldRefForInput;
            const std::uint32_t peerHeldFormIdForRetry = peerHeldRefForInput ? peerHeldRefForInput->GetFormID() : 0u;
            const bool peerStillHoldingRetryObject =
                peerHoldingLooseObject &&
                (!peerHeldJoinRetryState.active ||
                    (peerHeldJoinRetryState.peerFormId != 0 && peerHeldJoinRetryState.peerFormId == peerHeldFormIdForRetry));
            const char* retryLastRefusalBeforeUpdate =
                peerHeldJoinRetryState.lastRefusalReason ? peerHeldJoinRetryState.lastRefusalReason : "none";
            const auto retryAttemptsBeforeUpdate = peerHeldJoinRetryState.attempts;
            const auto peerHeldRetryDecision = peer_held_join_retry_policy::update(
                peerHeldJoinRetryState,
                peer_held_join_retry_policy::Input{
                    .rawHeld = rawGrabInput.held,
                    .rawPressed = rawGrabInput.pressed,
                    .rawReleased = rawGrabInput.released,
                    .normalGrabSuppressed = false,
                    .handHolding = hand.isHolding(),
                    .peerHoldingLooseObject = peerHoldingLooseObject,
                    .peerStillHoldingSameObject = peerStillHoldingRetryObject,
                    .unrelatedSelection = unrelatedSelectionForPeerJoin,
                    .grabSucceeded = false,
                    .peerFormId = peerHeldFormIdForRetry,
                    .deltaSeconds = frame.deltaSeconds,
                    .config = peer_held_join_retry_policy::Config{
                        .enabled = g_rockConfig.rockGrabInputIntentStateEnabled,
                        .leewaySeconds = g_rockConfig.rockGrabInputLeewaySeconds,
                        .forceSeconds = g_rockConfig.rockGrabInputForceSeconds,
                    },
                });
            if (peerHeldRetryDecision.started) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand peer-held join retry started: peerFormID={:08X} window={:.3f}s interval={:.3f}s",
                    hand.handName(),
                    peerHeldJoinRetryState.peerFormId,
                    peerHeldJoinRetryState.windowSeconds,
                    peer_held_join_retry_policy::retryIntervalSeconds(peer_held_join_retry_policy::Config{
                        .enabled = g_rockConfig.rockGrabInputIntentStateEnabled,
                        .leewaySeconds = g_rockConfig.rockGrabInputLeewaySeconds,
                        .forceSeconds = g_rockConfig.rockGrabInputForceSeconds,
                    }));
            }
            if (peerHeldRetryDecision.cancelled) {
                grab_input_intent_policy::reset(inputIntentState);
                if (!peerHeldRetryDecision.success &&
                    hand.hasSelection() &&
                    !hand.getSelection().isFarSelection &&
                    hand.getSelection().refr == peerHeldRefForInput) {
                    grabInput.pressed = false;
                    grabInput.syntheticPressed = false;
                }
                if (peerHeldRetryDecision.success) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand peer-held join retry succeeded: reason={} peerFormID={:08X} attempts={}",
                        hand.handName(),
                        peerHeldRetryDecision.reason,
                        peerHeldFormIdForRetry,
                        retryAttemptsBeforeUpdate);
                } else {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand peer-held join retry cancelled: reason={} peerFormID={:08X} attempts={} lastRefusal={}",
                        hand.handName(),
                        peerHeldRetryDecision.reason,
                        peerHeldFormIdForRetry,
                        retryAttemptsBeforeUpdate,
                        retryLastRefusalBeforeUpdate);
                }
            }

            bool peerHeldRetryAttemptDue = peerHeldRetryDecision.attempt && peerHeldJoinRetryState.active;
            bool peerHeldRetryRefreshedSelection = false;
            if (peerHeldRetryAttemptDue) {
                const char* refusalReason = "not-attempted";
                peerHeldRetryRefreshedSelection = attemptPeerHeldCloseJoinSelection(&refusalReason);
                if (peerHeldJoinRetryState.active) {
                    peerHeldJoinRetryState.lastRefusalReason = peerHeldRetryRefreshedSelection ? "selection-acquired" : (refusalReason ? refusalReason : "unknown");
                }
                if (peerHeldRetryRefreshedSelection) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand peer-held join retry refreshed close selection: peerFormID={:08X} attempt={} source={}",
                        hand.handName(),
                        peerHeldJoinRetryState.peerFormId,
                        peerHeldJoinRetryState.attempts,
                        refusalReason ? refusalReason : "unknown");
                }
            }

            const bool peerHeldRetryCommitIntent =
                peerHeldRetryAttemptDue &&
                peerHeldJoinRetryState.active &&
                rawGrabInput.held &&
                !hand.isHolding() &&
                hand.hasSelection() &&
                !hand.getSelection().isFarSelection &&
                hand.getSelection().refr == peerHeldRefForInput;

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
            auto attemptSelectedGrab = [&]() {
                const auto& transform = handInput.rawHandWorld;

                const auto sharedContext = makeGrabSharedObjectContext(hand, isLeft);
                const bool grabbedFromPullCatchCommit = hand.hasPendingPullCatchCommit();
                prepareDynamicWorldCarCollisionForGrab(frame.bhkWorld, hknp, hand.getSelection().refr);
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
            auto dispatchMouthConsumeEvent = [&](GrabEventType type,
                                                 RE::TESObjectREFR* refr,
                                                 std::uint32_t formID,
                                                 std::uint32_t primaryBodyId,
                                                 const mouth_consume::Decision& decision) {
                GrabEventData eventData{};
                eventData.type = type;
                eventData.sourceKind = GrabEventSourceKind::HeldObject;
                eventData.isLeft = isLeft;
                eventData.refr = refr;
                eventData.formID = formID != 0 ? formID : (refr ? refr->GetFormID() : 0);
                eventData.primaryBodyId = primaryBodyId;
                eventData.positionGame[0] = decision.mouthCenterGame.x;
                eventData.positionGame[1] = decision.mouthCenterGame.y;
                eventData.positionGame[2] = decision.mouthCenterGame.z;
                eventData.flags |= ROCK_GRAB_EVENT_FLAG_POSITION_VALID;
                if (std::isfinite(decision.speedGameUnitsPerSecond)) {
                    eventData.speedGameUnitsPerSecond = decision.speedGameUnitsPerSecond;
                    eventData.flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
                }
                eventData.intensityHint = std::clamp(std::isfinite(decision.confidence) ? decision.confidence : 0.0f, 0.0f, 1.0f);
                eventData.flags |= ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID;
                dispatchGrabEvent(eventData);
            };

            /*
             * Native idle-grip harvesting is acquisition preparation, not a
             * hover-haptic side effect. Offer the retained held loose weapon or
             * open-hand selection before grab input is committed below. The
             * retained reference crosses native asynchronous progress safely;
             * the visual equip bridge alone owns only a scene model. A null
             * candidate still advances an in-flight load, so pull travel can
             * hide the load without blocking the frame thread.
             */
            RE::NiPointer<RE::TESObjectREFR> nativeIdleGripCandidate{};
            if (hand.isHoldingLooseWeapon()) {
                nativeIdleGripCandidate = hand.getSavedObjectState().retainedRef;
            } else if (!hand.isHolding() && hand.hasSelection() && !input_remap_runtime::isMenuInputActive()) {
                nativeIdleGripCandidate = hand.getSelection().retainedRef;
            }
            native_idle_grip_preharvest::observeCandidate(std::move(nativeIdleGripCandidate));

            loose_weapon_grip_zone::updateHeldLooseWeapon(
                isLeft,
                gripZoneSettleEquipEnabled && hand.isHoldingLooseWeapon(),
                hand.getHeldRef(),
                hand.getState() == HandState::HeldBody,
                frame.deltaSeconds,
                _equippedWeaponHandlingSettings.gripZoneEquipRadiusGameUnits);

            /*
             * Grip-zone hover probe: while either OPEN hand's selection
             * candidate is a loose weapon, feel out whether grabbing right now
             * would land the palm inside the firing-grip zone (and therefore
             * equip after the settle into that same physical hand while the
             * addon authority is active). Both hands
             * use the same projected FRIK firing-grip radius; grenades never
             * reach the equip path so they never hum. Vibration stops on grab
             * because the hover candidate goes null while holding.
             */
            RE::TESObjectREFR* gripZoneHoverCandidate = nullptr;
            if (_equippedWeaponHandlingSettings.gripZoneHoverHapticsEnabled &&
                gripZoneSettleEquipEnabled &&
                g_rockConfig.rockInputRemapEnabled &&
                !hand.isHolding() &&
                hand.hasSelection() &&
                !input_remap_runtime::isMenuInputActive()) {
                auto* selectionRef = hand.getSelection().refr;
                if (selectionRef && !loose_grenade_runtime::isGrenadeRef(selectionRef)) {
                    gripZoneHoverCandidate = selectionRef;
                }
            }
            loose_weapon_grip_zone::updateHoverCandidateWeapon(
                isLeft,
                gripZoneHoverCandidate,
                _equippedWeaponHandlingSettings.gripZoneEquipRadiusGameUnits);
            if (loose_weapon_grip_zone::isGripZoneHoverInsideRadius(isLeft)) {
                (void)_feedbackHaptics.queue(
                    isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                    _equippedWeaponHandlingSettings.gripZoneHoverHapticIntensity);
            }

            if (hand.isHolding()) {
                const Hand& peer = isLeft ? _rightHand : _leftHand;
                auto* heldRefForGameplay = hand.getHeldRef();
                const bool heldLooseGrenade = loose_grenade_runtime::isGrenadeRef(heldRefForGameplay);
                const bool peerHoldingSameObject =
                    heldRefForGameplay && peer.isHolding() && peer.getHeldRef() == heldRefForGameplay;
                const bool replayedSameHandTrigger = triggerEquipIntent.pending &&
                    heldRefForGameplay && triggerEquipIntent.formID == heldRefForGameplay->GetFormID();
                const bool heldWeaponEquipTriggerPressed = heldWeaponEquipTriggerPressedEdge || replayedSameHandTrigger;
                if (replayedSameHandTrigger || (heldWeaponEquipTriggerPressedEdge && hand.isHoldingLooseWeapon())) {
                    triggerEquipIntent = {};
                }
                const bool heldWeaponGripZoneEquipSettled = !heldLooseGrenade &&
                    loose_weapon_grip_zone::isGripZoneEquipSettled(
                        isLeft,
                        _equippedWeaponHandlingSettings.gripZoneEquipSettleSeconds);
                const bool heldWeaponEquipRequested = input_remap_policy::shouldRequestHeldWeaponEquip(input_remap_policy::HeldWeaponEquipInput{
                    .remapEnabled = g_rockConfig.rockInputRemapEnabled,
                    .gameplayInputAllowed = true,
                    .menuInputActive = input_remap_runtime::isMenuInputActive(),
                    .heldWeaponAtFrameStart = heldWeaponAtFrameStart,
                    .heldWeaponNow = hand.isHoldingLooseWeapon(),
                    .heldWeaponHand = isLeft ? input_remap_policy::Hand::Left : input_remap_policy::Hand::Right,
                    .triggerInputHand = isLeft ? input_remap_policy::Hand::Left : input_remap_policy::Hand::Right,
                    .triggerPressedEdge = heldWeaponEquipTriggerPressed,
                    .gripZoneEquipEnabled = gripZoneSettleEquipEnabled,
                    .gripZoneEquipSettled = heldWeaponGripZoneEquipSettled,
                });

                auto equipHeldWeaponFromHand = [&](const bool triggeredByInput, const char* requestReason, const char* logAction) {
                    const auto transitionReason = triggeredByInput ?
                        held_weapon_instant_transition::RequestReason::SameHandTrigger :
                        held_weapon_instant_transition::RequestReason::GripZoneSettle;
                    if (peerHoldingSameObject) {
                        ROCK_LOG_WARN(Hand,
                            "{} hand {} held weapon equip blocked: peer hand still holding formID={:08X}",
                            hand.handName(),
                            logAction ? logAction : "requested",
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0u);
                        return true;
                    }

                    auto* player = RE::PlayerCharacter::GetSingleton();
                    const std::uint32_t nativeStateBeforeEquip =
                        f4vr::getNativeWeaponState(player);
                    if (!held_weapon_equip_state_policy::canBeginEquip(nativeStateBeforeEquip)) {
                        const bool triggerIntentRearmed =
                            held_weapon_equip_state_policy::shouldRearmTrigger(nativeStateBeforeEquip, triggeredByInput) &&
                            heldRefForGameplay;
                        if (triggerIntentRearmed) {
                            triggerEquipIntent = HeldWeaponTriggerEquipIntent{
                                .pending = true,
                                .formID = heldRefForGameplay->GetFormID(),
                                .remainingSeconds = 0.35f,
                            };
                        }
                        ROCK_LOG_SAMPLE_WARN(
                            Hand,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "{} hand {} held weapon equip deferred/blocked before release formID={:08X} weaponState={}({}) triggerIntentRearmed={}",
                            hand.handName(),
                            logAction ? logAction : "requested",
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0u,
                            nativeStateBeforeEquip,
                            held_weapon_equip_state_policy::nativeWeaponStateName(nativeStateBeforeEquip),
                            triggerIntentRearmed ? "yes" : "no");
                        return true;
                    }

                    const auto instantReadiness =
                        held_weapon_instant_transition::readinessFor(player);
                    if (!instantReadiness.ready) {
                        ROCK_LOG_SAMPLE_WARN(
                            Hand,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "{} hand {} held weapon equip blocked before release formID={:08X} instantTransition={}",
                            hand.handName(),
                            logAction ? logAction : "requested",
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0u,
                            held_weapon_instant_transition::readinessReasonName(
                                instantReadiness.reason));
                        return true;
                    }

                    PendingEquippedWeaponPrimaryOnlyGripStart pendingGripStart{};
                    pendingGripStart.pending = equipped_weapon_manual_ownership_policy::shouldStartHeldWeaponEquipOwnership(
                        equipped_weapon_manual_ownership_policy::HeldWeaponEquipOwnershipInput{
                            .modes = firingGripModes,
                            .handIsLeft = isLeft,
                            .gripHeld = rawGrabInput.held,
                        });
                    pendingGripStart.isLeft = isLeft;
                    const bool handCarryAvailable = !isLeft || ambidextrousHandoffAvailable;
                    const bool capturedLooseHold = pendingGripStart.pending &&
                        handCarryAvailable &&
                        loose_weapon_grip_zone::tryGetFiringHandWeaponLocal(
                            isLeft,
                            pendingGripStart.firingHandWeaponLocal,
                            pendingGripStart.firingGripWeaponLocal);
                    pendingGripStart.hasFiringHandWeaponLocal = capturedLooseHold;
                    pendingGripStart.hasFiringGripWeaponLocal = capturedLooseHold;
                    if (pendingGripStart.pending && isLeft && !capturedLooseHold) {
                        ROCK_LOG_SAMPLE_WARN(
                            Hand,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "left hand {} held weapon equip blocked: canonical weapon-relative left carry unavailable addonAuthority={} ambidextrousFiring={} grabHeld={} hFRIKBlockers={} gripFrame={}",
                            logAction ? logAction : "requested",
                            _equippedWeaponHandlingSettings.externalAuthorityActive ? "yes" : "no",
                            ambidextrousHandoffAvailable ? "yes" : "no",
                            rawGrabInput.held ? "yes" : "no",
                            handCarryAvailable ? "yes" : "no",
                            pendingGripStart.hasFiringHandWeaponLocal ? "yes" : "no");
                        return true;
                    }

                    hand.captureHeldReleaseMotion(hknp, handInput.rawHandWorld, frame.deltaSeconds);
                    auto* heldRef = hand.getHeldRef();
                    const auto previousEquippedWeaponFormID =
                        currentEquippedWeaponFormId();
                    const auto previousNativeInstanceNode =
                        previousEquippedWeaponFormID != 0 ?
                        equipped_weapon_visual_state::observe(
                            previousEquippedWeaponFormID).exactInstance :
                        nullptr;
                    hand.stopSelectionHighlight();
                    Hand& peerHandForVisualState = isLeft ? _rightHand : _leftHand;
                    if (heldRef && peerHandForVisualState.hasSelection() && peerHandForVisualState.getSelection().refr == heldRef) {
                        peerHandForVisualState.clearSelectionState(false);
                    }
                    std::uint32_t heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                    const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                    auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                    releaseContext.disposition = GrabReleaseDisposition::PendingInventoryTransfer;
                    releaseContext.reason = requestReason ? requestReason : "held-weapon-equip";
                    auto releaseOutcome = hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                    if (heldRef) {
                        releaseObject(heldRef, claimOwnerForHand(isLeft));
                    }

                    const auto equipResult = weapon_equip_transfer::transferHeldWeaponToPlayerAndEquip(weapon_equip_transfer::EquipInput{
                        .heldRef = releaseOutcome.takeRetainedReference(),
                        .transitionReason = transitionReason,
                    });
                    const std::uint32_t nativeStateAfterEquip =
                        f4vr::getNativeWeaponState(player);
                    const auto immediateVisual = equipResult.committed && equipResult.weapon ?
                        equipped_weapon_visual_state::observe(
                            equipResult.weapon->formID,
                            reinterpret_cast<std::uintptr_t>(previousNativeInstanceNode)) :
                        equipped_weapon_visual_state::Snapshot{};
                    bool equipBridgeStarted = false;
                    if (equipResult.success) {
                        const auto transitionSource = triggeredByInput ?
                            EquippedWeaponTransitionCoordinator::Source::HeldTriggerEquip :
                            EquippedWeaponTransitionCoordinator::Source::HeldGripZoneEquip;
                        equipBridgeStarted = _equippedWeaponTransition.beginHeldTransition(
                            EquippedWeaponTransitionCoordinator::ExpectedIdentity{
                                .formID = equipResult.weapon ? equipResult.weapon->formID : equipResult.observedEquippedFormID,
                                .instanceData = equipResult.requestedInstanceData,
                                .previousFormID = equipResult.previousEquippedFormID,
                                .previousInstanceData = equipResult.previousEquippedInstanceData,
                                .previousNativeInstanceNode =
                                    reinterpret_cast<std::uintptr_t>(
                                        previousNativeInstanceNode),
                            },
                            transitionSource,
                            EquipVisualBridge::BeginInput{
                            .worldModel = equipResult.detachedWorldModel,
                            .weaponFormID = equipResult.weapon ? equipResult.weapon->formID : equipResult.observedEquippedFormID,
                            .isLeftHand = isLeft,
                            .weapon = equipResult.weapon,
                            .hasFiringHandWeaponLocal = pendingGripStart.hasFiringHandWeaponLocal,
                            .firingHandWeaponLocal = pendingGripStart.firingHandWeaponLocal,
                            .sourceSchedulerSequence =
                                _currentPreFrikSchedulerSequence,
                            .timeoutSeconds = _equippedWeaponHandlingSettings.equipVisualBridgeTimeoutSeconds,
                            .blendSeconds = _equippedWeaponHandlingSettings.equipVisualBridgeBlendSeconds,
                        });
                    }
                    if (heldFormID == 0 && equipResult.formID != 0) {
                        heldFormID = equipResult.formID;
                    }

                    auto* postEquipRef = equipResult.untransferredRef.get();
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, postEquipRef, heldFormID, 0);
                    if (!equipResult.success && !equipResult.transferredToInventory) {
                        hand.applyReleaseVelocitySnapshot(hknp, releaseOutcome.velocity);
                    }
                    dispatchHeldObjectEventByFormID(GrabEventType::Released, postEquipRef, heldFormID, primaryBodyId);
                    const bool physicalCarryPendingArmed =
                        equipResult.success && pendingGripStart.pending;
                    if (physicalCarryPendingArmed) {
                        pendingGripStart.targetWeaponFormID = equipResult.weapon ?
                            equipResult.weapon->formID :
                            equipResult.observedEquippedFormID;
                        pendingGripStart.targetWeaponInstanceData =
                            equipResult.requestedInstanceData;
                        pendingGripStart.previousWeaponFormID =
                            equipResult.previousEquippedFormID;
                        pendingGripStart.previousWeaponInstanceData =
                            equipResult.previousEquippedInstanceData;
                        pendingGripStart.remainingSeconds = 10.0f;
                        _pendingEquippedWeaponPrimaryOnlyGripStart = pendingGripStart;
                    }
                    const auto& actionTrace = equipResult.instantTransition.actionTrace;
                    ROCK_LOG_INFO(Hand,
                        "{} hand {} held weapon equip formID={:08X} success={} managerAccepted={} committed={} equippedStackMatch={} equipReason={} requestReason={} transition={} readiness={} count={} stack={} stackEvidence={} stacks={}->{} mutations={} instanceMatch={} requestedInstance={:#x} observedInstance={:#x} transferred={} observedEquipped={:08X} equipIndex={} weaponState={}({})->{}({}) traceCount={} traceSheathe={} traceDraw={} traceFaults=0x{:02X} nativeInstance={} nativeAncestorsVisible={} nativeLocalVisible={} immediateEquip={} visualBridge={} physicalCarryPending={} physicalCarryHand={}",
                        hand.handName(),
                        logAction ? logAction : "requested",
                        heldFormID,
                        equipResult.success ? "yes" : "no",
                        equipResult.instantTransition.managerAccepted ? "yes" : "no",
                        equipResult.committed ? "yes" : "no",
                        equipResult.matchedEquippedStack ? "yes" : "no",
                        weapon_equip_transfer::equipReasonName(equipResult.reason),
                        held_weapon_instant_transition::requestReasonName(
                            transitionReason),
                        held_weapon_instant_transition::immediateEquipCodeName(
                            equipResult.instantTransition.code),
                        held_weapon_instant_transition::readinessReasonName(
                            instantReadiness.reason),
                        equipResult.count,
                        equipResult.stackID,
                        weapon_inventory_stack_selection_policy::evidenceName(
                            equipResult.stackSelectionEvidence),
                        equipResult.preTransferStackCount,
                        equipResult.postTransferStackCount,
                        equipResult.stackMutationCandidateCount,
                        equipResult.matchedInstanceData ? "yes" : "no",
                        equipResult.requestedInstanceData,
                        equipResult.observedEquippedInstanceData,
                        equipResult.transferredToInventory ? "yes" : "no",
                        equipResult.observedEquippedFormID,
                        equipResult.observedEquipIndex,
                        nativeStateBeforeEquip,
                        held_weapon_equip_state_policy::nativeWeaponStateName(nativeStateBeforeEquip),
                        nativeStateAfterEquip,
                        held_weapon_equip_state_policy::nativeWeaponStateName(nativeStateAfterEquip),
                        actionTrace.count,
                        held_weapon_instant_transition_policy::actionCount(
                            actionTrace,
                            held_weapon_instant_transition_policy::NativeAction::Sheathe),
                        held_weapon_instant_transition_policy::actionCount(
                            actionTrace,
                            held_weapon_instant_transition_policy::NativeAction::Draw),
                        static_cast<unsigned int>(
                            (actionTrace.playerMismatch ? 0x01u : 0u) |
                            (actionTrace.unexpectedCaller ? 0x02u : 0u) |
                            (actionTrace.nestedScope ? 0x04u : 0u) |
                            (actionTrace.overflow ? 0x08u : 0u)),
                        immediateVisual.exactInstance ? "yes" : "no",
                        immediateVisual.ancestorPathVisible ? "yes" : "no",
                        immediateVisual.instanceLocallyVisible ? "yes" : "no",
                        equipResult.usedImmediateEquip ? "yes" : "no",
                        equipBridgeStarted ? "yes" : "no",
                        physicalCarryPendingArmed ? "yes" : "no",
                        physicalCarryPendingArmed ? (isLeft ? "left" : "right") : "none");
                    input_remap_runtime::setHandHeldWeapon(isLeft, false);
                    clearGameplayCandidatesForHand(hand, isLeft);
                    return true;
                };

                if (heldLooseGrenade) {
                    if (heldWeaponEquipTriggerPressed) {
                        static_cast<void>(armHeldLooseGrenade(hand, frame));
                    }
                } else if (heldWeaponEquipRequested) {
                    const bool triggeredByInput = heldWeaponEquipTriggerPressed;
                    const char* requestReason = triggeredByInput ? "same-hand-trigger-held-weapon-equip" :
                                                                   "grip-zone-held-weapon-equip";
                    const char* logAction = triggeredByInput ? "trigger" : "grip-zone";
                    if (equipHeldWeaponFromHand(triggeredByInput, requestReason, logAction)) {
                        return;
                    }
                }

                const auto consumeEligibility = mouth_consume::evaluateEligibility(mouth_consume::EligibilityInput{
                    .enabled = g_rockConfig.rockMouthConsumeEnabled,
                    .allowPoison = g_rockConfig.rockMouthConsumeAllowPoison,
                    .peerHoldingSameObject = peerHoldingSameObject,
                    .heldRef = heldRefForGameplay,
                    .savedState = &hand.getSavedObjectState(),
                });

                mouth_consume::Decision consumeDecision{};
                if (consumeEligibility.eligible) {
                    consumeDecision = mouth_consume::evaluate(mouth_consume::DetectorInput{
                            .hasHmdFrame = frame.hasHmdFrame,
                            .hmdPositionWorld = frame.hmdPositionWorld,
                            .hmdForwardWorld = frame.hmdForwardWorld,
                            .objectProbe = makeMouthConsumeObjectProbe(hknp, hand, handInput),
                            .hasObjectProbe = true,
                            .handProbe = makeMouthConsumeHandProbe(handInput),
                            .hasHandProbe = true,
                            .deltaSeconds = frame.deltaSeconds,
                            .config = makeMouthConsumeDetectorConfig(),
                        },
                        mouthConsumeState);
                } else {
                    clearMouthConsumeForHand(hand, isLeft);
                }

                bool consumeCandidateActive = false;
                if (consumeEligibility.eligible && consumeDecision.candidate) {
                    if (hand.getState() == HandState::StashCandidate) {
                        hand.cancelStashCandidate();
                    }
                    if (hand.getState() == HandState::HeldBody) {
                        hand.beginConsumeCandidate();
                    }
                    if (hand.getState() == HandState::ConsumeCandidate) {
                        consumeCandidateActive = true;
                        const bool pulseDue = _dynamicPushElapsedSeconds >= mouthConsumeState.nextCandidatePulseTimeSeconds;
                        if (consumeDecision.enteredCandidate || consumeDecision.changedCandidate || pulseDue) {
                            dispatchMouthConsumeEvent(
                                GrabEventType::ConsumeCandidate,
                                heldRefForGameplay,
                                heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0,
                                hand.getSavedObjectState().bodyId.value,
                                consumeDecision);
                            mouthConsumeState.nextCandidatePulseTimeSeconds =
                                _dynamicPushElapsedSeconds + (std::max)(0.02f, g_rockConfig.rockMouthConsumeCandidateHapticIntervalSeconds);
                        }
                    }
                } else {
                    hand.cancelConsumeCandidate();
                }

                const auto stashEligibility = !consumeCandidateActive ?
                    shoulder_stash::evaluateEligibility(shoulder_stash::EligibilityInput{
                        .enabled = g_rockConfig.rockShoulderStashEnabled,
                        .peerHoldingSameObject = peerHoldingSameObject,
                        .heldRef = heldRefForGameplay,
                        .savedState = &hand.getSavedObjectState(),
                    }) :
                    shoulder_stash::EligibilityResult{ .eligible = false, .reason = shoulder_stash::EligibilityReason::SharedHeldObject };

                shoulder_stash::Decision stashDecision{};
                if (!consumeCandidateActive && stashEligibility.eligible) {
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

                if (!consumeCandidateActive && stashEligibility.eligible && stashDecision.candidate) {
                    if (hand.getState() == HandState::HeldBody) {
                        hand.beginStashCandidate();
                    }
                    if (hand.getState() == HandState::StashCandidate) {
                        const bool pulseDue = _dynamicPushElapsedSeconds >= shoulderStashState.nextCandidatePulseTimeSeconds;
                        if (stashDecision.enteredCandidate || stashDecision.changedCandidate || pulseDue) {
                            dispatchShoulderStashEvent(
                                GrabEventType::StashCandidate,
                                heldRefForGameplay,
                                heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0,
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
                    hand.captureHeldReleaseMotion(hknp, handInput.rawHandWorld, frame.deltaSeconds);
                    auto* heldRef = hand.getHeldRef();
                    std::uint32_t heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                    if (consumeEligibility.eligible && consumeDecision.confirmedForCommit && hand.getState() == HandState::ConsumeCandidate) {
                        /*
                         * Mouth consume mirrors shoulder stash's two-phase release:
                         * detach the grab without throw velocity first, then let the
                         * native consume/activation path take ownership. Only failures
                         * that leave a world ref behind get the captured throw velocity.
                         */
                        auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                        releaseContext.disposition = GrabReleaseDisposition::PendingConsumeTransfer;
                        releaseContext.reason = "mouth-consume-pending-transfer";
                        const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                        auto releaseOutcome = hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                        if (heldRef) {
                            releaseObject(heldRef, claimOwnerForHand(isLeft));
                        }
                        const auto consumeResult = mouth_consume::transferToPlayerConsume(mouth_consume::ConsumeInput{
                            .heldRef = releaseOutcome.takeRetainedReference(),
                            .allowPoison = g_rockConfig.rockMouthConsumeAllowPoison,
                        });
                        if (heldFormID == 0 && consumeResult.formID != 0) {
                            heldFormID = consumeResult.formID;
                        }

                        const bool failedBeforeOwnershipTransfer =
                            !consumeResult.attempted || consumeResult.reason == mouth_consume::ConsumeReason::ActivateRefFailed;
                        auto* postConsumeRef = consumeResult.untransferredRef.get();
                        dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, postConsumeRef, heldFormID, 0);
                        if (consumeResult.success) {
                            dispatchMouthConsumeEvent(GrabEventType::Consumed, nullptr, heldFormID, primaryBodyId, consumeDecision);
                        } else {
                            if (failedBeforeOwnershipTransfer) {
                                hand.applyReleaseVelocitySnapshot(hknp, releaseOutcome.velocity);
                            }
                            dispatchHeldObjectEventByFormID(GrabEventType::Released, postConsumeRef, heldFormID, primaryBodyId);
                        }
                        ROCK_LOG_INFO(Hand,
                            "{} hand mouth consume release formID={:08X} success={} consumeReason={} count={} confidence={:.2f} distance={:.1f} speed={:.1f}",
                            hand.handName(),
                            heldFormID,
                            consumeResult.success ? "yes" : "no",
                            mouth_consume::consumeReasonName(consumeResult.reason),
                            consumeResult.count,
                            consumeDecision.confidence,
                            consumeDecision.distanceGameUnits,
                            consumeDecision.speedGameUnitsPerSecond);
                        clearGameplayCandidatesForHand(hand, isLeft);
                        return;
                    }
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
                        auto releaseOutcome = hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                        if (heldRef) {
                            releaseObject(heldRef, claimOwnerForHand(isLeft));
                        }
                        const auto transferResult = shoulder_stash::transferToPlayerInventory(shoulder_stash::TransferInput{
                            .heldRef = releaseOutcome.takeRetainedReference(),
                        });
                        if (heldFormID == 0 && transferResult.formID != 0) {
                            heldFormID = transferResult.formID;
                        }

                        auto* postTransferRef = transferResult.untransferredRef.get();
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
                        clearGameplayCandidatesForHand(hand, isLeft);
                        return;
                    }
                    hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(hand, isLeft));
                    if (heldRef)
                        releaseObject(heldRef, claimOwnerForHand(isLeft));
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                    dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                    clearGameplayCandidatesForHand(hand, isLeft);
                } else {
                    const auto& transform = handInput.rawHandWorld;
                    auto* heldRef = hand.getHeldRef();
                    auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                    logPalmClockSampleForHand("game-before-held-update",
                        hand,
                        hknp,
                        &transform,
                        _palmClockGameFrameIndex.load(std::memory_order_acquire),
                        _palmClockGameDeltaSeconds.load(std::memory_order_acquire),
                        nullptr);
                    hand.updateHeldObject(hknp,
                        transform,
                        frame.deltaSeconds,
                        g_rockConfig.rockGrabForceFadeInTime,
                        g_rockConfig.rockGrabTauMin,
                        &_bodyBoneColliders,
                        _currentPreFrikSchedulerSequence,
                        makeGrabReleaseContext(hand, isLeft));
                    if (heldRef && !hand.isHolding()) {
                        releaseObject(heldRef, claimOwnerForHand(isLeft));
                        dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                        dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                        clearGameplayCandidatesForHand(hand, isLeft);
                    }
                }
            } else {
                clearGameplayCandidatesForHand(hand, isLeft);
                if (grabInput.pressed && !peerHeldRetryAttemptDue) {
                    const char* refusalReason = "not-attempted";
                    (void)attemptPeerHeldCloseJoinSelection(&refusalReason);
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
                        hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-release");
                        hand.clearSelectionState(true);
                        releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                        return;
                    }
                    if (!hand.advancePullCatchCommit(frame.deltaSeconds, g_rockConfig.rockPullCatchRetryMaxTimeSeconds)) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand cancelled pull catch commit because retry window expired ({:.3f}s)",
                            hand.handName(),
                            g_rockConfig.rockPullCatchRetryMaxTimeSeconds);
                        hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-retry-expired");
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

                if (grabInput.pressed || peerHeldRetryCommitIntent || (pullCatchCommitPending && grabInput.held) || (actorEquipmentDropHandoffReady && grabInput.held)) {
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
                            hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-blocked");
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
                            hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-far-selection");
                            hand.clearSelectionState(true);
                            releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                            return;
                        }
                        auto* selectedRef = hand.getSelection().refr;
                        const auto selectedBodyId = hand.getSelection().bodyId.value;
                        const auto& transform = handInput.rawHandWorld;
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
                        dispatchSimpleGrabEvent(GrabEventType::PullCatchAttempt, isLeft, pullCatchRef, hand.getSelection().bodyId.value);
                    }
                    const bool peerHeldRetryWasActiveForCommit =
                        peerHeldJoinRetryState.active &&
                        rawGrabInput.held &&
                        !hand.isHolding() &&
                        hand.hasSelection() &&
                        !hand.getSelection().isFarSelection &&
                        hand.getSelection().refr == peerHeldRefForInput;
                    const bool grabbed = attemptSelectedGrab();
                    if (grabbed && peerHeldRetryWasActiveForCommit) {
                        const auto peerFormId = peerHeldJoinRetryState.peerFormId;
                        const auto attempts = peerHeldJoinRetryState.attempts;
                        peer_held_join_retry_policy::reset(peerHeldJoinRetryState);
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand peer-held join retry succeeded: peerFormID={:08X} attempts={}",
                            hand.handName(),
                            peerFormId,
                            attempts);
                    } else if (!grabbed && peerHeldRetryWasActiveForCommit && peerHeldJoinRetryState.active) {
                        peerHeldJoinRetryState.lastRefusalReason = "grab-commit-refused";
                        ROCK_LOG_SAMPLE_DEBUG(Hand,
                            g_rockConfig.rockLogSampleMilliseconds,
                            "{} hand retaining peer-held join retry after grab commit refused; grip still held",
                            hand.handName());
                    }
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
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-release");
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
                        hand.finishPullPrepAsPhysicalDropIfActive("pull-arrived-blocked");
                        hand.clearSelectionState(true);
                        releaseObject(pulledRef, claimOwnerForHand(isLeft));
                        return;
                    }

                    dispatchSimpleGrabEvent(GrabEventType::PullCatchAttempt, isLeft, pulledRef, hand.getSelection().bodyId.value);
                    const bool grabbed = attemptSelectedGrab();
                    if (!grabbed && (!hand.hasSelection() || !hand.hasPendingPullCatchCommit())) {
                        hand.finishPullPrepAsPhysicalDropIfActive("pull-grab-refused");
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
        publishHandInputOwnership(_rightHand, false);
        processHand(_leftHand, true);
        publishHandInputOwnership(_leftHand, true);

        if (_rightHand.isHolding() ||
            _touchGrabRuntime.isHandActive(false)) {
            _twoHandedGrip.cancelHandVisualReturn(
                false,
                _rightHand.isHolding() ?
                    "generic-grab-acquired" :
                    "touch-grab-active");
        }
        if (frame.right.disabled ||
            _touchGrabRuntime.isHandActive(false)) {
            _rightHand.cancelGrabVisualReturn(
                frame.right.disabled ?
                    "hand-disabled" :
                    "touch-grab-active");
        } else {
            _rightHand.updateGrabVisualReturn(
                frame.right.rawHandWorld,
                frame.deltaSeconds,
                _currentPreFrikSchedulerSequence);
        }
        if (_leftHand.isHolding() ||
            _touchGrabRuntime.isHandActive(true)) {
            _twoHandedGrip.cancelHandVisualReturn(
                true,
                _leftHand.isHolding() ?
                    "generic-grab-acquired" :
                    "touch-grab-active");
        }
        if (frame.left.disabled ||
            _touchGrabRuntime.isHandActive(true)) {
            _leftHand.cancelGrabVisualReturn(
                frame.left.disabled ?
                    "hand-disabled" :
                    "touch-grab-active");
        } else {
            _leftHand.updateGrabVisualReturn(
                frame.left.rawHandWorld,
                frame.deltaSeconds,
                _currentPreFrikSchedulerSequence);
        }
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
