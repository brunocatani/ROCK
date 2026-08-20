/*
 * PhysicsInteraction LIFECYCLE: construction, init, shutdown, destruction, and the
 * epoch bookkeeping that decides when generated bodies must be rebuilt.
 *
 * Read this file to learn when the subsystem exists and when it stops existing.
 * Per-frame work lives in core/frame/. This file owns only the boundaries.
 *
 * The ctor stores s_instance and the dtor clears it. The workbench menu sink and
 * PhysicsHooks.cpp reach in through that pointer, so the store/clear pair must stay
 * in the ctor/dtor and must not move to init/shutdown.
 *
 * Also owns the teardown helpers that several callers share: releaseHeldObjectsForTeardown,
 * restoreAllHandCollisionLeases, and yieldFrameAndDispatch. They live here because
 * shutdown and the frame-interrupt paths both need the same ordered teardown.
 */

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
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
#include "physics-interaction/hand/HeldBodyRenderPose.h"
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
#include "physics-interaction/hand/skeleton/HandSkeleton.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/debug/overlay/DebugBodyOverlay.h"
#include "physics-interaction/debug/overlay/DebugOverlayPolicy.h"
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
#include "physics-interaction/native/havok/HavokRuntime.h"
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
#include "physics-interaction/weapon/collision/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/equip/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/native_anim/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/native_anim/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/native_anim/NativeEquippedWeaponDraw.h"
#include "physics-interaction/weapon/equip/WeaponTransitionAnimationAcceleration.h"
#include "physics-interaction/weapon/equip/PipboyEquipRuntime.h"
#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/equip/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsRayCast.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/debug/PhysicsWorldOriginDiagnostics.h"
#include "physics-interaction/collision/PushAssist.h"
#include "physics-interaction/hand/selection/HandSelection.h"
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
    using namespace physics_interaction_detail;

    namespace
    {
        constexpr std::array<std::string_view, 5> kWeaponCollisionWorkbenchExitMenuNames{
            "ExamineMenu",
            "PowerArmorModMenu",
            "RobotModMenu",
            "Crafting Menu",
            "CraftingMenu",
        };

        std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkRegistered{ false };
        std::atomic<bool> s_weaponCollisionWorkbenchExitMenuSinkMissingUILogged{ false };


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


        bool registerWeaponCollisionWorkbenchExitMenuSink()
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


        void clearFiringGripInputState()
        {
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
            input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
        }

    }

    bool physics_interaction_detail::ensureWeaponCollisionWorkbenchExitMenuSinkRegistered()
    {
        return registerWeaponCollisionWorkbenchExitMenuSink();
    }

    void physics_interaction_detail::clearEquippedWeaponFiringGripInputState()
    {
        clearFiringGripInputState();
    }

    PhysicsInteraction::PhysicsInteraction(std::uint32_t skeletonGeneration, std::uint32_t providerGeneration)
    {
        s_instance.store(this, std::memory_order_release);
        _lifecycleState.skeletonGeneration = skeletonGeneration == 0 ? 1 : skeletonGeneration;
        _lifecycleState.providerGeneration = providerGeneration == 0 ? 1 : providerGeneration;
        _skeletonGenerationAtomic.store(_lifecycleState.skeletonGeneration, std::memory_order_release);
        _providerGenerationAtomic.store(_lifecycleState.providerGeneration, std::memory_order_release);
        _generatedBodyStepDrive.setDriveCallbacks(
            &PhysicsInteraction::onHeldBodyRenderPoseBeforeWholeStep,
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


    void PhysicsInteraction::releaseHeldObjectsForTeardown(
        RE::hknpWorld* world,
        GrabReleaseCollisionRestoreMode restoreMode)
    {
        const auto releaseHand = [&](Hand& hand, bool isLeft) {
            if (!hand.isHolding()) {
                return;
            }

            // Save the ref before the hand clears its held state.
            auto* heldRef = hand.getHeldRef();
            hand.releaseGrabbedObject(
                world,
                restoreMode,
                makeGrabReleaseContext(hand, isLeft));
            if (heldRef) {
                releaseObject(heldRef, claimOwnerForHand(isLeft));
            }
        };

        releaseHand(_rightHand, false);
        releaseHand(_leftHand, true);
    }

    void PhysicsInteraction::yieldFrameAndDispatch(bool resetWeaponGrip)
    {
        if (resetWeaponGrip) {
            _twoHandedGrip.reset();
        }
        _pendingEquippedWeaponPrimaryOnlyGripStart = {};
        clearEquippedWeaponFiringGripInputState();
        _shoulderStashStates = {};
        _mouthConsumeStates = {};
        _feedbackHaptics.reset();

        // Provider callbacks still receive every yielded game frame.
        ::rock::provider::dispatchFrameCallbacks(*this);
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
            restoreAllHandCollisionLeases(hknp);
            releaseHeldObjectsForTeardown(
                hknp,
                GrabReleaseCollisionRestoreMode::Immediate);
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
            clearAllHandCollisionSuppressionState();
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
        clearWeaponContact(true);
        clearWeaponContact(false);
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
        // This repeats the stale-world clear above. Keep the lifecycle reset
        // idempotent until runtime testing proves that the first clear can go.
        clearAllHandCollisionSuppressionState();
        _nativePlayerCollisionSuppressedBodies = {};
        _nativePlayerCollisionSuppressedBodyCount = 0;
        _nativePlayerCollisionSuppressionRefreshFrames = 0;
        _nativePlayerCollisionSuppressionOverflowLogged = false;

        cleanupGrabConstraintVtable();

        ROCK_LOG_INFO(Init, "ROCK physics module shut down");
    }


}
