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


        void clearEquippedWeaponFiringGripInputState()
        {
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
            input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
        }

        [[nodiscard]] float pointLength(const RE::NiPoint3& value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
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


        const char* weaponDiagnosticNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }

            const char* name = node->name.c_str();
            return name ? name : "";
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
                        restoreAllHandCollisionLeases(hknpMenu);
                        releaseHeldObjectsForTeardown(
                            hknpMenu,
                            GrabReleaseCollisionRestoreMode::Delayed);
                    }
                } else {
                    clearAllHandCollisionSuppressionState();
                }
            }
            debug::ClearFrame();
            auto* snapshotBhk = getPlayerBhkWorld();
            auto* snapshotHknp = snapshotBhk ? getHknpWorld(snapshotBhk) : nullptr;
            if (snapshotBhk && snapshotHknp) {
                _dynamicWorldCarCollision.restoreAll(snapshotBhk, snapshotHknp, "menu-blocked");
            } else {
                _dynamicWorldCarCollision.abandon();
            }
            observeLifecycleFrame(snapshotBhk, snapshotHknp, ::rock::provider::RockProviderLifecycleReason::MenuBlocked);
            restoreHeldMassMovementSlowdown("menu-blocked");
            yieldFrameAndDispatch(false);
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
            debug::ClearFrame();
            restoreHeldMassMovementSlowdown("world-unavailable");
            yieldFrameAndDispatch(true);
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
            releaseHeldObjectsForTeardown(
                hknp,
                GrabReleaseCollisionRestoreMode::Immediate);
            restoreAllHandCollisionLeases(hknp);
            _twoHandedGrip.reset();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _bodyContactRuntime.reset();
            clearWeaponContact(true);
            clearWeaponContact(false);

            destroyHandCollisions(bhk);
            destroyBodyBoneCollisions(bhk);
            _weaponCollision.invalidateForScaleChange(hknp);
            markGeneratedBodiesInvalidated();
            clearAllHandCollisionSuppressionState();
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
                yieldFrameAndDispatch(true);
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
            yieldFrameAndDispatch(true);
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

        // Keep the shared collision matrix at the configured policy.
        serviceCollisionLayerDrift(hknp);

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

        EquippedWeaponFrame weaponFrame{
            .weaponNode = weaponNode,
            .generationKey = currentWeaponGenerationKey,
            .ownershipKey = currentEquippedWeaponOwnershipKey,
            .rightHandWeaponEquipped = rightHandWeaponEquipped,
            .retainedWeaponCollisionActive = retainedWeaponCollisionActive,
            .rightHandWeaponAuthorityActive =
                rightHandWeaponAuthorityActive,
            .rightHandWeaponAuthorityActiveBeforeGrip =
                rightHandWeaponAuthorityActiveBeforeGrip,
            .leftSupportGripActive = leftSupportGripActive,
            .rightPartGripActive = rightPartGripActive,
        };
        // Acquire hand evidence before any grip can change ownership.
        serviceWeaponContactAcquisition(frame, weaponFrame);
        // Resolve stash, grip, drop, and hand assignment in input order.
        serviceEquippedWeaponGripFrame(frame, weaponFrame);
        // Publish dynamic weapon authority after the grip solve is final.
        finishDynamicWeaponFrame(frame, weaponFrame);

        rightHandWeaponAuthorityActive =
            weaponFrame.rightHandWeaponAuthorityActive;
        leftSupportGripActive = weaponFrame.leftSupportGripActive;
        rightPartGripActive = weaponFrame.rightPartGripActive;
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

    void PhysicsInteraction::serviceWeaponContactAcquisition(
        const PhysicsFrameContext& frame,
        EquippedWeaponFrame& weaponFrame)
    {
        auto* weaponNode = weaponFrame.weaponNode;
        auto& leftWeaponContact = weaponFrame.leftContact;
        auto& rightWeaponContact = weaponFrame.rightContact;
        auto& leftWeaponContactSource = weaponFrame.leftContactSource;

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
                        clearWeaponContact(isLeft);
                    }
                }
            } else {
                const auto missed = missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
                if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                    clearWeaponContact(isLeft);
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

        weaponFrame.observedWeapon = observedEquippedWeapon;
        weaponFrame.observedFormId = observedEquippedWeaponFormID;
        weaponFrame.observedInstanceData =
            observedEquippedWeaponInstanceData;
        weaponFrame.shoulderStashActive =
            equippedWeaponShoulderStashActive;
        weaponFrame.menuInputActive = inputBlockingMenuActive;
        weaponFrame.pendingPrimaryStartMatches =
            pendingPrimaryStartMatchesCurrentWeapon;
        weaponFrame.firingHandIsLeft = firingHandIsLeft;
        weaponFrame.supportHandIsLeft = supportHandIsLeft;
    }

    void PhysicsInteraction::serviceEquippedWeaponGripFrame(
        const PhysicsFrameContext& frame,
        EquippedWeaponFrame& weaponFrame)
    {
        auto* hknp = frame.hknpWorld;
        auto* weaponNode = weaponFrame.weaponNode;
        const auto currentWeaponGenerationKey =
            weaponFrame.generationKey;
        const auto currentEquippedWeaponOwnershipKey =
            weaponFrame.ownershipKey;
        auto& leftWeaponContact = weaponFrame.leftContact;
        auto& rightWeaponContact = weaponFrame.rightContact;
        const auto leftWeaponContactSource =
            weaponFrame.leftContactSource;
        auto* observedEquippedWeapon =
            weaponFrame.observedWeapon;
        const auto observedEquippedWeaponFormID =
            weaponFrame.observedFormId;
        const auto observedEquippedWeaponInstanceData =
            weaponFrame.observedInstanceData;
        const bool equippedWeaponShoulderStashActive =
            weaponFrame.shoulderStashActive;
        const bool inputBlockingMenuActive =
            weaponFrame.menuInputActive;
        const bool pendingPrimaryStartMatchesCurrentWeapon =
            weaponFrame.pendingPrimaryStartMatches;
        const bool firingHandIsLeft =
            weaponFrame.firingHandIsLeft;
        const bool supportHandIsLeft =
            weaponFrame.supportHandIsLeft;
        const bool rightHandWeaponEquipped =
            weaponFrame.rightHandWeaponEquipped;
        const bool retainedWeaponCollisionActive =
            weaponFrame.retainedWeaponCollisionActive;
        const bool rightHandWeaponAuthorityActiveBeforeGrip =
            weaponFrame.rightHandWeaponAuthorityActiveBeforeGrip;
        auto& rightHandWeaponAuthorityActive =
            weaponFrame.rightHandWeaponAuthorityActive;
        auto& leftSupportGripActive =
            weaponFrame.leftSupportGripActive;
        auto& rightPartGripActive =
            weaponFrame.rightPartGripActive;
        auto& drivenSourceNodes =
            weaponFrame.drivenSourceNodes;
        auto& drivenSourceNodeCount =
            weaponFrame.drivenSourceNodeCount;
        auto*& gunstockProjectileNode =
            weaponFrame.gunstockProjectileNode;
        const auto& runtime = runtime_state::currentFrame();

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

                if (stashDecision.candidate &&
                    g_rockConfig.rockShoulderStashHapticsEnabled &&
                    shouldEmitShoulderStashCandidatePulse(
                        stashDecision,
                        stashState,
                        _dynamicPushElapsedSeconds)) {
                        (void)_feedbackHaptics.queue(
                            stashHandIsLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                            g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
                            shoulder_stash_haptic_policy::computeCandidatePulseIntensity(stashDecision.confidence,
                                shoulder_stash_haptic_policy::CandidatePulseConfig{
                                    .enabled = true,
                                    .baseIntensity = g_rockConfig.rockShoulderStashCandidateHapticBaseIntensity,
                                    .maxIntensity = g_rockConfig.rockShoulderStashCandidateHapticIntensity,
                                }));
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
        gunstockProjectileNode =
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
    }

    void PhysicsInteraction::finishDynamicWeaponFrame(
        const PhysicsFrameContext& frame,
        EquippedWeaponFrame& weaponFrame)
    {
        auto* hknp = frame.hknpWorld;
        auto* weaponNode = weaponFrame.weaponNode;
        const auto currentWeaponGenerationKey =
            weaponFrame.generationKey;
        const auto& drivenSourceNodes =
            weaponFrame.drivenSourceNodes;
        const auto drivenSourceNodeCount =
            weaponFrame.drivenSourceNodeCount;
        auto* gunstockProjectileNode =
            weaponFrame.gunstockProjectileNode;

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

    void PhysicsInteraction::clearWeaponContact(bool isLeft)
    {
        auto& bodyId = isLeft ?
            _leftWeaponContactBodyId : _rightWeaponContactBodyId;
        auto& partKind = isLeft ?
            _leftWeaponContactPartKind : _rightWeaponContactPartKind;
        auto& reloadRole = isLeft ?
            _leftWeaponContactReloadRole : _rightWeaponContactReloadRole;
        auto& supportRole = isLeft ?
            _leftWeaponContactSupportRole : _rightWeaponContactSupportRole;
        auto& socketRole = isLeft ?
            _leftWeaponContactSocketRole : _rightWeaponContactSocketRole;
        auto& actionRole = isLeft ?
            _leftWeaponContactActionRole : _rightWeaponContactActionRole;
        auto& gripPose = isLeft ?
            _leftWeaponContactGripPose : _rightWeaponContactGripPose;
        auto& missedFrames = isLeft ?
            _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;

        // Publish the invalid body first so readers stop using this bank.
        bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        partKind.store(
            static_cast<std::uint32_t>(WeaponPartKind::Other),
            std::memory_order_release);
        reloadRole.store(
            static_cast<std::uint32_t>(WeaponReloadRole::None),
            std::memory_order_release);
        supportRole.store(
            static_cast<std::uint32_t>(WeaponSupportGripRole::None),
            std::memory_order_release);
        socketRole.store(
            static_cast<std::uint32_t>(WeaponSocketRole::None),
            std::memory_order_release);
        actionRole.store(
            static_cast<std::uint32_t>(WeaponActionRole::None),
            std::memory_order_release);
        gripPose.store(
            static_cast<std::uint32_t>(WeaponGripPoseId::None),
            std::memory_order_release);
        missedFrames.store(
            WEAPON_CONTACT_TIMEOUT_FRAMES + 1,
            std::memory_order_release);
        _weaponInteractionAcquisitionStates[isLeft ? 0u : 1u] = {};
    }

    void PhysicsInteraction::publishWeaponInteractionContact(
        bool isLeft,
        WeaponInteractionContact& contact)
    {
        auto& partKind = isLeft ?
            _leftWeaponContactPartKind : _rightWeaponContactPartKind;
        auto& reloadRole = isLeft ?
            _leftWeaponContactReloadRole : _rightWeaponContactReloadRole;
        auto& supportRole = isLeft ?
            _leftWeaponContactSupportRole : _rightWeaponContactSupportRole;
        auto& socketRole = isLeft ?
            _leftWeaponContactSocketRole : _rightWeaponContactSocketRole;
        auto& actionRole = isLeft ?
            _leftWeaponContactActionRole : _rightWeaponContactActionRole;
        auto& gripPose = isLeft ?
            _leftWeaponContactGripPose : _rightWeaponContactGripPose;
        auto& sequence = isLeft ?
            _leftWeaponContactSequence : _rightWeaponContactSequence;
        auto& missedFrames = isLeft ?
            _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;

        partKind.store(
            static_cast<std::uint32_t>(contact.partKind),
            std::memory_order_release);
        reloadRole.store(
            static_cast<std::uint32_t>(contact.reloadRole),
            std::memory_order_release);
        supportRole.store(
            static_cast<std::uint32_t>(contact.supportGripRole),
            std::memory_order_release);
        socketRole.store(
            static_cast<std::uint32_t>(contact.socketRole),
            std::memory_order_release);
        actionRole.store(
            static_cast<std::uint32_t>(contact.actionRole),
            std::memory_order_release);
        gripPose.store(
            static_cast<std::uint32_t>(contact.fallbackGripPose),
            std::memory_order_release);
        contact.sequence =
            sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        missedFrames.store(0, std::memory_order_release);
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

        const GrabInputFrame input{
            .frame = &frame,
            .farHmdConeGate = farHmdConeGate,
            .worldGeneration = worldGeneration,
            .skeletonGeneration = skeletonGeneration,
            .providerGeneration = providerGeneration,
            .collisionGeneration = collisionGeneration,
            .grabButton = grabButton,
            .rightHandWeaponEquipped = rightHandWeaponEquipped,
            .ambidextrousHandoffAvailable =
                ambidextrousHandoffAvailable,
            .primaryDetachEnabled =
                firingGripModes.primaryDetachEnabled,
            .gripZoneSettleEquipEnabled =
                gripZoneSettleEquipEnabled,
        };

        processGrabInputHand(_rightHand, false, input);
        publishHandInputOwnership(_rightHand, false);
        processGrabInputHand(_leftHand, true, input);
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

    void PhysicsInteraction::processGrabInputHand(
        Hand& hand,
        bool isLeft,
        const GrabInputFrame& input)
    {
        const auto& frame = *input.frame;
        auto* hknp = frame.hknpWorld;
        const auto worldGeneration = input.worldGeneration;
        const auto skeletonGeneration = input.skeletonGeneration;
        const auto providerGeneration = input.providerGeneration;
        const auto collisionGeneration = input.collisionGeneration;
        const int grabButton = input.grabButton;
        const bool rightHandWeaponEquipped =
            input.rightHandWeaponEquipped;
        const bool ambidextrousHandoffAvailable =
            input.ambidextrousHandoffAvailable;
        const bool gripZoneSettleEquipEnabled =
            input.gripZoneSettleEquipEnabled;
        const auto& farHmdConeGate = input.farHmdConeGate;
        const equipped_weapon_manual_ownership_policy::
            FiringGripModeAvailability firingGripModes{
                .primaryDetachEnabled =
                    input.primaryDetachEnabled,
                .ambidextrousHandoffAvailable =
                    ambidextrousHandoffAvailable,
            };

        const auto clearShoulderStashForHand =
            [&](Hand& targetHand, bool targetIsLeft) {
                shoulder_stash::resetRuntime(
                    _shoulderStashStates[targetIsLeft ? 1u : 0u]);
                targetHand.cancelStashCandidate();
            };
        const auto clearMouthConsumeForHand =
            [&](Hand& targetHand, bool targetIsLeft) {
                mouth_consume::resetRuntime(
                    _mouthConsumeStates[targetIsLeft ? 1u : 0u]);
                targetHand.cancelConsumeCandidate();
            };
        const auto clearGameplayCandidatesForHand =
            [&](Hand& targetHand, bool targetIsLeft) {
                clearShoulderStashForHand(
                    targetHand,
                    targetIsLeft);
                clearMouthConsumeForHand(
                    targetHand,
                    targetIsLeft);
            };
        const auto releaseSuppressedHeldObject =
            [&](Hand& targetHand,
                bool targetIsLeft,
                const char* reason) {
                auto* heldRef = targetHand.getHeldRef();
                const auto heldFormID =
                    heldRef ? heldRef->GetFormID() : 0u;
                targetHand.releaseGrabbedObject(
                    hknp,
                    GrabReleaseCollisionRestoreMode::Delayed,
                    makeGrabReleaseContext(
                        targetHand,
                        targetIsLeft));
                if (heldRef) {
                    releaseObject(
                        heldRef,
                        claimOwnerForHand(targetIsLeft));
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: released held object because normal grab input is suppressed ({})",
                    targetHand.handName(),
                    reason ? reason : "unknown");
                dispatchPhysicsMessage(
                    kPhysMsg_OnRelease,
                    targetIsLeft,
                    heldRef,
                    heldFormID,
                    0);
                dispatchSimpleGrabEvent(
                    GrabEventType::Released,
                    targetIsLeft,
                    heldRef);
            };

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
                    if (shouldEmitShoulderStashCandidatePulse(
                            stashDecision,
                            shoulderStashState,
                            _dynamicPushElapsedSeconds)) {
                        dispatchShoulderStashEvent(
                            GrabEventType::StashCandidate,
                            heldRefForGameplay,
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0,
                            hand.getSavedObjectState().bodyId.value,
                            stashDecision);
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
    }

}
