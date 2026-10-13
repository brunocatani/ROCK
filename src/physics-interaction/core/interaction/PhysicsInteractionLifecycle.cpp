#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/native/BipodFrozenRetirement.h"
#include "api/StorageService.h"
#include "api/AuthoredInteractionService.h"
#include "api/WeaponPartCollisionService.h"
#include "physics-interaction/visual/GripZoneIndicators.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/weapon/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"

// PhysicsInteraction lifecycle: construction, init/shutdown, skeleton and provider lifecycle notes, generated-body lifecycle, collision layer registration, hand/body collision creation, and world accessors. Includes the provider API surface (PhysicsInteractionProvider.inl).

namespace rock
{
    PhysicsInteraction::PhysicsInteraction(std::uint32_t skeletonGeneration, std::uint32_t providerGeneration)
    {
        s_instance.store(this, std::memory_order_release);
        _lifecycle.state.skeletonGeneration = skeletonGeneration == 0 ? 1 : skeletonGeneration;
        _lifecycle.state.providerGeneration = providerGeneration == 0 ? 1 : providerGeneration;
        _lifecycle.skeletonGenerationAtomic.store(_lifecycle.state.skeletonGeneration, std::memory_order_release);
        _lifecycle.providerGenerationAtomic.store(_lifecycle.state.providerGeneration, std::memory_order_release);
        _generatedBodyStepDrive.setDriveCallbacks(
            nullptr,
            &PhysicsInteraction::onGeneratedColliderPhysicsSubstep,
            &PhysicsInteraction::onCustomGrabAuthorityBetweenStep,
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
        _secondaryWeaponCollision.configureSecondaryWeapon();
        _secondaryWeaponCollision.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _dynamicWeaponCollision.setPhysicsCallbackGate(generatedBodyCallbackGate);
        _secondaryDynamicWeaponCollision.configureSecondaryWeapon();
        _secondaryDynamicWeaponCollision.setPhysicsCallbackGate(generatedBodyCallbackGate);
        native_secondary_presentation::setCollisionResolver(this, &PhysicsInteraction::resolveSecondaryWeaponCollision);
        native_interaction_service::initialize(generatedBodyCallbackGate,
            [](void* context,void* bhk){static_cast<PhysicsInteraction*>(context)->_dynamicWeaponCollision.retireAll(bhk);},
            [](void* context,RE::hknpWorld* world){static_cast<PhysicsInteraction*>(context)->registerCollisionLayer(world);},this);
        _twoHandedGrip.setWeaponVisualIntentObserver(
            &_dynamicWeaponCollision,
            &DynamicWeaponCollisionRuntime::observeWeaponVisualIntent);
        _twoHandedGrip.setSurfaceSupportRuntime(&_dynamicWeaponCollision);
        clearLooseGrenadeImpactWatches();

        native_player_collision::install();
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
        native_secondary_presentation::setCollisionResolver(nullptr, nullptr);

        _equipped.authoredPrimaryFiringGrip.reset("physics-destroyed", _twoHandedGrip);

        if (_lifecycle.initialized) {
            shutdown();
        }
        native_interaction_service::detach(&_generatedBodyStepDrive.callbackGate());
        ROCK_LOG_INFO(Init, "ROCK Physics Module — destroyed");
    }

    void PhysicsInteraction::noteSkeletonLifecycle(std::uint32_t skeletonGeneration, ::rock::provider::RockProviderLifecycleReason reason)
    {
        auto* bhk = getPlayerBhkWorld();
        auto* world = bhk && bhk == _lifecycle.cachedBhkWorld ? getHknpWorld(bhk) : nullptr;
        clearNativePlayerCollisionFilter(world == _lifecycle.cachedHknpWorld ? world : nullptr);
        _equipped.authoredPrimaryFiringGrip.reset("skeleton-lifecycle", _twoHandedGrip);
        physics_lifecycle::noteGeneration(_lifecycle.state.skeletonGeneration, skeletonGeneration, _lifecycle.state, reason);
        physics_lifecycle::noteReason(_lifecycle.state, reason);
        markGeneratedBodiesInvalidated();
        _lifecycle.state.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        _lifecycle.state.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::GeneratedBodiesValid);
        _lifecycle.state.flags |= static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::LoadingOrWorldTransition);
        _lifecycle.flagsAtomic.store(_lifecycle.state.flags, std::memory_order_release);
        _lifecycle.skeletonGenerationAtomic.store(_lifecycle.state.skeletonGeneration, std::memory_order_release);
        _lifecycle.stableFrameCountAtomic.store(_lifecycle.state.stableFrameCount, std::memory_order_release);
        _lifecycle.lastReasonAtomic.store(static_cast<std::uint32_t>(_lifecycle.state.lastReason), std::memory_order_release);
        _lifecycle.hknpWorldAtomic.store(nullptr, std::memory_order_release);
    }

    void PhysicsInteraction::noteProviderLifecycle(std::uint32_t providerGeneration, ::rock::provider::RockProviderLifecycleReason reason)
    {
        _equipped.authoredPrimaryFiringGrip.reset("provider-lifecycle", _twoHandedGrip);
        physics_lifecycle::noteGeneration(_lifecycle.state.providerGeneration, providerGeneration, _lifecycle.state, reason);
        physics_lifecycle::noteReason(_lifecycle.state, reason);
        markGeneratedBodiesInvalidated();
        _lifecycle.state.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        _lifecycle.flagsAtomic.store(_lifecycle.state.flags, std::memory_order_release);
        _lifecycle.providerGenerationAtomic.store(_lifecycle.state.providerGeneration, std::memory_order_release);
        _lifecycle.stableFrameCountAtomic.store(_lifecycle.state.stableFrameCount, std::memory_order_release);
        _lifecycle.lastReasonAtomic.store(static_cast<std::uint32_t>(_lifecycle.state.lastReason), std::memory_order_release);
    }

    bool PhysicsInteraction::generatedBodiesExistForConfig() const
    {
        return _rightHand.hasCollisionBody() && _leftHand.hasCollisionBody();
    }

    bool PhysicsInteraction::generatedBodiesLiveIn(RE::bhkWorld* bhk, RE::hknpWorld* hknp) const
    {
        return generatedBodiesExistForConfig() && bhk && hknp &&
               _lifecycle.generatedBodiesBhkWorld == bhk && _lifecycle.generatedBodiesHknpWorld == hknp;
    }

    void PhysicsInteraction::markGeneratedBodiesRebuilt(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        if (!bhk || !hknp || !generatedBodiesExistForConfig()) {
            markGeneratedBodiesInvalidated();
            return;
        }

        _lifecycle.generatedBodiesBhkWorld = bhk;
        _lifecycle.generatedBodiesHknpWorld = hknp;
        _lifecycle.collisionGenerationAtomic.fetch_add(1, std::memory_order_acq_rel);
        refreshGeneratedBodyContactRegistry();
    }

    void PhysicsInteraction::markGeneratedBodiesInvalidated()
    {
        const auto collisionGeneration =
            _lifecycle.collisionGenerationAtomic.fetch_add(
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
            ::rock::api::touch::TouchGrabReleaseReason::
                GenerationChanged,
            collisionGeneration);
        // Close callback entry and drain any native step already traversing
        // ROCK-owned body banks before clearing registry or wrapper state.
        _generatedBodyStepDrive.reset();
        resetHeldPlacement();
        if (currentBhkWorld != _lifecycle.generatedBodiesBhkWorld) {
            bipod_frozen_retirement::cancelForWorldLoss();
            _bipodRetirementVisual.abandonSceneGraph();
            _bipodRetirementHandle = {};
        }

        clearGeneratedBodyContactRegistry();
        const bool generatedWorldStillLive =
            currentBhkWorld &&
            currentBhkWorld == _lifecycle.generatedBodiesBhkWorld &&
            currentHknpWorld &&
            currentHknpWorld == _lifecycle.generatedBodiesHknpWorld;
        native_interaction_service::clear(generatedWorldStillLive ? currentHknpWorld : nullptr,
            generatedWorldStillLive ? currentBhkWorld : nullptr);
        _looseWeaponCollision.clear(generatedWorldStillLive ? currentHknpWorld : nullptr);
        _objectRestCollision.clear(generatedWorldStillLive ? currentHknpWorld : nullptr);_objectRestRequests={};
        clearEjectionContacts(generatedWorldStillLive ? currentHknpWorld : nullptr);
        if (generatedWorldStillLive) {
            _dynamicHandCollision.retireAll(_lifecycle.generatedBodiesBhkWorld);
            _dynamicWeaponCollision.retireAll(_lifecycle.generatedBodiesBhkWorld);
            _secondaryDynamicWeaponCollision.retireAll(_lifecycle.generatedBodiesBhkWorld);
        } else {
            _dynamicHandCollision.reset();
            _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
            _secondaryDynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
        }
        _lifecycle.generatedBodiesBhkWorld = nullptr;
        _lifecycle.generatedBodiesHknpWorld = nullptr;
        _lifecycle.state.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::GeneratedBodiesValid);
        _lifecycle.state.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        _lifecycle.flagsAtomic.store(_lifecycle.state.flags, std::memory_order_release);
        _lifecycle.stableFrameCountAtomic.store(_lifecycle.state.stableFrameCount, std::memory_order_release);
        _lifecycle.hknpWorldAtomic.store(nullptr, std::memory_order_release);
        _frame.completedPhysicsSolveSequence.store(0, std::memory_order_release);
        for (auto& visual : _drop.visuals) {
            if (generatedWorldStillLive) visual.release("generated-bodies-invalidated");
            else visual.abandonSceneGraph();
        }
        _drop.nativeHandoffs = {};
        _grabInput.shoulderStashStates = {};
        _grabInput.mouthConsumeStates = {};
        _feedbackHaptics.reset();
    }

    void PhysicsInteraction::clearGeneratedBodyContactRegistry()
    {
        _contacts.generatedBodyRegistry.clear();
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
                if (!hand.tryGetHandColliderMetadataAtIndex(i, bodyId, metadata) || !metadata.valid) {
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

        std::array<WeaponCollision::WeaponContactState, MAX_WEAPON_COLLISION_BODIES> weaponStates{};
        const auto weaponCount = _weaponCollision.copyWeaponContactStatesAtomic(weaponStates);
        for (std::size_t i = 0; i < weaponCount; ++i) {
            const auto& state = weaponStates[i];
            const auto& contact = state.contact;
            if (!contact.valid) {
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

            if (state.hasSampledVelocity) {
                entry.flags |= kFlagSampledVelocity;
                entry.sampledVelocityHavokX = state.sampledVelocityHavok[0];
                entry.sampledVelocityHavokY = state.sampledVelocityHavok[1];
                entry.sampledVelocityHavokZ = state.sampledVelocityHavok[2];
            }
            addEntry(entry);
        }

        const std::uint32_t bodyCount = (std::min)(_bodyBoneColliders.getBodyCount(), static_cast<std::uint32_t>(kBodyBoneColliderBodyCount));
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const std::uint32_t bodyId = _bodyBoneColliders.getBodyIdAtomic(i);
            BodyBoneColliderMetadata metadata{};
            if (!_bodyBoneColliders.tryGetBodyMetadataAtIndexAtomic(i, bodyId, metadata) || !metadata.valid) {
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

        _contacts.generatedBodyRegistry.publish(entries.data(), entryCount);
    }

    void PhysicsInteraction::observeLifecycleFrame(RE::bhkWorld* bhk, RE::hknpWorld* hknp, ::rock::provider::RockProviderLifecycleReason reasonHint)
    {
        const auto& runtime = runtime_state::currentFrame();
        physics_lifecycle::FrameInputs inputs{};
        inputs.bhkWorld = reinterpret_cast<std::uintptr_t>(bhk);
        inputs.hknpWorld = reinterpret_cast<std::uintptr_t>(hknp);
        inputs.providerReady = _lifecycle.initialized.load(std::memory_order_acquire) && runtime.visualAuthorityAvailable;
        inputs.skeletonReady = runtime.localSkeletonReady;
        inputs.menuBlocking = runtime.localMenuBlocking;
        inputs.configBlocking = runtime.compatibilityConfigBlocking;
        inputs.generatedBodiesValid = generatedBodiesLiveIn(bhk, hknp);
        inputs.reasonHint = reasonHint;

        physics_lifecycle::observeFrame(_lifecycle.state, inputs);
        _lifecycle.cachedBhkWorld = bhk;
        _lifecycle.cachedHknpWorld = hknp;
        _lifecycle.flagsAtomic.store(_lifecycle.state.flags, std::memory_order_release);
        _lifecycle.lastReasonAtomic.store(static_cast<std::uint32_t>(_lifecycle.state.lastReason), std::memory_order_release);
        _lifecycle.worldGenerationAtomic.store(_lifecycle.state.worldGeneration, std::memory_order_release);
        _lifecycle.skeletonGenerationAtomic.store(_lifecycle.state.skeletonGeneration, std::memory_order_release);
        _lifecycle.providerGenerationAtomic.store(_lifecycle.state.providerGeneration, std::memory_order_release);
        _lifecycle.stableFrameCountAtomic.store(_lifecycle.state.stableFrameCount, std::memory_order_release);
        _lifecycle.hknpWorldAtomic.store(hknp, std::memory_order_release);
    }

    bool PhysicsInteraction::physicsWritesAllowedForWorld(RE::hknpWorld* world) const
    {
        if (!world || world != _lifecycle.hknpWorldAtomic.load(std::memory_order_acquire)) {
            return false;
        }

        return ::rock::provider::hasLifecycleFlag(
            _lifecycle.flagsAtomic.load(std::memory_order_acquire),
            ::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
    }

#include "physics-interaction/core/PhysicsInteractionProvider.inl"
    bool PhysicsInteraction::validateCriticalOffsets() const
    {
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

    void PhysicsInteraction::init()
    {
        if (_lifecycle.initialized) {
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
        if (!nativeMeleeSuppressionHooksInstalled && !g_rockConfig.rockEnableVanillaMelee) {
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

        if (!frik_visual_authority::blockOffHandWeaponGripping(
                "ROCK_Physics",
                true)) {
            ROCK_LOG_CRITICAL(
                Init,
                "ROCK DISABLED: mandatory hFRIK off-hand weapon-grip suppression could not be acquired");
            return;
        }

        physics_scale::refreshAndLogIfChanged();
        _lifecycle.cachedBhkWorld = bhk;
        _lifecycle.cachedHknpWorld = hknp;
        if (!refreshHandBoneCache()) {
            ROCK_LOG_WARN(Init, "HandBoneCache not ready during init; runtime remains on pre-00 transform paths");
        }

        registerCollisionLayer(hknp);
        if (!_layers.registered) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: collision layer registration failed");
            (void)frik_visual_authority::blockOffHandWeaponGripping(
                "ROCK_Physics",
                false);
            _lifecycle.cachedBhkWorld = nullptr;
            _lifecycle.cachedHknpWorld = nullptr;
            return;
        }

        if (!createHandCollisions(hknp, bhk)) {
            ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: hand collision body creation failed");
            (void)frik_visual_authority::blockOffHandWeaponGripping(
                "ROCK_Physics",
                false);
            _lifecycle.cachedBhkWorld = nullptr;
            _lifecycle.cachedHknpWorld = nullptr;
            return;
        }

        if (g_rockConfig.rockBodyBoneCollidersEnabled && !createBodyBoneCollisions(hknp, bhk)) {
            ROCK_LOG_WARN(Init, "Body bone colliders were not available during init; runtime update will retry");
        }

        _contacts.handActivity.reset();
        _contacts.bodyRuntime.reset();
        subscribeContactEvents(hknp);

        _weaponCollision.init(hknp, bhk);
        _secondaryWeaponCollision.init(hknp, bhk);
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        ROCK_LOG_INFO(Init, "hFRIK off-hand weapon gripping disabled; ROCK is the sole off-hand weapon authority");

        {
            captureHandColliderBones();
            _rightHand.updateCollisionTransform(hknp, getInteractionHandTransform(false), 0.011f, _handColliderBoneSnapshot);
            _leftHand.updateCollisionTransform(hknp, getInteractionHandTransform(true), 0.011f, _handColliderBoneSnapshot);
            _bodyBoneColliders.update(hknp, 0.011f);
            ROCK_LOG_INFO(Init, "Initial bone-derived hand collider transforms updated");
        }

        _frame.hasPrevPositions = false;
        _diagnostics.deltaLogCounter = 0;
        _diagnostics.contactLogCounter = 0;
        _contacts.bodyRuntime.reset();
        _contacts.hapticClockSeconds = 0.0f;
        _grabEvents.heldImpactHapticCooldownUntil.clear();
        _grabEvents.frameCounter = 0;
        _grabInput.shoulderStashStates = {};
        _grabInput.mouthConsumeStates = {};
        _feedbackHaptics.reset();
        _grabInput.intentStates = {};
        _grabInput.peerHeldJoinRetryStates = {};
        _grabInput.heldWeaponTriggerEquipIntents = {};
        _forceGrab.committedThisFrame = {};
        equipped_weapon_toggle_grab_policy::reset(
            _equipped.toggleGrabState);
        _equipped.toggleGrabReleasePressConsumedThisFrame = {};
        _equipped.holsterInputStates = {};
        _equipped.holsterInputConsumedThisFrame = {};
        _equipped.shoulderSheath = {};
        input_remap_runtime::setEquippedWeaponShoulderSheathActive(false);
        _equipped.stashStates = {};
        _equipped.sheathRetrievalStates = {};
        equipped_weapon_shoulder::reset(
            _equipped.shoulderCoordinator);
        _equipped.shoulderGestureConsumedThisFrame = {};
        _grabInput.bareFistGuardState = {};
        _frame.completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _drop.nativeHandoffs = {};
        clearLooseGrenadeRuntimeState();
        _equipped.transition.pendingGrip() = {};
        _equipped.handlingSettings = {};
        _equipped.handlingModeInitialized = false;
        _equipped.handlingModeReconcilePending = false;
        _equipped.defaultEquipObservedOwnershipKey = 0;
        equipped_weapon_handling_runtime::reset();
        abandonWeaponHandSuppression();
        _contacts.rightTouch.clear();
        _contacts.leftTouch.clear();
        _contacts.lastHeldImpactPairRight.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _contacts.lastHeldImpactPairLeft.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _contacts.handActivity.reset();

        _lifecycle.initialized = true;
        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::None);
        markGeneratedBodiesRebuilt(bhk, hknp);
        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesRebuilt);

        dispatchPhysicsMessage(kPhysMsg_OnPhysicsInit, false);

        ROCK_LOG_INFO(Init, "ROCK physics module initialized — bhkWorld={}, hknpWorld={}, R_body={}, L_body={}", static_cast<const void*>(bhk), static_cast<const void*>(hknp),
            _rightHand.getCollisionBodyId().value, _leftHand.getCollisionBodyId().value);
    }

    void PhysicsInteraction::shutdown(::rock::provider::RockProviderLifecycleReason reason)
    {
        resetHeldPlacement();
        if (reason == provider::RockProviderLifecycleReason::GameLoaded ||
            reason == provider::RockProviderLifecycleReason::WorldChanged ||
            reason == provider::RockProviderLifecycleReason::WorldUnavailable ||
            reason == provider::RockProviderLifecycleReason::Shutdown)
            bipod_frozen_retirement::cancelForWorldLoss();
        _bipodRetirementVisual.release("physics-shutdown");
        _bipodRetirementHandle = {};
        _equipped.gripResumePending = _equipped.continuityGrip.pending;
        cancelBareFistMode("physics-shutdown");
        _grabInput.bareFistDrawOwned = false;
        input_remap_runtime::setBareFistDrawState(0, false, false);
        weapon_transition_animation_acceleration::cancel("physics-shutdown");
        grip_zone_indicators::Clear();
        debug::ShutdownShapePipeline();
        input_remap_runtime::setRealMeleeWeaponEquipped(false);
        equipped_weapon_handling_runtime::reset();
        _equipped.handlingSettings = {};
        _equipped.handlingModeInitialized = false;
        _equipped.handlingModeReconcilePending = false;
        equipped_weapon_toggle_grab_policy::reset(
            _equipped.toggleGrabState);
        _equipped.toggleGrabReleasePressConsumedThisFrame = {};
        _equipped.holsterInputStates = {};
        _equipped.holsterInputConsumedThisFrame = {};
        _equipped.authoredPrimaryFiringGrip.reset("physics-shutdown", _twoHandedGrip);
        if (!_lifecycle.initialized) {
            _equipped.shoulderSheath = {};
            input_remap_runtime::setEquippedWeaponShoulderSheathActive(false);
            _equipped.stashStates = {};
            _equipped.sheathRetrievalStates = {};
            equipped_weapon_shoulder::reset(
                _equipped.shoulderCoordinator);
            _equipped.shoulderGestureConsumedThisFrame = {};
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
            _lifecycle.cachedBhkWorld &&
            currentBhk == _lifecycle.cachedBhkWorld &&
            _lifecycle.cachedHknpWorld &&
            currentHknp == _lifecycle.cachedHknpWorld;
        native_interaction_service::clear(worldValid ? currentHknp : nullptr, worldValid ? currentBhk : nullptr);

        _looseWeaponCollision.clear(worldValid ? currentHknp : nullptr);
        _objectRestCollision.clear(worldValid ? currentHknp : nullptr);_objectRestRequests={};
        clearEjectionContacts(worldValid ? currentHknp : nullptr);
        storage_service::shutdown(worldValid ? currentHknp : nullptr);
        storage_interaction::shutdown(worldValid ? currentHknp : nullptr);
        weapon_part_collision::shutdown(worldValid ? currentHknp : nullptr);
        authored_interaction::reset();
        native_player_collision::publishAuthoredHands(worldValid ? currentHknp : nullptr, {});
        if (worldValid) {
            auto* hknp = getHknpWorld(_lifecycle.cachedBhkWorld);
            _dynamicWorldCarCollision.restoreAll(_lifecycle.cachedBhkWorld, hknp, "shutdown");
            _touchGrabRuntime.releaseAll(
                _lifecycle.cachedBhkWorld,
                hknp,
                ::rock::api::touch::TouchGrabReleaseReason::
                    GenerationChanged,
                _lifecycle.collisionGenerationAtomic.load(
                    std::memory_order_acquire));
            unsubscribeContactEvents(hknp);
            clearNativePlayerCollisionFilter(hknp);
            restoreNativeGrenadeCollisionSuppression(hknp);
            clearWeaponHandSuppression(hknp, false);
            clearWeaponHandSuppression(hknp, true);
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
            _dynamicWeaponCollision.retireAll(_lifecycle.cachedBhkWorld);
            _secondaryDynamicWeaponCollision.retireAll(_lifecycle.cachedBhkWorld);
            _weaponCollision.destroyWeaponBody(hknp);
            _secondaryWeaponCollision.destroyWeaponBody(hknp);
            destroyBodyBoneCollisions(_lifecycle.cachedBhkWorld);
            destroyHandCollisions(_lifecycle.cachedBhkWorld);
        } else {
            _dynamicWorldCarCollision.abandon();
            _touchGrabRuntime.abandonAll(
                ::rock::api::touch::TouchGrabReleaseReason::
                    WorldLost);
            unsubscribeContactEvents(nullptr);
            ROCK_LOG_INFO(Init, "World stale or null — skipping Havok body destruction");
            _rightHand.abandonHavokStateAfterWorldLoss();
            _leftHand.abandonHavokStateAfterWorldLoss();
            _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
            _secondaryDynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
            _weaponCollision.abandonHavokStateAfterWorldLoss();
            _secondaryWeaponCollision.abandonHavokStateAfterWorldLoss();
            native_idle_grip_preharvest::clearHeldWarmups();
            _equipped.transition.abandonSceneGraph();
            _bodyBoneColliders.reset();
            abandonWeaponHandSuppression();
            _suppression.nativePlayerRefreshFrames = 0;
            _suppression.nativePlayerOverflowLogged = false;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        clearEquippedWeaponShoulderSheath("physics-shutdown");
        _suppression.nativeGrenadeLeases.clearTracking();
        _twoHandedGrip.reset();
        _equipped.transition.pendingGrip() = {};
        clearProviderInventoryEquip();
        clearPendingForceGrabCommits();
        clearLooseGrenadeRuntimeState();
        clearEquippedWeaponFiringGripInputState();
        _contacts.bodyRuntime.reset();
        _grabInput.shoulderStashStates = {};
        _grabInput.mouthConsumeStates = {};
        _feedbackHaptics.reset();
        native_idle_grip_preharvest::clearHeldWarmups();
        _equipped.transition.shutdown();
        _weaponCollision.shutdown();
        _secondaryWeaponCollision.shutdown();
        _providerSources.clear();
        _bodyBoneColliders.reset();
        _generatedBodyStepDrive.reset();
        _frame.completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _drop.nativeHandoffs = {};
        markGeneratedBodiesInvalidated();
        collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        ::rock::provider::clearExternalBodiesForProviderLoss();
        clearLeftWeaponContact();
        clearRightWeaponContact();
        releaseAllObjects();
        _rightHand.reset();
        _leftHand.reset();

        _lifecycle.cachedBhkWorld = nullptr;
        _lifecycle.cachedHknpWorld = nullptr;
        _layers = {};
        _lifecycle.initialized = false;
        observeLifecycleFrame(nullptr, nullptr, reason);
        _frame.hasPrevPositions = false;
        _handBoneCache.reset();
        _handColliderBoneReader.resetCache();
        _handColliderBoneSnapshot = {};
        _finalPoseBoneReader.resetCache();
        _finalPoseBoneSnapshot = {};
        _grabEvents.heldImpactHapticCooldownUntil.clear();
        _grabEvents.frameCounter = 0;
        _grabInput.mouthConsumeStates = {};
        _feedbackHaptics.reset();
        _grabInput.intentStates = {};
        _grabInput.peerHeldJoinRetryStates = {};
        _grabInput.heldWeaponTriggerEquipIntents = {};
        _forceGrab.committedThisFrame = {};
        _equipped.stashStates = {};
        _equipped.sheathRetrievalStates = {};
        equipped_weapon_shoulder::reset(
            _equipped.shoulderCoordinator);
        _equipped.shoulderGestureConsumedThisFrame = {};
        _grabInput.bareFistGuardState = {};
        _lifecycle.bodyBoneColliderCreateRetryFrames = 0;
        _lifecycle.handColliderCreateRetryFrames = 0;
        _contacts.rightTouch.clear();
        _contacts.leftTouch.clear();
        _contacts.lastHeldImpactPairRight.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _contacts.lastHeldImpactPairLeft.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _contacts.handActivity.reset();
        _contacts.bodyRuntime.reset();
        abandonWeaponHandSuppression();
        _suppression.nativePlayerRefreshFrames = 0;
        _suppression.nativePlayerOverflowLogged = false;

        native_player_collision::abandon();
        cleanupGrabConstraintVtable();

        ROCK_LOG_INFO(Init, "ROCK physics module shut down");
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

        const bool extensionPairsChanged =
            !collision_layer_policy::rockDynamicExtensionPairsMatch(matrix, native_interaction_service::dynamicObstacleLayers());
        const bool retiredWeaponHandCollision =
            native_player_collision::installed();
        const bool retiredWeaponPairsChanged =
            !collision_layer_policy::layerPairSymmetricMatches(matrix,
                collision_layer_policy::ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY,
                collision_layer_policy::FO4_LAYER_WEAPON, retiredWeaponHandCollision) ||
            !collision_layer_policy::layerPairSymmetricMatches(matrix,
                collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY,
                collision_layer_policy::FO4_LAYER_WEAPON, retiredWeaponHandCollision);
        auto mutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        collision_layer_policy::applyRockGeneratedLayerPolicies(
            matrix,
            g_rockConfig.rockHandCollisionStaticWorldEnabled,
            g_rockConfig.rockWeaponCollisionBlocksProjectiles,
            g_rockConfig.rockWeaponCollisionBlocksSpells,
            native_interaction_service::dynamicObstacleLayers(),
            retiredWeaponHandCollision);
        if (extensionPairsChanged || retiredWeaponPairsChanged) {
            _dynamicHandCollision.refreshCollisionFilters(world);
        }
        if (extensionPairsChanged) {
            _dynamicWeaponCollision.refreshCollisionFilter(world);
            _secondaryDynamicWeaponCollision.refreshCollisionFilter(world);
        }
        if (retiredWeaponPairsChanged) {
            ROCK_LOG_INFO(Config, "Retired weapon hand collision: requested={} active={} pairFilter={}",
                true, retiredWeaponHandCollision, native_player_collision::installed());
        }
        _layers = {
            .registered = true,
            .handStaticWorld = g_rockConfig.rockHandCollisionStaticWorldEnabled,
            .weaponBlocksProjectiles = g_rockConfig.rockWeaponCollisionBlocksProjectiles,
            .weaponBlocksSpells = g_rockConfig.rockWeaponCollisionBlocksSpells,
            .extensionObstacleLayers = native_interaction_service::dynamicObstacleLayers(),
        };

        ROCK_LOG_INFO(Config,
            "Registered ROCK collision layers: hand={} mask=0x{:016X}, weapon={} mask=0x{:016X}, reload={} mask=0x{:016X}, body={} mask=0x{:016X}, handStaticWorld={}, projectiles={}, spells={}, extensionObstacleLayers=0x{:016X}",
            collision_layer_policy::ROCK_LAYER_HAND,
            matrix[collision_layer_policy::ROCK_LAYER_HAND],
            collision_layer_policy::ROCK_LAYER_WEAPON,
            matrix[collision_layer_policy::ROCK_LAYER_WEAPON],
            collision_layer_policy::ROCK_LAYER_RELOAD,
            matrix[collision_layer_policy::ROCK_LAYER_RELOAD],
            collision_layer_policy::ROCK_LAYER_BODY,
            matrix[collision_layer_policy::ROCK_LAYER_BODY],
            _layers.handStaticWorld ? "enabled" : "disabled",
            _layers.weaponBlocksProjectiles ? "enabled" : "disabled",
            _layers.weaponBlocksSpells ? "enabled" : "disabled",
            _layers.extensionObstacleLayers);
    }

    bool PhysicsInteraction::createHandCollisions(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!runtime_state::isLocalSkeletonReady()) {
            ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
            return false;
        }

        const RE::NiTransform rightRollAuthorityWorld = getInteractionHandTransform(false);
        const RE::NiTransform leftRollAuthorityWorld = getInteractionHandTransform(true);

        const bool rightOk = _rightHand.createCollision(world, bhkWorld, rightRollAuthorityWorld);

        const bool leftOk = _leftHand.createCollision(world, bhkWorld, leftRollAuthorityWorld);

        if (!rightOk || !leftOk) {
            ROCK_LOG_ERROR(Hand, "Hand collision creation failed (rightOk={}, leftOk={})", rightOk, leftOk);
            if (rightOk)
                _rightHand.destroyCollision(bhkWorld);
            if (leftOk)
                _leftHand.destroyCollision(bhkWorld);
            return false;
        }

        ROCK_LOG_INFO(Hand,
            "Bone-derived hand collision created: rightBodies={} leftBodies={} requireAnchor={} completeFingerSkeleton=true",
            _rightHand.getHandColliderBodyCount(),
            _leftHand.getHandColliderBodyCount(),
            g_rockConfig.rockHandBoneCollidersRequirePalmAnchor ? "true" : "false");

        _lifecycle.handColliderCreateRetryFrames = 0;
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
            ::rock::api::touch::TouchGrabReleaseReason::
                GenerationChanged,
            _lifecycle.collisionGenerationAtomic.load(
                std::memory_order_acquire));
        clearGeneratedBodyContactRegistry();
        _rightHand.destroyCollision(bhkWorld);
        _leftHand.destroyCollision(bhkWorld);
        _handColliderBoneReader.resetCache();
        _handColliderBoneSnapshot = {};
        _finalPoseBoneReader.resetCache();
        _finalPoseBoneSnapshot = {};
        _lifecycle.handColliderCreateRetryFrames = 0;
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

        _lifecycle.bodyBoneColliderCreateRetryFrames = 0;
        _contacts.bodyRuntime.reset();
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
        _contacts.bodyRuntime.reset();
        _lifecycle.bodyBoneColliderCreateRetryFrames = 0;
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
