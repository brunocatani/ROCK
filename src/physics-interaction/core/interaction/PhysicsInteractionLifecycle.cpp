#include "physics-interaction/core/PhysicsInteractionInternal.h"

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

        _equipped.authoredPrimaryFiringGrip.reset("physics-destroyed", _twoHandedGrip);

        if (_lifecycle.initialized) {
            shutdown();
        }
        ROCK_LOG_INFO(Init, "ROCK Physics Module — destroyed");
    }

    void PhysicsInteraction::noteSkeletonLifecycle(std::uint32_t skeletonGeneration, ::rock::provider::RockProviderLifecycleReason reason)
    {
        _equipped.authoredPrimaryFiringGrip.reset("skeleton-lifecycle", _twoHandedGrip);
        physics_lifecycle::noteSkeletonGeneration(_lifecycle.state, skeletonGeneration, reason);
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
        physics_lifecycle::noteProviderGeneration(_lifecycle.state, providerGeneration, reason);
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

    bool PhysicsInteraction::generatedBodiesMatchLifecycle(RE::bhkWorld* bhk, RE::hknpWorld* hknp) const
    {
        return generatedBodiesExistForConfig() &&
               _lifecycle.generatedBodiesBhkWorld == bhk &&
               _lifecycle.generatedBodiesHknpWorld == hknp &&
               _lifecycle.generatedBodiesWorldGeneration != 0 &&
               _lifecycle.generatedBodiesWorldGeneration == _lifecycle.state.worldGeneration &&
               _lifecycle.generatedBodiesSkeletonGeneration == _lifecycle.state.skeletonGeneration &&
               _lifecycle.generatedBodiesProviderGeneration == _lifecycle.state.providerGeneration;
    }

    void PhysicsInteraction::markGeneratedBodiesRebuilt(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
    {
        if (!bhk || !hknp || !generatedBodiesExistForConfig()) {
            markGeneratedBodiesInvalidated();
            return;
        }

        _lifecycle.generatedBodiesBhkWorld = bhk;
        _lifecycle.generatedBodiesHknpWorld = hknp;
        _lifecycle.generatedBodiesWorldGeneration = _lifecycle.state.worldGeneration;
        _lifecycle.generatedBodiesSkeletonGeneration = _lifecycle.state.skeletonGeneration;
        _lifecycle.generatedBodiesProviderGeneration = _lifecycle.state.providerGeneration;
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
            provider::RockProviderTouchGrabReleaseReasonV1::
                GenerationChanged,
            collisionGeneration);
        // Close callback entry and drain any native step already traversing
        // ROCK-owned body banks before clearing registry or wrapper state.
        _generatedBodyStepDrive.reset();
        clearGeneratedBodyContactRegistry();
        const bool generatedWorldStillLive =
            currentBhkWorld &&
            currentBhkWorld == _lifecycle.generatedBodiesBhkWorld &&
            currentHknpWorld &&
            currentHknpWorld == _lifecycle.generatedBodiesHknpWorld;
        if (generatedWorldStillLive) {
            _dynamicHandCollision.retireAll(_lifecycle.generatedBodiesBhkWorld);
            _dynamicWeaponCollision.retireAll(_lifecycle.generatedBodiesBhkWorld);
        } else {
            _dynamicHandCollision.reset();
            _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
        }
        _lifecycle.generatedBodiesBhkWorld = nullptr;
        _lifecycle.generatedBodiesHknpWorld = nullptr;
        _lifecycle.generatedBodiesWorldGeneration = 0;
        _lifecycle.generatedBodiesSkeletonGeneration = 0;
        _lifecycle.generatedBodiesProviderGeneration = 0;
        _lifecycle.state.generatedBodiesValid = false;
        _lifecycle.state.generatedBodiesWorldGeneration = 0;
        _lifecycle.state.generatedBodiesSkeletonGeneration = 0;
        _lifecycle.state.generatedBodiesProviderGeneration = 0;
        _lifecycle.state.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::GeneratedBodiesValid);
        _lifecycle.state.flags &= ~static_cast<std::uint32_t>(::rock::provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        _lifecycle.flagsAtomic.store(_lifecycle.state.flags, std::memory_order_release);
        _lifecycle.stableFrameCountAtomic.store(_lifecycle.state.stableFrameCount, std::memory_order_release);
        _lifecycle.hknpWorldAtomic.store(nullptr, std::memory_order_release);
        _frame.completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _drop.momentumHandoffs = {};
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

        _contacts.generatedBodyRegistry.publish(entries.data(), entryCount);
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
            _lifecycle.state.worldGeneration,
            _lifecycle.state.skeletonGeneration,
            _lifecycle.state.providerGeneration);

        _dynamicWeaponCollision.retireAll(bhk);
        destroyHandCollisions(bhk);
        destroyBodyBoneCollisions(bhk);

        if (!createHandCollisions(hknp, bhk)) {
            ROCK_LOG_WARN(Init, "Generated body lifecycle rebuild failed while creating hand colliders");
            markGeneratedBodiesInvalidated();
            _lifecycle.handColliderCreateRetryFrames = 120;
            return false;
        }

        if (g_rockConfig.rockBodyBoneCollidersEnabled && !createBodyBoneCollisions(hknp, bhk)) {
            ROCK_LOG_WARN(Init, "Generated body lifecycle rebuild continuing without body bone colliders; runtime update will retry");
            _lifecycle.bodyBoneColliderCreateRetryFrames = 120;
        }

        _rightHand.updateCollisionTransform(hknp, getInteractionHandTransform(false), 0.011f);
        _leftHand.updateCollisionTransform(hknp, getInteractionHandTransform(true), 0.011f);
        _bodyBoneColliders.update(hknp, 0.011f);
        _contacts.bodyRuntime.reset();
        markGeneratedBodiesRebuilt(bhk, hknp);
        return generatedBodiesMatchLifecycle(bhk, hknp);
    }

    void PhysicsInteraction::observeLifecycleFrame(RE::bhkWorld* bhk, RE::hknpWorld* hknp, ::rock::provider::RockProviderLifecycleReason reasonHint)
    {
        const auto& runtime = runtime_state::currentFrame();
        physics_lifecycle::FrameInputs inputs{};
        inputs.bhkWorld = reinterpret_cast<std::uintptr_t>(bhk);
        inputs.hknpWorld = reinterpret_cast<std::uintptr_t>(hknp);
        inputs.skeletonGeneration = _lifecycle.state.skeletonGeneration;
        inputs.providerGeneration = _lifecycle.state.providerGeneration;
        inputs.providerReady = _lifecycle.initialized.load(std::memory_order_acquire) && runtime.visualAuthorityAvailable;
        inputs.skeletonReady = runtime.localSkeletonReady;
        inputs.menuBlocking = runtime.localMenuBlocking;
        inputs.configBlocking = runtime.compatibilityConfigBlocking;
        inputs.generatedBodiesValid = generatedBodiesExistForConfig();
        inputs.generatedBodiesWorldGeneration = _lifecycle.generatedBodiesWorldGeneration;
        inputs.generatedBodiesSkeletonGeneration = _lifecycle.generatedBodiesSkeletonGeneration;
        inputs.generatedBodiesProviderGeneration = _lifecycle.generatedBodiesProviderGeneration;
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
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        ROCK_LOG_INFO(Init, "hFRIK off-hand weapon gripping disabled; ROCK is the sole off-hand weapon authority");

        {
            _rightHand.updateCollisionTransform(hknp, getInteractionHandTransform(false), 0.011f);
            _leftHand.updateCollisionTransform(hknp, getInteractionHandTransform(true), 0.011f);
            _bodyBoneColliders.update(hknp, 0.011f);
            ROCK_LOG_INFO(Init, "Initial bone-derived hand collider transforms updated");
        }

        _rightHand.preloadSelectionBeam();
        _leftHand.preloadSelectionBeam();
        (void)_authoredSupportGripIndicator.preload();

        _frame.hasPrevPositions = false;
        _diagnostics.deltaLogCounter = 0;
        _diagnostics.contactLogCounter = 0;
        _contacts.bodyRuntime.reset();
        _contacts.dynamicPushElapsedSeconds = 0.0f;
        _contacts.dynamicPushCooldownUntil.clear();
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
        _equipped.shoulderSheath = {};
        input_remap_runtime::setEquippedWeaponShoulderSheathActive(false);
        _equipped.stashStates = {};
        _equipped.sheathRetrievalStates = {};
        equipped_weapon_shoulder::reset(
            _equipped.shoulderCoordinator);
        _equipped.shoulderGestureConsumedThisFrame = {};
        _grabInput.bareFistGuardState = {};
        _frame.completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _drop.momentumHandoffs = {};
        clearLooseGrenadeRuntimeState();
        _equipped.pendingPrimaryOnlyGripStart = {};
        _equipped.handlingSettings = {};
        _equipped.handlingModeInitialized = false;
        _equipped.handlingModeReconcilePending = false;
        equipped_weapon_handling_runtime::reset();
        clearEquippedWeaponPostDropCollisionSuppressionState();
        _contacts.lastBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastSourceRight.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastSourceLeft.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);
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
        weapon_transition_animation_acceleration::cancel("physics-shutdown");
        debug::ShutdownShapePipeline();
        input_remap_runtime::setRealMeleeWeaponEquipped(false);
        equipped_weapon_handling_runtime::reset();
        _equipped.handlingSettings = {};
        _equipped.handlingModeInitialized = false;
        _equipped.handlingModeReconcilePending = false;
        equipped_weapon_toggle_grab_policy::reset(
            _equipped.toggleGrabState);
        _equipped.toggleGrabReleasePressConsumedThisFrame = {};
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

        if (worldValid) {
            _authoredSupportGripIndicator.shutdown();
            auto* hknp = getHknpWorld(_lifecycle.cachedBhkWorld);
            _dynamicWorldCarCollision.restoreAll(_lifecycle.cachedBhkWorld, hknp, "shutdown");
            _touchGrabRuntime.releaseAll(
                _lifecycle.cachedBhkWorld,
                hknp,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    GenerationChanged,
                _lifecycle.collisionGenerationAtomic.load(
                    std::memory_order_acquire));
            unsubscribeContactEvents(hknp);
            restoreNativePlayerCollisionSuppression(hknp, "shutdown");
            restoreRightHandCollisionAfterDominantWeapon(hknp);
            restoreHandCollisionAfterWeaponSupport(hknp, true, true);
            restoreHandCollisionAfterWeaponSupport(hknp, false, true);
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
            _dynamicWeaponCollision.retireAll(_lifecycle.cachedBhkWorld);
            _weaponCollision.destroyWeaponBody(hknp);
            destroyBodyBoneCollisions(_lifecycle.cachedBhkWorld);
            destroyHandCollisions(_lifecycle.cachedBhkWorld);
        } else {
            _authoredSupportGripIndicator.abandonSceneGraph();
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
            _equipped.transition.abandonSceneGraph();
            _bodyBoneColliders.reset();
            _suppression.rightDominantSuppressed.store(false, std::memory_order_release);
            _suppression.leftWeaponSupportSuppressed.store(false, std::memory_order_release);
            _suppression.rightWeaponSupportSuppressed.store(false, std::memory_order_release);
            _suppression.rightDominantLeases.clearTracking();
            _suppression.leftWeaponSupportLeases.clearTracking();
            _suppression.rightWeaponSupportLeases.clearTracking();
            clearEquippedWeaponPostDropCollisionSuppressionState();
            _suppression.nativePlayerBodies = {};
            _suppression.nativePlayerBodyCount = 0;
            _suppression.nativePlayerRefreshFrames = 0;
            _suppression.nativePlayerOverflowLogged = false;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        clearEquippedWeaponShoulderSheath("physics-shutdown");
        _twoHandedGrip.reset();
        _equipped.pendingPrimaryOnlyGripStart = {};
        clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin::ProviderForceGrabCommand);
        clearLooseGrenadeRuntimeState();
        clearEquippedWeaponFiringGripInputState();
        _contacts.bodyRuntime.reset();
        _grabInput.shoulderStashStates = {};
        _grabInput.mouthConsumeStates = {};
        _feedbackHaptics.reset();
        _equipped.transition.shutdown();
        _weaponCollision.shutdown();
        _bodyBoneColliders.reset();
        _generatedBodyStepDrive.reset();
        _frame.completedPhysicsSolveSequence.store(0, std::memory_order_release);
        _drop.momentumHandoffs = {};
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
        _layers.registered = false;
        _layers.expectedHandMask = 0;
        _layers.expectedWeaponMask = 0;
        _layers.expectedReloadMask = 0;
        _layers.expectedBodyMask = 0;
        _layers.expectedDynamicHandProxyMask = 0;
        _layers.expectedDynamicLeftHandProxyMask = 0;
        _layers.expectedDynamicWeaponProxyMask = 0;
        _layers.expectedDynamicWorldCarClutterMask = 0;
        _layers.expectedDynamicWorldCarLargeClutterMask = 0;
        _layers.originalNativeCharacterControllerMask = 0;
        _layers.expectedNativeCharacterControllerMask = 0;
        _layers.nativeControllerPolicyCaptured = false;
        _layers.nativeControllerPolicyEnabled = false;
        _lifecycle.initialized = false;
        observeLifecycleFrame(nullptr, nullptr, reason);
        _frame.hasPrevPositions = false;
        _diagnostics.heldMassLogCounter = 0;
        _handBoneCache.reset();
        _diagnostics.handCacheResolveLogCounter = 0;
        _diagnostics.paritySummaryCounter = 0;
        _diagnostics.parityEnabledLogged = false;
        _diagnostics.runtimeScaleLogged = false;
        _diagnostics.rawHandParityStates = {};
        _contacts.dynamicPushCooldownUntil.clear();
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
        _contacts.lastBodyRight.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastBodyLeft.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastSourceRight.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastSourceLeft.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastBodyWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastSourceWeapon.store(0xFFFFFFFF, std::memory_order_release);
        _contacts.lastHeldImpactPairRight.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _contacts.lastHeldImpactPairLeft.store(INVALID_HELD_IMPACT_PAIR, std::memory_order_release);
        _contacts.handActivity.reset();
        _contacts.bodyRuntime.reset();
        _suppression.rightDominantLeases.clearTracking();
        _suppression.leftWeaponSupportLeases.clearTracking();
        _suppression.rightWeaponSupportLeases.clearTracking();
        _suppression.rightDominantSuppressed.store(false, std::memory_order_release);
        _suppression.leftWeaponSupportSuppressed.store(false, std::memory_order_release);
        _suppression.rightWeaponSupportSuppressed.store(false, std::memory_order_release);
        clearEquippedWeaponPostDropCollisionSuppressionState();
        _suppression.nativePlayerBodies = {};
        _suppression.nativePlayerBodyCount = 0;
        _suppression.nativePlayerRefreshFrames = 0;
        _suppression.nativePlayerOverflowLogged = false;

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

        if (!_layers.nativeControllerPolicyCaptured) {
            _layers.originalNativeCharacterControllerMask = matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER];
            _layers.nativeControllerPolicyCaptured = true;
        }

        collision_layer_policy::applyRockGeneratedLayerPolicies(
            matrix,
            g_rockConfig.rockHandCollisionStaticWorldEnabled,
            g_rockConfig.rockWeaponCollisionBlocksProjectiles,
            g_rockConfig.rockWeaponCollisionBlocksSpells);
        collision_layer_policy::applyNativeCharacterControllerObjectSuppressionPolicy(
            matrix,
            g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled,
            _layers.originalNativeCharacterControllerMask);

        _layers.expectedHandMask = collision_layer_policy::buildRockHandExpectedMask(true, g_rockConfig.rockHandCollisionStaticWorldEnabled);
        _layers.expectedWeaponMask =
            collision_layer_policy::buildRockWeaponExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                true);
        _layers.expectedReloadMask =
            collision_layer_policy::buildRockReloadExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockHandCollisionStaticWorldEnabled);
        _layers.expectedBodyMask = collision_layer_policy::buildRockBodyExpectedMask();
        _layers.expectedDynamicHandProxyMask =
            collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                false);
        _layers.expectedDynamicLeftHandProxyMask =
            collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                true);
        _layers.expectedDynamicWeaponProxyMask =
            collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask();
        _layers.expectedDynamicWorldCarClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER];
        _layers.expectedDynamicWorldCarLargeClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER];
        _layers.expectedNativeCharacterControllerMask =
            collision_layer_policy::nativeCharacterControllerExpectedMask(
                _layers.originalNativeCharacterControllerMask,
                g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled);
        _layers.nativeControllerPolicyEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
        _layers.registered = true;

        const bool nativeControllerObjectPairsMatch =
            collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix, _layers.expectedNativeCharacterControllerMask);
        const char* nativeControllerObjectStatus =
            _layers.nativeControllerPolicyEnabled ?
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
                collision_layer_policy::maskEnablesLayer(_layers.expectedHandMask, collision_layer_policy::FO4_LAYER_BIPED)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_BIPED,
                        collision_layer_policy::maskEnablesLayer(_layers.expectedWeaponMask, collision_layer_policy::FO4_LAYER_BIPED)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_DEADBIP,
                collision_layer_policy::maskEnablesLayer(_layers.expectedHandMask, collision_layer_policy::FO4_LAYER_DEADBIP)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_DEADBIP,
                        collision_layer_policy::maskEnablesLayer(_layers.expectedWeaponMask, collision_layer_policy::FO4_LAYER_DEADBIP)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::FO4_LAYER_BIPED_NO_CC,
                collision_layer_policy::maskEnablesLayer(_layers.expectedHandMask, collision_layer_policy::FO4_LAYER_BIPED_NO_CC)) &&
                    collision_layer_policy::layerPairSymmetricMatches(
                        matrix,
                        collision_layer_policy::ROCK_LAYER_WEAPON,
                        collision_layer_policy::FO4_LAYER_BIPED_NO_CC,
                        collision_layer_policy::maskEnablesLayer(_layers.expectedWeaponMask, collision_layer_policy::FO4_LAYER_BIPED_NO_CC)) ?
                "ok" :
                "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_HAND,
                collision_layer_policy::maskEnablesLayer(_layers.expectedBodyMask, collision_layer_policy::ROCK_LAYER_HAND)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_WEAPON,
                collision_layer_policy::maskEnablesLayer(_layers.expectedBodyMask, collision_layer_policy::ROCK_LAYER_WEAPON)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::maskEnablesLayer(_layers.expectedBodyMask, collision_layer_policy::ROCK_LAYER_BODY)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_STATIC,
                collision_layer_policy::maskEnablesLayer(_layers.expectedBodyMask, collision_layer_policy::FO4_LAYER_STATIC)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_ANIMSTATIC,
                collision_layer_policy::maskEnablesLayer(_layers.expectedBodyMask, collision_layer_policy::FO4_LAYER_ANIMSTATIC)) ? "ok" : "bad",
            collision_layer_policy::layerPairSymmetricMatches(
                matrix,
                collision_layer_policy::ROCK_LAYER_BODY,
                collision_layer_policy::FO4_LAYER_CLUTTER,
                collision_layer_policy::maskEnablesLayer(_layers.expectedBodyMask, collision_layer_policy::FO4_LAYER_CLUTTER)) ? "ok" : "bad",
            !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::ROCK_LAYER_BODY, collision_layer_policy::FO4_LAYER_ITEMPICK) &&
                    !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::FO4_LAYER_ITEMPICK, collision_layer_policy::ROCK_LAYER_BODY) ?
                "ok" :
                "bad",
            !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::ROCK_LAYER_BODY, collision_layer_policy::FO4_LAYER_CHARCONTROLLER) &&
                    !collision_layer_policy::layerPairEnabledFromRow(matrix, collision_layer_policy::FO4_LAYER_CHARCONTROLLER, collision_layer_policy::ROCK_LAYER_BODY) ?
                "ok" :
                "bad",
            g_rockConfig.rockHandCollisionStaticWorldEnabled ? "enabled" : "disabled",
            "enabled",
            "enabled",
            g_rockConfig.rockWeaponCollisionBlocksProjectiles ? "enabled" : "disabled",
            g_rockConfig.rockWeaponCollisionBlocksSpells ? "enabled" : "disabled",
            nativeControllerObjectStatus);
        ROCK_LOG_INFO(
            Config,
            "Registered dynamic weapon proxy layer={} worldOnlyMask=0x{:016X}",
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY,
            collision_layer_policy::matrixAddressableMask(
                _layers.expectedDynamicWeaponProxyMask));
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
            provider::RockProviderTouchGrabReleaseReasonV1::
                GenerationChanged,
            _lifecycle.collisionGenerationAtomic.load(
                std::memory_order_acquire));
        clearGeneratedBodyContactRegistry();
        _rightHand.destroyCollision(bhkWorld);
        _leftHand.destroyCollision(bhkWorld);
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
