/*
 * Contact routing is kept as a separate core fragment because it bridges the native hknp contact signal, hand semantic state, weapon contacts, and push assist. Keeping it in the PhysicsInteraction translation unit preserves the existing anonymous-namespace helpers while making the frame loop readable.
 */
    void PhysicsInteraction::resolveContacts(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::ContactResolve);

        auto* bhk = frame.bhkWorld;
        auto* hknp = frame.hknpWorld;
        auto rightContactBody = _lastContactBodyRight.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        auto rightSourceBody = _lastContactSourceRight.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (rightContactBody != 0xFFFFFFFF) {
            resolveAndLogContact("Right", bhk, hknp, RE::hknpBodyId{ rightContactBody });
            if (rightSourceBody == 0xFFFFFFFF) {
                rightSourceBody = _rightHand.getCollisionBodyId().value;
            }
            applyDynamicPushAssist("Right", bhk, hknp, rightSourceBody, rightContactBody, false, &_rightHand);
        }

        auto leftContactBody = _lastContactBodyLeft.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        auto leftSourceBody = _lastContactSourceLeft.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (leftContactBody != 0xFFFFFFFF) {
            resolveAndLogContact("Left", bhk, hknp, RE::hknpBodyId{ leftContactBody });
            if (leftSourceBody == 0xFFFFFFFF) {
                leftSourceBody = _leftHand.getCollisionBodyId().value;
            }
            applyDynamicPushAssist("Left", bhk, hknp, leftSourceBody, leftContactBody, false, &_leftHand);
        }

        auto weaponContactBody = _lastContactBodyWeapon.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        auto weaponSourceBody = _lastContactSourceWeapon.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
        if (weaponContactBody != 0xFFFFFFFF && weaponSourceBody != 0xFFFFFFFF) {
            applyDynamicPushAssist("Weapon", bhk, hknp, weaponSourceBody, weaponContactBody, true);
        }

        auto readBodyMass = [](RE::hknpWorld* world, std::uint32_t bodyId) {
            if (!world || bodyId == 0xFFFFFFFF || bodyId == object_physics_body_set::INVALID_BODY_ID) {
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
        };

        auto readBodySpeedGameUnits = [](RE::hknpWorld* world, std::uint32_t bodyId) {
            if (!world || bodyId == 0xFFFFFFFF || bodyId == object_physics_body_set::INVALID_BODY_ID) {
                return 0.0f;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
            if (!motion) {
                return 0.0f;
            }

            const float speedHavok = std::sqrt(
                motion->linearVelocity.x * motion->linearVelocity.x +
                motion->linearVelocity.y * motion->linearVelocity.y +
                motion->linearVelocity.z * motion->linearVelocity.z);
            return std::isfinite(speedHavok) ? speedHavok * havokToGameScale() : 0.0f;
        };

        auto processHeldImpact = [&](Hand& hand,
                                     bool isLeft,
                                     std::atomic<std::uint64_t>& pairAtomic) {
            std::uint32_t heldBody = kInvalidAtomicBodyId;
            std::uint32_t otherBody = kInvalidAtomicBodyId;
            const auto packedPair = pairAtomic.exchange(kInvalidHeldImpactPair, std::memory_order_acq_rel);
            if (!unpackHeldImpactPair(packedPair, heldBody, otherBody) || !hand.isHolding()) {
                return;
            }

            dispatchHeldImpactGrabEvent(
                isLeft,
                hand.getHeldRef(),
                heldBody,
                otherBody,
                readBodyMass(hknp, heldBody),
                readBodySpeedGameUnits(hknp, heldBody));
        };

        processHeldImpact(_rightHand, false, _lastHeldImpactPairRight);
        processHeldImpact(_leftHand, true, _lastHeldImpactPairLeft);
    }
    void PhysicsInteraction::applyDynamicPushAssist(const char* sourceName,
        RE::bhkWorld* bhk,
        RE::hknpWorld* hknp,
        std::uint32_t sourceBodyId,
            std::uint32_t targetBodyId,
        bool sourceIsWeapon,
        const Hand* sourceHand)
    {
        if (!bhk || !hknp || sourceBodyId == 0xFFFFFFFF || targetBodyId == 0xFFFFFFFF ||
            sourceBodyId == object_physics_body_set::INVALID_BODY_ID || targetBodyId == object_physics_body_set::INVALID_BODY_ID || sourceBodyId == targetBodyId) {
            return;
        }
        if (::rock::provider::isExternalBodyDynamicPushSuppressed(targetBodyId)) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} is registered as external suppressing ROCK dynamic push",
                sourceName,
                targetBodyId);
            return;
        }
        if (held_object_body_set_policy::containsAnyBody(_rightHand.getHeldBodyIds(), _leftHand.getHeldBodyIds(), targetBodyId)) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: target body {} is owned by an active held grab",
                sourceName,
                targetBodyId);
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
        if (!sourceIsWeapon && sourceHand) {
            scanOptions.heldBySameHand = &sourceHand->getHeldBodyIds();
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
        if (!sourceIsWeapon && collision_layer_policy::isActorOrBipedLayer(targetRecord->collisionLayer)) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push actor/ragdoll accepted: sourceBody={} targetBody={} layer={} motionId={} motionType={}",
                sourceName,
                sourceBodyId,
                targetBodyId,
                targetRecord->collisionLayer,
                targetRecord->motionId,
                static_cast<int>(targetRecord->motionType));
        }

        const auto uniqueMotionRecords = bodySet.uniqueAcceptedMotionRecords();
        if (uniqueMotionRecords.empty()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push skipped: accepted target body {} produced no unique motion bodies",
                sourceName,
                targetBodyId);
            return;
        }

        auto* sourceMotion = havok_runtime::getBodyMotion(hknp, RE::hknpBodyId{ sourceBodyId });
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
        for (const auto* record : uniqueMotionRecords) {
            if (!record) {
                continue;
            }
            physics_recursive_wrappers::activateBody(hknp, record->bodyId);
            if (push_assist::applyLinearImpulse(record->collisionObject, push.impulse)) {
                ++appliedCount;
            }
        }

        if (appliedCount > 0) {
            _dynamicPushCooldownUntil[cooldownKey] =
                _dynamicPushElapsedSeconds + (std::max)(0.0f, g_rockConfig.rockDynamicPushCooldownSeconds);
            auto* baseObj = targetRef->GetObjectReference();
            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const std::string nameStr = objName.empty() ? std::string("(unnamed)") : std::string(objName);
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} dynamic push applied: '{}' formID={:08X} targetBody={} layer={} acceptedBodies={} uniqueMotions={} impulse=({:.3f},{:.3f},{:.3f})",
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

        std::uint32_t filterInfo = 0;
        if (!havok_runtime::tryReadFilterInfo(hknp, bodyId, filterInfo)) {
            return;
        }
        auto layer = filterInfo & 0x7F;

        auto* ref = resolveBodyToRef(bhk, hknp, bodyId);
        if (ref) {
            auto* baseObj = ref->GetObjectReference();
            const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";
            auto objName = baseObj ? RE::TESFullName::GetFullName(*baseObj, false) : std::string_view{};
            const std::string nameStr = objName.empty() ? std::string("(unnamed)") : std::string(objName);

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
        if (!world) {
            ROCK_LOG_ERROR(Init, "Contact event subscription skipped because world is null");
            return;
        }

        void* signal = world->GetEventSignal(RE::hknpEventType::kContact);
        if (!signal) {
            ROCK_LOG_ERROR(Init, "Failed to get contact event signal");
            return;
        }

        auto* currentWorld = s_contactEventBridge.world.load(std::memory_order_acquire);
        auto* currentSignal = s_contactEventBridge.signal.load(std::memory_order_acquire);
        const auto currentSnapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
            .world = reinterpret_cast<std::uintptr_t>(currentWorld),
            .signal = reinterpret_cast<std::uintptr_t>(currentSignal),
            .active = currentWorld != nullptr && currentSignal != nullptr,
        };
        const auto plan = contact_signal_subscription_policy::planSubscription(
            currentSnapshot,
            reinterpret_cast<std::uintptr_t>(world),
            reinterpret_cast<std::uintptr_t>(signal),
            s_contactEventBridge.hasRetainedNativeSlot(world, signal));

        if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::IgnoreNullSignal) {
            ROCK_LOG_ERROR(Init, "Contact event subscription skipped because world or signal is null");
            return;
        }

        if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::AlreadySubscribed) {
            _contactEventSignal.store(signal, std::memory_order_release);
            _contactEventWorld.store(world, std::memory_order_release);
            s_contactEventBridge.signal.store(signal, std::memory_order_release);
            s_contactEventBridge.world.store(world, std::memory_order_release);
            s_contactEventBridge.instance.store(this, std::memory_order_release);
            const auto epoch = s_contactEventBridge.subscriptionEpoch.load(std::memory_order_acquire);
            if (!s_contactEventBridge.rememberRetainedNativeSlot(world, signal, epoch)) {
                ROCK_LOG_WARN(Init, "Contact event retained-slot table full while reusing bridge slot; future duplicate suppression may be degraded");
            }
            ROCK_LOG_DEBUG(Init, "Contact event signal already subscribed for current world; reusing native bridge slot");
            return;
        }

        if (plan.replaceExistingRuntimeStateWithoutUnsubscribe) {
            ROCK_LOG_INFO(
                Init,
                "Replacing contact event bridge state without native unsubscribe (action={})",
                static_cast<std::uint32_t>(plan.action));
        }

        ContactEventCallbackInfo cbInfo{};
        cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
        cbInfo.ctx = 0;

        typedef void subscribe_ext_t(void* signal, void* userData, void* callbackInfo);
        static REL::Relocation<subscribe_ext_t> subscribeExt{ REL::Offset(offsets::kFunc_SubscribeContactEvent) };
        subscribeExt(signal, static_cast<void*>(&s_contactEventBridge), &cbInfo);

        _contactEventSignal.store(signal, std::memory_order_release);
        _contactEventWorld.store(world, std::memory_order_release);
        s_contactEventBridge.signal.store(signal, std::memory_order_release);
        s_contactEventBridge.world.store(world, std::memory_order_release);
        s_contactEventBridge.instance.store(this, std::memory_order_release);
        const auto epoch = s_contactEventBridge.subscriptionEpoch.fetch_add(1, std::memory_order_acq_rel) + 1;
        if (!s_contactEventBridge.rememberRetainedNativeSlot(world, signal, epoch)) {
            ROCK_LOG_WARN(Init, "Contact event retained-slot table full after native subscription; future duplicate suppression may be degraded");
        }
        ROCK_LOG_INFO(
            Init,
            "Subscribed contact event bridge slot (epoch={}, action={})",
            epoch,
            static_cast<std::uint32_t>(plan.action));
    }

    void PhysicsInteraction::unsubscribeContactEvents(RE::hknpWorld* liveWorld)
    {
        auto* localWorld = _contactEventWorld.exchange(nullptr, std::memory_order_acq_rel);
        void* localSignal = _contactEventSignal.exchange(nullptr, std::memory_order_acq_rel);

        auto* expectedInstance = this;
        const bool deactivatedCurrentInstance = s_contactEventBridge.instance.compare_exchange_strong(
            expectedInstance,
            nullptr,
            std::memory_order_acq_rel,
            std::memory_order_acquire);

        auto* bridgeWorld = s_contactEventBridge.world.load(std::memory_order_acquire);
        void* bridgeSignal = s_contactEventBridge.signal.load(std::memory_order_acquire);
        const auto bridgeSnapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
            .world = reinterpret_cast<std::uintptr_t>(bridgeWorld),
            .signal = reinterpret_cast<std::uintptr_t>(bridgeSignal),
            .active = bridgeWorld != nullptr && bridgeSignal != nullptr,
        };

        if (!contact_signal_subscription_policy::isActiveSubscription(bridgeSnapshot)) {
            return;
        }

        const bool retainNativeSlot = contact_signal_subscription_policy::shouldRetainNativeSlotAfterDeactivation(
            bridgeSnapshot);
        if (retainNativeSlot) {
            ROCK_LOG_INFO(
                Init,
                "Deactivated contact event bridge; native slots retained for hknpWorld cleanup (instanceCleared={}, world={}, signal={}, liveWorld={})",
                deactivatedCurrentInstance ? "yes" : "no",
                static_cast<const void*>(bridgeWorld),
                bridgeSignal,
                static_cast<const void*>(liveWorld));
            return;
        }

        ROCK_LOG_INFO(
            Init,
            "Deactivated contact event bridge with no active native slot (instanceCleared={}, localWorld={}, localSignal={}, liveWorld={})",
            deactivatedCurrentInstance ? "yes" : "no",
            static_cast<const void*>(localWorld),
            localSignal,
            static_cast<const void*>(liveWorld));
    }

    void PhysicsInteraction::onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData)
    {
        performance_profiler::addEventCount(performance_profiler::Scope::NativeContactCallback);
        onContactCallbackSeh(userData, worldPtrHolder, contactEventData);
    }

    void PhysicsInteraction::onContactCallbackSeh(void* userData, void** worldPtrHolder, void* contactEventData)
    {
        __try {
            onContactCallbackUnsafe(userData, worldPtrHolder, contactEventData);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            onContactCallbackException();
        }
    }

    void PhysicsInteraction::onContactCallbackUnsafe(void* userData, void** worldPtrHolder, void* contactEventData)
    {
        if (!s_hooksEnabled.load(std::memory_order_acquire))
            return;
        if (userData != static_cast<void*>(&s_contactEventBridge)) {
            return;
        }

        auto* bridge = static_cast<ContactEventSubscriptionBridge*>(userData);
        auto* self = bridge->instance.load(std::memory_order_acquire);
        if (self && self->_initialized.load(std::memory_order_acquire)) {
            auto* subscribedWorld = bridge->world.load(std::memory_order_acquire);
            auto* subscribedSignal = bridge->signal.load(std::memory_order_acquire);
            const auto snapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
                .world = reinterpret_cast<std::uintptr_t>(subscribedWorld),
                .signal = reinterpret_cast<std::uintptr_t>(subscribedSignal),
                .active = subscribedWorld != nullptr && subscribedSignal != nullptr,
            };

            std::uintptr_t callbackWorld = 0;
            if (worldPtrHolder) {
                callbackWorld = reinterpret_cast<std::uintptr_t>(*worldPtrHolder);
            }

            const auto acceptance = contact_signal_subscription_policy::evaluateCallbackAcceptance(snapshot, callbackWorld);
            if (!acceptance.accept) {
                return;
            }

            self->handleContactEvent(reinterpret_cast<RE::hknpWorld*>(acceptance.effectiveWorld), contactEventData);
        }
    }

    void PhysicsInteraction::onContactCallbackException()
    {
        static int sehLogCounter = 0;
        if (sehLogCounter++ % 100 == 0) {
            logger::error(
                "[ROCK::Contact] SEH exception caught on physics thread (count={}) — "
                "likely stale world during cell transition",
                sehLogCounter);
        }
        s_hooksEnabled.store(false, std::memory_order_release);
    }

    void PhysicsInteraction::handleContactEvent(RE::hknpWorld* world, void* contactEventData)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::NativeContactCallback);

        if (!contactEventData)
            return;

        auto* data = reinterpret_cast<std::uint8_t*>(contactEventData);
        std::uint32_t bodyIdA = *reinterpret_cast<std::uint32_t*>(data + 0x08);
        std::uint32_t bodyIdB = *reinterpret_cast<std::uint32_t*>(data + 0x0C);

        if (!contact_pipeline_policy::isValidBodyId(bodyIdA) || !contact_pipeline_policy::isValidBodyId(bodyIdB) || bodyIdA == bodyIdB) {
            return;
        }

        if (!havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdA }) ||
            !havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdB })) {
            return;
        }

        havok_runtime::ContactSignalPointResult rawContactPoint{};
        bool rawContactPointEvaluated = false;
        bool hasRawContactPoint = false;
        auto ensureRawContactPoint = [&]() {
            if (!rawContactPointEvaluated) {
                hasRawContactPoint = havok_runtime::tryExtractContactSignalPoint(world, contactEventData, rawContactPoint);
                rawContactPointEvaluated = true;
            }
            return hasRawContactPoint;
        };

        const auto rightId = _rightHand.getCollisionBodyId().value;
        const auto leftId = _leftHand.getCollisionBodyId().value;

        struct HandContactSource
        {
            bool valid = false;
            bool isLeft = false;
            bool primaryAnchor = false;
            HandColliderBodyMetadata metadata{};
        };

        auto classifyHandBody = [](const Hand& hand, std::uint32_t bodyId, bool isLeft, std::uint32_t anchorId) {
            HandContactSource source{};
            HandColliderBodyMetadata metadata{};
            if (hand.tryGetHandColliderMetadata(bodyId, metadata)) {
                source.valid = true;
                source.isLeft = isLeft;
                source.primaryAnchor = metadata.primaryPalmAnchor || bodyId == anchorId;
                source.metadata = metadata;
                source.metadata.isLeft = isLeft;
                return source;
            }

            if (bodyId == anchorId && bodyId != INVALID_BODY_ID) {
                source.valid = true;
                source.isLeft = isLeft;
                source.primaryAnchor = true;
                source.metadata.valid = true;
                source.metadata.isLeft = isLeft;
                source.metadata.primaryPalmAnchor = true;
                source.metadata.bodyId = bodyId;
                source.metadata.role = hand_collider_semantics::HandColliderRole::PalmAnchor;
            }
            return source;
        };

        const auto bodyARight = classifyHandBody(_rightHand, bodyIdA, false, rightId);
        const auto bodyBRight = classifyHandBody(_rightHand, bodyIdB, false, rightId);
        const auto bodyALeft = classifyHandBody(_leftHand, bodyIdA, true, leftId);
        const auto bodyBLeft = classifyHandBody(_leftHand, bodyIdB, true, leftId);
        const bool bodyAIsRight = bodyARight.valid;
        const bool bodyBIsRight = bodyBRight.valid;
        const bool bodyAIsLeft = bodyALeft.valid;
        const bool bodyBIsLeft = bodyBLeft.valid;
        const bool bodyAIsExternal = ::rock::provider::isExternalBodyId(bodyIdA);
        const bool bodyBIsExternal = ::rock::provider::isExternalBodyId(bodyIdB);
        const bool bodyAIsRightHeld = _rightHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsRightHeld = _rightHand.isHeldBodyId(bodyIdB);
        const bool bodyAIsLeftHeld = _leftHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsLeftHeld = _leftHand.isHeldBodyId(bodyIdB);
        const bool bodyAIsWeapon = _weaponCollision.isWeaponBodyIdAtomic(bodyIdA);
        const bool bodyBIsWeapon = _weaponCollision.isWeaponBodyIdAtomic(bodyIdB);
        BodyBoneColliderMetadata bodyABodyMetadata{};
        BodyBoneColliderMetadata bodyBBodyMetadata{};
        const bool bodyAIsBody = _bodyBoneColliders.tryGetBodyMetadataAtomic(bodyIdA, bodyABodyMetadata);
        const bool bodyBIsBody = _bodyBoneColliders.tryGetBodyMetadataAtomic(bodyIdB, bodyBBodyMetadata);
        const bool bodyAIsRockSource = bodyAIsRight || bodyAIsLeft || bodyAIsRightHeld || bodyAIsLeftHeld || bodyAIsWeapon || bodyAIsBody;
        const bool bodyBIsRockSource = bodyBIsRight || bodyBIsLeft || bodyBIsRightHeld || bodyBIsLeftHeld || bodyBIsWeapon || bodyBIsBody;

        if (contact_pipeline_policy::shouldSkipContactSignalBeforeLayerRead(contact_pipeline_policy::ContactSignalPrefilter{
                .bodyIdA = bodyIdA,
                .bodyIdB = bodyIdB,
                .bodyAIsRockSource = bodyAIsRockSource,
                .bodyBIsRockSource = bodyBIsRockSource,
            })) {
            return;
        }

        auto readBodyFilterInfo = [world](std::uint32_t bodyId) {
            std::uint32_t filterInfo = 0;
            if (world && havok_runtime::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, filterInfo)) {
                return filterInfo;
            }
            return contact_evidence::kUnknownFilterInfo;
        };

        auto filterInfoToLayer = [](std::uint32_t filterInfo) {
            return filterInfo == contact_evidence::kUnknownFilterInfo ? contact_pipeline_policy::kUnknownLayer : (filterInfo & 0x7Fu);
        };

        const std::uint32_t bodyAFilterInfo = readBodyFilterInfo(bodyIdA);
        const std::uint32_t bodyBFilterInfo = readBodyFilterInfo(bodyIdB);
        const std::uint32_t bodyALayer = filterInfoToLayer(bodyAFilterInfo);
        const std::uint32_t bodyBLayer = filterInfoToLayer(bodyBFilterInfo);

        auto makeEndpoint = [&](std::uint32_t bodyId, std::uint32_t layer, bool isRightHand, bool isLeftHand, bool isWeapon, bool isRightHeld, bool isLeftHeld, bool isBody, bool isExternal) {
            using contact_pipeline_policy::ContactEndpoint;
            using contact_pipeline_policy::ContactEndpointKind;

            ContactEndpoint endpoint{};
            endpoint.bodyId = bodyId;
            endpoint.layer = layer;
            if (isRightHand) {
                endpoint.kind = ContactEndpointKind::RightHand;
            } else if (isLeftHand) {
                endpoint.kind = ContactEndpointKind::LeftHand;
            } else if (isWeapon) {
                endpoint.kind = ContactEndpointKind::Weapon;
            } else if (isRightHeld) {
                endpoint.kind = ContactEndpointKind::RightHeldObject;
            } else if (isLeftHeld) {
                endpoint.kind = ContactEndpointKind::LeftHeldObject;
            } else if (isBody) {
                endpoint.kind = ContactEndpointKind::Body;
            } else if (isExternal) {
                endpoint.kind = ContactEndpointKind::External;
            } else {
                endpoint.kind = contact_pipeline_policy::classifyNonRockLayer(layer);
            }
            return endpoint;
        };

        const auto endpointA = makeEndpoint(bodyIdA, bodyALayer, bodyAIsRight, bodyAIsLeft, bodyAIsWeapon, bodyAIsRightHeld, bodyAIsLeftHeld, bodyAIsBody, bodyAIsExternal);
        const auto endpointB = makeEndpoint(bodyIdB, bodyBLayer, bodyBIsRight, bodyBIsLeft, bodyBIsWeapon, bodyBIsRightHeld, bodyBIsLeftHeld, bodyBIsBody, bodyBIsExternal);
        const auto contactRoute = contact_pipeline_policy::classifyContact(endpointA, endpointB);

        auto handSourceFor = [&](std::uint32_t bodyId) -> const HandContactSource* {
            if (bodyARight.valid && bodyARight.metadata.bodyId == bodyId) {
                return &bodyARight;
            }
            if (bodyBRight.valid && bodyBRight.metadata.bodyId == bodyId) {
                return &bodyBRight;
            }
            if (bodyALeft.valid && bodyALeft.metadata.bodyId == bodyId) {
                return &bodyALeft;
            }
            if (bodyBLeft.valid && bodyBLeft.metadata.bodyId == bodyId) {
                return &bodyBLeft;
            }
            return nullptr;
        };

        auto bodySourceFor = [&](std::uint32_t bodyId) -> const BodyBoneColliderMetadata* {
            if (bodyABodyMetadata.valid && bodyABodyMetadata.bodyId == bodyId) {
                return &bodyABodyMetadata;
            }
            if (bodyBBodyMetadata.valid && bodyBBodyMetadata.bodyId == bodyId) {
                return &bodyBBodyMetadata;
            }
            return nullptr;
        };

        auto fillSourceVelocity = [&](std::uint32_t sourceBodyId,
                                      ::rock::provider::RockProviderExternalSourceKind sourceKind,
                                      const HandColliderBodyMetadata* handMetadata,
                                      ::rock::provider::RockProviderExternalContactV2& contact) {
            if (handMetadata && handMetadata->valid && handMetadata->hasSampledLinearVelocityHavok &&
                havok_runtime::isFinite3(handMetadata->sampledLinearVelocityHavok)) {
                std::copy_n(handMetadata->sampledLinearVelocityHavok, 4, contact.sourceVelocityHavok);
                return;
            }

            if (sourceKind == ::rock::provider::RockProviderExternalSourceKind::Weapon) {
                float sampledWeaponVelocity[4]{};
                if (_weaponCollision.tryGetWeaponBodySampledVelocityAtomic(sourceBodyId, sampledWeaponVelocity) &&
                    havok_runtime::isFinite3(sampledWeaponVelocity)) {
                    std::copy_n(sampledWeaponVelocity, 4, contact.sourceVelocityHavok);
                    return;
                }
            }

            if (!world || sourceBodyId == INVALID_CONTACT_BODY_ID) {
                return;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ sourceBodyId });
            if (!motion) {
                return;
            }

            contact.sourceVelocityHavok[0] = motion->linearVelocity.x;
            contact.sourceVelocityHavok[1] = motion->linearVelocity.y;
            contact.sourceVelocityHavok[2] = motion->linearVelocity.z;
        };

        auto tryFillAggregateContactPoint = [world](std::uint32_t sourceBodyId,
                                                    std::uint32_t externalBodyId,
                                                    ::rock::provider::RockProviderExternalContactV2& contact) {
            if (!world || sourceBodyId == INVALID_CONTACT_BODY_ID || externalBodyId == INVALID_CONTACT_BODY_ID) {
                return false;
            }

            RE::NiTransform sourceTransform{};
            RE::NiTransform targetTransform{};
            if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ sourceBodyId }, sourceTransform) ||
                !havok_runtime::tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ externalBodyId }, targetTransform)) {
                return false;
            }

            if (!std::isfinite(sourceTransform.translate.x) || !std::isfinite(sourceTransform.translate.y) || !std::isfinite(sourceTransform.translate.z) ||
                !std::isfinite(targetTransform.translate.x) || !std::isfinite(targetTransform.translate.y) || !std::isfinite(targetTransform.translate.z)) {
                return false;
            }

            const float scale = gameToHavokScale();
            contact.contactPointHavok[0] = targetTransform.translate.x * scale;
            contact.contactPointHavok[1] = targetTransform.translate.y * scale;
            contact.contactPointHavok[2] = targetTransform.translate.z * scale;
            contact.contactPointHavok[3] = 0.0f;

            const float dx = (targetTransform.translate.x - sourceTransform.translate.x) * scale;
            const float dy = (targetTransform.translate.y - sourceTransform.translate.y) * scale;
            const float dz = (targetTransform.translate.z - sourceTransform.translate.z) * scale;
            const float lenSq = dx * dx + dy * dy + dz * dz;
            if (std::isfinite(lenSq) && lenSq > 0.000001f) {
                const float invLen = 1.0f / std::sqrt(lenSq);
                contact.contactNormalHavok[0] = dx * invLen;
                contact.contactNormalHavok[1] = dy * invLen;
                contact.contactNormalHavok[2] = dz * invLen;
            }

            contact.contactPointWeightSum = 0.0f;
            contact.quality = ::rock::provider::RockProviderExternalContactQuality::AggregateImpulse;
            return true;
        };

        auto publishExternalContact = [&](std::uint32_t sourceBodyId,
                                          std::uint32_t externalBodyId,
                                          ::rock::provider::RockProviderExternalSourceKind sourceKind,
                                          ::rock::provider::RockProviderHand sourceHand,
                                          const HandColliderBodyMetadata* handMetadata = nullptr) {
            if (sourceBodyId == INVALID_CONTACT_BODY_ID || externalBodyId == INVALID_CONTACT_BODY_ID || sourceBodyId == externalBodyId) {
                return;
            }

            ::rock::provider::RockProviderExternalContactV2 contact{};
            contact.sourceBodyId = sourceBodyId;
            contact.targetExternalBodyId = externalBodyId;
            contact.sourceKind = sourceKind;
            contact.sourceHand = sourceHand;
            contact.quality = ::rock::provider::RockProviderExternalContactQuality::BodyPairOnly;
            fillSourceVelocity(sourceBodyId, sourceKind, handMetadata, contact);

            if (ensureRawContactPoint()) {
                contact.quality = ::rock::provider::RockProviderExternalContactQuality::RawPoint;
                contact.contactPointWeightSum = rawContactPoint.contactPointWeightSum;
                std::copy_n(rawContactPoint.contactPointHavok, 4, contact.contactPointHavok);
                std::copy_n(rawContactPoint.contactNormalHavok, 4, contact.contactNormalHavok);
            } else {
                tryFillAggregateContactPoint(sourceBodyId, externalBodyId, contact);
            }

            if (handMetadata && handMetadata->valid) {
                contact.sourceRole = static_cast<std::uint32_t>(handMetadata->role);
                contact.sourcePartKind = static_cast<std::uint32_t>(handMetadata->finger);
                contact.sourceSubRole = static_cast<std::uint32_t>(handMetadata->segment);
            } else if (sourceKind == ::rock::provider::RockProviderExternalSourceKind::Weapon) {
                WeaponInteractionContact weaponContact{};
                if (_weaponCollision.tryGetWeaponContactAtomic(sourceBodyId, weaponContact)) {
                    contact.sourcePartKind = static_cast<std::uint32_t>(weaponContact.partKind);
                    contact.sourceRole = static_cast<std::uint32_t>(weaponContact.reloadRole);
                    contact.sourceSubRole = static_cast<std::uint32_t>(weaponContact.supportGripRole);
                }
            }

            ::rock::provider::recordExternalContact(contact);
        };

        auto endpointKindForEvidence = [](contact_pipeline_policy::ContactEndpointKind kind) {
            using SourceKind = contact_pipeline_policy::ContactEndpointKind;
            using EvidenceKind = contact_evidence::NativeContactEndpointKind;
            switch (kind) {
            case SourceKind::RightHand:
                return EvidenceKind::RightHand;
            case SourceKind::LeftHand:
                return EvidenceKind::LeftHand;
            case SourceKind::Weapon:
                return EvidenceKind::Weapon;
            case SourceKind::RightHeldObject:
                return EvidenceKind::RightHeldObject;
            case SourceKind::LeftHeldObject:
                return EvidenceKind::LeftHeldObject;
            case SourceKind::External:
                return EvidenceKind::External;
            case SourceKind::WorldSurface:
                return EvidenceKind::WorldSurface;
            case SourceKind::DynamicProp:
                return EvidenceKind::DynamicProp;
            case SourceKind::Actor:
                return EvidenceKind::Actor;
            case SourceKind::QueryOnly:
                return EvidenceKind::QueryOnly;
            default:
                return EvidenceKind::Unknown;
            }
        };

        auto fillNativeSourceVelocity = [world](std::uint32_t sourceBodyId, contact_evidence::NativeContactEvidenceRecord& evidence) {
            if (!world || !contact_evidence::isValidBodyId(sourceBodyId)) {
                return;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ sourceBodyId });
            if (!motion) {
                return;
            }

            const float scale = havokToGameScale();
            evidence.sourceVelocityGame = RE::NiPoint3{
                motion->linearVelocity.x * scale,
                motion->linearVelocity.y * scale,
                motion->linearVelocity.z * scale,
            };
        };

        auto publishNativeContactEvidence = [&](const HandColliderBodyMetadata* handMetadata = nullptr) {
            if (!contactRoute.recordWorldSurfaceEvidence ||
                !contact_pipeline_policy::isHand(contactRoute.source.kind) ||
                !contact_evidence::isValidBodyId(contactRoute.sourceBodyId) ||
                !contact_evidence::isValidBodyId(contactRoute.targetBodyId) ||
                contactRoute.sourceBodyId == contactRoute.targetBodyId) {
                return;
            }

            if (!ensureRawContactPoint()) {
                return;
            }

            contact_evidence::NativeContactEvidenceRecord evidence{};
            evidence.frame = _handContactActivity.currentFrame();
            evidence.sourceBodyId = contactRoute.sourceBodyId;
            evidence.targetBodyId = contactRoute.targetBodyId;
            evidence.sourceLayer = contactRoute.source.layer;
            evidence.targetLayer = contactRoute.target.layer;
            evidence.sourceFilterInfo = contactRoute.sourceBodyId == bodyIdA ? bodyAFilterInfo : bodyBFilterInfo;
            evidence.targetFilterInfo = contactRoute.targetBodyId == bodyIdA ? bodyAFilterInfo : bodyBFilterInfo;
            evidence.sourceKind = endpointKindForEvidence(contactRoute.source.kind);
            evidence.targetKind = endpointKindForEvidence(contactRoute.target.kind);
            evidence.sourceIsLeft = contact_pipeline_policy::isLeftOwned(contactRoute.source.kind);
            evidence.targetIsLeft = contact_pipeline_policy::isLeftOwned(contactRoute.target.kind);
            fillNativeSourceVelocity(contactRoute.sourceBodyId, evidence);

            const float scale = havokToGameScale();
            evidence.quality = contact_evidence::NativeContactQuality::RawPoint;
            evidence.contactPointWeightSum = rawContactPoint.contactPointWeightSum;
            evidence.contactPointGame = RE::NiPoint3{
                rawContactPoint.contactPointHavok[0] * scale,
                rawContactPoint.contactPointHavok[1] * scale,
                rawContactPoint.contactPointHavok[2] * scale,
            };
            evidence.contactNormalGame = RE::NiPoint3{
                rawContactPoint.contactNormalHavok[0],
                rawContactPoint.contactNormalHavok[1],
                rawContactPoint.contactNormalHavok[2],
            };

            if (handMetadata && handMetadata->valid) {
                evidence.sourceRole = static_cast<std::uint32_t>(handMetadata->role);
                evidence.sourcePartKind = static_cast<std::uint32_t>(handMetadata->finger);
                evidence.sourceSubRole = static_cast<std::uint32_t>(handMetadata->segment);
            } else if (contactRoute.source.kind == contact_pipeline_policy::ContactEndpointKind::Weapon) {
                WeaponInteractionContact weaponContact{};
                if (_weaponCollision.tryGetWeaponContactAtomic(contactRoute.sourceBodyId, weaponContact)) {
                    evidence.sourcePartKind = static_cast<std::uint32_t>(weaponContact.partKind);
                    evidence.sourceRole = static_cast<std::uint32_t>(weaponContact.reloadRole);
                    evidence.sourceSubRole = static_cast<std::uint32_t>(weaponContact.supportGripRole);
                }
            }

            _nativeContactEvidence.record(evidence);
        };

        auto recordBodyContactEvidence = [&]() {
            if (!contactRoute.recordBodyContact || !contact_pipeline_policy::isBody(contactRoute.source.kind)) {
                return;
            }

            const auto* bodyMetadata = bodySourceFor(contactRoute.sourceBodyId);
            if (!bodyMetadata || !bodyMetadata->valid) {
                return;
            }

            body_contact_runtime::BodyContactRecord record{};
            record.frame = _handContactActivity.currentFrame();
            record.bodyId = contactRoute.sourceBodyId;
            record.targetBodyId = contactRoute.targetBodyId;
            record.bodyLayer = contactRoute.source.layer;
            record.targetLayer = contactRoute.target.layer;
            record.role = bodyMetadata->role;
            record.zone = bodyMetadata->zone;
            record.side = bodyMetadata->side;
            record.descriptorIndex = bodyMetadata->descriptorIndex;
            record.targetKind = contactRoute.target.kind;
            record.inPowerArmor = bodyMetadata->inPowerArmor;
            if (const auto* targetBodyMetadata = bodySourceFor(contactRoute.targetBodyId); targetBodyMetadata && targetBodyMetadata->valid) {
                record.targetRole = targetBodyMetadata->role;
                record.targetZone = targetBodyMetadata->zone;
                record.targetSide = targetBodyMetadata->side;
                record.targetDescriptorIndex = targetBodyMetadata->descriptorIndex;
                record.targetInPowerArmor = targetBodyMetadata->inPowerArmor;
            }
            if (ensureRawContactPoint()) {
                const float scale = havokToGameScale();
                record.contactPointGame = RE::NiPoint3{
                    rawContactPoint.contactPointHavok[0] * scale,
                    rawContactPoint.contactPointHavok[1] * scale,
                    rawContactPoint.contactPointHavok[2] * scale,
                };
                record.hasContactPointGame = true;
            }

            _bodyContactRuntime.record(record);
        };

        auto notifyHeldExternalContact = [&](Hand& hand,
                                             std::atomic<std::uint64_t>& impactPair,
                                             bool bodyAIsHeld,
                                             bool bodyBIsHeld) {
            if (!bodyAIsHeld && !bodyBIsHeld) {
                return;
            }

            const std::uint32_t heldId = bodyAIsHeld ? bodyIdA : bodyIdB;
            const std::uint32_t other = bodyAIsHeld ? bodyIdB : bodyIdA;
            const auto decision = held_object_contact_policy::evaluateHeldExternalContact(
                held_object_contact_policy::HeldExternalContactInput{
                    .handHolding = hand.isHoldingAtomic(),
                    .bodyAIsHeld = bodyAIsHeld,
                    .bodyBIsHeld = bodyBIsHeld,
                    .otherIsRightHand = _rightHand.isHandColliderBodyId(other),
                    .otherIsLeftHand = _leftHand.isHandColliderBodyId(other),
                    .otherIsRightPalmBody = other == rightId,
                    .otherIsLeftPalmBody = other == leftId,
                    .otherIsBodyCollider = _bodyBoneColliders.isColliderBodyIdAtomic(other),
                    .otherIsExternalProvider = ::rock::provider::isExternalBodyId(other),
                });
            if (decision.sameHeldObject) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} held self-contact suppressed: bodyA={} bodyB={}",
                    hand.handName(),
                    bodyIdA,
                    bodyIdB);
                return;
            }
            if (!decision.notify) {
                return;
            }

            RE::NiPoint3 contactPointHavok{};
            RE::NiPoint3 contactNormalHavok{};
            bool hasContactNormal = false;
            if (ensureRawContactPoint()) {
                contactPointHavok = RE::NiPoint3{
                    rawContactPoint.contactPointHavok[0],
                    rawContactPoint.contactPointHavok[1],
                    rawContactPoint.contactPointHavok[2],
                };
                contactNormalHavok = RE::NiPoint3{
                    rawContactPoint.contactNormalHavok[0],
                    rawContactPoint.contactNormalHavok[1],
                    rawContactPoint.contactNormalHavok[2],
                };
                const float normalLengthSq =
                    contactNormalHavok.x * contactNormalHavok.x +
                    contactNormalHavok.y * contactNormalHavok.y +
                    contactNormalHavok.z * contactNormalHavok.z;
                hasContactNormal = std::isfinite(normalLengthSq) && normalLengthSq > 1.0e-6f;
            }

            const std::uint32_t otherLayer = bodyAIsHeld ? bodyBLayer : bodyALayer;
            hand.notifyHeldBodyContact(heldId, other, otherLayer, contactPointHavok, contactNormalHavok, hasContactNormal);
            impactPair.store(packHeldImpactPair(heldId, other), std::memory_order_release);
        };

        notifyHeldExternalContact(_rightHand, _lastHeldImpactPairRight, bodyAIsRightHeld, bodyBIsRightHeld);
        notifyHeldExternalContact(_leftHand, _lastHeldImpactPairLeft, bodyAIsLeftHeld, bodyBIsLeftHeld);

        recordBodyContactEvidence();

        /*
         * hknp can still deliver a body pair from the step boundary after ROCK
         * has already leased a hand body into no-collision ownership. Keep
         * held-object contact keepalive above this point, then suppress every
         * generated hand-side effect below it: native evidence, provider
         * contacts, weapon support contact, semantic touch, and dynamic push.
         */
        const bool rightBodyPairSuppressed =
            (bodyAIsRight || bodyBIsRight) &&
            (_rightDominantWeaponCollisionSuppressed.load(std::memory_order_acquire) ||
                _rightHand.hasContactEvidenceSuppressedAtomic());
        const bool leftBodyPairSuppressed =
            (bodyAIsLeft || bodyBIsLeft) &&
            (_leftWeaponSupportCollisionSuppressed.load(std::memory_order_acquire) ||
                _leftHand.hasContactEvidenceSuppressedAtomic());
        if (rightBodyPairSuppressed || leftBodyPairSuppressed) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "Suppressed hand body contact skipped: route={} rightSuppressed={} leftSuppressed={} bodyA={} bodyB={}",
                contact_pipeline_policy::routeName(contactRoute.route),
                rightBodyPairSuppressed ? "yes" : "no",
                leftBodyPairSuppressed ? "yes" : "no",
                bodyIdA,
                bodyIdB);
            return;
        }

        auto routeSourceHandContactEvidenceSuppressed = [&]() {
            if (contact_pipeline_policy::isRightHand(contactRoute.source.kind)) {
                return isHandContactEvidenceSuppressed(false);
            }
            if (contact_pipeline_policy::isLeftHand(contactRoute.source.kind)) {
                return isHandContactEvidenceSuppressed(true);
            }
            return false;
        };
        if (routeSourceHandContactEvidenceSuppressed()) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "Contact evidence skipped for stronger hand owner: route={} sourceBody={} targetBody={}",
                contact_pipeline_policy::routeName(contactRoute.route),
                contactRoute.sourceBodyId,
                contactRoute.targetBodyId);
            return;
        }

        bool isRight = bodyAIsRight || bodyBIsRight;
        bool isLeft = bodyAIsLeft || bodyBIsLeft;
        const HandColliderBodyMetadata* routeHandMetadata = nullptr;
        if (contact_pipeline_policy::isHand(contactRoute.source.kind)) {
            const auto* handSource = handSourceFor(contactRoute.sourceBodyId);
            routeHandMetadata = handSource && handSource->valid ? &handSource->metadata : nullptr;
        }

        publishNativeContactEvidence(routeHandMetadata);

        if (contactRoute.publishExternalContact) {
            publishExternalContact(contactRoute.sourceBodyId, contactRoute.targetBodyId, contactRoute.providerSourceKind, contactRoute.providerSourceHand, routeHandMetadata);
        }

        if (contactRoute.driveWeaponDynamicPush) {
            _lastContactSourceWeapon.store(contactRoute.sourceBodyId, std::memory_order_release);
            _lastContactBodyWeapon.store(contactRoute.targetBodyId, std::memory_order_release);
        }

        if (contactRoute.recordWorldSurfaceEvidence) {
            int logCount = _contactLogCounter.fetch_add(1, std::memory_order_relaxed);
            if (logCount % 60 == 0) {
                ROCK_LOG_DEBUG(Hand,
                    "Surface contact evidence: route={} sourceBody={} targetBody={} targetLayer={}",
                    contact_pipeline_policy::routeName(contactRoute.route),
                    contactRoute.sourceBodyId,
                    contactRoute.targetBodyId,
                    contactRoute.target.layer == contact_pipeline_policy::kUnknownLayer ? 0xFFFFFFFFu : contactRoute.target.layer);
            }
        }

        auto publishLeftWeaponContactFromPhysics = [&](const WeaponInteractionContact& weaponContact, std::uint32_t bodyId) {
            _leftWeaponContactPartKind.store(static_cast<std::uint32_t>(weaponContact.partKind), std::memory_order_release);
            _leftWeaponContactReloadRole.store(static_cast<std::uint32_t>(weaponContact.reloadRole), std::memory_order_release);
            _leftWeaponContactSupportRole.store(static_cast<std::uint32_t>(weaponContact.supportGripRole), std::memory_order_release);
            _leftWeaponContactSocketRole.store(static_cast<std::uint32_t>(weaponContact.socketRole), std::memory_order_release);
            _leftWeaponContactActionRole.store(static_cast<std::uint32_t>(weaponContact.actionRole), std::memory_order_release);
            _leftWeaponContactGripPose.store(static_cast<std::uint32_t>(weaponContact.fallbackGripPose), std::memory_order_release);
            _leftWeaponContactSequence.fetch_add(1, std::memory_order_acq_rel);
            _leftWeaponContactMissedFrames.store(0, std::memory_order_release);
            _leftWeaponContactBodyId.store(bodyId, std::memory_order_release);
        };

        if (!isRight && !isLeft) {
            return;
        }

        if (contactRoute.route == contact_pipeline_policy::ContactRoute::RockInternal) {
            return;
        }

        if (contactRoute.drivesWeaponSupportContact && contact_pipeline_policy::isLeftHand(contactRoute.source.kind)) {
            WeaponInteractionContact weaponContact{};
            if (_weaponCollision.tryGetWeaponContactAtomic(contactRoute.targetBodyId, weaponContact)) {
                publishLeftWeaponContactFromPhysics(weaponContact, contactRoute.targetBodyId);
            }
        }

        const auto* handSource = contactRoute.recordHandSemanticContact ? handSourceFor(contactRoute.sourceBodyId) : nullptr;
        if (!handSource || !handSource->valid) {
            return;
        }

        const auto contactActivity = _handContactActivity.registerHandContact(handSource->isLeft, handSource->metadata.bodyId, contactRoute.targetBodyId);
        if (contactActivity.newlyActive && g_rockConfig.rockDebugVerboseLogging) {
            ROCK_LOG_DEBUG(Hand,
                "ContactActivity: {} {} body={} target={} frame={} inserted={} evictedStale={}",
                handSource->isLeft ? "Left" : "Right",
                hand_collider_semantics::roleName(handSource->metadata.role),
                handSource->metadata.bodyId,
                contactRoute.targetBodyId,
                contactActivity.frame,
                contactActivity.inserted ? "yes" : "no",
                contactActivity.evictedStale ? "yes" : "no");
        }

        hand_semantic_contact_state::SemanticContactVector semanticContactPointGame{};
        hand_semantic_contact_state::SemanticContactVector semanticContactNormalGame{};
        const hand_semantic_contact_state::SemanticContactVector* semanticContactPoint = nullptr;
        const hand_semantic_contact_state::SemanticContactVector* semanticContactNormal = nullptr;
        if (ensureRawContactPoint()) {
            const float scale = havokToGameScale();
            semanticContactPointGame = hand_semantic_contact_state::SemanticContactVector{
                rawContactPoint.contactPointHavok[0] * scale,
                rawContactPoint.contactPointHavok[1] * scale,
                rawContactPoint.contactPointHavok[2] * scale,
            };
            semanticContactNormalGame = hand_semantic_contact_state::SemanticContactVector{
                rawContactPoint.contactNormalHavok[0],
                rawContactPoint.contactNormalHavok[1],
                rawContactPoint.contactNormalHavok[2],
            };
            if (hand_semantic_contact_state::isFiniteVector(semanticContactPointGame)) {
                semanticContactPoint = &semanticContactPointGame;
            }
            const float normalLengthSquared =
                semanticContactNormalGame.x * semanticContactNormalGame.x +
                semanticContactNormalGame.y * semanticContactNormalGame.y +
                semanticContactNormalGame.z * semanticContactNormalGame.z;
            if (hand_semantic_contact_state::isFiniteVector(semanticContactNormalGame) &&
                std::isfinite(normalLengthSquared) &&
                normalLengthSquared > 1.0e-6f) {
                semanticContactNormal = &semanticContactNormalGame;
            }
        }

        if (handSource->isLeft) {
            _leftHand.recordSemanticContact(handSource->metadata, contactRoute.targetBodyId, semanticContactPoint, semanticContactNormal);
            if (contactRoute.driveHandDynamicPush) {
                _lastContactSourceLeft.store(handSource->metadata.bodyId, std::memory_order_release);
                _lastContactBodyLeft.store(contactRoute.targetBodyId, std::memory_order_release);
            }
        } else {
            _rightHand.recordSemanticContact(handSource->metadata, contactRoute.targetBodyId, semanticContactPoint, semanticContactNormal);
            if (contactRoute.driveHandDynamicPush) {
                _lastContactSourceRight.store(handSource->metadata.bodyId, std::memory_order_release);
                _lastContactBodyRight.store(contactRoute.targetBodyId, std::memory_order_release);
            }
        }

        int logCount = _contactLogCounter.fetch_add(1, std::memory_order_relaxed);
        if (logCount % 30 == 0) {
            ROCK_LOG_DEBUG(Hand,
                "Contact: {} {} body={} hit body {} route={}",
                handSource->isLeft ? "Left" : "Right",
                hand_collider_semantics::roleName(handSource->metadata.role),
                handSource->metadata.bodyId,
                contactRoute.targetBodyId,
                contact_pipeline_policy::routeName(contactRoute.route));
        }
    }
