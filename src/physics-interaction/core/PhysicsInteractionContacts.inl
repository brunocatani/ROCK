/*
 * Contact routing is kept as a separate core fragment because it bridges the native hknp contact signal, hand semantic state, weapon contacts, and push assist. Keeping it in the PhysicsInteraction translation unit preserves the existing anonymous-namespace helpers while making the frame loop readable.
 */
    void PhysicsInteraction::resolveContacts(const PhysicsFrameContext& frame)
    {
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

        auto* currentWorld = _contactEventWorld.load(std::memory_order_acquire);
        auto* currentSignal = _contactEventSignal.load(std::memory_order_acquire);
        const auto currentSnapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
            .world = reinterpret_cast<std::uintptr_t>(currentWorld),
            .signal = reinterpret_cast<std::uintptr_t>(currentSignal),
            .active = currentWorld != nullptr && currentSignal != nullptr,
        };
        const auto plan = contact_signal_subscription_policy::planSubscription(
            currentSnapshot,
            reinterpret_cast<std::uintptr_t>(world),
            reinterpret_cast<std::uintptr_t>(signal));

        if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::IgnoreNullSignal) {
            ROCK_LOG_ERROR(Init, "Contact event subscription skipped because world or signal is null");
            return;
        }

        if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::AlreadySubscribed) {
            ROCK_LOG_DEBUG(Init, "Contact event signal already subscribed for current world");
            return;
        }

        if (plan.unsubscribeExistingSignal) {
            unsubscribeContactEvents(world);
        } else if (plan.clearExistingWithoutUnsubscribe) {
            _contactEventWorld.store(nullptr, std::memory_order_release);
            _contactEventSignal.store(nullptr, std::memory_order_release);
            ROCK_LOG_INFO(Init, "Cleared stale contact event subscription state before subscribing new world");
        }

        ContactEventCallbackInfo cbInfo{};
        cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
        cbInfo.ctx = 0;

        typedef void subscribe_ext_t(void* signal, void* userData, void* callbackInfo);
        static REL::Relocation<subscribe_ext_t> subscribeExt{ REL::Offset(offsets::kFunc_SubscribeContactEvent) };
        subscribeExt(signal, static_cast<void*>(this), &cbInfo);

        _contactEventSignal.store(signal, std::memory_order_release);
        _contactEventWorld.store(world, std::memory_order_release);
        ROCK_LOG_INFO(Init, "Subscribed to contact events");
    }

    void PhysicsInteraction::unsubscribeContactEvents(RE::hknpWorld* liveWorld)
    {
        const auto world = _contactEventWorld.load(std::memory_order_acquire);
        const auto signal = _contactEventSignal.load(std::memory_order_acquire);
        const auto snapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
            .world = reinterpret_cast<std::uintptr_t>(world),
            .signal = reinterpret_cast<std::uintptr_t>(signal),
            .active = world != nullptr && signal != nullptr,
        };

        if (!contact_signal_subscription_policy::isActiveSubscription(snapshot)) {
            return;
        }

        _contactEventWorld.store(nullptr, std::memory_order_release);
        _contactEventSignal.store(nullptr, std::memory_order_release);

        if (!contact_signal_subscription_policy::canUnsubscribeFromWorld(snapshot, reinterpret_cast<std::uintptr_t>(liveWorld))) {
            ROCK_LOG_INFO(Init, "Cleared contact event subscription state without native unsubscribe because the subscribed world is stale");
            return;
        }

        ContactEventCallbackInfo cbInfo{};
        cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
        cbInfo.ctx = 0;

        std::uint8_t removed = 0;
        typedef void* unsubscribe_ext_t(void* signal, std::uint8_t* outRemoved, void* userData, void* callbackInfo, std::uint64_t callbackInfoSize);
        static REL::Relocation<unsubscribe_ext_t> unsubscribeExt{ REL::Offset(offsets::kFunc_UnsubscribeSignalCallback) };
        unsubscribeExt(signal, &removed, static_cast<void*>(this), &cbInfo, sizeof(cbInfo));

        ROCK_LOG_INFO(Init, "Unsubscribed from contact events (removed={})", removed != 0 ? "yes" : "no");
    }

    void PhysicsInteraction::onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData)
    {
        __try {
            if (!s_hooksEnabled.load(std::memory_order_acquire))
                return;
            auto* self = s_instance.load(std::memory_order_acquire);
            if (self && self->_initialized.load(std::memory_order_acquire)) {
                auto* subscribedWorld = self->_contactEventWorld.load(std::memory_order_acquire);
                auto* subscribedSignal = self->_contactEventSignal.load(std::memory_order_acquire);
                const auto snapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
                    .world = reinterpret_cast<std::uintptr_t>(subscribedWorld),
                    .signal = reinterpret_cast<std::uintptr_t>(subscribedSignal),
                    .active = subscribedWorld != nullptr && subscribedSignal != nullptr,
                };
                if (userData != static_cast<void*>(self)) {
                    return;
                }

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

    void PhysicsInteraction::handleContactEvent(RE::hknpWorld* world, void* contactEventData)
    {
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
        const bool bodyAIsRockSource = bodyAIsRight || bodyAIsLeft || bodyAIsRightHeld || bodyAIsLeftHeld || bodyAIsWeapon;
        const bool bodyBIsRockSource = bodyBIsRight || bodyBIsLeft || bodyBIsRightHeld || bodyBIsLeftHeld || bodyBIsWeapon;

        if (contact_pipeline_policy::shouldSkipContactSignalBeforeLayerRead(contact_pipeline_policy::ContactSignalPrefilter{
                .bodyIdA = bodyIdA,
                .bodyIdB = bodyIdB,
                .bodyAIsRockSource = bodyAIsRockSource,
                .bodyBIsRockSource = bodyBIsRockSource,
            })) {
            return;
        }

        auto readBodyLayer = [world](std::uint32_t bodyId) {
            std::uint32_t filterInfo = 0;
            if (world && havok_runtime::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, filterInfo)) {
                return filterInfo & 0x7F;
            }
            return contact_pipeline_policy::kUnknownLayer;
        };

        const std::uint32_t bodyALayer = readBodyLayer(bodyIdA);
        const std::uint32_t bodyBLayer = readBodyLayer(bodyIdB);

        auto makeEndpoint = [&](std::uint32_t bodyId, std::uint32_t layer, bool isRightHand, bool isLeftHand, bool isWeapon, bool isRightHeld, bool isLeftHeld, bool isExternal) {
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
            } else if (isExternal) {
                endpoint.kind = ContactEndpointKind::External;
            } else {
                endpoint.kind = contact_pipeline_policy::classifyNonRockLayer(layer);
            }
            return endpoint;
        };

        const auto endpointA = makeEndpoint(bodyIdA, bodyALayer, bodyAIsRight, bodyAIsLeft, bodyAIsWeapon, bodyAIsRightHeld, bodyAIsLeftHeld, bodyAIsExternal);
        const auto endpointB = makeEndpoint(bodyIdB, bodyBLayer, bodyBIsRight, bodyBIsLeft, bodyBIsWeapon, bodyBIsRightHeld, bodyBIsLeftHeld, bodyBIsExternal);
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

        auto fillSourceVelocity = [world](std::uint32_t sourceBodyId, ::rock::provider::RockProviderExternalContactV2& contact) {
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
            fillSourceVelocity(sourceBodyId, contact);

            if (ensureRawContactPoint()) {
                contact.quality = ::rock::provider::RockProviderExternalContactQuality::RawPoint;
                contact.contactPointWeightSum = rawContactPoint.contactPointWeightSum;
                std::copy_n(rawContactPoint.contactPointHavok, 4, contact.contactPointHavok);
                std::copy_n(rawContactPoint.contactNormalHavok, 4, contact.contactNormalHavok);
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

        if (_rightHand.isHoldingAtomic()) {
            if (bodyAIsRightHeld || bodyBIsRightHeld) {
                std::uint32_t heldId = bodyAIsRightHeld ? bodyIdA : bodyIdB;
                std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
                if (!_rightHand.isHandColliderBodyId(other) && !_leftHand.isHandColliderBodyId(other) && other != rightId && other != leftId &&
                    !::rock::provider::isExternalBodyId(other)) {
                    _rightHand.notifyHeldBodyContact();
                }
            }
        }
        if (_leftHand.isHoldingAtomic()) {
            if (bodyAIsLeftHeld || bodyBIsLeftHeld) {
                std::uint32_t heldId = bodyAIsLeftHeld ? bodyIdA : bodyIdB;
                std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
                if (!_rightHand.isHandColliderBodyId(other) && !_leftHand.isHandColliderBodyId(other) && other != rightId && other != leftId &&
                    !::rock::provider::isExternalBodyId(other)) {
                    _leftHand.notifyHeldBodyContact();
                }
            }
        }

        bool isRight = bodyAIsRight || bodyBIsRight;
        bool isLeft = bodyAIsLeft || bodyBIsLeft;

        if (contactRoute.publishExternalContact) {
            const HandColliderBodyMetadata* handMetadata = nullptr;
            if (contactRoute.providerSourceKind == ::rock::provider::RockProviderExternalSourceKind::Hand) {
                const auto* handSource = handSourceFor(contactRoute.sourceBodyId);
                handMetadata = handSource && handSource->valid ? &handSource->metadata : nullptr;
            }

            publishExternalContact(contactRoute.sourceBodyId, contactRoute.targetBodyId, contactRoute.providerSourceKind, contactRoute.providerSourceHand, handMetadata);
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

        if (handSource->isLeft) {
            _leftHand.recordSemanticContact(handSource->metadata, contactRoute.targetBodyId);
            if (contactRoute.driveHandDynamicPush) {
                _lastContactSourceLeft.store(handSource->metadata.bodyId, std::memory_order_release);
                _lastContactBodyLeft.store(contactRoute.targetBodyId, std::memory_order_release);
            }
        } else {
            _rightHand.recordSemanticContact(handSource->metadata, contactRoute.targetBodyId);
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
