/*
 * Contact routing is kept as a separate core fragment because it bridges the native hknp contact signal, hand semantic and touch state, and weapon contacts. Keeping it in the PhysicsInteraction translation unit preserves the existing anonymous-namespace helpers while making the frame loop readable.
 */
    void PhysicsInteraction::resolveContacts(const PhysicsFrameContext& frame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::ContactResolve);

        auto* hknp = frame.hknpWorld;

        auto readBodyMass = [](RE::hknpWorld* world, std::uint32_t bodyId) {
            if (!world || bodyId == 0xFFFFFFFF || bodyId == object_physics_body_set::INVALID_BODY_ID) {
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

        processHeldImpact(_rightHand, false, _contacts.lastHeldImpactPairRight);
        processHeldImpact(_leftHand, true, _contacts.lastHeldImpactPairLeft);
    }
    void PhysicsInteraction::subscribeContactEvents(RE::hknpWorld* world)
    {
        if (!world) {
            ROCK_LOG_ERROR(Init, "Physics event subscriptions skipped because world is null");
            return;
        }

        /*
         * FO4VR raw producers identify event key 2 as
         * hknpManifoldProcessedEvent and key 3 as hknpContactImpulseEvent.
         * Both use the same signal/delegate ABI and participant IDs at
         * event+0x08/+0x0C. Keep distinct bridges because their payloads and
         * admission semantics are intentionally different.
         */
        constexpr auto kManifoldProcessedEventType = static_cast<RE::hknpEventType::Enum>(2);

        auto subscribeBridge = [&](const RE::hknpEventType::Enum eventType,
                                   ContactEventSubscriptionBridge& bridge,
                                   std::atomic<RE::hknpWorld*>& localWorld,
                                   std::atomic<void*>& localSignal,
                                   const char* eventName) {
            void* signal = world->GetEventSignal(eventType);
            if (!signal) {
                ROCK_LOG_ERROR(Init, "Failed to get {} event signal", eventName);
                return;
            }

            auto* currentWorld = bridge.world.load(std::memory_order_acquire);
            auto* currentSignal = bridge.signal.load(std::memory_order_acquire);
            const auto currentSnapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
                .world = reinterpret_cast<std::uintptr_t>(currentWorld),
                .signal = reinterpret_cast<std::uintptr_t>(currentSignal),
                .active = currentWorld != nullptr && currentSignal != nullptr,
            };
            const auto plan = contact_signal_subscription_policy::planSubscription(
                currentSnapshot,
                reinterpret_cast<std::uintptr_t>(world),
                reinterpret_cast<std::uintptr_t>(signal),
                bridge.hasRetainedNativeSlot(world, signal));

            if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::IgnoreNullSignal) {
                ROCK_LOG_ERROR(Init, "{} event subscription skipped because world or signal is null", eventName);
                return;
            }

            localSignal.store(signal, std::memory_order_release);
            localWorld.store(world, std::memory_order_release);
            bridge.signal.store(signal, std::memory_order_release);
            bridge.world.store(world, std::memory_order_release);
            bridge.instance.store(this, std::memory_order_release);

            if (plan.action == contact_signal_subscription_policy::ContactSignalSubscriptionAction::AlreadySubscribed) {
                const auto epoch = bridge.subscriptionEpoch.load(std::memory_order_acquire);
                if (!bridge.rememberRetainedNativeSlot(world, signal, epoch)) {
                    ROCK_LOG_WARN(
                        Init,
                        "{} event retained-slot table full while reusing bridge slot; future duplicate suppression may be degraded",
                        eventName);
                }
                ROCK_LOG_DEBUG(Init, "{} event signal already subscribed for current world; reusing native bridge slot", eventName);
                return;
            }

            if (plan.replaceExistingRuntimeStateWithoutUnsubscribe) {
                ROCK_LOG_INFO(
                    Init,
                    "Replacing {} event bridge state without native unsubscribe (action={})",
                    eventName,
                    static_cast<std::uint32_t>(plan.action));
            }

            ContactEventCallbackInfo cbInfo{};
            cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
            cbInfo.ctx = 0;

            typedef void subscribe_ext_t(void* signal, void* userData, void* callbackInfo);
            static REL::Relocation<subscribe_ext_t> subscribeExt{ REL::Offset(offsets::kFunc_SubscribeContactEvent) };
            subscribeExt(signal, static_cast<void*>(&bridge), &cbInfo);

            const auto epoch = bridge.subscriptionEpoch.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (!bridge.rememberRetainedNativeSlot(world, signal, epoch)) {
                ROCK_LOG_WARN(
                    Init,
                    "{} event retained-slot table full after native subscription; future duplicate suppression may be degraded",
                    eventName);
            }
            ROCK_LOG_INFO(
                Init,
                "Subscribed {} event bridge slot (epoch={}, action={})",
                eventName,
                epoch,
                static_cast<std::uint32_t>(plan.action));
        };

        subscribeBridge(
            RE::hknpEventType::kContact,
            s_contactEventBridge,
            _contacts.eventWorld,
            _contacts.eventSignal,
            "contact-impulse");
        subscribeBridge(
            kManifoldProcessedEventType,
            s_manifoldProcessedEventBridge,
            _contacts.manifoldEventWorld,
            _contacts.manifoldEventSignal,
            "manifold-processed");
    }

    void PhysicsInteraction::unsubscribeContactEvents(RE::hknpWorld* liveWorld)
    {
        auto deactivateBridge = [&](ContactEventSubscriptionBridge& bridge,
                                    std::atomic<RE::hknpWorld*>& localWorldAtomic,
                                    std::atomic<void*>& localSignalAtomic,
                                    const char* eventName) {
            auto* localWorld = localWorldAtomic.exchange(nullptr, std::memory_order_acq_rel);
            void* localSignal = localSignalAtomic.exchange(nullptr, std::memory_order_acq_rel);

            auto* expectedInstance = this;
            const bool deactivatedCurrentInstance = bridge.instance.compare_exchange_strong(
                expectedInstance,
                nullptr,
                std::memory_order_acq_rel,
                std::memory_order_acquire);

            auto* bridgeWorld = bridge.world.load(std::memory_order_acquire);
            void* bridgeSignal = bridge.signal.load(std::memory_order_acquire);
            const auto bridgeSnapshot = contact_signal_subscription_policy::ContactSignalSubscriptionSnapshot{
                .world = reinterpret_cast<std::uintptr_t>(bridgeWorld),
                .signal = reinterpret_cast<std::uintptr_t>(bridgeSignal),
                .active = bridgeWorld != nullptr && bridgeSignal != nullptr,
            };

            if (!contact_signal_subscription_policy::isActiveSubscription(bridgeSnapshot)) {
                return;
            }

            if (contact_signal_subscription_policy::shouldRetainNativeSlotAfterDeactivation(bridgeSnapshot)) {
                ROCK_LOG_INFO(
                    Init,
                    "Deactivated {} event bridge; native slots retained for hknpWorld cleanup (instanceCleared={}, world={}, signal={}, liveWorld={})",
                    eventName,
                    deactivatedCurrentInstance ? "yes" : "no",
                    static_cast<const void*>(bridgeWorld),
                    bridgeSignal,
                    static_cast<const void*>(liveWorld));
                return;
            }

            ROCK_LOG_INFO(
                Init,
                "Deactivated {} event bridge with no active native slot (instanceCleared={}, localWorld={}, localSignal={}, liveWorld={})",
                eventName,
                deactivatedCurrentInstance ? "yes" : "no",
                static_cast<const void*>(localWorld),
                localSignal,
                static_cast<const void*>(liveWorld));
        };

        deactivateBridge(
            s_contactEventBridge,
            _contacts.eventWorld,
            _contacts.eventSignal,
            "contact-impulse");
        deactivateBridge(
            s_manifoldProcessedEventBridge,
            _contacts.manifoldEventWorld,
            _contacts.manifoldEventSignal,
            "manifold-processed");
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
        const bool contactImpulseRoute = userData == static_cast<void*>(&s_contactEventBridge);
        const bool manifoldProcessedRoute = userData == static_cast<void*>(&s_manifoldProcessedEventBridge);
        if (!contactImpulseRoute && !manifoldProcessedRoute) {
            return;
        }

        auto* bridge = static_cast<ContactEventSubscriptionBridge*>(userData);
        auto* self = bridge->instance.load(std::memory_order_acquire);
        if (self && self->_lifecycle.initialized.load(std::memory_order_acquire)) {
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

            auto* world = reinterpret_cast<RE::hknpWorld*>(acceptance.effectiveWorld);
            if (manifoldProcessedRoute) {
                self->handleManifoldProcessedEvent(world, contactEventData);
            } else {
                self->handleContactEvent(world, contactEventData);
            }
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

    void PhysicsInteraction::handleManifoldProcessedEvent(RE::hknpWorld* world, void* eventData)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::NativeContactCallback);

        if (!world || !eventData) {
            return;
        }

        /*
         * FO4VR 0x1418028B0 and 0x141802A40 independently construct this
         * exact 0xB0-byte key-2 record. Only the common header and participant
         * IDs are needed here; the solved pose remains owned by ROCK's
         * post-solve body sample rather than the pre-solve manifold payload.
         */
        constexpr std::uint16_t kExpectedRecordSize = 0xB0;
        constexpr std::uint16_t kManifoldProcessedEventKey = 2;
        auto* data = static_cast<const std::uint8_t*>(eventData);
        const auto recordSize = *reinterpret_cast<const std::uint16_t*>(data + 0x00);
        const auto eventKey = *reinterpret_cast<const std::uint16_t*>(data + 0x04);
        if (recordSize != kExpectedRecordSize || eventKey != kManifoldProcessedEventKey) {
            return;
        }

        const auto bodyIdA = *reinterpret_cast<const std::uint32_t*>(data + 0x08);
        const auto bodyIdB = *reinterpret_cast<const std::uint32_t*>(data + 0x0C);
        const auto shapeKeyA =
            *reinterpret_cast<const std::uint32_t*>(data + 0x10);
        const auto shapeKeyB =
            *reinterpret_cast<const std::uint32_t*>(data + 0x14);
        const auto manifoldPointCount =
            *reinterpret_cast<const std::int32_t*>(data + 0x30);
        if (!contact_pipeline_policy::isValidBodyId(bodyIdA) ||
            !contact_pipeline_policy::isValidBodyId(bodyIdB) ||
            bodyIdA == bodyIdB) {
            return;
        }

        DynamicHandCollisionRuntime::DynamicBodyContactSource
            dynamicBodySourceA{};
        DynamicHandCollisionRuntime::DynamicBodyContactSource
            dynamicBodySourceB{};
        const bool bodyAIsDynamicHand =
            _dynamicHandCollision.tryClassifyDynamicBodyContactSourceAtomic(
                bodyIdA,
                shapeKeyA,
                dynamicBodySourceA);
        const bool bodyBIsDynamicHand =
            _dynamicHandCollision.tryClassifyDynamicBodyContactSourceAtomic(
                bodyIdB,
                shapeKeyB,
                dynamicBodySourceB);
        const bool bodyAIsDynamicWeapon =
            _dynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdA) ||
            _secondaryDynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdA);
        const bool bodyBIsDynamicWeapon =
            _dynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdB) ||
            _secondaryDynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdB);
        const bool bodyAIsRightHeld = _rightHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsRightHeld = _rightHand.isHeldBodyId(bodyIdB);
        const bool bodyAIsLeftHeld = _leftHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsLeftHeld = _leftHand.isHeldBodyId(bodyIdB);
        const bool heldContact = bodyAIsRightHeld || bodyBIsRightHeld || bodyAIsLeftHeld || bodyBIsLeftHeld;
        const bool generatedContact = bodyAIsDynamicHand || bodyBIsDynamicHand || bodyAIsDynamicWeapon || bodyBIsDynamicWeapon;
        const bool looseCollisionContact=_looseWeaponCollision.watches(bodyIdA) || _looseWeaponCollision.watches(bodyIdB) ||
            _objectRestCollision.watches(bodyIdA) || _objectRestCollision.watches(bodyIdB);
        if (!generatedContact && !heldContact && !looseCollisionContact) return;
        // ID-only admission precedes native reads; accepted events retain all body checks.
        if (!havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdA }) ||
            !havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdB })) {
            return;
        }
        const bool solvedChildContact =
            manifoldPointCount > 0 && manifoldPointCount <= 4;
        if (solvedChildContact && looseCollisionContact && !heldContact) {
            RE::NiPoint3 point{};
            const float scale=havokToGameScale()/static_cast<float>(manifoldPointCount);
            for (std::int32_t i=0;i<manifoldPointCount;++i) {
                const auto* p=reinterpret_cast<const float*>(data+0x70+i*0x10);
                point.x+=p[0]*scale; point.y+=p[1]*scale; point.z+=p[2]*scale;
            }
            const auto* normal=reinterpret_cast<const float*>(data+0x40);
            _looseWeaponCollision.contact(world,bodyIdA,bodyIdB,point,{normal[0],normal[1],normal[2]});
            _objectRestCollision.contact(world,bodyIdA,bodyIdB,point,{normal[0],normal[1],normal[2]});
        }
        if (solvedChildContact && heldContact) {
            auto recordHeldPlacementContact = [&](Hand& hand, bool heldA, bool heldB) {
                if (!hand.isHoldingAtomic() || heldA == heldB) return;
                const auto held = heldA ? bodyIdA : bodyIdB;
                const auto other = heldA ? bodyIdB : bodyIdA;
                if (_rightHand.isHeldBodyId(other) || _leftHand.isHeldBodyId(other)) return;
                const auto surface = havok_runtime::snapshotBodyIdentity(world,RE::hknpBodyId{other});
                if (!surface.valid || !surface.body || !surface.body->shape) return;
                const auto layer = surface.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
                if (collision_layer_policy::isRockGeneratedColliderLayer(layer) ||
                    collision_layer_policy::isActorOrBipedLayer(layer) ||
                    layer == collision_layer_policy::FO4_LAYER_CHARCONTROLLER) return;
                using physics_body_classifier::BodyMotionType;
                const auto motion = physics_body_classifier::motionTypeFromBodyFlags(surface.body->flags);
                if (motion != BodyMotionType::Static && motion != BodyMotionType::Keyframed) return;
                hand.notifyHeldSurfaceContact(held,other);
                ROCK_LOG_SAMPLE_DEBUG(Hand,1000,
                    "Held placement manifold contact hand={} held={} surface={} layer={} points={}",
                    hand.handName(),held,other,layer,manifoldPointCount);
            };
            recordHeldPlacementContact(_rightHand,bodyAIsRightHeld,bodyBIsRightHeld);
            recordHeldPlacementContact(_leftHand,bodyAIsLeftHeld,bodyBIsLeftHeld);
        }
        // Held placement observation must not enter the existing hand/weapon
        // audio, support, haptic or grab-response routes for new participant pairs.
        if (!generatedContact) return;
        if (solvedChildContact) {
            native_impact_audio::observeManifold(world, bodyIdA, bodyIdB, shapeKeyA, shapeKeyB);
        }
        if (solvedChildContact && bodyAIsDynamicHand) {
            storage_interaction::contact(world,bodyIdB,dynamicBodySourceA.isLeft);
            _dynamicHandCollision.recordDynamicBodyContactCallback(
                dynamicBodySourceA,
                bodyBIsDynamicHand &&
                    dynamicBodySourceA.isLeft !=
                        dynamicBodySourceB.isLeft,
                bodyBIsDynamicWeapon);
        }
        if (solvedChildContact && bodyBIsDynamicHand) {
            storage_interaction::contact(world,bodyIdA,dynamicBodySourceB.isLeft);
            _dynamicHandCollision.recordDynamicBodyContactCallback(
                dynamicBodySourceB,
                bodyAIsDynamicHand &&
                    dynamicBodySourceA.isLeft !=
                        dynamicBodySourceB.isLeft,
                bodyAIsDynamicWeapon);
        }

        /*
         * Palm/fingertip twins opt into this recurring key-2 path because the
         * solver does not reliably emit key-3 impulse records for persistent
         * static-world contacts. The dedicated classifier excludes forearms.
         */
        dynamic_hand_surface_contact_state::ContactSource dynamicHandSourceA{};
        dynamic_hand_surface_contact_state::ContactSource dynamicHandSourceB{};
        const bool bodyAIsDynamicHandSurfaceSource =
            _dynamicHandCollision.classifySurfaceContactSource(dynamicBodySourceA, dynamicHandSourceA);
        const bool bodyBIsDynamicHandSurfaceSource =
            _dynamicHandCollision.classifySurfaceContactSource(dynamicBodySourceB, dynamicHandSourceB);
        if (solvedChildContact &&
            bodyAIsDynamicHandSurfaceSource !=
            bodyBIsDynamicHandSurfaceSource) {
            const auto& source = bodyAIsDynamicHandSurfaceSource ?
                dynamicHandSourceA :
                dynamicHandSourceB;
            const auto otherBodyId = bodyAIsDynamicHandSurfaceSource ?
                bodyIdB :
                bodyIdA;
            std::uint32_t otherFilterInfo = 0;
            const bool otherLayerRead = havok_runtime::tryReadFilterInfo(
                world,
                RE::hknpBodyId{ otherBodyId },
                otherFilterInfo);
            const auto otherLayer =
                otherFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
            hand_semantic_contact_state::SemanticContactVector pointGame{};
            hand_semantic_contact_state::SemanticContactVector normalGame{};
            const float scale = havokToGameScale();
            const auto* normalHavok =
                reinterpret_cast<const float*>(data + 0x40);
            normalGame = {
                normalHavok[0],
                normalHavok[1],
                normalHavok[2],
            };
            for (std::int32_t pointIndex = 0;
                 pointIndex < manifoldPointCount;
                 ++pointIndex) {
                const auto* pointHavok = reinterpret_cast<const float*>(
                    data + 0x70 + pointIndex * 0x10);
                pointGame.x += pointHavok[0] * scale;
                pointGame.y += pointHavok[1] * scale;
                pointGame.z += pointHavok[2] * scale;
            }
            const float inversePointCount =
                1.0f / static_cast<float>(manifoldPointCount);
            pointGame.x *= inversePointCount;
            pointGame.y *= inversePointCount;
            pointGame.z *= inversePointCount;
            _dynamicHandCollision.recordSurfaceManifoldProcessedCallback(
                source,
                otherBodyId,
                otherLayerRead,
                otherLayer,
                &pointGame,
                &normalGame);
        }

        // Each held weapon's proxy routes its own side; a gun-on-gun pair
        // reaches both, each seeing the other proxy as its obstacle.
        for (auto* proxyRuntime : { &_dynamicWeaponCollision, &_secondaryDynamicWeaponCollision }) {
            const bool bodyAIsWeaponProxy =
                proxyRuntime->isProxyBodyIdAtomic(bodyIdA);
            const bool bodyBIsWeaponProxy =
                proxyRuntime->isProxyBodyIdAtomic(bodyIdB);
            if (bodyAIsWeaponProxy != bodyBIsWeaponProxy) {
                const auto proxyBodyId = bodyAIsWeaponProxy ? bodyIdA : bodyIdB;
                const auto otherBodyId = bodyAIsWeaponProxy ? bodyIdB : bodyIdA;
                std::uint32_t otherFilterInfo = 0;
                const bool otherLayerRead = havok_runtime::tryReadFilterInfo(
                    world,
                    RE::hknpBodyId{ otherBodyId },
                    otherFilterInfo);
                const auto otherLayer =
                    otherFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
                RE::NiPoint3 supportPointGame{};
                if (solvedChildContact) {
                    // The same recurring manifold positions used by surface hands
                    // above qualify weapon support while a contact is at rest.
                    const float pointScale = havokToGameScale() / static_cast<float>(manifoldPointCount);
                    for (std::int32_t index = 0; index < manifoldPointCount; ++index) {
                        const auto* point = reinterpret_cast<const float*>(data + 0x70 + index * 0x10);
                        supportPointGame.x += point[0] * pointScale;
                        supportPointGame.y += point[1] * pointScale;
                        supportPointGame.z += point[2] * pointScale;
                    }
                }
                RE::NiPoint3 targetNormal{};
                if (solvedChildContact) {
                    // 1418028B0 copies the manifold normal to +0x40. Normals point
                    // B -> A; orient target -> weapon. Key-2 removal events leave
                    // this payload uninitialized and must not read it.
                    const auto* manifoldNormal=reinterpret_cast<const float*>(data+0x40);
                    const float sign=bodyAIsWeaponProxy?1.0f:-1.0f;
                    targetNormal={manifoldNormal[0]*sign,manifoldNormal[1]*sign,manifoldNormal[2]*sign};
                }
                proxyRuntime->recordObstacleManifoldProcessedCallback(
                    world,
                    proxyBodyId,
                    otherBodyId,
                    otherLayerRead,
                    otherLayer,
                    solvedChildContact ? &supportPointGame : nullptr,targetNormal,solvedChildContact?static_cast<std::uint32_t>(manifoldPointCount):0);
            }
        }
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

        /*
         * Dynamic-hand compound child identity exists only in the verified
         * key-2 shape keys. Key-3 impulse records carry the shared body ID but
         * cannot identify a semantic child, so they must not publish hand
         * contact or surface-grab evidence.
         */

        /*
         * The dynamic weapon proxy is intentionally absent from the normal
         * generated-body contact registry: it is solver/visual feedback, not
         * hand, gameplay, or provider contact evidence. Capture only a real
         * proxy-vs-obstacle callback before the ordinary registry prefilter can
         * discard this pair. Obstacle eligibility includes native NPC layer 33;
         * the experimental matrix policy controls whether those pairs collide.
         */
        const bool bodyAIsDynamicWeaponProxy =
            _dynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdA) ||
            _secondaryDynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdA);
        const bool bodyBIsDynamicWeaponProxy =
            _dynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdB) ||
            _secondaryDynamicWeaponCollision.isProxyBodyIdAtomic(bodyIdB);

        const auto rightId = _rightHand.getCollisionBodyId().value;
        const auto leftId = _leftHand.getCollisionBodyId().value;

        using generated_body_contact_registry::Classification;
        using generated_body_contact_registry::GeneratedBodyKind;
        using generated_body_contact_registry::hasFlag;
        using generated_body_contact_registry::kFlagPowerArmor;
        using generated_body_contact_registry::kFlagPrimaryAnchor;

        struct HandContactSource
        {
            bool valid = false;
            bool isLeft = false;
            bool primaryAnchor = false;
            HandColliderBodyMetadata metadata{};
        };

        struct WeaponContactSource
        {
            bool valid = false;
            WeaponInteractionContact contact{};
        };

        Classification bodyAClassification{};
        Classification bodyBClassification{};
        _contacts.generatedBodyRegistry.tryClassify(bodyIdA, bodyAClassification);
        _contacts.generatedBodyRegistry.tryClassify(bodyIdB, bodyBClassification);

        auto classifyHandBody = [](const Classification& classification) {
            HandContactSource source{};
            if (!classification.valid ||
                (classification.kind != GeneratedBodyKind::RightHand && classification.kind != GeneratedBodyKind::LeftHand)) {
                return source;
            }

            source.valid = true;
            source.isLeft = classification.kind == GeneratedBodyKind::LeftHand;
            source.primaryAnchor = hasFlag(classification.flags, kFlagPrimaryAnchor);
            source.metadata.valid = true;
            source.metadata.isLeft = source.isLeft;
            source.metadata.primaryPalmAnchor = source.primaryAnchor;
            source.metadata.bodyId = classification.bodyId;
            source.metadata.role = static_cast<hand_collider_semantics::HandColliderRole>(classification.role);
            source.metadata.finger = static_cast<hand_collider_semantics::HandFinger>(classification.partKind);
            source.metadata.segment = static_cast<hand_collider_semantics::HandFingerSegment>(classification.subRole);
            return source;
        };

        auto classifyWeaponBody = [](const Classification& classification) {
            WeaponContactSource source{};
            if (!classification.valid || classification.kind != GeneratedBodyKind::Weapon) {
                return source;
            }

            source.valid = true;
            source.contact.valid = true;
            source.contact.bodyId = classification.bodyId;
            source.contact.partKind = static_cast<WeaponPartKind>(classification.partKind);
            source.contact.reloadRole = static_cast<WeaponReloadRole>(classification.role);
            source.contact.supportGripRole = static_cast<WeaponSupportGripRole>(classification.subRole);
            source.contact.socketRole = static_cast<WeaponSocketRole>(classification.socketRole);
            source.contact.actionRole = static_cast<WeaponActionRole>(classification.actionRole);
            source.contact.fallbackGripPose = static_cast<WeaponGripPoseId>(classification.gripPose);
            source.contact.weaponGenerationKey = classification.generationKey;
            return source;
        };

        auto classifyBodyCollider = [](const Classification& classification) {
            BodyBoneColliderMetadata metadata{};
            if (!classification.valid || classification.kind != GeneratedBodyKind::Body) {
                return metadata;
            }

            metadata.valid = true;
            metadata.inPowerArmor = hasFlag(classification.flags, kFlagPowerArmor);
            metadata.role = static_cast<skeleton_bone_debug_math::BoneColliderRole>(classification.role);
            metadata.zone = static_cast<body_zone::BodyZoneKind>(classification.zone);
            metadata.side = static_cast<body_zone::BodyZoneSide>(classification.side);
            metadata.bodyId = classification.bodyId;
            metadata.descriptorIndex = classification.descriptorIndex;
            metadata.lengthGameUnits = classification.lengthGameUnits;
            metadata.radiusGameUnits = classification.radiusGameUnits;
            return metadata;
        };

        const auto bodyARight = bodyAClassification.kind == GeneratedBodyKind::RightHand ? classifyHandBody(bodyAClassification) : HandContactSource{};
        const auto bodyBRight = bodyBClassification.kind == GeneratedBodyKind::RightHand ? classifyHandBody(bodyBClassification) : HandContactSource{};
        const auto bodyALeft = bodyAClassification.kind == GeneratedBodyKind::LeftHand ? classifyHandBody(bodyAClassification) : HandContactSource{};
        const auto bodyBLeft = bodyBClassification.kind == GeneratedBodyKind::LeftHand ? classifyHandBody(bodyBClassification) : HandContactSource{};
        const auto bodyAWeapon = classifyWeaponBody(bodyAClassification);
        const auto bodyBWeapon = classifyWeaponBody(bodyBClassification);
        BodyBoneColliderMetadata bodyABodyMetadata = classifyBodyCollider(bodyAClassification);
        BodyBoneColliderMetadata bodyBBodyMetadata = classifyBodyCollider(bodyBClassification);
        const bool bodyAIsRight = bodyARight.valid;
        const bool bodyBIsRight = bodyBRight.valid;
        const bool bodyAIsLeft = bodyALeft.valid;
        const bool bodyBIsLeft = bodyBLeft.valid;
        const bool bodyAIsRightHeld = _rightHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsRightHeld = _rightHand.isHeldBodyId(bodyIdB);
        const bool bodyAIsLeftHeld = _leftHand.isHeldBodyId(bodyIdA);
        const bool bodyBIsLeftHeld = _leftHand.isHeldBodyId(bodyIdB);
        const bool bodyAIsWeapon = bodyAWeapon.valid;
        const bool bodyBIsWeapon = bodyBWeapon.valid;
        const bool bodyAIsBody = bodyABodyMetadata.valid;
        const bool bodyBIsBody = bodyBBodyMetadata.valid;
        const bool bodyAIsRockSource = bodyAIsRight || bodyAIsLeft || bodyAIsRightHeld || bodyAIsLeftHeld || bodyAIsWeapon || bodyAIsBody;
        const bool bodyBIsRockSource = bodyBIsRight || bodyBIsLeft || bodyBIsRightHeld || bodyBIsLeftHeld || bodyBIsWeapon || bodyBIsBody;

        auto looseGrenadeImpactBodyIsWatched = [&](std::uint32_t bodyId) {
            if (isInvalidGrabBodyId(bodyId)) {
                return false;
            }
            for (const auto& watchedBodyId : _forceGrab.grenadeImpactBodyIds) {
                if (watchedBodyId.load(std::memory_order_acquire) == bodyId) {
                    return true;
                }
            }
            return false;
        };

        if (!bodyAIsRockSource && !bodyBIsRockSource &&
            !bodyAIsDynamicWeaponProxy && !bodyBIsDynamicWeaponProxy &&
            !looseGrenadeImpactBodyIsWatched(bodyIdA) && !looseGrenadeImpactBodyIsWatched(bodyIdB)) return;
        if (!havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdA }) ||
            !havok_runtime::bodySlotLooksReadable(world, RE::hknpBodyId{ bodyIdB })) {
            return;
        }
        for (auto* proxyRuntime : { &_dynamicWeaponCollision, &_secondaryDynamicWeaponCollision }) {
            const bool proxyIsA = proxyRuntime->isProxyBodyIdAtomic(bodyIdA);
            if (proxyIsA == proxyRuntime->isProxyBodyIdAtomic(bodyIdB)) {
                continue;
            }
            const std::uint32_t proxyBodyId =
                proxyIsA ? bodyIdA : bodyIdB;
            const std::uint32_t otherBodyId =
                proxyIsA ? bodyIdB : bodyIdA;
            std::uint32_t otherFilterInfo = 0;
            const bool otherLayerRead = havok_runtime::tryReadFilterInfo(
                world,
                RE::hknpBodyId{ otherBodyId },
                otherFilterInfo);
            const std::uint32_t otherLayer =
                otherFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
            const bool rawContactPointValid =
                otherLayerRead &&
                collision_layer_policy::isDynamicWeaponProxySolverObstacleLayer(otherLayer) &&
                ensureRawContactPoint();
            proxyRuntime->recordObstacleContactCallback(
                world,
                proxyBodyId,
                otherBodyId,
                otherLayerRead,
                otherLayer,
                proxyIsA,
                rawContactPointValid ? &rawContactPoint : nullptr);
        }

        auto recordLooseGrenadeImpactIfArmed = [&]() {
            auto tryRecord = [&](std::uint32_t watchedBodyId,
                                 std::uint32_t otherBodyId,
                                 bool watchedIsHeld,
                                 bool otherIsRightHand,
                                 bool otherIsLeftHand) {
                if (!looseGrenadeImpactBodyIsWatched(watchedBodyId) || watchedIsHeld || otherIsRightHand || otherIsLeftHand ||
                    otherBodyId == rightId || otherBodyId == leftId || isInvalidGrabBodyId(otherBodyId)) {
                    return false;
                }

                _forceGrab.pendingGrenadeImpactPair.store(packHeldImpactPair(watchedBodyId, otherBodyId), std::memory_order_release);
                return true;
            };

            if (tryRecord(
                    bodyIdA,
                    bodyIdB,
                    bodyAIsRightHeld || bodyAIsLeftHeld,
                    bodyBIsRight,
                    bodyBIsLeft)) {
                return;
            }
            static_cast<void>(tryRecord(
                bodyIdB,
                bodyIdA,
                bodyBIsRightHeld || bodyBIsLeftHeld,
                bodyAIsRight,
                bodyAIsLeft));
        };

        recordLooseGrenadeImpactIfArmed();

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
            return contact_pipeline_policy::kUnknownLayer;
        };

        auto filterInfoToLayer = [](std::uint32_t filterInfo) {
            return filterInfo == contact_pipeline_policy::kUnknownLayer ? contact_pipeline_policy::kUnknownLayer : (filterInfo & 0x7Fu);
        };

        const std::uint32_t bodyAFilterInfo = readBodyFilterInfo(bodyIdA);
        const std::uint32_t bodyBFilterInfo = readBodyFilterInfo(bodyIdB);
        const std::uint32_t bodyALayer = filterInfoToLayer(bodyAFilterInfo);
        const std::uint32_t bodyBLayer = filterInfoToLayer(bodyBFilterInfo);

        auto makeEndpoint = [&](std::uint32_t bodyId, std::uint32_t layer, bool isRightHand, bool isLeftHand, bool isWeapon, bool isRightHeld, bool isLeftHeld, bool isBody) {
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
            } else {
                endpoint.kind = contact_pipeline_policy::classifyNonRockLayer(layer);
            }
            return endpoint;
        };

        const auto endpointA = makeEndpoint(bodyIdA, bodyALayer, bodyAIsRight, bodyAIsLeft, bodyAIsWeapon, bodyAIsRightHeld, bodyAIsLeftHeld, bodyAIsBody);
        const auto endpointB = makeEndpoint(bodyIdB, bodyBLayer, bodyBIsRight, bodyBIsLeft, bodyBIsWeapon, bodyBIsRightHeld, bodyBIsLeftHeld, bodyBIsBody);
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

        auto weaponSourceFor = [&](std::uint32_t bodyId) -> const WeaponContactSource* {
            if (bodyAWeapon.valid && bodyAWeapon.contact.bodyId == bodyId) {
                return &bodyAWeapon;
            }
            if (bodyBWeapon.valid && bodyBWeapon.contact.bodyId == bodyId) {
                return &bodyBWeapon;
            }
            return nullptr;
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
            record.frame = _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
            record.elapsedSeconds = _contacts.clock.seconds();
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

            _contacts.bodyRuntime.record(record);
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
            const bool otherIsA = other == bodyIdA;
            const auto decision = held_object_contact_policy::evaluateHeldExternalContact(
                held_object_contact_policy::HeldExternalContactInput{
                    .handHolding = hand.isHoldingAtomic(),
                    .bodyAIsHeld = bodyAIsHeld,
                    .bodyBIsHeld = bodyBIsHeld,
                    .otherIsRightHand = otherIsA ? bodyAIsRight : bodyBIsRight,
                    .otherIsLeftHand = otherIsA ? bodyAIsLeft : bodyBIsLeft,
                    .otherIsRightPalmBody = other == rightId,
                    .otherIsLeftPalmBody = other == leftId,
                    .otherIsBodyCollider = otherIsA ? bodyAIsBody : bodyBIsBody,
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

        notifyHeldExternalContact(_rightHand, _contacts.lastHeldImpactPairRight, bodyAIsRightHeld, bodyBIsRightHeld);
        notifyHeldExternalContact(_leftHand, _contacts.lastHeldImpactPairLeft, bodyAIsLeftHeld, bodyBIsLeftHeld);

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
            (weaponHandSuppressedFor(false, WeaponHandReason::Dominant) ||
                _rightHand.hasContactEvidenceSuppressedAtomic());
        const bool leftBodyPairSuppressed =
            (bodyAIsLeft || bodyBIsLeft) &&
            (weaponHandSuppressedFor(true, WeaponHandReason::Grip) ||
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

        auto publishWeaponContactFromPhysics = [&](bool isLeft, const WeaponInteractionContact& weaponContact, std::uint32_t bodyId) {
            auto& partKind = isLeft ? _weaponContact.left.partKind : _weaponContact.right.partKind;
            auto& reloadRole = isLeft ? _weaponContact.left.reloadRole : _weaponContact.right.reloadRole;
            auto& supportRole = isLeft ? _weaponContact.left.supportRole : _weaponContact.right.supportRole;
            auto& socketRole = isLeft ? _weaponContact.left.socketRole : _weaponContact.right.socketRole;
            auto& actionRole = isLeft ? _weaponContact.left.actionRole : _weaponContact.right.actionRole;
            auto& gripPose = isLeft ? _weaponContact.left.gripPose : _weaponContact.right.gripPose;
            auto& sequence = isLeft ? _weaponContact.left.sequence : _weaponContact.right.sequence;
            auto& missedFrames = isLeft ? _weaponContact.left.missedFrames : _weaponContact.right.missedFrames;
            auto& bodyIdAtomic = isLeft ? _weaponContact.left.bodyId : _weaponContact.right.bodyId;

            partKind.store(static_cast<std::uint32_t>(weaponContact.partKind), std::memory_order_release);
            reloadRole.store(static_cast<std::uint32_t>(weaponContact.reloadRole), std::memory_order_release);
            supportRole.store(static_cast<std::uint32_t>(weaponContact.supportGripRole), std::memory_order_release);
            socketRole.store(static_cast<std::uint32_t>(weaponContact.socketRole), std::memory_order_release);
            actionRole.store(static_cast<std::uint32_t>(weaponContact.actionRole), std::memory_order_release);
            gripPose.store(static_cast<std::uint32_t>(weaponContact.fallbackGripPose), std::memory_order_release);
            sequence.fetch_add(1, std::memory_order_acq_rel);
            missedFrames.store(0, std::memory_order_release);
            bodyIdAtomic.store(bodyId, std::memory_order_release);
        };

        if (contactRoute.route == contact_pipeline_policy::ContactRoute::RockInternal &&
            contactRoute.target.kind == contact_pipeline_policy::ContactEndpointKind::SecondaryWeapon) {
            ROCK_LOG_SAMPLE_INFO(Weapon, 1000, "AKIMBO weapon-on-weapon contact primaryBody={} secondaryBody={}",
                contactRoute.sourceBodyId, contactRoute.targetBodyId);
            return;
        }

        if (!isRight && !isLeft) {
            return;
        }

        if (contactRoute.route == contact_pipeline_policy::ContactRoute::RockInternal) {
            return;
        }

        if (contactRoute.drivesWeaponSupportContact && contact_pipeline_policy::isHand(contactRoute.source.kind)) {
            if (const auto* weaponSource = weaponSourceFor(contactRoute.targetBodyId); weaponSource && weaponSource->valid) {
                publishWeaponContactFromPhysics(contact_pipeline_policy::isLeftHand(contactRoute.source.kind), weaponSource->contact, contactRoute.targetBodyId);
            }
        }

        const auto* handSource = contactRoute.recordHandSemanticContact ? handSourceFor(contactRoute.sourceBodyId) : nullptr;
        if (!handSource || !handSource->valid) {
            return;
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

        auto& contactHand = handSource->isLeft ? _leftHand : _rightHand;
        contactHand.recordSemanticContact(handSource->metadata, contactRoute.targetBodyId, semanticContactPoint, semanticContactNormal,
            contactRoute.recordHandTouch);

        int logCount = _diagnostics.contactLogCounter.fetch_add(1, std::memory_order_relaxed);
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
