/*
 * Provider snapshot and query glue is split from lifecycle/update orchestration because it exposes ROCK state to external consumers without owning physics behavior. It stays in this translation unit to preserve helper visibility and public API behavior.
 */
    void PhysicsInteraction::fillProviderFrameSnapshot(::rock::provider::Publication& outSnapshot) const
    {
        const auto& runtime = runtime_state::currentFrame();
        outSnapshot.providerReady = isProviderReady() ? 1u : 0u;
        outSnapshot.frikSkeletonReady = runtime.localSkeletonReady ? 1u : 0u;
        outSnapshot.menuBlocking = runtime.localMenuBlocking ? 1u : 0u;
        outSnapshot.configBlocking = runtime.compatibilityConfigBlocking ? 1u : 0u;
        outSnapshot.gameToHavokScale = physics_scale::gameToHavok();
        outSnapshot.havokToGameScale = physics_scale::havokToGame();
        outSnapshot.physicsScaleRevision = physics_scale::revision();
        outSnapshot.lifecycleFlags = _lifecycle.flagsAtomic.load(std::memory_order_acquire);
        outSnapshot.lastLifecycleReason = static_cast<api::core::LifecycleReason>(
            _lifecycle.lastReasonAtomic.load(std::memory_order_acquire));
        outSnapshot.worldGeneration = _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        outSnapshot.skeletonGeneration = _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        outSnapshot.providerGeneration = _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        outSnapshot.stableFrameCount = _lifecycle.stableFrameCountAtomic.load(std::memory_order_acquire);
        /*
         * Truthful timing publication: the validity flag is set only for a
         * measured game delta. Consumers guard for non-positive values, so an
         * unmeasurable frame publishes zero instead of a fabricated rate.
         */
        outSnapshot.deltaSeconds = runtime.timing.valid ? runtime.timing.deltaSeconds : 0.0f;
        if (auto* playerNodes = f4vr::getPlayerNodes();
            playerNodes && playerNodes->HmdNode &&
            finiteNiTransform(playerNodes->HmdNode->world)) {
            fillProviderTransform(
                playerNodes->HmdNode->world,
                outSnapshot.hmdTransform);
            outSnapshot.flags |= static_cast<std::uint32_t>(
                ::rock::provider::PublicationFlag::HmdTransformValid);
            const auto rawForward =
                playerNodes->HmdNode->world.rotate.Transpose() *
                RE::NiPoint3(0.0f, 1.0f, 0.0f);
            RE::NiPoint3 normalizedForward{};
            if (selection_query_policy::tryNormalizeVectorForHmdCone(
                    rawForward,
                    normalizedForward)) {
                outSnapshot.hmdForwardWorld[0] = normalizedForward.x;
                outSnapshot.hmdForwardWorld[1] = normalizedForward.y;
                outSnapshot.hmdForwardWorld[2] = normalizedForward.z;
                outSnapshot.flags |= static_cast<std::uint32_t>(
                    ::rock::provider::PublicationFlag::HmdForwardValid);
            }
        }
        outSnapshot.primaryHand = _twoHandedGrip.isFiringHandLeft() ?
            api::Hand::Left :
            api::Hand::Right;
        outSnapshot.offhandHand =
            outSnapshot.primaryHand == api::Hand::Left ?
                api::Hand::Right :
                api::Hand::Left;
        outSnapshot.collisionGeneration =
            _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire);

        outSnapshot.weaponFormId = currentEquippedWeaponFormId();
        auto& weapon = outSnapshot.weapon;
        for (auto& bodyId : weapon.bodyIds) {
            bodyId = api::grab::kNoBody;
        }
        const auto weaponSnapshot = _weaponCollision.getWeaponBodySnapshotAtomic();
        weapon.weaponGenerationKey = weaponSnapshot.generationKey;
        weapon.bodyCount = (std::min)(weaponSnapshot.count, static_cast<std::uint32_t>(std::size(weapon.bodyIds)));
        for (std::uint32_t i = 0; i < weapon.bodyCount; ++i) {
            weapon.bodyIds[i] = weaponSnapshot.bodyIds[i];
        }

        // Providers get the controller hand ROCK interacts with, not the
        // rendered bone (ROCK's previous claim while a claim is active).
        RE::NiTransform providerHandWorld{};
        if (outSnapshot.providerReady && runtime.localSkeletonReady && _handBoneCache.isReady() && frik_hand_world_authority::tryGetRawHandWorld(false, providerHandWorld) && finiteNiTransform(providerHandWorld)) {
            fillProviderTransform(providerHandWorld, outSnapshot.rightHandTransform);
            outSnapshot.flags |= static_cast<std::uint32_t>(
                ::rock::provider::PublicationFlag::RightHandTransformValid);
        }
        if (outSnapshot.providerReady && runtime.localSkeletonReady && _handBoneCache.isReady() && frik_hand_world_authority::tryGetRawHandWorld(true, providerHandWorld) && finiteNiTransform(providerHandWorld)) {
            fillProviderTransform(providerHandWorld, outSnapshot.leftHandTransform);
            outSnapshot.flags |= static_cast<std::uint32_t>(
                ::rock::provider::PublicationFlag::LeftHandTransformValid);
        }

        outSnapshot.handReservation = ::rock::provider::currentHandReservation(outSnapshot.offhandHand);
    }

    bool PhysicsInteraction::isProviderWeaponBodyCurrent(
        const std::uint64_t weaponGenerationKey,
        const std::uint32_t bodyId) const
    {
        // Collider focus originates from the complete evidence catalog, while
        // the frame snapshot intentionally carries only a compact body prefix.
        if (weaponGenerationKey == 0 || bodyId == 0x7FFF'FFFF) {
            return false;
        }
        const auto snapshot =
            _weaponCollision.getWeaponBodySnapshotAtomic();
        if (snapshot.generationKey != weaponGenerationKey) {
            return false;
        }
        for (std::uint32_t index = 0;
             index < snapshot.count;
             ++index) {
            if (snapshot.bodyIds[index] == bodyId) {
                return true;
            }
        }
        return false;
    }

    bool PhysicsInteraction::queryProviderWorldRaycast(
        const api::collision::WorldRaycastRequest& request,
        api::collision::WorldRaycastResult& outResult) const
    {
        const RE::NiPoint3 start{
            request.startGame.x,
            request.startGame.y,
            request.startGame.z,
        };
        RE::NiPoint3 direction{
            request.directionGame.x,
            request.directionGame.y,
            request.directionGame.z,
        };
        const float directionLengthSquared =
            direction.x * direction.x +
            direction.y * direction.y +
            direction.z * direction.z;
        if (!std::isfinite(directionLengthSquared) ||
            directionLengthSquared <= 1.0e-8f) {
            return false;
        }
        const float inverseDirectionLength =
            1.0f / std::sqrt(directionLengthSquared);
        direction.x *= inverseDirectionLength;
        direction.y *= inverseDirectionLength;
        direction.z *= inverseDirectionLength;

        const RE::NiPoint3 end{
            start.x + direction.x * request.maxDistanceGame,
            start.y + direction.y * request.maxDistanceGame,
            start.z + direction.z * request.maxDistanceGame,
        };

        physics_ray_cast::ClosestSegmentResult rayResult{};
        if (!physics_ray_cast::castClosestSegment(
                _lifecycle.cachedBhkWorld,
                start,
                end,
                selection_query_policy::kFarClipRayFilterInfo,
                rayResult)) {
            return false;
        }

        outResult.hit = rayResult.hit ? 1u : 0u;
        outResult.flags = rayResult.hit ?
            static_cast<std::uint32_t>(
                api::collision::WorldRaycastResultFlag::Hit) :
            0u;
        outResult.hitFraction =
            rayResult.hit ? rayResult.hitFraction : 1.0f;
        outResult.hitDistanceGame =
            outResult.hitFraction * request.maxDistanceGame;
        outResult.hitPointGame = {
            start.x + direction.x * outResult.hitDistanceGame,
            start.y + direction.y * outResult.hitDistanceGame,
            start.z + direction.z * outResult.hitDistanceGame,
        };
        if (rayResult.normalValid) {
            outResult.flags |= static_cast<std::uint32_t>(
                api::collision::WorldRaycastResultFlag::
                    NormalValid);
            outResult.hitNormalGame = {
                rayResult.normalGame.x,
                rayResult.normalGame.y,
                rayResult.normalGame.z,
            };
        }
        outResult.worldGeneration =
            _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        outResult.skeletonGeneration =
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        outResult.providerGeneration =
            _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }
    void PhysicsInteraction::fillProviderWeaponPartGripStates(
        std::array<api::weaponparts::GripState, 2>& outStates) const
    {
        // Indexed [right, left] to match the getter's hand mapping. Published
        // after the grip update each frame, so consumers always read this
        // frame's grip decision alongside the frame snapshot.
        for (const bool isLeft : { false, true }) {
            auto& outState = outStates[isLeft ? 1u : 0u];
            outState = {};
            outState.hand = isLeft ? api::Hand::Left : api::Hand::Right;

            HandGripReport report{};
            _twoHandedGrip.getHandGripReport(isLeft, report);
            outState.gripKind = static_cast<api::weaponparts::GripKind>(report.kind);
            outState.active = report.active ? 1u : 0u;
            outState.attachOnly = report.attachOnly ? 1u : 0u;
            outState.authoredSupportGrip =
                report.authoredSupportGrip ? 1u : 0u;
            outState.gripSequence = report.gripSequence;
            outState.weaponGenerationKey = report.weaponGenerationKey;
            outState.bodyId = report.bodyId;
            outState.partKind = report.partKind;
            outState.reloadRole = report.reloadRole;
            outState.supportRole = report.supportRole;
            outState.socketRole = report.socketRole;
            outState.actionRole = report.actionRole;
            outState.sourceKey = providerWeaponSourceKey(report.weaponGenerationKey, report.sourceRoot);
            outState.targetOwner = report.providerOwnerToken;
            outState.targetGroupId = report.providerGroupId;
            outState.targetGrabMode = static_cast<api::weaponparts::GrabMode>(report.providerGrabMode);
            outState.hasHandPartLocal = report.hasHandPartLocal ? 1u : 0u;
            outState.handPartLocalSpace = report.handPartLocalIsSourceLocal ?
                api::weaponparts::GripLocalSpace::PartSourceLocal :
                api::weaponparts::GripLocalSpace::WeaponRootLocal;
            fillProviderTransform(report.handPartLocal, outState.handPartLocal);
            static_assert(sizeof(outState.sourceName) == std::tuple_size_v<decltype(report.sourceName)>);
            std::memcpy(outState.sourceName, report.sourceName.data(), sizeof(outState.sourceName));
            outState.sourceName[sizeof(outState.sourceName) - 1] = '\0';
            outState.omodFormId = report.omodFormId;
            outState.attachPointFormId = report.attachPointFormId;
            outState.classificationSource = report.classificationSource;
        }
    }

    bool PhysicsInteraction::queryProviderWeaponContactAtPoint(
        const api::weaponparts::ContactQuery& query,
        api::weaponparts::ContactResult& outResult) const
    {
        outResult = {};
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        if (!weaponNode) {
            return false;
        }

        WeaponInteractionContact contact{};
        const RE::NiPoint3 point{ query.pointGame.x, query.pointGame.y, query.pointGame.z };
        if (!_weaponCollision.tryFindInteractionContactNearPoint(weaponNode, point, query.radiusGame, contact)) {
            return false;
        }

        outResult.valid = contact.valid ? 1u : 0u;
        outResult.bodyId = contact.bodyId;
        outResult.partKind = static_cast<std::uint32_t>(contact.partKind);
        outResult.reloadRole = static_cast<std::uint32_t>(contact.reloadRole);
        outResult.supportRole = static_cast<std::uint32_t>(contact.supportGripRole);
        outResult.socketRole = static_cast<std::uint32_t>(contact.socketRole);
        outResult.actionRole = static_cast<std::uint32_t>(contact.actionRole);
        outResult.interactionKey = providerWeaponSourceKey(
            contact.weaponGenerationKey, reinterpret_cast<std::uintptr_t>(contact.interactionRoot));
        outResult.sourceKey = providerWeaponSourceKey(
            contact.weaponGenerationKey, reinterpret_cast<std::uintptr_t>(contact.sourceRoot));
        outResult.weaponGenerationKey = contact.weaponGenerationKey;
        outResult.probeDistanceGame = contact.probeDistanceGame;
        return contact.valid;
    }

    bool PhysicsInteraction::queryProviderWeaponClassification(api::weapon::Classification& outResult) const
    {
        outResult = {};
        const auto identity = _weaponCollision.getEquippedWeaponClassification();
        outResult.valid = identity.hasEquippedWeapon &&
                identity.classificationResolved ?
            1u :
            0u;
        outResult.weaponFormId = identity.formID;
        outResult.keywordFlags = identity.keywordFlags;
        outResult.sizeClass = static_cast<api::weapon::SizeClass>(identity.sizeClass);
        outResult.source = static_cast<api::weapon::ClassificationSource>(identity.classificationSource);
        outResult.weaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        using Source = WeaponClassificationSource;
        using Provenance =
            api::weapon::ClassificationProvenanceFlag;
        if (identity.keywordFlags != 0) {
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::KeywordEvidence);
        }
        if (identity.usedEffectiveInstanceKeywordData) {
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::EffectiveInstanceKeywordEvidence);
        }
        switch (identity.classificationSource) {
        case Source::Keyword:
            outResult.confidence = 1.0f;
            break;
        case Source::WeaponData:
            outResult.confidence = 1.0f;
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::WeaponDataEvidence);
            break;
        case Source::EquipSlot:
            outResult.confidence = 0.85f;
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::EquipSlotEvidence);
            break;
        default:
            outResult.confidence = 0.0f;
            break;
        }
        if (outResult.weaponGenerationKey != 0) {
            outResult.provenanceFlags |= static_cast<std::uint32_t>(
                Provenance::GenerationBound);
        }
        return identity.hasEquippedWeapon;
    }

    bool PhysicsInteraction::queryProviderWeaponGrip(
        api::weapon::Grip& outState) const
    {
        using Flag = api::weapon::GripFlag;

        outState = {};
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        if (!_lifecycle.initialized.load(std::memory_order_acquire) || !weaponNode) {
            return false;
        }

        outState.flags = static_cast<std::uint32_t>(Flag::Valid);
        outState.weaponFormId = currentEquippedWeaponFormId();
        outState.weaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();

        if (_twoHandedGrip.ownsWeaponTransform()) {
            outState.flags |= static_cast<std::uint32_t>(Flag::WeaponTransformOwned);
        }

        RE::NiTransform weaponWorld{};
        const bool solvedWeaponWorldValid =
            _twoHandedGrip.getSolvedWeaponTransform(weaponWorld) &&
            finiteNiTransform(weaponWorld);
        if (!solvedWeaponWorldValid && finiteNiTransform(weaponNode->world)) {
            weaponWorld = weaponNode->world;
        }
        if (solvedWeaponWorldValid || finiteNiTransform(weaponNode->world)) {
            fillProviderTransform(weaponWorld, outState.weaponWorld);
            outState.flags |= static_cast<std::uint32_t>(Flag::WeaponWorldValid);
        }

        RE::NiTransform rightHandInWeapon{};
        RE::NiTransform leftHandInWeapon{};
        if (_twoHandedGrip.getManualCycleRockGripBaselines(
                rightHandInWeapon,
                leftHandInWeapon)) {
            fillProviderTransform(rightHandInWeapon, outState.rightHandInWeapon);
            fillProviderTransform(leftHandInWeapon, outState.leftHandInWeapon);
            outState.flags |=
                static_cast<std::uint32_t>(Flag::RightHandInWeaponValid) |
                static_cast<std::uint32_t>(Flag::LeftHandInWeaponValid);
        }

        /*
         * EquippedWeaponData::fireNode is Bethesda's equip-time projectile
         * origin. It is valid before muzzle-flash presentation state exists,
         * so publishing it makes the muzzle snapshot available as soon as the
         * weapon is equipped instead of after its first shot.
         */
        if (const auto* projectileNode =
                getEquippedProjectileNode();
            projectileNode &&
            finiteNiTransform(projectileNode->world)) {
            const auto& projectileWorld = projectileNode->world;
            RE::NiPoint3 muzzleDirection{
                projectileWorld.rotate.entry[1][0],
                projectileWorld.rotate.entry[1][1],
                projectileWorld.rotate.entry[1][2],
            };
            const float directionLength = muzzleDirection.Length();
            if (std::isfinite(directionLength) &&
                directionLength > 0.000001f) {
                muzzleDirection /= directionLength;
                outState.muzzleOriginGame =
                    makeProviderPoint(projectileWorld.translate);
                outState.muzzleDirectionGame =
                    makeProviderPoint(muzzleDirection);
                outState.flags |=
                    static_cast<std::uint32_t>(Flag::MuzzleWorldValid);
            }
        }

        return true;
    }

    bool PhysicsInteraction::queryProviderWeaponHandling(
        ::rock::provider_state_policy::WeaponHandling& outState) const
    {
        outState = {};
        if (!_lifecycle.initialized.load(std::memory_order_acquire) ||
            !runtime_state::currentFrame().visualAuthorityAvailable ||
            !runtime_state::currentFrame().localSkeletonReady) return false;
        outState.firingHand = _twoHandedGrip.isFiringHandLeft() ?
            api::Hand::Left :
            api::Hand::Right;
        outState.weaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        outState.weaponFormId = currentEquippedWeaponFormId();
        if (outState.weaponFormId == 0 || outState.weaponFormId !=
            _weaponCollision.getCurrentObservedEquippedWeaponFormID()) outState.weaponGenerationKey = 0;
        outState.leftFiringAvailable = TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        outState.manualOwnership = _twoHandedGrip.isManualOwnershipActive();
        outState.partCarry = _twoHandedGrip.isPartCarryActive();
        outState.twoHandGrip = _twoHandedGrip.getState() == TwoHandedState::Gripping;
        if (currentEquippedWeaponOccupiesHand() && resolveEquippedWeaponInteractionNode()) {
            outState.weaponPresent = true;
            // Inventory/holster equips can remain in Inactive or Touching:
            // the native right hand still occupies the firing grip. The
            // internal occupancy predicate counts explicit ROCK grips only;
            // PartCarry is the state that actually vacates the firing grip.
            outState.firingGripOccupied = provider_state_policy::firingGripOccupied(
                true, outState.partCarry, _twoHandedGrip.isHandsFreeRetained());
        }
        return true;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidenceDetails(
        api::weaponparts::EvidenceDetail* outDetails,
        std::uint32_t maxDetails) const
    {
        if (!outDetails || maxDetails == 0) {
            return 0;
        }

        const auto descriptors = _weaponCollision.getProfileEvidenceDescriptors();
        const std::uint32_t count = (std::min)(
            (std::min)(
                maxDetails,
                api::weaponparts::kMaxEvidenceDetails),
            static_cast<std::uint32_t>(descriptors.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = descriptors[i];
            auto& out = outDetails[i];
            out = {};
            out.bodyId = descriptor.bodyId;
            out.partKind = static_cast<std::uint32_t>(descriptor.semantic.partKind);
            out.reloadRole = static_cast<std::uint32_t>(descriptor.semantic.reloadRole);
            out.supportRole = static_cast<std::uint32_t>(descriptor.semantic.supportGripRole);
            out.socketRole = static_cast<std::uint32_t>(descriptor.semantic.socketRole);
            out.actionRole = static_cast<std::uint32_t>(descriptor.semantic.actionRole);
            out.fallbackGripPose = static_cast<std::uint32_t>(descriptor.semantic.fallbackGripPose);
            out.interactionKey = providerWeaponSourceKey(descriptor.weaponGenerationKey, descriptor.geometryRootAddress);
            out.sourceKey = providerWeaponSourceKey(descriptor.weaponGenerationKey, descriptor.sourceRootAddress);
            out.weaponGenerationKey = descriptor.weaponGenerationKey;
            out.localBoundsGame.min = makeProviderPoint(descriptor.localBoundsGame.min);
            out.localBoundsGame.max = makeProviderPoint(descriptor.localBoundsGame.max);
            out.localBoundsGame.valid = descriptor.localBoundsGame.valid ? 1u : 0u;
            out.pointCount = descriptor.pointCount;
            copyProviderString(out.sourceName, sizeof(out.sourceName), descriptor.sourceName);
            out.omodFormId = descriptor.omodFormId;
            out.attachPointFormId = descriptor.semantic.attachPointFormId;
            out.classificationSource = static_cast<std::uint32_t>(descriptor.semantic.classificationSource);
        }

        return count;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEvidencePoints(
        std::uint32_t bodyId,
        api::Point3* outPoints,
        std::uint32_t maxPoints) const
    {
        if (!outPoints || maxPoints == 0) {
            return 0;
        }

        const auto evidence = _weaponCollision.getProfileEvidenceDescriptors();
        const auto* descriptor = evidence.find(bodyId);
        if (!descriptor) {
            return 0;
        }

        const std::uint32_t copied = (std::min)(maxPoints, static_cast<std::uint32_t>(descriptor->localMeshPointsGame.size()));
        for (std::uint32_t i = 0; i < copied; ++i) {
            outPoints[i] = makeProviderPoint(descriptor->localMeshPointsGame[i]);
        }

        return copied;
    }

    std::uint32_t PhysicsInteraction::getProviderWeaponEmitterCount() const
    {
        const auto snapshot = _weaponCollision.getWeaponEmitterSnapshot();
        return static_cast<std::uint32_t>((std::min)(snapshot.count, snapshot.emitters.size()));
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponEmitters(
        api::weapon::Emitter* outEmitters,
        std::uint32_t maxEmitters) const
    {
        if (!outEmitters || maxEmitters == 0) {
            return 0;
        }

        static_assert(MAX_WEAPON_EMITTERS == api::weapon::kMaxEmitters);
        const auto snapshot = _weaponCollision.getWeaponEmitterSnapshot();
        const std::uint32_t count = (std::min)(maxEmitters,
            static_cast<std::uint32_t>((std::min)(snapshot.count, snapshot.emitters.size())));
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = snapshot.emitters[i];
            auto& out = outEmitters[i];
            out = {};
            out.kind = static_cast<api::weapon::EmitterKind>(descriptor.kind);
            out.source = static_cast<api::weapon::EmitterSource>(descriptor.source);
            out.flags = static_cast<std::uint32_t>(api::weapon::EmitterFlag::TransformValid) |
                static_cast<std::uint32_t>(api::weapon::EmitterFlag::DirectionValid);
            if (descriptor.effectStateKnown) {
                out.flags |= static_cast<std::uint32_t>(api::weapon::EmitterFlag::EffectStateKnown);
            }
            if (descriptor.hasAddOnNodeValue) {
                out.flags |= static_cast<std::uint32_t>(api::weapon::EmitterFlag::HasAddOnNodeValue);
            }
            if (descriptor.omodFormId != 0) {
                out.flags |= static_cast<std::uint32_t>(api::weapon::EmitterFlag::HasOmod);
            }
            if (descriptor.attachPointFormId != 0) {
                out.flags |= static_cast<std::uint32_t>(api::weapon::EmitterFlag::HasAttachPoint);
            }
            out.active = descriptor.active ? 1u : 0u;
            out.visible = descriptor.visible ? 1u : 0u;
            out.addOnNodeValue = descriptor.addOnNodeValue;
            out.omodFormId = descriptor.omodFormId;
            out.attachPointFormId = descriptor.attachPointFormId;
            out.weaponGenerationKey = descriptor.weaponGenerationKey;
            std::copy(descriptor.rotate.begin(), descriptor.rotate.end(), out.weaponLocalTransform.rotate);
            std::copy(descriptor.translate.begin(), descriptor.translate.end(), out.weaponLocalTransform.translate);
            out.weaponLocalTransform.scale = descriptor.scale;
            out.forwardWeaponLocal = {
                descriptor.forwardWeaponLocal[0],
                descriptor.forwardWeaponLocal[1],
                descriptor.forwardWeaponLocal[2],
            };
            static_assert(sizeof(out.sourceName) == std::tuple_size_v<decltype(descriptor.sourceName)>);
            std::memcpy(out.sourceName, descriptor.sourceName.data(), sizeof(out.sourceName));
            out.sourceName[sizeof(out.sourceName) - 1] = '\0';
        }
        return count;
    }

    std::uint32_t PhysicsInteraction::copyProviderBodyContacts(
        api::collision::BodyContact* outContacts,
        std::uint32_t maxContacts) const
    {
        if (!outContacts || maxContacts == 0) {
            return 0;
        }

        std::array<body_contact_runtime::BodyContactRecord, body_contact_runtime::kMaxBodyContactRecords> records{};
        const auto requested = (std::min)(static_cast<std::size_t>(maxContacts), records.size());
        const auto copied = _contacts.bodyRuntime.snapshot(records.data(), requested);
        for (std::size_t i = 0; i < copied; ++i) {
            const auto& record = records[i];
            auto& out = outContacts[i];
            out = {};
            out.frameIndex = record.frame;
            out.bodyId = record.bodyId;
            out.targetBodyId = record.targetBodyId;
            out.bodyLayer = record.bodyLayer;
            out.targetLayer = record.targetLayer;
            out.zone = static_cast<api::collision::BodyZoneKind>(record.zone);
            out.side = static_cast<api::collision::BodyZoneSide>(record.side);
            out.role = static_cast<std::uint32_t>(record.role);
            out.descriptorIndex = record.descriptorIndex;
            out.targetKind = providerBodyContactTargetKind(record.targetKind);
            out.inPowerArmor = record.inPowerArmor ? 1u : 0u;
            out.hasContactPointGame = record.hasContactPointGame ? 1u : 0u;
            out.contactPointGame = makeProviderPoint(record.contactPointGame);
        }

        return static_cast<std::uint32_t>(copied);
    }

    void PhysicsInteraction::releaseProviderPowerArmorGrabs(const std::uint64_t ownerToken)
    {
        auto* bhkWorld = getPlayerBhkWorld();
        auto* world = bhkWorld ? getHknpWorld(bhkWorld) : nullptr;
        _touchGrabRuntime.releaseCommandOwner(ownerToken, bhkWorld, world,
            _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire));
    }

    bool PhysicsInteraction::getProviderHandTargetDetails(
        const provider::HandInteractionPublication& handState,
        api::grab::HandTargetDetails& out) const
    {
        using Flag = api::grab::TargetDetailFlag;
        out = {};
        const bool isLeft = handState.hand == api::Hand::Left;
        out.handState = handState;
        TouchGrabRuntime::HandReport touch{};
        if (_touchGrabRuntime.getHandReport(isLeft, touch)) {
            if (touch.hasSurfaceAnchor) {
                out.anchorGame = makeProviderPoint(touch.surfaceAnchorGame);
                out.flags |= static_cast<std::uint32_t>(Flag::Anchor);
            }
            if (touch.hasNormal) {
                out.normalGame = makeProviderPoint(touch.normalGame);
                out.flags |= static_cast<std::uint32_t>(Flag::Normal);
            }
            out.powerArmorPoint = touch.powerArmorPoint;
            out.sourceTriangleIndex = touch.sourceTriangleIndex;
            std::copy(std::begin(touch.meshPartName), std::end(touch.meshPartName), std::begin(out.meshPartName));
            if (out.sourceTriangleIndex != 0xFFFF'FFFFu) out.flags |= static_cast<std::uint32_t>(Flag::MeshPart);
        }
        auto* world = _lifecycle.cachedHknpWorld;
        const auto body = havok_runtime::snapshotBody(world, RE::hknpBodyId{out.handState.primaryBodyId});
        RE::TESObjectREFR* ref = nullptr;
        if (body.valid) {
            out.collisionLayer = body.collisionFilterInfo & 0x7Fu;
            out.flags |= static_cast<std::uint32_t>(Flag::Body);
            if (auto* node = havok_runtime::getOwnerNodeFromBody(body.body); node && node->name.c_str())
                strncpy_s(out.collisionNodeName, node->name.c_str(), _TRUNCATE);
            ref = reference_interaction::resolveBody(world, body.bodyId.value);
        }
        if (!ref && out.handState.targetFormId != 0)
            ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(out.handState.targetFormId);
        if (api::grab::ReferenceInteraction reference{}; reference_interaction::describe(ref, reference)) {
            out.flags |= static_cast<std::uint32_t>(Flag::Reference);
            out.referenceFormId = reference.referenceFormId;
            out.referenceNativeHandle = reference.referenceNativeHandle;
        }
        return true;
    }

    void PhysicsInteraction::fillProviderHandInteractionStates(
        std::array<provider::HandInteractionPublication, 2>& outStates) const
    {
        using Phase =
            api::grab::HandInteractionPhase;
        using Flag =
            api::grab::HandInteractionFlag;
        using TargetKind =
            api::grab::BodyContactTargetKind;

        const auto phaseForState = [](const HandState state) {
            switch (state) {
            case HandState::SelectedClose:
            case HandState::SelectedFar:
                return Phase::Selecting;
            case HandState::PrePullItem:
            case HandState::Pulled:
                return Phase::Pulling;
            case HandState::SelectionLocked:
            case HandState::PreGrabItem:
            case HandState::HeldInit:
            case HandState::GrabFromOtherHand:
            case HandState::GrabExternal:
                return Phase::Catching;
            case HandState::HeldBody:
            case HandState::SelectedTwoHand:
            case HandState::HeldTwoHanded:
            case HandState::LootOtherHand:
                return Phase::Holding;
            case HandState::StashCandidate:
                return Phase::StashCandidate;
            case HandState::ConsumeCandidate:
                return Phase::ConsumeCandidate;
            default:
                return Phase::Idle;
            }
        };
        const auto targetKindForSelection = [](const SelectedObject& selection) {
            return grab_target::isExtensionSelection(selection.targetKind) ?
                TargetKind::Actor :
                TargetKind::DynamicProp;
        };

        const auto worldGeneration =
            _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        const auto skeletonGeneration =
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        const auto providerGeneration =
            _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        const auto collisionGeneration =
            _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire);
        const bool primaryIsLeft = _twoHandedGrip.isFiringHandLeft();
        ::rock::provider_state_policy::WeaponHandling handling{};
        const bool stateAvailable = queryProviderWeaponHandling(handling);
        std::array<api::weaponparts::GripState, 2> grips{};
        if (stateAvailable) fillProviderWeaponPartGripStates(grips);

        // The touched object's form, or 0 when the hand is not touching or the
        // body no longer resolves to a reference.
        auto touchedFormId = [&](const Hand& hand) -> std::uint32_t {
            const std::uint32_t bodyId = hand.getTouchedBodyId();
            if (bodyId == hand_semantic_contact_state::kInvalidBodyId ||
                !_lifecycle.cachedBhkWorld || !_lifecycle.cachedHknpWorld) {
                return 0;
            }
            auto* ref = resolveBodyToRef(_lifecycle.cachedBhkWorld, _lifecycle.cachedHknpWorld, RE::hknpBodyId{ bodyId });
            return ref ? ref->GetFormID() : 0;
        };

        for (const bool isLeft : { false, true }) {
            const Hand& hand = isLeft ? _leftHand : _rightHand;
            auto& state = outStates[isLeft ? 1u : 0u];
            state = {};
            state.hand = isLeft ?
                api::Hand::Left :
                api::Hand::Right;
            state.worldGeneration = worldGeneration;
            state.skeletonGeneration = skeletonGeneration;
            state.providerGeneration = providerGeneration;
            state.collisionGeneration = collisionGeneration;
            if (!stateAvailable) continue;
            TouchGrabRuntime::HandReport touchGrabReport{};
            const bool touchGrabActive =
                _touchGrabRuntime.getHandReport(
                    isLeft,
                    touchGrabReport);
            state.phase = touchGrabActive ?
                Phase::Holding :
                phaseForState(hand.getState());
            const std::uint32_t touchedForm = touchedFormId(hand);
            if (!touchGrabActive &&
                state.phase == Phase::Idle && touchedForm != 0) {
                state.phase = Phase::Touching;
            }
            state.flags = static_cast<std::uint32_t>(Flag::Valid);
            if (isLeft == primaryIsLeft) {
                state.flags |= static_cast<std::uint32_t>(Flag::Primary);
            } else {
                state.flags |= static_cast<std::uint32_t>(Flag::Offhand);
            }

            if (touchGrabActive) {
                state.reservedTargetIdentity =
                    touchGrabReport.referenceNativeHandle;
                state.targetFormId =
                    touchGrabReport.referenceFormId;
                state.primaryBodyId = touchGrabReport.bodyId;
                state.targetKind =
                    touchGrabReport.kind ==
                            ::rock::api::touch::TouchGrabKind::FixedAnchor ?
                    TargetKind::WorldSurface :
                    TargetKind::DynamicProp;
                state.flags |= static_cast<std::uint32_t>(
                    Flag::TouchGrab);
                if (touchGrabReport.kind ==
                    ::rock::api::touch::TouchGrabKind::FixedAnchor) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::FixedSurfaceLatch);
                }
                if (touchGrabReport.globalSurface) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::GlobalSurfaceLatch);
                }
                state.surfaceGripMode =
                    touchGrabReport.surfaceGripMode;
                if (touchGrabReport.hasSurfaceAnchor) {
                    state.surfaceAnchorGame = {
                        touchGrabReport.surfaceAnchorGame.x,
                        touchGrabReport.surfaceAnchorGame.y,
                        touchGrabReport.surfaceAnchorGame.z
                    };
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::SurfaceAnchorValid);
                }
                if (touchGrabReport.surfaceGripMode ==
                    ::rock::api::touch::SurfaceGripMode::
                        MeshAnchor) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::MeshSurfaceAnchor);
                } else if (touchGrabReport.surfaceGripMode ==
                           ::rock::api::touch::SurfaceGripMode::
                                   CollisionFallback) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::MeshCollisionFallback);
                }
            } else if (hand.isHolding()) {
                auto* heldRef = hand.getHeldRef();
                state.targetFormId = heldRef ? heldRef->GetFormID() : 0;
                state.primaryBodyId = hand.getSavedObjectState().bodyId.value;
                state.targetKind = TargetKind::HeldObject;
                state.flags |= static_cast<std::uint32_t>(Flag::LooseObject);
                if (hand.isHoldingLooseWeapon()) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::LooseWeapon);
                }
            } else if (hand.hasSelection()) {
                const auto& selection = hand.getSelection();
                state.targetFormId =
                    selection.refr ? selection.refr->GetFormID() : 0;
                state.primaryBodyId = selection.bodyId.value;
                state.targetKind = targetKindForSelection(selection);
                if (grab_target::isPhysicalRockObject(selection.targetKind)) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::LooseObject);
                }
            } else if (touchedForm != 0) {
                state.targetFormId = touchedForm;
                state.targetKind = TargetKind::DynamicProp;
            }

            const auto& heldBodies = hand.getHeldBodyIds();
            state.heldBodyCount = static_cast<std::uint32_t>((std::min)(
                heldBodies.size(),
                std::size(state.heldBodyIds)));
            for (std::uint32_t i = 0; i < state.heldBodyCount; ++i) {
                state.heldBodyIds[i] = heldBodies[i];
            }
            if (heldBodies.size() > state.heldBodyCount) {
                state.flags |= static_cast<std::uint32_t>(
                    Flag::HeldBodyListTruncated);
            }

            provider_state_policy::applyWeaponOccupancy(
                state, handling, grips[isLeft ? 1u : 0u]);

            state.effectiveInputSuppressionFlags =
                ::rock::provider::currentHandInputSuppressionFlags(
                    state.hand);
            if (input_remap_runtime::ownsBareFistInput()) {
                state.effectiveInputSuppressionFlags |= static_cast<std::uint32_t>(
                    ::rock::api::input::HandInputSuppressionFlag::SuppressConfigModeChord);
            }
            if (state.effectiveInputSuppressionFlags != 0) {
                state.flags |= static_cast<std::uint32_t>(
                    Flag::InputSuppressed);
            }
            api::collision::HandCollisionAvailability
                collisionState{};
            if (queryProviderHandCollisionAvailability(
                    state.hand,
                    collisionState)) {
                state.collisionAvailabilityFlags = collisionState.flags;
                if ((collisionState.flags & static_cast<std::uint32_t>(
                         api::collision::HandCollisionAvailabilityFlag::CollisionAvailable)) != 0) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::CollisionAvailable);
                }
                if ((collisionState.flags & static_cast<std::uint32_t>(
                         api::collision::HandCollisionAvailabilityFlag::TransitionSuppressed)) != 0) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::TransitionSuppressed);
                }
                if ((collisionState.flags & static_cast<std::uint32_t>(
                         api::collision::HandCollisionAvailabilityFlag::DynamicOtherHandContact)) != 0) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::DynamicOtherHandContact);
                }
                if ((collisionState.flags & static_cast<std::uint32_t>(
                         api::collision::HandCollisionAvailabilityFlag::DynamicWeaponContact)) != 0) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::DynamicWeaponContact);
                }
                if ((collisionState.flags & static_cast<std::uint32_t>(
                         api::collision::HandCollisionAvailabilityFlag::DynamicWeaponPairSuppressed)) != 0) {
                    state.flags |= static_cast<std::uint32_t>(
                        Flag::DynamicWeaponPairSuppressed);
                }
            }
            state.worldGeneration = worldGeneration;
            state.skeletonGeneration = skeletonGeneration;
            state.providerGeneration = providerGeneration;
            state.collisionGeneration = collisionGeneration;
        }
    }

    bool PhysicsInteraction::queryProviderWeaponState(
        api::weapon::WeaponState& outState) const
    {
        using Flag =
            api::weapon::WeaponStateFlag;
        outState = {};
        const auto transition = _equipped.transition.getPublicSnapshot();
        outState.weaponFormId = currentEquippedWeaponFormId();
        outState.transitionWeaponFormId = transition.active ? transition.weaponFormID : 0;
        outState.terminalWeaponFormId = transition.terminalWeaponFormID;
        outState.weaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        outState.transitionSequence = transition.transitionSequence;
        outState.terminalSequence = transition.terminalSequence;
        const auto publicSource = [](EquippedWeaponTransitionCoordinator::Source source) {
            using Source = api::weapon::TransitionSource;
            switch (source) {
            case EquippedWeaponTransitionCoordinator::Source::ObservedEquip: return Source::ObservedEquip;
            case EquippedWeaponTransitionCoordinator::Source::HeldTriggerEquip: return Source::HeldTriggerEquip;
            case EquippedWeaponTransitionCoordinator::Source::HeldGripZoneEquip: return Source::HeldGripZoneEquip;
            case EquippedWeaponTransitionCoordinator::Source::MenuExit: return Source::MenuExit;
            case EquippedWeaponTransitionCoordinator::Source::WorkbenchExit: return Source::WorkbenchExit;
            }
            return Source::Unknown;
        };
        if (transition.active) outState.transitionSource = publicSource(transition.source);
        if (transition.terminalSequence != 0) outState.terminalSource = publicSource(transition.terminalSource);
        // Internal and public terminal enums have different numeric layouts.
        using Terminal = api::weapon::TransitionResult;
        using InternalTerminal = EquippedWeaponTransitionCoordinator::TerminalResult;
        switch (transition.terminalResult) {
        case InternalTerminal::None: outState.terminalResult = Terminal::None; break;
        case InternalTerminal::Completed: outState.terminalResult = Terminal::Completed; break;
        case InternalTerminal::WeaponUnequipped: outState.terminalResult = Terminal::WeaponUnequipped; break;
        case InternalTerminal::IdentityLost: outState.terminalResult = Terminal::IdentityLost; break;
        case InternalTerminal::ExpectedIdentityTimeout: outState.terminalResult = Terminal::ExpectedIdentityTimeout; break;
        case InternalTerminal::NativeAnimationHandoff: outState.terminalResult = Terminal::NativeAnimationHandoff; break;
        case InternalTerminal::WeaponNoLongerDrawn: outState.terminalResult = Terminal::WeaponNoLongerDrawn; break;
        case InternalTerminal::IntentionalShoulderSheathe: outState.terminalResult = Terminal::IntentionalShoulderSheathe; break;
        case InternalTerminal::RecoveryExhausted: outState.terminalResult = Terminal::RecoveryExhausted; break;
        case InternalTerminal::ProviderLost: outState.terminalResult = Terminal::ProviderLost; break;
        case InternalTerminal::Shutdown: outState.terminalResult = Terminal::Shutdown; break;
        }
        if (_lifecycle.initialized.load(std::memory_order_acquire) &&
            runtime_state::currentFrame().visualAuthorityAvailable &&
            runtime_state::currentFrame().localSkeletonReady) {
            outState.flags |= static_cast<std::uint32_t>(Flag::Valid);
        }
        if (outState.weaponFormId == 0 || outState.weaponFormId !=
            _weaponCollision.getCurrentObservedEquippedWeaponFormID()) outState.weaponGenerationKey = 0;
        const auto setFlag = [&outState](const Flag flag, const bool enabled) {
            if (enabled) {
                outState.flags |= static_cast<std::uint32_t>(flag);
            }
        };
        setFlag(Flag::IdentityPending, transition.identityPending);
        setFlag(Flag::DrawPending, transition.drawPending);
        setFlag(Flag::BridgePresented, transition.bridgePresented);
        const bool presentationKnown = transition.presentationKnown &&
            transition.presentationWeaponFormID == outState.weaponFormId &&
            (outState.flags & static_cast<std::uint32_t>(Flag::Valid));
        setFlag(Flag::NativeRenderable, presentationKnown && transition.nativeRenderable);
        setFlag(
            Flag::HandPoseHandoffComplete,
            presentationKnown && transition.handPoseHandoffComplete);
        setFlag(Flag::RecoveryExhausted, transition.recoveryExhausted);
        setFlag(Flag::TransitionActive, transition.active);
        setFlag(Flag::WeaponEquipped, outState.weaponFormId != 0);
        setFlag(Flag::WeaponPresent, outState.weaponFormId != 0 && resolveEquippedWeaponInteractionNode());
        setFlag(Flag::WeaponDrawn, outState.weaponFormId != 0 && runtime_state::currentFrame().weaponDrawn);
        setFlag(Flag::PresentationKnown, presentationKnown);
        setFlag(Flag::BipodInputReserved, g_rockConfig.rockBipodMode &&
            (outState.flags & static_cast<std::uint32_t>(Flag::Valid)) &&
            _dynamicWeaponCollision.surfaceSupportReservesInput(
                reinterpret_cast<std::uintptr_t>(resolveEquippedWeaponInteractionNode()), outState.weaponGenerationKey));
        outState.sample.worldGeneration =
            _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        outState.sample.skeletonGeneration =
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        outState.sample.providerGeneration =
            _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        return (outState.flags & static_cast<std::uint32_t>(Flag::Valid)) != 0 ||
               transition.transitionSequence != 0 ||
               transition.terminalSequence != 0;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponPartPoses(
        api::weaponparts::PartPose* outParts,
        const std::uint32_t maxParts) const
    {
        if (!outParts || maxParts == 0) {
            return 0;
        }
        using Flag = api::weaponparts::PoseFlag;
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        const auto currentGeneration =
            _weaponCollision.getCurrentWeaponGenerationKey();
        if (!weaponNode || currentGeneration == 0 ||
            !finiteNiTransform(weaponNode->world)) {
            return 0;
        }

        const auto descriptors =
            _weaponCollision.getProfileEvidenceDescriptors();
        const auto count = static_cast<std::uint32_t>((std::min)(
            static_cast<std::size_t>(maxParts),
            descriptors.size()));
        const auto weaponInverse =
            transform_math::invertTransform(weaponNode->world);
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& descriptor = descriptors[i];
            auto& out = outParts[i];
            out = {};
            out.frameIndex =
                _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
            out.weaponGenerationKey = descriptor.weaponGenerationKey;
            out.sourceKey = providerWeaponSourceKey(descriptor.weaponGenerationKey, descriptor.sourceRootAddress);
            out.bodyId = descriptor.bodyId;
            out.partKind = static_cast<std::uint32_t>(
                descriptor.semantic.partKind);
            out.omodFormId = descriptor.omodFormId;
            out.attachPointFormId = descriptor.semantic.attachPointFormId;
            copyProviderString(
                out.sourceName,
                sizeof(out.sourceName),
                descriptor.sourceName);
            if (!descriptor.valid ||
                descriptor.weaponGenerationKey != currentGeneration) {
                continue;
            }
            out.flags |= static_cast<std::uint32_t>(Flag::Valid);
            out.actionRole = static_cast<std::uint32_t>(
                descriptor.semantic.actionRole);
            auto* sourceNode = reinterpret_cast<RE::NiAVObject*>(
                descriptor.sourceRootAddress);
            if (!sourceNode || !finiteNiTransform(sourceNode->world)) {
                continue;
            }
            if (sourceNode->parent && finiteNiTransform(sourceNode->local)) {
                fillProviderTransform(
                    sourceNode->local,
                    out.sourceParentLocal);
                out.flags |= static_cast<std::uint32_t>(
                    Flag::SourceParentLocalValid);
            }
            const auto weaponRootLocal = transform_math::composeTransforms(
                weaponInverse,
                sourceNode->world);
            if (finiteNiTransform(weaponRootLocal)) {
                fillProviderTransform(
                    weaponRootLocal,
                    out.weaponRootLocal);
                out.flags |= static_cast<std::uint32_t>(
                    Flag::WeaponRootLocalValid);
            }
        }
        return count;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponPartDriveResults(
        const std::uint64_t ownerToken,
        api::weaponparts::DriveResult* outResults,
        const std::uint32_t maxResults) const
    {
        if (ownerToken == 0 || !outResults || maxResults == 0) {
            return 0;
        }
        std::uint32_t copied = 0;
        const auto count = (std::min)(
            _providerDrives.resultCount,
            static_cast<std::uint32_t>(
                _providerDrives.results.size()));
        for (std::uint32_t i = 0; i < count && copied < maxResults; ++i) {
            const auto& result = _providerDrives.results[i];
            if (result.owner == ownerToken) {
                outResults[copied++] = result;
            }
        }
        return copied;
    }

    bool PhysicsInteraction::queryProviderScopeSightState(
        api::weapon::ScopeSightState& outState) const
    {
        using Flag = api::weapon::ScopeSightFlag;
        outState = {};
        const auto anchor =
            _weaponCollision.getNativeScopeSightAnchorSnapshot();
        if (anchor.weaponGenerationKey == 0 || anchor.weaponFormID == 0) {
            return false;
        }

        const NativeScopeResolvedAnchorSnapshot resolvedAnchor =
            _twoHandedGrip.getNativeScopeResolvedAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity
            resolvedIdentity{
                .weaponGenerationKey = resolvedAnchor.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    resolvedAnchor.equippedWeaponOwnershipKey,
                .weaponFormID = resolvedAnchor.weaponFormID,
            };
        const native_scope_sight_anchor_policy::PublicationIdentity
            publishedIdentity{
                .weaponGenerationKey = anchor.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    anchor.equippedWeaponOwnershipKey,
                .weaponFormID = anchor.weaponFormID,
            };
        const bool resolvedAnchorMatchesPublication =
            resolvedAnchor.valid &&
            native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(
                resolvedIdentity,
                publishedIdentity);

        outState.weaponGenerationKey = anchor.weaponGenerationKey;
        outState.weaponFormId = anchor.weaponFormID;
        outState.nativeScopeOverlayIndex = anchor.nativeScopeOverlayIndex;
        outState.sightBodyCount = anchor.sightBodyCount;
        outState.anchorWeaponLocal = makeProviderPoint(
            resolvedAnchorMatchesPublication ?
                resolvedAnchor.anchorWeaponLocal :
                anchor.anchorWeaponLocal);
        outState.sightBoundsWeaponLocal.min = makeProviderPoint(
            anchor.sightBoundsMinWeaponLocal);
        outState.sightBoundsWeaponLocal.max = makeProviderPoint(
            anchor.sightBoundsMaxWeaponLocal);
        outState.sightBoundsWeaponLocal.valid = anchor.valid ? 1u : 0u;
        outState.flags |= static_cast<std::uint32_t>(Flag::Available);
        if (resolvedAnchorMatchesPublication || anchor.valid) {
            outState.flags |= static_cast<std::uint32_t>(Flag::AnchorValid);
        }
        if (anchor.valid) {
            outState.flags |= static_cast<std::uint32_t>(Flag::BoundsValid);
        }
        if (anchor.nativeScopeOverlayValid) {
            outState.flags |= static_cast<std::uint32_t>(
                Flag::NativeOverlayValid);
        }
        if (anchor.manualDirectTransitionRequired &&
            resolvedAnchorMatchesPublication) {
            outState.flags |= static_cast<std::uint32_t>(
                Flag::ManualDirectTransitionRequired);
        }

        const bool menuOpen = _twoHandedGrip.isScopeMenuOpenThisFrame();
        if (menuOpen) {
            outState.flags |= static_cast<std::uint32_t>(Flag::MenuOpen);
        }
        const auto activation =
            _twoHandedGrip.getNativeScopeActivationDebugSnapshot();
        outState.sample.publicationSequence = activation.publicationSequence;
        const bool active = activation.rendererStateValid ?
                                activation.rendererActive :
                                menuOpen;
        if (active &&
            activation.weaponGenerationKey == anchor.weaponGenerationKey) {
            outState.flags |= static_cast<std::uint32_t>(Flag::Active);
            outState.activationSource = g_rockConfig.rockEnableImmersiveScopes ?
                api::weapon::ScopeActivationSource::ManualInput :
                api::weapon::ScopeActivationSource::NativeGeometry;
        }

        const auto descriptors =
            _weaponCollision.getProfileEvidenceDescriptors();
        for (const auto& descriptor : descriptors) {
            if (!descriptor.valid ||
                descriptor.weaponGenerationKey != anchor.weaponGenerationKey ||
                (descriptor.semantic.partKind != WeaponPartKind::Scope &&
                    descriptor.semantic.partKind != WeaponPartKind::Sight)) {
                continue;
            }
            outState.sightBodyId = descriptor.bodyId;
            outState.omodFormId = descriptor.omodFormId;
            outState.attachPointFormId =
                descriptor.semantic.attachPointFormId;
            if (descriptor.semantic.partKind == WeaponPartKind::Scope) {
                break;
            }
        }
        const auto composition =
            _weaponCollision.getWeaponCompositionSnapshot();
        if (outState.sample.publicationSequence == 0) {
            outState.sample.publicationSequence = composition.publicationSequence;
        }
        outState.sample.frameIndex =
            _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
        outState.sample.worldGeneration =
            _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        outState.sample.skeletonGeneration =
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        outState.sample.providerGeneration =
            _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }

    bool PhysicsInteraction::queryProviderWeaponCompositionState(
        api::weapon::CompositionState& outState) const
    {
        outState = {};
        const auto snapshot =
            _weaponCollision.getWeaponCompositionSnapshot();
        if (snapshot.weaponGenerationKey == 0) {
            return false;
        }
        outState.weaponGenerationKey = snapshot.weaponGenerationKey;
        outState.compositionSignature = snapshot.compositionSignature;
        outState.weaponFormId = snapshot.weaponFormId;
        outState.entryCount = snapshot.entryCount;
        outState.semanticCoverageMask = snapshot.semanticCoverageMask;
        outState.missingCoverageMask = snapshot.missingCoverageMask;
        outState.publicationSequence = snapshot.publicationSequence;
        return true;
    }

    std::uint32_t PhysicsInteraction::copyProviderWeaponCompositionEntries(
        api::weapon::CompositionEntry* outEntries,
        const std::uint32_t maxEntries) const
    {
        if (!outEntries || maxEntries == 0) {
            return 0;
        }
        const auto snapshot =
            _weaponCollision.getWeaponCompositionSnapshot();
        const auto count = (std::min)(
            maxEntries,
            static_cast<std::uint32_t>((std::min)(
                static_cast<std::size_t>(snapshot.entryCount),
                snapshot.entries.size())));
        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& source = snapshot.entries[i];
            auto& destination = outEntries[i];
            destination = {};
            destination.omodFormId = source.omodFormId;
            destination.attachPointFormId = source.attachPointFormId;
            destination.stableIndex = source.stableIndex;
            destination.flags = source.flags;
            destination.semanticCoverageMask = source.semanticCoverageMask;
        }
        return count;
    }

    bool PhysicsInteraction::queryProviderAuthoredGripPose(
        api::weapon::AuthoredGripPose& outPose) const
    {
        using Flag = api::weapon::AuthoredGripPoseFlag;
        outPose = {};
        SelectedAuthoredGripPoseSnapshot snapshot{};
        if (!_twoHandedGrip.getSelectedAuthoredGripPoseSnapshot(snapshot) ||
            !snapshot.valid) {
            return false;
        }
        outPose.weaponGenerationKey = snapshot.weaponGenerationKey;
        outPose.weaponFormId = currentEquippedWeaponFormId();
        outPose.source =
            static_cast<api::weapon::AuthoredGripSource>(
                snapshot.source);
        outPose.variantKey = snapshot.variantKey;
        outPose.captureSequence = snapshot.captureSequence;
        outPose.flags = static_cast<std::uint32_t>(Flag::Valid);
        if (snapshot.rightHandValid) {
            fillProviderTransform(
                snapshot.rightHandWeaponLocal,
                outPose.rightHandInWeapon);
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::RightHandValid);
        }
        if (snapshot.leftHandValid) {
            fillProviderTransform(
                snapshot.leftHandWeaponLocal,
                outPose.leftHandInWeapon);
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::LeftHandValid);
        }
        outPose.rightFingerLocalTransformMask =
            snapshot.rightFingerLocalTransformMask;
        outPose.leftFingerLocalTransformMask =
            snapshot.leftFingerLocalTransformMask;
        for (std::size_t i = 0;
             i < snapshot.rightFingerLocalTransforms.size();
             ++i) {
            if ((snapshot.rightFingerLocalTransformMask & (1u << i)) != 0) {
                fillProviderTransform(
                    snapshot.rightFingerLocalTransforms[i],
                    outPose.rightFingerLocalTransforms[i]);
            }
            if ((snapshot.leftFingerLocalTransformMask & (1u << i)) != 0) {
                fillProviderTransform(
                    snapshot.leftFingerLocalTransforms[i],
                    outPose.leftFingerLocalTransforms[i]);
            }
        }
        if (snapshot.rightFingerLocalTransformMask != 0) {
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::RightFingersValid);
        }
        if (snapshot.leftFingerLocalTransformMask != 0) {
            outPose.flags |= static_cast<std::uint32_t>(
                Flag::LeftFingersValid);
        }
        outPose.worldGeneration =
            _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        outPose.skeletonGeneration =
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        outPose.providerGeneration =
            _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }

    bool PhysicsInteraction::queryProviderPresentedHandPose(
        const ::rock::api::Hand hand,
        ::rock::api::hands::PresentedHandPose& outPose,
        ::rock::provider::Publication* outMetadata) const
    {
        outPose = {};
        if (hand != api::Hand::Left && hand != api::Hand::Right) return false;
        const auto& captured = _providerDrives.presentedPoses[hand == api::Hand::Left ? 1u : 0u];
        if (!frik_visual_authority::isSkeletonReadyHint() || captured.frameIndex == 0 ||
            captured.worldGeneration != _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire) ||
            captured.skeletonGeneration != _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire) ||
            captured.providerGeneration != _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire)) return false;
        outPose = captured;
        if (outMetadata) *outMetadata = _providerDrives.presentedMetadata;
        return true;
    }

    void PhysicsInteraction::captureProviderPresentedHandPoses()
    {
        using Flag = ::rock::api::hands::PresentedHandPoseFlag;
        _providerDrives.presentedPoses = {};
        _providerDrives.presentedMetadata = {};
        _finalPoseBoneSnapshot.valid = false;
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return;
        }
        auto& skeleton = _finalPoseBoneSnapshot;
        if (!_finalPoseBoneReader.capture(
                skeleton_bone_debug_math::DebugSkeletonBoneMode::AllFlattenedBones,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                SkeletonBoneCaptureSpace::Rendered, skeleton)) {
            return;
        }
        const auto bones = _providerDrives.presentedPoseNames.bind(skeleton);
        // Freeze the control publication paired with this capture. An early
        // query next frame must not join an old pose to new interaction state.
        if (!provider::runtime::copyPublication(_providerDrives.presentedMetadata) ||
            _providerDrives.presentedMetadata.frameIndex != runtime_state::currentFrame().frameIndex) return;
        for (const bool isLeft : { false, true }) {
            auto& outPose = _providerDrives.presentedPoses[isLeft ? 1u : 0u];
            const auto* wrist = bones.find(isLeft ? "LArm_Hand" : "RArm_Hand");
            if (!wrist || !finiteNiTransform(wrist->world)) continue;
            outPose.hand = isLeft ? api::Hand::Left : api::Hand::Right;
            outPose.flags =
                static_cast<std::uint32_t>(Flag::Valid) |
                static_cast<std::uint32_t>(Flag::HandWorldValid);
            fillProviderTransform(wrist->world, outPose.handWorld);

            {
                for (std::size_t finger = 0; finger < 5; ++finger) {
                    for (std::size_t segment = 0; segment < 3; ++segment) {
                        const auto poseIndex = finger * 3 + segment;
                        const auto* bone = bones.find(
                            root_flattened_finger_skeleton_runtime::fingerBoneName(
                                isLeft,
                                finger,
                                segment));
                        if (!bone || bone->drawableParentSnapshotIndex < 0 ||
                            static_cast<std::size_t>(
                                bone->drawableParentSnapshotIndex) >=
                                skeleton.bones.size()) {
                            continue;
                        }
                        const auto& parent = skeleton.bones[
                            static_cast<std::size_t>(
                                bone->drawableParentSnapshotIndex)];
                        const auto local = transform_math::composeTransforms(
                            transform_math::invertTransform(parent.world),
                            bone->world);
                        if (!finiteNiTransform(local)) {
                            continue;
                        }
                        fillProviderTransform(
                            local,
                            outPose.fingerLocalTransforms[poseIndex]);
                        outPose.fingerLocalTransformMask |= 1u << poseIndex;
                    }
                }
                if (outPose.fingerLocalTransformMask != 0) {
                    outPose.flags |=
                        static_cast<std::uint32_t>(Flag::FingerLocalsValid) |
                        static_cast<std::uint32_t>(Flag::RootFlattenedReadback);
                }
            }
            outPose.worldGeneration =
                _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
            outPose.skeletonGeneration =
                _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
            outPose.providerGeneration =
                _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
            outPose.frameIndex = runtime_state::currentFrame().frameIndex;
            outPose.presentationSequence = outPose.frameIndex;
        }
    }

    std::uint32_t PhysicsInteraction::copyProviderSemanticHandContacts(
        const api::Hand handValue,
        const std::uint32_t maxFramesSinceContact,
        api::collision::SemanticHandContact* outContacts,
        const std::uint32_t maxContacts) const
    {
        if (!outContacts || maxContacts == 0) {
            return 0;
        }
        using Flag =
            api::collision::SemanticHandContactFlag;
        const bool isLeft =
            handValue == api::Hand::Left;
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        api::collision::HandCollisionAvailability
            availability{};
        (void)queryProviderHandCollisionAvailability(
            handValue,
            availability);
        const bool collisionAvailable =
            (availability.flags & static_cast<std::uint32_t>(
                 api::collision::HandCollisionAvailabilityFlag::CollisionAvailable)) != 0;
        const bool transitionSuppressed =
            (availability.flags & static_cast<std::uint32_t>(
                 api::collision::HandCollisionAvailabilityFlag::TransitionSuppressed)) != 0;

        std::uint32_t copied = 0;
        for (const auto role : hand_collider_semantics::kHandColliderRoles) {
            if (copied >= maxContacts) {
                break;
            }
            hand_semantic_contact_state::SemanticContactRecord record{};
            if (!hand.getFreshSemanticContactForRole(
                    role,
                    maxFramesSinceContact,
                    record)) {
                continue;
            }
            auto& out = outContacts[copied++];
            out = {};
            out.frameIndex =
                _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
            out.hand = handValue;
            out.role = static_cast<std::uint32_t>(record.role);
            out.finger = static_cast<std::uint32_t>(record.finger);
            out.segment = static_cast<std::uint32_t>(record.segment);
            out.handBodyId = record.handBodyId;
            out.targetBodyId = record.otherBodyId;
            if (record.framesSinceContact != 0) {
                out.contactState =
                    api::collision::SemanticContactState::End;
            } else if (record.contactRunStartFrame == record.contactFrame) {
                out.contactState =
                    api::collision::SemanticContactState::Begin;
            } else {
                out.contactState =
                    api::collision::SemanticContactState::Continued;
            }
            out.framesSinceContact = record.framesSinceContact;
            out.contactSequence = record.sequence;
            if (record.hasContactPointGame) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::ContactPointValid);
                out.contactPointGame = {
                    record.contactPointGame.x,
                    record.contactPointGame.y,
                    record.contactPointGame.z,
                };
            }
            if (record.hasContactNormalGame) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::ContactNormalValid);
                out.contactNormalGame = {
                    record.contactNormalGame.x,
                    record.contactNormalGame.y,
                    record.contactNormalGame.z,
                };
            }
            if (_lifecycle.cachedBhkWorld && _lifecycle.cachedHknpWorld) {
                if (auto* target = resolveBodyToRef(
                        _lifecycle.cachedBhkWorld,
                        _lifecycle.cachedHknpWorld,
                        RE::hknpBodyId{ record.otherBodyId })) {
                    out.targetFormId = target->GetFormID();
                    out.flags |= static_cast<std::uint32_t>(
                        Flag::TargetFormResolved);
                }
            }
            if (hand.isHeldBodyId(record.otherBodyId)) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::HeldObjectRelation);
            }
            if (collisionAvailable) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::CollisionAvailable);
            }
            if (transitionSuppressed) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::TransitionSuppressed);
            }
            out.collisionGeneration =
                _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire);
        }
        return copied;
    }

    std::uint32_t PhysicsInteraction::copyProviderPlayerColliderDescriptors(
        api::collision::PlayerColliderDescriptor* outDescriptors,
        const std::uint32_t maxDescriptors) const
    {
        if (!outDescriptors || maxDescriptors == 0) {
            return 0;
        }
        using Kind = api::collision::PlayerColliderKind;
        using Flag = api::collision::PlayerColliderFlag;
        const auto frameIndex =
            _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto collisionGeneration =
            _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire);
        const auto lifecycleFlags =
            _lifecycle.flagsAtomic.load(std::memory_order_acquire);
        const bool enabled =
            ::rock::provider::hasLifecycleFlag(
                lifecycleFlags,
                api::core::LifecycleFlag::GeneratedBodiesValid) &&
            ::rock::provider::hasLifecycleFlag(
                lifecycleFlags,
                api::core::LifecycleFlag::PhysicsWriteAllowed);
        const bool bodiesCurrent = ::rock::provider::hasLifecycleFlag(lifecycleFlags,
            api::core::LifecycleFlag::GeneratedBodiesValid);
        std::uint32_t copied = 0;

        const auto copyHand = [&](const Hand& hand, const bool isLeft) {
            const auto bodyCount = hand.getHandColliderBodyCount();
            for (std::uint32_t i = 0;
                 i < bodyCount && copied < maxDescriptors;
                 ++i) {
                const auto bodyId = hand.getHandColliderBodyIdAtomic(i);
                HandColliderBodyMetadata metadata{};
                if (!hand.tryGetHandColliderMetadataAtIndex(i, bodyId, metadata)) {
                    continue;
                }
                auto& out = outDescriptors[copied++];
                out = {};
                out.frameIndex = frameIndex;
                out.kind = Kind::Hand;
                out.hand = isLeft ?
                    api::Hand::Left :
                    api::Hand::Right;
                out.bodyId = bodyId;
                out.role = static_cast<std::uint32_t>(metadata.role);
                out.descriptorIndex = i;
                const bool handDisabled = isLeft ? s_leftHandDisabled.load(std::memory_order_acquire) :
                    s_rightHandDisabled.load(std::memory_order_acquire);
                out.flags = providerColliderFlags(_lifecycle.cachedHknpWorld, bodyId,
                    bodiesCurrent, enabled && !handDisabled);
                if (metadata.primaryPalmAnchor) {
                    out.flags |= static_cast<std::uint32_t>(
                        Flag::PrimaryPalmAnchor);
                    RE::NiTransform target{};
                    if (hand.tryGetPalmAnchorTarget(target) &&
                        finiteNiTransform(target)) {
                        fillProviderTransform(target, out.transform);
                        out.flags |= static_cast<std::uint32_t>(
                            Flag::TransformValid);
                    }
                }
                out.collisionGeneration = collisionGeneration;
            }
        };
        copyHand(_rightHand, false);
        copyHand(_leftHand, true);

        const auto bodyCount = _bodyBoneColliders.getBodyCount();
        for (std::uint32_t i = 0;
             i < bodyCount && copied < maxDescriptors;
             ++i) {
            const auto bodyId = _bodyBoneColliders.getBodyIdAtomic(i);
            BodyBoneColliderMetadata metadata{};
            if (!_bodyBoneColliders.tryGetBodyMetadataAtIndexAtomic(
                    i, bodyId,
                    metadata)) {
                continue;
            }
            auto& out = outDescriptors[copied++];
            out = {};
            out.frameIndex = frameIndex;
            out.kind = Kind::Body;
            out.bodyId = bodyId;
            out.role = static_cast<std::uint32_t>(metadata.role);
            out.zone = static_cast<api::collision::BodyZoneKind>(
                metadata.zone);
            out.side = static_cast<api::collision::BodyZoneSide>(
                metadata.side);
            out.descriptorIndex = metadata.descriptorIndex;
            out.lengthGameUnits = metadata.lengthGameUnits;
            out.radiusGameUnits = metadata.radiusGameUnits;
            out.flags = providerColliderFlags(_lifecycle.cachedHknpWorld, bodyId, bodiesCurrent, enabled);
            if (metadata.inPowerArmor) {
                out.flags |= static_cast<std::uint32_t>(
                    Flag::InPowerArmor);
            }
            out.collisionGeneration = collisionGeneration;
        }
        return copied;
    }

    bool PhysicsInteraction::queryProviderHandCollisionAvailability(
        const api::Hand handValue,
        api::collision::HandCollisionAvailability& outState) const
    {
        using Flag =
            api::collision::HandCollisionAvailabilityFlag;
        outState = {};
        const bool isLeft =
            handValue == api::Hand::Left;
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        outState.hand = handValue;
        outState.frameIndex =
            _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
        outState.collisionGeneration =
            _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire);
        outState.handBodyCount = hand.getHandColliderBodyCount();
        const auto lifecycleFlags = _lifecycle.flagsAtomic.load(std::memory_order_acquire);
        const bool bodiesCurrent = ::rock::provider::hasLifecycleFlag(lifecycleFlags,
            api::core::LifecycleFlag::GeneratedBodiesValid);
        const bool writesAllowed = ::rock::provider::hasLifecycleFlag(lifecycleFlags,
            api::core::LifecycleFlag::PhysicsWriteAllowed);
        const bool handDisabled = isLeft ? s_leftHandDisabled.load(std::memory_order_acquire) :
            s_rightHandDisabled.load(std::memory_order_acquire);
        if (writesAllowed) outState.flags |= static_cast<std::uint32_t>(Flag::PhysicsWritesAllowed);
        if (handDisabled) outState.flags |= static_cast<std::uint32_t>(Flag::HandDisabled);
        if (::rock::provider::hasLifecycleFlag(lifecycleFlags, api::core::LifecycleFlag::MenuBlocking))
            outState.flags |= static_cast<std::uint32_t>(Flag::MenuSuppressed);
        using ColliderFlag = api::collision::PlayerColliderFlag;
        for (std::uint32_t index = 0; index < outState.handBodyCount; ++index) {
            const auto flags = providerColliderFlags(_lifecycle.cachedHknpWorld,
                hand.getHandColliderBodyIdAtomic(index), bodiesCurrent, writesAllowed && !handDisabled);
            if (flags & static_cast<std::uint32_t>(ColliderFlag::FilterKnown)) ++outState.filterKnownBodyCount;
            if (flags & static_cast<std::uint32_t>(ColliderFlag::Enabled)) ++outState.collisionEnabledBodyCount;
            if (flags & static_cast<std::uint32_t>(ColliderFlag::CollisionSuppressed))
                outState.flags |= static_cast<std::uint32_t>(Flag::CollisionSuppressed);
        }
        if (outState.handBodyCount != 0 && outState.filterKnownBodyCount == outState.handBodyCount)
            outState.flags |= static_cast<std::uint32_t>(Flag::FilterKnown);
        if (outState.collisionEnabledBodyCount != 0)
            outState.flags |= static_cast<std::uint32_t>(Flag::CollisionAvailable);
        if (outState.handBodyCount != 0) {
            outState.flags |= static_cast<std::uint32_t>(Flag::BodiesReady);
        }

        dynamic_hand_collision_telemetry::Snapshot telemetry{};
        if (_dynamicHandCollision.getTelemetrySnapshot(telemetry)) {
            const auto& handTelemetry = telemetry.hands[isLeft ? 1u : 0u];
            outState.collisionSequence = telemetry.updateSequence;
            for (const auto& twin : handTelemetry.twins) {
                if (twin.bodyCreated) {
                    ++outState.dynamicTwinCount;
                }
            }
            if (outState.dynamicTwinCount == handTelemetry.twins.size()) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::DynamicTwinsReady);
            }
            if (telemetry.transitionCollisionSuppressed) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::TransitionSuppressed);
            }
            if (handTelemetry.dynamicInteractionsEnabled) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::DynamicInteractionsEnabled);
            }
            if (handTelemetry.pairFilterReady) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::DynamicPairFilterReady);
            }
            if (handTelemetry.otherHandContactMask != 0) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::DynamicOtherHandContact);
            }
            if (handTelemetry.weaponContactMask != 0) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::DynamicWeaponContact);
            }
            if (handTelemetry.weaponPairSuppressed) {
                outState.flags |= static_cast<std::uint32_t>(
                    Flag::DynamicWeaponPairSuppressed);
            }
            outState.otherHandContactSlots = handTelemetry.otherHandContactMask;
            outState.weaponContactSlots = handTelemetry.weaponContactMask;
            outState.dynamicCollisionLayer = handTelemetry.dynamicInteractionLayer;
            outState.pairSuppressionLeaseCount =
                handTelemetry.suppressedWeaponPairCount;
        }
        outState.worldGeneration =
            _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        outState.skeletonGeneration =
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        outState.providerGeneration =
            _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        return true;
    }
