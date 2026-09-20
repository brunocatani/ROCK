// Private Hands service operations. Included after shared owner/lifecycle helpers.

    bool ROCK_PROVIDER_CALL apiGetHandFrameV1(RockProviderHand hand, RockProviderHandFrameV1* outFrame)
    {
        provider_state_policy::clearQueryOutput(outFrame, 112);
        /*
         * Hand frames expose ROCK's hand authority as a value snapshot instead
         * of a NiNode lookup. Consumers need the same primary/offhand mapping,
         * body id, and root-flattened transform that ROCK drives each frame,
         * while ROCK deliberately does not promise a live scene node for that
         * authority surface.
         */
        if (!outFrame || outFrame->size < 112) {
            return false;
        }
        const auto requestedSize = outFrame->size;

        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return false;
        }

        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot) {
                return false;
            }
            snapshot = s_lastSnapshot;
        }

        const auto transformFlag = hand == RockProviderHand::Left ?
            RockProviderFrameEnrichmentFlagV1::LeftHandTransformValid :
            RockProviderFrameEnrichmentFlagV1::RightHandTransformValid;
        if (!apiIsProviderReady() || snapshot.providerReady == 0 ||
            !(snapshot.enrichmentFlags & static_cast<std::uint32_t>(transformFlag))) {
            return false;
        }

        const bool isLeft = hand == RockProviderHand::Left;
        RockProviderHandFrameV1 frame{};
        frame.hand = hand;
        frame.flags = static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Valid) |
                      static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::RootFlattenedAuthority);
        if (isLeft) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Left);
        }
        if (hand == snapshot.primaryHand) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Primary);
        }
        if (hand == snapshot.offhandHand) {
            frame.flags |= static_cast<std::uint32_t>(RockProviderHandFrameFlagV1::Offhand);
        }

        frame.transform = isLeft ? snapshot.leftHandTransform : snapshot.rightHandTransform;
        frame.bodyId = isLeft ? snapshot.leftHandBodyId : snapshot.rightHandBodyId;
        frame.state = isLeft ? snapshot.leftHandState : snapshot.rightHandState;
        frame.frameIndex = snapshot.frameIndex;
        frame.worldGeneration = snapshot.worldGeneration;
        frame.skeletonGeneration = snapshot.skeletonGeneration;
        frame.providerGeneration = snapshot.providerGeneration;
        frame.collisionGeneration = snapshot.collisionGeneration;
        frame.stateSequence = snapshot.stateSequence;
        const auto copySize = (std::min<std::size_t>)(
            requestedSize,
            sizeof(frame));
        std::memcpy(outFrame, &frame, copySize);
        outFrame->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    bool ROCK_PROVIDER_CALL apiGetPresentedHandFrameV1(
        const RockProviderHand hand,
        RockProviderHandFrameV1* outFrame)
    {
        provider_state_policy::clearQueryOutput(outFrame, 112);
        if (!outFrame ||
            outFrame->size < 112 ||
            (hand != RockProviderHand::Right &&
                hand != RockProviderHand::Left) ||
            !onAnimationOwnerThread() ||
            !apiIsProviderReady() ||
            !frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return false;
        }

        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        RockProviderPresentedHandPoseV1 pose{};
        if (!pi || !pi->isInitialized() || !pi->queryProviderPresentedHandPoseV1(hand, pose)) {
            return false;
        }
        const auto& providerTransform = pose.handWorld;
        if (!finiteProviderTransform(providerTransform)) {
            return false;
        }

        const auto requestedSize = outFrame->size;
        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (s_hasSnapshot) {
                snapshot = s_lastSnapshot;
            }
        }
        RockProviderHandFrameV1 frame{};
        frame.hand = hand;
        frame.flags = static_cast<std::uint32_t>(
                          RockProviderHandFrameFlagV1::Valid) |
                      static_cast<std::uint32_t>(
                          RockProviderHandFrameFlagV1::PresentedVisual);
        if (hand == RockProviderHand::Left) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Left);
        }
        if (hand == snapshot.primaryHand) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Primary);
        }
        if (hand == snapshot.offhandHand) {
            frame.flags |= static_cast<std::uint32_t>(
                RockProviderHandFrameFlagV1::Offhand);
        }
        frame.transform = providerTransform;
        frame.state = hand == RockProviderHand::Left ? snapshot.leftHandState : snapshot.rightHandState;
        frame.bodyId = hand == RockProviderHand::Left ? snapshot.leftHandBodyId : snapshot.rightHandBodyId;
        frame.frameIndex = pose.frameIndex;
        frame.worldGeneration = pose.worldGeneration;
        frame.skeletonGeneration = pose.skeletonGeneration;
        frame.providerGeneration = pose.providerGeneration;
        frame.collisionGeneration = snapshot.collisionGeneration;
        frame.stateSequence = snapshot.stateSequence;
        const auto copySize = (std::min<std::size_t>)(
            requestedSize,
            sizeof(frame));
        std::memcpy(outFrame, &frame, copySize);
        outFrame->size = static_cast<std::uint32_t>(copySize);
        return true;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiGetPresentedHandPoseV1(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        RockProviderPresentedHandPoseV1* outPose)
    {
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            provider_state_policy::clearQueryOutput(outPose);
            return RockProviderResultV1::HandUnavailable;
        }
        const auto result = queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PoseReadback,
            outPose,
            [hand](PhysicsInteraction& pi, RockProviderPresentedHandPoseV1& pose) {
                return pi.queryProviderPresentedHandPoseV1(hand, pose);
            },
            true);
        return result;
    }
