// Private Hands service operations. Included after shared owner/lifecycle helpers.
// They fill provider API major-2 Hands records directly; the endpoint has
// already validated and cleared each output record.

    inline api::Sample handsSample(const RockProviderFrameSnapshot& snapshot) noexcept
    {
        return { snapshot.frameIndex, snapshot.stateSequence, snapshot.worldGeneration,
            snapshot.skeletonGeneration, snapshot.providerGeneration, snapshot.collisionGeneration };
    }

    inline void copyHandsTransform(const RockProviderTransform& source, api::Transform& target) noexcept
    {
        std::copy_n(source.rotate, 9, target.rotate);
        std::copy_n(source.translate, 3, target.translate);
        target.scale = source.scale;
    }

    inline bool finiteHandsTransform(const api::Transform& value) noexcept
    {
        for (const float component : value.rotate) {
            if (!std::isfinite(component)) return false;
        }
        for (const float component : value.translate) {
            if (!std::isfinite(component)) return false;
        }
        return std::isfinite(value.scale);
    }

    api::Status apiGetHandFrame(const api::Hand hand, api::hands::HandFrame& outFrame)
    {
        /*
         * Hand frames expose ROCK's hand authority as a value snapshot instead
         * of a NiNode lookup: the root-flattened transform ROCK drives each
         * frame, without promising a live scene node for it.
         */
        if (hand != api::Hand::Right && hand != api::Hand::Left) {
            return api::Status::InvalidArgument;
        }
        RockProviderFrameSnapshot snapshot{};
        {
            std::scoped_lock lock(s_snapshotMutex);
            if (!s_hasSnapshot) {
                return api::Status::NotReady;
            }
            snapshot = s_lastSnapshot;
        }

        const bool isLeft = hand == api::Hand::Left;
        const auto transformFlag = isLeft ?
            RockProviderFrameEnrichmentFlagV1::LeftHandTransformValid :
            RockProviderFrameEnrichmentFlagV1::RightHandTransformValid;
        if (!apiIsProviderReady() || snapshot.providerReady == 0 ||
            !(snapshot.enrichmentFlags & static_cast<std::uint32_t>(transformFlag))) {
            return api::Status::NotReady;
        }

        using Flag = api::hands::HandFrameFlag;
        outFrame.hand = hand;
        outFrame.flags = static_cast<std::uint32_t>(Flag::Valid) |
                         static_cast<std::uint32_t>(Flag::RootFlattenedAuthority) |
                         (isLeft ? static_cast<std::uint32_t>(Flag::Left) : 0u);
        copyHandsTransform(isLeft ? snapshot.leftHandTransform : snapshot.rightHandTransform, outFrame.transform);
        outFrame.sample = handsSample(snapshot);
        return api::Status::Ok;
    }

    api::Status apiGetPresentedHandFrame(const api::Hand hand, api::hands::HandFrame& outFrame)
    {
        if (hand != api::Hand::Right && hand != api::Hand::Left) {
            return api::Status::InvalidArgument;
        }
        if (!onAnimationOwnerThread()) {
            return api::Status::WrongThread;
        }
        if (!apiIsProviderReady() ||
            !frik_visual_authority::isAvailable() ||
            !frik_visual_authority::isSkeletonReadyHint()) {
            return api::Status::NotReady;
        }

        const auto access = s_physicsInteraction.borrow();
        auto* pi = access.get();
        api::hands::PresentedHandPose pose{};
        RockProviderFrameSnapshot snapshot{};
        if (!pi || !pi->isInitialized() || !pi->queryProviderPresentedHandPose(hand, pose, &snapshot) ||
            !finiteHandsTransform(pose.handWorld)) {
            return api::Status::NotReady;
        }

        // The presentation keeps its own frame and generations; the control
        // publication frozen with that capture supplies the state sequence.
        using Flag = api::hands::HandFrameFlag;
        outFrame.hand = hand;
        outFrame.flags = static_cast<std::uint32_t>(Flag::Valid) |
                         static_cast<std::uint32_t>(Flag::PresentedVisual) |
                         (hand == api::Hand::Left ? static_cast<std::uint32_t>(Flag::Left) : 0u);
        outFrame.transform = pose.handWorld;
        outFrame.sample = { pose.frameIndex, snapshot.stateSequence, pose.worldGeneration,
            pose.skeletonGeneration, pose.providerGeneration, snapshot.collisionGeneration };
        return api::Status::Ok;
    }

    api::Status apiGetPresentedHandPose(
        const std::uint64_t ownerToken,
        const api::Hand hand,
        api::hands::PresentedHandPose& outPose)
    {
        if (hand != api::Hand::Right && hand != api::Hand::Left) {
            return api::Status::HandUnavailable;
        }
        return static_cast<api::Status>(queryPhysicsInteractionValueV1(
            ownerToken,
            RockProviderConsumerCapabilityV1::PoseReadback,
            &outPose,
            [hand](PhysicsInteraction& pi, api::hands::PresentedHandPose& pose) {
                return pi.queryProviderPresentedHandPose(hand, pose);
            },
            true));
    }
