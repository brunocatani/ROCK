Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    return boundary::readSample(owner, kInterfaceId, outSample);
}

Status ROCK_CALL getHeadPose(OwnerToken owner, HeadPoseV1* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample = provider::runtime::sample();
        result->valid = (frame.enrichmentFlags >> 1) & 3u;
        convert(result->transform, frame.hmdTransform);
        std::copy_n(frame.hmdForwardWorld, 3, result->forwardWorld);
        return Status::Ok;
    });
}

Status ROCK_CALL getRoles(OwnerToken owner, RolesV1* result) noexcept {
    if (const auto status=checkOutput(result); status!=Status::Ok) return status;
    return invoke(owner, kInterfaceId, 1, true, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample=provider::runtime::sample();
        result->primary=static_cast<Hand>(frame.primaryHand);
        result->offhand=static_cast<Hand>(frame.offhandHand);
        return Status::Ok;
    });
}
