Status ROCK_CALL getSample(OwnerToken owner, SampleV1* outSample) noexcept {
    return boundary::readSample(owner, kInterfaceId, outSample);
}
