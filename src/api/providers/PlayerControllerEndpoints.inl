Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return boundary::readSample(owner, kInterfaceId, outSample);
}
