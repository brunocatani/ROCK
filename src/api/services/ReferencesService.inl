// Private References service operations. Included after shared owner/lifecycle helpers.

    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryReferenceInteractionV1(std::uint64_t ownerToken,
        const RockProviderReferenceQueryV1* query, RockProviderReferenceInteractionV1* out)
    {
        provider_state_policy::clearQueryOutput(out);
        if (!out) return RockProviderResultV1::InvalidArgument;
        if (out->size != sizeof(*out)) return RockProviderResultV1::InvalidSize;
        if (out->version != ROCK_PROVIDER_API_VERSION) return RockProviderResultV1::UnsupportedVersion;
        const auto valid = validateTargetQuery(ownerToken, query, RockProviderConsumerCapabilityV1::TargetDetails);
        if (valid != RockProviderResultV1::Ok) return valid;
        if (!rock::reference_interaction::describe(rock::reference_interaction::resolveQuery(*query), *out, query->furnitureMarkerIndex)) return RockProviderResultV1::TargetUnavailable;
        out->frameIndex = currentGameFrameIndex();
        out->worldGeneration = s_currentWorldGeneration.load(std::memory_order_acquire);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryPowerArmorTargetV1(std::uint64_t ownerToken,
        const RockProviderReferenceQueryV1* query, RockProviderPowerArmorTargetV1* out)
    {
        provider_state_policy::clearQueryOutput(out);
        if (!out) return RockProviderResultV1::InvalidArgument;
        if (out->size != sizeof(*out)) return RockProviderResultV1::InvalidSize;
        if (out->version != ROCK_PROVIDER_API_VERSION) return RockProviderResultV1::UnsupportedVersion;
        const auto valid = validateTargetQuery(ownerToken, query, RockProviderConsumerCapabilityV1::PowerArmor);
        if (valid != RockProviderResultV1::Ok) return valid;
        if (!rock::reference_interaction::describePowerArmor(rock::reference_interaction::resolveQuery(*query), *out, query->furnitureMarkerIndex)) return RockProviderResultV1::TargetUnavailable;
        out->touchedReference.frameIndex = out->frameReference.frameIndex = currentGameFrameIndex();
        out->touchedReference.worldGeneration = out->frameReference.worldGeneration = s_currentWorldGeneration.load(std::memory_order_acquire);
        return RockProviderResultV1::Ok;
    }
