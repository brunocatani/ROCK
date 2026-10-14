// Private References service operations. Included after shared owner/lifecycle helpers.

    api::Status apiQueryReferenceInteractionV1(std::uint64_t ownerToken,
        const RockProviderReferenceQueryV1* query, RockProviderReferenceInteractionV1* out)
    {
        provider_state_policy::clearQueryOutput(out);
        if (!out) return api::Status::InvalidArgument;
        if (out->size != sizeof(*out)) return api::Status::InvalidSize;
        if (out->version != ROCK_PROVIDER_API_VERSION) return api::Status::UnsupportedVersion;
        const auto valid = validateTargetQuery(ownerToken, query);
        if (valid != api::Status::Ok) return valid;
        if (!rock::reference_interaction::describe(rock::reference_interaction::resolveQuery(*query), *out, query->furnitureMarkerIndex)) return api::Status::TargetUnavailable;
        out->frameIndex = currentGameFrameIndex();
        out->worldGeneration = s_currentWorldGeneration.load(std::memory_order_acquire);
        return api::Status::Ok;
    }

    api::Status apiQueryPowerArmorTargetV1(std::uint64_t ownerToken,
        const RockProviderReferenceQueryV1* query, RockProviderPowerArmorTargetV1* out)
    {
        provider_state_policy::clearQueryOutput(out);
        if (!out) return api::Status::InvalidArgument;
        if (out->size != sizeof(*out)) return api::Status::InvalidSize;
        if (out->version != ROCK_PROVIDER_API_VERSION) return api::Status::UnsupportedVersion;
        const auto valid = validateTargetQuery(ownerToken, query);
        if (valid != api::Status::Ok) return valid;
        if (!rock::reference_interaction::describePowerArmor(rock::reference_interaction::resolveQuery(*query), *out, query->furnitureMarkerIndex)) return api::Status::TargetUnavailable;
        out->touchedReference.frameIndex = out->frameReference.frameIndex = currentGameFrameIndex();
        out->touchedReference.worldGeneration = out->frameReference.worldGeneration = s_currentWorldGeneration.load(std::memory_order_acquire);
        return api::Status::Ok;
    }
