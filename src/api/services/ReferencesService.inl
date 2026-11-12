// Private References service operations. Included after shared owner/lifecycle helpers.

    api::Status queryReferenceInteraction(std::uint64_t ownerToken,
        const api::grab::ReferenceQuery& query, api::grab::ReferenceInteraction& out)
    {
        out = {};
        const auto valid = validateTargetQuery(ownerToken, query);
        if (valid != api::Status::Ok) return valid;
        if (!rock::reference_interaction::describe(rock::reference_interaction::resolveQuery(query), out, query.furnitureMarkerIndex)) return api::Status::TargetUnavailable;
        out.frameIndex = currentGameFrameIndex();
        out.worldGeneration = s_currentWorldGeneration.load(std::memory_order_acquire);
        return api::Status::Ok;
    }

    api::Status queryPowerArmorTarget(std::uint64_t ownerToken,
        const api::grab::ReferenceQuery& query, api::grab::PowerArmorTarget& out)
    {
        out = {};
        const auto valid = validateTargetQuery(ownerToken, query);
        if (valid != api::Status::Ok) return valid;
        if (!rock::reference_interaction::describePowerArmor(rock::reference_interaction::resolveQuery(query), out, query.furnitureMarkerIndex)) return api::Status::TargetUnavailable;
        out.touchedReference.frameIndex = out.frameReference.frameIndex = currentGameFrameIndex();
        out.touchedReference.worldGeneration = out.frameReference.worldGeneration = s_currentWorldGeneration.load(std::memory_order_acquire);
        return api::Status::Ok;
    }
