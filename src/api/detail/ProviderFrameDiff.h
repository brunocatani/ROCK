#pragma once

#include "api/ROCKProviderApi.h"

namespace rock::provider::detail
{
    [[nodiscard]] std::uint64_t advanceSequence(
        std::uint64_t sequence) noexcept;
    [[nodiscard]] bool sameHeldBodies(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept;
    [[nodiscard]] bool sameHandTarget(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept;
    [[nodiscard]] bool handGripActive(
        const RockProviderHandInteractionStateV1& state) noexcept;
    [[nodiscard]] bool sameHandGrip(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept;
    [[nodiscard]] bool sameHandInteractionPayload(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept;
    void assignHandInteractionSequences(
        RockProviderHandInteractionStateV1& current,
        const RockProviderHandInteractionStateV1& previous,
        bool hasPrevious);
    [[nodiscard]] bool sameLifecyclePayload(
        const RockProviderFrameSnapshot& left,
        const RockProviderFrameSnapshot& right) noexcept;
    [[nodiscard]] bool sameWeaponPayload(
        const RockProviderFrameSnapshot& left,
        const RockProviderFrameSnapshot& right) noexcept;
    [[nodiscard]] bool sameEquippedWeaponPayload(
        const RockProviderEquippedWeaponStateV1& left,
        const RockProviderEquippedWeaponStateV1& right) noexcept;
}
