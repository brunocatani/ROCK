#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include "RE/NetImmerse/NiPoint.h"
#include "physics-interaction/weapon/WeaponTypes.h"

namespace rock::grip_zone_indicators
{
    struct Frame
    {
        static constexpr std::size_t kCapacity = MAX_WEAPON_COLLISION_BODIES + 4;
        std::array<RE::NiPoint3, kCapacity> positions{};
        std::uint64_t gameFrameIndex{ 0 };
        float diameterGameUnits{ 0.0f };
        std::uint32_t count{ 0 };
    };

    // One producer: ROCK's frame/lifecycle callbacks. Consumers retain only
    // immutable values. EndFrame removes markers when presentation was skipped,
    // without invalidating a completed snapshot while its submit is in flight.
    void EndFrame(std::uint64_t gameFrameIndex) noexcept;
    void Publish(const Frame& frame);
    void Clear() noexcept;
    [[nodiscard]] std::shared_ptr<const Frame> Snapshot() noexcept;
}
