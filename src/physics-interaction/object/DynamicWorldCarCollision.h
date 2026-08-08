#pragma once

#include "RE/Bethesda/TESObjectREFRs.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE
{
    class bhkWorld;
    class hknpWorld;
}

namespace rock
{
    struct DynamicWorldCarTarget
    {
        RE::TESObjectREFR* ref = nullptr;
        std::uint32_t seedBodyId = 0x7FFF'FFFFu;
    };

    class DynamicWorldCarCollisionRuntime
    {
    public:
        void update(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const std::array<DynamicWorldCarTarget, 2>& desiredTargets);

        void restoreReference(
            RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            RE::TESObjectREFR* ref,
            const char* reason);

        void restoreAll(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const char* reason);
        void abandon() noexcept;

    private:
        static constexpr std::size_t kMaxTrackedTargets = 2;
        static constexpr std::size_t kMaxBodiesPerTarget = 64;

        struct TaggedBody
        {
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            std::uint32_t originalFilterInfo = 0;
            std::uint32_t taggedFilterInfo = 0;
        };

        struct TargetSlot
        {
            RE::ObjectRefHandle handle{};
            std::uint32_t formId = 0;
            std::array<TaggedBody, kMaxBodiesPerTarget> bodies{};
            std::size_t bodyCount = 0;

            [[nodiscard]] bool active() const noexcept { return formId != 0 && bodyCount != 0; }
            void clear() noexcept { *this = {}; }
        };

        [[nodiscard]] bool slotMatchesReference(const TargetSlot& slot, RE::TESObjectREFR* ref) const;
        [[nodiscard]] bool slotStillOwnsTags(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const TargetSlot& slot) const;
        bool tagReference(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const DynamicWorldCarTarget& target, TargetSlot& outSlot);
        std::size_t restoreTaggedBodiesForReference(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::TESObjectREFR* ref, std::uint32_t seedBodyId, const char* reason);
        std::size_t restoreSlot(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, TargetSlot& slot, const char* reason);

        std::array<TargetSlot, kMaxTrackedTargets> _slots{};
        RE::bhkWorld* _world = nullptr;
    };
}
