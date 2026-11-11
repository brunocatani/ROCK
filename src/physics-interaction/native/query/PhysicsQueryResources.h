#pragma once

#include "RE/Bethesda/bhkPickData.h"
#include "RE/Havok/hknpAllHitsCollector.h"

namespace rock::physics_query_resources
{
    // Validate the native ownership contract before a query can acquire resources.
    [[nodiscard]] bool nativeCleanupReady() noexcept;

    // RE::bhkPickData releases the world reference PickObject leaves in it.
    using PickData = RE::bhkPickData;

    // Owns both the inline hit storage and any native-allocated overflow buffer.
    // Keep the native object in place: copying it would alias its inline buffer.
    class AllHitsCollector final
    {
    public:
        AllHitsCollector() = default;
        ~AllHitsCollector() noexcept;
        AllHitsCollector(const AllHitsCollector&) = delete;
        AllHitsCollector& operator=(const AllHitsCollector&) = delete;
        AllHitsCollector(AllHitsCollector&&) = delete;
        AllHitsCollector& operator=(AllHitsCollector&&) = delete;

        [[nodiscard]] RE::hknpAllHitsCollector& get() noexcept { return _native; }

    private:
        RE::hknpAllHitsCollector _native;
    };
}
