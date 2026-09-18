#pragma once
#include <ROCK/Abi.h>
#include <array>
#include <atomic>

namespace RE { class NiAVObject; }
namespace rock::provider {
    // Owned by PhysicsInteraction. Node addresses never cross the public boundary.
    struct WeaponSourceRecord {
        std::uint32_t bodyId{0x7FFFFFFF};
        std::uint64_t weaponGenerationKey{};
        std::uint64_t sourceKey{};
        std::uint64_t parentKey{};
        api::Transform sourceParentLocal{};
        char name[64]{};
    };
    struct WeaponSourcePose {
        api::SampleV1 sample{};
        std::uint64_t weaponGenerationKey{};
        std::uint64_t sourceKey{};
        api::Transform sourceParentLocal{};
        api::Transform weaponRootLocal{};
        api::Transform world{};
    };
    struct WeaponSourceCatalog {
        struct Entry { RE::NiAVObject* node{}; WeaponSourceRecord value{}; };
        static constexpr std::size_t Capacity=4096;
        inline static std::atomic<std::uint64_t> nextKey{1};
        std::array<Entry,Capacity> entries{};
        std::uint32_t count{};
        std::uint64_t generation{};
        RE::NiAVObject* root{};
        bool overflow{};
    };
}
