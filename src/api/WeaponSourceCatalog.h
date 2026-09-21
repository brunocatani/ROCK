#pragma once
#include <ROCK/Abi.h>

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
}
