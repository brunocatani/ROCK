#pragma once

#include "WeaponSemanticTypes.h"

#include <cstdint>
#include <string>
#include <vector>

namespace frik::rock
{
    /*
     * ROCK publishes equipped-weapon evidence for external systems without
     * owning any reload profile or authoring policy. PAPER consumes these POD-ish
     * descriptors to decide how reload bodies map onto the current weapon, while
     * ROCK keeps the evidence tied only to generated collision geometry and
     * semantic contact classification.
     */

    struct WeaponEvidencePoint3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct WeaponEvidenceBounds3
    {
        WeaponEvidencePoint3 min{};
        WeaponEvidencePoint3 max{};
        bool valid{ false };
    };

    struct WeaponCollisionProfileEvidenceDescriptor
    {
        bool valid{ false };
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uintptr_t sourceRootAddress{ 0 };
        std::uintptr_t geometryRootAddress{ 0 };
        std::string sourceRootPath;
        std::string geometryRootPath;
        std::string sourceRootName;
        std::string geometryRootName;
        std::string sourceName;
        WeaponPartClassification semantic{};
        WeaponEvidenceBounds3 localBoundsGame{};
        std::vector<WeaponEvidencePoint3> localMeshPointsGame{};
        std::uint32_t pointCount{ 0 };
    };

    inline WeaponEvidencePoint3 makeWeaponEvidencePoint(float x, float y, float z)
    {
        return WeaponEvidencePoint3{ .x = x, .y = y, .z = z };
    }
}
