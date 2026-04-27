#include "HavokConvexShapeBuilder.h"

#include "HavokOffsets.h"
#include "PhysicsLog.h"

#include <algorithm>
#include <array>
#include <cstdint>

namespace frik::rock::havok_convex_shape_builder
{
    namespace
    {
        /*
         * FO4VR already ships a native convex-point builder. Ghidra verification
         * showed that 0x1416d4b30 consumes a strided hkVector4 point array and the
         * default config initialized by 0x1416d4ab0, with the same 0xfc max-vertex
         * budget Bethesda uses. Building generated weapon hulls through this path
         * keeps ownership and shape layout in Bethesda/Havok code instead of
         * manually patching hknpShape memory.
         */
        struct alignas(16) HkPoint
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            float w = 0.0f;
        };

        struct HkStridedPointArray
        {
            const void* data = nullptr;
            std::int32_t count = 0;
            std::int32_t stride = 0;
        };

        using InitConfig_t = void* (*)(void*);
        using BuildShapeFromPoints_t = RE::hknpShape* (*)(const HkStridedPointArray*, float, void*);
    }

    RE::hknpShape* buildConvexShapeFromLocalHavokPoints(const std::vector<RE::NiPoint3>& points, float convexRadius)
    {
        if (points.size() < 4) {
            ROCK_LOG_WARN(Weapon, "Convex shape build skipped: point count {} is below hull minimum", points.size());
            return nullptr;
        }

        std::vector<HkPoint> hkPoints;
        hkPoints.reserve(points.size());
        for (const auto& point : points) {
            hkPoints.push_back(HkPoint{ point.x, point.y, point.z, 0.0f });
        }

        alignas(16) std::array<std::uint8_t, 0x80> config{};
        static REL::Relocation<InitConfig_t> initConfig{ REL::Offset(offsets::kFunc_ConvexBuildConfig_Init) };
        initConfig(config.data());

        HkStridedPointArray array{ hkPoints.data(), static_cast<std::int32_t>(hkPoints.size()), static_cast<std::int32_t>(sizeof(HkPoint)) };

        static REL::Relocation<BuildShapeFromPoints_t> buildShape{ REL::Offset(offsets::kFunc_ConvexShape_FromPoints) };
        auto* shape = buildShape(&array, (std::max)(0.0f, convexRadius), config.data());
        if (!shape) {
            ROCK_LOG_WARN(Weapon, "Native convex shape builder returned null for {} points", points.size());
        }
        return shape;
    }
}
