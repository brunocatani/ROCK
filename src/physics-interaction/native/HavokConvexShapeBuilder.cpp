#include "physics-interaction/native/HavokConvexShapeBuilder.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace rock::havok_convex_shape_builder
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

        bool sharpFeatureConfigVerified()
        {
            // Verify the native constructor and the consumer of config +0x08
            // before overriding this otherwise opaque, native-owned layout.
            static const bool verified = [] {
                constexpr std::array<std::uint8_t, 13> initBytes{
                    0xC7, 0x01, 0x0A, 0xD7, 0x23, 0x3C, 0xC6, 0x41, 0x04, 0x01, 0x89, 0x51, 0x08
                };
                constexpr std::array<std::uint8_t, 7> shrinkBytes{
                    0xF3, 0x41, 0x0F, 0x10, 0x44, 0x24, 0x08
                };
                const REL::Relocation<std::uintptr_t> init{ REL::Offset(offsets::kFunc_ConvexBuildConfig_Init + 0x2B) };
                const REL::Relocation<std::uintptr_t> shrink{ REL::Offset(0x16DDFE4) };
                std::array<std::uint8_t, initBytes.size()> liveInit{};
                std::array<std::uint8_t, shrinkBytes.size()> liveShrink{};
                const bool matches =
                    native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(init.address()), liveInit.data(), liveInit.size()) &&
                    native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(shrink.address()), liveShrink.data(), liveShrink.size()) &&
                    liveInit == initBytes && liveShrink == shrinkBytes;
                if (!matches) {
                    ROCK_LOG_ERROR(Weapon, "Sharp-feature convex build rejected: native config signature mismatch");
                }
                return matches;
            }();
            return verified;
        }
    }

    RE::hknpShape* buildConvexShapeFromLocalHavokPoints(const std::vector<RE::NiPoint3>& points, float convexRadius, ConvexFit fit)
    {
        if (points.size() < 4) {
            ROCK_LOG_WARN(Weapon, "Convex shape build skipped: point count {} is below hull minimum", points.size());
            return nullptr;
        }
        if (fit == ConvexFit::PreserveSharpFeatures && !sharpFeatureConfigVerified()) {
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

        if (fit == ConvexFit::PreserveSharpFeatures) {
            /*
             * Weapon inputs already passed ROCK's bounded support fit. The
             * native defaults simplify them again at 0.01 Havok units, then
             * inset their planes by the convex radius. On an acute blade tip
             * that inset can retract the vertex far more than the radius.
             *
             * FO4VR 1.2.72 raw witnesses: 1416D4ADB..AE8 initializes tolerance
             * +0x00, shrink enabled +0x04, and relative shrink +0x08. The hull
             * path at 1416DDCE5/1416DDDB4 consumes the tolerance; 1416DDFE4..
             * 1416DE00B forwards relative shrink to 1418997D0 and retains its
             * adjusted radius. 14189997E..9BB clamps that control to [0,1];
             * 14189AA0C..AB38 limits the inset using radius-sized corner
             * displacements when it is 1. Keep native shrinking enabled so
             * Havok adjusts the margin together with the hull.
             */
            constexpr float simplificationTolerance = 0.0f;
            constexpr float relativeShrink = 1.0f;
            std::memcpy(config.data(), &simplificationTolerance, sizeof(simplificationTolerance));
            std::memcpy(config.data() + 0x08, &relativeShrink, sizeof(relativeShrink));
        }

        HkStridedPointArray array{ hkPoints.data(), static_cast<std::int32_t>(hkPoints.size()), static_cast<std::int32_t>(sizeof(HkPoint)) };

        static REL::Relocation<BuildShapeFromPoints_t> buildShape{ REL::Offset(offsets::kFunc_ConvexShape_FromPoints) };
        auto* shape = buildShape(&array, (std::max)(0.0f, convexRadius), config.data());
        if (!shape) {
            ROCK_LOG_WARN(Weapon, "Native convex shape builder returned null for {} points", points.size());
        }
        return shape;
    }
}
