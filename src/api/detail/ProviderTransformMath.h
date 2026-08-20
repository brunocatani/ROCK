#pragma once

#include "api/ROCKProviderApi.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

#include <cmath>
#include <cstddef>

namespace rock::provider::detail
{
    inline std::size_t boundedStringLength(const char* value, std::size_t capacity)
    {
        for (std::size_t i = 0; i < capacity; ++i) {
            if (value[i] == '\0') {
                return i;
            }
        }
        return capacity;
    }

    [[nodiscard]] inline bool finiteProviderTransform(
        const RockProviderTransform& transform,
        const float minimumScale = 0.000001f)
    {
        for (const float value : transform.rotate) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        return std::isfinite(transform.translate[0]) &&
               std::isfinite(transform.translate[1]) &&
               std::isfinite(transform.translate[2]) &&
               std::isfinite(transform.scale) &&
               std::abs(transform.scale) > minimumScale;
    }

    [[nodiscard]] inline bool finiteProviderPoint(
        const RockProviderPoint3& point)
    {
        return std::isfinite(point.x) &&
               std::isfinite(point.y) &&
               std::isfinite(point.z);
    }

    [[nodiscard]] inline RE::NiTransform toNiTransform(const RockProviderTransform& source)
    {
        RE::NiTransform target{};
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotate.entry[row][column] = source.rotate[row * 3 + column];
            }
        }
        target.translate = RE::NiPoint3(
            source.translate[0],
            source.translate[1],
            source.translate[2]);
        target.scale = source.scale;
        return target;
    }

    [[nodiscard]] constexpr frik_visual_authority::Hand toVisualHand(
        const RockProviderHand hand)
    {
        return hand == RockProviderHand::Left ?
            frik_visual_authority::Hand::Left :
            frik_visual_authority::Hand::Right;
    }
}
