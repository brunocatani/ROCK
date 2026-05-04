#pragma once

#include <cstddef>
#include <cmath>

#include "TransformMath.h"

#include "RE/NetImmerse/NiTransform.h"

namespace frik::rock::grab_transform_telemetry_overlay
{
    /*
     * Grab telemetry must be visually tied to the same live hand frame that
     * drives collision, grab pivots, and FRIK hand authority diagnostics.
     * A fixed screen corner made log matching possible but was not reliable
     * in the headset. This helper keeps label placement testable and uses the
     * existing NiTransform convention instead of adding renderer-local hand
     * math that could drift away from the grab system.
     */

    struct HandAttachedTextBasis
    {
        RE::NiPoint3 hand{};
        RE::NiPoint3 anchor{};
        RE::NiPoint3 panelRight{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 panelUp{ 0.0f, 0.0f, 1.0f };
    };

    inline RE::NiPoint3 normalizedOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        if (length < 1.0e-5f) {
            return fallback;
        }

        const float invLength = 1.0f / length;
        return RE::NiPoint3{ value.x * invLength, value.y * invLength, value.z * invLength };
    }

    inline HandAttachedTextBasis buildHandAttachedTextBasis(const RE::NiTransform& rawHandWorld, bool isLeft)
    {
        HandAttachedTextBasis result{};
        result.hand = rawHandWorld.translate;
        result.panelRight = normalizedOrFallback(transform_math::localVectorToWorld(rawHandWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }), RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        result.panelUp = normalizedOrFallback(transform_math::localVectorToWorld(rawHandWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });

        const RE::NiPoint3 lateral = normalizedOrFallback(transform_math::localVectorToWorld(rawHandWorld, RE::NiPoint3{ 0.0f, isLeft ? -1.0f : 1.0f, 0.0f }),
            RE::NiPoint3{ 0.0f, isLeft ? -1.0f : 1.0f, 0.0f });
        result.anchor = rawHandWorld.translate + lateral * 12.0f + result.panelUp * 18.0f;
        return result;
    }

    inline RE::NiPoint3 lineAnchor(const HandAttachedTextBasis& basis, std::size_t lineIndex)
    {
        constexpr float kLineSpacingGameUnits = 4.5f;
        return basis.anchor - basis.panelUp * (static_cast<float>(lineIndex) * kLineSpacingGameUnits);
    }
}
