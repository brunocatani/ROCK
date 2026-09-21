#pragma once

#include <vector>

#include "RE/Havok/hknpShape.h"
#include "RE/NetImmerse/NiPoint.h"

namespace rock::havok_convex_shape_builder
{
    enum class ConvexFit
    {
        NativeDefault,
        PreserveSharpFeatures
    };

    RE::hknpShape* buildConvexShapeFromLocalHavokPoints(const std::vector<RE::NiPoint3>& points, float convexRadius,
        ConvexFit fit = ConvexFit::NativeDefault);
}
