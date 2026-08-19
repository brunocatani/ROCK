#pragma once

#include <vector>

#include "RE/Havok/hknpShape.h"
#include "RE/NetImmerse/NiPoint.h"

namespace rock::havok_convex_shape_builder
{
    RE::hknpShape* buildConvexShapeFromLocalHavokPoints(const std::vector<RE::NiPoint3>& points, float convexRadius);
}
