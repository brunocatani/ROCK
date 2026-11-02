#pragma once

#include <cstdint>

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"

namespace rock::body_collision
{
    bool tryReadFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t& outFilterInfo);
    bool setFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t filterInfo);
}
