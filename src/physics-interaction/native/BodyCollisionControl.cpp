#include "physics-interaction/native/BodyCollisionControl.h"

#include "physics-interaction/native/HavokRuntime.h"

namespace rock::body_collision
{
    bool tryReadFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t& outFilterInfo)
    {
        return havok_runtime::tryReadFilterInfo(world, bodyId, outFilterInfo);
    }

    bool setFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t filterInfo)
    {
        return havok_runtime::setFilterInfo(world, bodyId, filterInfo);
    }
}
