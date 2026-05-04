#pragma once

#include <cstdint>

namespace frik::rock::active_object_prep_policy
{
    /*
     * Active grab/pull temporarily promotes the selected object tree through
     * FO4VR's recursive SetMotion wrapper before the constraint exists. If the
     * later setup step fails, ROCK must undo that temporary promotion for every
     * original non-dynamic state; successful grabs still keep objects dynamic on
     * release so they can settle naturally.
     */
    inline constexpr std::uint16_t kMotionPresetDynamic = 1;

    inline constexpr bool shouldRestoreMotionAfterFailedActivePrep(bool recursiveMotionWasConverted, std::uint16_t originalMotionPropsId)
    {
        return recursiveMotionWasConverted && originalMotionPropsId != kMotionPresetDynamic;
    }
}

