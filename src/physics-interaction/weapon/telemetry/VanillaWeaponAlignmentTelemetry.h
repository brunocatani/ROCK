#pragma once

#include "RE/NetImmerse/NiTransform.h"
#include <cstdint>

namespace rock
{
    struct AuthoredPrimaryFiringGripFrameInput;
}

namespace rock::vanilla_weapon_alignment_telemetry
{
    enum class Phase { BeforeFrik, AfterFrik, AfterRock };

    // Main/game thread only. The worker receives formatted text, never scene
    // pointers. Skeleton destruction drains and joins it outside frame capture.
    void initialize();
    void shutdown();
    void capture(Phase phase, std::uint64_t schedulerSequence);
    void recordInput(const AuthoredPrimaryFiringGripFrameInput& input);
    void recordSolve(std::uint32_t formId, std::uint64_t captureSequence,
        const char* source, const RE::NiTransform& handInWeapon,
        const RE::NiTransform& trackedHand, const RE::NiTransform& solvedWeapon);
}
