#pragma once

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/body/BodyContactRuntime.h"
#include "physics-interaction/stash/ShoulderStashMath.h"

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"

#include <cstdint>
#include <vector>

namespace rock::shoulder_stash
{
    /*
     * The detector treats shoulder stash as a head-relative back gesture first
     * so storing loose items does not require the player to touch their real
     * back. Generated FRIK body colliders and recent hknp contact remain as a
     * backup path for missing HMD evidence and as reusable body-zone evidence
     * for holsters, where physical collider precision is still the right tool.
     */
    struct DetectorConfig
    {
        bool enabled = true;
        bool useBodyZoneColliders = true;
        bool useHmdBackVolume = true;
        float enterPaddingGameUnits = 5.0f;
        float exitPaddingGameUnits = 8.0f;
        float minDwellSeconds = 0.08f;
        float maxSpeedGameUnitsPerSecond = 140.0f;
        int recentContactFrames = 4;
        int sustainedContactMissFrames = 18;
        RE::NiPoint3 hmdBackRightOffsetGameUnits{ 17.5f, -5.0f, -6.85f };
        RE::NiPoint3 hmdBackLeftOffsetGameUnits{ -17.5f, -5.0f, -6.85f };
        float hmdBackRadiusGameUnits = 11.0f;
    };

    struct DetectorInput
    {
        RE::hknpWorld* world = nullptr;
        const BodyBoneColliderSet* bodyColliders = nullptr;
        const body_contact_runtime::BodyContactRuntime* bodyContacts = nullptr;
        const std::vector<std::uint32_t>* heldBodyIds = nullptr;
        std::uint32_t contactFrame = 0;
        bool isLeftHand = false;
        Probe probe{};
        Probe hmdProbe{};
        bool hasHmdProbe = false;
        bool hasHmdFrame = false;
        RE::NiPoint3 hmdPositionWorld{};
        RE::NiPoint3 hmdForwardWorld{};
        float deltaSeconds = 0.0f;
        DetectorConfig config{};
    };

    void resetRuntime(RuntimeState& state) noexcept;

    [[nodiscard]] Decision evaluate(const DetectorInput& input, RuntimeState& runtime);
}
