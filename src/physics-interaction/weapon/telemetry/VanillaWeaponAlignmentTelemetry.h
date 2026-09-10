#pragma once

#include "RE/NetImmerse/NiTransform.h"
#include <cstdint>
#include <span>
#include <string_view>

namespace RE { class NiNode; }

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
        const RE::NiTransform& trackedHand, const RE::NiTransform& solvedWeapon,
        const RE::NiTransform& solvedHand);

    enum class ArmCaller : std::uint8_t { NativePrimary, NativeSupport, Other };
    // Invoked around the existing arm hook. Calls from any other thread are
    // ignored before touching the game-thread-owned logger or scene.
    std::uint64_t beginNativeArm(ArmCaller caller, std::uintptr_t returnAddress,
        RE::NiNode** weapon, RE::NiNode** offset) noexcept;
    void endNativeArm(std::uint64_t event, ArmCaller caller,
        RE::NiNode** weapon, RE::NiNode** offset) noexcept;

    struct AnimationBone
    {
        const char* name = nullptr;
        int bone = -1, parent = -1, track = -1;
        RE::NiTransform sampled{}, reference{};
        bool sampledValid = false, referenceValid = false, fromReference = false;
    };
    // Rate-limit before looking up optional bones. These observations never
    // change the pose selected for gameplay or cause a cache bypass.
    bool wantsAnimationSample(std::uint32_t formId) noexcept;
    void recordAnimationSample(std::uint32_t formId, std::uint64_t variant,
        std::uint64_t instance, std::uint64_t graphProfile, std::string_view clip,
        int animationType, std::uint32_t blendHint, std::span<const AnimationBone> bones) noexcept;
}
