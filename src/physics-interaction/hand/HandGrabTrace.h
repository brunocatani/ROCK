#pragma once

#include "physics-interaction/hand/Hand.h"

#include <cstdint>

namespace rock::hand_grab_detail
{
    struct AnchorClockRoomSample
    {
        RE::NiPoint3 position{};
        float yawDegrees = 0.0f;
        bool valid = false;
    };

    [[nodiscard]] grab_authority_source_clock::ControllerRootFrameSample samplePlayerControllerRootFrame() noexcept;
    [[nodiscard]] std::uint64_t nextGrabTimelineTraceId() noexcept;
    [[nodiscard]] bool grabTimelineTraceEnabled() noexcept;
    [[nodiscard]] bool shouldLogGrabTimelineSequence(std::uint64_t sequence) noexcept;
    [[nodiscard]] const char* releaseDispositionName(GrabReleaseDisposition disposition) noexcept;
    [[nodiscard]] const char* nodeDebugName(const RE::NiAVObject* node) noexcept;
    void logGrabNodeInfo(const char* handName,
        bool isLeft,
        const RE::NiAVObject* parentNode,
        const RE::NiAVObject* authoredGrabNode,
        const RE::NiTransform& desiredObjectWorld,
        const RE::NiTransform& handWorldTransform,
        const RE::NiPoint3& grabPivotAWorld,
        const char* grabPointMode);
    [[nodiscard]] AnchorClockRoomSample sampleAnchorClockRoom();
    void assignGrabClockFeedVec(float (&out)[3], const RE::NiPoint3& in) noexcept;
    void logRuntimeScaleIfChanged(bool isLeft,
        const char* handName,
        const RE::NiTransform& handWorldTransform,
        const RE::NiAVObject* collidableNode);
}
