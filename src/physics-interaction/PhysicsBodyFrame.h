// Havok body frames: which frame (body/shape or motion center of mass) a body
// transform comes from, body-id sentinels, and readability limits for body ids.
#pragma once

#include <cmath>
#include <cstdint>
#include <format>

#include "RE/NetImmerse/NiPoint.h"
#include "physics-interaction/VectorMath.h"

namespace rock::body_frame
{
    /*
     * ROCK keeps the hknp body/shape frame separate from the motion frame
     * because FO4VR exposes both and they are not interchangeable: the motion's
     * origin is the center of mass and its axes are the principal inertia axes.
     */
    inline constexpr std::uint32_t kFreeMotionIndex = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    /*
     * Long FO4VR sessions can push the hknp body array past 8192 while valid
     * generated keyframed bodies still live in the same world. The readability
     * guard only needs to reject invalid/sentinel indices and implausible body
     * array corruption; capping at 8192 made ROCK falsely tear down hands,
     * body bones, and weapon hulls once the native high-water mark grew.
     */
    inline constexpr std::uint32_t kMaxReadableBodyIndex = 0x000F'FFFF;

    enum class BodyFrameSource : std::uint8_t
    {
        BodyTransform = 0,
        MotionCenterOfMass,
        OwnerNode,
        ContactPoint,
        None
    };

    struct BodyFrameAnchor
    {
        RE::NiPoint3 position{};
        BodyFrameSource source{ BodyFrameSource::None };
    };

    inline bool hasUsableMotionIndex(std::uint32_t motionIndex)
    {
        // Busy exterior worlds allocate valid motions above 4096. This checks
        // identity only; dereferences must also check the live motion array.
        return motionIndex > 0 && motionIndex < kFreeMotionIndex;
    }

    inline bool motionSlotCanBeRead(std::uint32_t motionIndex, std::uint32_t slotCount, std::uint32_t capacityAndFlags)
    {
        const auto capacity = capacityAndFlags & 0x3FFF'FFFFu;
        return hasUsableMotionIndex(motionIndex) && motionIndex < slotCount && slotCount <= capacity &&
            capacity <= kMaxReadableBodyIndex + 1u;
    }

    inline bool bodySlotCanBeRead(std::uint32_t bodyId, std::uint32_t highWaterMark)
    {
        return bodyId != kInvalidBodyId && bodyId <= kMaxReadableBodyIndex && highWaterMark <= kMaxReadableBodyIndex &&
            bodyId <= highWaterMark;
    }

    inline BodyFrameAnchor chooseSelectionDistanceAnchor(bool hasContactPoint, const RE::NiPoint3& contactPoint,
        bool hasBodyTransform, const RE::NiPoint3& bodyTransform, bool hasOwnerNode, const RE::NiPoint3& ownerNode,
        bool hasMotionCenterOfMass, const RE::NiPoint3& motionCenterOfMass)
    {
        if (hasContactPoint) {
            return BodyFrameAnchor{ contactPoint, BodyFrameSource::ContactPoint };
        }
        if (hasBodyTransform) {
            return BodyFrameAnchor{ bodyTransform, BodyFrameSource::BodyTransform };
        }
        if (hasOwnerNode) {
            return BodyFrameAnchor{ ownerNode, BodyFrameSource::OwnerNode };
        }
        if (hasMotionCenterOfMass) {
            return BodyFrameAnchor{ motionCenterOfMass, BodyFrameSource::MotionCenterOfMass };
        }
        return BodyFrameAnchor{ RE::NiPoint3{}, BodyFrameSource::None };
    }

    using vector_math::distance;
}
