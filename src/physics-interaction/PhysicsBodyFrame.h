#pragma once

#include <cmath>
#include <cstdint>
#include <format>

#include "RE/NetImmerse/NiPoint.h"

namespace rock::body_frame
{
    /*
     * ROCK keeps the hknp body/shape frame separate from the motion center-of-mass
     * frame because FO4VR exposes both and they are not interchangeable. Dynamic
     * bodies publish their live rotation through the motion quaternion; collider
     * rendering, grab constraints, visual authority, and telemetry must therefore
     * use the motion-backed body frame when it exists and only fall back to the
     * body-array transform for bodies without a usable motion slot.
     */
    inline constexpr std::uint32_t kFreeMotionIndex = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kMaxUsableMotionIndex = 4096;
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kMaxReadableBodyIndex = 8192;

    enum class BodyFrameSource : std::uint8_t
    {
        BodyTransform = 0,
        MotionCenterOfMass,
        OwnerNode,
        ContactPoint,
        Fallback
    };

    struct BodyFrameAnchor
    {
        RE::NiPoint3 position{};
        BodyFrameSource source{ BodyFrameSource::Fallback };
    };

    inline bool hasUsableMotionIndex(std::uint32_t motionIndex)
    {
        return motionIndex > 0 && motionIndex < kMaxUsableMotionIndex && motionIndex != kFreeMotionIndex;
    }

    inline bool bodySlotCanBeRead(std::uint32_t bodyId, std::uint32_t highWaterMark)
    {
        return bodyId != kInvalidBodyId && bodyId <= kMaxReadableBodyIndex && highWaterMark <= kMaxReadableBodyIndex && bodyId <= highWaterMark;
    }

    inline BodyFrameSource chooseColliderFrameSource(bool hasBodyTransform, bool hasMotionCenterOfMass)
    {
        if (hasMotionCenterOfMass) {
            return BodyFrameSource::MotionCenterOfMass;
        }
        if (hasBodyTransform) {
            return BodyFrameSource::BodyTransform;
        }
        return BodyFrameSource::Fallback;
    }

    inline BodyFrameSource chooseLiveBodyFrameSource(bool hasBodyTransform, bool hasMotionTransform)
    {
        /*
         * Grab, visual authority, and telemetry must share the same live frame as
         * the collider overlay. hknp publishes dynamic-body rotation through the
         * motion quaternion, while the body-array transform is only a fallback for
         * bodies without a usable motion slot.
         */
        if (hasMotionTransform) {
            return BodyFrameSource::MotionCenterOfMass;
        }
        if (hasBodyTransform) {
            return BodyFrameSource::BodyTransform;
        }
        return BodyFrameSource::Fallback;
    }

    inline BodyFrameAnchor chooseSelectionDistanceAnchor(bool hasContactPoint,
        const RE::NiPoint3& contactPoint,
        bool hasBodyTransform,
        const RE::NiPoint3& bodyTransform,
        bool hasOwnerNode,
        const RE::NiPoint3& ownerNode,
        bool hasMotionCenterOfMass,
        const RE::NiPoint3& motionCenterOfMass,
        const RE::NiPoint3& fallback)
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
        return BodyFrameAnchor{ fallback, BodyFrameSource::Fallback };
    }

    inline float distance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        const float dx = lhs.x - rhs.x;
        const float dy = lhs.y - rhs.y;
        const float dz = lhs.z - rhs.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    inline const char* bodyFrameSourceName(BodyFrameSource source)
    {
        switch (source) {
        case BodyFrameSource::BodyTransform:
            return "bodyTransform";
        case BodyFrameSource::MotionCenterOfMass:
            return "motionCenterOfMass";
        case BodyFrameSource::OwnerNode:
            return "ownerNode";
        case BodyFrameSource::ContactPoint:
            return "contactPoint";
        case BodyFrameSource::Fallback:
        default:
            return "fallback";
        }
    }

    inline const char* bodyFrameSourceCode(BodyFrameSource source)
    {
        switch (source) {
        case BodyFrameSource::BodyTransform:
            return "BODY";
        case BodyFrameSource::MotionCenterOfMass:
            return "MOTION";
        case BodyFrameSource::OwnerNode:
            return "OWNER";
        case BodyFrameSource::ContactPoint:
            return "CONTACT";
        case BodyFrameSource::Fallback:
        default:
            return "FALLBACK";
        }
    }
}
