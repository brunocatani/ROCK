#pragma once

#include <cmath>
#include <cstdint>
#include <format>

#include "RE/NetImmerse/NiPoint.h"

namespace rock::body_frame
{
    /*
     * ROCK keeps the hknp body/shape frame separate from the motion center-of-mass
     * frame because FO4VR exposes both and they are not interchangeable. Generic
     * live-body diagnostics prefer the motion-backed center-of-mass frame when it
     * is readable. Dynamic grab uses both deliberately: visual/object relations
     * stay in BODY space, while custom constraint body-B math uses the live solver
     * frame and freezes the selected contact point into that frame.
     */
    inline constexpr std::uint32_t kFreeMotionIndex = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kMaxUsableMotionIndex = 4096;
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
         * Generic live body reads prefer the motion-backed solver frame when it is
         * readable. Callers that need the scene collision-object/visual frame must
         * use the BODY-specific helper instead.
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
