#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace frik::rock::hand_collider_semantics
{
    /*
     * ROCK now treats the live FRIK-mutated root skeleton as the hand collision
     * source of truth. The legacy box is not a gameplay fallback for this path:
     * Havok still needs one constraint body, but that body is a generated palm
     * anchor with the same semantic table as the finger bodies around it.
     */

    enum class HandColliderRuntimeMode : int
    {
        Disabled = 0,
        BoneDerivedHands = 1
    };

    enum class HandFinger : std::uint8_t
    {
        Thumb = 0,
        Index = 1,
        Middle = 2,
        Ring = 3,
        Pinky = 4,
        None = 255
    };

    enum class HandFingerSegment : std::uint8_t
    {
        Base = 0,
        Middle = 1,
        Tip = 2,
        None = 255
    };

    enum class HandColliderRole : std::uint8_t
    {
        PalmAnchor = 0,
        PalmFace,
        PalmBack,
        PalmHeel,
        ThumbPad,
        ThumbBase,
        ThumbMiddle,
        ThumbTip,
        IndexBase,
        IndexMiddle,
        IndexTip,
        MiddleBase,
        MiddleMiddle,
        MiddleTip,
        RingBase,
        RingMiddle,
        RingTip,
        PinkyBase,
        PinkyMiddle,
        PinkyTip
    };

    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    inline constexpr std::size_t kHandFingerCount = 5;
    inline constexpr std::size_t kHandFingerSegmentCount = 3;
    inline constexpr std::size_t kHandPalmRoleCount = 5;
    inline constexpr std::size_t kHandFingerRoleCount = kHandFingerCount * kHandFingerSegmentCount;
    inline constexpr std::size_t kHandColliderBodyCountPerHand = kHandPalmRoleCount + kHandFingerRoleCount;
    inline constexpr std::size_t kHandSegmentColliderBodyCountPerHand = kHandColliderBodyCountPerHand - 1;

    inline constexpr std::array<HandColliderRole, kHandColliderBodyCountPerHand> kHandColliderRoles{
        HandColliderRole::PalmAnchor,
        HandColliderRole::PalmFace,
        HandColliderRole::PalmBack,
        HandColliderRole::PalmHeel,
        HandColliderRole::ThumbPad,
        HandColliderRole::ThumbBase,
        HandColliderRole::ThumbMiddle,
        HandColliderRole::ThumbTip,
        HandColliderRole::IndexBase,
        HandColliderRole::IndexMiddle,
        HandColliderRole::IndexTip,
        HandColliderRole::MiddleBase,
        HandColliderRole::MiddleMiddle,
        HandColliderRole::MiddleTip,
        HandColliderRole::RingBase,
        HandColliderRole::RingMiddle,
        HandColliderRole::RingTip,
        HandColliderRole::PinkyBase,
        HandColliderRole::PinkyMiddle,
        HandColliderRole::PinkyTip
    };

    inline constexpr std::array<HandColliderRole, kHandSegmentColliderBodyCountPerHand> kHandNonAnchorColliderRoles{
        HandColliderRole::PalmFace,
        HandColliderRole::PalmBack,
        HandColliderRole::PalmHeel,
        HandColliderRole::ThumbPad,
        HandColliderRole::ThumbBase,
        HandColliderRole::ThumbMiddle,
        HandColliderRole::ThumbTip,
        HandColliderRole::IndexBase,
        HandColliderRole::IndexMiddle,
        HandColliderRole::IndexTip,
        HandColliderRole::MiddleBase,
        HandColliderRole::MiddleMiddle,
        HandColliderRole::MiddleTip,
        HandColliderRole::RingBase,
        HandColliderRole::RingMiddle,
        HandColliderRole::RingTip,
        HandColliderRole::PinkyBase,
        HandColliderRole::PinkyMiddle,
        HandColliderRole::PinkyTip
    };

    inline constexpr std::size_t colliderBodyCountPerHand()
    {
        return kHandColliderBodyCountPerHand;
    }

    inline constexpr std::size_t segmentColliderBodyCountPerHand()
    {
        return kHandSegmentColliderBodyCountPerHand;
    }

    inline constexpr HandColliderRuntimeMode sanitizeHandColliderRuntimeMode(int mode)
    {
        return mode == 0 ? HandColliderRuntimeMode::Disabled : HandColliderRuntimeMode::BoneDerivedHands;
    }

    inline constexpr bool isPalmRole(HandColliderRole role)
    {
        return role == HandColliderRole::PalmAnchor ||
               role == HandColliderRole::PalmFace ||
               role == HandColliderRole::PalmBack ||
               role == HandColliderRole::PalmHeel ||
               role == HandColliderRole::ThumbPad;
    }

    inline constexpr bool isFingerRole(HandColliderRole role)
    {
        return !isPalmRole(role);
    }

    inline constexpr bool isTipRole(HandColliderRole role)
    {
        return role == HandColliderRole::ThumbTip ||
               role == HandColliderRole::IndexTip ||
               role == HandColliderRole::MiddleTip ||
               role == HandColliderRole::RingTip ||
               role == HandColliderRole::PinkyTip;
    }

    inline constexpr HandFinger fingerForRole(HandColliderRole role)
    {
        switch (role) {
        case HandColliderRole::ThumbBase:
        case HandColliderRole::ThumbMiddle:
        case HandColliderRole::ThumbTip:
        case HandColliderRole::ThumbPad:
            return HandFinger::Thumb;
        case HandColliderRole::IndexBase:
        case HandColliderRole::IndexMiddle:
        case HandColliderRole::IndexTip:
            return HandFinger::Index;
        case HandColliderRole::MiddleBase:
        case HandColliderRole::MiddleMiddle:
        case HandColliderRole::MiddleTip:
            return HandFinger::Middle;
        case HandColliderRole::RingBase:
        case HandColliderRole::RingMiddle:
        case HandColliderRole::RingTip:
            return HandFinger::Ring;
        case HandColliderRole::PinkyBase:
        case HandColliderRole::PinkyMiddle:
        case HandColliderRole::PinkyTip:
            return HandFinger::Pinky;
        default:
            return HandFinger::None;
        }
    }

    inline constexpr HandFingerSegment segmentForRole(HandColliderRole role)
    {
        switch (role) {
        case HandColliderRole::ThumbBase:
        case HandColliderRole::IndexBase:
        case HandColliderRole::MiddleBase:
        case HandColliderRole::RingBase:
        case HandColliderRole::PinkyBase:
            return HandFingerSegment::Base;
        case HandColliderRole::ThumbMiddle:
        case HandColliderRole::IndexMiddle:
        case HandColliderRole::MiddleMiddle:
        case HandColliderRole::RingMiddle:
        case HandColliderRole::PinkyMiddle:
            return HandFingerSegment::Middle;
        case HandColliderRole::ThumbTip:
        case HandColliderRole::IndexTip:
        case HandColliderRole::MiddleTip:
        case HandColliderRole::RingTip:
        case HandColliderRole::PinkyTip:
            return HandFingerSegment::Tip;
        default:
            return HandFingerSegment::None;
        }
    }

    inline constexpr HandColliderRole roleForFingerSegment(HandFinger finger, HandFingerSegment segment)
    {
        switch (finger) {
        case HandFinger::Thumb:
            return segment == HandFingerSegment::Base ? HandColliderRole::ThumbBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::ThumbMiddle :
                   HandColliderRole::ThumbTip;
        case HandFinger::Index:
            return segment == HandFingerSegment::Base ? HandColliderRole::IndexBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::IndexMiddle :
                   HandColliderRole::IndexTip;
        case HandFinger::Middle:
            return segment == HandFingerSegment::Base ? HandColliderRole::MiddleBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::MiddleMiddle :
                   HandColliderRole::MiddleTip;
        case HandFinger::Ring:
            return segment == HandFingerSegment::Base ? HandColliderRole::RingBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::RingMiddle :
                   HandColliderRole::RingTip;
        case HandFinger::Pinky:
            return segment == HandFingerSegment::Base ? HandColliderRole::PinkyBase :
                   segment == HandFingerSegment::Middle ? HandColliderRole::PinkyMiddle :
                   HandColliderRole::PinkyTip;
        default:
            return HandColliderRole::PalmAnchor;
        }
    }

    inline constexpr const char* roleName(HandColliderRole role)
    {
        switch (role) {
        case HandColliderRole::PalmAnchor: return "PalmAnchor";
        case HandColliderRole::PalmFace: return "PalmFace";
        case HandColliderRole::PalmBack: return "PalmBack";
        case HandColliderRole::PalmHeel: return "PalmHeel";
        case HandColliderRole::ThumbPad: return "ThumbPad";
        case HandColliderRole::ThumbBase: return "ThumbBase";
        case HandColliderRole::ThumbMiddle: return "ThumbMiddle";
        case HandColliderRole::ThumbTip: return "ThumbTip";
        case HandColliderRole::IndexBase: return "IndexBase";
        case HandColliderRole::IndexMiddle: return "IndexMiddle";
        case HandColliderRole::IndexTip: return "IndexTip";
        case HandColliderRole::MiddleBase: return "MiddleBase";
        case HandColliderRole::MiddleMiddle: return "MiddleMiddle";
        case HandColliderRole::MiddleTip: return "MiddleTip";
        case HandColliderRole::RingBase: return "RingBase";
        case HandColliderRole::RingMiddle: return "RingMiddle";
        case HandColliderRole::RingTip: return "RingTip";
        case HandColliderRole::PinkyBase: return "PinkyBase";
        case HandColliderRole::PinkyMiddle: return "PinkyMiddle";
        case HandColliderRole::PinkyTip: return "PinkyTip";
        }
        return "Unknown";
    }
}
