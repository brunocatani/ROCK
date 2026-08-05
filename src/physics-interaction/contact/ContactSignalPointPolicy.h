#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rock::havok_runtime
{
    inline constexpr std::uint32_t kMaxContactSignalPoints = 4;

    enum class ContactSignalPointSelectionSource : std::uint32_t
    {
        None = 0,
        PositiveImpulse,
        NativeContactIndex,
        FirstFinitePoint,
    };

    struct ContactSignalPointResult
    {
        bool valid = false;
        std::uint32_t pointCount = 0;
        std::uint32_t nativeContactPointIndex = 0;
        std::uint32_t selectedPointIndex = 0;
        ContactSignalPointSelectionSource selectionSource{ ContactSignalPointSelectionSource::None };
        float contactPointWeightSum = 0.0f;
        float contactPointHavok[4]{};
        float contactNormalHavok[4]{};
        float contactPointsHavok[kMaxContactSignalPoints][4]{};
        float contactSeparationsHavok[kMaxContactSignalPoints]{};
        float contactImpulses[kMaxContactSignalPoints]{};
    };

    struct ContactSignalPointSelectionInput
    {
        std::uint32_t pointCount = 0;
        std::uint32_t contactIndex = 0;
        float contactSeparationsHavok[4]{};
        float contactImpulses[4]{};
        float contactNormalHavok[4]{};
        float contactPointsHavok[4][4]{};
    };

    [[nodiscard]] inline bool contactSignalFinite3(const float* value)
    {
        return value && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
    }

    inline void copyContactSignalVector4(const float* source, float* target)
    {
        target[0] = source[0];
        target[1] = source[1];
        target[2] = source[2];
        target[3] = source[3];
    }

    [[nodiscard]] inline bool selectContactSignalPoint(
        const ContactSignalPointSelectionInput& input,
        ContactSignalPointResult& outResult)
    {
        outResult = {};

        const std::uint32_t pointCount = (std::min)(input.pointCount, kMaxContactSignalPoints);
        if (pointCount == 0 || !contactSignalFinite3(input.contactNormalHavok)) {
            return false;
        }

        const float normalLengthSquared =
            input.contactNormalHavok[0] * input.contactNormalHavok[0] +
            input.contactNormalHavok[1] * input.contactNormalHavok[1] +
            input.contactNormalHavok[2] * input.contactNormalHavok[2];
        if (!std::isfinite(normalLengthSquared) || normalLengthSquared <= 0.000001f) {
            return false;
        }

        const float invNormalLength = 1.0f / std::sqrt(normalLengthSquared);
        outResult.contactNormalHavok[0] = input.contactNormalHavok[0] * invNormalLength;
        outResult.contactNormalHavok[1] = input.contactNormalHavok[1] * invNormalLength;
        outResult.contactNormalHavok[2] = input.contactNormalHavok[2] * invNormalLength;
        outResult.contactNormalHavok[3] = input.contactNormalHavok[3];

        std::uint32_t strongestPositiveIndex = pointCount;
        float strongestPositiveImpulse = 0.0f;
        float totalPositiveImpulse = 0.0f;
        for (std::uint32_t i = 0; i < pointCount; ++i) {
            copyContactSignalVector4(input.contactPointsHavok[i], outResult.contactPointsHavok[i]);
            outResult.contactSeparationsHavok[i] = input.contactSeparationsHavok[i];
            outResult.contactImpulses[i] = input.contactImpulses[i];

            const float impulse = input.contactImpulses[i];
            if (!std::isfinite(impulse) || impulse <= 0.0f) {
                continue;
            }
            totalPositiveImpulse += impulse;
            if (contactSignalFinite3(input.contactPointsHavok[i]) &&
                impulse > strongestPositiveImpulse) {
                strongestPositiveImpulse = impulse;
                strongestPositiveIndex = i;
            }
        }

        std::uint32_t selectedIndex = strongestPositiveIndex;
        if (selectedIndex < pointCount) {
            outResult.selectionSource = ContactSignalPointSelectionSource::PositiveImpulse;
        } else if (input.contactIndex < pointCount &&
                   contactSignalFinite3(input.contactPointsHavok[input.contactIndex])) {
            selectedIndex = input.contactIndex;
            outResult.selectionSource = ContactSignalPointSelectionSource::NativeContactIndex;
        } else {
            selectedIndex = pointCount;
            for (std::uint32_t i = 0; i < pointCount; ++i) {
                if (contactSignalFinite3(input.contactPointsHavok[i])) {
                    selectedIndex = i;
                    outResult.selectionSource = ContactSignalPointSelectionSource::FirstFinitePoint;
                    break;
                }
            }
        }

        if (selectedIndex >= pointCount) {
            outResult = {};
            return false;
        }

        outResult.pointCount = pointCount;
        outResult.nativeContactPointIndex = input.contactIndex;
        outResult.contactPointWeightSum = totalPositiveImpulse;
        outResult.selectedPointIndex = selectedIndex;
        copyContactSignalVector4(input.contactPointsHavok[selectedIndex], outResult.contactPointHavok);
        outResult.valid = true;
        return true;
    }
}
