#pragma once

#include <cstdint>
#include <cmath>

namespace RE { class NiAVObject; class NiPoint3; }

namespace rock::vanilla_weapon_grip_frame
{
    [[nodiscard]] constexpr bool hasVanillaModelRegistration(std::uint32_t formId) noexcept
    {
        return formId == 0x0015B043;
    }

    template <class Transform>
    [[nodiscard]] bool registrationAgrees(const Transform& receiver, const Transform& marker)
    {
        bool valid = std::abs(receiver.scale - 1.0f) <= 0.001f && std::abs(marker.scale - 1.0f) <= 0.001f &&
            std::isfinite(receiver.translate.x) && std::isfinite(receiver.translate.y) && std::isfinite(receiver.translate.z) &&
            std::abs(receiver.translate.x - marker.translate.x) <= 0.001f &&
            std::abs(receiver.translate.y - marker.translate.y) <= 0.001f &&
            std::abs(receiver.translate.z - marker.translate.z) <= 0.001f;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                const float expected = row == col ? 1.0f : 0.0f;
                valid = valid && std::abs(receiver.rotate.entry[row][col] - expected) <= 0.001f &&
                    std::abs(marker.rotate.entry[row][col] - expected) <= 0.001f;
            }
        }
        return valid;
    }
    // Animation poses stay in the library's original coordinate system.
    // Convert a borrowed copy for the actual model being gripped. No scene
    // writes, retained nodes, preset offsets, or animation rotation changes.
    [[nodiscard]] bool resolveModelTranslation(std::uint32_t formId,
        const RE::NiAVObject* model, RE::NiPoint3& outTranslation);

    template <class Transform, class Point>
    [[nodiscard]] constexpr Transform translateGrip(const Transform& authored, const Point& displacement)
    {
        auto result = authored;
        result.translate.x += displacement.x;
        result.translate.y += displacement.y;
        result.translate.z += displacement.z;
        return result;
    }
}
