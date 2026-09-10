#pragma once

#include <cstdint>
#include <cmath>
#include <array>

namespace RE { class NiAVObject; class NiPoint3; class NiTransform; class TESObjectWEAP; }

namespace rock::vanilla_weapon_grip_frame
{
    [[nodiscard]] constexpr bool isPipeWeapon(std::uint32_t formId) noexcept
    {
        return formId == 0x00024F55 || formId == 0x0014831A || formId == 0x0014831B;
    }

    struct PipePoseSignature
    {
        std::array<float, 3> translate;
        std::array<float, 9> rotate;
    };
    // Captured vanilla PipeRifle, PipeRiflePistol, and
    // HandmadeRevolverGripStraight idle grips. Paths alone cannot identify
    // these: replacement animations commonly retain Bethesda's filenames.
    inline constexpr std::array<PipePoseSignature, 3> kVanillaPipePoses{{
        {{2.086855f, -9.781346f, 2.116619f},
         {-0.0043318f, 0.7026392f, -0.7115333f, 0.9986392f, 0.0400211f, 0.0334413f, 0.0519736f, -0.7104201f, -0.7018564f}},
        {{3.2115898f, -9.7660217f, -6.1378632f},
         {-0.12021494f, 0.99067980f, 0.06404734f, 0.98687428f, 0.11224705f, 0.11610263f, 0.10783139f, 0.07716393f, -0.99117017f}},
        {{3.212708f, -8.610003f, -5.167619f},
         {-0.1211498f, 0.9905422f, 0.0644115f, 0.9867545f, 0.1131258f, 0.1162679f, 0.1078817f, 0.0776441f, -0.9911273f}},
    }};

    template <class Transform>
    [[nodiscard]] bool matchesVanillaPipePose(std::uint32_t formId, const Transform& hand)
    {
        if (!isPipeWeapon(formId) || !(std::abs(hand.scale - 1.0f) <= 0.00001f)) return false;
        for (const auto& pose : kVanillaPipePoses) {
            bool matches = std::abs(hand.translate.x - pose.translate[0]) <= 0.001f &&
                std::abs(hand.translate.y - pose.translate[1]) <= 0.001f &&
                std::abs(hand.translate.z - pose.translate[2]) <= 0.001f;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    matches = matches && std::abs(hand.rotate.entry[row][col] - pose.rotate[row * 3 + col]) <= 0.00001f;
                }
            }
            if (matches) return true;
        }
        return false;
    }

    template <class Transform, class Rotation, class Point, class MapPoint>
    [[nodiscard]] Transform withWristRotationAtPalm(const Transform& hand, const Rotation& rotation,
        const Point& palmLocal, MapPoint&& mapPoint)
    {
        const auto anchor = mapPoint(hand, palmLocal);
        auto result = hand;
        result.rotate = rotation;
        const auto moved = mapPoint(result, palmLocal);
        result.translate.x += anchor.x - moved.x;
        result.translate.y += anchor.y - moved.y;
        result.translate.z += anchor.z - moved.z;
        return result;
    }

    void correctPipeWrist(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* model,
        RE::NiTransform& hand);

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
