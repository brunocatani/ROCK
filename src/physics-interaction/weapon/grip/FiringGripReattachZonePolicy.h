// The firing-grip reattach zone: one region on each side of the weapon around
// the captured grip point. A free palm inside a region may take the firing grip
// back.
#pragma once

#include "physics-interaction/weapon/authored/AuthoredWeaponGripActivationPolicy.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace rock::firing_grip_reattach_zone_policy
{
    using Vec3 = authored_weapon_grip_activation_policy::Vec3;

    /*
     * Firing-grip reattach zone, one region per side of the weapon around the
     * captured grip point. The free palm re-takes the grip only while it sits
     * inside one of them. Either free hand may squeeze from either side
     * (ambidextrous takeover); the gate is symmetric in the lateral sign and
     * only reports which side the palm sits on. Neither shape needs approach
     * history.
     *
     * Sector (default): a quarter disc in the plane of the lateral and grip
     * DOWN axes, swept from straight out to straight down the grip, so a palm
     * arriving from the side or from below the grip both count. Its top edge
     * sits the cross radius above the grip point (the grip top on a pistol);
     * the cross radius also bounds it front and back along the barrel.
     * Cylinder (handling-provider opt-in): a cylinder along the lateral axis,
     * the reach long and the cross radius wide.
     */
    inline constexpr float kIndicatorOffsetGameUnits =
        authored_weapon_grip_activation_policy::kIndicatorOffsetGameUnits;
    inline constexpr float kAxisOrthogonalityTolerance = 0.05f;

    enum class Shape : std::uint8_t
    {
        Sector,
        Cylinder,
    };

    enum class Side : std::uint8_t
    {
        None,
        Left,
        Right,
    };

    struct ZoneInput
    {
        Vec3 gripWorld{};
        Vec3 palmWorld{};
        // Weapon LEFT at the grip: the seated canonical right palm normal.
        Vec3 weaponLeftAxisWorld{};
        // Down the grip from the same seated hand; used by the sector only.
        Vec3 weaponDownAxisWorld{};
        // Sector radius, or cylinder length from the grip along each side.
        float reachGameUnits{ 0.0f };
        // Cylinder cross-section radius; sector rise above the grip point
        // and half-thickness along the barrel.
        float radiusGameUnits{ 0.0f };
        Shape shape{ Shape::Sector };
    };

    struct ZoneResult
    {
        // Signed distance along the lateral axis; positive is weapon left.
        float alongAxisGameUnits{ 0.0f };
        float perpendicularDistanceGameUnits{ 0.0f };
        float radialDistanceGameUnits{ 0.0f };
        // Sector only: distance down the grip (negative above the grip
        // point) and along the barrel.
        float downGameUnits{ 0.0f };
        float forwardGameUnits{ 0.0f };
        Side side{ Side::None };
        bool axisValid{ false };
        // Cylinder: within the length. Sector: within the quarter disc.
        bool reachPass{ false };
        // Cylinder: within the cross radius. Sector: within the top edge and
        // the front/back thickness.
        bool radiusPass{ false };
        bool inside{ false };
        // Indicator marker seat: the grip point pushed out along the side the
        // palm sits on, so the marker never sits inside the weapon.
        Vec3 indicatorWorld{};
        bool indicatorValid{ false };
    };

    [[nodiscard]] inline bool isFinite(const Vec3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    [[nodiscard]] constexpr Vec3 cross(const Vec3& lhs, const Vec3& rhs)
    {
        return Vec3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    [[nodiscard]] inline ZoneResult evaluateZone(const ZoneInput& input)
    {
        namespace activation = authored_weapon_grip_activation_policy;
        ZoneResult result{};
        if (!isFinite(input.gripWorld) || !isFinite(input.palmWorld)) {
            return result;
        }
        const bool reachValid = std::isfinite(input.reachGameUnits) && input.reachGameUnits >= 0.0f;
        const bool radiusValid = std::isfinite(input.radiusGameUnits) && input.radiusGameUnits >= 0.0f;
        Vec3 lateralAxis{};
        result.axisValid = activation::tryNormalize(input.weaponLeftAxisWorld, lateralAxis);
        // The sector's DOWN must come from the same seated hand frame as the
        // lateral axis; a skewed pair means the frame is not a rotation.
        Vec3 downAxis{};
        if (result.axisValid && input.shape == Shape::Sector) {
            result.axisValid = activation::tryNormalize(input.weaponDownAxisWorld, downAxis) &&
                std::abs(activation::dot(lateralAxis, downAxis)) <= kAxisOrthogonalityTolerance;
        }
        if (!reachValid || !radiusValid || !result.axisValid) {
            return result;
        }

        const Vec3 offset = activation::subtract(input.palmWorld, input.gripWorld);
        result.radialDistanceGameUnits = activation::length(offset);
        const float along = activation::dot(offset, lateralAxis);
        const Vec3 perpendicular{
            offset.x - lateralAxis.x * along,
            offset.y - lateralAxis.y * along,
            offset.z - lateralAxis.z * along,
        };
        const float perpendicularDistance = activation::length(perpendicular);
        if (!std::isfinite(along) || !std::isfinite(perpendicularDistance)) {
            return result;
        }
        result.alongAxisGameUnits = along;
        result.perpendicularDistanceGameUnits = perpendicularDistance;
        result.side = along < 0.0f ? Side::Right : Side::Left;

        if (input.shape == Shape::Sector) {
            const float down = activation::dot(offset, downAxis);
            const float forward = activation::dot(offset, cross(lateralAxis, downAxis));
            if (!std::isfinite(down) || !std::isfinite(forward)) {
                return result;
            }
            result.downGameUnits = down;
            result.forwardGameUnits = forward;
            // Above the grip point the disc continues as a band up to the
            // top edge, so the outline runs straight down before it curves.
            const float discDown = (std::max)(down, 0.0f);
            result.reachPass = along * along + discDown * discDown <= input.reachGameUnits * input.reachGameUnits;
            result.radiusPass = down >= -input.radiusGameUnits && std::abs(forward) <= input.radiusGameUnits;
        } else {
            result.reachPass = std::abs(along) <= input.reachGameUnits;
            result.radiusPass = perpendicularDistance <= input.radiusGameUnits;
        }
        result.inside = result.reachPass && result.radiusPass;
        if (result.inside) {
            const float indicatorSign = result.side == Side::Left ? 1.0f : -1.0f;
            result.indicatorWorld = Vec3{
                input.gripWorld.x + lateralAxis.x * indicatorSign * kIndicatorOffsetGameUnits,
                input.gripWorld.y + lateralAxis.y * indicatorSign * kIndicatorOffsetGameUnits,
                input.gripWorld.z + lateralAxis.z * indicatorSign * kIndicatorOffsetGameUnits,
            };
            result.indicatorValid = isFinite(result.indicatorWorld);
        }
        return result;
    }

    struct OutlineSegment
    {
        Vec3 start{};
        Vec3 end{};
    };

    // Segments outlineSegments writes at most: 52 for the sector, 40 for the cylinder.
    inline constexpr std::size_t kMaxOutlineSegments = 52;

    /**
     * The zone's wireframe outline in world space, the region evaluateZone tests (palm and indicator unused).
     * Sector: each side's quarter disc and the band above it, drawn on the front and back faces with edges joining
     * them. Cylinder: rings at both ends and at the grip, joined by four lines. Writes nothing for an invalid zone.
     */
    [[nodiscard]] inline std::size_t outlineSegments(const ZoneInput& input, std::span<OutlineSegment> output)
    {
        namespace activation = authored_weapon_grip_activation_policy;
        constexpr std::size_t kArcSegments = 8;
        constexpr std::size_t kRingSegments = 12;
        const float reach = input.reachGameUnits;
        const float radius = input.radiusGameUnits;
        Vec3 lateral{};
        if (!isFinite(input.gripWorld) || !std::isfinite(reach) || !std::isfinite(radius) || reach <= 0.0f ||
            radius <= 0.0f || !activation::tryNormalize(input.weaponLeftAxisWorld, lateral)) {
            return 0;
        }
        // The two axes across the lateral one: the sector's down and forward, or any pair for the cylinder.
        Vec3 down{};
        if (input.shape == Shape::Sector) {
            if (!activation::tryNormalize(input.weaponDownAxisWorld, down) ||
                std::abs(activation::dot(lateral, down)) > kAxisOrthogonalityTolerance) {
                return 0;
            }
        } else {
            const Vec3 reference = std::abs(lateral.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 1.0f, 0.0f, 0.0f };
            if (!activation::tryNormalize(cross(reference, lateral), down)) {
                return 0;
            }
        }
        Vec3 forward{};
        if (!activation::tryNormalize(cross(lateral, down), forward)) {
            return 0;
        }

        const auto point = [&](const float along, const float downAmount, const float forwardAmount) {
            return Vec3{
                input.gripWorld.x + lateral.x * along + down.x * downAmount + forward.x * forwardAmount,
                input.gripWorld.y + lateral.y * along + down.y * downAmount + forward.y * forwardAmount,
                input.gripWorld.z + lateral.z * along + down.z * downAmount + forward.z * forwardAmount,
            };
        };
        std::size_t count = 0;
        const auto add = [&](const Vec3& start, const Vec3& end) {
            if (count < output.size()) {
                output[count++] = OutlineSegment{ start, end };
            }
        };
        constexpr float kQuarterTurn = 1.5707963f;

        if (input.shape == Shape::Sector) {
            // Per side, in (along, down): top edge, the drop to the grip's height, the quarter arc down the grip,
            // and the inner edge back up.
            std::array<std::pair<float, float>, kArcSegments + 3> outline{};
            outline[0] = { 0.0f, -radius };
            outline[1] = { reach, -radius };
            for (std::size_t segment = 0; segment <= kArcSegments; ++segment) {
                const float angle = static_cast<float>(segment) * kQuarterTurn / static_cast<float>(kArcSegments);
                outline[2 + segment] = { std::cos(angle) * reach, std::sin(angle) * reach };
            }
            for (const float sideSign : { 1.0f, -1.0f }) {
                for (const float face : { -radius, radius }) {
                    for (std::size_t index = 0; index < outline.size(); ++index) {
                        const auto& from = outline[index];
                        const auto& to = outline[(index + 1) % outline.size()];
                        add(point(sideSign * from.first, from.second, face), point(sideSign * to.first, to.second, face));
                    }
                }
                for (const std::size_t corner : { std::size_t{ 0 }, std::size_t{ 1 }, std::size_t{ 2 }, outline.size() - 1 }) {
                    const auto& at = outline[corner];
                    add(point(sideSign * at.first, at.second, -radius), point(sideSign * at.first, at.second, radius));
                }
            }
            return count;
        }

        for (const float along : { -reach, 0.0f, reach }) {
            for (std::size_t segment = 0; segment < kRingSegments; ++segment) {
                const float from = static_cast<float>(segment) * 4.0f * kQuarterTurn / static_cast<float>(kRingSegments);
                const float to = static_cast<float>(segment + 1) * 4.0f * kQuarterTurn / static_cast<float>(kRingSegments);
                add(point(along, std::cos(from) * radius, std::sin(from) * radius),
                    point(along, std::cos(to) * radius, std::sin(to) * radius));
            }
        }
        for (std::size_t line = 0; line < 4; ++line) {
            const float angle = static_cast<float>(line) * kQuarterTurn;
            const float downAmount = std::cos(angle) * radius;
            const float forwardAmount = std::sin(angle) * radius;
            add(point(-reach, downAmount, forwardAmount), point(reach, downAmount, forwardAmount));
        }
        return count;
    }

    [[nodiscard]] constexpr const char* sideName(const Side side)
    {
        switch (side) {
        case Side::Left:
            return "LEFT";
        case Side::Right:
            return "RIGHT";
        case Side::None:
        default:
            return "NONE";
        }
    }

    [[nodiscard]] constexpr const char* shapeName(const Shape shape)
    {
        switch (shape) {
        case Shape::Cylinder:
            return "CYLINDER";
        case Shape::Sector:
        default:
            return "SECTOR";
        }
    }
}
