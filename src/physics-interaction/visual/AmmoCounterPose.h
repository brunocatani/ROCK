#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

/*
 * Ammo counter placement: DIAMOND's weapon HUD, mirrored for the left hand.
 * The counter is fixed to the weapon at the firing palm, captured in weapon
 * space once per weapon and hand so transient hand corrections never move
 * it. Its plane lies on the palm side of the firing hand, the weapon's inside:
 * the weapon's left (-X) for a right firing hand, its right (+X) for a left
 * one. It sits kOutward game units off that side and fades in only while it
 * faces the head: hidden at 60 degrees, the subtle full opacity at 45.
 * Looking never moves or rotates it. Like the vanilla counter, the loaded
 * rounds sit left of the anchor and the reserve, smaller, to its right with
 * aligned bottoms.
 *
 * Weapon geometry is +Y muzzle-forward, +X right, +Z up. Panels are RPS UI
 * number overlays: width/height is the overlay contract's aspect and the
 * digits fill 85% of the height, centered in the panel.
 */
namespace rock::ammo_counter_pose
{
    using Point = std::array<float, 3>;

    inline constexpr float kNumberAspect = 1024.0f / 480.0f;
    inline constexpr float kDigitHeightFraction = 0.85f;
    inline constexpr float kLoadedWidth = 2.6f;
    inline constexpr float kReserveWidth = 1.6f;
    // Panel-plane offsets of each number's center from the anchor.
    inline constexpr float kLoadedRight = -1.0f;
    inline constexpr float kReserveRight = 1.2f;
    // DIAMOND's default ammo HUD offset: outward from the weapon's inside.
    inline constexpr float kOutward = 6.0f;
    inline constexpr float kMaxOpacity = 0.55f;

    struct Panel
    {
        Point center{};
        float width{ 0.0f };
        float height{ 0.0f };
    };

    struct Pose
    {
        Point right{};
        Point up{};
        Point front{};
        Panel loaded{};
        Panel reserve{};
        float opacity{ 0.0f };
        bool valid{ false };
    };

    // The firing palm in weapon space, kept while the weapon and hand stay.
    class WeaponAnchor
    {
    public:
        struct Identity
        {
            std::uint32_t weaponSlot{ 0 };
            std::uint32_t weaponFormId{ 0 };
            std::uint64_t weaponGenerationKey{ 0 };

            friend constexpr bool operator==(const Identity&, const Identity&) = default;
        };

        void clear() noexcept { _valid = false; }

        [[nodiscard]] const Point& update(const Identity& identity, const Point& palmInWeapon) noexcept
        {
            if (!_valid || !(identity == _identity)) {
                _identity = identity;
                _palmInWeapon = palmInWeapon;
                _valid = true;
            }
            return _palmInWeapon;
        }

    private:
        Identity _identity{};
        Point _palmInWeapon{};
        bool _valid{ false };
    };

    namespace detail
    {
        inline bool normalize(Point& value) noexcept
        {
            const float length = std::hypot(value[0], value[1], value[2]);
            if (!std::isfinite(length) || length < 0.0001f) return false;
            for (auto& component : value) component /= length;
            return true;
        }

        inline float dot(const Point& a, const Point& b) noexcept
        {
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        inline Point cross(const Point& a, const Point& b) noexcept
        {
            return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        }

        inline Point offset(const Point& origin, const Point& axisA, float a, const Point& axisB, float b) noexcept
        {
            return { origin[0] + axisA[0] * a + axisB[0] * b, origin[1] + axisA[1] * a + axisB[1] * b,
                origin[2] + axisA[2] * a + axisB[2] * b };
        }
    }

    // palm: the anchored firing palm in world space. weaponRight and weaponUp:
    // the weapon's +X and +Z in world space.
    [[nodiscard]] inline Pose resolve(const Point& palm, const Point& weaponRight, const Point& weaponUp,
        const bool firingHandIsLeft, const Point& head) noexcept
    {
        using namespace detail;
        for (unsigned i = 0; i < 3; ++i) {
            if (!std::isfinite(palm[i]) || !std::isfinite(head[i])) return {};
        }
        Pose pose{};
        const float inside = firingHandIsLeft ? 1.0f : -1.0f;
        pose.front = { weaponRight[0] * inside, weaponRight[1] * inside, weaponRight[2] * inside };
        if (!normalize(pose.front)) return {};
        pose.right = cross(weaponUp, pose.front);
        if (!normalize(pose.right)) return {};
        pose.up = cross(pose.front, pose.right);
        if (!normalize(pose.up)) return {};

        const Point anchor = offset(palm, pose.front, kOutward, pose.up, 0.0f);
        pose.loaded = { offset(anchor, pose.right, kLoadedRight, pose.up, 0.0f), kLoadedWidth, kLoadedWidth / kNumberAspect };
        const float reserveDrop = (pose.loaded.height - kReserveWidth / kNumberAspect) * kDigitHeightFraction * 0.5f;
        pose.reserve = { offset(anchor, pose.right, kReserveRight, pose.up, -reserveDrop), kReserveWidth,
            kReserveWidth / kNumberAspect };

        Point toHead{ head[0] - anchor[0], head[1] - anchor[1], head[2] - anchor[2] };
        if (!normalize(toHead)) return {};
        pose.opacity = kMaxOpacity * std::clamp((dot(pose.front, toHead) - 0.5f) / (0.70710678f - 0.5f), 0.0f, 1.0f);
        pose.valid = true;
        return pose;
    }
}
