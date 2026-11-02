#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

/*
 * Ammo counter placement for the three ammo counter styles.
 *
 * Minimal and Immersive: DIAMOND's weapon HUD, mirrored for the left hand.
 * The counter is fixed to the weapon at its firing grip: the authored firing
 * seat's palm in weapon space, so hand motion never moves it. Its plane lies on the palm side of the firing hand, the weapon's inside:
 * the weapon's left (-X) for a right firing hand, its right (+X) for a left
 * one. It sits kOutward game units off that side and fades in only while it
 * faces the head: hidden at 60 degrees, full opacity at 45.
 * Looking never moves or rotates it. Minimal, like the vanilla counter's
 * numbers, puts the loaded rounds left of the anchor and the reserve,
 * smaller, to its right with aligned bottoms. Immersive centers one HUD-style
 * panel (loaded over reserve in a bracket frame) on the same anchor.
 *
 * Vanilla: the game's own ammo screen on the back of the hand. FRIK's
 * _primaryWand.nif puts the wand HUD there; its ammo:0 quad is the screen the
 * vanilla counter draws on. The right hand's counter takes that quad's world
 * placement; the left hand's mirrors it across the hands, as ROCK mirrors
 * every hand-local relation: hand-local Z is reflected.
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

    // Immersive: RPS UI fits the bracketed HUD block to 92% of the panel
    // height, so this height gives digits of about 0.8 game units, smaller
    // than Minimal's loaded rounds.
    inline constexpr float kImmersivePanelHeight = 2.9f;
    inline constexpr float kImmersiveMaxOpacity = 0.85f;

    // Vanilla: the ammo:0 quad in its own node's space (FRIK
    // _primaryWand.nif). Texture right runs along kVanillaQuadRight and
    // texture up along kVanillaQuadUp; their cross product is the quad's
    // authored normal, the side that faces the viewer.
    inline constexpr Point kVanillaQuadCenter{ 11.519531f, -7.375000f, -5.960449f };
    inline constexpr Point kVanillaQuadRight{ 1.554687f, -0.664062f, -4.730469f };
    inline constexpr Point kVanillaQuadUp{ 0.0f, 5.945313f, -0.832032f };
    // The counter block takes about 70% of the quad's height; RPS UI fits
    // the block to 92% of the panel height.
    inline constexpr float kVanillaPanelHeightOfQuad = 0.76f;
    // A flat HUD screen is not seen from behind: the counter fades out as the
    // quad turns edge-on, between 80 and 90 degrees from the eye.
    inline constexpr float kVanillaFacingHidden = 0.0f;
    inline constexpr float kVanillaFacingFull = 0.17f;

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
        // Minimal: one panel per number.
        Panel loaded{};
        Panel reserve{};
        // Immersive and Vanilla: one panel holds the whole counter.
        Panel counter{};
        float opacity{ 0.0f };
        bool valid{ false };
    };

    // A node's world placement: the world images of its local axes (unit
    // length), its translation and its uniform scale.
    struct Frame
    {
        Point x{};
        Point y{};
        Point z{};
        Point origin{};
        float scale{ 1.0f };
    };

    namespace detail
    {
        inline bool normalize(Point& value) noexcept
        {
            const float length = std::hypot(value[0], value[1], value[2]);
            if (!std::isfinite(length) || length < 0.0001f)
                return false;
            for (auto& component : value)
                component /= length;
            return true;
        }

        inline float dot(const Point& a, const Point& b) noexcept { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

        inline Point cross(const Point& a, const Point& b) noexcept
        {
            return { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        }

        inline Point offset(const Point& origin, const Point& axisA, float a, const Point& axisB, float b) noexcept
        {
            return { origin[0] + axisA[0] * a + axisB[0] * b, origin[1] + axisA[1] * a + axisB[1] * b,
                origin[2] + axisA[2] * a + axisB[2] * b };
        }

        inline bool finite(const Point& value) noexcept
        {
            return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
        }

        inline bool finite(const Frame& frame) noexcept
        {
            return finite(frame.x) && finite(frame.y) && finite(frame.z) && finite(frame.origin) &&
                std::isfinite(frame.scale) && frame.scale > 0.0001f;
        }

        inline Point toWorldVector(const Frame& frame, const Point& local) noexcept
        {
            Point result{};
            for (unsigned i = 0; i < 3; ++i) {
                result[i] = frame.x[i] * local[0] + frame.y[i] * local[1] + frame.z[i] * local[2];
            }
            return result;
        }

        inline Point toWorldPoint(const Frame& frame, const Point& local) noexcept
        {
            const auto vector = toWorldVector(frame, local);
            return { frame.origin[0] + vector[0] * frame.scale, frame.origin[1] + vector[1] * frame.scale,
                frame.origin[2] + vector[2] * frame.scale };
        }

        inline Point toLocalVector(const Frame& frame, const Point& world) noexcept
        {
            return { dot(world, frame.x), dot(world, frame.y), dot(world, frame.z) };
        }

        inline Point toLocalPoint(const Frame& frame, const Point& world) noexcept
        {
            const Point relative{ world[0] - frame.origin[0], world[1] - frame.origin[1], world[2] - frame.origin[2] };
            auto local = toLocalVector(frame, relative);
            for (auto& component : local)
                component /= frame.scale;
            return local;
        }

        // The left hand's image of a right-hand-local value.
        inline Point mirrorAcrossHands(Point local) noexcept
        {
            local[2] = -local[2];
            return local;
        }

        // The weapon-inside counter plane at the firing palm and how directly
        // it faces the head (0 at 60 degrees or more, 1 at 45 or less).
        struct WeaponInside
        {
            Point right{};
            Point up{};
            Point front{};
            Point anchor{};
            float facing{ 0.0f };
        };

        inline bool weaponInside(const Point& palm, const Point& weaponRight, const Point& weaponUp,
            const bool firingHandIsLeft, const Point& head, WeaponInside& out) noexcept
        {
            if (!finite(palm) || !finite(head))
                return false;
            const float inside = firingHandIsLeft ? 1.0f : -1.0f;
            out.front = { weaponRight[0] * inside, weaponRight[1] * inside, weaponRight[2] * inside };
            if (!normalize(out.front))
                return false;
            out.right = cross(weaponUp, out.front);
            if (!normalize(out.right))
                return false;
            out.up = cross(out.front, out.right);
            if (!normalize(out.up))
                return false;
            out.anchor = offset(palm, out.front, kOutward, out.up, 0.0f);
            Point toHead{ head[0] - out.anchor[0], head[1] - out.anchor[1], head[2] - out.anchor[2] };
            if (!normalize(toHead))
                return false;
            out.facing = std::clamp((dot(out.front, toHead) - 0.5f) / (0.70710678f - 0.5f), 0.0f, 1.0f);
            return true;
        }
    }

    // Minimal. palm: the anchored firing palm in world space. weaponRight and
    // weaponUp: the weapon's +X and +Z in world space.
    [[nodiscard]] inline Pose resolve(const Point& palm, const Point& weaponRight, const Point& weaponUp,
        const bool firingHandIsLeft, const Point& head) noexcept
    {
        using namespace detail;
        WeaponInside plane{};
        if (!weaponInside(palm, weaponRight, weaponUp, firingHandIsLeft, head, plane))
            return {};
        Pose pose{};
        pose.right = plane.right;
        pose.up = plane.up;
        pose.front = plane.front;
        pose.loaded = { offset(plane.anchor, pose.right, kLoadedRight, pose.up, 0.0f), kLoadedWidth,
            kLoadedWidth / kNumberAspect };
        const float reserveDrop = (pose.loaded.height - kReserveWidth / kNumberAspect) * kDigitHeightFraction * 0.5f;
        pose.reserve = { offset(plane.anchor, pose.right, kReserveRight, pose.up, -reserveDrop), kReserveWidth,
            kReserveWidth / kNumberAspect };
        pose.opacity = kMaxOpacity * plane.facing;
        pose.valid = true;
        return pose;
    }

    // Immersive: Minimal's plane, anchor and fade with one HUD-style panel.
    [[nodiscard]] inline Pose resolveImmersive(const Point& palm, const Point& weaponRight, const Point& weaponUp,
        const bool firingHandIsLeft, const Point& head) noexcept
    {
        using namespace detail;
        WeaponInside plane{};
        if (!weaponInside(palm, weaponRight, weaponUp, firingHandIsLeft, head, plane))
            return {};
        Pose pose{};
        pose.right = plane.right;
        pose.up = plane.up;
        pose.front = plane.front;
        pose.counter = { plane.anchor, kImmersivePanelHeight * kNumberAspect, kImmersivePanelHeight };
        pose.opacity = kImmersiveMaxOpacity * plane.facing;
        pose.valid = true;
        return pose;
    }

    // Vanilla. quadNode: the ammo:0 node's world frame. On the left hand,
    // rightHandBone and leftHandBone are the hand bones' world frames: the
    // quad's placement relative to the right hand is mirrored onto the left.
    // opacity: the game's HUD opacity.
    [[nodiscard]] inline Pose resolveVanilla(const Frame& quadNode, const bool onLeftHand, const Frame& rightHandBone,
        const Frame& leftHandBone, const Point& head, const float opacity) noexcept
    {
        using namespace detail;
        if (!finite(quadNode) || !finite(head) || !std::isfinite(opacity))
            return {};
        Point center = toWorldPoint(quadNode, kVanillaQuadCenter);
        Point up = toWorldVector(quadNode, kVanillaQuadUp);
        const Point right = toWorldVector(quadNode, kVanillaQuadRight);
        const float quadHeight = std::hypot(kVanillaQuadUp[0], kVanillaQuadUp[1], kVanillaQuadUp[2]) * quadNode.scale;
        Point front = cross(right, up);
        if (onLeftHand) {
            if (!finite(rightHandBone) || !finite(leftHandBone))
                return {};
            center = toWorldPoint(leftHandBone, mirrorAcrossHands(toLocalPoint(rightHandBone, center)));
            up = toWorldVector(leftHandBone, mirrorAcrossHands(toLocalVector(rightHandBone, up)));
            front = toWorldVector(leftHandBone, mirrorAcrossHands(toLocalVector(rightHandBone, front)));
        }
        Pose pose{};
        pose.front = front;
        if (!normalize(pose.front))
            return {};
        pose.right = cross(up, pose.front);
        if (!normalize(pose.right))
            return {};
        pose.up = cross(pose.front, pose.right);
        if (!normalize(pose.up))
            return {};
        if (!finite(center) || !std::isfinite(quadHeight) || quadHeight <= 0.0001f)
            return {};
        const float height = quadHeight * kVanillaPanelHeightOfQuad;
        pose.counter = { center, height * kNumberAspect, height };
        Point toHead{ head[0] - center[0], head[1] - center[1], head[2] - center[2] };
        if (!normalize(toHead))
            return {};
        const float facing = std::clamp(
            (dot(pose.front, toHead) - kVanillaFacingHidden) / (kVanillaFacingFull - kVanillaFacingHidden), 0.0f, 1.0f);
        pose.opacity = std::clamp(opacity, 0.0f, 1.0f) * facing;
        pose.valid = true;
        return pose;
    }
}
