#pragma once

#include "RE/NetImmerse/NiPoint.h"

#include <array>
#include <cstddef>
#include <cstdint>

/*
 * Debug view of the capsule finger solve (GrabFingerCapsuleSweep.h).
 *
 * Before a grab, the hand runs the same solve against the object it has
 * selected, with the hand where it is drawn, and reports the spheres of each
 * finger at the curl the solve chose plus the fingertip's path over the whole
 * sweep. While holding, it reports the spheres along the drawn fingers.
 *
 * The same snapshot is the probe contact check of a close grab
 * (Hand::evaluateGrabProbeContact): the grab commits only when a finger
 * touches the visual mesh, either by its state (Contact, ClosedPastIntersection,
 * Blocked) or by a sphere within kTouchMarginGameUnits of the mesh. Native
 * collision never stands in for that contact: object collision often sits
 * well off the visible surface, which made grabs where no finger touched it.
 */
namespace rock::grab_finger_probe_preview
{
    enum class FingerState : std::uint8_t
    {
        Free,                   // closed to the minimum curl without touching
        Contact,                // closed onto the surface
        ClosedPastIntersection, // the open pose intersected; closed inward past it onto the surface
        Blocked,                // the open pose intersected and nothing inward clears and touches; kept open
    };

    // A finger is 15-20 spheres at the hand collider radii; 5 fingers fit with room.
    inline constexpr std::size_t kMaxSpheresPerHand = 120;
    inline constexpr std::size_t kTipPathPoints = 12;
    // A sphere counts as touching within this gap of the surface.
    inline constexpr float kTouchMarginGameUnits = 0.15f;

    struct Sphere
    {
        RE::NiPoint3 center{};
        float radius = 0.0f;
        std::uint8_t finger = 0;
        bool touching = false;
    };

    struct Finger
    {
        std::array<RE::NiPoint3, kTipPathPoints> tipPath{};
        float curl = 1.0f;
        FingerState state = FingerState::Free;
        std::uint8_t tipPathCount = 0;
        bool valid = false;
    };

    struct Snapshot
    {
        std::array<Sphere, kMaxSpheresPerHand> spheres{};
        std::array<Finger, 5> fingers{};
        RE::NiPoint3 labelAnchor{};
        std::uint32_t sphereCount = 0;
        std::uint32_t triangleCount = 0;
        bool meshCoversWholeObject = true;
        // The mesh crosses the straight line between the live thumb and index
        // tips: the object is between the pads even when neither finger's
        // closing arc meets it (a cartridge, a coin).
        bool meshBetweenThumbAndIndex = false;
        bool holding = false;
        bool valid = false;
    };
}
