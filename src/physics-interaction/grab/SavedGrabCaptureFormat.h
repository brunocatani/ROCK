#pragma once

#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

/*
 * Companion capture record for a saved grab offset: everything needed to
 * REPLAY and SCORE that saved pose offline, with no game running.
 *
 * The offset file next to this one records the ANSWER (where the object sits
 * relative to the hand, and how the hand closed on it). On its own that is not
 * enough to fit or validate a pose solver: the same answer means different
 * things against a different mesh, a different hand, or a different mass. This
 * record carries the QUESTION - the exact mesh ROCK scored, the hand geometry
 * it was posed against, the object's physics, the contacts that resulted, and
 * the seat ROCK itself computed before the user corrected it.
 *
 * Write-only by design: nothing in ROCK reads captures back, so there is no
 * parse() here. They are ground-truth records consumed by offline tooling and
 * by the pose-solver test harness.
 *
 * SPACES. Every frame, point and normal is in one of exactly three spaces, and
 * the field name says which:
 *   - *ProxyLocal    : the live GrabAuthorityProxy palm frame, the SAME space
 *                      the saved offset's pose uses (grab_frame_math::
 *                      objectInGeneratedProxyLocalSpace). The proxy frame is
 *                      the palm pocket frame, so pocket geometry needs no
 *                      separate transform - see TuningCapture for its depth
 *                      and radius.
 *   - *ObjectLocal   : local to the object NODE whose mesh was extracted.
 *                      MeshCapture::nodeInObjectRoot relates it to the object
 *                      root, which is what the saved pose is expressed for.
 *   - *ObjectNode    : relative to that same object node.
 *
 * No engine types, no I/O: the format stays policy-testable.
 */
namespace rock::saved_grab_capture
{
    inline constexpr std::uint32_t kFormatVersion = 1;
    inline constexpr std::size_t kFingerCount = 5;
    inline constexpr std::size_t kFingerJointValueCount = 15;

    /*
     * Rigid frame in the same row convention as the saved offset file:
     * rotate[row * 3 + column], row = the frame's own axis expressed in the
     * reference space.
     */
    struct Frame
    {
        bool valid{ false };
        float translate[3]{ 0.0f, 0.0f, 0.0f };
        float rotate[9]{ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        float scale{ 1.0f };
    };

    /*
     * One driven hand collider, exactly as the live collider set drives it -
     * these are the volumes an object must not end up inside, so a solver
     * fitted against this data is fitted against the real hand rather than an
     * approximation of it.
     */
    struct ColliderSlot
    {
        bool valid{ false };
        std::string role;
        Frame frameProxyLocal;
        float length{ 0.0f };
        float radius{ 0.0f };
        float convexRadius{ 0.0f };
    };

    struct MeshCapture
    {
        bool valid{ false };
        std::uint32_t triangleCount{ 0 };
        // Flat triangle soup, 9 floats per triangle (v0,v1,v2), object-local.
        // This is the cached mesh the grab machinery itself scored, not the
        // NIF: replacers, scale, skinning and node selection are already
        // baked in, so an offline replay cannot silently score other geometry.
        std::vector<float> verticesObjectLocal;
        float aabbMinObjectLocal[3]{ 0.0f, 0.0f, 0.0f };
        float aabbMaxObjectLocal[3]{ 0.0f, 0.0f, 0.0f };
        float areaCentroidObjectLocal[3]{ 0.0f, 0.0f, 0.0f };
        // Object-node local -> object-root local. Identity when the grabbed
        // node IS the root; non-identity for nested collidable nodes, where
        // the saved pose (root-relative) and the mesh (node-relative) would
        // otherwise be silently misaligned.
        Frame nodeInObjectRoot;
    };

    struct PhysicsCapture
    {
        bool valid{ false };
        std::uint32_t bodyId{ 0 };
        // As reported by the runtime body (readGrabEventBodyMass).
        float mass{ 0.0f };
        // Havok body frame relative to the object node, recorded as raw data.
        Frame bodyInObjectNode;
        /*
         * Centre of mass in OBJECT-LOCAL space, from the held body's Havok
         * motion. Mass distribution is not derivable from the render mesh, and
         * "does it balance in the palm" is one of the terms a pose solver has
         * to explain, so this is a first-class field rather than an inference
         * from bodyInObjectNode.
         *
         * Offset authority: this binary's OWN reflection table, verified
         * 2026-07-25 - the hkClassMember record at 142e93930 names hknpMotion
         * member 'centerOfMassAndMassFactor' at +0x00 (xyz = world COM, w =
         * mass factor). Corroborated by the same table's +0x20 packed inertia
         * and +0x40/+0x50 velocities, both of which ROCK already depends on at
         * runtime.
         *
         * comTrusted is false when the value fails the mesh-bounds
         * plausibility gate. A wrong offset must degrade into a flagged record,
         * never a silently wrong label that gets fitted against.
         */
        bool hasCenterOfMass{ false };
        bool comTrusted{ false };
        float comObjectLocal[3]{ 0.0f, 0.0f, 0.0f };
    };

    /*
     * Where the hand actually touched the object in the saved pose, probed at
     * save time against the live mesh. This is the numeric definition of
     * "properly held" for that object: which fingers reached surface, where,
     * and against which surface normal.
     */
    struct FingerContactCapture
    {
        bool touching{ false };
        float pointObjectLocal[3]{ 0.0f, 0.0f, 0.0f };
        float normalObjectLocal[3]{ 0.0f, 0.0f, 0.0f };
        float curl{ 0.0f };
    };

    /*
     * The seat ROCK computed for this grab, before the user corrected it into
     * the saved pose. Every capture therefore carries a before/after pair: the
     * error a new solver has to remove, and the reason strings explaining what
     * the current stack decided and why.
     */
    struct SeatCapture
    {
        bool valid{ false };
        Frame objectProxyLocal;
        std::string shapeClass;
        float elongationRatio{ 0.0f };
        float secondElongationRatio{ 0.0f };
        float alignmentAngleDegrees{ 0.0f };
        std::string alignmentReason;
        float rollAngleDegrees{ 0.0f };
        std::string rollReason;
        float depthGameUnits{ 0.0f };
        float depthOffsetGameUnits{ 0.0f };
        std::string depthReason;
        float penetrationBackstopGameUnits{ 0.0f };
        std::string penetrationBackstopReason;
        float gripPointObjectLocal[3]{ 0.0f, 0.0f, 0.0f };
        float pivotProxyLocal[3]{ 0.0f, 0.0f, 0.0f };
        std::string seatMode;
        std::string pivotAuthoritySource;
    };

    // Grab tuning in force when the capture was taken, so a label recorded
    // under one tuning can never be silently read as if it were another.
    struct TuningCapture
    {
        float seatDepthMaxGameUnits{ 0.0f };
        float seatDepthFootprintRadiusGameUnits{ 0.0f };
        float seatPenetrationBackstopFootprintRadiusGameUnits{ 0.0f };
        float seatDepthSkinGameUnits{ 0.0f };
        float gripInsetGameUnits{ 0.0f };
        float pullPresentationMinElongationRatio{ 0.0f };
        float pullPresentationGripAxisTiltDegrees{ 0.0f };
        float seatRollMinSecondElongationRatio{ 0.0f };
        float pocketDepthGameUnits{ 0.0f };
        float pocketRadiusGameUnits{ 0.0f };
    };

    struct HandCapture
    {
        bool present{ false };
        // "closeGrab" | "pullCatch" | "forceGrab" | "unknown"
        std::string acquisition;
        // The saved (user-verified) pose, duplicated here so a capture file is
        // readable standalone without joining it to the offset file.
        Frame objectProxyLocal;
        float objectScale{ 1.0f };
        MeshCapture mesh;
        PhysicsCapture physics;
        ColliderSlot palm;
        std::vector<ColliderSlot> fingerSegments;
        std::array<FingerContactCapture, kFingerCount> fingerContacts{};
        float fingerCurls[kFingerCount]{};
        bool hasFingerJointValues{ false };
        float fingerJointValues[kFingerJointValueCount]{};
        SeatCapture seat;
        TuningCapture tuning;
    };

    /*
     * One file per object PER HAND. Unlike the offset file, captures are never
     * read back, so a merged right+left document could not be updated without
     * discarding the other hand's record - the hand is part of the identity
     * instead.
     */
    struct SavedGrabCaptureFile
    {
        std::uint32_t formatVersion{ kFormatVersion };
        saved_grab_offset::FormRef object;
        std::string objectName;
        // "right" | "left"
        std::string hand;
        std::string rockVersion;
        // ISO-8601 UTC, e.g. "2026-07-25T18:04:11Z".
        std::string capturedUtc;
        HandCapture capture;
    };

    [[nodiscard]] std::string serialize(const SavedGrabCaptureFile& file);
}
