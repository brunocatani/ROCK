#pragma once

/*
 * Grab core policy is grouped here to keep object preparation, lifecycle, canonical frames, frame math, interaction decisions, and pull motion together.
 */


// ---- ActiveObjectPrepPolicy.h ----

#include <cstdint>

namespace frik::rock::active_object_prep_policy
{
    /*
     * Active grab/pull temporarily promotes the selected object tree through
     * FO4VR's recursive SetMotion wrapper before the constraint exists. If the
     * later setup step fails, ROCK must undo that temporary promotion for every
     * original non-dynamic state; successful grabs still keep objects dynamic on
     * release so they can settle naturally.
     */
    inline constexpr std::uint16_t kMotionPresetDynamic = 1;

    inline constexpr bool shouldRestoreMotionAfterFailedActivePrep(bool recursiveMotionWasConverted, std::uint16_t originalMotionPropsId)
    {
        return recursiveMotionWasConverted && originalMotionPropsId != kMotionPresetDynamic;
    }
}


// ---- ActiveGrabBodyLifecycle.h ----

/*
 * Active grab lifecycle is captured as a full object-body snapshot because
 * multi-body props can contain Fallout-owned bodies with different original
 * motion states. HIGGS preserves the held relationship and restores engine-owned
 * state at release boundaries; ROCK needs the same ownership boundary while
 * keeping ordinary loose dynamic props dynamic. This policy layer keeps those
 * release decisions pure, so runtime code can apply them without re-deriving
 * intent from whichever body happened to become the primary constraint target.
 */

#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace frik::rock::active_grab_body_lifecycle
{
    inline constexpr std::uint32_t kInvalidBodyId = object_physics_body_set::INVALID_BODY_ID;
    inline constexpr std::uint16_t kDynamicMotionPropertiesId = 1;

    enum class MotionRole : std::uint8_t
    {
        Unknown,
        LooseDynamic,
        SystemOwnedNonDynamic,
    };

    enum class BodyRestorePolicy : std::uint8_t
    {
        ProtectComplexSystemOwned,
        RestoreAllChanged,
    };

    enum class BodyRestoreReason : std::uint8_t
    {
        None,
        FailedGrabSetup,
        Release,
    };

    struct BodyLifecycleRecord
    {
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint32_t motionIndex = 0;
        std::uint16_t motionPropertiesId = kDynamicMotionPropertiesId;
        std::uint32_t bodyFlags = 0;
        std::uint32_t filterInfo = 0;
        std::uintptr_t ownerKey = 0;
        physics_body_classifier::BodyMotionType originalMotionType = physics_body_classifier::BodyMotionType::Unknown;
        MotionRole motionRole = MotionRole::Unknown;
        bool acceptedBeforePrep = false;
        bool motionChangedByRock = false;
        bool filterChangedByRock = false;
        bool hasDampingSnapshot = false;
        bool hasInertiaSnapshot = false;
    };

    struct BodyRestorePlanEntry
    {
        BodyLifecycleRecord record{};
        bool restoreMotion = false;
        bool restoreFilter = false;
    };

    struct MotionRestoreCommand
    {
        std::uintptr_t ownerKey = 0;
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint16_t motionPropertiesId = kDynamicMotionPropertiesId;
        physics_body_classifier::BodyMotionType motionType = physics_body_classifier::BodyMotionType::Unknown;
        bool recursive = false;
        bool force = true;
        bool activate = false;
    };

    struct BodyRestorePlan
    {
        BodyRestoreReason reason = BodyRestoreReason::None;
        BodyRestorePolicy policy = BodyRestorePolicy::ProtectComplexSystemOwned;
        std::vector<BodyRestorePlanEntry> entries;
        std::uint32_t motionRestoreCount = 0;
        std::uint32_t filterRestoreCount = 0;

        bool shouldRestoreMotion(std::uint32_t bodyId) const
        {
            const auto it = std::find_if(entries.begin(), entries.end(), [&](const BodyRestorePlanEntry& entry) {
                return entry.record.bodyId == bodyId;
            });
            return it != entries.end() && it->restoreMotion;
        }
    };

    struct BodyLifecycleAudit
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t convertedCount = 0;
        std::uint32_t restoredMotionCount = 0;
        std::uint32_t restoredFilterCount = 0;
        std::uint32_t dampingSnapshotCount = 0;
        std::uint32_t inertiaSnapshotCount = 0;
        std::uint32_t primaryBodyId = kInvalidBodyId;
        BodyRestoreReason reason = BodyRestoreReason::None;
        BodyRestorePolicy policy = BodyRestorePolicy::ProtectComplexSystemOwned;
    };

    inline MotionRole motionRoleFromBodyMotionType(physics_body_classifier::BodyMotionType motionType)
    {
        return motionType == physics_body_classifier::BodyMotionType::Dynamic ? MotionRole::LooseDynamic : MotionRole::SystemOwnedNonDynamic;
    }

    inline BodyLifecycleRecord makeLifecycleRecord(const object_physics_body_set::ObjectPhysicsBodyRecord& record)
    {
        BodyLifecycleRecord result{};
        result.bodyId = record.bodyId;
        result.motionIndex = record.motionId;
        result.motionPropertiesId = record.motionPropertiesId;
        result.bodyFlags = record.bodyFlags;
        result.filterInfo = record.filterInfo;
        result.ownerKey = reinterpret_cast<std::uintptr_t>(record.owningNode);
        result.originalMotionType = record.motionType;
        result.motionRole = motionRoleFromBodyMotionType(record.motionType);
        result.acceptedBeforePrep = record.accepted;
        return result;
    }

    inline bool shouldCaptureBeforeActivePrep(physics_body_classifier::BodyRejectReason reason)
    {
        using physics_body_classifier::BodyRejectReason;
        switch (reason) {
        case BodyRejectReason::None:
        case BodyRejectReason::StaticMotion:
        case BodyRejectReason::KeyframedPassive:
        case BodyRejectReason::NotDynamicAfterActivePrep:
            return true;
        default:
            return false;
        }
    }

    class BodyLifecycleSnapshot
    {
    public:
        void clear() { _records.clear(); }

        void capture(const BodyLifecycleRecord& record)
        {
            if (record.bodyId == kInvalidBodyId) {
                return;
            }
            if (find(record.bodyId)) {
                return;
            }
            _records.push_back(record);
        }

        void captureAcceptedBeforePrep(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
        {
            for (const auto& record : bodySet.records) {
                if (record.accepted) {
                    capture(makeLifecycleRecord(record));
                }
            }
        }

        void captureBeforeActivePrep(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
        {
            for (const auto& record : bodySet.records) {
                if (shouldCaptureBeforeActivePrep(record.rejectReason)) {
                    capture(makeLifecycleRecord(record));
                }
            }
        }

        BodyLifecycleRecord* find(std::uint32_t bodyId)
        {
            const auto it = std::find_if(_records.begin(), _records.end(), [&](const BodyLifecycleRecord& record) {
                return record.bodyId == bodyId;
            });
            return it != _records.end() ? &*it : nullptr;
        }

        const BodyLifecycleRecord* find(std::uint32_t bodyId) const
        {
            const auto it = std::find_if(_records.begin(), _records.end(), [&](const BodyLifecycleRecord& record) {
                return record.bodyId == bodyId;
            });
            return it != _records.end() ? &*it : nullptr;
        }

        void markPreparedBody(std::uint32_t bodyId, bool motionChangedByRock)
        {
            if (auto* record = find(bodyId)) {
                record->motionChangedByRock = record->motionChangedByRock || motionChangedByRock;
            }
        }

        void markPreparedBodies(const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet)
        {
            for (const auto& record : preparedBodySet.records) {
                if (!record.accepted) {
                    continue;
                }
                if (auto* captured = find(record.bodyId)) {
                    captured->motionChangedByRock = true;
                }
            }
        }

        void markFilterChanged(std::uint32_t bodyId)
        {
            if (auto* record = find(bodyId)) {
                record->filterChangedByRock = true;
            }
        }

        BodyRestorePlan restorePlanForFailure() const { return makeRestorePlan(BodyRestoreReason::FailedGrabSetup, BodyRestorePolicy::RestoreAllChanged); }

        BodyRestorePlan restorePlanForRelease(BodyRestorePolicy policy) const { return makeRestorePlan(BodyRestoreReason::Release, policy); }

        std::vector<MotionRestoreCommand> makeMotionRestoreCommands(const BodyRestorePlan& plan) const
        {
            std::vector<MotionRestoreCommand> commands;
            commands.reserve(plan.motionRestoreCount);
            for (const auto& entry : plan.entries) {
                if (!entry.restoreMotion || entry.record.ownerKey == 0 || entry.record.bodyId == kInvalidBodyId) {
                    continue;
                }
                commands.push_back(MotionRestoreCommand{
                    .ownerKey = entry.record.ownerKey,
                    .bodyId = entry.record.bodyId,
                    .motionPropertiesId = entry.record.motionPropertiesId,
                    .motionType = entry.record.originalMotionType == physics_body_classifier::BodyMotionType::Unknown ?
                                       physics_body_classifier::motionTypeFromMotionPropertiesId(entry.record.motionPropertiesId) :
                                       entry.record.originalMotionType,
                    .recursive = false,
                    .force = true,
                    .activate = false,
                });
            }
            return commands;
        }

        std::size_t size() const { return _records.size(); }

        std::uint32_t convertedCount() const
        {
            return static_cast<std::uint32_t>(std::count_if(_records.begin(), _records.end(), [](const BodyLifecycleRecord& record) {
                return record.motionChangedByRock;
            }));
        }

        std::uint32_t dampingSnapshotCount() const
        {
            return static_cast<std::uint32_t>(std::count_if(_records.begin(), _records.end(), [](const BodyLifecycleRecord& record) {
                return record.hasDampingSnapshot;
            }));
        }

        std::uint32_t inertiaSnapshotCount() const
        {
            return static_cast<std::uint32_t>(std::count_if(_records.begin(), _records.end(), [](const BodyLifecycleRecord& record) {
                return record.hasInertiaSnapshot;
            }));
        }

        const std::vector<BodyLifecycleRecord>& records() const { return _records; }

    private:
        BodyRestorePlan makeRestorePlan(BodyRestoreReason reason, BodyRestorePolicy policy) const
        {
            BodyRestorePlan plan{};
            plan.reason = reason;
            plan.policy = policy;
            plan.entries.reserve(_records.size());

            for (const auto& record : _records) {
                BodyRestorePlanEntry entry{};
                entry.record = record;
                entry.restoreFilter = true;

                if (reason == BodyRestoreReason::FailedGrabSetup) {
                    entry.restoreMotion = record.motionChangedByRock;
                } else if (policy == BodyRestorePolicy::RestoreAllChanged) {
                    entry.restoreMotion = record.motionChangedByRock;
                } else {
                    entry.restoreMotion = record.motionChangedByRock && record.motionRole == MotionRole::SystemOwnedNonDynamic;
                }

                if (entry.restoreMotion) {
                    ++plan.motionRestoreCount;
                }
                if (entry.restoreFilter) {
                    ++plan.filterRestoreCount;
                }
                plan.entries.push_back(entry);
            }

            return plan;
        }

        std::vector<BodyLifecycleRecord> _records;
    };

    inline BodyLifecycleAudit makeLifecycleAudit(const BodyLifecycleSnapshot& snapshot, const BodyRestorePlan& plan, std::uint32_t primaryBodyId)
    {
        return BodyLifecycleAudit{
            .bodyCount = static_cast<std::uint32_t>(snapshot.size()),
            .convertedCount = snapshot.convertedCount(),
            .restoredMotionCount = plan.motionRestoreCount,
            .restoredFilterCount = plan.filterRestoreCount,
            .dampingSnapshotCount = snapshot.dampingSnapshotCount(),
            .inertiaSnapshotCount = snapshot.inertiaSnapshotCount(),
            .primaryBodyId = primaryBodyId,
            .reason = plan.reason,
            .policy = plan.policy,
        };
    }
}

// ---- CanonicalGrabFrame.h ----

/*
 * ROCK keeps one canonical object-local grab frame so constraint pivots, visible
 * hand ownership, mesh finger solving, and debug logging all read the same data.
 * The previous scattered fields worked, but every new grab-quality feature had to
 * remember the same hand/body/node transforms separately. This wrapper preserves
 * the existing transform convention while making the grab frame an explicit piece
 * of held-object state.
 */

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/grab/GrabContact.h"

#include <array>
#include <cstdint>
#include <vector>

namespace RE
{
    class NiAVObject;
}

namespace frik::rock
{
    inline constexpr std::size_t kMaxGrabContactPatchSamples = 5;

    struct GrabLocalTriangle
    {
        RE::NiPoint3 v0{};
        RE::NiPoint3 v1{};
        RE::NiPoint3 v2{};
    };

    struct CanonicalGrabFrame
    {
        RE::NiTransform rawHandSpace{};
        RE::NiTransform constraintHandSpace{};
        RE::NiTransform handBodyToRawHandAtGrab{};
        RE::NiTransform bodyLocal{};
        RE::NiTransform rootBodyLocal{};
        RE::NiTransform ownerBodyLocal{};
        RE::NiTransform liveHandWorldAtGrab{};
        RE::NiTransform handBodyWorldAtGrab{};
        RE::NiTransform objectNodeWorldAtGrab{};
        RE::NiTransform desiredObjectWorldAtGrab{};
        RE::NiPoint3 pivotAHandBodyLocalGame{};
        RE::NiPoint3 grabPivotWorldAtGrab{};
        RE::NiPoint3 surfacePointWorldAtGrab{};
        RE::NiPoint3 surfacePointLocal{};
        RE::NiPoint3 surfaceHitLocal{};
        RE::NiPoint3 surfaceNormalLocal{};
        RE::NiPoint3 oppositionThumbWorldAtGrab{};
        RE::NiPoint3 oppositionOpposingWorldAtGrab{};
        RE::NiPoint3 oppositionThumbObjectLocal{};
        RE::NiPoint3 oppositionOpposingObjectLocal{};
        RE::NiPoint3 multiFingerGripCenterWorldAtGrab{};
        RE::NiPoint3 multiFingerHandCenterWorldAtGrab{};
        RE::NiPoint3 multiFingerAverageNormalWorldAtGrab{};
        RE::NiPoint3 surfacePointBodyLocalGame{};
        RE::NiPoint3 pivotBBodyLocalGame{};
        grab_surface_frame_math::GrabSurfaceFrame<RE::NiPoint3> surfaceFrameLocal{};
        grab_surface_frame_math::GrabOrientationMode orientationModeUsed{ grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation };
        grab_surface_frame_math::GrabSurfaceAlignmentDecision surfaceAlignmentDecision{ grab_surface_frame_math::GrabSurfaceAlignmentDecision::RejectedMode };
        std::array<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>, kMaxGrabContactPatchSamples> contactPatchSamples{};
        std::uint32_t surfaceTriangleIndex = 0xFFFF'FFFF;
        std::uint32_t surfaceShapeKey = 0xFFFF'FFFF;
        std::uint32_t surfaceShapeCollisionFilterInfo = 0;
        std::uint32_t contactPatchSampleCount = 0;
        std::uint32_t multiFingerContactGroupCount = 0;
        float surfaceHitFraction = 1.0f;
        float surfaceFrameConfidence = 0.0f;
        float surfacePivotToSurfaceDistanceGameUnits = 0.0f;
        float surfaceSelectionToMeshDistanceGameUnits = 0.0f;
        float contactPatchMeshSnapDeltaGameUnits = 0.0f;
        float multiFingerContactSpreadGameUnits = 0.0f;
        float handScaleAtGrab = 1.0f;
        const char* bodyResolutionReason = "none";
        const char* surfaceFrameFallbackReason = "none";
        const char* oppositionFrameReason = "none";
        const char* multiFingerContactReason = "none";
        const char* visualAuthorityContactReason = "none";
        /*
         * HIGGS only fades dynamic-grab motor authority when the object must be
         * synced from an initial/custom alignment. ROCK stores that decision in
         * the canonical frame so the constraint, visual hand, and diagnostics
         * agree about whether startup softness is part of this grab or stale
         * legacy behavior.
         */
        const char* motorFadeReason = "none";
        std::vector<GrabLocalTriangle> localMeshTriangles;
        RE::NiAVObject* heldNode = nullptr;
        bool hasMeshPoseData = false;
        bool hasSurfaceHit = false;
        bool hasSurfaceFrame = false;
        bool hasSurfaceShapeKey = false;
        bool hasFrozenPivotB = false;
        bool hasContactPatch = false;
        bool hasMultiFingerContactPatch = false;
        bool hasTelemetryCapture = false;
        bool hasOppositionFrame = false;
        bool visualAuthorityContactValid = false;
        bool fadeInGrabConstraint = false;

        void clear()
        {
            rawHandSpace = RE::NiTransform();
            constraintHandSpace = RE::NiTransform();
            handBodyToRawHandAtGrab = RE::NiTransform();
            bodyLocal = RE::NiTransform();
            rootBodyLocal = RE::NiTransform();
            ownerBodyLocal = RE::NiTransform();
            liveHandWorldAtGrab = RE::NiTransform();
            handBodyWorldAtGrab = RE::NiTransform();
            objectNodeWorldAtGrab = RE::NiTransform();
            desiredObjectWorldAtGrab = RE::NiTransform();
            pivotAHandBodyLocalGame = {};
            grabPivotWorldAtGrab = {};
            surfacePointWorldAtGrab = {};
            surfacePointLocal = {};
            surfaceHitLocal = {};
            surfaceNormalLocal = {};
            oppositionThumbWorldAtGrab = {};
            oppositionOpposingWorldAtGrab = {};
            oppositionThumbObjectLocal = {};
            oppositionOpposingObjectLocal = {};
            multiFingerGripCenterWorldAtGrab = {};
            multiFingerHandCenterWorldAtGrab = {};
            multiFingerAverageNormalWorldAtGrab = {};
            surfacePointBodyLocalGame = {};
            pivotBBodyLocalGame = {};
            surfaceFrameLocal = {};
            orientationModeUsed = grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation;
            surfaceAlignmentDecision = grab_surface_frame_math::GrabSurfaceAlignmentDecision::RejectedMode;
            contactPatchSamples = {};
            surfaceTriangleIndex = 0xFFFF'FFFF;
            surfaceShapeKey = 0xFFFF'FFFF;
            surfaceShapeCollisionFilterInfo = 0;
            contactPatchSampleCount = 0;
            multiFingerContactGroupCount = 0;
            surfaceHitFraction = 1.0f;
            surfaceFrameConfidence = 0.0f;
            surfacePivotToSurfaceDistanceGameUnits = 0.0f;
            surfaceSelectionToMeshDistanceGameUnits = 0.0f;
            contactPatchMeshSnapDeltaGameUnits = 0.0f;
            multiFingerContactSpreadGameUnits = 0.0f;
            handScaleAtGrab = 1.0f;
            bodyResolutionReason = "none";
            surfaceFrameFallbackReason = "none";
            oppositionFrameReason = "none";
            multiFingerContactReason = "none";
            visualAuthorityContactReason = "none";
            motorFadeReason = "none";
            localMeshTriangles.clear();
            heldNode = nullptr;
            hasMeshPoseData = false;
            hasSurfaceHit = false;
            hasSurfaceFrame = false;
            hasSurfaceShapeKey = false;
            hasFrozenPivotB = false;
            hasContactPatch = false;
            hasMultiFingerContactPatch = false;
            hasTelemetryCapture = false;
            hasOppositionFrame = false;
            visualAuthorityContactValid = false;
            fadeInGrabConstraint = false;
        }

        bool isValid() const { return heldNode != nullptr || hasMeshPoseData || !localMeshTriangles.empty(); }
    };
}

// ---- GrabInteractionPolicy.h ----

#include <cstdint>
#include <string_view>

namespace frik::rock::grab_interaction_policy
{
    /*
     * HIGGS treats a far-selected object as an active interaction candidate and
     * transitions it into pull/grab handling from that state. ROCK's object-tree
     * prep already owns the FO4VR-specific recursive dynamic conversion, so this
     * policy only decides whether a selected object is allowed to enter that
     * active path. The important invariant is that configured far selection must
     * not be made inert by a second hard-coded distance gate.
     */
    inline bool canAttemptSelectedObjectGrab(bool isFarSelection, float selectionDistance, float configuredFarRange)
    {
        if (!isFarSelection) {
            return true;
        }

        return configuredFarRange > 0.0f && selectionDistance >= 0.0f && selectionDistance <= configuredFarRange;
    }

    /*
     * Selection and activation must share one gameplay guard. ROCK can select via
     * either close collision hits or far swept-sphere hits, but both paths feed the
     * same dynamic-object preparation. Keeping these exclusions here prevents far
     * pull from bypassing the close-grab checks for doors, furniture, fixed
     * activators, and live actors.
     */
    inline bool shouldBlockSelectedObjectInteraction(std::string_view formType, bool isLiveNpc, bool hasMotionProps, std::uint16_t motionPropsId)
    {
        if (formType == "DOOR" || formType == "CONT" || formType == "TERM" || formType == "FURN") {
            return true;
        }

        if (formType == "ACTI" && hasMotionProps) {
            const auto motionPreset = static_cast<std::uint8_t>(motionPropsId & 0xFF);
            if (motionPreset == 0 || motionPreset == 2) {
                return true;
            }
        }

        return formType == "NPC_" && isLiveNpc;
    }
}

// ---- GrabFrameMath.h ----

/*
 * ROCK's calibrated grab pivot is authored in semantic handspace, but the live
 * grab has two different consumers: the rendered FRIK hand, and the keyframed
 * physics hand body. HIGGS keeps one coherent palm-to-object relation; ROCK needs
 * the same coherence while preserving its separate visual and collision frames.
 * These helpers keep that split explicit so pivot A, pivot B, reverse visual
 * alignment, and debug measurements cannot silently drift onto different frames.
 */

#include "physics-interaction/TransformMath.h"

#include <utility>

namespace frik::rock::grab_frame_math
{
    template <class Transform>
    struct SplitGrabFrame
    {
        using Vector = decltype(std::declval<Transform>().translate);

        Transform shiftedObjectWorld{};
        Transform rawHandSpace{};
        Transform constraintHandSpace{};
        Transform handBodyToRawHandAtGrab{};
        Transform desiredBodyHandBodySpace{};
        Vector pivotAHandBodyLocal{};
        Vector pivotBBodyLocal{};
    };

    template <class Transform, class Vector>
    inline Transform shiftObjectToAlignSurfaceWithPivot(Transform objectWorld, const Vector& grabPivotWorld, const Vector& surfacePointWorld)
    {
        objectWorld.translate.x += grabPivotWorld.x - surfacePointWorld.x;
        objectWorld.translate.y += grabPivotWorld.y - surfacePointWorld.y;
        objectWorld.translate.z += grabPivotWorld.z - surfacePointWorld.z;
        return objectWorld;
    }

    template <class Transform>
    inline Transform objectInFrameSpace(const Transform& frameWorld, const Transform& objectWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(frameWorld), objectWorld);
    }

    template <class Transform>
    inline Transform desiredBodyInHandBodySpace(const Transform& constraintHandSpace, const Transform& bodyLocal)
    {
        return transform_math::composeTransforms(constraintHandSpace, bodyLocal);
    }

    template <class Transform, class Vector>
    inline Vector computePivotAHandBodyLocal(const Transform& handBodyWorld, const Vector& grabPivotWorld)
    {
        return transform_math::worldPointToLocal(handBodyWorld, grabPivotWorld);
    }

    template <class Transform, class Vector>
    inline Vector computePivotBBodyLocal(const Transform& desiredBodyTransformHandBodySpace, const Vector& pivotAHandBodyLocal)
    {
        return transform_math::localPointToWorld(transform_math::invertTransform(desiredBodyTransformHandBodySpace), pivotAHandBodyLocal);
    }

    template <class Transform>
    inline Transform computeVisualHandFromHeldNode(const Transform& heldNodeWorld, const Transform& rawHandSpace)
    {
        return transform_math::composeTransforms(heldNodeWorld, transform_math::invertTransform(rawHandSpace));
    }

    template <class Transform, class Vector>
    inline SplitGrabFrame<Transform> buildSplitGrabFrame(const Transform& rawHandWorld,
        const Transform& handBodyWorld,
        const Transform& objectNodeWorld,
        const Transform& bodyLocal,
        const Vector& grabPivotWorld,
        const Vector& surfacePointWorld)
    {
        SplitGrabFrame<Transform> result{};
        result.shiftedObjectWorld = shiftObjectToAlignSurfaceWithPivot(objectNodeWorld, grabPivotWorld, surfacePointWorld);
        result.rawHandSpace = objectInFrameSpace(rawHandWorld, result.shiftedObjectWorld);
        result.constraintHandSpace = objectInFrameSpace(handBodyWorld, result.shiftedObjectWorld);
        result.handBodyToRawHandAtGrab = objectInFrameSpace(handBodyWorld, rawHandWorld);
        result.desiredBodyHandBodySpace = desiredBodyInHandBodySpace(result.constraintHandSpace, bodyLocal);
        result.pivotAHandBodyLocal = computePivotAHandBodyLocal(handBodyWorld, grabPivotWorld);
        result.pivotBBodyLocal = computePivotBBodyLocal(result.desiredBodyHandBodySpace, result.pivotAHandBodyLocal);
        return result;
    }

    template <class Transform, class Vector>
    inline SplitGrabFrame<Transform> buildSplitGrabFrameFromDesiredObject(const Transform& rawHandWorld,
        const Transform& handBodyWorld,
        const Transform& desiredObjectWorld,
        const Transform& bodyLocal,
        const Vector& grabPivotWorld)
    {
        SplitGrabFrame<Transform> result{};
        result.shiftedObjectWorld = desiredObjectWorld;
        result.rawHandSpace = objectInFrameSpace(rawHandWorld, desiredObjectWorld);
        result.constraintHandSpace = objectInFrameSpace(handBodyWorld, desiredObjectWorld);
        result.handBodyToRawHandAtGrab = objectInFrameSpace(handBodyWorld, rawHandWorld);
        result.desiredBodyHandBodySpace = desiredBodyInHandBodySpace(result.constraintHandSpace, bodyLocal);
        result.pivotAHandBodyLocal = computePivotAHandBodyLocal(handBodyWorld, grabPivotWorld);
        result.pivotBBodyLocal = computePivotBBodyLocal(result.desiredBodyHandBodySpace, result.pivotAHandBodyLocal);
        return result;
    }
}

// ---- PullMotionMath.h ----

#include <algorithm>
#include <cmath>

namespace frik::rock::pull_motion_math
{
    template <class Vec3>
    struct PullMotionInput
    {
        Vec3 handHavok{};
        Vec3 objectPointHavok{};
        Vec3 previousTargetHavok{};
        float elapsedSeconds = 0.0f;
        float durationSeconds = 0.0f;
        float applyVelocitySeconds = 0.2f;
        float trackHandSeconds = 0.1f;
        float destinationOffsetHavok = 0.01f;
        float maxVelocityHavok = 10.0f;
        bool hasPreviousTarget = false;
    };

    template <class Vec3>
    struct PullMotionResult
    {
        Vec3 targetHavok{};
        Vec3 velocityHavok{};
        float durationRemainingSeconds = 0.0f;
        bool applyVelocity = false;
        bool refreshTarget = false;
        bool expired = false;
    };

    inline float computePullDurationSeconds(float distanceHavok, float a, float b, float c)
    {
        const float clampedDistance = (std::max)(0.0f, distanceHavok);
        return (std::max)(0.001f, a + b * std::exp(-c * clampedDistance));
    }

    inline float angularVelocityKeepForDamping(float damping, float deltaTime)
    {
        if (!std::isfinite(damping) || damping <= 0.0f) {
            return 1.0f;
        }
        const float dt = (std::isfinite(deltaTime) && deltaTime > 0.0f) ? deltaTime : (1.0f / 90.0f);
        return 1.0f / (1.0f + damping * dt);
    }

    template <class Vec3>
    float vectorLength(const Vec3& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    template <class Vec3>
    Vec3 clampLength(Vec3 value, float maxLength)
    {
        if (maxLength <= 0.0f) {
            return value;
        }

        const float length = vectorLength(value);
        if (length <= maxLength || length <= 0.0001f) {
            return value;
        }

        const float scale = maxLength / length;
        value.x *= scale;
        value.y *= scale;
        value.z *= scale;
        return value;
    }

    template <class Vec3>
    PullMotionResult<Vec3> computePullMotion(const PullMotionInput<Vec3>& input)
    {
        PullMotionResult<Vec3> result{};
        result.refreshTarget = !input.hasPreviousTarget || input.elapsedSeconds <= input.trackHandSeconds;
        result.targetHavok = result.refreshTarget ? input.handHavok : input.previousTargetHavok;
        if (result.refreshTarget) {
            result.targetHavok.z += input.destinationOffsetHavok;
        }

        if (input.elapsedSeconds > input.applyVelocitySeconds) {
            result.expired = true;
            return result;
        }

        result.durationRemainingSeconds = input.durationSeconds - input.elapsedSeconds;
        if (result.durationRemainingSeconds <= 0.001f) {
            result.expired = true;
            return result;
        }

        Vec3 horizontalDelta{
            result.targetHavok.x - input.objectPointHavok.x,
            result.targetHavok.y - input.objectPointHavok.y,
            0.0f,
        };

        const float verticalDelta = result.targetHavok.z - input.objectPointHavok.z;
        result.velocityHavok.x = horizontalDelta.x / result.durationRemainingSeconds;
        result.velocityHavok.y = horizontalDelta.y / result.durationRemainingSeconds;
        result.velocityHavok.z = 0.5f * 9.81f * result.durationRemainingSeconds + verticalDelta / result.durationRemainingSeconds;
        result.velocityHavok = clampLength(result.velocityHavok, input.maxVelocityHavok);
        result.applyVelocity = true;
        return result;
    }
}
