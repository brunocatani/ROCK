#pragma once

/*
 * Grab core policy is grouped here to keep object preparation, lifecycle, canonical frames, frame math, interaction decisions, and pull motion together.
 */


// ---- ActiveObjectPrepPolicy.h ----

#include <cstdint>

namespace rock::active_object_prep_policy
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
 * motion states. ROCK preserves the held relationship and restores engine-owned
 * state at release boundaries while keeping ordinary loose dynamic props
 * dynamic. This policy layer keeps those release decisions pure, so runtime code
 * can apply them without re-deriving intent from whichever body happened to
 * become the primary constraint target.
 */

#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rock::active_grab_body_lifecycle
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

    inline constexpr BodyRestorePolicy releaseRestorePolicyForTargetKind(grab_target::Kind targetKind) noexcept
    {
        /*
         * Whole actor ragdolls are Fallout-owned physics systems even when the
         * selected body is already dynamic. Return every touched actor body and
         * filter to its captured state before the engine later detaches the cell.
         */
        return targetKind == grab_target::Kind::DeadActorBody ? BodyRestorePolicy::RestoreAllChanged :
                                                               BodyRestorePolicy::ProtectComplexSystemOwned;
    }

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
        bool capturedAfterPrep = false;
        bool originalStateKnown = true;
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
        std::uint32_t latePreparedBodyCount = 0;
        bool incompleteNativeScan = false;
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
        void clear()
        {
            _records.clear();
            _latePreparedBodyCount = 0;
            _incompleteNativeScan = false;
        }

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
            observeScanDiagnostics(bodySet);
            for (const auto& record : bodySet.records) {
                if (record.accepted) {
                    capture(makeLifecycleRecord(record));
                }
            }
        }

        void captureBeforeActivePrep(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
        {
            observeScanDiagnostics(bodySet);
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
            observeScanDiagnostics(preparedBodySet);
            for (const auto& record : preparedBodySet.records) {
                if (!record.accepted) {
                    continue;
                }
                if (auto* captured = find(record.bodyId)) {
                    captured->motionChangedByRock = true;
                    continue;
                }

                auto lateRecord = makeLifecycleRecord(record);
                lateRecord.acceptedBeforePrep = false;
                lateRecord.capturedAfterPrep = true;
                lateRecord.originalStateKnown = false;
                lateRecord.motionChangedByRock = true;
                capture(lateRecord);
                ++_latePreparedBodyCount;
                _incompleteNativeScan = true;
            }
        }

        void markIncompleteNativeScan()
        {
            _incompleteNativeScan = true;
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

        std::uint32_t latePreparedBodyCount() const { return _latePreparedBodyCount; }

        bool hasIncompleteNativeScan() const { return _incompleteNativeScan; }

        const std::vector<BodyLifecycleRecord>& records() const { return _records; }

    private:
        void observeScanDiagnostics(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
        {
            if (bodySet.diagnostics.scanFailures > 0 ||
                bodySet.diagnostics.invalidPhysicsSystems > 0 ||
                bodySet.diagnostics.depthLimitSkips > 0) {
                _incompleteNativeScan = true;
            }
        }

        BodyRestorePlan makeRestorePlan(BodyRestoreReason reason, BodyRestorePolicy policy) const
        {
            /*
             * Release restore must keep one coherent body contract. If ROCK keeps
             * a loose object dynamic after a physical drop, restoring its pre-grab
             * collision filter can leave it dynamic but non-colliding. Failure and
             * explicit restore-all paths still put every captured filter back;
             * protected release restores filters only for system-owned non-dynamic
             * bodies whose motion ownership is also returned to the engine.
             */
            BodyRestorePlan plan{};
            plan.reason = reason;
            plan.policy = policy;
            plan.entries.reserve(_records.size());

            for (const auto& record : _records) {
                BodyRestorePlanEntry entry{};
                entry.record = record;

                if (reason == BodyRestoreReason::FailedGrabSetup) {
                    entry.restoreMotion = record.originalStateKnown && record.motionChangedByRock;
                } else if (policy == BodyRestorePolicy::RestoreAllChanged) {
                    entry.restoreMotion = record.originalStateKnown && record.motionChangedByRock;
                } else {
                    entry.restoreMotion =
                        record.originalStateKnown && record.motionChangedByRock && record.motionRole == MotionRole::SystemOwnedNonDynamic;
                }

                if (reason == BodyRestoreReason::FailedGrabSetup || policy == BodyRestorePolicy::RestoreAllChanged) {
                    entry.restoreFilter = record.originalStateKnown;
                } else {
                    entry.restoreFilter = record.originalStateKnown && record.motionRole == MotionRole::SystemOwnedNonDynamic;
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
        std::uint32_t _latePreparedBodyCount = 0;
        bool _incompleteNativeScan = false;
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
            .latePreparedBodyCount = snapshot.latePreparedBodyCount(),
            .incompleteNativeScan = snapshot.hasIncompleteNativeScan(),
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

namespace rock
{
    inline constexpr std::size_t kMaxGrabContactPatchSamples = grab_contact_patch_math::kContactPatchProbePatternSampleCount;

    struct GrabLocalTriangle
    {
        RE::NiPoint3 v0{};
        RE::NiPoint3 v1{};
        RE::NiPoint3 v2{};
    };

    struct ImmutableGrabCaptureTelemetry
    {
        /*
         * Capture telemetry is intentionally separate from the live grab frame.
         * The live frame may be rewritten by acquisition/reacquire logic because
         * it feeds the active constraint, but the capture record is the original
         * evidence trail used by debug overlays to answer "what did this grab
         * select?" without being contaminated by later solver authority changes.
         */
        RE::NiTransform liveHandWorld{};
        RE::NiTransform handBodyWorld{};
        RE::NiTransform objectNodeWorld{};
        RE::NiTransform bodyWorld{};
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiTransform bodyLocal{};
        RE::NiPoint3 pivotAHandBodyLocalGame{};
        RE::NiPoint3 grabPivotWorld{};
        RE::NiPoint3 gripPointWorld{};
        RE::NiPoint3 gripPointLocal{};
        RE::NiPoint3 gripEvidenceLocal{};
        RE::NiPoint3 gripNormalLocal{};
        RE::NiTransform gripSourceNodeWorld{};
        RE::NiPoint3 gripPointSourceNodeLocal{};
        RE::NiPoint3 gripNormalSourceNodeLocal{};
        RE::NiPoint3 gripPointBodyLocalGame{};
        RE::NiPoint3 pivotBBodyLocalGame{};
        RE::NiPoint3 pivotBConstraintLocalGame{};
        std::uint32_t sourceBodyId = 0x7FFF'FFFF;
        std::uint32_t gripEvidenceTriangleIndex = 0xFFFF'FFFF;
        std::uint32_t gripEvidenceShapeKey = 0xFFFF'FFFF;
        float pocketToGripDistanceGameUnits = 0.0f;
        float selectionToGripEvidenceDistanceGameUnits = 0.0f;
        float longObjectLeverGameUnits = 0.0f;
        float pivotAuthorityPositionConfidence = 0.0f;
        const char* activeGrabPointMode = "none";
        const char* pivotAuthoritySource = "none";
        const char* palmSeatPointMode = "none";
        const char* fingerEvidencePointMode = "none";
        RE::NiAVObject* gripSourceNode = nullptr;
        bool valid = false;
        bool hasGripPoint = false;
        bool hasGripSourceNodePoint = false;
        bool hasGripSourceNodeNormal = false;
        bool hasFrozenPivotB = false;
        bool hasMeshPoseData = false;
        bool positionOnlyPivot = false;
        bool normalTrusted = false;

        void clear()
        {
            *this = ImmutableGrabCaptureTelemetry{};
        }
    };

    enum class GrabSeatMode : std::uint8_t
    {
        None,
        PinchPocket,
        SupportGroup,
    };

    inline const char* grabSeatModeName(GrabSeatMode mode)
    {
        switch (mode) {
        case GrabSeatMode::PinchPocket:
            return "pinchPocket";
        case GrabSeatMode::SupportGroup:
            return "supportGroup";
        default:
            return "none";
        }
    }

    struct CanonicalGrabFrame
    {
        RE::NiTransform rawHandSpace{};
        RE::NiTransform proxyAuthorityHandSpace{};
        RE::NiTransform proxyAuthorityBodyHandSpace{};
        RE::NiTransform handBodyToRawHandAtGrab{};
        RE::NiTransform bodyLocal{};
        RE::NiTransform rootBodyLocal{};
        RE::NiTransform ownerBodyLocal{};
        RE::NiTransform liveHandWorldAtGrab{};
        RE::NiTransform handBodyWorldAtGrab{};
        RE::NiTransform objectNodeWorldAtGrab{};
        RE::NiTransform bodyWorldAtGrab{};
        RE::NiTransform desiredObjectWorldAtGrab{};
        RE::NiTransform desiredBodyWorldAtGrab{};
        RE::NiPoint3 pivotAHandBodyLocalGame{};
        RE::NiPoint3 grabPivotWorldAtGrab{};
        RE::NiPoint3 gripPointWorldAtGrab{};
        RE::NiPoint3 palmSeatPointWorldAtGrab{};
        RE::NiPoint3 pinchPocketWorldAtGrab{};
        RE::NiPoint3 pinchAxisWorldAtGrab{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 fingerEvidencePointWorldAtGrab{};
        RE::NiPoint3 gripPointLocal{};
        RE::NiPoint3 gripEvidenceLocal{};
        RE::NiPoint3 gripNormalLocal{};
        RE::NiTransform gripSourceNodeWorldAtGrab{};
        RE::NiPoint3 gripPointSourceNodeLocal{};
        RE::NiPoint3 gripNormalSourceNodeLocal{};
        RE::NiPoint3 multiFingerGripCenterWorldAtGrab{};
        RE::NiPoint3 multiFingerHandCenterWorldAtGrab{};
        RE::NiPoint3 multiFingerAverageNormalWorldAtGrab{};
        std::array<RE::NiPoint3, 5> fingerPoseTargetLocal{};
        std::array<RE::NiPoint3, 5> fingerPoseTargetNormalLocal{};
        std::array<std::uint8_t, 5> fingerPoseTargetValid{};
        std::array<std::uint8_t, 5> fingerPoseTargetNormalValid{};
        RE::NiPoint3 gripPointBodyLocalGame{};
        RE::NiPoint3 pivotBBodyLocalGame{};
        RE::NiPoint3 pivotBConstraintLocalGame{};
        RE::NiPoint3 supportFrameNormalBodyLocal{};
        RE::NiPoint3 supportFrameAxisBodyLocal{};
        RE::NiPoint3 supportFrameBinormalBodyLocal{};
        std::array<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>, kMaxGrabContactPatchSamples> contactPatchSamples{};
        std::uint32_t gripEvidenceTriangleIndex = 0xFFFF'FFFF;
        std::uint32_t gripEvidenceShapeKey = 0xFFFF'FFFF;
        std::uint32_t gripEvidenceShapeCollisionFilterInfo = 0;
        std::uint32_t contactPatchSampleCount = 0;
        std::uint32_t multiFingerContactGroupCount = 0;
        std::uint32_t fingerPoseTargetCount = 0;
        float gripEvidenceHitFraction = 1.0f;
        float pocketToGripDistanceGameUnits = 0.0f;
        float selectionToGripEvidenceDistanceGameUnits = 0.0f;
        float contactPatchMeshSnapDeltaGameUnits = 0.0f;
        float multiFingerContactSpreadGameUnits = 0.0f;
        float longObjectLeverGameUnits = 0.0f;
        float gripSupportConfidence = 0.0f;
        float gripSupportSpanGameUnits = 0.0f;
        float gripSupportPivotShiftGameUnits = 0.0f;
        float pivotAuthorityPositionConfidence = 0.0f;
        float handScaleAtGrab = 1.0f;
        float lastSeatedPivotReacquireLocalDeltaGameUnits = 0.0f;
        std::uint64_t traceId = 0;
        std::uint64_t traceTargetWriteSequence = 0;
        std::uint32_t seatedPivotReacquireCount = 0;
        const char* bodyResolutionReason = "none";
        const char* multiFingerContactReason = "none";
        const char* activeGrabPointMode = "none";
        const char* pivotAuthoritySource = "none";
        const char* palmSeatPointMode = "none";
        const char* fingerEvidencePointMode = "none";
        const char* fingerPoseAimReason = "none";
        const char* gripSupportReason = "none";
        const char* lastSeatedPivotReacquireReason = "none";
        const char* lastSeatedPivotReacquirePhase = "none";
        GrabSeatMode seatMode = GrabSeatMode::None;
        grab_support_model_math::GripSupportKind gripSupportKind = grab_support_model_math::GripSupportKind::None;
        /*
         * ROCK only fades the dynamic grab when the object must be synced from
         * an initial/custom alignment. The canonical frame stores that decision
         * so the native spring and diagnostics agree about whether startup
         * softness is part of this grab.
         */
        const char* motorFadeReason = "none";
        ImmutableGrabCaptureTelemetry captureTelemetry{};
        std::vector<GrabLocalTriangle> localMeshTriangles;
        RE::NiAVObject* heldNode = nullptr;
        RE::NiAVObject* gripSourceNode = nullptr;
        bool hasMeshPoseData = false;
        bool hasGripPoint = false;
        bool hasGripSourceNodePoint = false;
        bool hasGripSourceNodeNormal = false;
        bool hasGripEvidenceShapeKey = false;
        bool hasFrozenPivotB = false;
        bool hasContactPatch = false;
        bool hasContactPatchEvidence = false;
        bool hasMultiFingerContactPatch = false;
        bool hasPalmSeatPoint = false;
        bool hasPinchPocket = false;
        bool hasFingerEvidencePoint = false;
        bool activeGrabPointUsesMultiFingerEvidence = false;
        bool hasGripSupportModel = false;
        bool hasSupportFrameNormal = false;
        bool hasSupportFrameAxis = false;
        bool hasSupportFrameBinormal = false;
        bool gripSupportAuthoredPivot = false;
        bool pivotAuthorityPositionOnly = false;
        bool pivotAuthorityNormalTrusted = false;
        bool requiresSettledVisualHandRelation = false;
        bool hasTelemetryCapture = false;
        bool hasSeatedPivotReacquire = false;
        bool fingerPoseAimValid = false;
        bool fadeInGrabConstraint = false;

        void freezeCaptureTelemetry(std::uint32_t sourceBodyId)
        {
            captureTelemetry.liveHandWorld = liveHandWorldAtGrab;
            captureTelemetry.handBodyWorld = handBodyWorldAtGrab;
            captureTelemetry.objectNodeWorld = objectNodeWorldAtGrab;
            captureTelemetry.bodyWorld = bodyWorldAtGrab;
            captureTelemetry.desiredObjectWorld = desiredObjectWorldAtGrab;
            captureTelemetry.desiredBodyWorld = desiredBodyWorldAtGrab;
            captureTelemetry.bodyLocal = bodyLocal;
            captureTelemetry.pivotAHandBodyLocalGame = pivotAHandBodyLocalGame;
            captureTelemetry.grabPivotWorld = grabPivotWorldAtGrab;
            captureTelemetry.gripPointWorld = gripPointWorldAtGrab;
            captureTelemetry.gripPointLocal = gripPointLocal;
            captureTelemetry.gripEvidenceLocal = gripEvidenceLocal;
            captureTelemetry.gripNormalLocal = gripNormalLocal;
            captureTelemetry.gripSourceNodeWorld = gripSourceNodeWorldAtGrab;
            captureTelemetry.gripPointSourceNodeLocal = gripPointSourceNodeLocal;
            captureTelemetry.gripNormalSourceNodeLocal = gripNormalSourceNodeLocal;
            captureTelemetry.gripSourceNode = gripSourceNode;
            captureTelemetry.gripPointBodyLocalGame = gripPointBodyLocalGame;
            captureTelemetry.pivotBBodyLocalGame = pivotBBodyLocalGame;
            captureTelemetry.pivotBConstraintLocalGame = pivotBConstraintLocalGame;
            captureTelemetry.sourceBodyId = sourceBodyId;
            captureTelemetry.gripEvidenceTriangleIndex = gripEvidenceTriangleIndex;
            captureTelemetry.gripEvidenceShapeKey = gripEvidenceShapeKey;
            captureTelemetry.pocketToGripDistanceGameUnits = pocketToGripDistanceGameUnits;
            captureTelemetry.selectionToGripEvidenceDistanceGameUnits = selectionToGripEvidenceDistanceGameUnits;
            captureTelemetry.longObjectLeverGameUnits = longObjectLeverGameUnits;
            captureTelemetry.pivotAuthorityPositionConfidence = pivotAuthorityPositionConfidence;
            captureTelemetry.activeGrabPointMode = activeGrabPointMode;
            captureTelemetry.pivotAuthoritySource = pivotAuthoritySource;
            captureTelemetry.palmSeatPointMode = palmSeatPointMode;
            captureTelemetry.fingerEvidencePointMode = fingerEvidencePointMode;
            captureTelemetry.hasGripPoint = hasGripPoint;
            captureTelemetry.hasGripSourceNodePoint = hasGripSourceNodePoint;
            captureTelemetry.hasGripSourceNodeNormal = hasGripSourceNodeNormal;
            captureTelemetry.hasFrozenPivotB = hasFrozenPivotB;
            captureTelemetry.hasMeshPoseData = hasMeshPoseData;
            captureTelemetry.positionOnlyPivot = pivotAuthorityPositionOnly;
            captureTelemetry.normalTrusted = pivotAuthorityNormalTrusted;
            captureTelemetry.valid = hasTelemetryCapture;
        }

        void clear()
        {
            rawHandSpace = RE::NiTransform();
            proxyAuthorityHandSpace = RE::NiTransform();
            proxyAuthorityBodyHandSpace = RE::NiTransform();
            handBodyToRawHandAtGrab = RE::NiTransform();
            bodyLocal = RE::NiTransform();
            rootBodyLocal = RE::NiTransform();
            ownerBodyLocal = RE::NiTransform();
            liveHandWorldAtGrab = RE::NiTransform();
            handBodyWorldAtGrab = RE::NiTransform();
            objectNodeWorldAtGrab = RE::NiTransform();
            bodyWorldAtGrab = RE::NiTransform();
            desiredObjectWorldAtGrab = RE::NiTransform();
            desiredBodyWorldAtGrab = RE::NiTransform();
            pivotAHandBodyLocalGame = {};
            grabPivotWorldAtGrab = {};
            gripPointWorldAtGrab = {};
            palmSeatPointWorldAtGrab = {};
            pinchPocketWorldAtGrab = {};
            pinchAxisWorldAtGrab = { 1.0f, 0.0f, 0.0f };
            fingerEvidencePointWorldAtGrab = {};
            gripPointLocal = {};
            gripEvidenceLocal = {};
            gripNormalLocal = {};
            gripSourceNodeWorldAtGrab = RE::NiTransform();
            gripPointSourceNodeLocal = {};
            gripNormalSourceNodeLocal = {};
            multiFingerGripCenterWorldAtGrab = {};
            multiFingerHandCenterWorldAtGrab = {};
            multiFingerAverageNormalWorldAtGrab = {};
            fingerPoseTargetLocal = {};
            fingerPoseTargetNormalLocal = {};
            fingerPoseTargetValid = {};
            fingerPoseTargetNormalValid = {};
            gripPointBodyLocalGame = {};
            pivotBBodyLocalGame = {};
            pivotBConstraintLocalGame = {};
            supportFrameNormalBodyLocal = {};
            supportFrameAxisBodyLocal = {};
            supportFrameBinormalBodyLocal = {};
            contactPatchSamples = {};
            gripEvidenceTriangleIndex = 0xFFFF'FFFF;
            gripEvidenceShapeKey = 0xFFFF'FFFF;
            gripEvidenceShapeCollisionFilterInfo = 0;
            contactPatchSampleCount = 0;
            multiFingerContactGroupCount = 0;
            fingerPoseTargetCount = 0;
            gripEvidenceHitFraction = 1.0f;
            pocketToGripDistanceGameUnits = 0.0f;
            selectionToGripEvidenceDistanceGameUnits = 0.0f;
            contactPatchMeshSnapDeltaGameUnits = 0.0f;
            multiFingerContactSpreadGameUnits = 0.0f;
            longObjectLeverGameUnits = 0.0f;
            gripSupportConfidence = 0.0f;
            gripSupportSpanGameUnits = 0.0f;
            gripSupportPivotShiftGameUnits = 0.0f;
            pivotAuthorityPositionConfidence = 0.0f;
            handScaleAtGrab = 1.0f;
            lastSeatedPivotReacquireLocalDeltaGameUnits = 0.0f;
            traceId = 0;
            traceTargetWriteSequence = 0;
            seatedPivotReacquireCount = 0;
            bodyResolutionReason = "none";
            multiFingerContactReason = "none";
            activeGrabPointMode = "none";
            pivotAuthoritySource = "none";
            palmSeatPointMode = "none";
            fingerEvidencePointMode = "none";
            fingerPoseAimReason = "none";
            gripSupportReason = "none";
            lastSeatedPivotReacquireReason = "none";
            lastSeatedPivotReacquirePhase = "none";
            seatMode = GrabSeatMode::None;
            gripSupportKind = grab_support_model_math::GripSupportKind::None;
            motorFadeReason = "none";
            captureTelemetry.clear();
            localMeshTriangles.clear();
            heldNode = nullptr;
            gripSourceNode = nullptr;
            hasMeshPoseData = false;
            hasGripPoint = false;
            hasGripSourceNodePoint = false;
            hasGripSourceNodeNormal = false;
            hasGripEvidenceShapeKey = false;
            hasFrozenPivotB = false;
            hasContactPatch = false;
            hasContactPatchEvidence = false;
            hasMultiFingerContactPatch = false;
            hasPalmSeatPoint = false;
            hasPinchPocket = false;
            hasFingerEvidencePoint = false;
            activeGrabPointUsesMultiFingerEvidence = false;
            hasGripSupportModel = false;
            hasSupportFrameNormal = false;
            hasSupportFrameAxis = false;
            hasSupportFrameBinormal = false;
            gripSupportAuthoredPivot = false;
            pivotAuthorityPositionOnly = false;
            pivotAuthorityNormalTrusted = false;
            requiresSettledVisualHandRelation = false;
            hasTelemetryCapture = false;
            hasSeatedPivotReacquire = false;
            fingerPoseAimValid = false;
            fadeInGrabConstraint = false;
        }

        bool isValid() const { return heldNode != nullptr || hasMeshPoseData || !localMeshTriangles.empty(); }
    };
}

// ---- GrabInteractionPolicy.h ----

#include "physics-interaction/grab/GrabInteractionPolicy.h"

// ---- GrabFrameMath.h ----

/*
 * ROCK's calibrated grab pivot is authored in semantic handspace, but the live
 * grab has two different consumers: the root-flattened hand frame and the
 * generated physics hand body. ROCK keeps one coherent palm-to-object relation
 * while preserving separate visual and collision frames. These helpers keep that
 * split explicit so pivot A, pivot B, captured object frames, and debug
 * measurements cannot silently drift onto different conventions.
 */

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandColliderTypes.h"

#include <span>
#include <utility>

namespace rock::grab_frame_math
{
    template <class Transform>
    struct SplitGrabFrame
    {
        using Vector = decltype(std::declval<Transform>().translate);

        Transform shiftedObjectWorld{};
        Transform rawHandSpace{};
        Transform handBodyToRawHandAtGrab{};
        Vector pivotAHandBodyLocal{};
    };

    template <class Transform, class Vector>
    inline Transform shiftObjectToAlignGripWithPocket(Transform objectWorld, const Vector& grabPocketWorld, const Vector& gripPointWorld)
    {
        objectWorld.translate.x += grabPocketWorld.x - gripPointWorld.x;
        objectWorld.translate.y += grabPocketWorld.y - gripPointWorld.y;
        objectWorld.translate.z += grabPocketWorld.z - gripPointWorld.z;
        return objectWorld;
    }

    template <class Transform>
    inline Transform objectInFrameSpace(const Transform& frameWorld, const Transform& objectWorld)
    {
        return transform_math::composeTransforms(transform_math::invertTransform(frameWorld), objectWorld);
    }

    template <class Transform>
    inline Transform objectInGeneratedProxyLocalSpace(const Transform& proxyWorld, const Transform& objectWorld)
    {
        using Vector = decltype(std::declval<Transform>().translate);

        Transform result = transform_math::makeIdentityTransform<Transform>();
        result.translate = hand_bone_collider_geometry_math::generatedColliderWorldPointToLocal(proxyWorld, objectWorld.translate);
        Transform proxyRotationWorld = proxyWorld;
        proxyRotationWorld.scale = 1.0f;

        const Vector objectAxisXWorld{ objectWorld.rotate.entry[0][0], objectWorld.rotate.entry[0][1], objectWorld.rotate.entry[0][2] };
        const Vector objectAxisYWorld{ objectWorld.rotate.entry[1][0], objectWorld.rotate.entry[1][1], objectWorld.rotate.entry[1][2] };
        const Vector objectAxisZWorld{ objectWorld.rotate.entry[2][0], objectWorld.rotate.entry[2][1], objectWorld.rotate.entry[2][2] };
        const Vector objectAxisXProxyLocal =
            hand_bone_collider_geometry_math::generatedColliderWorldVectorToLocal(proxyRotationWorld, objectAxisXWorld);
        const Vector objectAxisYProxyLocal =
            hand_bone_collider_geometry_math::generatedColliderWorldVectorToLocal(proxyRotationWorld, objectAxisYWorld);
        const Vector objectAxisZProxyLocal =
            hand_bone_collider_geometry_math::generatedColliderWorldVectorToLocal(proxyRotationWorld, objectAxisZWorld);

        result.rotate.entry[0][0] = objectAxisXProxyLocal.x;
        result.rotate.entry[0][1] = objectAxisXProxyLocal.y;
        result.rotate.entry[0][2] = objectAxisXProxyLocal.z;
        result.rotate.entry[1][0] = objectAxisYProxyLocal.x;
        result.rotate.entry[1][1] = objectAxisYProxyLocal.y;
        result.rotate.entry[1][2] = objectAxisYProxyLocal.z;
        result.rotate.entry[2][0] = objectAxisZProxyLocal.x;
        result.rotate.entry[2][1] = objectAxisZProxyLocal.y;
        result.rotate.entry[2][2] = objectAxisZProxyLocal.z;

        result.scale = proxyWorld.scale != 0.0f ? objectWorld.scale / proxyWorld.scale : objectWorld.scale;
        return result;
    }

    template <class Transform>
    inline Transform objectFromGeneratedProxyLocalSpace(const Transform& proxyWorld, const Transform& objectProxyLocal)
    {
        using Vector = decltype(std::declval<Transform>().translate);

        Transform result = transform_math::makeIdentityTransform<Transform>();
        result.translate =
            proxyWorld.translate +
            hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(proxyWorld, objectProxyLocal.translate);
        Transform proxyRotationWorld = proxyWorld;
        proxyRotationWorld.scale = 1.0f;

        const Vector objectAxisXProxyLocal{
            objectProxyLocal.rotate.entry[0][0],
            objectProxyLocal.rotate.entry[0][1],
            objectProxyLocal.rotate.entry[0][2],
        };
        const Vector objectAxisYProxyLocal{
            objectProxyLocal.rotate.entry[1][0],
            objectProxyLocal.rotate.entry[1][1],
            objectProxyLocal.rotate.entry[1][2],
        };
        const Vector objectAxisZProxyLocal{
            objectProxyLocal.rotate.entry[2][0],
            objectProxyLocal.rotate.entry[2][1],
            objectProxyLocal.rotate.entry[2][2],
        };
        const Vector objectAxisXWorld =
            hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(proxyRotationWorld, objectAxisXProxyLocal);
        const Vector objectAxisYWorld =
            hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(proxyRotationWorld, objectAxisYProxyLocal);
        const Vector objectAxisZWorld =
            hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(proxyRotationWorld, objectAxisZProxyLocal);

        result.rotate.entry[0][0] = objectAxisXWorld.x;
        result.rotate.entry[0][1] = objectAxisXWorld.y;
        result.rotate.entry[0][2] = objectAxisXWorld.z;
        result.rotate.entry[1][0] = objectAxisYWorld.x;
        result.rotate.entry[1][1] = objectAxisYWorld.y;
        result.rotate.entry[1][2] = objectAxisYWorld.z;
        result.rotate.entry[2][0] = objectAxisZWorld.x;
        result.rotate.entry[2][1] = objectAxisZWorld.y;
        result.rotate.entry[2][2] = objectAxisZWorld.z;

        result.scale = proxyWorld.scale * objectProxyLocal.scale;
        return result;
    }

    template <class Transform, class Vector>
    inline Vector computePivotAHandBodyLocal(const Transform& handBodyWorld, const Vector& grabPivotWorld)
    {
        /*
         * The active hand body for proxy grab authority is the generated palm
         * proxy frame, whose physical local axes are stored as Ni columns.
         */
        return hand_bone_collider_geometry_math::generatedColliderWorldPointToLocal(handBodyWorld, grabPivotWorld);
    }

    template <class Transform>
    inline Transform computeFrameFromCapturedObject(const Transform& heldNodeWorld, const Transform& rawHandSpace)
    {
        return transform_math::composeTransforms(heldNodeWorld, transform_math::invertTransform(rawHandSpace));
    }

    template <class Transform, class Vector>
    inline SplitGrabFrame<Transform> buildSplitGrabFrameFromDesiredObject(const Transform& rawHandWorld,
        const Transform& handBodyWorld,
        const Transform& desiredObjectWorld,
        const Vector& grabPivotWorld)
    {
        SplitGrabFrame<Transform> result{};
        result.shiftedObjectWorld = desiredObjectWorld;
        result.rawHandSpace = objectInFrameSpace(rawHandWorld, desiredObjectWorld);
        result.handBodyToRawHandAtGrab = objectInGeneratedProxyLocalSpace(handBodyWorld, rawHandWorld);
        result.pivotAHandBodyLocal = computePivotAHandBodyLocal(handBodyWorld, grabPivotWorld);
        return result;
    }
}

// ---- GrabAuthorityFrameMath.h ----

/*
 * Active grab authority is one resolved point frozen through one coherent set of
 * body/proxy relations. Evidence may choose the point, but it must not become a
 * second live motor truth after the frame is frozen.
 */

namespace rock::grab_authority_frame_math
{
    inline constexpr std::uint32_t kInvalidGrabAuthorityBodyId = 0x7FFF'FFFFu;

    enum class GrabAuthorityPivotSource : std::uint8_t
    {
        None,
        PinchPocket,
        GripSupportModel,
        PalmPocketMesh,
        SelectionMeshSnap,
        CollisionFallback
    };

    inline const char* grabAuthorityPivotSourceName(GrabAuthorityPivotSource source)
    {
        switch (source) {
        case GrabAuthorityPivotSource::PinchPocket:
            return "pinchPocket";
        case GrabAuthorityPivotSource::GripSupportModel:
            return "gripSupportModel";
        case GrabAuthorityPivotSource::PalmPocketMesh:
            return "palmPocketMesh";
        case GrabAuthorityPivotSource::SelectionMeshSnap:
            return "selectionMeshSnap";
        case GrabAuthorityPivotSource::CollisionFallback:
            return "collisionFallback";
        case GrabAuthorityPivotSource::None:
        default:
            return "none";
        }
    }

    inline int grabAuthorityPivotSourcePriority(GrabAuthorityPivotSource source)
    {
        switch (source) {
        case GrabAuthorityPivotSource::PinchPocket:
            return 0;
        case GrabAuthorityPivotSource::GripSupportModel:
            return 1;
        case GrabAuthorityPivotSource::PalmPocketMesh:
            return 2;
        case GrabAuthorityPivotSource::SelectionMeshSnap:
            return 3;
        case GrabAuthorityPivotSource::CollisionFallback:
            return 4;
        case GrabAuthorityPivotSource::None:
        default:
            return 100;
        }
    }

    inline bool isFinalGrabAuthorityPivotSource(GrabAuthorityPivotSource source)
    {
        return source == GrabAuthorityPivotSource::PinchPocket ||
               source == GrabAuthorityPivotSource::GripSupportModel;
    }

    template <class Vector>
    inline bool isFiniteVector(const Vector& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    template <class Transform>
    inline bool isFiniteTransform(const Transform& value)
    {
        bool rotationFinite = true;
        for (std::uint32_t row = 0; row < 3; ++row) {
            for (std::uint32_t column = 0; column < 3; ++column) {
                rotationFinite = rotationFinite && std::isfinite(value.rotate.entry[row][column]);
            }
        }
        return rotationFinite && isFiniteVector(value.translate) && std::isfinite(value.scale) && value.scale > 0.0001f;
    }

    template <class Vector>
    struct GrabAuthorityPivotCandidate
    {
        bool valid = false;
        GrabAuthorityPivotSource source = GrabAuthorityPivotSource::None;
        Vector pointWorld{};
        Vector normalWorld{};
        bool normalValid = false;
        std::uint32_t bodyId = kInvalidGrabAuthorityBodyId;
        const void* sourceNode = nullptr;
        const char* reason = "notEvaluated";
    };

    template <class Vector>
    using ResolvedGrabAuthorityPivot = GrabAuthorityPivotCandidate<Vector>;

    template <class Vector>
    inline ResolvedGrabAuthorityPivot<Vector> resolveGrabAuthorityPivot(
        std::span<const GrabAuthorityPivotCandidate<Vector>> candidates)
    {
        ResolvedGrabAuthorityPivot<Vector> selected{};
        selected.reason = "noFinalPinchOrSupportAuthority";
        int selectedPriority = grabAuthorityPivotSourcePriority(GrabAuthorityPivotSource::None);

        for (const auto& candidate : candidates) {
            if (!candidate.valid ||
                candidate.source == GrabAuthorityPivotSource::None ||
                candidate.bodyId == kInvalidGrabAuthorityBodyId ||
                !isFiniteVector(candidate.pointWorld)) {
                continue;
            }
            if (!isFinalGrabAuthorityPivotSource(candidate.source)) {
                continue;
            }

            const int priority = grabAuthorityPivotSourcePriority(candidate.source);
            if (!selected.valid || priority < selectedPriority) {
                selected = candidate;
                selectedPriority = priority;
            }
        }

        if (selected.valid) {
            selected.reason = selected.reason ? selected.reason : grabAuthorityPivotSourceName(selected.source);
        }
        return selected;
    }

    template <class Transform, class Vector>
    inline bool alignLocalPointInTransformToLocalTarget(Transform& transform, const Vector& localPoint, const Vector& targetPoint)
    {
        if (!isFiniteTransform(transform) || !isFiniteVector(localPoint) || !isFiniteVector(targetPoint)) {
            return false;
        }

        const Vector currentPoint = transform_math::localPointToWorld(transform, localPoint);
        if (!isFiniteVector(currentPoint)) {
            return false;
        }

        transform.translate.x += targetPoint.x - currentPoint.x;
        transform.translate.y += targetPoint.y - currentPoint.y;
        transform.translate.z += targetPoint.z - currentPoint.z;
        return isFiniteTransform(transform);
    }

    template <class Transform>
    struct GrabAuthorityFrameFreezeInput
    {
        using Vector = decltype(std::declval<Transform>().translate);

        Transform rawHandWorld{};
        Transform proxyWorld{};
        Transform proxyAuthorityFrameWorld{};
        Transform objectWorld{};
        Transform bodyWorld{};
        Transform constraintBodyWorld{};
        Transform rootBodyLocal{};
        Transform ownerBodyLocal{};
        Transform desiredObjectWorld{};
        Transform desiredBodyWorld{};
        Vector pivotAWorld{};
        Vector gripPointWorld{};
        Vector visualNormalWorld{};
        GrabAuthorityPivotSource source = GrabAuthorityPivotSource::None;
        bool hasDesiredObjectWorld = false;
        bool hasDesiredBodyWorld = false;
        bool visualNormalValid = false;
    };

    template <class Transform>
    struct FrozenGrabAuthorityFrame
    {
        using Vector = decltype(std::declval<Transform>().translate);

        bool valid = false;
        GrabAuthorityPivotSource source = GrabAuthorityPivotSource::None;
        Vector pivotAWorld{};
        Vector pivotBBodyLocalGame{};
        Vector pivotBConstraintLocalGame{};
        Transform bodyLocal{};
        Transform bodyWorldAtGrab{};
        Transform rootBodyLocal{};
        Transform ownerBodyLocal{};
        Vector gripPointLocal{};
        Vector grabPivotWorldAtGrab{};
        Vector gripPointWorldAtGrab{};
        Transform desiredObjectWorld{};
        Transform desiredBodyWorld{};
        Transform rawHandSpace{};
        Transform handBodyToRawHandAtGrab{};
        Vector pivotAHandBodyLocalGame{};
        Transform proxyAuthorityHandSpace{};
        Transform proxyAuthorityBodyHandSpace{};
        Vector visualNormalWorld{};
        bool visualNormalValid = false;
    };

    template <class Transform>
    inline FrozenGrabAuthorityFrame<Transform> freezeGrabAuthorityFrame(
        const GrabAuthorityFrameFreezeInput<Transform>& input)
    {
        FrozenGrabAuthorityFrame<Transform> frozen{};
        frozen.source = input.source;
        frozen.pivotAWorld = input.pivotAWorld;
        frozen.grabPivotWorldAtGrab = input.pivotAWorld;
        frozen.gripPointWorldAtGrab = input.gripPointWorld;
        frozen.rootBodyLocal = input.rootBodyLocal;
        frozen.ownerBodyLocal = input.ownerBodyLocal;
        frozen.bodyWorldAtGrab = input.bodyWorld;
        frozen.visualNormalWorld = input.visualNormalWorld;
        frozen.visualNormalValid = input.visualNormalValid && isFiniteVector(input.visualNormalWorld);

        if (!isFinalGrabAuthorityPivotSource(input.source)) {
            return frozen;
        }

        if (!isFiniteTransform(input.rawHandWorld) ||
            !isFiniteTransform(input.proxyWorld) ||
            !isFiniteTransform(input.proxyAuthorityFrameWorld) ||
            !isFiniteTransform(input.objectWorld) ||
            !isFiniteTransform(input.bodyWorld) ||
            !isFiniteTransform(input.constraintBodyWorld) ||
            !isFiniteVector(input.pivotAWorld) ||
            !isFiniteVector(input.gripPointWorld)) {
            return frozen;
        }

        frozen.bodyLocal = transform_math::composeTransforms(
            transform_math::invertTransform(input.objectWorld),
            input.bodyWorld);
        if (!isFiniteTransform(frozen.bodyLocal)) {
            return frozen;
        }
        /*
         * The solver target is BODY authority. Visual object space may describe
         * mesh evidence, but it must not define body-B's frozen motor frame.
         */
        frozen.desiredBodyWorld = input.hasDesiredBodyWorld ?
            input.desiredBodyWorld :
            grab_frame_math::shiftObjectToAlignGripWithPocket(
                input.bodyWorld,
                input.pivotAWorld,
                input.gripPointWorld);
        if (!isFiniteTransform(frozen.desiredBodyWorld)) {
            return frozen;
        }

        frozen.gripPointLocal = transform_math::worldPointToLocal(input.objectWorld, input.gripPointWorld);
        frozen.pivotBBodyLocalGame = transform_math::worldPointToLocal(input.bodyWorld, input.gripPointWorld);
        frozen.pivotBConstraintLocalGame = transform_math::worldPointToLocal(input.constraintBodyWorld, input.gripPointWorld);
        frozen.pivotAHandBodyLocalGame = grab_frame_math::computePivotAHandBodyLocal(input.proxyWorld, input.pivotAWorld);
        if (!isFiniteVector(frozen.gripPointLocal) ||
            !isFiniteVector(frozen.pivotBBodyLocalGame) ||
            !isFiniteVector(frozen.pivotBConstraintLocalGame) ||
            !isFiniteVector(frozen.pivotAHandBodyLocalGame)) {
            return frozen;
        }

        /*
         * BODY is the solver authority. The selected BODY-local pivot B must be
         * the point written into transform-B, so freeze the generated/proxy BODY
         * relation around that exact point. The proxy parent is a generated
         * collider frame, so use the same column-authored local conversion as
         * the collider and pivot code instead of normal NiTransform inverse math.
         * Otherwise a relation-implied pivot can replace the selected grip point
         * and feed Havok a coherent but wrong linear/angular frame pair.
         */
        frozen.proxyAuthorityBodyHandSpace =
            grab_frame_math::objectInGeneratedProxyLocalSpace(input.proxyWorld, frozen.desiredBodyWorld);
        if (!alignLocalPointInTransformToLocalTarget(
                frozen.proxyAuthorityBodyHandSpace,
                frozen.pivotBConstraintLocalGame,
                frozen.pivotAHandBodyLocalGame)) {
            return frozen;
        }

        frozen.desiredBodyWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(input.proxyWorld, frozen.proxyAuthorityBodyHandSpace);
        if (!isFiniteTransform(frozen.desiredBodyWorld)) {
            return frozen;
        }

        frozen.desiredObjectWorld = transform_math::composeTransforms(
            frozen.desiredBodyWorld,
            transform_math::invertTransform(frozen.bodyLocal));
        if (!isFiniteTransform(frozen.desiredObjectWorld)) {
            return frozen;
        }

        const auto splitFrame = grab_frame_math::buildSplitGrabFrameFromDesiredObject(
            input.rawHandWorld,
            input.proxyWorld,
            frozen.desiredObjectWorld,
            input.pivotAWorld);
        frozen.rawHandSpace = splitFrame.rawHandSpace;
        frozen.handBodyToRawHandAtGrab = splitFrame.handBodyToRawHandAtGrab;
        frozen.pivotAHandBodyLocalGame = splitFrame.pivotAHandBodyLocal;
        frozen.proxyAuthorityHandSpace =
            grab_frame_math::objectInGeneratedProxyLocalSpace(input.proxyWorld, frozen.desiredObjectWorld);
        frozen.valid =
            isFiniteVector(frozen.pivotBBodyLocalGame) &&
            isFiniteVector(frozen.pivotBConstraintLocalGame) &&
            isFiniteVector(frozen.pivotAHandBodyLocalGame) &&
            isFiniteTransform(frozen.bodyLocal) &&
            isFiniteTransform(frozen.desiredBodyWorld) &&
            isFiniteTransform(frozen.rawHandSpace) &&
            isFiniteTransform(frozen.proxyAuthorityHandSpace) &&
            isFiniteTransform(frozen.proxyAuthorityBodyHandSpace);
        return frozen;
    }
}

// ---- PullMotionMath.h ----

#include <algorithm>
#include <cmath>

namespace rock::pull_motion_math
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
        float ownerGraceSeconds = 1.0f;
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
        /*
         * ROCK drives the pulled body for a short predicted-velocity window,
         * then keeps the pulled-object owner alive until pullDuration + grace so
         * heavy or cluttered objects can still be reacquired by the close catch
         * path after the impulse phase ends.
         */
        const float elapsedSeconds = std::isfinite(input.elapsedSeconds) ? (std::max)(0.0f, input.elapsedSeconds) : 0.0f;
        const float durationSeconds = std::isfinite(input.durationSeconds) ? (std::max)(0.0f, input.durationSeconds) : 0.0f;
        const float applyVelocitySeconds = std::isfinite(input.applyVelocitySeconds) ? (std::max)(0.0f, input.applyVelocitySeconds) : 0.0f;
        const float ownerGraceSeconds = std::isfinite(input.ownerGraceSeconds) ? (std::max)(0.0f, input.ownerGraceSeconds) : 0.0f;
        const float trackHandSeconds = std::isfinite(input.trackHandSeconds) ? (std::max)(0.0f, input.trackHandSeconds) : 0.0f;

        PullMotionResult<Vec3> result{};
        result.refreshTarget = !input.hasPreviousTarget || elapsedSeconds <= trackHandSeconds;
        result.targetHavok = result.refreshTarget ? input.handHavok : input.previousTargetHavok;
        if (result.refreshTarget) {
            result.targetHavok.z += input.destinationOffsetHavok;
        }

        if (elapsedSeconds > durationSeconds + ownerGraceSeconds) {
            result.expired = true;
            return result;
        }

        const float rawDurationRemainingSeconds = durationSeconds - elapsedSeconds;
        result.durationRemainingSeconds = (std::max)(0.0f, rawDurationRemainingSeconds);
        if (elapsedSeconds > applyVelocitySeconds || rawDurationRemainingSeconds <= 0.001f) {
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
