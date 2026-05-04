#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

#include "physics-interaction/object/GeometryBodyResolver.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"

namespace RE
{
    class NiAVObject;
    class TESObjectREFR;
    class bhkWorld;
    class hknpWorld;
}

namespace frik::rock::object_physics_body_set
{
    inline constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

    struct PurePoint3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        constexpr PurePoint3() = default;
        constexpr PurePoint3(float xIn, float yIn, float zIn) : x(xIn), y(yIn), z(zIn) {}

        template <class Vec3>
        constexpr PurePoint3(const Vec3& value) : x(value.x), y(value.y), z(value.z)
        {}
    };

    struct PureBodyRecord
    {
        std::uint32_t bodyId = 0x7FFF'FFFF;
        std::uint32_t motionId = 0;
        bool accepted = false;
        PurePoint3 position{};
        std::uintptr_t ownerKey = 0;
    };

    enum class PrimaryBodyChoiceReason : std::uint8_t
    {
        None,
        PreferredHitAccepted,
        SurfaceOwnerAccepted,
        NearestAcceptedFallback,
        NoAcceptedBody,
    };

    struct PrimaryBodyChoice
    {
        std::uint32_t bodyId = 0x7FFF'FFFF;
        PrimaryBodyChoiceReason reason = PrimaryBodyChoiceReason::None;
    };

    class PureBodySet
    {
    public:
        std::vector<PureBodyRecord> records;

        std::vector<std::uint32_t> acceptedBodyIds() const
        {
            std::vector<std::uint32_t> result;
            for (const auto& record : records) {
                if (record.accepted) {
                    result.push_back(record.bodyId);
                }
            }
            return result;
        }

        std::vector<std::uint32_t> uniqueAcceptedMotionBodyIds() const
        {
            std::vector<std::uint32_t> result;
            std::unordered_set<std::uint32_t> seenMotionIds;
            for (const auto& record : records) {
                if (!record.accepted) {
                    continue;
                }
                if (seenMotionIds.insert(record.motionId).second) {
                    result.push_back(record.bodyId);
                }
            }
            return result;
        }

        std::uint32_t duplicateAcceptedMotionSkips() const
        {
            std::uint32_t skips = 0;
            std::unordered_set<std::uint32_t> seenMotionIds;
            for (const auto& record : records) {
                if (!record.accepted) {
                    continue;
                }
                if (!seenMotionIds.insert(record.motionId).second) {
                    ++skips;
                }
            }
            return skips;
        }

        bool containsAcceptedBody(std::uint32_t bodyId) const
        {
            return std::any_of(records.begin(), records.end(), [&](const PureBodyRecord& record) { return record.accepted && record.bodyId == bodyId; });
        }

        PrimaryBodyChoice choosePrimaryBody(std::uint32_t preferredBodyId, PurePoint3 targetPoint) const
        {
            if (containsAcceptedBody(preferredBodyId)) {
                return PrimaryBodyChoice{ .bodyId = preferredBodyId, .reason = PrimaryBodyChoiceReason::PreferredHitAccepted };
            }

            float bestDistance = (std::numeric_limits<float>::max)();
            std::uint32_t bestBodyId = INVALID_BODY_ID;
            for (const auto& record : records) {
                if (!record.accepted) {
                    continue;
                }
                const float dx = record.position.x - targetPoint.x;
                const float dy = record.position.y - targetPoint.y;
                const float dz = record.position.z - targetPoint.z;
                const float distSq = dx * dx + dy * dy + dz * dz;
                if (distSq < bestDistance) {
                    bestDistance = distSq;
                    bestBodyId = record.bodyId;
                }
            }

            if (bestBodyId == INVALID_BODY_ID) {
                return PrimaryBodyChoice{ .bodyId = INVALID_BODY_ID, .reason = PrimaryBodyChoiceReason::NoAcceptedBody };
            }
            return PrimaryBodyChoice{ .bodyId = bestBodyId, .reason = PrimaryBodyChoiceReason::NearestAcceptedFallback };
        }

        PrimaryBodyChoice choosePrimaryBodyWithSurfaceOwner(std::uint32_t preferredBodyId, std::uintptr_t ownerKey, PurePoint3 targetPoint) const
        {
            std::uint32_t ownerBodyId = INVALID_BODY_ID;
            if (ownerKey != 0) {
                for (const auto& record : records) {
                    if (record.accepted && record.ownerKey == ownerKey) {
                        ownerBodyId = record.bodyId;
                        break;
                    }
                }
            }

            const auto nearestChoice = choosePrimaryBody(INVALID_BODY_ID, targetPoint);
            const auto resolution = geometry_body_resolver::resolveGeometryBody(geometry_body_resolver::GeometryBodyResolutionInput{
                .triangleOwnerBodyId = ownerBodyId,
                .selectedBodyId = preferredBodyId,
                .nearestBodyId = nearestChoice.bodyId,
                .triangleOwnerUsable = containsAcceptedBody(ownerBodyId),
                .selectedUsable = containsAcceptedBody(preferredBodyId),
                .nearestUsable = nearestChoice.bodyId != INVALID_BODY_ID,
            });
            switch (resolution.source) {
            case geometry_body_resolver::GeometryBodyResolutionSource::TriangleOwner:
                return PrimaryBodyChoice{ .bodyId = resolution.bodyId, .reason = PrimaryBodyChoiceReason::SurfaceOwnerAccepted };
            case geometry_body_resolver::GeometryBodyResolutionSource::SelectedBody:
                return PrimaryBodyChoice{ .bodyId = resolution.bodyId, .reason = PrimaryBodyChoiceReason::PreferredHitAccepted };
            case geometry_body_resolver::GeometryBodyResolutionSource::NearestAccepted:
                return PrimaryBodyChoice{ .bodyId = resolution.bodyId, .reason = PrimaryBodyChoiceReason::NearestAcceptedFallback };
            default:
                return PrimaryBodyChoice{ .bodyId = INVALID_BODY_ID, .reason = PrimaryBodyChoiceReason::NoAcceptedBody };
            }
        }
    };

    class BodySetBuilder
    {
    public:
        void addForTest(const PureBodyRecord& record)
        {
            if (record.bodyId == INVALID_BODY_ID) {
                return;
            }
            if (!_seenBodyIds.insert(record.bodyId).second) {
                return;
            }
            _set.records.push_back(record);
        }

        PureBodySet build() const { return _set; }

    private:
        PureBodySet _set;
        std::unordered_set<std::uint32_t> _seenBodyIds;
    };

    struct BodySetScanOptions
    {
        physics_body_classifier::InteractionMode mode = physics_body_classifier::InteractionMode::ActiveGrab;
        std::uint32_t rightHandBodyId = INVALID_BODY_ID;
        std::uint32_t leftHandBodyId = INVALID_BODY_ID;
        std::uint32_t sourceWeaponBodyId = INVALID_BODY_ID;
        std::uint32_t sourceBodyId = INVALID_BODY_ID;
        const std::vector<std::uint32_t>* heldBySameHand = nullptr;
        int maxDepth = 10;
    };

    struct ObjectPhysicsBodyRecord
    {
        std::uint32_t bodyId = INVALID_BODY_ID;
        std::uint32_t motionId = 0;
        std::uint32_t collisionLayer = 0;
        std::uint32_t filterInfo = 0;
        std::uint16_t motionPropertiesId = 0;
        physics_body_classifier::BodyMotionType motionType = physics_body_classifier::BodyMotionType::Unknown;
        std::uint32_t bodyFlags = 0;
        PurePoint3 positionGame{};
        RE::NiAVObject* owningNode = nullptr;
        void* collisionObject = nullptr;
        RE::TESObjectREFR* resolvedRef = nullptr;
        bool accepted = false;
        physics_body_classifier::BodyRejectReason rejectReason = physics_body_classifier::BodyRejectReason::None;
    };

    struct BodySetDiagnostics
    {
        std::uint32_t visitedNodes = 0;
        std::uint32_t collisionObjects = 0;
        std::uint32_t duplicateBodySkips = 0;
        std::uint32_t duplicateMotionSkips = 0;
        std::array<std::uint32_t, 32> rejectCounts{};
    };

    class ObjectPhysicsBodySet
    {
    public:
        RE::TESObjectREFR* rootRef = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        std::vector<ObjectPhysicsBodyRecord> records;
        mutable BodySetDiagnostics diagnostics;

        std::size_t acceptedCount() const;
        std::size_t rejectedCount() const;
        std::vector<std::uint32_t> acceptedBodyIds() const;
        std::vector<std::uint32_t> uniqueAcceptedMotionBodyIds() const;
        std::vector<const ObjectPhysicsBodyRecord*> uniqueAcceptedMotionRecords() const;
        bool containsAcceptedBody(std::uint32_t bodyId) const;
        const ObjectPhysicsBodyRecord* findRecord(std::uint32_t bodyId) const;
        const ObjectPhysicsBodyRecord* findAcceptedRecordByOwnerNode(RE::NiAVObject* ownerNode) const;
        PrimaryBodyChoice choosePrimaryBody(std::uint32_t preferredBodyId, PurePoint3 targetPointGame) const;
        PrimaryBodyChoice choosePrimaryBodyWithSurfaceOwner(std::uint32_t preferredBodyId, RE::NiAVObject* ownerNode, PurePoint3 targetPointGame) const;
    };

    bool hasCollisionObjectInSubtree(RE::NiAVObject* root, int maxDepth);

    ObjectPhysicsBodySet scanObjectPhysicsBodySet(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::TESObjectREFR* ref, const BodySetScanOptions& options);
}
