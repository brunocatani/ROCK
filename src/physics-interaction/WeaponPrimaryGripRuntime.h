#pragma once

#include "MeshGrab.h"
#include "PalmTransform.h"
#include "TransformMath.h"
#include "WeaponCollisionGeometryMath.h"
#include "WeaponPartClassifier.h"
#include "WeaponPrimaryGripMath.h"
#include "WeaponTwoHandedGripMath.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <limits>
#include <vector>

namespace frik::rock::weapon_primary_grip_runtime
{
    /*
     * Primary grip discovery is shared by one-handed weapon authority and
     * two-handed support grip. HIGGS records one hand-to-object relationship
     * and advances dependent systems from that relationship; ROCK follows that
     * model by deriving one mesh-local hand frame here, then letting the caller
     * decide whether it publishes a one-hand or two-hand FRIK authority frame.
     */
    using Candidate = weapon_primary_grip_math::PrimaryGripCandidate<RE::NiPoint3>;
    using Selection = weapon_primary_grip_math::PrimaryGripSelection<RE::NiTransform>;

    struct RuntimeSelection
    {
        Selection selection{};
        std::size_t candidateCount{ 0 };
    };

    inline RE::NiPoint3 worldToWeaponLocalPoint(const RE::NiPoint3& worldPos, const RE::NiNode* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::worldPointToLocal(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, worldPos);
    }

    inline void expandBounds(RE::NiPoint3& minValue, RE::NiPoint3& maxValue, const RE::NiPoint3& value)
    {
        minValue.x = (std::min)(minValue.x, value.x);
        minValue.y = (std::min)(minValue.y, value.y);
        minValue.z = (std::min)(minValue.z, value.z);
        maxValue.x = (std::max)(maxValue.x, value.x);
        maxValue.y = (std::max)(maxValue.y, value.y);
        maxValue.z = (std::max)(maxValue.z, value.z);
    }

    inline bool buildPrimaryGripCandidate(RE::NiAVObject* object, const RE::NiNode* weaponNode, Candidate& outCandidate)
    {
        if (!object || !weaponNode) {
            return false;
        }

        const std::string_view objectName = object->name.c_str() ? object->name.c_str() : "";
        const auto classification = classifyWeaponPartName(objectName);
        if (!weapon_primary_grip_math::isPrimaryGripName(objectName) &&
            classification.partKind != WeaponPartKind::Grip &&
            classification.partKind != WeaponPartKind::Receiver) {
            return false;
        }

        std::vector<TriangleData> triangles;
        extractAllTriangles(object, triangles, 4);
        if (triangles.empty()) {
            return false;
        }

        RE::NiPoint3 minValue{
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)()
        };
        RE::NiPoint3 maxValue{
            (std::numeric_limits<float>::lowest)(),
            (std::numeric_limits<float>::lowest)(),
            (std::numeric_limits<float>::lowest)()
        };
        RE::NiPoint3 normalSum{};

        for (const auto& triangle : triangles) {
            expandBounds(minValue, maxValue, worldToWeaponLocalPoint(triangle.v0, weaponNode));
            expandBounds(minValue, maxValue, worldToWeaponLocalPoint(triangle.v1, weaponNode));
            expandBounds(minValue, maxValue, worldToWeaponLocalPoint(triangle.v2, weaponNode));
            const RE::NiPoint3 worldNormal = normalize(cross(sub(triangle.v1, triangle.v0), sub(triangle.v2, triangle.v0)));
            const RE::NiPoint3 localNormal = transform_math::worldVectorToLocal(weaponNode->world, worldNormal);
            normalSum.x += localNormal.x;
            normalSum.y += localNormal.y;
            normalSum.z += localNormal.z;
        }

        outCandidate.name = objectName;
        outCandidate.partKind = classification.partKind;
        outCandidate.localMin = minValue;
        outCandidate.localMax = maxValue;
        outCandidate.localCenter = RE::NiPoint3{
            (minValue.x + maxValue.x) * 0.5f,
            (minValue.y + maxValue.y) * 0.5f,
            (minValue.z + maxValue.z) * 0.5f
        };
        outCandidate.triangleCount = triangles.size();
        outCandidate.localSurfaceNormal = normalize(normalSum);
        outCandidate.hasSurfaceNormal = dot(outCandidate.localSurfaceNormal, outCandidate.localSurfaceNormal) > 0.0f;
        return true;
    }

    inline void collectPrimaryGripCandidates(RE::NiAVObject* object, const RE::NiNode* weaponNode, std::vector<Candidate>& outCandidates, int depth)
    {
        if (!object || depth <= 0 || (object->flags.flags & 1) != 0) {
            return;
        }

        Candidate candidate{};
        if (buildPrimaryGripCandidate(object, weaponNode, candidate)) {
            outCandidates.push_back(candidate);
        }

        auto* niNode = object->IsNode();
        if (!niNode) {
            return;
        }

        auto& children = niNode->GetRuntimeData().children;
        for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
            collectPrimaryGripCandidates(children[i].get(), weaponNode, outCandidates, depth - 1);
        }
    }

    inline RuntimeSelection selectPrimaryGripFrameForWeapon(RE::NiNode* weaponNode, const RE::NiTransform& fallbackFrame, bool iniOverrideActive, int maxDepth = 8)
    {
        RuntimeSelection result{};
        if (!weaponNode) {
            result.selection = weapon_primary_grip_math::selectPrimaryGripFrame<RE::NiTransform, RE::NiPoint3>({}, fallbackFrame, iniOverrideActive);
            return result;
        }

        std::vector<Candidate> candidates;
        collectPrimaryGripCandidates(weaponNode, weaponNode, candidates, maxDepth);
        result.candidateCount = candidates.size();
        result.selection = weapon_primary_grip_math::selectPrimaryGripFrame<RE::NiTransform, RE::NiPoint3>(candidates, fallbackFrame, iniOverrideActive);
        return result;
    }

    inline RE::NiTransform alignHandFrameToGripFrame(
        const RE::NiTransform& currentHandTransform,
        bool isLeft,
        const RE::NiTransform& weaponWorld,
        const RE::NiTransform& gripWeaponLocalFrame,
        const RE::NiPoint3& fallbackPalmPosition,
        const RE::NiPoint3& gripWorldPoint,
        bool useMeshFrameRotation)
    {
        RE::NiTransform adjusted = weapon_two_handed_grip_math::alignHandFrameToGripPoint(currentHandTransform, fallbackPalmPosition, gripWorldPoint);
        if (!useMeshFrameRotation) {
            return adjusted;
        }

        const RE::NiTransform gripWorldFrame = transform_math::composeTransforms(weaponWorld, gripWeaponLocalFrame);
        adjusted.rotate = gripWorldFrame.rotate;
        const RE::NiPoint3 pivotWithMeshRotation = computeGrabPivotAPositionFromHandBasis(adjusted, isLeft);
        adjusted.translate.x += gripWorldPoint.x - pivotWithMeshRotation.x;
        adjusted.translate.y += gripWorldPoint.y - pivotWithMeshRotation.y;
        adjusted.translate.z += gripWorldPoint.z - pivotWithMeshRotation.z;
        return adjusted;
    }
}
