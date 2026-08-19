#pragma once

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/grab/HandGrabContactEvidence.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rock::hand_grab_detail
{
    inline constexpr std::size_t kMaxRuntimeGripSupportSamples = 16;

    struct RuntimeGripSupportModel
    {
        grab_support_model_math::GripSupportModel<RE::NiPoint3> model{};
        std::array<grab_support_model_math::GripSupportSample<RE::NiPoint3>, kMaxRuntimeGripSupportSamples> samples{};
        std::uint32_t sampleCount = 0;
        std::uint32_t meshProbeHitCount = 0;
        std::uint32_t rejectedOwnerCount = 0;
        std::uint32_t rejectedDistanceCount = 0;
        bool valid = false;
    };

    struct GrabMeshLongAxisResult
    {
        RE::NiPoint3 axisWorld{};
        RE::NiPoint3 secondAxisWorld{};
        float elongationRatio = 0.0f;
        float secondElongationRatio = 0.0f;
        std::uint32_t triangleCount = 0;
        const char* reason = "notEvaluated";
        bool valid = false;
    };

    [[nodiscard]] const char* gripSupportActivePointMode(grab_support_model_math::GripSupportKind kind);
    bool forceRuntimeGripSupportAuthority(
        RuntimeGripSupportModel& support,
        const RE::NiPoint3& activeGripPointWorld,
        const RE::NiPoint3& activeGripNormalWorld,
        const RE::NiPoint3& fallbackNormalWorld,
        const RE::NiPoint3& fallbackAxisWorld,
        float confidenceFloor,
        const char* reason);
    [[nodiscard]] RuntimeGripSupportModel buildRuntimeGripSupportModel(
        const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const RE::NiTransform& objectWorldTransform,
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
        const std::vector<GrabLocalTriangle>& localMeshTriangles,
        const RuntimeGrabContactPatch& contactPatch,
        const RuntimePinchPocketCandidate& pinchPocket,
        const RE::NiPoint3& activeGripPointWorld,
        const RE::NiPoint3& activeGripNormalWorld,
        const RE::NiPoint3& grabPivotAWorld,
        const RE::NiPoint3& palmNormalWorld,
        const RE::NiPoint3& fingerAxisWorld,
        const RE::NiPoint3& acrossPalmAxisWorld,
        float longObjectLeverGameUnits);
    [[nodiscard]] GrabMeshLongAxisResult computeGrabMeshLongAxis(
        const std::vector<TriangleData>& worldTriangles);
}
