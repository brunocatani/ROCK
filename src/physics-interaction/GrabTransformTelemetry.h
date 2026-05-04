#pragma once

/*
 * Grab transform telemetry is diagnostic-only: it compares ROCK's live grab
 * frames against the HIGGS reverse-hand convention without changing the active
 * constraint, visual authority, or finger pose path. Keeping the math and stamped
 * formatting here lets the VR overlay and log rows report the same values, so an
 * in-game screenshot can be matched to the exact numeric frame in ROCK.log.
 */

#include "GrabFrameMath.h"
#include "PhysicsBodyFrame.h"
#include "TransformMath.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace frik::rock::grab_transform_telemetry
{
    struct FrameStamp
    {
        std::uint32_t session = 0;
        std::uint64_t frame = 0;
        std::uint64_t tickMs = 0;
    };

    struct TransformDelta
    {
        float positionGameUnits = 0.0f;
        float rotationDegrees = 0.0f;
    };

    struct AxisAlignmentDots
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    template <class Vector>
    struct PointPairDelta
    {
        Vector delta{};
        float distance = 0.0f;
    };

    struct RuntimeSample
    {
        std::uint32_t heldFormId = 0;
        std::uint32_t heldBodyId = 0x7FFF'FFFF;
        std::uint32_t handBodyId = 0x7FFF'FFFF;
        std::uint32_t heldMotionIndex = body_frame::kFreeMotionIndex;
        std::uint32_t handMotionIndex = body_frame::kFreeMotionIndex;
        body_frame::BodyFrameSource heldBodySource{ body_frame::BodyFrameSource::Fallback };
        body_frame::BodyFrameSource handBodySource{ body_frame::BodyFrameSource::Fallback };

        RE::NiTransform rawHandWorld{};
        RE::NiTransform handBodyWorld{};
        RE::NiTransform heldBodyWorld{};
        RE::NiTransform heldNodeWorld{};
        RE::NiTransform heldBodyDerivedNodeWorld{};
        RE::NiTransform liveHandWorldAtGrab{};
        RE::NiTransform handBodyWorldAtGrab{};
        RE::NiTransform objectNodeWorldAtGrab{};
        RE::NiTransform desiredObjectWorldAtGrab{};
        RE::NiTransform currentRawDesiredObjectWorld{};
        RE::NiTransform currentConstraintDesiredObjectWorld{};
        RE::NiTransform currentConstraintDesiredBodyWorld{};
        RE::NiTransform rawHandSpace{};
        RE::NiTransform constraintHandSpace{};
        RE::NiTransform handBodyToRawHandAtGrab{};
        RE::NiTransform bodyLocal{};
        RE::NiTransform higgsReverseTargetWorld{};
        RE::NiTransform constraintReverseTargetWorld{};
        RE::NiTransform rockVisualTargetWorld{};
        RE::NiTransform adjustedHandWorld{};

        RE::NiPoint3 pivotAWorld{};
        RE::NiPoint3 pivotBWorld{};
        RE::NiPoint3 pivotDeltaWorld{};
        RE::NiPoint3 constraintTransformBLocalGame{};
        RE::NiPoint3 desiredTransformBLocalGame{};
        PointPairDelta<RE::NiPoint3> transformBLocalDelta{};

        TransformDelta rawToHandBody{};
        TransformDelta heldNodeToDesiredObjectAtGrab{};
        TransformDelta heldNodeToRawDesiredObject{};
        TransformDelta heldNodeToConstraintDesiredObject{};
        TransformDelta bodyDerivedNodeToHeldNode{};
        TransformDelta heldBodyToConstraintDesiredBody{};
        TransformDelta rawToHiggsReverse{};
        TransformDelta rawToConstraintReverse{};
        TransformDelta handBodyToConstraintReverse{};
        TransformDelta rawToRockVisual{};
        TransformDelta rawToAdjusted{};
        TransformDelta higgsToConstraintReverse{};
        TransformDelta higgsToRockVisual{};
        AxisAlignmentDots rawToHiggsReverseAxes{};
        AxisAlignmentDots rawToConstraintReverseAxes{};
        AxisAlignmentDots rawToRockVisualAxes{};

        float pivotErrorGameUnits = 0.0f;
        float targetColumnsToConstraintInverseDegrees = 0.0f;
        float targetRowsToConstraintInverseDegrees = 0.0f;
        float targetColumnsToConstraintForwardDegrees = 0.0f;
        float targetColumnsToTransformBDegrees = 0.0f;
        float angularMotorTau = 0.0f;
        float angularMotorDamping = 0.0f;
        float angularMotorMaxForce = 0.0f;
        float linearMotorTau = 0.0f;
        float linearMotorMaxForce = 0.0f;
        float heldBodyMass = 0.0f;
        bool valid = false;
        bool isLeft = false;
        bool visualAuthorityEnabled = false;
        bool hasHandBodyWorld = false;
        bool hasHeldBodyWorld = false;
        bool hasHeldNodeWorld = false;
        bool hasHeldBodyDerivedNodeWorld = false;
        bool hasGrabStartFrames = false;
        bool hasHiggsReverseTarget = false;
        bool hasConstraintReverseTarget = false;
        bool hasRockVisualTarget = false;
        bool rockVisualTargetUsedSurfaceFrame = false;
        bool hasAdjustedHandWorld = false;
        bool hasConstraintAngularTelemetry = false;
        bool ragdollMotorEnabled = false;
    };

    template <class Vector>
    inline float vectorLength(const Vector& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    template <class Vector>
    inline PointPairDelta<Vector> measurePointPair(const Vector& a, const Vector& b)
    {
        PointPairDelta<Vector> result{};
        result.delta = Vector{ a.x - b.x, a.y - b.y, a.z - b.z };
        result.distance = vectorLength(result.delta);
        return result;
    }

    template <class Transform>
    inline Transform computeCapturedObjectFromLiveHand(const Transform& liveHandWorld, const Transform& capturedObjectHandSpace)
    {
        return transform_math::composeTransforms(liveHandWorld, capturedObjectHandSpace);
    }

    template <class Transform>
    inline Transform computeCurrentDesiredObjectFromFrame(const Transform& frameWorld, const Transform& capturedObjectFrameSpace)
    {
        return transform_math::composeTransforms(frameWorld, capturedObjectFrameSpace);
    }

    template <class Transform>
    inline Transform computeCurrentDesiredBodyFromHandBody(const Transform& handBodyWorld, const Transform& constraintHandSpace, const Transform& bodyLocal)
    {
        return transform_math::composeTransforms(handBodyWorld, grab_frame_math::desiredBodyInHandBodySpace(constraintHandSpace, bodyLocal));
    }

    template <class Transform>
    inline Transform computeReverseTargetFromCapturedSpace(const Transform& heldNodeWorld, const Transform& capturedObjectHandSpace)
    {
        return grab_frame_math::computeVisualHandFromHeldNode(heldNodeWorld, capturedObjectHandSpace);
    }

    template <class Transform>
    inline Transform computeHiggsReverseTarget(const Transform& heldNodeWorld, const Transform& rawHandSpace)
    {
        return computeReverseTargetFromCapturedSpace(heldNodeWorld, rawHandSpace);
    }

    template <class Transform>
    inline Transform computeConstraintReverseTarget(const Transform& heldNodeWorld, const Transform& constraintHandSpace)
    {
        return computeReverseTargetFromCapturedSpace(heldNodeWorld, constraintHandSpace);
    }

    template <class Matrix, class Vector>
    inline Vector matrixColumn(const Matrix& matrix, int column)
    {
        column = (std::max)(0, (std::min)(column, 2));
        return Vector{ matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column] };
    }

    template <class Vector>
    inline float directionDeltaDegrees(const Vector& a, const Vector& b)
    {
        const float aLength = vectorLength(a);
        const float bLength = vectorLength(b);
        if (aLength <= 1.0e-8f || bLength <= 1.0e-8f) {
            return 0.0f;
        }

        const float dot = (a.x * b.x + a.y * b.y + a.z * b.z) / (aLength * bLength);
        return std::acos((std::clamp)(dot, -1.0f, 1.0f)) * (180.0f / 3.14159265358979323846f);
    }

    template <class Vector>
    inline float directionDot(const Vector& a, const Vector& b)
    {
        const float aLength = vectorLength(a);
        const float bLength = vectorLength(b);
        if (aLength <= 1.0e-8f || bLength <= 1.0e-8f) {
            return 0.0f;
        }

        return (std::clamp)((a.x * b.x + a.y * b.y + a.z * b.z) / (aLength * bLength), -1.0f, 1.0f);
    }

    template <class Matrix>
    inline AxisAlignmentDots axisAlignmentDots(const Matrix& a, const Matrix& b)
    {
        using Vector = RE::NiPoint3;
        AxisAlignmentDots result{};
        result.x = directionDot(matrixColumn<Matrix, Vector>(a, 0), matrixColumn<Matrix, Vector>(b, 0));
        result.y = directionDot(matrixColumn<Matrix, Vector>(a, 1), matrixColumn<Matrix, Vector>(b, 1));
        result.z = directionDot(matrixColumn<Matrix, Vector>(a, 2), matrixColumn<Matrix, Vector>(b, 2));
        return result;
    }

    template <class Matrix>
    inline float rotationDeltaDegrees(const Matrix& a, const Matrix& b)
    {
        using Vector = RE::NiPoint3;
        const float x = directionDeltaDegrees(matrixColumn<Matrix, Vector>(a, 0), matrixColumn<Matrix, Vector>(b, 0));
        const float y = directionDeltaDegrees(matrixColumn<Matrix, Vector>(a, 1), matrixColumn<Matrix, Vector>(b, 1));
        const float z = directionDeltaDegrees(matrixColumn<Matrix, Vector>(a, 2), matrixColumn<Matrix, Vector>(b, 2));
        return (std::max)(x, (std::max)(y, z));
    }

    template <class Transform>
    inline TransformDelta measureTransformDelta(const Transform& a, const Transform& b)
    {
        TransformDelta result{};
        result.positionGameUnits = measurePointPair(a.translate, b.translate).distance;
        result.rotationDegrees = rotationDeltaDegrees(a.rotate, b.rotate);
        return result;
    }

    template <class Transform>
    inline TransformDelta measureHeldNodeVsDesiredObject(const Transform& heldNodeWorld, const Transform& desiredObjectWorldAtGrab)
    {
        return measureTransformDelta(heldNodeWorld, desiredObjectWorldAtGrab);
    }

    inline const char* handLabel(bool isLeft)
    {
        return isLeft ? "L" : "R";
    }

    inline std::string formatStampPrefix(const FrameStamp& stamp, bool isLeft, const char* phase)
    {
        char buffer[128]{};
        std::snprintf(buffer,
            sizeof(buffer),
            "GTEL session=%u frame=%llu tickMs=%llu hand=%s phase=%s",
            stamp.session,
            static_cast<unsigned long long>(stamp.frame),
            static_cast<unsigned long long>(stamp.tickMs),
            handLabel(isLeft),
            phase ? phase : "unknown");
        return std::string(buffer);
    }
}
