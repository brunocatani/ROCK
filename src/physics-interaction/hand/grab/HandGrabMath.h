#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/collision/HandColliderTypes.h"
#include "physics-interaction/native/havok/HavokOffsets.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace rock::hand_grab_detail
{
    inline RE::NiPoint3 getMatrixColumn(const RE::NiMatrix3& matrix, int column) { return RE::NiPoint3(matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column]); }
    
    inline RE::NiPoint3 getMatrixRow(const RE::NiMatrix3& matrix, int row) { return RE::NiPoint3(matrix.entry[row][0], matrix.entry[row][1], matrix.entry[row][2]); }
    
    inline RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (lengthSquared <= 1.0e-8f) {
            return RE::NiPoint3{};
        }
    
        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
    }
    
    inline float lengthSquared(const RE::NiPoint3& value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    inline float pointDistanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const RE::NiPoint3 delta = a - b;
        return std::sqrt(lengthSquared(delta));
    }
    
    inline RE::NiPoint3 crossProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }
    
    inline float dotProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }
    
    inline float maxAbsDelta(const std::array<float, 12>& lhs, const std::array<float, 12>& rhs)
    {
        float maxDelta = 0.0f;
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            maxDelta = (std::max)(maxDelta, std::fabs(lhs[i] - rhs[i]));
        }
        return maxDelta;
    }
    
    inline RE::NiPoint3 scalePoint(const RE::NiPoint3& value, float scalar)
    {
        return RE::NiPoint3{ value.x * scalar, value.y * scalar, value.z * scalar };
    }
    
    inline RE::NiPoint3 projectOntoPlane(const RE::NiPoint3& value, const RE::NiPoint3& normal)
    {
        const RE::NiPoint3 normalizedNormal = normalizeOrZero(normal);
        if (lengthSquared(normalizedNormal) <= 0.000001f) {
            return {};
        }
        return value - scalePoint(normalizedNormal, dotProduct(value, normalizedNormal));
    }
    
    inline RE::NiPoint3 stablePerpendicularAxis(const RE::NiPoint3& normal)
    {
        const RE::NiPoint3 normalizedNormal = normalizeOrZero(normal);
        if (lengthSquared(normalizedNormal) <= 0.000001f) {
            return {};
        }
    
        const RE::NiPoint3 reference = std::fabs(normalizedNormal.z) < 0.75f ? RE::NiPoint3{ 0.0f, 0.0f, 1.0f } : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 axis = normalizeOrZero(crossProduct(reference, normalizedNormal));
        if (lengthSquared(axis) <= 0.000001f) {
            axis = normalizeOrZero(crossProduct(RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, normalizedNormal));
        }
        return axis;
    }
    
    inline RE::NiPoint3 gamePointToHavokPoint(const RE::NiPoint3& value)
    {
        const float scale = physics_scale::gameToHavok();
        return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
    }
    
    inline RE::NiMatrix3 storedRotationFromConventionalRows(const RE::NiPoint3 rows[3])
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = rows[0].x;
        result.entry[0][1] = rows[1].x;
        result.entry[0][2] = rows[2].x;
        result.entry[1][0] = rows[0].y;
        result.entry[1][1] = rows[1].y;
        result.entry[1][2] = rows[2].y;
        result.entry[2][0] = rows[0].z;
        result.entry[2][1] = rows[1].z;
        result.entry[2][2] = rows[2].z;
        return result;
    }
    
    inline RE::NiMatrix3 axisAngleStored(const RE::NiPoint3& axisRaw, float angle)
    {
        const RE::NiPoint3 axis = normalizeOrZero(axisRaw);
        if (lengthSquared(axis) <= 0.000001f || !std::isfinite(angle)) {
            return transform_math::makeIdentityRotation<RE::NiMatrix3>();
        }
    
        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float cosTheta = std::cos(angle);
        const float sinTheta = std::sin(angle);
        const float oneMinusCos = 1.0f - cosTheta;
    
        const RE::NiPoint3 rows[3]{
            RE::NiPoint3{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            RE::NiPoint3{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            RE::NiPoint3{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };
        return storedRotationFromConventionalRows(rows);
    }
    
    inline RE::NiMatrix3 applyWorldRotationToStoredBasis(const RE::NiMatrix3& worldRotationStored, const RE::NiMatrix3& baseRotation)
    {
        RE::NiMatrix3 result{};
        for (int axis = 0; axis < 3; ++axis) {
            const RE::NiPoint3 basis{ baseRotation.entry[axis][0], baseRotation.entry[axis][1], baseRotation.entry[axis][2] };
            const RE::NiPoint3 rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis);
            result.entry[axis][0] = rotated.x;
            result.entry[axis][1] = rotated.y;
            result.entry[axis][2] = rotated.z;
        }
        return result;
    }
    
    inline RE::NiPoint3 angularVelocityFromRotationDelta(const RE::NiMatrix3& previous, const RE::NiMatrix3& current, float deltaTime)
    {
        return held_object_physics_math::angularVelocityFromRotationDelta<RE::NiMatrix3, RE::NiPoint3>(previous, current, deltaTime);
    }
    
    inline RE::NiTransform invertTransform(const RE::NiTransform& transform) { return transform_math::invertTransform(transform); }
    
    inline RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child) { return transform_math::composeTransforms(parent, child); }
    
    /*
     * A grab stores the held node as a body-local offset, so the node world comes
     * back by undoing that offset against the live body world.
     */
    inline RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld, const RE::NiTransform& bodyLocalTransform)
    {
        return multiplyTransforms(bodyWorld, invertTransform(bodyLocalTransform));
    }
    
    inline float translationDeltaGameUnits(const RE::NiTransform& a, const RE::NiTransform& b)
    {
        const RE::NiPoint3 delta = a.translate - b.translate;
        return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }
    
    inline float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
    {
        const RE::NiMatrix3 delta = a.Transpose() * b;
        float cosTheta = (delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2] - 1.0f) * 0.5f;
        if (cosTheta < -1.0f) {
            cosTheta = -1.0f;
        } else if (cosTheta > 1.0f) {
            cosTheta = 1.0f;
        }
        return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
    }
    
    inline float axisDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
        const float lenA = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        const float lenB = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
        if (lenA < 0.0001f || lenB < 0.0001f) {
            return -1.0f;
        }
    
        float cosTheta = dot / (lenA * lenB);
        if (cosTheta < -1.0f) {
            cosTheta = -1.0f;
        } else if (cosTheta > 1.0f) {
            cosTheta = 1.0f;
        }
        return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
    }
    
    inline float matrixDeterminant(const RE::NiMatrix3& matrix)
    {
        return matrix.entry[0][0] * (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
               matrix.entry[0][1] * (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
               matrix.entry[0][2] * (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
    }
    
    struct GrabPalmBasisDelta
    {
        float rotationDegrees = -1.0f;
        float xAxisDegrees = -1.0f;
        float yAxisDegrees = -1.0f;
        float zAxisDegrees = -1.0f;
        float rawDeterminant = 0.0f;
        float proxyDeterminant = 0.0f;
    };
    
    inline RE::NiPoint3 frameAxisWorld(const RE::NiTransform& transform, const RE::NiPoint3& localAxis)
    {
        return normalizeOrZero(transform_math::localVectorToWorld(transform, localAxis));
    }
    
    inline RE::NiPoint3 generatedFrameAxisWorld(const RE::NiTransform& transform, const RE::NiPoint3& localAxis)
    {
        return normalizeOrZero(hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(transform, localAxis));
    }
    
    inline GrabPalmBasisDelta computeGrabPalmBasisDelta(const RE::NiTransform& rawHandWorld, const RE::NiTransform& proxyWorld)
    {
        GrabPalmBasisDelta result{};
        result.rotationDegrees = rotationDeltaDegrees(rawHandWorld.rotate, proxyWorld.rotate);
        result.xAxisDegrees = axisDeltaDegrees(
            frameAxisWorld(rawHandWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
            generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }));
        result.yAxisDegrees = axisDeltaDegrees(
            frameAxisWorld(rawHandWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
            generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }));
        result.zAxisDegrees = axisDeltaDegrees(
            frameAxisWorld(rawHandWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
            generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }));
        result.rawDeterminant = matrixDeterminant(rawHandWorld.rotate);
        result.proxyDeterminant = matrixDeterminant(proxyWorld.rotate);
        return result;
    }
    
    inline float max3(float a, float b, float c) { return (std::max)((std::max)(a, b), c); }
    
    inline float maxColumnAxisDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
    {
        return max3(
            axisDeltaDegrees(getMatrixColumn(a, 0), getMatrixColumn(b, 0)),
            axisDeltaDegrees(getMatrixColumn(a, 1), getMatrixColumn(b, 1)),
            axisDeltaDegrees(getMatrixColumn(a, 2), getMatrixColumn(b, 2)));
    }
    
    inline RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix)
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = hkMatrix[0];
        result.entry[1][0] = hkMatrix[1];
        result.entry[2][0] = hkMatrix[2];
        result.entry[0][1] = hkMatrix[4];
        result.entry[1][1] = hkMatrix[5];
        result.entry[2][1] = hkMatrix[6];
        result.entry[0][2] = hkMatrix[8];
        result.entry[1][2] = hkMatrix[9];
        result.entry[2][2] = hkMatrix[10];
        return result;
    }
    
    inline RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix)
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = hkMatrix[0];
        result.entry[0][1] = hkMatrix[1];
        result.entry[0][2] = hkMatrix[2];
        result.entry[1][0] = hkMatrix[4];
        result.entry[1][1] = hkMatrix[5];
        result.entry[1][2] = hkMatrix[6];
        result.entry[2][0] = hkMatrix[8];
        result.entry[2][1] = hkMatrix[9];
        result.entry[2][2] = hkMatrix[10];
        return result;
    }
    
    inline RE::NiTransform rotationOnlyTransform(const RE::NiMatrix3& rotation)
    {
        RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
        result.rotate = rotation;
        return result;
    }
    
    inline RE::NiMatrix3 frameToFrameRotation(const RE::NiMatrix3& fromWorldRotation, const RE::NiMatrix3& toWorldRotation)
    {
        const RE::NiTransform fromWorld = rotationOnlyTransform(fromWorldRotation);
        const RE::NiTransform toWorld = rotationOnlyTransform(toWorldRotation);
        return transform_math::composeTransforms(transform_math::invertTransform(fromWorld), toWorld).rotate;
    }
    
    inline RE::NiTransform makeIdentityTransform()
    {
        return transform_math::makeIdentityTransform<RE::NiTransform>();
    }
    
    inline std::vector<GrabLocalTriangle> cacheTrianglesInLocalSpace(const std::vector<TriangleData>& worldTriangles, const RE::NiTransform& nodeWorld)
    {
        std::vector<GrabLocalTriangle> localTriangles;
        localTriangles.reserve(worldTriangles.size());
        for (const auto& triangle : worldTriangles) {
            localTriangles.push_back(GrabLocalTriangle{
                transform_math::worldPointToLocal(nodeWorld, triangle.v0),
                transform_math::worldPointToLocal(nodeWorld, triangle.v1),
                transform_math::worldPointToLocal(nodeWorld, triangle.v2),
            });
        }
        return localTriangles;
    }
    
    constexpr std::size_t kMaxGrabRuntimeSurfaceContactTriangles = 2048;
    constexpr std::size_t kMaxGrabRuntimeFingerPoseTriangles = 2048;
    
    inline float triangleDistanceSquaredToPoint(const TriangleData& triangle, const RE::NiPoint3& point)
    {
        const RE::NiPoint3 centroid = (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
        return (std::min)({
            lengthSquared(centroid - point),
            lengthSquared(triangle.v0 - point),
            lengthSquared(triangle.v1 - point),
            lengthSquared(triangle.v2 - point),
        });
    }
    
    struct RankedGrabTriangle
    {
        float distanceSquared = 0.0f;
        std::size_t index = 0;
    };
    
    inline bool rankedGrabTriangleLess(const RankedGrabTriangle& lhs, const RankedGrabTriangle& rhs)
    {
        if (lhs.distanceSquared == rhs.distanceSquared) {
            return lhs.index < rhs.index;
        }
        return lhs.distanceSquared < rhs.distanceSquared;
    }
    
    /*
     * Nearest-N triangle selection. Big meshes make the contact scan expensive, so
     * the callers keep only the triangles closest to the point they work around.
     * triangleOf maps a stored element to the geometry to measure: the surface path
     * carries per-triangle data around its triangle, the finger-pose path does not.
     */
    template <typename TriangleElement, typename TriangleOf>
    inline std::vector<TriangleElement> selectNearestTriangles(
        const std::vector<TriangleElement>& sourceTriangles,
        const RE::NiPoint3& centerWorld,
        std::size_t maxTriangles,
        TriangleOf triangleOf)
    {
        // Fail open: an unusable center or an already-small set is passed through.
        if (sourceTriangles.size() <= maxTriangles || maxTriangles == 0 || !grab_three_phase::isFinite(centerWorld)) {
            return sourceTriangles;
        }
    
        std::vector<RankedGrabTriangle> rankedTriangles;
        rankedTriangles.reserve(sourceTriangles.size());
        for (std::size_t i = 0; i < sourceTriangles.size(); ++i) {
            rankedTriangles.push_back(RankedGrabTriangle{
                triangleDistanceSquaredToPoint(triangleOf(sourceTriangles[i]), centerWorld),
                i,
            });
        }
    
        // nth_element puts the nearest maxTriangles first; the sort after it only
        // orders that prefix, so the cost stays linear in the discarded tail.
        const auto selectedEnd = rankedTriangles.begin() + maxTriangles;
        std::nth_element(rankedTriangles.begin(), selectedEnd, rankedTriangles.end(), rankedGrabTriangleLess);
        std::sort(rankedTriangles.begin(), selectedEnd, rankedGrabTriangleLess);
    
        std::vector<TriangleElement> selectedTriangles;
        selectedTriangles.reserve(maxTriangles);
        for (auto it = rankedTriangles.begin(); it != selectedEnd; ++it) {
            selectedTriangles.push_back(sourceTriangles[it->index]);
        }
        return selectedTriangles;
    }
    
    inline std::vector<GrabSurfaceTriangleData> selectNearestGrabSurfaceTriangles(
        const std::vector<GrabSurfaceTriangleData>& sourceTriangles,
        const RE::NiPoint3& centerWorld,
        std::size_t maxTriangles)
    {
        return selectNearestTriangles(sourceTriangles, centerWorld, maxTriangles,
            [](const GrabSurfaceTriangleData& element) -> const TriangleData& { return element.triangle; });
    }
    
    inline std::vector<TriangleData> selectNearestGrabFingerPoseTriangles(
        const std::vector<TriangleData>& sourceTriangles,
        const RE::NiPoint3& centerWorld,
        std::size_t maxTriangles)
    {
        return selectNearestTriangles(sourceTriangles, centerWorld, maxTriangles,
            [](const TriangleData& element) -> const TriangleData& { return element; });
    }
    inline RE::NiTransform reconstructBodyWorldFromProxyInBody(const RE::NiTransform& proxyWorld,
        const RE::NiMatrix3& proxyInBodyRotation,
        const RE::NiPoint3& transformBLocalGame,
        const RE::NiPoint3& pivotAProxyLocalGame)
    {
        RE::NiTransform proxyInBody = makeIdentityTransform();
        proxyInBody.rotate = proxyInBodyRotation;
        const RE::NiPoint3 rotatedPivotA = transform_math::localVectorToWorld(proxyInBody, pivotAProxyLocalGame);
        proxyInBody.translate = transformBLocalGame - rotatedPivotA;
        const RE::NiTransform bodyInProxy = transform_math::invertTransform(proxyInBody);
        return grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, bodyInProxy);
    }

    inline RE::NiTransform reconstructSolverEffectiveBodyWorld(const RE::NiTransform& proxyWorld,
        const RE::NiMatrix3& transformARotation,
        const RE::NiMatrix3& transformBRotation,
        const RE::NiMatrix3& targetBRcaRotation,
        const RE::NiPoint3& transformBLocalGame,
        const RE::NiPoint3& anchorAWorld,
        float bodyScale)
    {
        const RE::NiMatrix3 constraintAWorldRotation =
            transform_math::composeTransforms(proxyWorld, rotationOnlyTransform(transformARotation)).rotate;
        const RE::NiMatrix3 desiredConstraintBWorldRotation =
            transform_math::composeTransforms(
                rotationOnlyTransform(constraintAWorldRotation),
                rotationOnlyTransform(transform_math::transposeRotation(targetBRcaRotation)))
                .rotate;
        const RE::NiMatrix3 desiredBodyRotation =
            transform_math::composeTransforms(
                rotationOnlyTransform(desiredConstraintBWorldRotation),
                rotationOnlyTransform(transform_math::transposeRotation(transformBRotation)))
                .rotate;

        RE::NiTransform result = makeIdentityTransform();
        result.rotate = desiredBodyRotation;
        result.scale = std::isfinite(bodyScale) && bodyScale > 0.0f ? bodyScale : 1.0f;
        result.translate = anchorAWorld - transform_math::localVectorToWorld(result, transformBLocalGame);
        return result;
    }

    struct GrabConstraintAtomDiagnostics
    {
        RE::NiMatrix3 transformAColumns{};
        RE::NiMatrix3 transformBColumns{};
        RE::NiMatrix3 targetRows{};
        RE::NiMatrix3 targetColumns{};
        RE::NiPoint3 transformBTranslationGame{};
        bool ragdollMotorEnabled = false;
    };

    inline GrabConstraintAtomDiagnostics decodeGrabConstraintAtoms(
        const float* transformARotation,
        const float* transformBRotation,
        const float* transformBTranslation,
        const float* targetBRca,
        bool ragdollMotorEnabled)
    {
        GrabConstraintAtomDiagnostics result{};
        result.transformAColumns = transformARotation ? matrixFromHkColumns(transformARotation) : makeIdentityTransform().rotate;
        result.transformBColumns = matrixFromHkColumns(transformBRotation);
        result.targetRows = matrixFromHkRows(targetBRca);
        result.targetColumns = matrixFromHkColumns(targetBRca);
        result.transformBTranslationGame = RE::NiPoint3{
            transformBTranslation[0] * havokToGameScale(),
            transformBTranslation[1] * havokToGameScale(),
            transformBTranslation[2] * havokToGameScale(),
        };
        result.ragdollMotorEnabled = ragdollMotorEnabled;
        return result;
    }

    inline GrabConstraintAtomDiagnostics decodeGrabConstraintAtoms(const void* data)
    {
        const auto* constraintData = static_cast<const char*>(data);
        return decodeGrabConstraintAtoms(
            reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_A_COL0),
            reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_COL0),
            reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS),
            reinterpret_cast<const float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA),
            *(constraintData + ATOM_RAGDOLL_MOT + 0x02) != 0);
    }

    struct GrabConstraintRelationDiagnostics
    {
        RE::NiTransform proxyInBodyBeforeTargetWrite{};
        RE::NiPoint3 relationTransformBLocalGame{};
        RE::NiTransform relationInverseBodyWorld{};
        RE::NiTransform atomRowsBodyWorld{};
        float targetToHiggsRelationDegrees = 0.0f;
        float transformBFrozenDeltaDegrees = 0.0f;
        float transformBRelationDeltaGameUnits = 0.0f;
    };

    inline GrabConstraintRelationDiagnostics buildGrabConstraintRelationDiagnostics(
        const GrabConstraintAtomDiagnostics& atoms,
        const RE::NiTransform& proxyWorld,
        const RE::NiTransform& desiredBodyInProxy,
        const RE::NiPoint3& pivotAProxyLocalGame)
    {
        GrabConstraintRelationDiagnostics result{};
        result.proxyInBodyBeforeTargetWrite =
            grab_constraint_math::proxyInBodyFromBodyInProxy(desiredBodyInProxy);
        result.relationTransformBLocalGame =
            grab_constraint_math::computeHiggsTransformBTranslationGameFromProxyInBody(
                result.proxyInBodyBeforeTargetWrite,
                pivotAProxyLocalGame);
        result.relationInverseBodyWorld = reconstructBodyWorldFromProxyInBody(
            proxyWorld,
            result.proxyInBodyBeforeTargetWrite.rotate,
            result.relationTransformBLocalGame,
            pivotAProxyLocalGame);
        result.atomRowsBodyWorld = reconstructBodyWorldFromProxyInBody(
            proxyWorld,
            atoms.targetRows,
            atoms.transformBTranslationGame,
            pivotAProxyLocalGame);

        const RE::NiTransform desiredBodyToProxy = invertTransform(desiredBodyInProxy);
        result.targetToHiggsRelationDegrees =
            rotationDeltaDegrees(atoms.targetRows, result.proxyInBodyBeforeTargetWrite.rotate);
        result.transformBFrozenDeltaDegrees =
            rotationDeltaDegrees(atoms.transformBColumns, desiredBodyToProxy.rotate);
        result.transformBRelationDeltaGameUnits =
            pointDistanceGameUnits(atoms.transformBTranslationGame, result.relationTransformBLocalGame);
        return result;
    }
    inline RE::NiPoint3 rotationCorrectionAxisWorld(const RE::NiMatrix3& current, const RE::NiMatrix3& target)
    {
        return normalizeOrZero(angularVelocityFromRotationDelta(current, target, 1.0f));
    }

    inline float vectorMagnitude(const RE::NiPoint3& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    inline bool tryGetGrabAuthorityBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
    {
        /*
         * Dynamic grab object-side state is measured from the hknp BODY slot.
         * The hand side is ROCK's hidden no-contact proxy. The held object's
         * contact pivot and visual node relation stay in BODY space; MOTION is
         * COM/weight/diagnostic data only and never grip authority.
         */
        outTransform = makeIdentityTransform();
        return tryGetBodyArrayWorldTransform(world, bodyId, outTransform);
    }
    
    
}
