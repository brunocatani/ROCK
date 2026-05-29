#pragma once

#include "RockConfig.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/hand/HandSkeleton.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cmath>

namespace rock::custom_oga
{
    inline constexpr float kPalmDepthGameUnits = 0.75f;
    inline constexpr float kPalmOriginDepthDivisor = 3.0f;

    struct Frame
    {
        RE::NiTransform world = transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiTransform handLocalTransform = transform_math::makeIdentityTransform<RE::NiTransform>();
        RE::NiPoint3 handPositionWorld{};
        RE::NiPoint3 fingerCenterWorld{};
        RE::NiPoint3 palmPlaneCenterWorld{};
        RE::NiPoint3 crossPalmDirectionWorld{};
        RE::NiPoint3 xAxisWorld{};
        RE::NiPoint3 yAxisWorld{};
        RE::NiPoint3 zAxisWorld{};
        bool valid = false;
    };

    inline RE::NiPoint3 add(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    inline RE::NiPoint3 sub(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    inline RE::NiPoint3 mul(const RE::NiPoint3& value, float scalar)
    {
        return RE::NiPoint3{ value.x * scalar, value.y * scalar, value.z * scalar };
    }

    inline float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    inline float lengthSquared(const RE::NiPoint3& value)
    {
        return dot(value, value);
    }

    inline bool finitePoint(const RE::NiPoint3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    inline RE::NiPoint3 normalizeOr(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lenSq = lengthSquared(value);
        if (!std::isfinite(lenSq) || lenSq <= 1.0e-8f) {
            return fallback;
        }

        const float invLen = 1.0f / std::sqrt(lenSq);
        return mul(value, invLen);
    }

    inline RE::NiPoint3 projectOntoPlane(const RE::NiPoint3& value, const RE::NiPoint3& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    inline RE::NiMatrix3 matrixFromAxes(const RE::NiPoint3& xAxis, const RE::NiPoint3& yAxis, const RE::NiPoint3& zAxis)
    {
        RE::NiMatrix3 matrix{};
        matrix.entry[0][0] = xAxis.x;
        matrix.entry[1][0] = xAxis.y;
        matrix.entry[2][0] = xAxis.z;
        matrix.entry[0][1] = yAxis.x;
        matrix.entry[1][1] = yAxis.y;
        matrix.entry[2][1] = yAxis.z;
        matrix.entry[0][2] = zAxis.x;
        matrix.entry[1][2] = zAxis.y;
        matrix.entry[2][2] = zAxis.z;
        return matrix;
    }

    inline RE::NiTransform applyHandLocalTransform(const RE::NiTransform& handWorld, const RE::NiTransform& handLocalTransform)
    {
        return transform_math::composeTransforms(handWorld, handLocalTransform);
    }

    inline RE::NiPoint3 computeOffsetLocalGame(bool isLeft)
    {
        return isLeft ? g_rockConfig.rockLeftCustomOGAOffsetGameUnits : g_rockConfig.rockRightCustomOGAOffsetGameUnits;
    }

    inline bool build(
        const RE::NiTransform& handWorld,
        const RE::NiTransform& rollAuthorityWorld,
        const std::array<RE::NiPoint3, 5>& fingerBasesWorld,
        const RE::NiPoint3& localOffsetGameUnits,
        Frame& outFrame)
    {
        outFrame = {};
        if (!finitePoint(handWorld.translate)) {
            return false;
        }

        const RE::NiPoint3 kFallbackX{ 1.0f, 0.0f, 0.0f };
        const RE::NiPoint3 kFallbackY{ 0.0f, 1.0f, 0.0f };
        const RE::NiPoint3 kFallbackZ{ 0.0f, 0.0f, 1.0f };

        RE::NiPoint3 fingerCenter{};
        for (const auto& fingerBase : fingerBasesWorld) {
            if (!finitePoint(fingerBase)) {
                return false;
            }
            fingerCenter = add(fingerCenter, fingerBase);
        }
        fingerCenter = mul(fingerCenter, 1.0f / static_cast<float>(fingerBasesWorld.size()));

        RE::NiPoint3 palmCenter = handWorld.translate;
        for (const auto& fingerBase : fingerBasesWorld) {
            palmCenter = add(palmCenter, fingerBase);
        }
        palmCenter = mul(palmCenter, 1.0f / static_cast<float>(fingerBasesWorld.size() + 1));

        const RE::NiPoint3 palmDepthAxisWorld = normalizeOr(
            transform_math::rotateLocalVectorToWorld(rollAuthorityWorld.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
            kFallbackY);
        const float depthOffset = dot(sub(palmCenter, handWorld.translate), palmDepthAxisWorld);
        const RE::NiPoint3 palmPlaneCenterWorld = sub(palmCenter, mul(palmDepthAxisWorld, depthOffset));
        const float palmOriginDepthOffset = -std::fabs(kPalmDepthGameUnits) / kPalmOriginDepthDivisor;

        outFrame.world = handWorld;
        outFrame.world.translate = add(palmPlaneCenterWorld, mul(palmDepthAxisWorld, palmOriginDepthOffset));
        outFrame.world.rotate = rollAuthorityWorld.rotate;
        outFrame.world.scale = 1.0f;
        if (finitePoint(localOffsetGameUnits)) {
            outFrame.world.translate = add(outFrame.world.translate, transform_math::localVectorToWorld(outFrame.world, localOffsetGameUnits));
        }
        outFrame.handLocalTransform = transform_math::composeTransforms(transform_math::invertTransform(handWorld), outFrame.world);
        const RE::NiPoint3 xAxisWorld = normalizeOr(transform_math::rotateLocalVectorToWorld(outFrame.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }), kFallbackX);
        const RE::NiPoint3 yAxisWorld = normalizeOr(transform_math::rotateLocalVectorToWorld(outFrame.world.rotate, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), kFallbackY);
        const RE::NiPoint3 zAxisWorld = normalizeOr(transform_math::rotateLocalVectorToWorld(outFrame.world.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }), kFallbackZ);
        const RE::NiPoint3 crossPalmDirectionWorld = zAxisWorld;
        outFrame.handPositionWorld = handWorld.translate;
        outFrame.fingerCenterWorld = fingerCenter;
        outFrame.palmPlaneCenterWorld = palmPlaneCenterWorld;
        outFrame.crossPalmDirectionWorld = crossPalmDirectionWorld;
        outFrame.xAxisWorld = xAxisWorld;
        outFrame.yAxisWorld = yAxisWorld;
        outFrame.zAxisWorld = zAxisWorld;
        outFrame.valid = true;
        return true;
    }

    inline bool resolveLive(
        bool isLeft,
        const RE::NiTransform& handWorld,
        const RE::NiTransform& rollAuthorityWorld,
        Frame& outFrame)
    {
        outFrame = {};

        root_flattened_finger_skeleton_runtime::Snapshot snapshot{};
        if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, snapshot) || !snapshot.valid) {
            return false;
        }

        std::array<RE::NiPoint3, 5> fingerBasesWorld{};
        for (std::size_t finger = 0; finger < fingerBasesWorld.size(); ++finger) {
            if (!snapshot.fingers[finger].valid) {
                return false;
            }
            fingerBasesWorld[finger] = snapshot.fingers[finger].points[0];
        }

        return build(handWorld, rollAuthorityWorld, fingerBasesWorld, computeOffsetLocalGame(isLeft), outFrame);
    }
}
