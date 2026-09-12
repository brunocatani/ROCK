#pragma once

/*
 * Fixed-budget grab-seat refinement. The historical fitted six-DOF search
 * produced useful seats but was not suitable for the owner frame: it scanned
 * dense meshes hundreds of times and later ran again during held reacquire.
 * This policy keeps the one-sided fitted objective, but ranks one compile-time
 * rotation schedule against a fixed local triangle proxy. It has no iteration,
 * heap allocation, engine pointers, time source, logging, or held-state owner.
 */

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/GrabCore.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace rock::grab_pose_candidate_selector
{
    inline constexpr std::size_t kMaxProxyTriangles = 64;
    inline constexpr std::size_t kNearestProxyTriangles = 32;
    inline constexpr std::size_t kMaxHandCapsules = 19;
    inline constexpr std::size_t kMaxFingerTips = 5;

    struct RotationCandidate
    {
        std::uint8_t axisIndex = 0;
        float angleRadians = 0.0f;
        const char* name = "identity";
    };

    inline constexpr float kDegreesToRadians = 0.01745329252f;
    inline constexpr std::array<RotationCandidate, 13> kRotationCandidates{
        RotationCandidate{ 0, 0.0f, "identity" },
        RotationCandidate{ 0, 4.0f * kDegreesToRadians, "finger+4" },
        RotationCandidate{ 0, -4.0f * kDegreesToRadians, "finger-4" },
        RotationCandidate{ 0, 8.0f * kDegreesToRadians, "finger+8" },
        RotationCandidate{ 0, -8.0f * kDegreesToRadians, "finger-8" },
        RotationCandidate{ 1, 4.0f * kDegreesToRadians, "palm+4" },
        RotationCandidate{ 1, -4.0f * kDegreesToRadians, "palm-4" },
        RotationCandidate{ 1, 8.0f * kDegreesToRadians, "palm+8" },
        RotationCandidate{ 1, -8.0f * kDegreesToRadians, "palm-8" },
        RotationCandidate{ 2, 4.0f * kDegreesToRadians, "cross+4" },
        RotationCandidate{ 2, -4.0f * kDegreesToRadians, "cross-4" },
        RotationCandidate{ 2, 8.0f * kDegreesToRadians, "cross+8" },
        RotationCandidate{ 2, -8.0f * kDegreesToRadians, "cross-8" },
    };

    struct ProxyTriangle
    {
        GrabLocalTriangle triangle{};
        RE::NiPoint3 center{};
        float boundRadius = 0.0f;
        std::size_t sourceIndex = 0;
    };

    struct TriangleProxy
    {
        std::array<ProxyTriangle, kMaxProxyTriangles> triangles{};
        std::size_t count = 0;
        std::size_t sourceCount = 0;

        [[nodiscard]] bool valid() const noexcept { return count > 0; }
        [[nodiscard]] std::span<const ProxyTriangle> view() const noexcept
        {
            return std::span<const ProxyTriangle>(triangles.data(), count);
        }
    };

    struct HandCapsule
    {
        RE::NiPoint3 aWorld{};
        RE::NiPoint3 bWorld{};
        float radiusGameUnits = 0.0f;
    };

    struct HandModel
    {
        std::array<HandCapsule, kMaxHandCapsules> capsules{};
        std::array<RE::NiPoint3, kMaxFingerTips> tipCentersWorld{};
        std::array<float, kMaxFingerTips> tipRadiiGameUnits{};
        RE::NiPoint3 palmCenterWorld{};
        RE::NiPoint3 palmNormalWorld{};
        float palmRadiusGameUnits = 0.0f;
        std::size_t capsuleCount = 0;
        std::size_t tipCount = 0;
        bool valid = false;
    };

    struct TermValues
    {
        float touch = 0.0f;
        float behindPalm = 0.0f;
        float palmProximity = 0.0f;
        float overPenetration = 0.0f;
        float wrap = 0.0f;
        float rodAxis = 0.0f;
    };

    struct CandidateEvaluation
    {
        TermValues terms{};
        float objectiveScore = (std::numeric_limits<float>::max)();
        float totalScore = (std::numeric_limits<float>::max)();
        std::uint32_t exactDistanceTests = 0;
        bool valid = false;
    };

    struct SelectionDecision
    {
        CandidateEvaluation seed{};
        CandidateEvaluation selected{};
        std::size_t candidateIndex = 0;
        std::size_t evaluatedCandidateCount = 0;
        bool applied = false;
        const char* reason = "notEvaluated";
    };

    namespace detail
    {
        [[nodiscard]] inline RE::NiPoint3 add(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        }

        [[nodiscard]] inline RE::NiPoint3 subtract(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        [[nodiscard]] inline RE::NiPoint3 scale(const RE::NiPoint3& value, float scalar)
        {
            return RE::NiPoint3{ value.x * scalar, value.y * scalar, value.z * scalar };
        }

        [[nodiscard]] inline float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        [[nodiscard]] inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{
                lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.x * rhs.y - lhs.y * rhs.x,
            };
        }

        [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value)
        {
            return dot(value, value);
        }

        [[nodiscard]] inline RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
        {
            const float squared = lengthSquared(value);
            if (!std::isfinite(squared) || squared <= 0.000001f) {
                return RE::NiPoint3{};
            }
            return scale(value, 1.0f / std::sqrt(squared));
        }

        [[nodiscard]] inline bool finitePoint(const RE::NiPoint3& value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
        }

        [[nodiscard]] inline bool finiteTriangle(const GrabLocalTriangle& triangle)
        {
            return finitePoint(triangle.v0) && finitePoint(triangle.v1) && finitePoint(triangle.v2);
        }

        [[nodiscard]] inline bool finiteTransform(const RE::NiTransform& transform)
        {
            if (!finitePoint(transform.translate) || !std::isfinite(transform.scale) ||
                std::abs(transform.scale) <= 0.000001f) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        [[nodiscard]] inline RE::NiPoint3 closestPointOnTriangle(
            const RE::NiPoint3& point,
            const GrabLocalTriangle& triangle)
        {
            const RE::NiPoint3 ab = subtract(triangle.v1, triangle.v0);
            const RE::NiPoint3 ac = subtract(triangle.v2, triangle.v0);
            const RE::NiPoint3 ap = subtract(point, triangle.v0);
            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f) {
                return triangle.v0;
            }

            const RE::NiPoint3 bp = subtract(point, triangle.v1);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) {
                return triangle.v1;
            }

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
                return add(triangle.v0, scale(ab, d1 / (d1 - d3)));
            }

            const RE::NiPoint3 cp = subtract(point, triangle.v2);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6) {
                return triangle.v2;
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
                return add(triangle.v0, scale(ac, d2 / (d2 - d6)));
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
                const RE::NiPoint3 bc = subtract(triangle.v2, triangle.v1);
                return add(triangle.v1, scale(bc, (d4 - d3) / ((d4 - d3) + (d5 - d6))));
            }

            const float inverse = 1.0f / (va + vb + vc);
            return add(triangle.v0, add(scale(ab, vb * inverse), scale(ac, vc * inverse)));
        }

        [[nodiscard]] inline float pointTriangleDistanceSquared(
            const RE::NiPoint3& point,
            const GrabLocalTriangle& triangle)
        {
            return lengthSquared(subtract(point, closestPointOnTriangle(point, triangle)));
        }

        [[nodiscard]] inline float pointSegmentDistanceSquared(
            const RE::NiPoint3& point,
            const RE::NiPoint3& a,
            const RE::NiPoint3& b)
        {
            const RE::NiPoint3 axis = subtract(b, a);
            const float axisSquared = lengthSquared(axis);
            const float ratio = axisSquared > 0.000001f ?
                                    std::clamp(dot(subtract(point, a), axis) / axisSquared, 0.0f, 1.0f) :
                                    0.0f;
            return lengthSquared(subtract(point, add(a, scale(axis, ratio))));
        }

        [[nodiscard]] inline float segmentSegmentDistanceSquared(
            const RE::NiPoint3& p1,
            const RE::NiPoint3& q1,
            const RE::NiPoint3& p2,
            const RE::NiPoint3& q2)
        {
            constexpr float epsilon = 0.000001f;
            const RE::NiPoint3 d1 = subtract(q1, p1);
            const RE::NiPoint3 d2 = subtract(q2, p2);
            const RE::NiPoint3 r = subtract(p1, p2);
            const float a = dot(d1, d1);
            const float e = dot(d2, d2);
            const float f = dot(d2, r);
            float s = 0.0f;
            float t = 0.0f;
            if (a <= epsilon && e <= epsilon) {
                return lengthSquared(subtract(p1, p2));
            }
            if (a <= epsilon) {
                t = std::clamp(f / e, 0.0f, 1.0f);
            } else {
                const float c = dot(d1, r);
                if (e <= epsilon) {
                    s = std::clamp(-c / a, 0.0f, 1.0f);
                } else {
                    const float b = dot(d1, d2);
                    const float denominator = a * e - b * b;
                    if (std::abs(denominator) > epsilon) {
                        s = std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f);
                    }
                    t = (b * s + f) / e;
                    if (t < 0.0f) {
                        t = 0.0f;
                        s = std::clamp(-c / a, 0.0f, 1.0f);
                    } else if (t > 1.0f) {
                        t = 1.0f;
                        s = std::clamp((b - c) / a, 0.0f, 1.0f);
                    }
                }
            }
            const RE::NiPoint3 first = add(p1, scale(d1, s));
            const RE::NiPoint3 second = add(p2, scale(d2, t));
            return lengthSquared(subtract(first, second));
        }

        [[nodiscard]] inline bool segmentIntersectsTriangle(
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const GrabLocalTriangle& triangle)
        {
            constexpr float epsilon = 0.000001f;
            const RE::NiPoint3 direction = subtract(b, a);
            const RE::NiPoint3 edge1 = subtract(triangle.v1, triangle.v0);
            const RE::NiPoint3 edge2 = subtract(triangle.v2, triangle.v0);
            const RE::NiPoint3 p = cross(direction, edge2);
            const float determinant = dot(edge1, p);
            if (std::abs(determinant) <= epsilon) {
                return false;
            }
            const float inverse = 1.0f / determinant;
            const RE::NiPoint3 t = subtract(a, triangle.v0);
            const float u = dot(t, p) * inverse;
            if (u < 0.0f || u > 1.0f) {
                return false;
            }
            const RE::NiPoint3 q = cross(t, edge1);
            const float v = dot(direction, q) * inverse;
            if (v < 0.0f || u + v > 1.0f) {
                return false;
            }
            const float ratio = dot(edge2, q) * inverse;
            return ratio >= 0.0f && ratio <= 1.0f;
        }

        [[nodiscard]] inline float segmentTriangleDistanceSquared(
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const GrabLocalTriangle& triangle)
        {
            if (segmentIntersectsTriangle(a, b, triangle)) {
                return 0.0f;
            }
            float best = (std::min)(pointTriangleDistanceSquared(a, triangle), pointTriangleDistanceSquared(b, triangle));
            best = (std::min)(best, segmentSegmentDistanceSquared(a, b, triangle.v0, triangle.v1));
            best = (std::min)(best, segmentSegmentDistanceSquared(a, b, triangle.v1, triangle.v2));
            return (std::min)(best, segmentSegmentDistanceSquared(a, b, triangle.v2, triangle.v0));
        }

        [[nodiscard]] inline ProxyTriangle makeProxyTriangle(
            const GrabLocalTriangle& source,
            std::size_t sourceIndex)
        {
            ProxyTriangle result{};
            result.triangle = source;
            result.sourceIndex = sourceIndex;
            result.center = scale(add(add(source.v0, source.v1), source.v2), 1.0f / 3.0f);
            result.boundRadius = std::sqrt((std::max)({
                lengthSquared(subtract(source.v0, result.center)),
                lengthSquared(subtract(source.v1, result.center)),
                lengthSquared(subtract(source.v2, result.center)),
            })) * 1.001f + 0.001f;
            return result;
        }

        [[nodiscard]] inline bool containsSourceIndex(const TriangleProxy& proxy, std::size_t sourceIndex)
        {
            for (std::size_t i = 0; i < proxy.count; ++i) {
                if (proxy.triangles[i].sourceIndex == sourceIndex) {
                    return true;
                }
            }
            return false;
        }
    }

    [[nodiscard]] inline TriangleProxy buildTriangleProxy(
        std::span<const GrabLocalTriangle> source,
        const RE::NiPoint3& gripPointLocal)
    {
        TriangleProxy proxy{};
        proxy.sourceCount = source.size();
        if (source.empty() || !detail::finitePoint(gripPointLocal)) {
            return proxy;
        }

        if (source.size() <= kMaxProxyTriangles) {
            for (std::size_t i = 0; i < source.size(); ++i) {
                if (detail::finiteTriangle(source[i])) {
                    proxy.triangles[proxy.count++] = detail::makeProxyTriangle(source[i], i);
                }
            }
            return proxy;
        }

        struct RankedTriangle
        {
            float distanceSquared = (std::numeric_limits<float>::max)();
            std::size_t sourceIndex = 0;
        };
        std::array<RankedTriangle, kNearestProxyTriangles> nearest{};
        std::size_t nearestCount = 0;
        for (std::size_t sourceIndex = 0; sourceIndex < source.size(); ++sourceIndex) {
            if (!detail::finiteTriangle(source[sourceIndex])) {
                continue;
            }
            const float distanceSquared = detail::pointTriangleDistanceSquared(gripPointLocal, source[sourceIndex]);
            if (!std::isfinite(distanceSquared)) {
                continue;
            }
            if (nearestCount < nearest.size()) {
                nearest[nearestCount++] = RankedTriangle{ distanceSquared, sourceIndex };
                continue;
            }
            std::size_t worst = 0;
            for (std::size_t i = 1; i < nearestCount; ++i) {
                if (nearest[i].distanceSquared > nearest[worst].distanceSquared ||
                    (nearest[i].distanceSquared == nearest[worst].distanceSquared &&
                        nearest[i].sourceIndex > nearest[worst].sourceIndex)) {
                    worst = i;
                }
            }
            if (distanceSquared < nearest[worst].distanceSquared ||
                (distanceSquared == nearest[worst].distanceSquared && sourceIndex < nearest[worst].sourceIndex)) {
                nearest[worst] = RankedTriangle{ distanceSquared, sourceIndex };
            }
        }
        std::sort(nearest.begin(), nearest.begin() + nearestCount, [](const RankedTriangle& lhs, const RankedTriangle& rhs) {
            return lhs.distanceSquared < rhs.distanceSquared ||
                   (lhs.distanceSquared == rhs.distanceSquared && lhs.sourceIndex < rhs.sourceIndex);
        });
        for (std::size_t i = 0; i < nearestCount && proxy.count < proxy.triangles.size(); ++i) {
            proxy.triangles[proxy.count++] = detail::makeProxyTriangle(source[nearest[i].sourceIndex], nearest[i].sourceIndex);
        }

        constexpr std::size_t globalSlots = kMaxProxyTriangles - kNearestProxyTriangles;
        for (std::size_t slot = 0; slot < globalSlots && proxy.count < proxy.triangles.size(); ++slot) {
            const std::size_t sourceIndex = globalSlots > 1 ?
                                                slot * (source.size() - 1) / (globalSlots - 1) :
                                                0;
            if (!detail::finiteTriangle(source[sourceIndex]) || detail::containsSourceIndex(proxy, sourceIndex)) {
                continue;
            }
            proxy.triangles[proxy.count++] = detail::makeProxyTriangle(source[sourceIndex], sourceIndex);
        }
        for (std::size_t sourceIndex = 0;
             sourceIndex < source.size() && proxy.count < proxy.triangles.size();
             ++sourceIndex) {
            if (detail::finiteTriangle(source[sourceIndex]) && !detail::containsSourceIndex(proxy, sourceIndex)) {
                proxy.triangles[proxy.count++] = detail::makeProxyTriangle(source[sourceIndex], sourceIndex);
            }
        }
        return proxy;
    }

    [[nodiscard]] inline CandidateEvaluation evaluateCandidate(
        const TriangleProxy& proxy,
        const HandModel& hand,
        const RE::NiTransform& objectWorld,
        const RE::NiPoint3& longAxisObjectLocal,
        bool rodShape,
        float correctionAngleRadians)
    {
        CandidateEvaluation result{};
        if (!proxy.valid() || !hand.valid || hand.capsuleCount == 0 ||
            !detail::finiteTransform(objectWorld) || !detail::finitePoint(hand.palmNormalWorld)) {
            return result;
        }

        const float objectScale = std::abs(objectWorld.scale);
        const RE::NiPoint3 palmLocal = transform_math::worldPointToLocal(objectWorld, hand.palmCenterWorld);
        const RE::NiPoint3 normalLocal = detail::normalizeOrZero(
            transform_math::worldVectorToLocal(objectWorld, hand.palmNormalWorld));
        if (!detail::finitePoint(palmLocal) || detail::lengthSquared(normalLocal) <= 0.000001f) {
            return result;
        }

        auto nearestSurfaceDistanceGameUnits = [&](const RE::NiPoint3& pointLocal) {
            float bestSquared = (std::numeric_limits<float>::max)();
            for (const auto& proxyTriangle : proxy.view()) {
                ++result.exactDistanceTests;
                bestSquared = (std::min)(bestSquared,
                    detail::pointTriangleDistanceSquared(pointLocal, proxyTriangle.triangle));
            }
            return std::sqrt((std::max)(0.0f, bestSquared)) * objectScale;
        };

        const float palmGap = nearestSurfaceDistanceGameUnits(palmLocal) - hand.palmRadiusGameUnits;
        float minGap = (std::numeric_limits<float>::max)();
        float worstPenetration = 0.0f;
        for (std::size_t capsuleIndex = 0; capsuleIndex < hand.capsuleCount; ++capsuleIndex) {
            const auto& capsule = hand.capsules[capsuleIndex];
            const RE::NiPoint3 aLocal = transform_math::worldPointToLocal(objectWorld, capsule.aWorld);
            const RE::NiPoint3 bLocal = transform_math::worldPointToLocal(objectWorld, capsule.bWorld);
            if (!detail::finitePoint(aLocal) || !detail::finitePoint(bLocal)) {
                return CandidateEvaluation{};
            }
            float bestLocal = (std::numeric_limits<float>::max)();
            for (const auto& proxyTriangle : proxy.view()) {
                const float admit = bestLocal + proxyTriangle.boundRadius;
                if (detail::pointSegmentDistanceSquared(proxyTriangle.center, aLocal, bLocal) >= admit * admit) {
                    continue;
                }
                ++result.exactDistanceTests;
                const float distanceLocal = std::sqrt((std::max)(0.0f,
                    detail::segmentTriangleDistanceSquared(aLocal, bLocal, proxyTriangle.triangle)));
                bestLocal = (std::min)(bestLocal, distanceLocal);
            }
            if (!std::isfinite(bestLocal)) {
                return CandidateEvaluation{};
            }
            const float distanceGameUnits = bestLocal * objectScale;
            minGap = (std::min)(minGap, distanceGameUnits - capsule.radiusGameUnits);
            worstPenetration = (std::max)(worstPenetration,
                capsule.radiusGameUnits - distanceGameUnits);
        }

        float wrapSum = 0.0f;
        for (std::size_t tipIndex = 0; tipIndex < hand.tipCount; ++tipIndex) {
            const RE::NiPoint3 tipLocal = transform_math::worldPointToLocal(
                objectWorld,
                hand.tipCentersWorld[tipIndex]);
            if (!detail::finitePoint(tipLocal)) {
                return CandidateEvaluation{};
            }
            const float gap = std::clamp(
                nearestSurfaceDistanceGameUnits(tipLocal) - hand.tipRadiiGameUnits[tipIndex],
                0.0f,
                4.0f);
            wrapSum += gap * gap;
        }

        float behindPalmDepthGameUnits = 0.0f;
        constexpr float behindPalmFootprintRadiusGameUnits = 6.0f;
        const float footprintRadiusLocal = behindPalmFootprintRadiusGameUnits / objectScale;
        for (const auto& proxyTriangle : proxy.view()) {
            const RE::NiPoint3 vertices[3]{
                proxyTriangle.triangle.v0,
                proxyTriangle.triangle.v1,
                proxyTriangle.triangle.v2,
            };
            for (const auto& vertex : vertices) {
                const RE::NiPoint3 relative = detail::subtract(vertex, palmLocal);
                const float axialLocal = detail::dot(relative, normalLocal);
                if (axialLocal >= 0.0f) {
                    continue;
                }
                const RE::NiPoint3 lateral = detail::subtract(relative,
                    detail::scale(normalLocal, axialLocal));
                if (detail::lengthSquared(lateral) <= footprintRadiusLocal * footprintRadiusLocal) {
                    behindPalmDepthGameUnits = (std::max)(
                        behindPalmDepthGameUnits,
                        -axialLocal * objectScale);
                }
            }
        }

        const float touchRaw = (std::max)(0.0f, minGap);
        result.terms.touch = touchRaw * touchRaw;
        const float behindPalmRaw = (std::max)(0.0f, behindPalmDepthGameUnits - 2.0f);
        result.terms.behindPalm = behindPalmRaw * behindPalmRaw;
        const float palmProximityRaw = (std::max)(0.0f, palmGap - 2.5f);
        result.terms.palmProximity = palmProximityRaw * palmProximityRaw;
        const float overPenetrationRaw = (std::max)(0.0f, worstPenetration - 1.5f);
        result.terms.overPenetration = overPenetrationRaw * overPenetrationRaw;
        result.terms.wrap = hand.tipCount > 0 ? wrapSum / static_cast<float>(hand.tipCount) : 0.0f;
        if (rodShape) {
            const RE::NiPoint3 axis = detail::normalizeOrZero(longAxisObjectLocal);
            const float alignment = detail::dot(axis, normalLocal);
            result.terms.rodAxis = alignment * alignment;
        }

        // Round-3 fitted one-sided objective. Girth is deliberately absent:
        // the current analytic seat already selects the rod section and this
        // bounded local proxy does not own a complete longitudinal profile.
        result.objectiveScore =
            145.2505f * result.terms.touch +
            150.0f * result.terms.behindPalm +
            12.9162f * result.terms.palmProximity +
            2.7735f * result.terms.overPenetration +
            1.5205f * result.terms.wrap +
            14.9627f * result.terms.rodAxis;
        result.totalScore = result.objectiveScore +
                            6.0f * correctionAngleRadians * correctionAngleRadians;
        result.valid = std::isfinite(result.totalScore);
        return result;
    }

    [[nodiscard]] inline SelectionDecision selectBestCandidate(
        std::span<const CandidateEvaluation> evaluations)
    {
        SelectionDecision decision{};
        decision.evaluatedCandidateCount = evaluations.size();
        if (evaluations.empty() || !evaluations.front().valid) {
            decision.reason = "seedInvalid";
            return decision;
        }
        decision.seed = evaluations.front();
        decision.selected = evaluations.front();
        decision.reason = "seedKept";
        constexpr float improvementEpsilon = 0.0001f;
        for (std::size_t candidateIndex = 1; candidateIndex < evaluations.size(); ++candidateIndex) {
            const auto& candidate = evaluations[candidateIndex];
            if (!candidate.valid) {
                continue;
            }
            if (candidate.totalScore + improvementEpsilon < decision.selected.totalScore) {
                decision.candidateIndex = candidateIndex;
                decision.selected = candidate;
            }
        }
        decision.applied = decision.candidateIndex != 0;
        if (decision.applied) {
            decision.reason = "boundedObjectiveImproved";
        }
        return decision;
    }
}
