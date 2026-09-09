#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rock::grab_palm_seat
{
    struct Point
    {
        float along = 0.0f;
        float across = 0.0f;
        float depth = 0.0f;
    };

    struct DepthResult
    {
        float depthGameUnits = 0.0f;
        std::uint32_t footprintSampleCount = 0;
        const char* reason = "notEvaluated";
        bool valid = false;
    };

    // Clip each triangle to the palm's projected rectangle before measuring
    // depth. A tab beside the palm cannot displace the seat. Clipping also
    // retains coarse faces whose vertices all lie outside the footprint.
    template <class Triangles, class ProjectPoint>
    DepthResult computeDepth(const Triangles& triangles, float halfAlong, float halfAcross,
        float maxDepth, ProjectPoint project)
    {
        DepthResult result{};
        if (!std::isfinite(halfAlong) || !std::isfinite(halfAcross) ||
            halfAlong <= 0.0f || halfAcross <= 0.0f || !std::isfinite(maxDepth) || maxDepth < 0.0f) {
            result.reason = "invalidPalmFootprint";
            return result;
        }
        if (triangles.empty()) {
            result.reason = "noLocalTriangles";
            return result;
        }
        if (maxDepth == 0.0f) {
            result.reason = "seatDepthDisabled";
            result.valid = true;
            return result;
        }
        float bestDepth = 0.0f;
        std::uint32_t samples = 0;
        for (const auto& triangle : triangles) {
            // A triangle clipped by four planes has at most seven vertices.
            std::array<Point, 8> polygon{ project(triangle.v0), project(triangle.v1), project(triangle.v2) };
            std::size_t count = 3;
            for (std::size_t i = 0; i < count; ++i) {
                const auto& point = polygon[i];
                if (!std::isfinite(point.along) || !std::isfinite(point.across) || !std::isfinite(point.depth)) {
                    result.reason = "nonFinitePalmMesh";
                    return result;
                }
            }
            for (std::size_t plane = 0; plane < 4 && count > 0; ++plane) {
                const float extent = plane < 2 ? halfAlong : halfAcross;
                auto insideDistance = [&](const Point& point) {
                    const float coordinate = plane < 2 ? point.along : point.across;
                    return static_cast<double>(extent) - ((plane % 2 == 0) ? coordinate : -coordinate);
                };
                std::array<Point, 8> clipped{};
                std::size_t clippedCount = 0;
                auto append = [&](const Point& point) {
                    if (clippedCount == clipped.size()) {
                        return false;
                    }
                    clipped[clippedCount++] = point;
                    return true;
                };
                auto previous = polygon[count - 1];
                double previousDistance = insideDistance(previous);
                for (std::size_t i = 0; i < count; ++i) {
                    const auto current = polygon[i];
                    const double currentDistance = insideDistance(current);
                    if ((previousDistance < 0.0 && currentDistance > 0.0) ||
                        (previousDistance > 0.0 && currentDistance < 0.0)) {
                        const double t = previousDistance / (previousDistance - currentDistance);
                        auto interpolate = [t](float from, float to) {
                            return static_cast<float>(from + t * (static_cast<double>(to) - from));
                        };
                        if (!append(Point{ interpolate(previous.along, current.along),
                                interpolate(previous.across, current.across), interpolate(previous.depth, current.depth) })) {
                            result.reason = "palmClipCapacityExceeded";
                            return result;
                        }
                    }
                    if (currentDistance >= 0.0 && !append(current)) {
                        result.reason = "palmClipCapacityExceeded";
                        return result;
                    }
                    previous = current;
                    previousDistance = currentDistance;
                }
                polygon = clipped;
                count = clippedCount;
            }
            for (std::size_t i = 0; i < count; ++i) {
                if (polygon[i].depth > 0.0f) {
                    ++samples;
                    bestDepth = (std::max)(bestDepth, polygon[i].depth);
                }
            }
        }
        result.depthGameUnits = (std::min)(bestDepth, maxDepth);
        result.footprintSampleCount = samples;
        result.reason = samples > 0 ? "palmFootprintMeshDepth" : "noMeshInsidePalmFootprint";
        result.valid = true;
        return result;
    }
}
