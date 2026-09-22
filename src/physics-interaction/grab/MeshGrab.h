#pragma once

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/grab/SkinnedSurfaceMath.h"
#include "physics-interaction/grab/SegmentVisibilityPolicy.h"
#include "physics-interaction/grenade/LooseMolotovVisualPolicy.h"
#include "physics-interaction/native/NativeMemory.h"
#include "RE/Fallout.h"
#include "physics-interaction/grab/SkinnedBoneOwner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

#include "RE/Bethesda/BSVisit.h"

namespace rock
{

    namespace VROffset
    {
        constexpr int rendererData = 0x188;
        constexpr int vertexDesc = 0x190;
        constexpr int geometryType = 0x198;
        constexpr int numTriangles = 0x1A0;
        constexpr int numVertices = 0x1A4;
        constexpr int skinInstance = 0x180;

        constexpr int dynamicDataSize = 0x1B0;
        constexpr int dynamicLock = 0x1B8;
        constexpr int dynamicLockCnt = 0x1BC;
        constexpr int dynamicVertices = 0x1C0;
        constexpr int dynamicSegments = 0x1C8;

        constexpr std::uint8_t kTypeBSTriShape = 3;
        constexpr std::uint8_t kTypeBSDynamicTriShape = 4;

        constexpr std::uintptr_t kFuncDynamicTriShapeLockVertices = 0x1C43B70;
        constexpr std::uintptr_t kFuncDynamicTriShapeUnlockVertices = 0x1C43BD0;
    }

    namespace BSSkinOffset
    {
        // Live counts, not capacities: native resize 141C358D0/141C35960,
        // iterator 1402888E0, render palette 141DABC60/141DB6CA0.
        constexpr int bonesData = 0x10;
        constexpr int bonesCount = 0x20;
        constexpr int worldTransforms = 0x28;
        constexpr int worldTransformCount = 0x38;
        constexpr int boneData = 0x40;
        constexpr int extraScale = 0x50;
        constexpr int transformArrayData = 0x10;
        constexpr int transformArrayCount = 0x20;
    }

    constexpr std::uint32_t kMaxMeshExtractionTriangles = 1'000'000;
    static_assert(sizeof(RE::BSSkin::Instance) == 0xC0);
    static_assert(sizeof(RE::BSSkin::BoneData::BoneTransform) == 0x50);

    inline RE::NiPoint3 transformPoint(const RE::NiTransform& t, const RE::NiPoint3& p)
    {
        /*
         * Mesh extraction must match FO4VR's native NiTransform compose
         * convention. Ghidra verification of the engine compose path showed
         * child-local vectors are applied through the parent column basis.
         */
        RE::NiPoint3 scaled(p.x * t.scale, p.y * t.scale, p.z * t.scale);
        RE::NiPoint3 rotated;
        rotated.x = t.rotate.entry[0][0] * scaled.x + t.rotate.entry[1][0] * scaled.y + t.rotate.entry[2][0] * scaled.z;
        rotated.y = t.rotate.entry[0][1] * scaled.x + t.rotate.entry[1][1] * scaled.y + t.rotate.entry[2][1] * scaled.z;
        rotated.z = t.rotate.entry[0][2] * scaled.x + t.rotate.entry[1][2] * scaled.y + t.rotate.entry[2][2] * scaled.z;
        return RE::NiPoint3(rotated.x + t.translate.x, rotated.y + t.translate.y, rotated.z + t.translate.z);
    }

    struct TriangleData
    {
        RE::NiPoint3 v0, v1, v2;

        void applyTransform(const RE::NiTransform& t)
        {
            v0 = transformPoint(t, v0);
            v1 = transformPoint(t, v1);
            v2 = transformPoint(t, v2);
        }
    };

    enum class GrabSurfaceSourceKind : std::uint8_t
    {
        Static,
        Dynamic,
        Skinned,
        CollisionQuery,
        Fallback
    };

    struct GrabSurfaceVertexInfluence
    {
        RE::NiAVObject* bone = nullptr;
        float weight = 0.0f;
    };

    struct GrabSurfaceTriangleData
    {
        TriangleData triangle{};
        RE::NiAVObject* sourceNode = nullptr;
        RE::BSTriShape* sourceShape = nullptr;
        std::uint32_t triangleIndex = 0;
        GrabSurfaceSourceKind sourceKind = GrabSurfaceSourceKind::Static;
        std::array<std::array<GrabSurfaceVertexInfluence, 4>, 3> skinInfluences{};
        bool hasSkinInfluences = false;
    };

    // Owned by one acquisition. Rebuild after filtering/replacing its surface
    // vector; neither this index nor its query scratch survives the acquisition.
    // Contiguous leaves preserve source order (including equal-distance hits)
    // and build in linear time without sorting the full visual mesh.
    class GrabSurfaceQueryIndex
    {
    public:
        struct RankedTriangle
        {
            float distanceSquared;
            std::size_t index;
        };

        void build(const std::vector<GrabSurfaceTriangleData>& triangles, std::size_t minimumIndexedTriangles = 2048)
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabMeshQueryIndexBuild);
            _source = triangles.data();
            _size = triangles.size();
            _nodes.clear();
            _nearest.clear();
            _cachedCount = 0;
            if (_size > minimumIndexedTriangles) {
                _nodes.reserve((_size / kLeafSize + 1) * 4);
                buildNode(triangles, 0, _size);
            }
        }

        template <class Limit, class Visit>
        std::size_t visit(const std::vector<GrabSurfaceTriangleData>& triangles,
            const RE::NiPoint3& point, Limit&& limitSquared, Visit&& visitor, bool nearestFirst = false) const
        {
            std::size_t tested = 0;
            if (_source != triangles.data() || _size != triangles.size() || _nodes.empty() || !finite(point)) {
                for (std::size_t i = 0; i < triangles.size(); ++i) {
                    visitor(i);
                    ++tested;
                }
                return tested;
            }
            if (nearestFirst) {
                // Balanced range splits bound the stack by the address width.
                std::array<std::size_t, std::numeric_limits<std::size_t>::digits> stack{};
                std::size_t pending = 1;
                while (pending) {
                    const auto n = stack[--pending];
                    const auto& node = _nodes[n];
                    if (distanceSquared(node, point) > limitSquared()) continue;
                    if (node.end - node.begin <= kLeafSize) {
                        for (auto i = node.begin; i < node.end; ++i) { visitor(i); ++tested; }
                    } else {
                        auto nearNode = n + 1;
                        auto farNode = _nodes[nearNode].escape;
                        if (distanceSquared(_nodes[farNode], point) < distanceSquared(_nodes[nearNode], point)) std::swap(nearNode, farNode);
                        stack[pending++] = farNode;
                        stack[pending++] = nearNode;
                    }
                }
                return tested;
            }
            for (std::size_t n = 0; n < _nodes.size();) {
                const auto& node = _nodes[n];
                if (distanceSquared(node, point) > limitSquared()) {
                    n = node.escape;
                } else if (node.end - node.begin <= kLeafSize) {
                    for (auto i = node.begin; i < node.end; ++i) {
                        visitor(i);
                        ++tested;
                    }
                    ++n;
                } else {
                    ++n;
                }
            }
            return tested;
        }

        const std::vector<RankedTriangle>& nearest(const std::vector<GrabSurfaceTriangleData>& triangles,
            const RE::NiPoint3& point, std::size_t count) const
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabTriangleSelection);
            if (_cachedSource == triangles.data() && _cachedSize == triangles.size() && _cachedCount == count &&
                point.x == _cachedPoint.x && point.y == _cachedPoint.y && point.z == _cachedPoint.z && !_nearest.empty()) {
                performance_profiler::observeValue(performance_profiler::ValueMetric::GrabTriangleSelectionTests, 0);
                return _nearest;
            }
            _nearest.clear();
            _cachedCount = count;
            _cachedPoint = point;
            _cachedSource = triangles.data();
            _cachedSize = triangles.size();
            count = (std::min)(count, triangles.size());
            if (count == 0) return _nearest;
            _nearest.reserve(count);
            const auto tested = visit(triangles, point,
                [&] { return _nearest.size() == count ? _nearest.front().distanceSquared : std::numeric_limits<float>::infinity(); },
                [&](std::size_t i) {
                    const auto& triangle = triangles[i].triangle;
                    const RE::NiPoint3 centroid = (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
                    const RankedTriangle candidate{
                        (std::min)({ vector_math::lengthSquared(centroid - point), vector_math::lengthSquared(triangle.v0 - point),
                            vector_math::lengthSquared(triangle.v1 - point), vector_math::lengthSquared(triangle.v2 - point) }), i };
                    if (_nearest.size() < count) {
                        _nearest.push_back(candidate);
                        std::push_heap(_nearest.begin(), _nearest.end(), less);
                    } else if (less(candidate, _nearest.front())) {
                        std::pop_heap(_nearest.begin(), _nearest.end(), less);
                        _nearest.back() = candidate;
                        std::push_heap(_nearest.begin(), _nearest.end(), less);
                    }
                }, true);
            std::sort_heap(_nearest.begin(), _nearest.end(), less);
            performance_profiler::observeValue(performance_profiler::ValueMetric::GrabTriangleSelectionTests, tested);
            return _nearest;
        }

    private:
        static constexpr std::size_t kLeafSize = 16;
        struct Node
        {
            RE::NiPoint3 low{ INFINITY, INFINITY, INFINITY };
            RE::NiPoint3 high{ -INFINITY, -INFINITY, -INFINITY };
            std::size_t begin = 0, end = 0, escape = 0;
        };

        static bool finite(const RE::NiPoint3& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
        static bool less(const RankedTriangle& a, const RankedTriangle& b)
        {
            return a.distanceSquared == b.distanceSquared ? a.index < b.index : a.distanceSquared < b.distanceSquared;
        }
        static void extend(Node& node, const RE::NiPoint3& p)
        {
            if (!finite(p)) {
                node.low = { -INFINITY, -INFINITY, -INFINITY };
                node.high = { INFINITY, INFINITY, INFINITY };
                return;
            }
            node.low = { (std::min)(node.low.x, p.x), (std::min)(node.low.y, p.y), (std::min)(node.low.z, p.z) };
            node.high = { (std::max)(node.high.x, p.x), (std::max)(node.high.y, p.y), (std::max)(node.high.z, p.z) };
        }
        std::size_t buildNode(const std::vector<GrabSurfaceTriangleData>& triangles, std::size_t begin, std::size_t end)
        {
            const auto index = _nodes.size();
            _nodes.emplace_back();
            Node node;
            node.begin = begin;
            node.end = end;
            if (end - begin <= kLeafSize) {
                for (auto i = begin; i < end; ++i) {
                    const auto& t = triangles[i].triangle;
                    extend(node, t.v0); extend(node, t.v1); extend(node, t.v2);
                }
            } else {
                const auto middle = begin + (end - begin) / 2;
                const auto left = buildNode(triangles, begin, middle);
                const auto right = buildNode(triangles, middle, end);
                extend(node, _nodes[left].low); extend(node, _nodes[left].high);
                extend(node, _nodes[right].low); extend(node, _nodes[right].high);
            }
            node.escape = _nodes.size();
            _nodes[index] = node;
            return index;
        }
        static double distanceSquared(const Node& node, const RE::NiPoint3& point)
        {
            // Outward padding covers float interpolation/centroid rounding at
            // world-coordinate magnitudes. Prune only strictly outside so ties
            // and inclusive distance thresholds still reach the exact predicate.
            const auto axis = [](float p, float lo, float hi) {
                const double scale = (std::max)({ 1.0, std::abs(static_cast<double>(p)),
                    std::abs(static_cast<double>(lo)), std::abs(static_cast<double>(hi)) });
                const double padding = 32.0 * std::numeric_limits<float>::epsilon() * scale;
                return (std::max)({ 0.0, static_cast<double>(lo) - p - padding, static_cast<double>(p) - hi - padding });
            };
            const auto x = axis(point.x, node.low.x, node.high.x);
            const auto y = axis(point.y, node.low.y, node.high.y);
            const auto z = axis(point.z, node.low.z, node.high.z);
            return x * x + y * y + z * z;
        }
        const GrabSurfaceTriangleData* _source = nullptr;
        std::size_t _size = 0;
        std::vector<Node> _nodes;
        mutable std::vector<RankedTriangle> _nearest;
        mutable RE::NiPoint3 _cachedPoint{};
        mutable std::size_t _cachedCount = 0;
        mutable const GrabSurfaceTriangleData* _cachedSource = nullptr;
        mutable std::size_t _cachedSize = 0;
    };

    struct GrabSurfaceHit
    {
        RE::NiPoint3 position{};
        RE::NiPoint3 normal{};
        int triangleIndex = -1;
        float distance = 1e30f;
        RE::NiAVObject* sourceNode = nullptr;
        RE::BSTriShape* sourceShape = nullptr;
        std::uint32_t sourceTriangleIndex = 0;
        TriangleData triangle{};
        GrabSurfaceSourceKind sourceKind = GrabSurfaceSourceKind::Fallback;
        std::uint32_t shapeKey = 0xFFFF'FFFF;
        std::uint32_t shapeCollisionFilterInfo = 0;
        float hitFraction = 1.0f;
        float pivotToSurfaceDistanceGameUnits = 0.0f;
        float selectionToMeshDistanceGameUnits = 0.0f;
        float signedAlongPalmDistanceGameUnits = 0.0f;
        float lateralPalmDistanceGameUnits = 0.0f;
        bool hasSelectionHit = false;
        bool resolvedOwnerMatchesBody = false;
        bool hasTriangle = false;
        bool hasSkinInfluences = false;
        bool hasShapeKey = false;
        bool valid = false;
    };

    inline const char* grabSurfaceSourceKindName(GrabSurfaceSourceKind kind)
    {
        switch (kind) {
        case GrabSurfaceSourceKind::Static:
            return "static";
        case GrabSurfaceSourceKind::Dynamic:
            return "dynamic";
        case GrabSurfaceSourceKind::Skinned:
            return "skinned";
        case GrabSurfaceSourceKind::CollisionQuery:
            return "collisionQuery";
        case GrabSurfaceSourceKind::Fallback:
        default:
            return "fallback";
        }
    }

    inline bool hasAnySkinInfluence(const std::array<std::array<GrabSurfaceVertexInfluence, 4>, 3>& skinInfluences)
    {
        for (const auto& vertexInfluences : skinInfluences) {
            for (const auto& influence : vertexInfluences) {
                if (influence.bone && influence.weight > 0.0f) {
                    return true;
                }
            }
        }
        return false;
    }

    inline void appendSurfaceTriangle(std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles,
        const TriangleData& triangle,
        RE::BSTriShape* sourceShape,
        std::uint32_t triangleIndex,
        GrabSurfaceSourceKind sourceKind,
        const std::array<std::array<GrabSurfaceVertexInfluence, 4>, 3>* skinInfluences = nullptr)
    {
        if (!outSurfaceTriangles) {
            return;
        }

        auto& surfaceTriangle = outSurfaceTriangles->emplace_back();
        surfaceTriangle.triangle = triangle;
        surfaceTriangle.sourceNode = sourceShape;
        surfaceTriangle.sourceShape = sourceShape;
        surfaceTriangle.triangleIndex = triangleIndex;
        surfaceTriangle.sourceKind = sourceKind;
        if (skinInfluences) {
            surfaceTriangle.skinInfluences = *skinInfluences;
            surfaceTriangle.hasSkinInfluences = hasAnySkinInfluence(*skinInfluences);
        }
    }

    inline float halfToFloat(std::uint16_t h)
    {
        std::uint32_t sign = (h >> 15) & 0x1;
        std::uint32_t exponent = (h >> 10) & 0x1F;
        std::uint32_t mantissa = h & 0x3FF;

        std::uint32_t f;

        if (exponent == 0) {
            if (mantissa == 0) {
                f = sign << 31;
            } else {
                exponent = 0;
                while ((mantissa & 0x400) == 0) {
                    mantissa <<= 1;
                    exponent++;
                }
                mantissa &= 0x3FF;

                f = (sign << 31) | ((114 - exponent) << 23) | (mantissa << 13);
            }
        } else if (exponent == 0x1F) {
            f = (sign << 31) | (0xFF << 23) | (mantissa << 13);
        } else {
            f = (sign << 31) | ((exponent - 15 + 127) << 23) | (mantissa << 13);
        }

        float result;
        std::memcpy(&result, &f, sizeof(float));
        return result;
    }

    inline RE::NiPoint3 readVertexPosition(const std::uint8_t* vertexBase, std::uint32_t posOffset, bool fullPrec)
    {
        const std::uint8_t* posPtr = vertexBase + posOffset;
        if (fullPrec) {
            const float* fp = reinterpret_cast<const float*>(posPtr);
            return RE::NiPoint3(fp[0], fp[1], fp[2]);
        } else {
            const std::uint16_t* hp = reinterpret_cast<const std::uint16_t*>(posPtr);
            return RE::NiPoint3(halfToFloat(hp[0]), halfToFloat(hp[1]), halfToFloat(hp[2]));
        }
    }

    struct TriShapeRawGeometry
    {
        void* rendererData = nullptr;
        std::uint32_t numTriangles = 0;
        std::uint16_t numVertices = 0;
        std::uint64_t vertexDesc = 0;
        std::uint16_t* triangles = nullptr;
    };

    struct MeshExtractionStats
    {
        std::uint32_t visitedShapes = 0;
        std::uint32_t staticShapes = 0;
        std::uint32_t dynamicShapes = 0;
        std::uint32_t skinnedShapes = 0;
        std::uint32_t dynamicSkinnedSkipped = 0;
        std::uint32_t emptyShapes = 0;
        std::uint32_t staticTriangles = 0;
        std::uint32_t dynamicTriangles = 0;
        std::uint32_t skinnedTriangles = 0;

        [[nodiscard]] std::uint32_t totalTriangles() const noexcept { return staticTriangles + dynamicTriangles + skinnedTriangles; }
    };

    inline bool readTriShapeRawGeometry(RE::BSTriShape* triShape, TriShapeRawGeometry& out)
    {
        if (!triShape)
            return false;

        // Flame render geometry belongs to the armed-state visual, never to the
        // bottle's grab/finger surface. Preserve this reader's guarded boundary.
        const char* shapeName = nullptr;
        std::array<char, sizeof(loose_molotov_visual_policy::kVisualShapeName)> name{};
        if (!native_memory::tryReadField(triShape, offsetof(RE::NiObjectNET, name), shapeName))
            return false;
        if (shapeName && native_memory::guardedCopyFromMemory(shapeName, name.data(), name.size()) &&
            name.back() == '\0' && loose_molotov_visual_policy::isVisualOnlyShape(name.data()))
            return false;

        if (!native_memory::tryReadField(triShape, VROffset::rendererData, out.rendererData))
            return false;
        if (!out.rendererData)
            return false;

        if (!native_memory::tryReadField(triShape, VROffset::numTriangles, out.numTriangles) ||
            !native_memory::tryReadField(triShape, VROffset::numVertices, out.numVertices) ||
            !native_memory::tryReadField(triShape, VROffset::vertexDesc, out.vertexDesc))
            return false;
        if (out.numTriangles == 0 || out.numVertices == 0)
            return false;
        if (out.numTriangles > kMaxMeshExtractionTriangles)
            return false;

        void* triangleDataPtr = nullptr;
        if (!native_memory::tryReadField(out.rendererData, 0x10, triangleDataPtr))
            return false;
        if (!triangleDataPtr)
            return false;

        if (!native_memory::tryReadField(triangleDataPtr, 0x08, out.triangles) || !out.triangles)
            return false;

        const auto triangleBytes = static_cast<std::size_t>(out.numTriangles) * 3u * sizeof(std::uint16_t);
        return native_memory::pointerRangeLooksReadable(out.triangles, triangleBytes);
    }

    inline std::uint8_t* readStaticVertexBlock(void* rendererData)
    {
        if (!rendererData)
            return nullptr;

        void* vertexDataPtr = nullptr;
        if (!native_memory::tryReadField(rendererData, 0x08, vertexDataPtr))
            return nullptr;
        if (!vertexDataPtr)
            return nullptr;

        std::uint8_t* vertices = nullptr;
        return native_memory::tryReadField(vertexDataPtr, 0x08, vertices) ? vertices : nullptr;
    }

    inline bool vertexBufferRangeLooksReadable(const std::uint8_t* vertices, std::uint32_t vertexCount, std::uint32_t vertexStride)
    {
        if (!vertices || vertexCount == 0 || vertexStride == 0) {
            return false;
        }

        const auto byteCount = static_cast<std::size_t>(vertexCount) * static_cast<std::size_t>(vertexStride);
        return native_memory::pointerRangeLooksReadable(vertices, byteCount);
    }

    inline bool finiteFloatArray(const float* values, std::size_t count)
    {
        if (!values) {
            return false;
        }

        for (std::size_t i = 0; i < count; ++i) {
            if (!std::isfinite(values[i])) {
                return false;
            }
        }
        return true;
    }

    inline bool isDynamicTriShape(RE::BSTriShape* triShape)
    {
        if (!triShape)
            return false;
        std::uint8_t geometryType = 0;
        return native_memory::tryReadField(triShape, VROffset::geometryType, geometryType) && geometryType == VROffset::kTypeBSDynamicTriShape;
    }

    inline RE::NiPoint3 readDynamicVertexPosition(const std::uint8_t* vertexBase)
    {
        const auto* hp = reinterpret_cast<const std::uint16_t*>(vertexBase);
        return RE::NiPoint3(halfToFloat(hp[0]), halfToFloat(hp[1]), halfToFloat(hp[2]));
    }

    class DynamicTriShapeVertexLock
    {
    public:
        explicit DynamicTriShapeVertexLock(RE::BSTriShape* triShape) : _shape(triShape)
        {
            if (!_shape)
                return;

            using lock_t = std::uint8_t* (*)(void*);
            static REL::Relocation<lock_t> lock{ REL::Offset(VROffset::kFuncDynamicTriShapeLockVertices) };
            _vertices = lock(_shape);
            _locked = true;
        }

        ~DynamicTriShapeVertexLock()
        {
            if (!_locked || !_shape)
                return;

            using unlock_t = int (*)(void*);
            static REL::Relocation<unlock_t> unlock{ REL::Offset(VROffset::kFuncDynamicTriShapeUnlockVertices) };
            unlock(_shape);
        }

        DynamicTriShapeVertexLock(const DynamicTriShapeVertexLock&) = delete;
        DynamicTriShapeVertexLock& operator=(const DynamicTriShapeVertexLock&) = delete;

        [[nodiscard]] const std::uint8_t* vertices() const noexcept { return _vertices; }

    private:
        RE::BSTriShape* _shape = nullptr;
        std::uint8_t* _vertices = nullptr;
        bool _locked = false;
    };

    inline int extractTrianglesFromDynamicTriShape(
        RE::BSTriShape* triShape,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles = nullptr,
        std::vector<TriangleData>* outLocalTriangles = nullptr)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::MeshDynamicExtraction);
        [[maybe_unused]] const char* shapeName = triShape->name.c_str() ? triShape->name.c_str() : "(null)";

        TriShapeRawGeometry geometry;
        if (!readTriShapeRawGeometry(triShape, geometry))
            return 0;

        std::uint32_t dynamicStride = static_cast<std::uint32_t>((geometry.vertexDesc >> 2) & 0x3C);
        if (dynamicStride < 6) {
            ROCK_LOG_WARN(MeshGrab, "Dynamic '{}': bad dynamic stride {}", shapeName, dynamicStride);
            return 0;
        }

        std::uint32_t dynamicDataSize = 0;
        if (!native_memory::tryReadField(triShape, VROffset::dynamicDataSize, dynamicDataSize)) {
            ROCK_LOG_WARN(MeshGrab, "Dynamic '{}': unreadable dynamic data size", shapeName);
            return 0;
        }
        std::uint64_t requiredBytes = static_cast<std::uint64_t>(dynamicStride) * geometry.numVertices;
        if (dynamicDataSize < requiredBytes) {
            ROCK_LOG_WARN(MeshGrab, "Dynamic '{}': vertex buffer too small (size={}, required={}, verts={}, stride={})", shapeName, dynamicDataSize, requiredBytes,
                geometry.numVertices, dynamicStride);
            return 0;
        }

        DynamicTriShapeVertexLock lockedVertices(triShape);
        const std::uint8_t* verts = lockedVertices.vertices();
        if (!verts) {
            ROCK_LOG_WARN(MeshGrab, "Dynamic '{}': null dynamic vertex buffer", shapeName);
            return 0;
        }
        if (!vertexBufferRangeLooksReadable(verts, geometry.numVertices, dynamicStride)) {
            ROCK_LOG_WARN(MeshGrab, "Dynamic '{}': unreadable dynamic vertex buffer (verts={}, stride={})", shapeName, geometry.numVertices, dynamicStride);
            return 0;
        }

        RE::NiTransform worldTransform = triShape->world;

        int added = 0;
        for (std::uint32_t i = 0; i < geometry.numTriangles; i++) {
            std::uint16_t i0 = geometry.triangles[i * 3 + 0];
            std::uint16_t i1 = geometry.triangles[i * 3 + 1];
            std::uint16_t i2 = geometry.triangles[i * 3 + 2];

            if (i0 >= geometry.numVertices || i1 >= geometry.numVertices || i2 >= geometry.numVertices) {
                continue;
            }

            TriangleData localTriangle;
            localTriangle.v0 = readDynamicVertexPosition(verts + i0 * dynamicStride);
            localTriangle.v1 = readDynamicVertexPosition(verts + i1 * dynamicStride);
            localTriangle.v2 = readDynamicVertexPosition(verts + i2 * dynamicStride);
            TriangleData worldTriangle = localTriangle;
            worldTriangle.applyTransform(worldTransform);

            outTriangles.push_back(worldTriangle);
            if (outLocalTriangles) {
                outLocalTriangles->push_back(localTriangle);
            }
            appendSurfaceTriangle(outSurfaceTriangles, worldTriangle, triShape, i, GrabSurfaceSourceKind::Dynamic);
            added++;
        }

        return added;
    }

    inline bool readVisibleTriangleRanges(RE::BSTriShape* shape, std::uint32_t triangleCount,
        segment_visibility::VisibleRanges& visible)
    {
        std::uint8_t type = 0;
        if (!native_memory::tryReadField(shape, VROffset::geometryType, type)) return false;
        visible = {};
        if (type != 8) { visible.ranges[0] = {0,triangleCount}; visible.count = 1; visible.valid = true; return true; }
        // BSSubIndexTriShape constructor 141D56860 and accessor 141D571D0.
        static const bool verified = [] {
            constexpr std::array<std::uint8_t,8> expected{0x48,0x8B,0x81,0xB0,0x01,0x00,0x00,0xC3};
            std::array<std::uint8_t,8> live{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Module::get().base()+0x1D571D0),
                live.data(),live.size()) && live==expected;
        }();
        void* data=nullptr;
        void* shared=nullptr;
        const segment_visibility::Segment* entries=nullptr;
        const std::uint32_t* rootMap=nullptr;
        std::uint32_t rootCount=0, segmentCount=0, specialRoot=0, contiguousCount=0;
        std::uint8_t contiguous=0;
        if (!verified || !native_memory::tryReadField(shape,0x1B0,data) || !data ||
            !native_memory::tryReadField(data,0x10,shared) ||
            !native_memory::tryReadField(data,0x18,entries) ||
            !native_memory::tryReadField(data,0x2C,rootCount) ||
            !native_memory::tryReadField(data,0x30,segmentCount) ||
            !native_memory::tryReadField(data,0x34,contiguousCount) ||
            !native_memory::tryReadField(data,0x38,specialRoot) ||
            !native_memory::tryReadField(data,0x3D,contiguous) ||
            rootCount>segment_visibility::kMaxSegments || segmentCount>segment_visibility::kMaxSegments ||
            (segmentCount && !entries) || (shared && !native_memory::tryReadField(shared,0x18,rootMap))) {
            ROCK_LOG_SAMPLE_WARN(MeshGrab,2000,"Mesh visibility rejected '{}': invalid segment arrays roots={} segments={}",
                shape->name.c_str(),rootCount,segmentCount);
            return false;
        }
        // Native render consumers 141DA03D0 and 142891340 use this optimized whole range.
        if (contiguous && contiguousCount) {
            if (contiguousCount>triangleCount) return false;
            visible.ranges[0] = {0,contiguousCount}; visible.count = 1; visible.valid = true;
            return true;
        }
        std::array<segment_visibility::Segment,segment_visibility::kMaxSegments> segments{};
        std::array<std::uint32_t,segment_visibility::kMaxSegments> roots{};
        if (segmentCount && !native_memory::guardedCopyFromMemory(entries,segments.data(),segmentCount*sizeof(segments[0]))) return false;
        for (std::uint32_t i=0;i<rootCount;++i) {
            auto index=i==specialRoot ? 0u : i;
            if (shared) {
                if (!rootMap || !native_memory::tryReadValue(rootMap+index,roots[i])) return false;
            } else roots[i]=index;
        }
        const auto ranges=segment_visibility::resolve({segments.data(),segmentCount},{roots.data(),rootCount},triangleCount);
        if (!ranges.valid) {
            ROCK_LOG_SAMPLE_WARN(MeshGrab,2000,"Mesh visibility rejected '{}': invalid segment hierarchy/range",shape->name.c_str());
            return false;
        }
        visible = ranges;
        return true;
    }

    inline bool readVisibleTriangles(RE::BSTriShape* shape, std::uint32_t triangleCount, std::vector<std::uint8_t>& visible)
    {
        segment_visibility::VisibleRanges ranges{};
        if (!readVisibleTriangleRanges(shape, triangleCount, ranges)) return false;
        if (ranges.count == 1 && ranges.ranges[0].first == 0 && ranges.ranges[0].count == triangleCount) return true;
        visible.assign(triangleCount,0);
        for (std::size_t i=0;i<ranges.count;++i) std::fill_n(visible.begin()+ranges.ranges[i].first,ranges.ranges[i].count,1);
        return true;
    }

    // The caller validates the buffers and visibility mask. Cache only vertices
    // referenced by admitted triangles; hidden/invalid triangles still do no work.
    inline int appendStaticMeshTriangles(const TriShapeRawGeometry& geometry,
        const std::uint8_t* vertices, std::uint32_t stride, std::uint32_t positionOffset, bool fullPrecision,
        const std::vector<std::uint8_t>& visibleTriangles, const RE::NiTransform& worldTransform,
        RE::BSTriShape* sourceShape, std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles,
        std::vector<TriangleData>* outLocalTriangles)
    {
        struct Vertex { RE::NiPoint3 local{}, world{}; bool ready = false; };
        std::vector<Vertex> cache(geometry.numVertices);
        std::size_t transformed = 0;
        const auto vertex = [&](std::uint16_t index) -> const Vertex& {
            auto& value = cache[index];
            if (!value.ready) {
                value.local = readVertexPosition(vertices + index * stride, positionOffset, fullPrecision);
                value.world = transformPoint(worldTransform, value.local);
                value.ready = true;
                ++transformed;
            }
            return value;
        };
        const auto visibleCount = visibleTriangles.empty() ? geometry.numTriangles :
            static_cast<std::size_t>(std::count_if(visibleTriangles.begin(), visibleTriangles.end(), [](auto value) { return value != 0; }));
        const auto reserveAppend = [visibleCount](auto& output) {
            const auto required = output.size() + visibleCount;
            if (required > output.capacity()) output.reserve((std::max)(required, output.capacity() * 2));
        };
        int added = 0;
        for (std::uint32_t i = 0; i < geometry.numTriangles; ++i) {
            if (!visibleTriangles.empty() && !visibleTriangles[i]) continue;
            const auto i0 = geometry.triangles[i * 3];
            const auto i1 = geometry.triangles[i * 3 + 1];
            const auto i2 = geometry.triangles[i * 3 + 2];
            if (i0 >= geometry.numVertices || i1 >= geometry.numVertices || i2 >= geometry.numVertices) continue;
            if (added == 0) {
                reserveAppend(outTriangles);
                if (outSurfaceTriangles) reserveAppend(*outSurfaceTriangles);
                if (outLocalTriangles) reserveAppend(*outLocalTriangles);
            }
            const auto& v0 = vertex(i0);
            const auto& v1 = vertex(i1);
            const auto& v2 = vertex(i2);
            const TriangleData triangle{ v0.world, v1.world, v2.world };
            outTriangles.push_back(triangle);
            if (outLocalTriangles) outLocalTriangles->push_back({ v0.local, v1.local, v2.local });
            appendSurfaceTriangle(outSurfaceTriangles, triangle, sourceShape, i, GrabSurfaceSourceKind::Static);
            ++added;
        }
        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshStaticVerticesTransformed, transformed);
        return added;
    }

    inline int extractTrianglesFromTriShape(
        RE::BSTriShape* triShape,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles = nullptr,
        std::vector<TriangleData>* outLocalTriangles = nullptr)
    {
        if (isDynamicTriShape(triShape)) {
            return extractTrianglesFromDynamicTriShape(triShape, outTriangles, outSurfaceTriangles, outLocalTriangles);
        }

        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::MeshStaticExtraction);
        TriShapeRawGeometry geometry;
        if (!readTriShapeRawGeometry(triShape, geometry))
            return 0;

        std::uint32_t vtxStride = static_cast<std::uint32_t>(geometry.vertexDesc & 0xF) * 4;

        std::uint32_t posOffset = static_cast<std::uint32_t>((geometry.vertexDesc >> 2) & 0x3C);
        bool fullPrecision = ((geometry.vertexDesc >> 54) & 1) != 0;

        std::uint32_t minStride = fullPrecision ? (posOffset + 12) : (posOffset + 6);
        if (vtxStride < minStride)
            return 0;

        auto* verts = readStaticVertexBlock(geometry.rendererData);
        if (!verts)
            return 0;
        if (!vertexBufferRangeLooksReadable(verts, geometry.numVertices, vtxStride))
            return 0;

        RE::NiTransform worldTransform = triShape->world;

        std::vector<std::uint8_t> visibleTriangles;
        if (!readVisibleTriangles(triShape, geometry.numTriangles, visibleTriangles)) return 0;
        return appendStaticMeshTriangles(geometry, verts, vtxStride, posOffset, fullPrecision,
            visibleTriangles, worldTransform, triShape, outTriangles, outSurfaceTriangles, outLocalTriangles);
    }

    inline bool isSkinned(RE::BSTriShape* triShape)
    {
        void* skinInst = nullptr;
        if (!native_memory::tryReadField(triShape, VROffset::skinInstance, skinInst)) {
            return false;
        }
        return skinInst != nullptr;
    }

    // Select work from visible, valid triangles before skinning. Every admitted
    // vertex still uses the same full blend; hidden or unreferenced data cannot
    // contribute to the returned mesh.
    inline std::vector<std::uint8_t> referencedMeshVertices(
        std::span<const std::uint16_t> indices, std::size_t vertexCount,
        std::span<const std::uint8_t> visibleTriangles)
    {
        std::vector<std::uint8_t> referenced(vertexCount, 0);
        for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
            if (!visibleTriangles.empty() &&
                (i / 3 >= visibleTriangles.size() || !visibleTriangles[i / 3])) continue;
            const auto a = indices[i], b = indices[i + 1], c = indices[i + 2];
            if (a >= vertexCount || b >= vertexCount || c >= vertexCount) continue;
            referenced[a] = referenced[b] = referenced[c] = 1;
        }
        return referenced;
    }

    inline std::array<float, 4> skinVertexWeights(const std::uint8_t* vertex, std::uint32_t skinOffset)
    {
        const auto* weights = reinterpret_cast<const std::uint16_t*>(vertex + skinOffset);
        const float w0 = halfToFloat(weights[0]), w1 = halfToFloat(weights[1]), w2 = halfToFloat(weights[2]);
        return { w0, w1, w2, 1.0f - w0 - w1 - w2 };
    }

    inline std::vector<std::uint8_t> referencedSkinBones(
        const std::uint8_t* vertices, std::uint32_t stride, std::uint32_t skinOffset,
        std::span<const std::uint8_t> referencedVertices, std::size_t boneCount)
    {
        std::vector<std::uint8_t> referenced(boneCount, 0);
        for (std::size_t vi = 0; vi < referencedVertices.size(); ++vi) {
            if (!referencedVertices[vi]) continue;
            const auto* vertex = vertices + vi * stride;
            const auto weights = skinVertexWeights(vertex, skinOffset);
            const auto* indices = vertex + skinOffset + 8;
            for (std::size_t k = 0; k < weights.size(); ++k) {
                if (std::isfinite(weights[k]) && weights[k] > 0.0f && indices[k] < boneCount) {
                    referenced[indices[k]] = 1;
                }
            }
        }
        return referenced;
    }

    inline int extractTrianglesFromSkinnedTriShape(
        RE::BSTriShape* triShape,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles = nullptr,
        bool allowPositionOnlySkinnedSurface = false,
        std::vector<TriangleData>* outLocalTriangles = nullptr)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::MeshSkinnedExtraction);
        [[maybe_unused]] const char* shapeName = triShape->name.c_str() ? triShape->name.c_str() : "(null)";
        const bool dynamicSkinned = isDynamicTriShape(triShape);

        void* rendererData = nullptr;
        if (!native_memory::tryReadField(triShape, VROffset::rendererData, rendererData)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable rendererData field", shapeName);
            return 0;
        }
        if (!rendererData) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null rendererData", shapeName);
            return 0;
        }

        std::uint32_t numTris = 0;
        std::uint16_t numVerts = 0;
        std::uint64_t vtxDescRaw = 0;
        if (!native_memory::tryReadField(triShape, VROffset::numTriangles, numTris) ||
            !native_memory::tryReadField(triShape, VROffset::numVertices, numVerts) ||
            !native_memory::tryReadField(triShape, VROffset::vertexDesc, vtxDescRaw)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable geometry fields", shapeName);
            return 0;
        }
        std::uint32_t vtxStride = static_cast<std::uint32_t>(vtxDescRaw & 0xF) * 4;

        std::uint32_t posOffset = static_cast<std::uint32_t>((vtxDescRaw >> 2) & 0x3C);
        bool fullPrecision = ((vtxDescRaw >> 54) & 1) != 0;

        std::uint32_t skinOffset = static_cast<std::uint32_t>((vtxDescRaw >> 26) & 0x3C);

        std::uint32_t minPosSize = fullPrecision ? (posOffset + 12) : (posOffset + 6);
        std::uint32_t minSkinEnd = skinOffset + 12;
        std::uint32_t minStride = dynamicSkinned ? minSkinEnd : (std::max)(minPosSize, minSkinEnd);
        if (numTris == 0 || numTris > kMaxMeshExtractionTriangles || numVerts == 0 || vtxStride < minStride) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': bad geometry (tris={}, verts={}, stride={}, minNeeded={})", shapeName, numTris, numVerts, vtxStride, minStride);
            return 0;
        }

        std::uint32_t dynamicStride = 0;
        if (dynamicSkinned) {
            dynamicStride = static_cast<std::uint32_t>((vtxDescRaw >> 2) & 0x3C);
            if (dynamicStride < 6) {
                ROCK_LOG_WARN(MeshGrab, "Skinned '{}': bad dynamic stride {}", shapeName, dynamicStride);
                return 0;
            }
        }

        void* vertexDataPtr = nullptr;
        void* triangleDataPtr = nullptr;
        if (!native_memory::tryReadField(rendererData, 0x08, vertexDataPtr) || !native_memory::tryReadField(rendererData, 0x10, triangleDataPtr)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable vertex/triangle data pointers", shapeName);
            return 0;
        }
        if (!vertexDataPtr || !triangleDataPtr) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null vertex/triangle data", shapeName);
            return 0;
        }

        std::uint8_t* verts = nullptr;
        std::uint16_t* tris = nullptr;
        if (!native_memory::tryReadField(vertexDataPtr, 0x08, verts) || !native_memory::tryReadField(triangleDataPtr, 0x08, tris)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable CPU buffer pointers", shapeName);
            return 0;
        }
        if (!verts || !tris) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null CPU buffers", shapeName);
            return 0;
        }
        if (!vertexBufferRangeLooksReadable(verts, numVerts, vtxStride) ||
            !native_memory::pointerRangeLooksReadable(tris, static_cast<std::size_t>(numTris) * 3u * sizeof(std::uint16_t))) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable CPU buffers (tris={}, verts={}, stride={})", shapeName, numTris, numVerts, vtxStride);
            return 0;
        }

        std::uint32_t dynamicDataSize = 0;
        if (dynamicSkinned && !native_memory::tryReadField(triShape, VROffset::dynamicDataSize, dynamicDataSize)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable dynamic data size", shapeName);
            return 0;
        }
        const std::uint64_t requiredDynamicBytes = static_cast<std::uint64_t>(dynamicStride) * numVerts;
        if (dynamicSkinned && dynamicDataSize < requiredDynamicBytes) {
            ROCK_LOG_WARN(MeshGrab,
                "Skinned '{}': dynamic vertex buffer too small (size={}, required={}, verts={}, stride={})",
                shapeName,
                dynamicDataSize,
                requiredDynamicBytes,
                numVerts,
                dynamicStride);
            return 0;
        }

        DynamicTriShapeVertexLock dynamicVertexLock(dynamicSkinned ? triShape : nullptr);
        const std::uint8_t* dynamicVerts = dynamicVertexLock.vertices();
        if (dynamicSkinned && !dynamicVerts) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null dynamic vertex buffer", shapeName);
            return 0;
        }
        if (dynamicSkinned && !vertexBufferRangeLooksReadable(dynamicVerts, numVerts, dynamicStride)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable dynamic vertex buffer (verts={}, stride={})", shapeName, numVerts, dynamicStride);
            return 0;
        }

        RE::BSSkin::Instance* skinInst = nullptr;
        if (!native_memory::tryReadField(triShape, VROffset::skinInstance, skinInst)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable skinInstance field", shapeName);
            return 0;
        }
        if (!skinInst) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null skinInstance (should not happen)", shapeName);
            return 0;
        }

        // Verify the live image before using the corrected native skin layout.
        if (!nativeSkinLayoutVerified()) return 0;

        std::uint32_t boneCount = 0, nodeCount = 0, bindCount = 0;
        RE::NiAVObject** boneNodes = nullptr;
        const RE::NiTransform** worldTransforms = nullptr;
        void* boneData = nullptr;
        const char* skinToBoneArray = nullptr;
        const char* extraScales = nullptr;
        RE::NiAVObject* skinRoot = nullptr;
        if (!native_memory::tryReadField(skinInst, BSSkinOffset::worldTransformCount, boneCount) ||
            !native_memory::tryReadField(skinInst, BSSkinOffset::bonesCount, nodeCount) ||
            boneCount == 0 || boneCount > 512 || nodeCount > boneCount ||
            !native_memory::tryReadField(skinInst, BSSkinOffset::bonesData, boneNodes) ||
            !native_memory::tryReadField(skinInst, BSSkinOffset::worldTransforms, worldTransforms) ||
            !native_memory::tryReadField(skinInst, BSSkinOffset::boneData, boneData) ||
            !native_memory::tryReadField(boneData, BSSkinOffset::transformArrayData, skinToBoneArray) ||
            !native_memory::tryReadField(boneData, BSSkinOffset::transformArrayCount, bindCount) ||
            bindCount < boneCount || bindCount > 512 || !worldTransforms || !skinToBoneArray ||
            !native_memory::tryReadField(skinInst, 0x48, skinRoot) ||
            !native_memory::tryReadField(skinInst, BSSkinOffset::extraScale, extraScales)) {
            ROCK_LOG_SAMPLE_WARN(MeshGrab, 2000, "Skinned '{}': invalid live palette arrays nodes={} worlds={} binds={}",
                shapeName, nodeCount, boneCount, bindCount);
            return 0;
        }

        std::vector<std::uint8_t> visibleTriangles;
        if (!readVisibleTriangles(triShape, numTris, visibleTriangles)) return 0;
        const auto referencedVertices = referencedMeshVertices(
            { tris, static_cast<std::size_t>(numTris) * 3 }, numVerts, visibleTriangles);
        const auto referencedBones = referencedSkinBones(verts, vtxStride, skinOffset, referencedVertices, boneCount);
        const auto evaluatedVertices = std::count(referencedVertices.begin(), referencedVertices.end(), 1);
        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshSkinnedVerticesSource, numVerts);
        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshSkinnedVerticesEvaluated, evaluatedVertices);
        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshSkinnedBonesSource, boneCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshSkinnedBonesEvaluated,
            std::count(referencedBones.begin(), referencedBones.end(), 1));
        if (evaluatedVertices == 0) return 0;

        struct BoneCombined
        {
            skinned_surface_math::Affine matrix{};
            bool valid = false;
        };
        std::vector<BoneCombined> boneTransforms(boneCount);
        std::vector<RE::NiAVObject*> boneNodesByIndex(boneCount, nullptr);
        std::uint32_t invalidBoneNodePointers = 0;
        std::uint32_t invalidBoneTransforms = 0;
        for (std::uint32_t b = 0; b < boneCount; ++b) {
            if (!referencedBones[b]) continue;
            const RE::NiTransform* worldTransform = nullptr;
            skinned_surface_math::Transform boneWorld{}, skinToBone{};
            std::array<float, 3> extraScale{};
            if (!native_memory::tryReadValue(worldTransforms + b, worldTransform) || !worldTransform ||
                !native_memory::guardedCopyFromMemory(worldTransform, boneWorld.data(), sizeof(boneWorld)) ||
                !native_memory::guardedCopyFromMemory(skinToBoneArray + b * 0x50 + 0x10, skinToBone.data(), sizeof(skinToBone)) ||
                (extraScales && !native_memory::guardedCopyFromMemory(extraScales + b * 0x10, extraScale.data(), sizeof(extraScale))) ||
                !skinned_surface_math::worldFromSkin(boneWorld, skinToBone, extraScale, boneTransforms[b].matrix)) {
                ++invalidBoneTransforms;
                continue;
            }
            boneTransforms[b].valid = true;
            RE::NiAVObject* boneNode = nullptr;
            if (b < nodeCount && boneNodes && native_memory::tryReadValue(boneNodes + b, boneNode) && boneNode &&
                native_memory::pointerRangeLooksReadable(boneNode, sizeof(RE::NiAVObject))) {
                boneNodesByIndex[b] = boneNode;
            } else {
                boneNodesByIndex[b] = resolveFlattenedSkinBoneOwner(skinRoot, worldTransform);
                if (!boneNodesByIndex[b]) ++invalidBoneNodePointers;
            }
        }

        ROCK_LOG_DEBUG(MeshGrab, "Skinned '{}': extracting {} tris, {} verts, {} bones (stride={}, skinOff={}, fullPrec={}, dynamic={})", shapeName, numTris, numVerts,
            boneCount, vtxStride, skinOffset, fullPrecision ? 1 : 0, dynamicSkinned ? 1 : 0);

        std::vector<RE::NiPoint3> worldVerts(numVerts);
        std::vector<RE::NiPoint3> dynamicLocalVerts;
        if (dynamicSkinned && outLocalTriangles) {
            dynamicLocalVerts.resize(numVerts);
        }
        std::vector<std::uint8_t> worldVertexValid(numVerts, 0);
        std::vector<std::uint8_t> vertexSkinInfluencesValid(numVerts, 0);
        std::vector<std::array<GrabSurfaceVertexInfluence, 4>> vertexInfluences(numVerts);
        std::uint32_t invalidSkinnedVertices = 0;
        std::uint32_t positionOnlySkinnedVertices = 0;
        for (std::uint16_t vi = 0; vi < numVerts; vi++) {
            if (!referencedVertices[vi]) continue;
            const std::uint8_t* vtx = verts + vi * vtxStride;

            const RE::NiPoint3 bindPos = dynamicSkinned ?
                readDynamicVertexPosition(dynamicVerts + vi * dynamicStride) :
                readVertexPosition(vtx, posOffset, fullPrecision);

            const std::uint8_t* idxPtr = vtx + skinOffset + 8;
            std::uint8_t bi0 = idxPtr[0];
            std::uint8_t bi1 = idxPtr[1];
            std::uint8_t bi2 = idxPtr[2];
            std::uint8_t bi3 = idxPtr[3];

            const auto weights = skinVertexWeights(vtx, skinOffset);
            std::array<const skinned_surface_math::Affine*,4> weightedMatrices{};
            std::uint8_t indices[4] = { bi0, bi1, bi2, bi3 };

            float validWeight = 0.0f;
            bool missingWeightedBone = false;
            bool hasWeightedBoneInfluence = false;
            bool completeBoneOwners = true;
            for (int k = 0; k < 4; k++) {
                float w = weights[k];
                if (!std::isfinite(w) || w <= 0.0f)
                    continue;

                std::uint8_t bIdx = indices[k];
                if (bIdx >= boneCount) {
                    missingWeightedBone = true;
                    continue;
                }
                if (!boneTransforms[bIdx].valid) {
                    missingWeightedBone = true;
                    continue;
                }
                weightedMatrices[k] = &boneTransforms[bIdx].matrix;
                validWeight += w;
                hasWeightedBoneInfluence = hasWeightedBoneInfluence || boneNodesByIndex[bIdx] != nullptr;
                completeBoneOwners = completeBoneOwners && boneNodesByIndex[bIdx] != nullptr;
                vertexInfluences[vi][k] = GrabSurfaceVertexInfluence{ boneNodesByIndex[bIdx], w };
            }

            if (missingWeightedBone || validWeight <= 0.00001f ||
                !skinned_surface_math::blendVertex(weightedMatrices, weights, bindPos, triShape->world.translate, worldVerts[vi])) {
                ++invalidSkinnedVertices;
                continue;
            }
            if (dynamicSkinned && outLocalTriangles) {
                dynamicLocalVerts[vi] = transform_math::worldPointToLocal(triShape->world, worldVerts[vi]);
            }
            if (hasWeightedBoneInfluence && completeBoneOwners) {
                vertexSkinInfluencesValid[vi] = 1;
            } else {
                ++positionOnlySkinnedVertices;
                if (outSurfaceTriangles && !allowPositionOnlySkinnedSurface) continue;
            }
            worldVertexValid[vi] = 1;
        }

        int added = 0;
        std::uint32_t skippedInvalidVertexTriangles = 0;
        for (std::uint32_t i = 0; i < numTris; i++) {
            if (!visibleTriangles.empty() && !visibleTriangles[i]) continue;
            std::uint16_t i0 = tris[i * 3 + 0];
            std::uint16_t i1 = tris[i * 3 + 1];
            std::uint16_t i2 = tris[i * 3 + 2];

            if (i0 >= numVerts || i1 >= numVerts || i2 >= numVerts)
                continue;
            if (!worldVertexValid[i0] || !worldVertexValid[i1] || !worldVertexValid[i2]) {
                ++skippedInvalidVertexTriangles;
                continue;
            }

            TriangleData tri;
            tri.v0 = worldVerts[i0];
            tri.v1 = worldVerts[i1];
            tri.v2 = worldVerts[i2];

            outTriangles.push_back(tri);
            if (dynamicSkinned && outLocalTriangles) {
                outLocalTriangles->push_back(TriangleData{
                    .v0 = dynamicLocalVerts[i0],
                    .v1 = dynamicLocalVerts[i1],
                    .v2 = dynamicLocalVerts[i2],
                });
            }
            std::array<std::array<GrabSurfaceVertexInfluence, 4>, 3> skinInfluences{};
            skinInfluences[0] = vertexInfluences[i0];
            skinInfluences[1] = vertexInfluences[i1];
            skinInfluences[2] = vertexInfluences[i2];
            const bool triangleHasSkinInfluences =
                vertexSkinInfluencesValid[i0] && vertexSkinInfluencesValid[i1] && vertexSkinInfluencesValid[i2] && hasAnySkinInfluence(skinInfluences);
            appendSurfaceTriangle(outSurfaceTriangles, tri, triShape, i, GrabSurfaceSourceKind::Skinned, triangleHasSkinInfluences ? &skinInfluences : nullptr);
            added++;
        }

        if (invalidBoneNodePointers != 0 || invalidBoneTransforms != 0 || invalidSkinnedVertices != 0 || skippedInvalidVertexTriangles != 0) {
            ROCK_LOG_SAMPLE_WARN(MeshGrab, 2000,
                "Skinned '{}': skipped invalid data bonePtrs={} boneTransforms={} vertices={} triangles={}",
                shapeName,
                invalidBoneNodePointers,
                invalidBoneTransforms,
                invalidSkinnedVertices,
                skippedInvalidVertexTriangles);
        }
        ROCK_LOG_DEBUG(MeshGrab, "Skinned '{}': extracted {} triangles (positionOnlyVertices={})", shapeName, added, positionOnlySkinnedVertices);
        return added;
    }

    inline void extractAllTriangles(RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        int maxDepth = 10,
        MeshExtractionStats* stats = nullptr)
    {
        if (!root || maxDepth <= 0)
            return;

        if (root->flags.flags & 1)
            return;

        auto* triShape = root->IsTriShape();
        if (triShape) {
            if (stats) {
                stats->visitedShapes++;
            }

            const auto before = outTriangles.size();
            const bool dynamic = isDynamicTriShape(triShape);
            const bool skinned = isSkinned(triShape);

            if (skinned) {
                if (stats) {
                    stats->skinnedShapes++;
                }
                extractTrianglesFromSkinnedTriShape(triShape, outTriangles);
                const auto added = static_cast<std::uint32_t>(outTriangles.size() - before);
                if (stats) {
                    stats->skinnedTriangles += added;
                    if (dynamic && added == 0) {
                        stats->dynamicSkinnedSkipped++;
                    }
                }
            } else if (dynamic) {
                if (stats) {
                    stats->dynamicShapes++;
                }
                extractTrianglesFromTriShape(triShape, outTriangles);
                if (stats) {
                    stats->dynamicTriangles += static_cast<std::uint32_t>(outTriangles.size() - before);
                }
            } else {
                if (stats) {
                    stats->staticShapes++;
                }
                extractTrianglesFromTriShape(triShape, outTriangles);
                if (stats) {
                    stats->staticTriangles += static_cast<std::uint32_t>(outTriangles.size() - before);
                }
            }

            if (stats && outTriangles.size() == before) {
                stats->emptyShapes++;
            }
            return;
        }

        auto* niNode = root->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            for (auto i = decltype(kids.size()){ 0 }; i < kids.size(); i++) {
                auto* kid = kids[i].get();
                if (kid)
                    extractAllTriangles(kid, outTriangles, maxDepth - 1, stats);
            }
        }
    }

    inline void extractSurfaceTrianglesFromTriShape(RE::BSTriShape* triShape,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>& outSurfaceTriangles,
        MeshExtractionStats* stats,
        bool allowPositionOnlySkinnedSurface)
    {
        if (!triShape) {
            return;
        }

        if (stats) {
            stats->visitedShapes++;
        }

        const auto before = outTriangles.size();
        const bool dynamic = isDynamicTriShape(triShape);
        const bool skinned = isSkinned(triShape);

        if (skinned) {
            if (stats) {
                stats->skinnedShapes++;
            }
            extractTrianglesFromSkinnedTriShape(triShape, outTriangles, &outSurfaceTriangles, allowPositionOnlySkinnedSurface);
            const auto added = static_cast<std::uint32_t>(outTriangles.size() - before);
            if (stats) {
                stats->skinnedTriangles += added;
                if (dynamic && added == 0) {
                    stats->dynamicSkinnedSkipped++;
                }
            }
        } else if (dynamic) {
            if (stats) {
                stats->dynamicShapes++;
            }
            extractTrianglesFromTriShape(triShape, outTriangles, &outSurfaceTriangles);
            if (stats) {
                stats->dynamicTriangles += static_cast<std::uint32_t>(outTriangles.size() - before);
            }
        } else {
            if (stats) {
                stats->staticShapes++;
            }
            extractTrianglesFromTriShape(triShape, outTriangles, &outSurfaceTriangles);
            if (stats) {
                stats->staticTriangles += static_cast<std::uint32_t>(outTriangles.size() - before);
            }
        }

        if (stats && outTriangles.size() == before) {
            stats->emptyShapes++;
        }
    }

    inline void extractAllSurfaceTrianglesRecursive(RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>& outSurfaceTriangles,
        int maxDepth = 10,
        MeshExtractionStats* stats = nullptr,
        bool allowPositionOnlySkinnedSurface = false)
    {
        if (!root || maxDepth <= 0)
            return;

        if (root->flags.flags & 1)
            return;

        auto* triShape = root->IsTriShape();
        if (triShape) {
            extractSurfaceTrianglesFromTriShape(triShape, outTriangles, outSurfaceTriangles, stats, allowPositionOnlySkinnedSurface);
            return;
        }

        auto* niNode = root->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            for (auto i = decltype(kids.size()){ 0 }; i < kids.size(); i++) {
                auto* kid = kids[i].get();
                if (kid)
                    extractAllSurfaceTrianglesRecursive(kid, outTriangles, outSurfaceTriangles, maxDepth - 1, stats, allowPositionOnlySkinnedSurface);
            }
        }
    }

    inline void extractAllSurfaceTrianglesWithScenegraphVisitor(RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>& outSurfaceTriangles,
        MeshExtractionStats* stats = nullptr,
        bool allowPositionOnlySkinnedSurface = false)
    {
        if (!root) {
            return;
        }

        /*
         * Some loaded objects expose render geometry through the engine geometry
         * visitor even when the selected root node itself is culled or otherwise
         * not useful to the hand-rolled NiNode walk. Keep this as a zero-triangle
         * recovery path so strict grab still requires real render geometry.
         */
        RE::BSVisit::TraverseScenegraphGeometries(root, [&](RE::BSGeometry* geometry) -> RE::BSVisit::BSVisitControl {
            if (!geometry) {
                return RE::BSVisit::BSVisitControl::kContinue;
            }

            auto* triShape = geometry->IsTriShape();
            if (!triShape) {
                return RE::BSVisit::BSVisitControl::kContinue;
            }

            if (triShape->flags.flags & 1) {
                return RE::BSVisit::BSVisitControl::kContinue;
            }

            extractSurfaceTrianglesFromTriShape(triShape, outTriangles, outSurfaceTriangles, stats, allowPositionOnlySkinnedSurface);
            return RE::BSVisit::BSVisitControl::kContinue;
        });
    }

    inline void extractAllSurfaceTriangles(RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>& outSurfaceTriangles,
        int maxDepth = 10,
        MeshExtractionStats* stats = nullptr,
        bool allowPositionOnlySkinnedSurface = false)
    {
        const auto beforeTriangles = outTriangles.size();
        extractAllSurfaceTrianglesRecursive(root, outTriangles, outSurfaceTriangles, maxDepth, stats, allowPositionOnlySkinnedSurface);
        if (outTriangles.size() != beforeTriangles || !root || maxDepth <= 0) {
            return;
        }

        extractAllSurfaceTrianglesWithScenegraphVisitor(root, outTriangles, outSurfaceTriangles, stats, allowPositionOnlySkinnedSurface);
    }

    struct BoundedSurfaceMeshExtraction
    {
        MeshExtractionStats stats{};
        std::uint32_t candidateShapes = 0;
        std::uint32_t examinedTriangles = 0;
        std::uint32_t visitedNodes = 0;
        bool nodeBudgetExceeded = false;
        bool shapeBudgetExceeded = false;
        bool triangleBudgetExceeded = false;

        [[nodiscard]] bool hasTriangles() const noexcept
        {
            return stats.totalTriangles() != 0;
        }
    };

    inline void extractBoundedSurfaceTrianglesRecursive(
        RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>& outSurfaceTriangles,
        int maxDepth,
        std::uint32_t maxShapes,
        std::uint32_t maxTriangles,
        BoundedSurfaceMeshExtraction& result,
        bool allowPositionOnlySkinnedSurface = false)
    {
        if (!root || maxDepth <= 0 || maxShapes == 0 || maxTriangles == 0) {
            return;
        }
        if (result.visitedNodes >= 4096) {
            result.nodeBudgetExceeded = true;
            return;
        }
        ++result.visitedNodes;
        if (root->flags.flags & 1) {
            return;
        }
        if (auto* triShape = root->IsTriShape()) {
            if (result.nodeBudgetExceeded || result.candidateShapes >= maxShapes) {
                result.shapeBudgetExceeded = true;
                return;
            }
            ++result.candidateShapes;

            TriShapeRawGeometry geometry{};
            if (!readTriShapeRawGeometry(triShape, geometry)) {
                ++result.stats.emptyShapes;
                return;
            }
            const std::uint32_t remaining =
                result.examinedTriangles < maxTriangles ?
                maxTriangles - result.examinedTriangles :
                0;
            if (geometry.numTriangles > remaining) {
                result.triangleBudgetExceeded = true;
                return;
            }

            result.examinedTriangles += geometry.numTriangles;
            extractSurfaceTrianglesFromTriShape(
                triShape,
                outTriangles,
                outSurfaceTriangles,
                &result.stats,
                allowPositionOnlySkinnedSurface);
            return;
        }

        auto* node = root->IsNode();
        if (!node) {
            return;
        }
        auto& children = node->GetRuntimeData().children;
        for (auto index = decltype(children.size()){ 0 };
             index < children.size();
             ++index) {
            if (result.nodeBudgetExceeded) return;
            if (result.candidateShapes >= maxShapes) {
                result.shapeBudgetExceeded = true;
                return;
            }
            auto* child = children[index].get();
            if (!child) {
                continue;
            }
            extractBoundedSurfaceTrianglesRecursive(
                child,
                outTriangles,
                outSurfaceTriangles,
                maxDepth - 1,
                maxShapes,
                maxTriangles,
                result,
                allowPositionOnlySkinnedSurface);
        }
    }

    inline BoundedSurfaceMeshExtraction extractBoundedSurfaceTriangles(
        RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>& outSurfaceTriangles,
        int maxDepth,
        std::uint32_t maxShapes,
        std::uint32_t maxTriangles,
        bool allowPositionOnlySkinnedSurface = false)
    {
        BoundedSurfaceMeshExtraction result{};
        extractBoundedSurfaceTrianglesRecursive(
            root,
            outTriangles,
            outSurfaceTriangles,
            maxDepth,
            maxShapes,
            maxTriangles,
            result,
            allowPositionOnlySkinnedSurface);
        return result;
    }

    inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b) { return vector_math::dot(a, b); }

    inline RE::NiPoint3 cross(const RE::NiPoint3& a, const RE::NiPoint3& b) { return vector_math::cross(a, b); }

    inline RE::NiPoint3 normalize(const RE::NiPoint3& v)
    {
        float len = std::sqrt(vector_math::lengthSquared(v));
        if (len < 1e-8f)
            return RE::NiPoint3(0, 0, 0);
        return RE::NiPoint3(v.x / len, v.y / len, v.z / len);
    }

    inline RE::NiPoint3 sub(const RE::NiPoint3& a, const RE::NiPoint3& b) { return RE::NiPoint3(a.x - b.x, a.y - b.y, a.z - b.z); }

    inline RE::NiPoint3 closestPointOnTriangleToPoint(const RE::NiPoint3& point, const TriangleData& tri, float& outDistSq)
    {
        const RE::NiPoint3 ab = sub(tri.v1, tri.v0);
        const RE::NiPoint3 ac = sub(tri.v2, tri.v0);
        const RE::NiPoint3 ap = sub(point, tri.v0);
        const float d1 = dot(ab, ap);
        const float d2 = dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) {
            outDistSq = dot(sub(point, tri.v0), sub(point, tri.v0));
            return tri.v0;
        }

        const RE::NiPoint3 bp = sub(point, tri.v1);
        const float d3 = dot(ab, bp);
        const float d4 = dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {
            outDistSq = dot(sub(point, tri.v1), sub(point, tri.v1));
            return tri.v1;
        }

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            const float v = d1 / (d1 - d3);
            const RE::NiPoint3 result{ tri.v0.x + ab.x * v, tri.v0.y + ab.y * v, tri.v0.z + ab.z * v };
            outDistSq = dot(sub(point, result), sub(point, result));
            return result;
        }

        const RE::NiPoint3 cp = sub(point, tri.v2);
        const float d5 = dot(ab, cp);
        const float d6 = dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            outDistSq = dot(sub(point, tri.v2), sub(point, tri.v2));
            return tri.v2;
        }

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            const float w = d2 / (d2 - d6);
            const RE::NiPoint3 result{ tri.v0.x + ac.x * w, tri.v0.y + ac.y * w, tri.v0.z + ac.z * w };
            outDistSq = dot(sub(point, result), sub(point, result));
            return result;
        }

        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            const RE::NiPoint3 bc = sub(tri.v2, tri.v1);
            const RE::NiPoint3 result{ tri.v1.x + bc.x * w, tri.v1.y + bc.y * w, tri.v1.z + bc.z * w };
            outDistSq = dot(sub(point, result), sub(point, result));
            return result;
        }

        const float denom = 1.0f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        const RE::NiPoint3 result{ tri.v0.x + ab.x * v + ac.x * w, tri.v0.y + ab.y * v + ac.y * w, tri.v0.z + ab.z * v + ac.z * w };
        outDistSq = dot(sub(point, result), sub(point, result));
        return result;
    }

    inline RE::NiPoint3 closestPointOnTriangleToLine(const RE::NiPoint3& linePoint, const RE::NiPoint3& lineDir, const TriangleData& tri, float& outDistSq)
    {
        RE::NiPoint3 edge0 = sub(tri.v1, tri.v0);
        RE::NiPoint3 edge1 = sub(tri.v2, tri.v0);
        RE::NiPoint3 triNormal = normalize(cross(edge0, edge1));

        float denom = dot(triNormal, lineDir);
        float t = 0.0f;
        if (std::abs(denom) > 1e-8f) {
            t = dot(sub(tri.v0, linePoint), triNormal) / denom;
        }

        RE::NiPoint3 lineOnPlane(linePoint.x + lineDir.x * t, linePoint.y + lineDir.y * t, linePoint.z + lineDir.z * t);

        RE::NiPoint3 v0ToP = sub(lineOnPlane, tri.v0);
        float d00 = dot(edge0, edge0);
        float d01 = dot(edge0, edge1);
        float d11 = dot(edge1, edge1);
        float d20 = dot(v0ToP, edge0);
        float d21 = dot(v0ToP, edge1);

        float baryDenom = d00 * d11 - d01 * d01;
        if (std::abs(baryDenom) < 1e-12f) {
            outDistSq = (std::numeric_limits<float>::max)();
            return tri.v0;
        }
        float invDenom = 1.0f / baryDenom;
        float u = (d11 * d20 - d01 * d21) * invDenom;
        float v = (d00 * d21 - d01 * d20) * invDenom;

        if (u >= 0.0f && v >= 0.0f && u + v <= 1.0f) {
            outDistSq = dot(sub(lineOnPlane, linePoint), sub(lineOnPlane, linePoint));
            return lineOnPlane;
        }

        auto closestOnSegment = [](const RE::NiPoint3& p, const RE::NiPoint3& a, const RE::NiPoint3& b) -> RE::NiPoint3 {
            RE::NiPoint3 ab = sub(b, a);
            float t2 = dot(sub(p, a), ab) / dot(ab, ab);
            t2 = (std::max)(0.0f, (std::min)(1.0f, t2));
            return RE::NiPoint3(a.x + ab.x * t2, a.y + ab.y * t2, a.z + ab.z * t2);
        };

        RE::NiPoint3 c0 = closestOnSegment(lineOnPlane, tri.v0, tri.v1);
        RE::NiPoint3 c1 = closestOnSegment(lineOnPlane, tri.v1, tri.v2);
        RE::NiPoint3 c2 = closestOnSegment(lineOnPlane, tri.v2, tri.v0);

        RE::NiPoint3 d0 = sub(c0, lineOnPlane);
        RE::NiPoint3 d1 = sub(c1, lineOnPlane);
        RE::NiPoint3 d2 = sub(c2, lineOnPlane);

        float dist0 = dot(d0, d0);
        float dist1 = dot(d1, d1);
        float dist2 = dot(d2, d2);

        RE::NiPoint3 closest = c0;
        float minDist = dist0;
        if (dist1 < minDist) {
            closest = c1;
            minDist = dist1;
        }
        if (dist2 < minDist) {
            closest = c2;
            minDist = dist2;
        }

        RE::NiPoint3 toClosest = sub(closest, linePoint);
        outDistSq = dot(toClosest, toClosest);
        return closest;
    }

    struct GrabPoint
    {
        RE::NiPoint3 position;
        RE::NiPoint3 normal;
        int triangleIndex = -1;
        float distance = 1e30f;
        float signedAlongPalmDistanceGameUnits = 0.0f;
        float lateralPalmDistanceGameUnits = 0.0f;
    };

    inline RE::NiAVObject* resolveDominantSurfaceOwnerNode(const GrabSurfaceTriangleData& surfaceTriangle, const RE::NiPoint3& hitPoint)
    {
        if (!surfaceTriangle.hasSkinInfluences) {
            return surfaceTriangle.sourceNode;
        }

        // Three vertices, four influences each. Preserve influence order for
        // deterministic ties at a joint instead of relying on hash iteration.
        std::array<std::pair<RE::NiAVObject*, float>, 12> accumulated{};
        std::size_t ownerCount = 0;
        const RE::NiPoint3 vertices[3] = { surfaceTriangle.triangle.v0, surfaceTriangle.triangle.v1, surfaceTriangle.triangle.v2 };
        for (std::size_t vertex = 0; vertex < 3; ++vertex) {
            const RE::NiPoint3 delta = sub(vertices[vertex], hitPoint);
            float distance = std::sqrt(dot(delta, delta));
            if (distance < 0.001f) {
                distance = 0.001f;
            }
            const float distanceWeight = 1.0f / distance;
            for (const auto& influence : surfaceTriangle.skinInfluences[vertex]) {
                if (!influence.bone || influence.weight <= 0.0f) {
                    continue;
                }
                std::size_t owner = 0;
                for (; owner < ownerCount; ++owner) if (accumulated[owner].first == influence.bone) break;
                if (owner == ownerCount) accumulated[ownerCount++].first = influence.bone;
                accumulated[owner].second += influence.weight * distanceWeight;
            }
        }

        RE::NiAVObject* best = nullptr;
        float bestWeight = 0.0f;
        for (const auto& [bone, weight] : accumulated) {
            if (bone && weight > bestWeight) {
                best = bone;
                bestWeight = weight;
            }
        }
        return best ? best : surfaceTriangle.sourceNode;
    }

    template <class TriangleRange>
    inline bool findClosestGrabPoint(const TriangleRange& triangles,
        const RE::NiPoint3& palmPos,
        const RE::NiPoint3& palmDir,
        float lateralWeight,
        float directionalWeight,
        GrabPoint& outResult,
        float behindPalmToleranceGameUnits = 0.0f,
        int* outRejectedBehindPalm = nullptr)
    {
        float bestDist = 1e30f;
        int bestIdx = -1;
        RE::NiPoint3 bestPos;
        float bestSignedAlong = 0.0f;
        float bestLateral = 0.0f;
        const float behindTolerance = std::max(0.0f, std::isfinite(behindPalmToleranceGameUnits) ? behindPalmToleranceGameUnits : 0.0f);

        for (int i = 0; i < static_cast<int>(triangles.size()); i++) {
            const auto& tri = triangles[i];

            RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));

            if (dot(triNormal, palmDir) > 0.0f)
                continue;

            float distSq;
            RE::NiPoint3 closest = closestPointOnTriangleToLine(palmPos, palmDir, tri, distSq);

            RE::NiPoint3 toClosest = sub(closest, palmPos);
            float dirComponent = dot(toClosest, palmDir);
            if (dirComponent < -behindTolerance) {
                if (outRejectedBehindPalm) {
                    ++(*outRejectedBehindPalm);
                }
                continue;
            }
            RE::NiPoint3 dirVec(palmDir.x * dirComponent, palmDir.y * dirComponent, palmDir.z * dirComponent);
            RE::NiPoint3 latVec = sub(toClosest, dirVec);

            float dirDist = dirComponent * dirComponent;
            float latDist = dot(latVec, latVec);
            float weightedDist = directionalWeight * dirDist + lateralWeight * latDist;

            if (weightedDist < bestDist) {
                bestDist = weightedDist;
                bestIdx = i;
                bestPos = closest;
                bestSignedAlong = dirComponent;
                bestLateral = std::sqrt(latDist);
            }
        }

        if (bestIdx >= 0) {
            const auto& tri = triangles[bestIdx];
            outResult.position = bestPos;
            outResult.normal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));
            outResult.triangleIndex = bestIdx;
            outResult.distance = bestDist;
            outResult.signedAlongPalmDistanceGameUnits = bestSignedAlong;
            outResult.lateralPalmDistanceGameUnits = bestLateral;
            return true;
        }
        return false;
    }

    inline bool findClosestGrabSurfaceHit(const std::vector<GrabSurfaceTriangleData>& triangles,
        const RE::NiPoint3& palmPos,
        const RE::NiPoint3& palmDir,
        float lateralWeight,
        float directionalWeight,
        GrabSurfaceHit& outResult,
        float behindPalmToleranceGameUnits = 0.0f,
        int* outRejectedBehindPalm = nullptr)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::MeshDirectionalQuery);
        float bestDist = 1e30f;
        int bestIdx = -1;
        RE::NiPoint3 bestPos;
        float bestSignedAlong = 0.0f;
        float bestLateral = 0.0f;
        const float behindTolerance = std::max(0.0f, std::isfinite(behindPalmToleranceGameUnits) ? behindPalmToleranceGameUnits : 0.0f);

        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshDirectionalQueryTriangles, triangles.size());
        for (int i = 0; i < static_cast<int>(triangles.size()); i++) {
            const auto& surfaceTriangle = triangles[i];
            const auto& tri = surfaceTriangle.triangle;

            RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));

            if (dot(triNormal, palmDir) > 0.0f)
                continue;

            float distSq;
            RE::NiPoint3 closest = closestPointOnTriangleToLine(palmPos, palmDir, tri, distSq);

            RE::NiPoint3 toClosest = sub(closest, palmPos);
            float dirComponent = dot(toClosest, palmDir);
            if (dirComponent < -behindTolerance) {
                if (outRejectedBehindPalm) {
                    ++(*outRejectedBehindPalm);
                }
                continue;
            }
            RE::NiPoint3 dirVec(palmDir.x * dirComponent, palmDir.y * dirComponent, palmDir.z * dirComponent);
            RE::NiPoint3 latVec = sub(toClosest, dirVec);

            float dirDist = dirComponent * dirComponent;
            float latDist = dot(latVec, latVec);
            float weightedDist = directionalWeight * dirDist + lateralWeight * latDist;

            if (weightedDist < bestDist) {
                bestDist = weightedDist;
                bestIdx = i;
                bestPos = closest;
                bestSignedAlong = dirComponent;
                bestLateral = std::sqrt(latDist);
            }
        }

        if (bestIdx >= 0) {
            const auto& surfaceTriangle = triangles[bestIdx];
            const auto& tri = surfaceTriangle.triangle;
            outResult.position = bestPos;
            outResult.normal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));
            outResult.triangleIndex = bestIdx;
            outResult.distance = bestDist;
            outResult.sourceNode = resolveDominantSurfaceOwnerNode(surfaceTriangle, bestPos);
            outResult.sourceShape = surfaceTriangle.sourceShape;
        outResult.sourceTriangleIndex = surfaceTriangle.triangleIndex;
            outResult.triangle = tri;
            outResult.sourceKind = surfaceTriangle.sourceKind;
            outResult.hasSkinInfluences = surfaceTriangle.hasSkinInfluences;
            outResult.hasTriangle = true;
            outResult.signedAlongPalmDistanceGameUnits = bestSignedAlong;
            outResult.lateralPalmDistanceGameUnits = bestLateral;
            outResult.valid = true;
            return true;
        }
        return false;
    }

    inline bool findClosestGrabSurfaceHitToPoint(const std::vector<GrabSurfaceTriangleData>& triangles,
        const RE::NiPoint3& point,
        const RE::NiPoint3& expectedNormal,
        float maxDistanceGameUnits,
        float maxNormalAngleDegrees,
        GrabSurfaceHit& outResult,
        const GrabSurfaceQueryIndex* queryIndex = nullptr)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::MeshPointQuery);
        const RE::NiPoint3 expected = normalize(expectedNormal);
        if (dot(expected, expected) <= 0.0f || maxDistanceGameUnits < 0.0f) {
            return false;
        }

        const float maxDistSq = maxDistanceGameUnits * maxDistanceGameUnits;
        const float minNormalDot = std::cos(std::clamp(maxNormalAngleDegrees, 0.0f, 179.0f) * 3.14159265358979323846f / 180.0f);
        float bestDistSq = (std::numeric_limits<float>::max)();
        int bestIdx = -1;
        RE::NiPoint3 bestPoint{};
        RE::NiPoint3 bestNormal{};

        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshPointQueryTriangles, triangles.size());
        const auto testTriangle = [&](std::size_t i) {
            const auto& surfaceTriangle = triangles[i];
            const auto& tri = surfaceTriangle.triangle;
            RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v0)));
            if (dot(triNormal, triNormal) <= 0.0f) {
                return;
            }
            if (dot(triNormal, expected) < 0.0f) {
                triNormal = RE::NiPoint3{ -triNormal.x, -triNormal.y, -triNormal.z };
            }
            if (dot(triNormal, expected) < minNormalDot) {
                return;
            }

            float distSq = 0.0f;
            const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(point, tri, distSq);
            if (distSq > maxDistSq || distSq >= bestDistSq) {
                return;
            }

            bestDistSq = distSq;
            bestIdx = static_cast<int>(i);
            bestPoint = candidate;
            bestNormal = triNormal;
        };
        std::size_t tested = triangles.size();
        if (queryIndex) {
            tested = queryIndex->visit(triangles, point,
                [&] { return (std::min)(maxDistSq, bestDistSq); }, testTriangle);
        } else {
            for (std::size_t i = 0; i < triangles.size(); ++i) testTriangle(i);
        }
        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshPointQueryTriangleTests, tested);

        if (bestIdx < 0) {
            return false;
        }

        const auto& surfaceTriangle = triangles[bestIdx];
        outResult.position = bestPoint;
        outResult.normal = bestNormal;
        outResult.triangleIndex = bestIdx;
        outResult.distance = bestDistSq;
        outResult.sourceNode = resolveDominantSurfaceOwnerNode(surfaceTriangle, bestPoint);
        outResult.sourceShape = surfaceTriangle.sourceShape;
        outResult.sourceTriangleIndex = surfaceTriangle.triangleIndex;
        outResult.triangle = surfaceTriangle.triangle;
        outResult.sourceKind = surfaceTriangle.sourceKind;
        outResult.hasSkinInfluences = surfaceTriangle.hasSkinInfluences;
        outResult.hasTriangle = true;
        outResult.valid = true;
        return true;
    }

    inline bool findClosestGrabSurfaceHitToPointPositionOnly(const std::vector<GrabSurfaceTriangleData>& triangles,
        const RE::NiPoint3& point,
        const RE::NiPoint3& preferredNormal,
        float maxDistanceGameUnits,
        GrabSurfaceHit& outResult,
        const GrabSurfaceQueryIndex* queryIndex = nullptr)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::MeshPointQuery);
        if (maxDistanceGameUnits < 0.0f || !std::isfinite(maxDistanceGameUnits)) {
            return false;
        }

        const RE::NiPoint3 preferred = normalize(preferredNormal);
        const bool hasPreferredNormal = dot(preferred, preferred) > 0.0f;
        const float maxDistSq = maxDistanceGameUnits * maxDistanceGameUnits;
        float bestDistSq = (std::numeric_limits<float>::max)();
        int bestIdx = -1;
        RE::NiPoint3 bestPoint{};
        RE::NiPoint3 bestNormal{};
        float bestSignedAlong = 0.0f;
        float bestLateral = 0.0f;

        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshPointQueryTriangles, triangles.size());
        const auto testTriangle = [&](std::size_t i) {
            const auto& surfaceTriangle = triangles[i];
            const auto& tri = surfaceTriangle.triangle;
            RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v0)));
            if (dot(triNormal, triNormal) <= 0.0f) {
                return;
            }
            if (hasPreferredNormal && dot(triNormal, preferred) < 0.0f) {
                triNormal = RE::NiPoint3{ -triNormal.x, -triNormal.y, -triNormal.z };
            }

            float distSq = 0.0f;
            const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(point, tri, distSq);
            if (distSq > maxDistSq || distSq >= bestDistSq) {
                return;
            }

            float signedAlong = 0.0f;
            float lateral = std::sqrt(distSq);
            if (hasPreferredNormal) {
                const RE::NiPoint3 toCandidate = sub(candidate, point);
                signedAlong = dot(toCandidate, preferred);
                const RE::NiPoint3 alongVec{ preferred.x * signedAlong, preferred.y * signedAlong, preferred.z * signedAlong };
                const RE::NiPoint3 lateralVec = sub(toCandidate, alongVec);
                lateral = std::sqrt((std::max)(0.0f, dot(lateralVec, lateralVec)));
            }

            bestDistSq = distSq;
            bestIdx = static_cast<int>(i);
            bestPoint = candidate;
            bestNormal = triNormal;
            bestSignedAlong = signedAlong;
            bestLateral = lateral;
        };
        std::size_t tested = triangles.size();
        if (queryIndex) {
            tested = queryIndex->visit(triangles, point,
                [&] { return (std::min)(maxDistSq, bestDistSq); }, testTriangle);
        } else {
            for (std::size_t i = 0; i < triangles.size(); ++i) testTriangle(i);
        }
        performance_profiler::observeValue(performance_profiler::ValueMetric::MeshPointQueryTriangleTests, tested);

        if (bestIdx < 0) {
            return false;
        }

        const auto& surfaceTriangle = triangles[bestIdx];
        outResult.position = bestPoint;
        outResult.normal = bestNormal;
        outResult.triangleIndex = bestIdx;
        outResult.distance = bestDistSq;
        outResult.sourceNode = resolveDominantSurfaceOwnerNode(surfaceTriangle, bestPoint);
        outResult.sourceShape = surfaceTriangle.sourceShape;
        outResult.sourceTriangleIndex = surfaceTriangle.triangleIndex;
        outResult.triangle = surfaceTriangle.triangle;
        outResult.sourceKind = surfaceTriangle.sourceKind;
        outResult.hasSkinInfluences = surfaceTriangle.hasSkinInfluences;
        outResult.hasTriangle = true;
        outResult.signedAlongPalmDistanceGameUnits = bestSignedAlong;
        outResult.lateralPalmDistanceGameUnits = bestLateral;
        outResult.valid = true;
        return true;
    }
}
