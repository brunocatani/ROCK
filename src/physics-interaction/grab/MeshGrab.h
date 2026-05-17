#pragma once

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/native/NativeMemory.h"
#include "RE/Fallout.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <unordered_map>
#include <vector>

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

    constexpr std::uint32_t kMaxMeshExtractionTriangles = 1'000'000;

    struct TriangleData
    {
        RE::NiPoint3 v0, v1, v2;

        void applyTransform(const RE::NiTransform& t)
        {
            auto xform = [&](const RE::NiPoint3& p) -> RE::NiPoint3 {
                /*
                 * Mesh extraction must match FO4VR's native NiTransform compose
                 * convention. Ghidra verification of the engine compose path
                 * showed child-local vectors are applied through the parent
                 * column basis. Weapon hulls are baked from these world
                 * triangles once at creation; using the opposite basis makes the
                 * baked hull depend on how tilted the weapon was when the
                 * collision was created.
                 */
                RE::NiPoint3 scaled(p.x * t.scale, p.y * t.scale, p.z * t.scale);
                RE::NiPoint3 rotated;
                rotated.x = t.rotate.entry[0][0] * scaled.x + t.rotate.entry[1][0] * scaled.y + t.rotate.entry[2][0] * scaled.z;
                rotated.y = t.rotate.entry[0][1] * scaled.x + t.rotate.entry[1][1] * scaled.y + t.rotate.entry[2][1] * scaled.z;
                rotated.z = t.rotate.entry[0][2] * scaled.x + t.rotate.entry[1][2] * scaled.y + t.rotate.entry[2][2] * scaled.z;
                return RE::NiPoint3(rotated.x + t.translate.x, rotated.y + t.translate.y, rotated.z + t.translate.z);
            };
            v0 = xform(v0);
            v1 = xform(v1);
            v2 = xform(v2);
        }
    };

    enum class GrabSurfaceSourceKind : std::uint8_t
    {
        Static,
        Dynamic,
        Skinned,
        CollisionQuery,
        AuthoredNode,
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

    struct GrabSurfaceHit
    {
        RE::NiPoint3 position{};
        RE::NiPoint3 normal{};
        int triangleIndex = -1;
        float distance = 1e30f;
        RE::NiAVObject* sourceNode = nullptr;
        RE::BSTriShape* sourceShape = nullptr;
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
        case GrabSurfaceSourceKind::AuthoredNode:
            return "authoredNode";
        case GrabSurfaceSourceKind::Fallback:
        default:
            return "fallback";
        }
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

        GrabSurfaceTriangleData surfaceTriangle{};
        surfaceTriangle.triangle = triangle;
        surfaceTriangle.sourceNode = sourceShape;
        surfaceTriangle.sourceShape = sourceShape;
        surfaceTriangle.triangleIndex = triangleIndex;
        surfaceTriangle.sourceKind = sourceKind;
        if (skinInfluences) {
            surfaceTriangle.skinInfluences = *skinInfluences;
            surfaceTriangle.hasSkinInfluences = true;
        }
        outSurfaceTriangles->push_back(surfaceTriangle);
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
        std::uint32_t blacklistedShapes = 0;

        [[nodiscard]] std::uint32_t totalTriangles() const noexcept { return staticTriangles + dynamicTriangles + skinnedTriangles; }
    };

    inline std::string_view meshExtractionNodeName(const RE::NiAVObject* node)
    {
        if (!node || !node->name.c_str()) {
            return {};
        }
        return std::string_view(node->name.c_str());
    }

    inline bool shouldSkipMeshExtractionNode(RE::NiAVObject* node, std::string_view grabNodeNameBlacklist, MeshExtractionStats* stats)
    {
        if (!node || !node->IsTriShape()) {
            return false;
        }

        if (!grab_node_name_policy::shouldSkipNodeForMeshSurface(grabNodeNameBlacklist, meshExtractionNodeName(node))) {
            return false;
        }

        if (stats) {
            ++stats->blacklistedShapes;
        }
        return true;
    }

    inline bool readTriShapeRawGeometry(RE::BSTriShape* triShape, TriShapeRawGeometry& out)
    {
        if (!triShape)
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
        std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles = nullptr)
    {
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

            TriangleData tri;
            tri.v0 = readDynamicVertexPosition(verts + i0 * dynamicStride);
            tri.v1 = readDynamicVertexPosition(verts + i1 * dynamicStride);
            tri.v2 = readDynamicVertexPosition(verts + i2 * dynamicStride);
            tri.applyTransform(worldTransform);

            outTriangles.push_back(tri);
            appendSurfaceTriangle(outSurfaceTriangles, tri, triShape, i, GrabSurfaceSourceKind::Dynamic);
            added++;
        }

        return added;
    }

    inline int extractTrianglesFromTriShape(
        RE::BSTriShape* triShape,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles = nullptr)
    {
        if (isDynamicTriShape(triShape)) {
            return extractTrianglesFromDynamicTriShape(triShape, outTriangles, outSurfaceTriangles);
        }

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

        int added = 0;
        for (std::uint32_t i = 0; i < geometry.numTriangles; i++) {
            std::uint16_t i0 = geometry.triangles[i * 3 + 0];
            std::uint16_t i1 = geometry.triangles[i * 3 + 1];
            std::uint16_t i2 = geometry.triangles[i * 3 + 2];

            if (i0 >= geometry.numVertices || i1 >= geometry.numVertices || i2 >= geometry.numVertices)
                continue;

            TriangleData tri;
            tri.v0 = readVertexPosition(verts + i0 * vtxStride, posOffset, fullPrecision);
            tri.v1 = readVertexPosition(verts + i1 * vtxStride, posOffset, fullPrecision);
            tri.v2 = readVertexPosition(verts + i2 * vtxStride, posOffset, fullPrecision);

            tri.applyTransform(worldTransform);
            outTriangles.push_back(tri);
            appendSurfaceTriangle(outSurfaceTriangles, tri, triShape, i, GrabSurfaceSourceKind::Static);
            added++;
        }
        return added;
    }

    inline bool isSkinned(RE::BSTriShape* triShape)
    {
        void* skinInst = nullptr;
        if (!native_memory::tryReadField(triShape, VROffset::skinInstance, skinInst)) {
            return false;
        }
        return skinInst != nullptr;
    }

    inline int extractTrianglesFromSkinnedTriShape(
        RE::BSTriShape* triShape,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>* outSurfaceTriangles = nullptr)
    {
        [[maybe_unused]] const char* shapeName = triShape->name.c_str() ? triShape->name.c_str() : "(null)";

        if (isDynamicTriShape(triShape)) {
            ROCK_LOG_DEBUG(MeshGrab, "Skipping skinned BSDynamicTriShape '{}' (dynamic+skinned extraction not yet verified)", shapeName);
            return 0;
        }

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
        std::uint32_t minStride = (std::max)(minPosSize, minSkinEnd);
        if (numTris == 0 || numTris > kMaxMeshExtractionTriangles || numVerts == 0 || vtxStride < minStride) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': bad geometry (tris={}, verts={}, stride={}, minNeeded={})", shapeName, numTris, numVerts, vtxStride, minStride);
            return 0;
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

        char* skinInst = nullptr;
        if (!native_memory::tryReadField(triShape, VROffset::skinInstance, skinInst)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable skinInstance field", shapeName);
            return 0;
        }
        if (!skinInst) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null skinInstance (should not happen)", shapeName);
            return 0;
        }

        std::uint32_t boneCount = 0;
        if (!native_memory::tryReadField(skinInst, 0x38, boneCount)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable boneCount", shapeName);
            return 0;
        }
        if (boneCount == 0 || boneCount > 512) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': suspicious boneCount={}", shapeName, boneCount);
            return 0;
        }

        RE::NiNode** boneNodes = nullptr;
        if (!native_memory::tryReadField(skinInst, 0x18, boneNodes)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable bone node array pointer", shapeName);
            return 0;
        }
        if (!boneNodes) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null bone node array", shapeName);
            return 0;
        }

        char* boneData = nullptr;
        if (!native_memory::tryReadField(skinInst, 0x40, boneData)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable BSSkin::BoneData pointer", shapeName);
            return 0;
        }
        if (!boneData) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null BSSkin::BoneData", shapeName);
            return 0;
        }
        char* skinToBoneArray = nullptr;
        if (!native_memory::tryReadField(boneData, 0x10, skinToBoneArray)) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': unreadable skinToBone transform array pointer", shapeName);
            return 0;
        }
        if (!skinToBoneArray) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null skinToBone transform array", shapeName);
            return 0;
        }

        struct BoneCombined
        {
            float m[12];
            RE::NiNode* node = nullptr;
            bool valid;
        };
        std::vector<BoneCombined> boneTransforms(boneCount);
        std::uint32_t invalidBoneNodePointers = 0;
        std::uint32_t invalidBoneTransforms = 0;

        for (std::uint32_t b = 0; b < boneCount; b++) {
            boneTransforms[b].valid = false;
            boneTransforms[b].node = nullptr;

            RE::NiNode* boneNode = nullptr;
            if (!native_memory::tryReadValue(boneNodes + b, boneNode) || !boneNode ||
                !native_memory::pointerRangeLooksReadable(reinterpret_cast<const char*>(boneNode) + 0x70, 0x40)) {
                ++invalidBoneNodePointers;
                continue;
            }

            std::array<float, 16> boneWorld{};
            std::array<float, 16> skinToBone{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const char*>(boneNode) + 0x70, boneWorld.data(), boneWorld.size() * sizeof(float)) ||
                !native_memory::guardedCopyFromMemory(skinToBoneArray + b * 0x50 + 0x10, skinToBone.data(), skinToBone.size() * sizeof(float)) ||
                !finiteFloatArray(boneWorld.data(), boneWorld.size()) ||
                !finiteFloatArray(skinToBone.data(), skinToBone.size())) {
                ++invalidBoneTransforms;
                continue;
            }

            const float* bwRot = boneWorld.data();
            const float* bwTrans = boneWorld.data() + 12;
            float bwScale = boneWorld[15];
            const float* stb = skinToBone.data();
            float stbScale = skinToBone[15];

            float combinedScale = bwScale * stbScale;

            float cr[9];
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    cr[r * 3 + c] = (bwRot[r * 4 + 0] * stb[0 * 4 + c] + bwRot[r * 4 + 1] * stb[1 * 4 + c] + bwRot[r * 4 + 2] * stb[2 * 4 + c]) * combinedScale;
                }
            }

            float stbTx = stb[12], stbTy = stb[13], stbTz = stb[14];
            float ct[3];
            for (int r = 0; r < 3; r++) {
                ct[r] = (bwRot[r * 4 + 0] * stbTx + bwRot[r * 4 + 1] * stbTy + bwRot[r * 4 + 2] * stbTz) * bwScale + bwTrans[r];
            }

            boneTransforms[b].m[0] = cr[0];
            boneTransforms[b].m[1] = cr[1];
            boneTransforms[b].m[2] = cr[2];
            boneTransforms[b].m[3] = ct[0];
            boneTransforms[b].m[4] = cr[3];
            boneTransforms[b].m[5] = cr[4];
            boneTransforms[b].m[6] = cr[5];
            boneTransforms[b].m[7] = ct[1];
            boneTransforms[b].m[8] = cr[6];
            boneTransforms[b].m[9] = cr[7];
            boneTransforms[b].m[10] = cr[8];
            boneTransforms[b].m[11] = ct[2];
            boneTransforms[b].node = boneNode;
            boneTransforms[b].valid = true;
        }

        ROCK_LOG_DEBUG(MeshGrab, "Skinned '{}': extracting {} tris, {} verts, {} bones (stride={}, skinOff={}, fullPrec={})", shapeName, numTris, numVerts, boneCount, vtxStride,
            skinOffset, fullPrecision ? 1 : 0);

        std::vector<RE::NiPoint3> worldVerts(numVerts);
        std::vector<std::uint8_t> worldVertexValid(numVerts, 0);
        std::vector<std::array<GrabSurfaceVertexInfluence, 4>> vertexInfluences(numVerts);
        std::uint32_t invalidSkinnedVertices = 0;
        for (std::uint16_t vi = 0; vi < numVerts; vi++) {
            const std::uint8_t* vtx = verts + vi * vtxStride;

            RE::NiPoint3 bindPos = readVertexPosition(vtx, posOffset, fullPrecision);

            const std::uint16_t* weightPtr = reinterpret_cast<const std::uint16_t*>(vtx + skinOffset);
            float w0 = halfToFloat(weightPtr[0]);
            float w1 = halfToFloat(weightPtr[1]);
            float w2 = halfToFloat(weightPtr[2]);
            float w3 = 1.0f - w0 - w1 - w2;

            const std::uint8_t* idxPtr = vtx + skinOffset + 8;
            std::uint8_t bi0 = idxPtr[0];
            std::uint8_t bi1 = idxPtr[1];
            std::uint8_t bi2 = idxPtr[2];
            std::uint8_t bi3 = idxPtr[3];

            float weights[4] = { w0, w1, w2, w3 };
            std::uint8_t indices[4] = { bi0, bi1, bi2, bi3 };

            float wx = 0.0f, wy = 0.0f, wz = 0.0f;
            float validWeight = 0.0f;
            bool missingWeightedBone = false;
            for (int k = 0; k < 4; k++) {
                float w = weights[k];
                if (!std::isfinite(w) || w <= 0.00001f)
                    continue;

                std::uint8_t bIdx = indices[k];
                if (bIdx >= boneCount || !boneTransforms[bIdx].valid) {
                    missingWeightedBone = true;
                    continue;
                }

                const float* m = boneTransforms[bIdx].m;

                float rx = m[0] * bindPos.x + m[1] * bindPos.y + m[2] * bindPos.z + m[3];
                float ry = m[4] * bindPos.x + m[5] * bindPos.y + m[6] * bindPos.z + m[7];
                float rz = m[8] * bindPos.x + m[9] * bindPos.y + m[10] * bindPos.z + m[11];

                wx += w * rx;
                wy += w * ry;
                wz += w * rz;
                validWeight += w;
                vertexInfluences[vi][k] = GrabSurfaceVertexInfluence{ boneTransforms[bIdx].node, w };
            }

            if (missingWeightedBone || validWeight <= 0.00001f || !std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
                ++invalidSkinnedVertices;
                continue;
            }

            worldVerts[vi] = RE::NiPoint3(wx, wy, wz);
            worldVertexValid[vi] = 1;
        }

        int added = 0;
        std::uint32_t skippedInvalidVertexTriangles = 0;
        for (std::uint32_t i = 0; i < numTris; i++) {
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
            std::array<std::array<GrabSurfaceVertexInfluence, 4>, 3> skinInfluences{};
            skinInfluences[0] = vertexInfluences[i0];
            skinInfluences[1] = vertexInfluences[i1];
            skinInfluences[2] = vertexInfluences[i2];
            appendSurfaceTriangle(outSurfaceTriangles, tri, triShape, i, GrabSurfaceSourceKind::Skinned, &skinInfluences);
            added++;
        }

        if (invalidBoneNodePointers != 0 || invalidBoneTransforms != 0 || invalidSkinnedVertices != 0 || skippedInvalidVertexTriangles != 0) {
            ROCK_LOG_WARN(MeshGrab,
                "Skinned '{}': skipped invalid data bonePtrs={} boneTransforms={} vertices={} triangles={}",
                shapeName,
                invalidBoneNodePointers,
                invalidBoneTransforms,
                invalidSkinnedVertices,
                skippedInvalidVertexTriangles);
        }
        ROCK_LOG_DEBUG(MeshGrab, "Skinned '{}': extracted {} triangles", shapeName, added);
        return added;
    }

    inline void extractAllTriangles(RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        int maxDepth = 10,
        MeshExtractionStats* stats = nullptr,
        std::string_view grabNodeNameBlacklist = {})
    {
        if (!root || maxDepth <= 0)
            return;

        if (root->flags.flags & 1)
            return;

        if (shouldSkipMeshExtractionNode(root, grabNodeNameBlacklist, stats))
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
                if (dynamic && stats) {
                    stats->dynamicSkinnedSkipped++;
                } else if (stats) {
                    stats->skinnedShapes++;
                }
                extractTrianglesFromSkinnedTriShape(triShape, outTriangles);
                if (stats && !dynamic) {
                    stats->skinnedTriangles += static_cast<std::uint32_t>(outTriangles.size() - before);
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
                    extractAllTriangles(kid, outTriangles, maxDepth - 1, stats, grabNodeNameBlacklist);
            }
        }
    }

    inline void extractAllSurfaceTriangles(RE::NiAVObject* root,
        std::vector<TriangleData>& outTriangles,
        std::vector<GrabSurfaceTriangleData>& outSurfaceTriangles,
        int maxDepth = 10,
        MeshExtractionStats* stats = nullptr,
        std::string_view grabNodeNameBlacklist = {})
    {
        if (!root || maxDepth <= 0)
            return;

        if (root->flags.flags & 1)
            return;

        if (shouldSkipMeshExtractionNode(root, grabNodeNameBlacklist, stats))
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
                if (dynamic && stats) {
                    stats->dynamicSkinnedSkipped++;
                } else if (stats) {
                    stats->skinnedShapes++;
                }
                extractTrianglesFromSkinnedTriShape(triShape, outTriangles, &outSurfaceTriangles);
                if (stats && !dynamic) {
                    stats->skinnedTriangles += static_cast<std::uint32_t>(outTriangles.size() - before);
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
            return;
        }

        auto* niNode = root->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            for (auto i = decltype(kids.size()){ 0 }; i < kids.size(); i++) {
                auto* kid = kids[i].get();
                if (kid)
                    extractAllSurfaceTriangles(kid, outTriangles, outSurfaceTriangles, maxDepth - 1, stats, grabNodeNameBlacklist);
            }
        }
    }

    inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

    inline RE::NiPoint3 cross(const RE::NiPoint3& a, const RE::NiPoint3& b) { return RE::NiPoint3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

    inline RE::NiPoint3 normalize(const RE::NiPoint3& v)
    {
        float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
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

        std::unordered_map<RE::NiAVObject*, float> accumulated;
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
                accumulated[influence.bone] += influence.weight * distanceWeight;
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

    inline bool findClosestGrabPoint(const std::vector<TriangleData>& triangles,
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
        float bestDist = 1e30f;
        int bestIdx = -1;
        RE::NiPoint3 bestPos;
        float bestSignedAlong = 0.0f;
        float bestLateral = 0.0f;
        const float behindTolerance = std::max(0.0f, std::isfinite(behindPalmToleranceGameUnits) ? behindPalmToleranceGameUnits : 0.0f);

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
            outResult.triangle = tri;
            outResult.sourceKind = surfaceTriangle.sourceKind;
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
        GrabSurfaceHit& outResult)
    {
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

        for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
            const auto& surfaceTriangle = triangles[i];
            const auto& tri = surfaceTriangle.triangle;
            RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v0)));
            if (dot(triNormal, triNormal) <= 0.0f) {
                continue;
            }
            if (dot(triNormal, expected) < 0.0f) {
                triNormal = RE::NiPoint3{ -triNormal.x, -triNormal.y, -triNormal.z };
            }
            if (dot(triNormal, expected) < minNormalDot) {
                continue;
            }

            float distSq = 0.0f;
            const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(point, tri, distSq);
            if (distSq > maxDistSq || distSq >= bestDistSq) {
                continue;
            }

            bestDistSq = distSq;
            bestIdx = i;
            bestPoint = candidate;
            bestNormal = triNormal;
        }

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
        outResult.triangle = surfaceTriangle.triangle;
        outResult.sourceKind = surfaceTriangle.sourceKind;
        outResult.hasTriangle = true;
        outResult.valid = true;
        return true;
    }

    inline bool findClosestGrabSurfaceHitToPointPositionOnly(const std::vector<GrabSurfaceTriangleData>& triangles,
        const RE::NiPoint3& point,
        const RE::NiPoint3& preferredNormal,
        float maxDistanceGameUnits,
        GrabSurfaceHit& outResult)
    {
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

        for (int i = 0; i < static_cast<int>(triangles.size()); ++i) {
            const auto& surfaceTriangle = triangles[i];
            const auto& tri = surfaceTriangle.triangle;
            RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v0)));
            if (dot(triNormal, triNormal) <= 0.0f) {
                continue;
            }
            if (hasPreferredNormal && dot(triNormal, preferred) < 0.0f) {
                triNormal = RE::NiPoint3{ -triNormal.x, -triNormal.y, -triNormal.z };
            }

            float distSq = 0.0f;
            const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(point, tri, distSq);
            if (distSq > maxDistSq || distSq >= bestDistSq) {
                continue;
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
            bestIdx = i;
            bestPoint = candidate;
            bestNormal = triNormal;
            bestSignedAlong = signedAlong;
            bestLateral = lateral;
        }

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
        outResult.triangle = surfaceTriangle.triangle;
        outResult.sourceKind = surfaceTriangle.sourceKind;
        outResult.hasTriangle = true;
        outResult.signedAlongPalmDistanceGameUnits = bestSignedAlong;
        outResult.lateralPalmDistanceGameUnits = bestLateral;
        outResult.valid = true;
        return true;
    }
}
