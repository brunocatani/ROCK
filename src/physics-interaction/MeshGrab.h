#pragma once

#include "PhysicsLog.h"
#include "RE/Fallout.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace frik::rock
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

    struct TriangleData
    {
        RE::NiPoint3 v0, v1, v2;

        void applyTransform(const RE::NiTransform& t)
        {
            auto xform = [&](const RE::NiPoint3& p) -> RE::NiPoint3 {
                /*
                 * Mesh extraction must match FO4VR's native NiTransform compose
                 * convention, not CommonLib's row-style point helper. Ghidra
                 * verification of the engine compose path showed child-local
                 * vectors are applied through the parent column basis. Weapon
                 * hulls are baked from these world triangles once at creation;
                 * using the opposite basis makes the baked hull depend on how
                 * tilted the weapon was when the collision was created.
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

        auto* base = reinterpret_cast<char*>(triShape);
        out.rendererData = *reinterpret_cast<void**>(base + VROffset::rendererData);
        if (!out.rendererData)
            return false;

        out.numTriangles = *reinterpret_cast<std::uint32_t*>(base + VROffset::numTriangles);
        out.numVertices = *reinterpret_cast<std::uint16_t*>(base + VROffset::numVertices);
        out.vertexDesc = *reinterpret_cast<std::uint64_t*>(base + VROffset::vertexDesc);
        if (out.numTriangles == 0 || out.numVertices == 0)
            return false;

        auto* triangleDataPtr = *reinterpret_cast<void**>(reinterpret_cast<char*>(out.rendererData) + 0x10);
        if (!triangleDataPtr)
            return false;

        out.triangles = *reinterpret_cast<std::uint16_t**>(reinterpret_cast<char*>(triangleDataPtr) + 0x08);
        return out.triangles != nullptr;
    }

    inline std::uint8_t* readStaticVertexBlock(void* rendererData)
    {
        if (!rendererData)
            return nullptr;

        auto* vertexDataPtr = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + 0x08);
        if (!vertexDataPtr)
            return nullptr;

        return *reinterpret_cast<std::uint8_t**>(reinterpret_cast<char*>(vertexDataPtr) + 0x08);
    }

    inline bool isDynamicTriShape(RE::BSTriShape* triShape)
    {
        if (!triShape)
            return false;
        auto* base = reinterpret_cast<char*>(triShape);
        return *reinterpret_cast<std::uint8_t*>(base + VROffset::geometryType) == VROffset::kTypeBSDynamicTriShape;
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

    inline int extractTrianglesFromDynamicTriShape(RE::BSTriShape* triShape, std::vector<TriangleData>& outTriangles)
    {
        auto* base = reinterpret_cast<char*>(triShape);
        const char* shapeName = triShape->name.c_str() ? triShape->name.c_str() : "(null)";

        TriShapeRawGeometry geometry;
        if (!readTriShapeRawGeometry(triShape, geometry))
            return 0;

        std::uint32_t dynamicStride = static_cast<std::uint32_t>((geometry.vertexDesc >> 2) & 0x3C);
        if (dynamicStride < 6) {
            ROCK_LOG_WARN(MeshGrab, "Dynamic '{}': bad dynamic stride {}", shapeName, dynamicStride);
            return 0;
        }

        std::uint32_t dynamicDataSize = *reinterpret_cast<std::uint32_t*>(base + VROffset::dynamicDataSize);
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
            added++;
        }

        return added;
    }

    inline int extractTrianglesFromTriShape(RE::BSTriShape* triShape, std::vector<TriangleData>& outTriangles)
    {
        if (isDynamicTriShape(triShape)) {
            return extractTrianglesFromDynamicTriShape(triShape, outTriangles);
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
            added++;
        }
        return added;
    }

    inline bool isSkinned(RE::BSTriShape* triShape)
    {
        auto* base = reinterpret_cast<char*>(triShape);
        auto* skinInst = *reinterpret_cast<void**>(base + VROffset::skinInstance);
        return skinInst != nullptr;
    }

    inline int extractTrianglesFromSkinnedTriShape(RE::BSTriShape* triShape, std::vector<TriangleData>& outTriangles)
    {
        auto* base = reinterpret_cast<char*>(triShape);
        const char* shapeName = triShape->name.c_str() ? triShape->name.c_str() : "(null)";

        if (isDynamicTriShape(triShape)) {
            ROCK_LOG_DEBUG(MeshGrab, "Skipping skinned BSDynamicTriShape '{}' (dynamic+skinned extraction not yet verified)", shapeName);
            return 0;
        }

        auto* rendererData = *reinterpret_cast<void**>(base + VROffset::rendererData);
        if (!rendererData) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null rendererData", shapeName);
            return 0;
        }

        std::uint32_t numTris = *reinterpret_cast<std::uint32_t*>(base + VROffset::numTriangles);
        std::uint16_t numVerts = *reinterpret_cast<std::uint16_t*>(base + VROffset::numVertices);
        std::uint64_t vtxDescRaw = *reinterpret_cast<std::uint64_t*>(base + VROffset::vertexDesc);
        std::uint32_t vtxStride = static_cast<std::uint32_t>(vtxDescRaw & 0xF) * 4;

        std::uint32_t posOffset = static_cast<std::uint32_t>((vtxDescRaw >> 2) & 0x3C);
        bool fullPrecision = ((vtxDescRaw >> 54) & 1) != 0;

        std::uint32_t skinOffset = static_cast<std::uint32_t>((vtxDescRaw >> 26) & 0x3C);

        std::uint32_t minPosSize = fullPrecision ? (posOffset + 12) : (posOffset + 6);
        std::uint32_t minSkinEnd = skinOffset + 12;
        std::uint32_t minStride = (std::max)(minPosSize, minSkinEnd);
        if (numTris == 0 || numVerts == 0 || vtxStride < minStride) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': bad geometry (tris={}, verts={}, stride={}, minNeeded={})", shapeName, numTris, numVerts, vtxStride, minStride);
            return 0;
        }

        auto* vertexDataPtr = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + 0x08);
        auto* triangleDataPtr = *reinterpret_cast<void**>(reinterpret_cast<char*>(rendererData) + 0x10);
        if (!vertexDataPtr || !triangleDataPtr) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null vertex/triangle data", shapeName);
            return 0;
        }

        auto* verts = *reinterpret_cast<std::uint8_t**>(reinterpret_cast<char*>(vertexDataPtr) + 0x08);
        auto* tris = *reinterpret_cast<std::uint16_t**>(reinterpret_cast<char*>(triangleDataPtr) + 0x08);
        if (!verts || !tris) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null CPU buffers", shapeName);
            return 0;
        }

        auto* skinInst = *reinterpret_cast<char**>(base + VROffset::skinInstance);
        if (!skinInst) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null skinInstance (should not happen)", shapeName);
            return 0;
        }

        std::uint32_t boneCount = *reinterpret_cast<std::uint32_t*>(skinInst + 0x38);
        if (boneCount == 0 || boneCount > 512) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': suspicious boneCount={}", shapeName, boneCount);
            return 0;
        }

        auto** boneNodes = *reinterpret_cast<RE::NiNode***>(skinInst + 0x18);
        if (!boneNodes) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null bone node array", shapeName);
            return 0;
        }

        auto* boneData = *reinterpret_cast<char**>(skinInst + 0x40);
        if (!boneData) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null BSSkin::BoneData", shapeName);
            return 0;
        }
        auto* skinToBoneArray = *reinterpret_cast<char**>(boneData + 0x10);
        if (!skinToBoneArray) {
            ROCK_LOG_WARN(MeshGrab, "Skinned '{}': null skinToBone transform array", shapeName);
            return 0;
        }

        struct BoneCombined
        {
            float m[12];
            bool valid;
        };
        std::vector<BoneCombined> boneTransforms(boneCount);

        for (std::uint32_t b = 0; b < boneCount; b++) {
            boneTransforms[b].valid = false;

            auto* boneNode = boneNodes[b];
            if (!boneNode)
                continue;

            auto* boneWorldPtr = reinterpret_cast<char*>(boneNode) + 0x70;
            const float* bwRot = reinterpret_cast<const float*>(boneWorldPtr);
            const float* bwTrans = reinterpret_cast<const float*>(boneWorldPtr + 0x30);
            float bwScale = *reinterpret_cast<const float*>(boneWorldPtr + 0x3C);

            const float* stb = reinterpret_cast<const float*>(skinToBoneArray + b * 0x50 + 0x10);
            float stbScale = *reinterpret_cast<const float*>(skinToBoneArray + b * 0x50 + 0x4C);

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
            boneTransforms[b].valid = true;
        }

        ROCK_LOG_DEBUG(MeshGrab, "Skinned '{}': extracting {} tris, {} verts, {} bones (stride={}, skinOff={}, fullPrec={})", shapeName, numTris, numVerts, boneCount, vtxStride,
            skinOffset, fullPrecision ? 1 : 0);

        std::vector<RE::NiPoint3> worldVerts(numVerts);
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
            for (int k = 0; k < 4; k++) {
                float w = weights[k];
                if (w <= 0.0f)
                    continue;

                std::uint8_t bIdx = indices[k];
                if (bIdx >= boneCount || !boneTransforms[bIdx].valid)
                    continue;

                const float* m = boneTransforms[bIdx].m;

                float rx = m[0] * bindPos.x + m[1] * bindPos.y + m[2] * bindPos.z + m[3];
                float ry = m[4] * bindPos.x + m[5] * bindPos.y + m[6] * bindPos.z + m[7];
                float rz = m[8] * bindPos.x + m[9] * bindPos.y + m[10] * bindPos.z + m[11];

                wx += w * rx;
                wy += w * ry;
                wz += w * rz;
            }

            worldVerts[vi] = RE::NiPoint3(wx, wy, wz);
        }

        int added = 0;
        for (std::uint32_t i = 0; i < numTris; i++) {
            std::uint16_t i0 = tris[i * 3 + 0];
            std::uint16_t i1 = tris[i * 3 + 1];
            std::uint16_t i2 = tris[i * 3 + 2];

            if (i0 >= numVerts || i1 >= numVerts || i2 >= numVerts)
                continue;

            TriangleData tri;
            tri.v0 = worldVerts[i0];
            tri.v1 = worldVerts[i1];
            tri.v2 = worldVerts[i2];

            outTriangles.push_back(tri);
            added++;
        }

        ROCK_LOG_DEBUG(MeshGrab, "Skinned '{}': extracted {} triangles", shapeName, added);
        return added;
    }

    inline void extractAllTriangles(RE::NiAVObject* root, std::vector<TriangleData>& outTriangles, int maxDepth = 10, MeshExtractionStats* stats = nullptr)
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
                    extractAllTriangles(kid, outTriangles, maxDepth - 1, stats);
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
    };

    inline bool findClosestGrabPoint(const std::vector<TriangleData>& triangles, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmDir, float lateralWeight,
        float directionalWeight, GrabPoint& outResult)
    {
        float bestDist = 1e30f;
        int bestIdx = -1;
        RE::NiPoint3 bestPos;

        for (int i = 0; i < static_cast<int>(triangles.size()); i++) {
            const auto& tri = triangles[i];

            RE::NiPoint3 triNormal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));

            if (dot(triNormal, palmDir) > 0.0f)
                continue;

            float distSq;
            RE::NiPoint3 closest = closestPointOnTriangleToLine(palmPos, palmDir, tri, distSq);

            RE::NiPoint3 toClosest = sub(closest, palmPos);
            float dirComponent = dot(toClosest, palmDir);
            RE::NiPoint3 dirVec(palmDir.x * dirComponent, palmDir.y * dirComponent, palmDir.z * dirComponent);
            RE::NiPoint3 latVec = sub(toClosest, dirVec);

            float dirDist = dirComponent * dirComponent;
            float latDist = dot(latVec, latVec);
            float weightedDist = directionalWeight * dirDist + lateralWeight * latDist;

            if (weightedDist < bestDist) {
                bestDist = weightedDist;
                bestIdx = i;
                bestPos = closest;
            }
        }

        if (bestIdx >= 0) {
            const auto& tri = triangles[bestIdx];
            outResult.position = bestPos;
            outResult.normal = normalize(cross(sub(tri.v1, tri.v0), sub(tri.v2, tri.v1)));
            outResult.triangleIndex = bestIdx;
            outResult.distance = bestDist;
            return true;
        }
        return false;
    }
}
