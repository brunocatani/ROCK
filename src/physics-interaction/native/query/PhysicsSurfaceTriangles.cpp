#include "physics-interaction/native/query/PhysicsSurfaceTriangles.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/ConvexHullMesh.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/havok/HavokWorldLock.h"
#include "physics-interaction/native/havok/PhysicsUtils.h"
#include "physics-interaction/native/query/PhysicsQueryResources.h"
#include "physics-interaction/native/query/SelectionFilterInfo.h"

#include "RE/Havok/hknpAabbQuery.h"
#include "RE/Havok/hknpCollisionResult.h"
#include "RE/Havok/hknpShape.h"

#include <Windows.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <vector>
#include <xmmintrin.h>

namespace rock::physics_surface_triangles
{
    namespace
    {
        // Leaf decoders (getLeafShapes, vtable +0xA8) by the shape vtable that
        // owns them; only these shapes are decoded through the slot.
        constexpr std::size_t kSlotGetLeafShapes = 0xA8;
        struct Decoder
        {
            std::uintptr_t vtable;
            std::uintptr_t function;
            const char* name;
        };
        constexpr std::array<Decoder, 3> kDecoders{ {
            { 0x2EDF418, 0x1602860, "compressed mesh" },
            { 0x2EDF518, 0x16E2080, "static compound" },
            { 0x2DFE118, 0x156B200, "compressed heightfield" },
        } };
        // hknpWorld::queryAabb prologue: MOV RAX,RSP; MOV [RAX+18],R8; PUSH RBP; PUSH R14; PUSH R15.
        constexpr std::array<std::uint8_t, 12> kQueryAabbPrologue{ 0x48, 0x8B, 0xC4, 0x4C, 0x89, 0x40, 0x18, 0x55, 0x41,
            0x56, 0x41, 0x57 };

        // The decoders' output collector (audit wave 2). ROCK owns it; the
        // largest write seen ends near +0x1480.
        constexpr std::size_t kCollectorSize = 0x1800;
        constexpr std::size_t kCollectorTransform = 0x00;  // columns +0x00/+0x10/+0x20, translation +0x30
        constexpr std::size_t kCollectorScale = 0x40;
        constexpr std::size_t kCollectorFlags = 0x50;
        constexpr std::size_t kCollectorShapeOut = 0x58;
        constexpr std::size_t kCollectorKeyPathValue = 0x78;
        constexpr std::size_t kCollectorKeyPathBits = 0x7C;
        constexpr std::size_t kCollectorKeyPathCount = 0x180;
        constexpr std::size_t kCollectorEntries = 0x190;  // entry 0: v0..v3, hkVector4 each
        constexpr std::size_t kCollectorEntryFlags = 0x424;
        constexpr std::uint32_t kFlagTriangleBatch = 0x10;
        constexpr std::uint16_t kLeafQuad = 0x1000;
        constexpr std::uint32_t kNoShapeKey = 0xFFFFFFFFu;
        // Decoded leaves per query: past this the result is truncated.
        constexpr int kMaxDecodedHits = 2048;
        constexpr int kMaxHullVertices = 256;

        enum class Binary
        {
            Unchecked,
            Valid,
            Invalid
        };
        Binary s_binary = Binary::Unchecked;

        bool binaryValid()
        {
            if (s_binary != Binary::Unchecked) {
                return s_binary == Binary::Valid;
            }
            s_binary = Binary::Invalid;
            const auto base = REL::Module::get().base();
            if (std::memcmp(reinterpret_cast<const void*>(base + offsets::kFunc_World_QueryAabb), kQueryAabbPrologue.data(),
                    kQueryAabbPrologue.size()) != 0) {
                ROCK_LOG_ERROR(Hand, "Surface triangles disabled: hknpWorld::queryAabb at +{:X} is not the verified code",
                    offsets::kFunc_World_QueryAabb);
                return false;
            }
            for (const auto& decoder : kDecoders) {
                const auto slot = *reinterpret_cast<const std::uintptr_t*>(base + decoder.vtable + kSlotGetLeafShapes);
                if (slot != base + decoder.function) {
                    ROCK_LOG_ERROR(Hand, "Surface triangles disabled: the {} leaf decoder slot holds +{:X}, not +{:X}",
                        decoder.name, slot - base, decoder.function);
                    return false;
                }
            }
            s_binary = Binary::Valid;
            return true;
        }

        const Decoder* decoderOf(const void* shape)
        {
            const auto vtable = *reinterpret_cast<const std::uintptr_t*>(shape);
            const auto base = REL::Module::get().base();
            for (const auto& decoder : kDecoders) {
                if (vtable == base + decoder.vtable) {
                    return &decoder;
                }
            }
            return nullptr;
        }

        alignas(16) std::array<std::byte, kCollectorSize> s_collector{};  // frame thread only

        template <class T>
        T& field(std::size_t offset)
        {
            return *reinterpret_cast<T*>(s_collector.data() + offset);
        }

        // Before every decode: identity transform, unit scale, no flags, no
        // key path (the compound accumulates path state in the collector).
        void resetCollector()
        {
            auto* m = &field<float>(kCollectorTransform);
            std::memset(m, 0, 0x40);
            m[0] = m[5] = m[10] = 1.0f;
            auto* scale = &field<float>(kCollectorScale);
            scale[0] = scale[1] = scale[2] = scale[3] = 1.0f;
            field<std::uint32_t>(kCollectorFlags) = 0;
            field<const void*>(kCollectorShapeOut) = nullptr;
            field<std::uint32_t>(kCollectorKeyPathValue) = 0;
            field<std::uint32_t>(kCollectorKeyPathBits) = 0;
            field<std::uint32_t>(kCollectorKeyPathCount) = 0;
        }

        struct Point
        {
            float x, y, z;
        };

        // Where decoded triangles go: body frame (Havok rows as stored) and
        // the caller's array.
        struct Sink
        {
            Triangle* out = nullptr;
            std::uint32_t capacity = 0;
            std::uint32_t count = 0;
            bool truncated = false;
            std::uint32_t layer = 0;
            const float* body = nullptr;  // hknpBody: axes +0x00/+0x10/+0x20, position +0x30
        };

        // Leaf space to body space through the collector's transform.
        Point throughCollector(const float* v)
        {
            const auto* m = &field<float>(kCollectorTransform);
            return { m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12], m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13],
                m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14] };
        }

        RE::NiPoint3 toWorldGame(const float* body, const Point& p, float havokToGame)
        {
            return { (body[0] * p.x + body[4] * p.y + body[8] * p.z + body[12]) * havokToGame,
                (body[1] * p.x + body[5] * p.y + body[9] * p.z + body[13]) * havokToGame,
                (body[2] * p.x + body[6] * p.y + body[10] * p.z + body[14]) * havokToGame };
        }

        void emit(Sink& sink, const Point& p0, const Point& p1, const Point& p2)
        {
            if (sink.count >= sink.capacity) {
                sink.truncated = true;
                return;
            }
            const float scale = physics_scale::havokToGame();
            Triangle t{};
            t.a = toWorldGame(sink.body, p0, scale);
            t.b = toWorldGame(sink.body, p1, scale);
            t.c = toWorldGame(sink.body, p2, scale);
            const RE::NiPoint3 e1 = t.b - t.a, e2 = t.c - t.a;
            const RE::NiPoint3 n{ e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x };
            const float length = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
            if (!std::isfinite(length) || length < 1.0e-6f) {
                return;
            }
            t.normal = { n.x / length, n.y / length, n.z / length };
            t.layer = sink.layer;
            sink.out[sink.count++] = t;
        }

        // A convex shape's hull from its support vertices, through the collector transform.
        void emitHull(Sink& sink, const RE::hknpShape* shape, bool throughCollectorTransform)
        {
            const int vertexCount = shape->GetNumberOfSupportVertices();
            if (vertexCount <= 0 || vertexCount > kMaxHullVertices) {
                return;
            }
            std::array<RE::hkVector4f, kMaxHullVertices> buffer{};
            const auto* raw = shape->GetSupportVertices(reinterpret_cast<RE::hkcdVertex*>(buffer.data()), vertexCount);
            if (!raw) {
                return;
            }
            const auto* vertices = reinterpret_cast<const float*>(raw);
            std::vector<convex_hull_mesh::Vec3> points;
            points.reserve(static_cast<std::size_t>(vertexCount));
            for (int i = 0; i < vertexCount; ++i) {
                const auto* v = vertices + i * 4;
                const Point p = throughCollectorTransform ? throughCollector(v) : Point{ v[0], v[1], v[2] };
                points.push_back({ p.x, p.y, p.z });
            }
            for (const auto& tri : convex_hull_mesh::triangulateConvexHullFaces(points)) {
                const auto& a = points[tri[0]];
                const auto& b = points[tri[1]];
                const auto& c = points[tri[2]];
                emit(sink, { a.x, a.y, a.z }, { b.x, b.y, b.z }, { c.x, c.y, c.z });
            }
        }

        // One leaf of a decodable shape: entry 0 of a triangle batch (two
        // triangles for a quad), or its convex leaf.
        void emitLeaf(Sink& sink, const void* shape, std::uint32_t key)
        {
            resetCollector();
            using GetLeafShapes = int (*)(const void*, const std::uint32_t*, int, void*);
            const auto decode =
                *reinterpret_cast<const GetLeafShapes*>(*reinterpret_cast<const std::uintptr_t*>(shape) + kSlotGetLeafShapes);
            if (decode(shape, &key, 1, s_collector.data()) <= 0) {
                return;
            }
            if (field<std::uint32_t>(kCollectorFlags) & kFlagTriangleBatch) {
                const auto* v = &field<float>(kCollectorEntries);
                const Point p0 = throughCollector(v), p1 = throughCollector(v + 4), p2 = throughCollector(v + 8);
                emit(sink, p0, p1, p2);
                if (field<std::uint16_t>(kCollectorEntryFlags) & kLeafQuad) {
                    emit(sink, p0, p2, throughCollector(v + 12));
                }
                return;
            }
            if (const auto* leaf = field<const RE::hknpShape*>(kCollectorShapeOut)) {
                emitHull(sink, leaf, true);
            }
        }

        struct Hits
        {
            const RE::hknpCollisionResult* data;
            int size;
        };

        // Decodes every hit; SEH-guarded so a bad shape refuses instead of crashing.
        void decodeHits(RE::hknpWorld* world, Hits hits, Sink& sink)
        {
            std::uint32_t lastBody = 0xFFFFFFFFu;
            bool lastVisible = false;
            const int count = hits.size < kMaxDecodedHits ? hits.size : kMaxDecodedHits;
            if (hits.size > kMaxDecodedHits) {
                sink.truncated = true;
            }
            for (int i = 0; i < count && !(sink.truncated && sink.count >= sink.capacity); ++i) {
                const auto& hit = hits.data[i];
                const std::uint32_t bodyId = hit.hitBodyInfo.m_bodyId.value;
                if (bodyId != lastBody) {
                    lastBody = bodyId;
                    std::uint32_t filterInfo = 0;
                    lastVisible = havok_runtime::tryReadFilterInfo(world, hit.hitBodyInfo.m_bodyId, filterInfo) &&
                        collision_layer_policy::isVisibleSurfaceLayer(filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK);
                    const auto* body = lastVisible ? havok_runtime::getBody(world, hit.hitBodyInfo.m_bodyId) : nullptr;
                    lastVisible = body && *reinterpret_cast<const void* const*>(reinterpret_cast<const std::byte*>(body) + 0x48);
                    sink.body = reinterpret_cast<const float*>(body);
                    sink.layer = filterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK;
                }
                if (!lastVisible) {
                    continue;
                }
                const auto* shape =
                    *reinterpret_cast<const RE::hknpShape* const*>(reinterpret_cast<const std::byte*>(sink.body) + 0x48);
                const std::uint32_t key = hit.hitBodyInfo.m_shapeKey.storage;
                if (key != kNoShapeKey && decoderOf(shape)) {
                    emitLeaf(sink, shape, key);
                } else if (key == kNoShapeKey) {
                    emitHull(sink, shape, false);
                }
            }
        }

        bool guardedDecode(RE::hknpWorld* world, Hits hits, Sink* sink)
        {
            __try {
                decodeHits(world, hits, *sink);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }
    }

    bool collect(RE::hknpWorld* world, const RE::NiPoint3& centreGame, float radiusGame, Triangle* out,
        std::uint32_t capacity, Result& result)
    {
        result = {};
        if (!world || !out || !capacity || !physics_query_resources::nativeCleanupReady() || !binaryValid()) {
            return false;
        }
        auto* filterRef = havok_runtime::getQueryFilterRef(world);
        if (!filterRef) {
            return false;
        }
        const float scale = physics_scale::gameToHavok();
        RE::hknpAabbQuery query{};
        query.filterRef = filterRef;
        query.materialId = 0xFFFF;
        query.collisionFilterInfo = selection_query_policy::kFarClipRayFilterInfo;
        query.aabbMin[0] = (centreGame.x - radiusGame) * scale;
        query.aabbMin[1] = (centreGame.y - radiusGame) * scale;
        query.aabbMin[2] = (centreGame.z - radiusGame) * scale;
        query.aabbMax[0] = (centreGame.x + radiusGame) * scale;
        query.aabbMax[1] = (centreGame.y + radiusGame) * scale;
        query.aabbMax[2] = (centreGame.z + radiusGame) * scale;

        physics_query_resources::AllHitsCollector ownedHits;
        auto& hits = ownedHits.get();
        // Reset in place, as castSelectionSphere does: the inline hit storage
        // must keep pointing at this collector.
        hits.hints = 0;
        hits.earlyOutThreshold.real = _mm_set1_ps((std::numeric_limits<float>::max)());
        hits.hits._size = 0;
        Sink sink{ out, capacity };
        bool decoded = false;
        {
            using QueryAabb = void (*)(RE::hknpWorld*, RE::hknpAabbQuery*, RE::hknpCollisionQueryCollector*);
            static REL::Relocation<QueryAabb> queryAabb{ REL::Offset(offsets::kFunc_World_QueryAabb) };
            havok_world_lock::ScopedWorldReadLock worldReadLock(world);
            queryAabb(world, &query, &hits);
            decoded = guardedDecode(world, { hits.hits._data, hits.hits._size }, &sink);
        }
        if (!decoded) {
            s_binary = Binary::Invalid;
            ROCK_LOG_ERROR(Hand, "Surface triangles disabled: decoding a shape near ({:.0f},{:.0f},{:.0f}) faulted",
                centreGame.x, centreGame.y, centreGame.z);
            return false;
        }
        result.count = sink.count;
        result.truncated = sink.truncated;
        return true;
    }
}
