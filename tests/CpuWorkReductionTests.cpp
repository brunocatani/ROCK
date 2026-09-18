#include "physics-interaction/contact/GeneratedBodyContactRegistry.h"
#include "physics-interaction/consume/ImmersiveAidContact.h"
#include "physics-interaction/grab/MotionBodySearchBatch.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
#include "physics-interaction/native/PhysicsSystemBodyScanCache.h"
#include "physics-interaction/weapon/GeneratedWeaponGeometry.h"
#include "physics-interaction/weapon/WeaponEvidenceSnapshot.h"

#include <cassert>
#include <cstdio>
#include <map>
#include <random>

namespace
{
    void registryEquivalence()
    {
        using namespace rock::generated_body_contact_registry;
        Registry<32> registry;
        std::mt19937 random(713);
        std::array<Entry, 32> entries{};
        for (unsigned frame = 0; frame < 1000; ++frame) {
            const auto count = frame % 33;
            for (std::size_t i = 0; i < count; ++i) {
                if (frame % 4 == 0) entries[i].bodyId = random() % 25;
                entries[i].kind = i % 7 == 0 ? GeneratedBodyKind::Unknown : GeneratedBodyKind::Weapon;
                entries[i].generationKey = frame;
                entries[i].sampledVelocityHavokX = static_cast<float>(frame + i);
            }
            registry.publish(entries.data(), count);
            std::map<std::uint32_t, std::vector<const Entry*>> expected;
            for (std::size_t i = 0; i < count; ++i)
                if (isValidBodyId(entries[i].bodyId) && entries[i].kind != GeneratedBodyKind::Unknown)
                    expected[entries[i].bodyId].push_back(&entries[i]);
            std::uint32_t unique = 0;
            for (std::uint32_t id = 0; id < 26; ++id) {
                Classification found{};
                const auto it = expected.find(id);
                const bool valid = it != expected.end() && it->second.size() == 1;
                assert(registry.tryClassify(id, found) == valid);
                if (valid) {
                    ++unique;
                    assert(found.generationKey == frame);
                    assert(found.sampledVelocityHavokX == it->second.front()->sampledVelocityHavokX);
                }
            }
            assert(registry.count() == unique);
            if (frame % 23 == 0) { registry.clear(); assert(registry.count() == 0); }
        }
    }

    void exactMeshContacts()
    {
        std::mt19937 random(991);
        std::uniform_real_distribution<float> position(-40.0f, 40.0f);
        std::uniform_real_distribution<float> radius(0.0001f, 3.0f);
        std::vector<rock::GrabLocalTriangle> mesh;
        for (unsigned i = 0; i < 800; ++i) {
            const RE::NiPoint3 point{ position(random), position(random), position(random) };
            mesh.push_back({ point, point + RE::NiPoint3{ 0.02f, 0, 0 }, point + RE::NiPoint3{ 0, 0.8f, 0 } });
        }
        mesh.push_back({ {}, {}, {} }); // Degenerate triangles keep the exact predicate.
        rock::HeldContactMeshCache cache;
        cache.prepare(mesh);
        assert(cache.valid && cache.indexed);
        std::uint64_t indexedWork = 0, linearWork = 0;
        for (unsigned query = 0; query < 5000; ++query) {
            const RE::NiPoint3 a{ position(random), position(random), position(random) };
            const auto b = a + RE::NiPoint3{ 0.4f, -0.2f, 1.1f };
            const float r = radius(random);
            rock::WeaponTriangleIndex::QueryStats linear{}, indexed{};
            const bool expected = rock::immersive_aid::capsuleTouchesMesh(mesh, cache.minimum, cache.maximum, a, b, r, nullptr, &linear);
            const bool actual = rock::immersive_aid::capsuleTouchesMesh(mesh, cache.minimum, cache.maximum, a, b, r, &cache.index, &indexed);
            assert(actual == expected);
            indexedWork += indexed.triangles;
            linearWork += linear.triangles;
        }
        assert(indexedWork * 4 < linearWork);
        // Explicit cap/tip/tangent cases on every original needle vertex.
        for (const auto& triangle : mesh) {
            const auto a = triangle.v0 + RE::NiPoint3{ 0, 0, 0.01f };
            for (float r : { 0.009999f, 0.01f, 0.010001f }) {
                const bool expected = rock::immersive_aid::capsuleTouchesMesh(mesh, cache.minimum, cache.maximum, a, a, r);
                assert(rock::immersive_aid::capsuleTouchesMesh(mesh, cache.minimum, cache.maximum, a, a, r, &cache.index) == expected);
            }
        }
        std::printf("Exact injection workload: %llu -> %llu triangle predicates, identical contacts\n",
            static_cast<unsigned long long>(linearWork), static_cast<unsigned long long>(indexedWork));
        cache.clear();
        mesh.front().v0.x = std::numeric_limits<float>::quiet_NaN();
        cache.prepare(mesh);
        assert(cache.prepared && !cache.valid);
        cache.clear();
        mesh = { { {}, { 1, 0, 0 }, { 0, 1, 0 } } };
        cache.prepare(mesh);
        assert(cache.valid && !cache.indexed && cache.maximum.x == 1);
        cache.clear();
        mesh.clear();
        cache.prepare(mesh);
        assert(!cache.valid);
    }

    void dampingAndMotionEnumeration()
    {
        using namespace rock::nearby_grab_damping;
        MotionBodySearchBatch batch;
        batch.add(8); batch.add(9); batch.add(10); batch.add(8);
        unsigned scans = 0, slots = 0;
        std::map<std::uint32_t, std::uint32_t> live{ { 10, 8 }, { 9000, 9 } };
        const auto enumerate = [&](auto&& observe) {
            ++scans;
            for (std::uint32_t id = 0; id <= 9000; ++id) {
                ++slots;
                const auto it = live.find(id);
                if (it != live.end() && observe(it->second, id)) break;
            }
            return true;
        };
        const auto valid = [&](std::uint32_t body, std::uint32_t motion) {
            const auto it = live.find(body);
            return it != live.end() && it->second == motion;
        };
        assert(batch.find(8, enumerate, valid).bodyId == 10);
        assert(scans == 1 && slots == 11); // B's valid preferred body must not extend A's scan.
        assert(batch.find(8, enumerate, valid).bodyId == 10 && scans == 1);
        assert(batch.find(9, enumerate, valid).bodyId == 9000 && scans == 2);
        // Recycling a positive hint requires a fresh search and skips invalid candidates.
        live[10] = 7; live[15] = 8; live[20] = 8;
        assert(batch.find(8, enumerate, [&](auto body, auto motion) {
            return body != 15 && valid(body, motion);
        }).bodyId == 20);
        assert(batch.find(10, enumerate, valid).freshAbsence);
        const auto oldScans = scans;
        live[30] = 10;
        assert(batch.find(10, enumerate, valid).bodyId == 30 && scans == oldScans + 1);
        MotionBodySearchBatch hints;
        hints.add(8); hints.add(9);
        assert(hints.find(9, enumerate, valid).bodyId == 9000);
        const auto hintScans = scans;
        assert(hints.find(8, enumerate, valid).bodyId == 15 && scans == hintScans);
        MotionBodySearchBatch missing;
        missing.add(11);
        assert(missing.find(11, enumerate, valid).freshAbsence);
        const auto absentScans = scans;
        assert(missing.find(11, enumerate, valid).freshAbsence && scans == absentScans + 1);
        MotionBodySearchBatch failed;
        failed.add(10);
        assert(!failed.find(10, [](auto&&) { return false; }, valid).freshAbsence);
        MotionBodySearchBatch full;
        for (unsigned motion = 1; motion <= 129; ++motion) full.add(motion);
        const auto fullScans = scans;
        assert(!full.find(129, enumerate, valid).freshAbsence && scans == fullScans);
        std::printf("Demand-driven damping search: 11 slots instead of 9001; live hints and fresh absence verified\n");

        PureDampingCandidateSet candidates;
        candidates.add({ .bodyId = 1, .motionId = 8, .accepted = true });
        candidates.add({ .bodyId = 2, .motionId = 8, .accepted = true });
        candidates.add({ .bodyId = 3, .motionId = 9, .accepted = true, .heldBySameHand = true });
        candidates.add({ .bodyId = 4, .motionId = 9, .accepted = true });
        assert(candidates.uniqueAcceptedMotionBodyIds().size() == 2);
        assert(candidates.uniqueAcceptedMotionBodyIds()[0] == 1 && candidates.uniqueAcceptedMotionBodyIds()[1] == 4);
        assert(candidates.duplicateMotionSkips() == 1);

        rock::object_physics_body_set::ObjectPhysicsBodySet bodies;
        for (unsigned i = 0; i < 200; ++i) {
            rock::object_physics_body_set::ObjectPhysicsBodyRecord body{};
            body.bodyId = i;
            body.motionId = i / 2;
            body.accepted = true;
            bodies.records.push_back(body);
        }
        unsigned visited = 0;
        assert(bodies.forEachUniqueAcceptedMotion([&](const auto& body) {
            assert(body.bodyId == visited * 2); ++visited;
        }) == 100);
        assert(visited == 100 && bodies.diagnostics.duplicateMotionSkips == 100);
    }

    void evidenceOwnership()
    {
        using rock::WeaponEvidenceSnapshot;
        WeaponEvidenceSnapshot empty;
        assert(empty.size() == 0 && empty.find(1) == nullptr);
        auto records = std::make_shared<WeaponEvidenceSnapshot::Records>();
        records->resize(4);
        (*records)[0].bodyId = 8; // Invalid records must not shadow the first valid match.
        (*records)[1].valid = true; (*records)[1].bodyId = 8;
        (*records)[1].weaponGenerationKey = 100;
        (*records)[1].sourceName = "scope";
        (*records)[1].localMeshPointsGame.resize(1000);
        (*records)[1].pointCount = 1000;
        (*records)[2] = (*records)[1]; (*records)[2].sourceName = "second";
        (*records)[3].valid = true; (*records)[3].bodyId = 0x7FFF'FFFF;
        WeaponEvidenceSnapshot publication{ records };
        const auto retained = publication;
        const auto* found = retained.find(8);
        assert(found == &(*records)[1] && found->sourceName == "scope");
        assert(found->pointCount == found->localMeshPointsGame.size());
        assert(retained.find(0x7FFF'FFFF) == nullptr && retained.find(9) == nullptr);
        const auto* points = found->localMeshPointsGame.data();
        publication = WeaponEvidenceSnapshot{ std::make_shared<const WeaponEvidenceSnapshot::Records>() };
        records.reset();
        assert(publication.size() == 0 && retained.size() == 4);
        assert(retained.find(8) == found && found->localMeshPointsGame.data() == points);
        assert(found->weaponGenerationKey == 100);
    }

    void scanAndGeometryOwnership()
    {
        using rock::havok_runtime::PhysicsSystemBodyScanCache;
        PhysicsSystemBodyScanCache cache;
        int system = 0, instance = 0, world = 0;
        std::array<std::uint32_t, 3> ids{ 7, 9, 7 };
        const PhysicsSystemBodyScanCache::Key key{ &system, &instance, &world, ids.data(), 3 };
        cache.remember(key, ids);
        ids[0] = 100;
        assert(cache.find(key, 3)[0] == 7); // Owns copies, not native array storage.
        auto changed = key; changed.bodyCount = 2;
        assert(cache.find(changed, 2).empty());
        assert(cache.find(key, 4).empty());
        assert(cache.find(key, 1).size() == 1);

        auto mesh = std::make_shared<rock::GeneratedWeaponMeshGeometry>();
        mesh->localTrianglesGame.push_back({});
        rock::GeneratedWeaponMeshIndices indices;
        indices.localIndex.build(mesh->localTrianglesGame);
        auto hull = std::make_shared<rock::GeneratedWeaponHullGeometry>();
        hull->mesh = mesh;
        hull->localPointsGame.push_back({ 1, 2, 3 });
        std::shared_ptr<const rock::GeneratedWeaponHullGeometry> source = std::move(hull);
        auto pending = source;
        auto active = pending;
        const auto* originalPoints = source->localPointsGame.data();
        std::weak_ptr<const rock::GeneratedWeaponHullGeometry> lifetime = source;
        source.reset(); pending.reset(); mesh.reset();
        assert(!lifetime.expired() && active->localPointsGame.data() == originalPoints);
        assert(active->mesh->localTrianglesGame.size() == 1);
        active.reset();
        assert(lifetime.expired());
    }
}

int main()
{
    registryEquivalence();
    exactMeshContacts();
    dampingAndMotionEnumeration();
    scanAndGeometryOwnership();
    evidenceOwnership();
}
