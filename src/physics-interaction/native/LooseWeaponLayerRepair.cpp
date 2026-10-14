#include "physics-interaction/native/LooseWeaponLayerRepair.h"

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpBody.h"
#include "RE/NetImmerse/NiAVObject.h"

#include <array>
#include <chrono>

namespace rock::loose_weapon_layer_repair
{
    namespace
    {
        bool isLoosePhysicalWeapon(RE::TESObjectREFR* ref)
        {
            auto* base = ref ? ref->GetObjectReference() : nullptr;
            auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
            if (!weapon) return false;
            const auto type = weapon->weaponData.type.get();
            // Guns and melee use physical loose-weapon collision. Explosive
            // attack bodies keep their native projectile/trap semantics.
            return type == RE::WEAPON_TYPE::kGun || weapon_type_policy::isEquippedMelee(type);
        }

        struct WorldWriteScope
        {
            RE::hknpWorld* world;
            explicit WorldWriteScope(RE::hknpWorld* value) :
                world(havok_world_lock::detail::currentThreadInsidePhysicsStep() ? nullptr : value)
            {
                if (world) {
                    using Lock = void (*)(RE::hknpWorld*);
                    static REL::Relocation<Lock> lock{ REL::Offset(0x1DF5FB0) };
                    lock(world);
                }
            }
            ~WorldWriteScope()
            {
                if (world) {
                    using Lock = void (*)(RE::hknpWorld*);
                    static REL::Relocation<Lock> unlock{ REL::Offset(0x1DF5FD0) };
                    unlock(world);
                }
            }
        };

        struct Candidate
        {
            havok_runtime::BodySnapshot body{};
            std::uint32_t repairedFilter{};
        };

        struct Result
        {
            const char* stage{ "native-initialization" };
            std::uint32_t nodes{}, bodies{}, candidates{}, changed{}, body{ 0x7FFF'FFFFu };
            std::uint32_t originalFilter{}, originalFlags{}, originalMotion{};
            bool complete{}, rolledBack{ true };
            double repairMs{};
        };

        bool stillOwned(RE::TESObjectREFR* ref, RE::hknpWorld* world, const Candidate& candidate)
        {
            const auto live = havok_runtime::snapshotBody(world, candidate.body.bodyId);
            return live.valid && live.body && live.body == candidate.body.body &&
                live.motionIndex == candidate.body.motionIndex &&
                live.ownerNode == candidate.body.ownerNode &&
                live.collisionObject == candidate.body.collisionObject && live.ownerNode &&
                RE::TESObjectREFR::FindReferenceFor3D(live.ownerNode) == ref;
        }

        void repair(RE::TESObjectREFR* ref, RE::bhkWorld* bhk, RE::hknpWorld* world, Result& result)
        {
            using namespace object_physics_body_set;
            BodySetScanOptions options{};
            options.requireSameResolvedRef = true;
            options.allowUnresolvedRefBodies = false;
            options.maxDepth = 32;
            ObjectPhysicsBodyScanCursor cursor;
            ObjectPhysicsBodyScanCache cache;
            result.stage = "scan-start";
            if (!beginObjectPhysicsBodyScanCache(ref, options, cursor, cache)) return;
            const auto step = advanceObjectPhysicsBodyScanCache(world, options,
                { 2048, 128, 512 }, cursor, cache);
            result.nodes = step.visitedNodes;
            result.stage = "scan-completion";
            if (!step.finished || step.invalidated) return;
            const auto set = buildObjectPhysicsBodySetFromScanCache(bhk, world, ref, options, cache);
            const auto& d = set.diagnostics;
            result.bodies = static_cast<std::uint32_t>(set.records.size());
            if (d.scanFailures || d.invalidPhysicsSystems || d.foreignRefBodySkips ||
                d.unresolvedRefBodySkips || d.depthLimitSkips || d.staleCacheEntrySkips ||
                d.benignScanSkips != d.emptyScanSkips) return;

            std::array<Candidate, 64> candidates{};
            result.stage = "body-ownership";
            for (const auto& record : set.records) {
                // Normal weapon bodies require no second ownership walk.
                if (!correctedFilter(record.filterInfo, record.bodyFlags, record.motionId,
                        record.refResolutionKnown && record.resolvedRef == ref)) continue;
                const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{ record.bodyId });
                result.body = record.bodyId;
                if (!live.valid || !live.body || !live.ownerNode || !live.collisionObject ||
                    !record.refResolutionKnown || record.resolvedRef != ref ||
                    RE::TESObjectREFR::FindReferenceFor3D(live.ownerNode) != ref) return;
                const auto filter = correctedFilter(live.collisionFilterInfo, live.body->flags,
                    live.motionIndex, true);
                if (!filter) continue;
                if (!live.motion) { result.stage = "body-motion"; return; }
                if (result.candidates == candidates.size()) { result.stage = "body-capacity"; return; }
                candidates[result.candidates++] = { live, *filter };
            }

            // Qualification is complete before the first write. Native filter
            // updates invalidate collision caches (14153AF5F..14153AF8E);
            // don't replace this with a raw field write or skip its rebuild.
            result.stage = "filter-write";
            for (std::uint32_t i = 0; i < result.candidates; ++i) {
                const auto& candidate = candidates[i];
                result.body = candidate.body.bodyId.value;
                result.originalFilter = candidate.body.collisionFilterInfo;
                result.originalFlags = candidate.body.body->flags;
                result.originalMotion = candidate.body.motionIndex;
                std::uint32_t filter{};
                if (!stillOwned(ref, world, candidate) ||
                    !havok_runtime::tryReadFilterInfo(world, candidate.body.bodyId, filter) ||
                    filter != candidate.body.collisionFilterInfo) break;
                // Count attempted writes, including a failed readback, so they
                // participate in rollback before the world writer is released.
                result.changed = i + 1;
                if (!havok_runtime::setFilterInfo(world, candidate.body.bodyId, candidate.repairedFilter) ||
                    !havok_runtime::tryReadFilterInfo(world, candidate.body.bodyId, filter) ||
                    filter != candidate.repairedFilter) break;
                if (i + 1 == result.candidates) result.complete = true;
            }
            if (!result.candidates) result.complete = true;
            if (result.complete) { result.stage = "complete"; return; }

            for (std::uint32_t i = 0; i < result.changed; ++i) {
                const auto& candidate = candidates[i];
                std::uint32_t filter{};
                const bool owned = stillOwned(ref, world, candidate) &&
                    havok_runtime::tryReadFilterInfo(world, candidate.body.bodyId, filter);
                if (owned && filter == candidate.body.collisionFilterInfo) continue;
                const bool restored = owned && filter == candidate.repairedFilter &&
                    havok_runtime::setFilterInfo(world, candidate.body.bodyId, candidate.body.collisionFilterInfo) &&
                    havok_runtime::tryReadFilterInfo(world, candidate.body.bodyId, filter) &&
                    filter == candidate.body.collisionFilterInfo;
                result.rolledBack = result.rolledBack && restored;
            }
        }
    }

    void initialize(RE::TESObjectREFR* ref, void (*original)(RE::TESObjectREFR*))
    {
        if (!isLoosePhysicalWeapon(ref) || ref->IsDeleted() || ref->IsDisabled()) { original(ref); return; }
        auto* cell = ref->GetParentCell();
        auto* bhk = cell ? cell->GetbhkWorld() : nullptr;
        auto* world = bhk ? havok_runtime::getHknpWorldFromBhk(bhk) : nullptr;
        if (!world) { original(ref); return; }

        Result result;
        {
            // Reuse the already installed TESObjectREFR::InitHavok observer.
            // 1403EAB75/1403EAE3A enclose attachment in this recursive writer;
            // 141DF9305/141DF944F independently show nested attachment locking.
            // Holding the outer writer prevents a solve between native creation
            // and correction. Initializer-thread ownership ends with this call.
            WorldWriteScope write(world);
            original(ref);
            const auto started = std::chrono::steady_clock::now();
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::LooseWeaponLayerRepair);
            try {
                result.stage = "reference-world";
                if (ref->GetParentCell() == cell && ref->Get3D() && !ref->IsDeleted() && !ref->IsDisabled() &&
                    havok_runtime::getHknpWorldFromBhk(bhk) == world) repair(ref, bhk, world, result);
            } catch (...) {
                // Scan allocations precede all writes. The write/readback loop
                // uses fixed storage and native noexcept-style wrappers.
                result.stage = "repair-exception";
                result.rolledBack = result.changed == 0;
            }
            result.repairMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            if (result.complete) performance_profiler::addEventCount(
                performance_profiler::Scope::LooseWeaponLayerRepair, result.changed);
        }
        if (result.complete && result.changed) {
            ROCK_LOG_INFO(Weapon, "Loose weapon layer repair ref={:08X} base={:08X} nodes={} bodies={} corrected={} lastBody={} filter={:08X} flags={:08X} motion={} repairMs={:.3f}",
                ref->GetFormID(), ref->GetObjectReference()->GetFormID(), result.nodes, result.bodies,
                result.changed, result.body, result.originalFilter, result.originalFlags, result.originalMotion, result.repairMs);
        } else if (!result.complete) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Loose weapon layer repair incomplete ref={:08X} stage={} nodes={} bodies={} candidates={} attempted={} rollback={} body={} repairMs={:.3f}",
                ref->GetFormID(), result.stage, result.nodes, result.bodies, result.candidates,
                result.changed, result.rolledBack, result.body, result.repairMs);
        }
    }
}
