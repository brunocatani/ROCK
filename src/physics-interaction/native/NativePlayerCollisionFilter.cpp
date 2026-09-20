#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/native/ShellCasingGrace.h"

#include <REL/Relocation.h>
#include <RE/Bethesda/TESObjectREFRs.h>
#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>

namespace rock::native_player_collision
{
    namespace
    {
        using FilterPairs = int (*)(void*, RE::hknpWorld*, BodyPair*, int);
        using RebuildBodyCaches = void (*)(RE::hknpWorld*, std::uint32_t);

        struct Snapshot
        {
            RE::hknpWorld* world{ nullptr };
            std::array<BodyIdentity, kMaximumPlayerBodies> bodies{};
            std::size_t count{ 0 };
        };

        // Explicit process-lifetime service. install/publish/abandon run on the
        // game thread; callbacks hold only a nonblocking read lease after the
        // original native filter returns. No native lock is taken under this gate.
        PhysicsCallbackQuiescenceGate s_gate;
        Snapshot s_snapshot;
        FilterPairs s_original{ nullptr };
        bool s_installed{ false };
        std::atomic<std::uint64_t> s_removedPairs{ 0 };
        std::atomic<std::uint64_t> s_preservedPlayerPairs{ 0 };
        std::atomic<std::uint64_t> s_staleIdentities{ 0 };
        std::atomic<std::uint64_t> s_unresolvedWeaponOwners{ 0 };
        std::uint64_t s_nextReportMs{ 0 };

        BodyIdentity identity(const havok_runtime::BodySnapshot& body)
        {
            return { body.bodyId.value, body.motionIndex,
                reinterpret_cast<std::uintptr_t>(body.collisionObject),
                reinterpret_cast<std::uintptr_t>(body.ownerNode) };
        }

        const BodyIdentity* findPlayerBody(const Snapshot& snapshot, std::uint32_t id)
        {
            const auto end = snapshot.bodies.begin() + snapshot.count;
            const auto found = std::lower_bound(snapshot.bodies.begin(), end, id,
                [](const BodyIdentity& body, std::uint32_t candidate) { return body.bodyId < candidate; });
            return found != end && found->bodyId == id ? &*found : nullptr;
        }

        int filterPairs(void* filter, RE::hknpWorld* world, BodyPair* pairs, int count) noexcept
        {
            const int nativeAdmitted = s_original(filter, world, pairs, count);
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::NativePlayerPairFilter);
            const int admitted = nativeAdmitted > 0 && nativeAdmitted <= count ?
                shell_casing_grace::filterPairs(world, pairs, nativeAdmitted) : nativeAdmitted;
            auto lease = s_gate.tryEnterCallback();
            if (!lease || !world || world != s_snapshot.world || s_snapshot.count == 0 ||
                !pairs || admitted <= 0 || admitted > count) {
                return admitted;
            }

            std::uint64_t preserved = 0;
            std::uint64_t stale = 0;
            const int kept = filterPhysicalPairs(pairs, admitted, [&](const BodyPair& pair) {
                const auto* expectedA = findPlayerBody(s_snapshot, pair.bodyA);
                const auto* expectedB = findPlayerBody(s_snapshot, pair.bodyB);
                if (!expectedA && !expectedB) {
                    return false;
                }
                const auto a = havok_runtime::snapshotBody(world, RE::hknpBodyId{ pair.bodyA });
                const auto b = havok_runtime::snapshotBody(world, RE::hknpBodyId{ pair.bodyB });
                if (!a.valid || !b.valid) {
                    ++stale;
                    return false;
                }
                const bool playerA = expectedA && matchesLiveBody(*expectedA, identity(a));
                const bool playerB = expectedB && matchesLiveBody(*expectedB, identity(b));
                if ((expectedA && !playerA) || (expectedB && !playerB)) {
                    ++stale;
                }
                const bool looseWeaponContact = playerA ? isLooseWeaponBody(b) :
                    playerB ? isLooseWeaponBody(a) : false;
                const bool suppress = suppressPhysicalPair(playerA, playerB,
                    a.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK,
                    b.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK,
                    looseWeaponContact);
                if (!suppress && (playerA || playerB)) {
                    ++preserved;
                }
                return suppress;
            });
            s_removedPairs.fetch_add(admitted - kept, std::memory_order_relaxed);
            s_preservedPlayerPairs.fetch_add(preserved, std::memory_order_relaxed);
            s_staleIdentities.fetch_add(stale, std::memory_order_relaxed);
            return kept;
        }

        void invalidatePublishedBodies(const Snapshot& before, const Snapshot& after)
        {
            static REL::Relocation<RebuildBodyCaches> rebuild{
                REL::Offset(offsets::kFunc_RebuildBodyCollisionCaches) };
            const auto invalidate = [&](const BodyIdentity& body) {
                const auto live = havok_runtime::snapshotBody(after.world, RE::hknpBodyId{ body.bodyId });
                if (live.valid && matchesLiveBody(body, identity(live))) {
                    // Native setter and counted-pair mutations use this exact
                    // routine. Publication rebuilds the protected set, including
                    // pairs admitted while its read gate was briefly paused.
                    // Body collision filter bits never change.
                    rebuild(after.world, body.bodyId);
                }
            };
            if (before.world == after.world) {
                for (std::size_t i = 0; i < before.count; ++i) {
                    invalidate(before.bodies[i]);
                }
            }
            for (std::size_t i = 0; i < after.count; ++i) {
                const auto* previous = before.world == after.world ? findPlayerBody(before, after.bodies[i].bodyId) : nullptr;
                if (!previous || *previous != after.bodies[i]) {
                    invalidate(after.bodies[i]);
                }
            }
        }
    }

    bool isLooseWeaponBody(const havok_runtime::BodySnapshot& body) noexcept
    {
        if (!body.valid ||
            (body.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK) != collision_layer_policy::FO4_LAYER_WEAPON) {
            return false;
        }
        // The same read-only resolver is used by native contact handling.
        // FO4VR 3F0890 climbs scene ownership; 3F0913..3F0931 replaces an
        // attached WEAP reference with its parent actor when one exists.
        // No reference or scene pointer survives this callback.
        __try {
            auto* ref = body.ownerNode ? RE::TESObjectREFR::FindReferenceFor3D(body.ownerNode) : nullptr;
            auto* base = ref ? ref->GetObjectReference() : nullptr;
            if (base) {
                return base->Is(RE::ENUM_FORM_ID::kWEAP);
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            // Unknown ownership keeps melee contacts native. Report from the
            // game-thread publication path, never format logs on a worker.
        }
        s_unresolvedWeaponOwners.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    bool install() noexcept
    {
        if (s_installed) {
            return true;
        }
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            return false;
        }
        constexpr std::array<std::uint8_t, 15> pairPrefix{
            0x40, 0x56, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56,
            0x48, 0x83, 0xEC, 0x28, 0x4D, 0x8B, 0xE0 };
        constexpr std::array<std::uint8_t, 16> rebuildPrefix{
            0x48, 0x89, 0x5C, 0x24, 0x18, 0x89, 0x54, 0x24,
            0x10, 0x55, 0x56, 0x57, 0x48, 0x83, 0xEC, 0x20 };
        const auto target = REL::Offset(offsets::kFunc_BhkCollisionFilter_FilterBodyPairs).address();
        const auto rebuild = REL::Offset(offsets::kFunc_RebuildBodyCollisionCaches).address();
        const auto slot = REL::Offset(offsets::kVtableEntry_BhkCollisionFilter_FilterBodyPairs).address();
        std::uintptr_t current = 0;
        std::array<std::uint8_t, pairPrefix.size()> actualPair{};
        std::array<std::uint8_t, rebuildPrefix.size()> actualRebuild{};
        if (!native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(slot), current) || current != target ||
            !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(target), actualPair.data(), actualPair.size()) || actualPair != pairPrefix ||
            !native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(rebuild), actualRebuild.data(), actualRebuild.size()) || actualRebuild != rebuildPrefix) {
            ROCK_LOG_ERROR(Init, "Native player contact filter unavailable: simulation slot or cache-rebuild bytes did not match FO4VR");
            return false;
        }
        DWORD protection = 0;
        if (!VirtualProtect(reinterpret_cast<void*>(slot), sizeof(void*), PAGE_READWRITE, &protection)) {
            ROCK_LOG_ERROR(Init, "Native player contact filter unavailable: cannot write simulation vtable slot");
            return false;
        }
        s_original = reinterpret_cast<FilterPairs>(target);
        const auto replaced = InterlockedCompareExchangePointer(reinterpret_cast<void* volatile*>(slot),
            reinterpret_cast<void*>(&filterPairs), reinterpret_cast<void*>(target));
        DWORD ignored = 0;
        if (!VirtualProtect(reinterpret_cast<void*>(slot), sizeof(void*), protection, &ignored)) {
            ROCK_LOG_WARN(Init, "Native player contact filter installed but vtable page protection restoration failed");
        }
        if (replaced != reinterpret_cast<void*>(target)) {
            ROCK_LOG_ERROR(Init, "Native player contact filter unavailable: simulation slot changed during installation");
            return false;
        }
        s_installed = true;
        shell_casing_grace::install();
        ROCK_LOG_INFO(Init, "Native player contact filter installed: simulation pairs only; native body filters and ray/shape query slots preserved");
        return true;
    }

    void publish(RE::hknpWorld* world, std::span<const BodyIdentity> bodies)
    {
        if (!s_installed || !world) {
            abandon();
            return;
        }
        Snapshot next{};
        next.world = world;
        next.count = (std::min)(bodies.size(), next.bodies.size());
        if (next.count != 0) {
            std::copy_n(bodies.begin(), next.count, next.bodies.begin());
        }
        std::sort(next.bodies.begin(), next.bodies.begin() + next.count,
            [](const BodyIdentity& a, const BodyIdentity& b) { return a.bodyId < b.bodyId; });
        const Snapshot previous = s_snapshot; // Only this game thread writes it.
        const bool changed = previous.world != next.world || previous.count != next.count ||
            !std::equal(next.bodies.begin(), next.bodies.begin() + next.count, previous.bodies.begin());
        if (changed) {
            auto mutation = s_gate.pauseForMutation();
            s_snapshot = next;
        }
        s_gate.resumeCallbacks();
        if (changed) {
            invalidatePublishedBodies(previous, next);
        }

        if (previous.world != next.world || previous.count != next.count) {
            ROCK_LOG_INFO(PhysicsSafety, "Native player contact filter snapshot: bodies={} world={:p}; native hit filters unchanged",
                next.count, static_cast<void*>(world));
        }
        const auto now = GetTickCount64();
        if (now >= s_nextReportMs) {
            s_nextReportMs = now + 5000;
            const auto removed = s_removedPairs.exchange(0, std::memory_order_relaxed);
            const auto preserved = s_preservedPlayerPairs.exchange(0, std::memory_order_relaxed);
            const auto stale = s_staleIdentities.exchange(0, std::memory_order_relaxed);
            const auto unresolvedWeapons = s_unresolvedWeaponOwners.exchange(0, std::memory_order_relaxed);
            if (removed || preserved || stale || unresolvedWeapons) {
                ROCK_LOG_INFO(PhysicsSafety, "Native player contact filter: physicalPairsRemoved={} playerPairsPreserved={} staleIdentitiesPreserved={} unresolvedWeaponOwnersPreserved={}",
                    removed, preserved, stale, unresolvedWeapons);
            }
        }
    }

    void abandon() noexcept
    {
        s_gate.pauseAndWait();
        s_snapshot = {};
    }
}
