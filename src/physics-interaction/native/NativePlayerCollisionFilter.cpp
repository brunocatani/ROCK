#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/native/MagazineEjectionSnapshot.h"
#include "api/StorageInteractionService.h"
#include "physics-interaction/weapon/LooseWeaponCollisionPolicy.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/performance/ContactPairProfile.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/native/ShellCasingGrace.h"

#include <REL/Relocation.h>
#include <RE/Bethesda/TESObjectREFRs.h>
#include <RE/Bethesda/PlayerCharacter.h>
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
        Snapshot s_authoredHands; // Same game publisher and callback gate.
        struct WeaponHandSnapshot
        {
            RE::hknpWorld* world{ nullptr };
            std::array<WeaponHandBody, kMaximumWeaponHandBodies> bodies{};
            std::size_t count{ 0 };
        };
        WeaponHandSnapshot s_weaponHands; // Same game publisher and callback gate.
        std::atomic<std::uint64_t> s_weaponHandRejectedPairs{ 0 };
        // Independent game-thread publisher and physics-worker read gate.
        // Player-body refresh must never bypass an active blade exception;
        // only blade publication pauses this gate and rebuilds its two bodies.
        PhysicsCallbackQuiescenceGate s_toolContactGate;
        ToolContactExclusion s_toolContactPair;
        PhysicsCallbackQuiescenceGate s_magazineEjectionGate;
        MagazineEjectionSnapshot s_magazineEjection;
        std::atomic<std::uint64_t> s_magazineEjectionRejected{ 0 };
        std::atomic<std::uint64_t> s_magazineEjectionHandPair{ 0 };
        FilterPairs s_original{ nullptr };
        bool s_installed{ false };
        std::atomic<std::uint64_t> s_removedPairs{ 0 };
        std::atomic<std::uint64_t> s_preservedPlayerPairs{ 0 };
        std::atomic<std::uint64_t> s_staleIdentities{ 0 };
        std::atomic<std::uint64_t> s_unresolvedWeaponOwners{ 0 };
        std::atomic<std::uint64_t> s_toolContactRejectedPairs{ 0 };
        std::atomic<std::uint64_t> s_authoredRejectedPairs{ 0 };
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

        enum class WeaponOwner : std::uint8_t { Unknown, Player, Other };

        WeaponOwner resolveWeaponOwner(const havok_runtime::BodySnapshot& body) noexcept
        {
            if (!body.valid || !body.ownerNode || !body.collisionObject) return WeaponOwner::Unknown;
            // Same callback-local native ownership resolver as isLooseWeaponBody.
            // 1403F0925..31 resolves an attached WEAP through its parent;
            // 14062669D..AB independently resolves a collision owner through it.
            __try {
                auto* player = RE::PlayerCharacter::GetSingleton();
                if (!player) return WeaponOwner::Unknown;
                auto* owner = RE::TESObjectREFR::FindReferenceFor3D(body.ownerNode);
                if (!owner) return WeaponOwner::Unknown;
                return owner == player ? WeaponOwner::Player : WeaponOwner::Other;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return WeaponOwner::Unknown;
            }
        }

        struct WeaponOwnerBatch
        {
            struct Entry { BodyIdentity body{}; WeaponOwner owner = WeaponOwner::Unknown; };
            std::array<Entry, 16> entries{};
            std::size_t count = 0;

            WeaponOwner resolve(const havok_runtime::BodySnapshot& body) noexcept
            {
                const auto live = identity(body);
                for (std::size_t i = 0; i < count; ++i) {
                    if (matchesLiveBody(entries[i].body, live)) return entries[i].owner;
                }
                const auto owner = resolveWeaponOwner(body);
                // Values live for this filter invocation only. Capacity limits
                // memoization, never admission; overflow resolves normally.
                if (count < entries.size()) entries[count++] = { live, owner };
                if (owner == WeaponOwner::Unknown) performance_profiler::addCounter(
                    performance_profiler::Counter::NativeWeaponOwnerUnresolved);
                else if (owner == WeaponOwner::Other) performance_profiler::addCounter(
                    performance_profiler::Counter::NativeWeaponOwnerResolvedOther);
                return owner;
            }
        };

        /*
         * One pair's body reads, made on first use and shared by every filter
         * in the pass: a pair's layers are read at most once, and the full
         * snapshots only when a filter that needs them matches the pair.
         */
        class PairReads
        {
        public:
            PairReads(RE::hknpWorld* world, const BodyPair& pair) noexcept : _world(world), _pair(pair) {}

            // kUnknownLayer when a body cannot be read.
            std::uint32_t layerA() noexcept { readLayers(); return _layerA; }
            std::uint32_t layerB() noexcept { readLayers(); return _layerB; }
            const havok_runtime::BodySnapshot& a() noexcept { readSnapshots(); return _a; }
            const havok_runtime::BodySnapshot& b() noexcept { readSnapshots(); return _b; }

        private:
            static std::uint32_t layerOf(const havok_runtime::BodySnapshot& body) noexcept
            {
                return body.valid ? body.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK :
                                    collision_layer_policy::kUnknownLayer;
            }

            std::uint32_t readLayer(std::uint32_t bodyId) const noexcept
            {
                std::uint32_t filter = 0;
                return havok_runtime::tryReadFilterInfo(_world, { bodyId }, filter) ?
                           filter & collision_layer_policy::FO4_LAYER_FILTER_MASK :
                           collision_layer_policy::kUnknownLayer;
            }

            void readLayers() noexcept
            {
                if (_layersRead) return;
                _layersRead = true;
                _layerA = _snapshotsRead ? layerOf(_a) : readLayer(_pair.bodyA);
                _layerB = _snapshotsRead ? layerOf(_b) : readLayer(_pair.bodyB);
            }

            void readSnapshots() noexcept
            {
                if (_snapshotsRead) return;
                _snapshotsRead = true;
                _a = havok_runtime::snapshotBody(_world, RE::hknpBodyId{ _pair.bodyA });
                _b = havok_runtime::snapshotBody(_world, RE::hknpBodyId{ _pair.bodyB });
            }

            RE::hknpWorld* _world;
            BodyPair _pair;
            bool _layersRead = false;
            bool _snapshotsRead = false;
            std::uint32_t _layerA = collision_layer_policy::kUnknownLayer;
            std::uint32_t _layerB = collision_layer_policy::kUnknownLayer;
            havok_runtime::BodySnapshot _a{};
            havok_runtime::BodySnapshot _b{};
        };

        bool rejectNativeWeaponSelfPair(RE::hknpWorld* world, const BodyPair& pair, PairReads& reads, WeaponOwnerBatch& owners)
        {
            using namespace collision_layer_policy;
            const auto layerA = reads.layerA();
            const auto layerB = reads.layerB();
            if (layerA == kUnknownLayer || layerB == kUnknownLayer || !isNativeWeaponSelfContactCandidate(layerA, layerB)) return false;
            const auto weaponId = layerA == FO4_LAYER_WEAPON ? pair.bodyA : pair.bodyB;
            const auto weapon = havok_runtime::snapshotBody(world, {weaponId});
            if (!weapon.valid || (weapon.collisionFilterInfo & FO4_LAYER_FILTER_MASK) != FO4_LAYER_WEAPON ||
                !suppressNativeWeaponSelfContact(layerA, layerB, owners.resolve(weapon) == WeaponOwner::Player)) return false;

            performance_profiler::addCounter(performance_profiler::Counter::NativeWeaponSelfPairsRejected);
            performance_profiler::observeContactPair({
                .world = reinterpret_cast<std::uintptr_t>(world), .bodyA = pair.bodyA, .bodyB = pair.bodyB,
                .layerA = layerA, .layerB = layerB,
            }, performance_profiler::ContactStage::NativeWeaponSelfRejected);
            return true;
        }

        void profilePairs(RE::hknpWorld* world, const BodyPair* pairs, int count,
            performance_profiler::ContactStage stage, performance_profiler::ValueMetric metric) noexcept
        {
            if (!performance_profiler::enabled() || !world || count < 0) return;
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::NativePairProfileBatch);
            performance_profiler::observeValue(metric, static_cast<std::uint64_t>(count));
            if (!pairs || count == 0) return;
            // Bound diagnostic work even if an engine batch is unusually large.
            // This truncates evidence only; the original filter sees every pair.
            constexpr int maxProfiledPairs = 4096;
            const int sampled = (std::min)(count, maxProfiledPairs);
            if (count > sampled) performance_profiler::addCounter(
                performance_profiler::Counter::ContactPairBatchTruncated, count - sampled);
            for (int i = 0; i < sampled; ++i) {
                performance_profiler::observeContactPair({
                    .world = reinterpret_cast<std::uintptr_t>(world),
                    .bodyA = pairs[i].bodyA, .bodyB = pairs[i].bodyB,
                }, stage);
            }
        }

        bool suppressToolContactPair(RE::hknpWorld* world, const BodyPair& pair, PairReads& reads) noexcept
        {
            if (!s_toolContactPair.matchesIds(pair)) return false;
            const auto& a = reads.a();
            const auto& b = reads.b();
            return a.valid && b.valid && s_toolContactPair.suppresses(reinterpret_cast<std::uintptr_t>(world),
                identity(a), identity(b), reads.layerA(), reads.layerB());
        }

        bool suppressMagazineEjectionPair(RE::hknpWorld* world, const BodyPair& pair, PairReads& reads) noexcept
        {
            if(!s_magazineEjection.containsPair(pair.bodyA,pair.bodyB))return false;
            const auto& a=reads.a();
            const auto& b=reads.b();
            if(!a.valid||!b.valid)return false;
            if(s_magazineEjection.suppressesHand(reinterpret_cast<std::uintptr_t>(world),identity(a),identity(b))) {
                const bool firstIsMagazine=MagazineEjectionSnapshot::find(s_magazineEjection.handProtected,s_magazineEjection.handProtectedCount,pair.bodyA)!=nullptr;
                const auto magazine=firstIsMagazine?pair.bodyA:pair.bodyB,hand=firstIsMagazine?pair.bodyB:pair.bodyA;
                std::uint64_t empty=0;s_magazineEjectionHandPair.compare_exchange_strong(empty,(std::uint64_t(magazine)<<32)|hand,std::memory_order_relaxed);
                return true;
            }
            return s_magazineEjection.suppresses(reinterpret_cast<std::uintptr_t>(world),identity(a),identity(b));
        }

        bool isSupportedFixedWeapon(const havok_runtime::BodySnapshot& body) noexcept
        {
            // Callback-local ownership only; no engine pointer is published.
            __try {
                auto* ref=body.ownerNode ? RE::TESObjectREFR::FindReferenceFor3D(body.ownerNode) : nullptr;
                auto* form=ref ? ref->GetObjectReference() : nullptr;
                auto* weapon=form ? form->As<RE::TESObjectWEAP>() : nullptr;
                return weapon && loose_weapon_collision_policy::eligibleType(weapon->weaponData.type.get());
            } __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
        }

        // Independent of native-player skeleton publication: this gate must
        // still protect dynamic weapons when that snapshot is empty.
        bool retiredWeaponHandPairsEnabled(RE::hknpWorld* world) noexcept
        {
            using namespace collision_layer_policy;
            const auto* matrix = havok_runtime::getCollisionFilterMatrix(world);
            return layerPairEnabledFromRow(matrix, ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY, FO4_LAYER_WEAPON) ||
                layerPairEnabledFromRow(matrix, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY, FO4_LAYER_WEAPON);
        }

        bool suppressRetiredWeaponHandPair(PairReads& reads) noexcept
        {
            using namespace collision_layer_policy;
            const auto layerA = reads.layerA();
            const auto layerB = reads.layerB();
            if (layerA == kUnknownLayer || layerB == kUnknownLayer || !isRetiredWeaponHandPair(layerA, layerB)) return false;
            const auto& weapon = layerA == FO4_LAYER_WEAPON ? reads.a() : reads.b();
            if (!weapon.valid || !weapon.body) return true;
            const auto motion = physics_body_classifier::motionTypeFromBodyFlags(weapon.body->flags);
            return !allowsRetiredWeaponHandContact(motion, true) || !isSupportedFixedWeapon(weapon);
        }

        bool suppressAuthoredHandPair(const BodyPair& pair, PairReads& reads) noexcept
        {
            const auto* a = findPlayerBody(s_authoredHands, pair.bodyA);
            const auto* b = findPlayerBody(s_authoredHands, pair.bodyB);
            if (!a && !b) return false;
            const auto& hand = a ? reads.a() : reads.b();
            const auto& other = a ? reads.b() : reads.a();
            return hand.valid && other.valid && suppressAuthoredHandWorldPair(a ? *a : *b,
                identity(hand), other.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK);
        }

        const WeaponHandBody* findWeaponHandBody(std::uint32_t id) noexcept
        {
            const auto end = s_weaponHands.bodies.begin() + s_weaponHands.count;
            const auto found = std::lower_bound(s_weaponHands.bodies.begin(), end, id,
                [](const WeaponHandBody& body, std::uint32_t candidate) { return body.bodyId < candidate; });
            return found != end && found->bodyId == id ? &*found : nullptr;
        }

        bool suppressWeaponHandPair(const BodyPair& pair, PairReads& reads) noexcept
        {
            const auto* a = findWeaponHandBody(pair.bodyA);
            const auto* b = findWeaponHandBody(pair.bodyB);
            if (!a == !b) return false;
            const auto otherLayer = a ? reads.layerB() : reads.layerA();
            return otherLayer != collision_layer_policy::kUnknownLayer &&
                suppressWeaponHandContact(otherLayer, (a ? a : b)->includeLooseWeapons);
        }

        void rebuildBodyIfLive(RE::hknpWorld* world, std::uint32_t bodyId)
        {
            static REL::Relocation<RebuildBodyCaches> rebuild{
                REL::Offset(offsets::kFunc_RebuildBodyCollisionCaches) };
            if (world && havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{ bodyId }).valid) {
                rebuild(world, bodyId);
            }
        }

        struct PlayerPairCounts
        {
            std::uint64_t preserved = 0;
            std::uint64_t stale = 0;
        };

        bool suppressPlayerPair(RE::hknpWorld* world, const BodyPair& pair, PairReads& reads,
            WeaponOwnerBatch& weaponOwners, PlayerPairCounts& counts) noexcept
        {
            const auto* expectedA = findPlayerBody(s_snapshot, pair.bodyA);
            const auto* expectedB = findPlayerBody(s_snapshot, pair.bodyB);
            if (!expectedA && !expectedB) {
                // Reject only positively owned duplicates in the native
                // admitted prefix, before child contacts/solver work. Keep
                // the late VRMeleeImpact guard for event-only bypass paths.
                return rejectNativeWeaponSelfPair(world, pair, reads, weaponOwners);
            }
            const auto& a = reads.a();
            const auto& b = reads.b();
            if (!a.valid || !b.valid) {
                ++counts.stale;
                return false;
            }
            const bool playerA = expectedA && matchesLiveBody(*expectedA, identity(a));
            const bool playerB = expectedB && matchesLiveBody(*expectedB, identity(b));
            if ((expectedA && !playerA) || (expectedB && !playerB)) {
                ++counts.stale;
            }
            const bool looseWeaponContact = playerA ? isLooseWeaponBody(b) :
                playerB ? isLooseWeaponBody(a) : false;
            const bool suppress = suppressPhysicalPair(playerA, playerB,
                a.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK,
                b.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK,
                looseWeaponContact);
            if (!suppress && (playerA || playerB)) {
                ++counts.preserved;
            }
            return suppress;
        }

        /*
         * ROCK's pair filters run as one pass over the pairs the native filter
         * admitted. Each filter is armed once per batch (its gate lease and
         * world check); each pair then goes through the armed filters in a
         * fixed order and stops at the first that rejects it, which keeps
         * every filter's decision and counter as when they ran as separate
         * passes. The pair's bodies are read once, on first use (PairReads).
         * Leases are nonblocking; a publisher that pauses a gate waits only
         * for this pass to end.
         */
        int filterPairs(void* filter, RE::hknpWorld* world, BodyPair* pairs, int count) noexcept
        {
            profilePairs(world, pairs, count, performance_profiler::ContactStage::SimulationInput,
                performance_profiler::ValueMetric::SimulationPairsInput);
            int nativeAdmitted = 0;
            {
                performance_profiler::ScopedTimer nativeTimer(performance_profiler::Scope::NativeSimulationPairFilter);
                nativeAdmitted = s_original(filter, world, pairs, count);
            }
            if (nativeAdmitted >= 0 && nativeAdmitted <= count) profilePairs(world, pairs, nativeAdmitted,
                performance_profiler::ContactStage::SimulationNative, performance_profiler::ValueMetric::SimulationPairsNative);
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::NativePlayerPairFilter);
            if (!world || !pairs || nativeAdmitted <= 0 || nativeAdmitted > count) {
                if (nativeAdmitted >= 0 && nativeAdmitted <= count) profilePairs(world, pairs, nativeAdmitted,
                    performance_profiler::ContactStage::SimulationKept, performance_profiler::ValueMetric::SimulationPairsKept);
                return nativeAdmitted;
            }

            const auto worldAddress = reinterpret_cast<std::uintptr_t>(world);
            const bool shellGrace = shell_casing_grace::filtersPairsIn(world);
            auto toolLease = s_toolContactGate.tryEnterCallback();
            const bool toolContact = toolLease && worldAddress == s_toolContactPair.world;
            auto magazineLease = s_magazineEjectionGate.tryEnterCallback();
            const bool magazineEjection = magazineLease && s_magazineEjection.world == worldAddress;
            const bool retiredWeapon = retiredWeaponHandPairsEnabled(world);
            const bool storage = storage_interaction::filtersPairs();
            auto lease = s_gate.tryEnterCallback();
            const bool authoredHands = lease && world == s_authoredHands.world && s_authoredHands.count != 0;
            const bool weaponHands = lease && world == s_weaponHands.world && s_weaponHands.count != 0;
            const bool playerBodies = lease && world == s_snapshot.world && s_snapshot.count != 0;
            if (!shellGrace && !toolContact && !magazineEjection && !retiredWeapon && !storage &&
                !authoredHands && !weaponHands && !playerBodies) {
                profilePairs(world, pairs, nativeAdmitted, performance_profiler::ContactStage::SimulationKept,
                    performance_profiler::ValueMetric::SimulationPairsKept);
                return nativeAdmitted;
            }

            WeaponOwnerBatch weaponOwners;
            PlayerPairCounts playerCounts;
            std::uint64_t toolRejected = 0;
            std::uint64_t magazineRejected = 0;
            std::uint64_t authoredRejected = 0;
            std::uint64_t weaponHandRejected = 0;
            std::uint64_t playerRemoved = 0;
            const int kept = filterPhysicalPairs(pairs, nativeAdmitted, [&](const BodyPair& pair) {
                PairReads reads(world, pair);
                if (shellGrace && shell_casing_grace::suppressesPair(world, pair.bodyA, pair.bodyB)) return true;
                if (toolContact && suppressToolContactPair(world, pair, reads)) { ++toolRejected; return true; }
                if (magazineEjection && suppressMagazineEjectionPair(world, pair, reads)) { ++magazineRejected; return true; }
                if (retiredWeapon && suppressRetiredWeaponHandPair(reads)) return true;
                if (storage && !storage_interaction::allowsPair(world, pair.bodyA, reads.layerA(), pair.bodyB, reads.layerB())) return true;
                if (authoredHands && suppressAuthoredHandPair(pair, reads)) { ++authoredRejected; return true; }
                if (weaponHands && suppressWeaponHandPair(pair, reads)) { ++weaponHandRejected; return true; }
                if (playerBodies && suppressPlayerPair(world, pair, reads, weaponOwners, playerCounts)) { ++playerRemoved; return true; }
                return false;
            });
            if (toolRejected) s_toolContactRejectedPairs.fetch_add(toolRejected, std::memory_order_relaxed);
            if (magazineRejected) s_magazineEjectionRejected.fetch_add(magazineRejected, std::memory_order_relaxed);
            if (authoredRejected) s_authoredRejectedPairs.fetch_add(authoredRejected, std::memory_order_relaxed);
            if (weaponHandRejected) s_weaponHandRejectedPairs.fetch_add(weaponHandRejected, std::memory_order_relaxed);
            if (playerRemoved) s_removedPairs.fetch_add(playerRemoved, std::memory_order_relaxed);
            if (playerCounts.preserved) s_preservedPlayerPairs.fetch_add(playerCounts.preserved, std::memory_order_relaxed);
            if (playerCounts.stale) s_staleIdentities.fetch_add(playerCounts.stale, std::memory_order_relaxed);
            profilePairs(world, pairs, kept, performance_profiler::ContactStage::SimulationKept,
                performance_profiler::ValueMetric::SimulationPairsKept);
            return kept;
        }

        void invalidateBodyIfCurrent(RE::hknpWorld* world, const BodyIdentity& body)
        {
            static REL::Relocation<RebuildBodyCaches> rebuild{
                REL::Offset(offsets::kFunc_RebuildBodyCollisionCaches) };
            const auto live = havok_runtime::snapshotBody(world, RE::hknpBodyId{ body.bodyId });
            if (live.valid && matchesLiveBody(body, identity(live))) {
                // Rebuild after publication, outside the read gates, so callbacks can
                // consume the new rule. Native pair mutations use this same
                // routine; no body filter bits or query filters are changed.
                rebuild(world, body.bodyId);
            }
        }

        void invalidatePublishedBodies(const Snapshot& before, const Snapshot& after)
        {
            if (before.world == after.world) {
                for (std::size_t i = 0; i < before.count; ++i) {
                    invalidateBodyIfCurrent(after.world, before.bodies[i]);
                }
            }
            for (std::size_t i = 0; i < after.count; ++i) {
                const auto* previous = before.world == after.world ? findPlayerBody(before, after.bodies[i].bodyId) : nullptr;
                if (!previous || *previous != after.bodies[i]) {
                    invalidateBodyIfCurrent(after.world, after.bodies[i]);
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

    bool installed() noexcept { return s_installed; }

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

    void publishAuthoredHands(RE::hknpWorld* world, std::span<const BodyIdentity> bodies)
    {
        Snapshot next{};
        if (s_installed && world && bodies.size() <= next.bodies.size()) {
            next.world = world;
            next.count = bodies.size();
            std::copy(bodies.begin(), bodies.end(), next.bodies.begin());
            std::sort(next.bodies.begin(), next.bodies.begin() + next.count,
                [](const auto& a, const auto& b) { return a.bodyId < b.bodyId; });
        }
        const auto previous = s_authoredHands;
        if (previous.world == next.world && previous.count == next.count &&
            std::equal(next.bodies.begin(), next.bodies.begin() + next.count, previous.bodies.begin())) return;
        {
            auto mutation = s_gate.pauseForMutation();
            s_authoredHands = next;
        }
        s_gate.resumeCallbacks();
        if (world) invalidatePublishedBodies(previous, next);
        ROCK_LOG_INFO(Hand, "Authored handle world collision: bodies={} world={:p} previousRejectedPairs={}",
            next.count, static_cast<void*>(world), s_authoredRejectedPairs.exchange(0, std::memory_order_relaxed));
    }

    void abandon() noexcept
    {
        s_gate.pauseAndWait();
        s_snapshot = {};
        s_authoredHands = {};
        s_weaponHands = {};
        s_toolContactGate.pauseAndWait();
        s_toolContactPair = {};
        s_toolContactRejectedPairs.store(0, std::memory_order_relaxed);
        clearMagazineEjection(nullptr);
    }

    void clearMagazineEjection(RE::hknpWorld* liveWorld)
    {
        const auto previous=s_magazineEjection;
        s_magazineEjectionGate.pauseAndWait();
        s_magazineEjection={};
        s_magazineEjectionHandPair.store(0,std::memory_order_relaxed);
        if(liveWorld&&previous.world==reinterpret_cast<std::uintptr_t>(liveWorld))
            for(std::size_t i=0;i<previous.magazineCount;++i)invalidateBodyIfCurrent(liveWorld,previous.magazines[i]);
    }

    std::uint64_t magazineEjectionRejectedCount() noexcept {return s_magazineEjectionRejected.load(std::memory_order_relaxed);}
    std::uint64_t takeMagazineEjectionHandPair() noexcept {return s_magazineEjectionHandPair.exchange(0,std::memory_order_relaxed);}

    bool publishMagazineEjection(RE::hknpWorld* world,std::span<const std::uint32_t> magazines,
        std::span<const std::uint32_t> weapons,std::span<const std::uint32_t> handProtected,std::span<const std::uint32_t> hands)
    {
        if(magazines.empty()||weapons.empty()){clearMagazineEjection(world);return s_installed&&world;}
        MagazineEjectionSnapshot next;next.world=reinterpret_cast<std::uintptr_t>(world);
        if(!s_installed||!world||magazines.size()>next.magazines.size()||weapons.size()>next.weapons.size()||handProtected.size()>next.handProtected.size()||hands.size()>next.hands.size())return false;
        const auto collect=[&](auto ids,auto& out,std::size_t& count) {
            for(auto id:ids) {
                const auto body=havok_runtime::snapshotBody(world,{id});
                const auto value=identity(body);
                if(!body.valid||!matchesLiveBody(value,value)) {
                    ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Ejection simulation identity rejected body={} valid={} collision=0x{:X} node=0x{:X}",id,body.valid,value.collisionObject,value.ownerNode);
                    return false;
                }
                out[count++]=value;
            }
            std::sort(out.begin(),out.begin()+count,[](const auto& a,const auto& b){return a.bodyId<b.bodyId;});
            return true;
        };
        if(!collect(magazines,next.magazines,next.magazineCount)||!collect(weapons,next.weapons,next.weaponCount)||
            !collect(handProtected,next.handProtected,next.handProtectedCount)||!collect(hands,next.hands,next.handCount))return false;
        const auto previous=s_magazineEjection;
        if(previous==next)return true;
        {
            auto mutation=s_magazineEjectionGate.pauseForMutation();
            s_magazineEjection=next;
        }
        s_magazineEjectionGate.resumeCallbacks();
        // Rebuild the affected magazines after publication, outside the callback
        // gate, for both acquisition and removal. World/ray filters stay intact.
        if(previous.world==next.world)for(std::size_t i=0;i<previous.magazineCount;++i)invalidateBodyIfCurrent(world,previous.magazines[i]);
        for(std::size_t i=0;i<next.magazineCount;++i) {
            const auto* old=previous.world==next.world?MagazineEjectionSnapshot::find(previous.magazines,previous.magazineCount,next.magazines[i].bodyId):nullptr;
            if(!old||*old!=next.magazines[i]||previous.hands!=next.hands||previous.handProtected!=next.handProtected)invalidateBodyIfCurrent(world,next.magazines[i]);
        }
        return true;
    }

    void publishWeaponHands(RE::hknpWorld* world, std::span<const WeaponHandBody> bodies)
    {
        WeaponHandSnapshot next{};
        if (s_installed && world) {
            next.world = world;
            next.count = (std::min)(bodies.size(), next.bodies.size());
            std::copy_n(bodies.begin(), next.count, next.bodies.begin());
            std::sort(next.bodies.begin(), next.bodies.begin() + next.count,
                [](const WeaponHandBody& a, const WeaponHandBody& b) { return a.bodyId < b.bodyId; });
        }
        const auto previous = s_weaponHands; // Only this game thread writes it.
        if (previous.world == next.world && previous.count == next.count &&
            std::equal(next.bodies.begin(), next.bodies.begin() + next.count, previous.bodies.begin())) return;
        {
            auto mutation = s_gate.pauseForMutation();
            s_weaponHands = next;
        }
        s_gate.resumeCallbacks();
        // Rebuild every body whose exclusion changed, after publication and
        // outside the read gate, so pairs already formed with the weapon drop
        // and pairs excluded until now form again.
        if (world) {
            auto findIn = [](const WeaponHandSnapshot& snapshot, std::uint32_t id) -> const WeaponHandBody* {
                for (std::size_t i = 0; i < snapshot.count; ++i) {
                    if (snapshot.bodies[i].bodyId == id) return &snapshot.bodies[i];
                }
                return nullptr;
            };
            if (previous.world == world) {
                for (std::size_t i = 0; i < previous.count; ++i) {
                    const auto* now = findIn(next, previous.bodies[i].bodyId);
                    if (!now || *now != previous.bodies[i]) rebuildBodyIfLive(world, previous.bodies[i].bodyId);
                }
            }
            for (std::size_t i = 0; i < next.count; ++i) {
                if (!(previous.world == world && findIn(previous, next.bodies[i].bodyId))) rebuildBodyIfLive(world, next.bodies[i].bodyId);
            }
        }
        ROCK_LOG_DEBUG(Hand, "Weapon-owned hand pair exclusion: bodies={} world={:p} previousRejectedPairs={}",
            next.count, static_cast<void*>(next.world), s_weaponHandRejectedPairs.exchange(0, std::memory_order_relaxed));
    }

    bool publishToolContactExclusion(RE::hknpWorld* world, std::uint32_t weaponBody, std::uint32_t targetBody)
    {
        if (!s_installed || !world) {
            ROCK_LOG_WARN(Weapon, "Tool contact exclusion rejected: stage=hook-or-world-unavailable installed={}", s_installed);
            return false;
        }
        const auto weapon = havok_runtime::snapshotBody(world, RE::hknpBodyId{ weaponBody });
        const auto target = havok_runtime::snapshotBody(world, RE::hknpBodyId{ targetBody });
        const ToolContactExclusion next{ reinterpret_cast<std::uintptr_t>(world), identity(weapon), identity(target),
            target.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK };
        if (!weapon.valid || !target.valid || !next.suppresses(next.world, next.weapon, next.target,
                weapon.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK,
                target.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK)) {
            ROCK_LOG_WARN(Weapon, "Tool contact exclusion rejected: stage=body-identity-or-layer weapon={} target={} valid={}/{} objects=0x{:X}/0x{:X} nodes=0x{:X}/0x{:X} filters=0x{:X}/0x{:X}",
                weaponBody, targetBody, weapon.valid, target.valid, next.weapon.collisionObject, next.target.collisionObject,
                next.weapon.ownerNode, next.target.ownerNode, weapon.collisionFilterInfo, target.collisionFilterInfo);
            return false;
        }
        // Only this game's equipped-tool owner may publish. A second request
        // cannot silently replace another live exception or a recycled body.
        if (s_toolContactPair.valid()) {
            const bool same = s_toolContactPair.world == next.world && s_toolContactPair.weapon == next.weapon &&
                s_toolContactPair.target == next.target && s_toolContactPair.targetLayer == next.targetLayer;
            if (!same) ROCK_LOG_WARN(Weapon, "Tool contact exclusion rejected: stage=previous-pair-still-owned");
            return same;
        }
        {
            auto mutation = s_toolContactGate.pauseForMutation();
            s_toolContactRejectedPairs.store(0, std::memory_order_relaxed);
            s_toolContactPair = next;
        }
        s_toolContactGate.resumeCallbacks();
        invalidateBodyIfCurrent(world, next.weapon);
        invalidateBodyIfCurrent(world, next.target);
        ROCK_LOG_INFO(Weapon, "Tool contact exclusion published: weapon={} target={} world=0x{:X}", weaponBody, targetBody, next.world);
        return true;
    }

    bool hasToolContactExclusion(RE::hknpWorld* world, std::uint32_t weaponBody, std::uint32_t targetBody) noexcept
    {
        if (!s_installed || !s_toolContactPair.valid() || s_toolContactPair.world != reinterpret_cast<std::uintptr_t>(world) ||
            s_toolContactPair.weapon.bodyId != weaponBody || s_toolContactPair.target.bodyId != targetBody) return false;
        const auto weapon = havok_runtime::snapshotBody(world, RE::hknpBodyId{ weaponBody });
        const auto target = havok_runtime::snapshotBody(world, RE::hknpBodyId{ targetBody });
        return weapon.valid && target.valid && s_toolContactPair.suppresses(reinterpret_cast<std::uintptr_t>(world),
            identity(weapon), identity(target),
            weapon.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK,
            target.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK);
    }

    std::uint64_t toolContactRejectedCount() noexcept
    {
        return s_toolContactRejectedPairs.load(std::memory_order_relaxed);
    }

    void clearToolContactExclusion(RE::hknpWorld* liveWorld)
    {
        const auto previous = s_toolContactPair;
        if (!previous.valid()) return;
        std::uint64_t rejectedPairs = 0;
        s_toolContactGate.pauseAndWait();
        s_toolContactPair = {};
        rejectedPairs = s_toolContactRejectedPairs.exchange(0, std::memory_order_relaxed);
        if (liveWorld && previous.world == reinterpret_cast<std::uintptr_t>(liveWorld)) {
            invalidateBodyIfCurrent(liveWorld, previous.weapon);
            invalidateBodyIfCurrent(liveWorld, previous.target);
        }
        ROCK_LOG_INFO(Weapon, "Tool contact exclusion cleared: weapon={} target={} rejectedPairs={}",
            previous.weapon.bodyId, previous.target.bodyId, rejectedPairs);
    }
}
