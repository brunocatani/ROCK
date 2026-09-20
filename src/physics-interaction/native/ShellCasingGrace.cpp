#include "physics-interaction/native/ShellCasingGrace.h"
#include "physics-interaction/native/ShellCasingGracePolicy.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/WeaponTypes.h"

#include <RE/Bethesda/PlayerCharacter.h>
#include <RE/Bethesda/TESBoundObjects.h>
#include <RE/NetImmerse/NiNode.h>
#include <F4SE/Trampoline.h>
#include <REL/Relocation.h>
#include <Windows.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <utility>

namespace rock::shell_casing_grace
{
    namespace
    {
        using Eject = void (*)(RE::TESObjectREFR*, const void*, std::uint32_t);
        using Attach = bool (*)(RE::bhkWorld*, RE::NiAVObject*, RE::NiAVObject*);
        using Rebuild = void (*)(RE::hknpWorld*, std::uint32_t);
        Eject s_eject{};
        Attach s_attach{};
        bool s_installed{};

        // Explicit process-lifetime service. Native birth and pre-collide
        // expiry are writers. Worker pair filters read atomic value snapshots;
        // no native object pointer is retained for later dereferencing. Slot
        // writers claim an odd version with one CAS and never wait. Atomic
        // payloads make failed seqlock reads legal even during world teardown.
        constexpr std::size_t kSlots = 256;
        constexpr std::size_t kProbes = 8;
        struct Record
        {
            BodyIdentity body;
            std::uintptr_t world{};
            std::uint64_t epoch{};
            std::uint64_t weaponGeneration{};
            std::uint64_t birth{};
            GraceWindow window;
        };
        struct Slot
        {
            std::atomic<std::uint64_t> version{};
            std::atomic<std::uint32_t> id{ kInvalidBody }, motion{};
            std::atomic<std::uintptr_t> collisionObject{}, world{};
            std::atomic<std::uint64_t> epoch{}, weaponGeneration{}, birth{}, bornSolve{};
            std::atomic<double> bornSeconds{}, expiresSeconds{};

            bool read(Record& out, std::uint64_t& stamp) const noexcept
            {
                stamp = version.load(std::memory_order_acquire);
                if (stamp & 1) return false;
                out = { { id.load(), motion.load(), collisionObject.load() }, world.load(),
                    epoch.load(), weaponGeneration.load(), birth.load(),
                    { bornSeconds.load(), expiresSeconds.load(), bornSolve.load() } };
                return version.load(std::memory_order_acquire) == stamp;
            }
            bool claim(std::uint64_t stamp) noexcept
            {
                return version.compare_exchange_strong(stamp, stamp + 1, std::memory_order_acq_rel);
            }
            void write(const Record& value, std::uint64_t stamp) noexcept
            {
                id.store(value.body.id); motion.store(value.body.motion);
                collisionObject.store(value.body.collisionObject); world.store(value.world);
                epoch.store(value.epoch); weaponGeneration.store(value.weaponGeneration);
                birth.store(value.birth); bornSolve.store(value.window.bornSolve);
                bornSeconds.store(value.window.bornSeconds); expiresSeconds.store(value.window.expiresSeconds);
                version.store(stamp + 2, std::memory_order_release);
            }
        };
        std::array<Slot, kSlots> s_slots;
        std::atomic<std::uint32_t> s_active{};
        std::atomic<RE::hknpWorld*> s_world{};
        std::atomic<std::uint64_t> s_epoch{ 1 }, s_birth{}, s_completedSolve{};
        std::atomic<double> s_simulatedSeconds{};
        std::atomic<float> s_milliseconds{ kDefaultMilliseconds };

        struct WeaponPublication
        {
            std::atomic<std::uint64_t> version{}, generation{};
            std::atomic<std::uint32_t> formId{}, count{};
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_COLLISION_BODIES> bodies{};
        } s_weapon;
        struct BirthContext
        {
            std::uint64_t weaponGeneration{};
            std::uint64_t birth{};
        };
        thread_local BirthContext s_birthContext;

        enum class Failure : std::uint32_t { None, Source, World, TreeBudget, BodyScan, BodyIdentity, Capacity, Timing };
        const char* failureName(Failure stage) noexcept
        {
            switch (stage) {
            case Failure::None: return "none";
            case Failure::Source: return "source-weapon";
            case Failure::World: return "attached-world";
            case Failure::TreeBudget: return "scene-traversal-budget";
            case Failure::BodyScan: return "physics-system-bodies";
            case Failure::BodyIdentity: return "live-body-identity";
            case Failure::Capacity: return "tracking-capacity";
            case Failure::Timing: return "unmeasured-physics-time";
            }
            return "unknown";
        }
        std::atomic<std::uint64_t> s_registered{}, s_suppressed{}, s_restored{}, s_stale{}, s_failures{};
        std::atomic<Failure> s_failure{};
        std::atomic<std::uint32_t> s_lastBody{ kInvalidBody }, s_lastRestoredBody{ kInvalidBody };
        std::atomic<double> s_lastRestoreMs{};
        std::uint64_t s_nextReportMs{}; // Game thread only.

        void fail(Failure stage) noexcept
        {
            s_failure.store(stage, std::memory_order_relaxed);
            s_failures.fetch_add(1, std::memory_order_relaxed);
        }

        BodyIdentity identity(const havok_runtime::BodyIdentitySnapshot& body) noexcept
        {
            return { body.bodyId.value, body.motionIndex, reinterpret_cast<std::uintptr_t>(body.collisionObject) };
        }

        std::uint64_t weaponGenerationFor(std::uint32_t formId) noexcept
        {
            const auto version = s_weapon.version.load(std::memory_order_acquire);
            if ((version & 1) || formId == 0 || s_weapon.formId.load() != formId || s_weapon.count.load() == 0) return 0;
            const auto generation = s_weapon.generation.load();
            return s_weapon.version.load(std::memory_order_acquire) == version ? generation : 0;
        }

        bool weaponContains(std::uint32_t body, std::uint64_t generation) noexcept
        {
            const auto version = s_weapon.version.load(std::memory_order_acquire);
            if ((version & 1) || !generation || s_weapon.generation.load() != generation) return false;
            std::size_t low = 0, high = (std::min)(s_weapon.count.load(), static_cast<std::uint32_t>(s_weapon.bodies.size()));
            while (low < high) {
                const auto mid = low + (high - low) / 2;
                const auto candidate = s_weapon.bodies[mid].load();
                if (candidate < body) low = mid + 1;
                else if (candidate > body) high = mid;
                else return s_weapon.version.load(std::memory_order_acquire) == version;
            }
            return false;
        }

        void recordBirth(RE::hknpWorld* world, std::uint32_t id) noexcept
        {
            const auto live = havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{ id });
            if (!live.valid || !live.collisionObject || !live.motionIndex) { fail(Failure::BodyIdentity); return; }
            if ((live.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK) != collision_layer_policy::FO4_LAYER_SHELLCASING) return;
            const auto epoch = s_epoch.load();
            Record born{ identity(live), reinterpret_cast<std::uintptr_t>(world), epoch,
                s_birthContext.weaponGeneration, s_birthContext.birth,
                makeWindow(s_simulatedSeconds.load(), s_completedSolve.load(), s_milliseconds.load()) };
            for (std::size_t probe = 0; probe < kProbes; ++probe) {
                auto& slot = s_slots[(id + probe) % kSlots];
                Record prior{};
                std::uint64_t stamp{};
                if (!slot.read(prior, stamp)) continue;
                const bool occupied = prior.body.id != kInvalidBody;
                if (occupied && prior.epoch == epoch && prior.body.id != id) continue;
                if (occupied && prior.birth == born.birth && prior.body.matches(born.body)) return;
                if (!slot.claim(stamp)) continue;
                if (!occupied) s_active.fetch_add(1);
                slot.write(born, stamp);
                s_registered.fetch_add(1, std::memory_order_relaxed);
                s_lastBody.store(id, std::memory_order_relaxed);
                return;
            }
            fail(Failure::Capacity);
        }

        bool visitBody(std::uint32_t id, void* context)
        {
            recordBirth(static_cast<RE::hknpWorld*>(context), id);
            return true;
        }

        void recordAttachedTree(RE::bhkWorld* bhk, RE::NiAVObject* root)
        {
            auto* world = havok_runtime::getHknpWorldFromBhk(bhk);
            if (!world || world != s_world.load() || !root) { fail(Failure::World); return; }
            std::array<RE::NiAVObject*, 128> pending{};
            std::size_t count = 1, visited = 0;
            pending[0] = root;
            while (count && visited++ < pending.size()) {
                auto* object = pending[--count];
                if (!object) continue;
                if (auto* collision = object->collisionObject.get()) {
                    const auto scan = havok_runtime::forEachPhysicsSystemBodyIdDetailed(collision, world, 32, &visitBody, world);
                    if (scan.status != havok_runtime::PhysicsSystemBodyScanStatus::Enumerated) fail(Failure::BodyScan);
                }
                if (auto* node = object->IsNode()) {
                    const auto& children = node->GetRuntimeData().children;
                    if (children.capacity() > pending.size() || count + children.capacity() > pending.size()) {
                        fail(Failure::TreeBudget); return;
                    }
                    for (std::uint16_t i = 0; i < children.capacity(); ++i) if (children[i]) pending[count++] = children[i].get();
                }
            }
            if (count) fail(Failure::TreeBudget);
        }

        void recordAttachedTreeGuarded(RE::bhkWorld* world, RE::NiAVObject* root) noexcept
        {
            __try { recordAttachedTree(world, root); }
            __except (EXCEPTION_EXECUTE_HANDLER) { fail(Failure::BodyIdentity); }
        }

        bool attachHook(RE::bhkWorld* world, RE::NiAVObject* physicsRoot, RE::NiAVObject* visualRoot)
        {
            const bool result = s_attach(world, physicsRoot, visualRoot);
            // FO4VR 71DAD9 acquires the world write lock; 71DAF2 attaches the
            // bodies; 71DB44 releases it. Capture here, before that release.
            if (s_birthContext.weaponGeneration) recordAttachedTreeGuarded(world, physicsRoot);
            return result;
        }

        void ejectHook(RE::TESObjectREFR* actor, const void* instance, std::uint32_t equipIndex)
        {
            const auto previous = std::exchange(s_birthContext, BirthContext{});
            if (actor == RE::PlayerCharacter::GetSingleton() && s_milliseconds.load() > 0.0f) {
                RE::TESObjectWEAP* weapon{};
                // 330C96 loads instance[0]; 330CE1 resolves its weapon data.
                // The caller 0FF41C1/0FF41D9 supplies that same native instance.
                if (native_memory::tryReadField(instance, 0, weapon) && weapon) {
                    s_birthContext.weaponGeneration = weaponGenerationFor(weapon->GetFormID());
                    s_birthContext.birth = s_birth.fetch_add(1) + 1;
                    if (!s_birthContext.weaponGeneration) fail(Failure::Source);
                } else fail(Failure::Source);
            }
            s_eject(actor, instance, equipIndex);
            s_birthContext = previous;
        }

        bool suppress(RE::hknpWorld* world, std::uint32_t casing, std::uint32_t weapon) noexcept
        {
            const auto epoch = s_epoch.load();
            for (std::size_t probe = 0; probe < kProbes; ++probe) {
                Record record{};
                std::uint64_t stamp{};
                const auto& slot = s_slots[(casing + probe) % kSlots];
                if (slot.id.load(std::memory_order_relaxed) != casing) continue;
                if (!slot.read(record, stamp) || record.body.id != casing || record.epoch != epoch ||
                    record.world != reinterpret_cast<std::uintptr_t>(world) || !weaponContains(weapon, record.weaponGeneration)) continue;
                const auto live = havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{ casing });
                return live.valid && record.body.matches(identity(live)) &&
                    (live.collisionFilterInfo & collision_layer_policy::FO4_LAYER_FILTER_MASK) == collision_layer_policy::FO4_LAYER_SHELLCASING &&
                    slot.version.load() == stamp && s_epoch.load() == epoch;
            }
            return false;
        }

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t offset, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(offset).address()), actual.data(), N) && actual == expected;
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        constexpr std::array<std::uint8_t, 22> ejectPrefix{
            0x4C,0x8B,0xDC,0x45,0x89,0x43,0x18,0x55,0x41,0x56,0x48,0x8D,0x6C,0x24,0x88,0x48,0x81,0xEC,0x78,0x01,0x00,0x00 };
        constexpr std::array<std::uint8_t, 5> attachCall{ 0xE8,0x59,0xB7,0x6D,0x01 };
        constexpr std::array<std::uint8_t, 26> initializerPrefix{
            0x40,0x56,0x57,0x41,0x54,0x41,0x56,0x48,0x83,0xEC,0x28,0x48,0x8B,0x01,0x4C,0x8B,0xE2,0x4C,0x8B,0xF1,0xFF,0x90,0x70,0x01,0x00,0x00 };
        if (!bytesMatch(0x330C80, ejectPrefix) || !bytesMatch(0x71DAF2, attachCall) || !bytesMatch(0x71D9B0, initializerPrefix)) {
            ROCK_LOG_ERROR(Init, "Shell casing grace unavailable: native ejection/locked attachment bytes differ from FO4VR");
            return false;
        }
        // Install the passive attachment observer first. Without the ejection
        // scope it does nothing, so failure of the second hook stays native.
        try {
            s_attach = reinterpret_cast<Attach>(F4SE::GetTrampoline().write_call<5>(REL::Offset(0x71DAF2).address(), &attachHook));
            void* original{};
            if (!entry_trampoline_hook::install("ShellCasingEject", 0x330C80, ejectPrefix.data(), ejectPrefix.size(),
                    reinterpret_cast<void*>(&ejectHook), original)) {
                REL::safe_write(REL::Offset(0x71DAF2).address(), attachCall.data(), attachCall.size());
                s_attach = nullptr;
                return false;
            }
            s_eject = reinterpret_cast<Eject>(original);
            s_installed = true;
            ROCK_LOG_INFO(Init, "Shell casing grace installed: native player ejection, capture before world unlock, simulation-time expiry");
            return true;
        } catch (...) {
            if (s_attach && !s_eject) {
                REL::safe_write(REL::Offset(0x71DAF2).address(), attachCall.data(), attachCall.size());
                s_attach = nullptr;
            }
            ROCK_LOG_ERROR(Init, "Shell casing grace installation failed; native casing collisions preserved");
            return false;
        }
    }

    void publishWeapon(std::uint32_t formId, std::uint64_t generation, std::span<const std::uint32_t> ids) noexcept
    {
        std::array<std::uint32_t, MAX_WEAPON_COLLISION_BODIES> sorted{};
        const auto count = (std::min)(ids.size(), sorted.size());
        std::copy_n(ids.begin(), count, sorted.begin());
        std::sort(sorted.begin(), sorted.begin() + count);
        const auto version = s_weapon.version.load();
        s_weapon.version.store(version + 1, std::memory_order_release);
        s_weapon.formId.store(formId); s_weapon.generation.store(generation);
        s_weapon.count.store(static_cast<std::uint32_t>(count));
        for (std::size_t i = 0; i < count; ++i) s_weapon.bodies[i].store(sorted[i]);
        s_weapon.version.store(version + 2, std::memory_order_release);
    }

    void prepareFrame(RE::hknpWorld* world, float milliseconds, double seconds, std::uint64_t solve) noexcept
    {
        if (s_world.load() != world) {
            s_epoch.fetch_add(1);
            s_simulatedSeconds.store(seconds); s_completedSolve.store(solve);
            s_world.store(world);
        }
        s_milliseconds.store(sanitizeMilliseconds(milliseconds));
        const auto now = GetTickCount64();
        if (now < s_nextReportMs) return;
        s_nextReportMs = now + 5000;
        const auto registered = s_registered.exchange(0), suppressed = s_suppressed.exchange(0);
        const auto restored = s_restored.exchange(0), stale = s_stale.exchange(0), failures = s_failures.exchange(0);
        if (registered || restored || suppressed || failures) {
            ROCK_LOG_INFO(Weapon, "Shell casing grace: registered={} suppressedPairs={} restored={} stale={} failures={} deepestStage={} active={} graceMs={:.3f} lastBornBody={} lastRestoredBody={} lastRestoreMs={:.3f}",
                registered, suppressed, restored, stale, failures, failureName(s_failure.exchange(Failure::None)),
                s_active.load(), s_milliseconds.load(), s_lastBody.load(), s_lastRestoredBody.load(), s_lastRestoreMs.load());
        }
    }

    void abandon() noexcept
    {
        s_world.store(nullptr);
        s_epoch.fetch_add(1);
        // Epoch invalidation is sufficient for readers; the next pre-collide
        // reclaims old slots without touching the previous world's bodies.
    }

    void beforeCollide(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing) noexcept
    {
        if (!world || world != s_world.load() || !s_active.load()) return;
        const auto epoch = s_epoch.load();
        const bool measured = timing.valid && !timing.usedFallback;
        for (auto& slot : s_slots) {
            Record record{};
            std::uint64_t stamp{};
            if (!slot.read(record, stamp) || record.body.id == kInvalidBody) continue;
            const bool currentWorldAddress = record.world == reinterpret_cast<std::uintptr_t>(world);
            const bool sameWorld = record.epoch == epoch && currentWorldAddress;
            const bool expired = record.window.expired(timing.elapsedSimulatedSeconds, timing.solveSequence);
            const bool keep = sameWorld && measured && !expired &&
                record.weaponGeneration == s_weapon.generation.load();
            if (keep || !slot.claim(stamp)) continue;
            slot.write(Record{}, stamp);
            s_active.fetch_sub(1);
            if (!currentWorldAddress) { s_stale.fetch_add(1); continue; }
            const auto live = havok_runtime::snapshotBodyIdentity(world, RE::hknpBodyId{ record.body.id });
            if (!live.valid || !record.body.matches(identity(live))) { s_stale.fetch_add(1); continue; }
            // Pair rejection can leave a cached exclusion. Reconsider this
            // casing before collision so contacts really resume at expiry.
            // A coordinator reset can preserve the live world: an old epoch
            // still restores a matching CURRENT-world body. The saved world
            // address is only compared; it is never dereferenced.
            static REL::Relocation<Rebuild> rebuild{ REL::Offset(offsets::kFunc_RebuildBodyCollisionCaches) };
            rebuild(world, record.body.id);
            s_restored.fetch_add(1);
            s_lastRestoredBody.store(record.body.id);
            s_lastRestoreMs.store((timing.elapsedSimulatedSeconds - record.window.bornSeconds) * 1000.0);
            if (!measured) fail(Failure::Timing);
        }
    }

    void afterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing) noexcept
    {
        if (world == s_world.load() && timing.valid && !timing.usedFallback) {
            s_simulatedSeconds.store(timing.elapsedSimulatedSeconds);
            s_completedSolve.store(timing.solveSequence);
        }
    }

    int filterPairs(RE::hknpWorld* world, native_player_collision::BodyPair* pairs, int count) noexcept
    {
        if (!pairs || count <= 0 || world != s_world.load() || !s_active.load()) return count;
        const auto kept = native_player_collision::filterPhysicalPairs(pairs, count, [&](const auto& pair) {
            return suppress(world, pair.bodyA, pair.bodyB) || suppress(world, pair.bodyB, pair.bodyA);
        });
        s_suppressed.fetch_add(count - kept, std::memory_order_relaxed);
        return kept;
    }
}
