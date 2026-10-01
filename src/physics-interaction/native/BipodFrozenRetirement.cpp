#include "physics-interaction/native/BipodFrozenRetirement.h"
#include "physics-interaction/native/NativeObjectPlacement.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/grab/HeldPlacementPolicy.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpBody.h"
#include <F4SE/Trampoline.h>
#include <Windows.h>
#include <array>
#include <atomic>
#include <cstring>

namespace rock::bipod_frozen_retirement
{
    namespace
    {
        enum class Stage : std::uint8_t { Empty, Armed, Bound, Initializing, Frozen, Quarantined, Finishing };
        using Factory = std::uint32_t* (*)(void*, std::uint32_t*, void*);
        using Position = void (*)(RE::TESObjectREFR*, const RE::NiPoint3&);
        using Initialize = void (*)(RE::TESObjectREFR*);
        using Detach = bool (*)(RE::TESObjectREFR*, RE::NiAVObject*);
        using Lock = void (*)(RE::hknpWorld*);
        Factory s_factory{};
        Position s_position{};
        Initialize s_initialize{};
        Detach s_detach{};
        Lock s_lock{}, s_unlock{};
        bool s_installed{};
        bool s_attempted{};

        // Immutable from Armed publication until the terminal initializer has
        // returned. Stage release/acquire publishes the copied values. Only
        // the game thread may recycle the slot; CAS serializes initializers.
        std::atomic<Stage> s_stage{ Stage::Empty };
        std::atomic<std::uint32_t> s_reference{};
        std::atomic<bool> s_cancelled{};
        std::uint32_t s_weapon{};
        std::uintptr_t s_world{};
        RE::NiTransform s_pose{};
        bool s_scriptStarted{}, s_recoveryAttempted{}; // Game thread only.
        thread_local bool s_removing{};
        thread_local bool s_creating{};

        bool freeze(RE::TESObjectREFR* ref, RE::bhkWorld* bhk, RE::hknpWorld* world)
        {
            using namespace object_physics_body_set;
            BodySetScanOptions options{};
            options.requireSameResolvedRef = true;
            ObjectPhysicsBodyScanCursor cursor;
            ObjectPhysicsBodyScanCache cache;
            if (!beginObjectPhysicsBodyScanCache(ref, options, cursor, cache)) return false;
            const auto step = advanceObjectPhysicsBodyScanCache(world, options,
                { 256, 64, held_placement_policy::kMaxBodies }, cursor, cache);
            const auto bodies = buildObjectPhysicsBodySetFromScanCache(bhk, world, ref, options, cache);
            const auto& d = bodies.diagnostics;
            const auto issues = d.scanFailures + d.invalidPhysicsSystems + d.benignScanSkips +
                d.foreignRefBodySkips + d.unresolvedRefBodySkips + d.weaponExpansionSkips +
                d.depthLimitSkips + d.staleCacheEntrySkips;
            std::array<std::uint32_t, held_placement_policy::kMaxBodies> ids{};
            std::size_t count = 0;
            for (const auto& body : bodies.records) {
                const bool preservedStatic = held_placement_policy::preserveStaticBody(body.rejectReason,
                    physics_body_classifier::motionTypeFromBodyFlags(body.bodyFlags), body.motionId,
                    body.refResolutionKnown && body.resolvedRef == ref);
                const bool alreadyFrozen = body.rejectReason == physics_body_classifier::BodyRejectReason::NotDynamicAfterActivePrep &&
                    physics_body_classifier::motionTypeFromBodyFlags(body.bodyFlags) == physics_body_classifier::BodyMotionType::Keyframed &&
                    body.motionId != 0 && body.refResolutionKnown && body.resolvedRef == ref;
                if ((!body.accepted && !preservedStatic && !alreadyFrozen) || count == ids.size()) return false;
                ids[count++] = body.bodyId;
            }
            if (!held_placement_policy::completeBodyScan(step.finished && !step.invalidated,
                    bodies.records.size(), count, issues)) return false;
            auto* root = ref->Get3D();
            if (!root || !dynamic_weapon_collision_policy::isFiniteTransform(root->world) ||
                std::abs(root->world.scale) < 0.0001f) return false;
            const auto rootInverse = transform_math::invertTransform(root->world);
            std::array<std::uint32_t, held_placement_policy::kMaxBodies> motions{};
            std::size_t motionCount = 0;
            for (const auto& body : bodies.records) {
                if (!body.motionId || std::find(motions.begin(), motions.begin() + motionCount, body.motionId) != motions.begin() + motionCount) continue;
                RE::NiTransform current{};
                if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{body.bodyId}, current)) return false;
                const auto target = transform_math::composeTransforms(s_pose,
                    transform_math::composeTransforms(rootInverse, current));
                if (!dynamic_weapon_collision_policy::isFiniteTransform(target) ||
                    !havok_runtime::setBodyTransformDeferred(world, body.bodyId, target, 1)) return false;
                motions[motionCount++] = body.motionId;
            }
            return native_object_placement::anchorRetiredWeapon(ref, world, s_pose, std::span(ids).first(count));
        }

        void initializeHook(RE::TESObjectREFR* ref)
        {
            if (s_cancelled.load(std::memory_order_acquire) || !ref ||
                ref->GetFormID() != s_reference.load(std::memory_order_acquire)) {
                s_initialize(ref);
                return;
            }
            auto stage = s_stage.load(std::memory_order_acquire);
            if (stage == Stage::Initializing || stage == Stage::Quarantined || stage == Stage::Finishing) return;
            if ((stage != Stage::Bound && stage != Stage::Frozen) ||
                !s_stage.compare_exchange_strong(stage, Stage::Initializing, std::memory_order_acq_rel)) {
                s_initialize(ref);
                return;
            }
            if (ref->GetFormID() != s_reference.load(std::memory_order_acquire)) {
                s_stage.store(stage, std::memory_order_release);
                s_initialize(ref);
                return;
            }
            auto* cell = ref->GetParentCell();
            auto* bhk = cell ? cell->GetbhkWorld() : nullptr;
            auto* world = bhk ? havok_runtime::getHknpWorldFromBhk(bhk) : nullptr;
            if (!world || reinterpret_cast<std::uintptr_t>(bhk) != s_world) {
                // Do not attach a pending item into an unqualified world.
                s_stage.store(Stage::Quarantined, std::memory_order_release);
                return;
            }
            // The native writer is recursive for its owning thread
            // (141B93345..141B93357); native attachment already nests it.
            // Physics-step callers already own the world and skip locking.
            const bool locked = !havok_world_lock::detail::currentThreadInsidePhysicsStep();
            if (locked) s_lock(world);
            bool frozen = false;
            try {
                s_initialize(ref);
                frozen = ref->Get3D() && freeze(ref, bhk, world);
            } catch (...) {
                ROCK_LOG_ERROR(Weapon, "Bipod frozen retirement failed during native initialization ref={:08X}", ref->GetFormID());
            }
            if (!frozen && ref->Get3D()) {
                // Failure after RemoveItem cannot restore equipped ownership.
                // Remove the new Havok representation BEFORE releasing the
                // writer. The game thread recovers this exact item to inventory.
                (void)s_detach(ref, ref->Get3D());
            }
            if (locked) s_unlock(world);
            ROCK_LOG_INFO(Weapon, "Bipod native retirement ref={:08X} frozen={} before-first-unlocked-solve=true", ref->GetFormID(), frozen);
            if (s_cancelled.exchange(false, std::memory_order_acq_rel)) {
                s_reference.store(0, std::memory_order_release);
                s_stage.store(Stage::Empty, std::memory_order_release);
            } else {
                s_stage.store(frozen ? Stage::Frozen : Stage::Quarantined, std::memory_order_release);
            }
        }

        void positionHook(RE::TESObjectREFR* ref, const RE::NiPoint3& position)
        {
            // 14011C23D is in the exact drop factory, before its cell/3D
            // publication. The factory scope excludes unrelated position writes.
            if (s_creating && s_stage.load(std::memory_order_acquire) == Stage::Armed &&
                ref && ref->GetObjectReference() && ref->GetObjectReference()->GetFormID() == s_weapon) {
                s_reference.store(ref->GetFormID(), std::memory_order_release);
                s_stage.store(Stage::Bound, std::memory_order_release);
            }
            s_position(ref, position);
        }

        std::uint32_t* factoryHook(void* factory, std::uint32_t* handle, void* data)
        {
            const auto previous = s_creating;
            s_creating = s_removing && s_stage.load(std::memory_order_acquire) == Stage::Armed;
            const auto result = s_factory(factory, handle, data);
            s_creating = previous;
            return result;
        }

        template<std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == expected;
        }
        bool callMatches(std::uintptr_t rva, std::uintptr_t target)
        {
            std::array<std::uint8_t, 5> bytes{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<void*>(REL::Offset(rva).address()), bytes.data(), bytes.size()) || bytes[0] != 0xE8) return false;
            std::int32_t displacement{};
            std::memcpy(&displacement, bytes.data() + 1, sizeof(displacement));
            return static_cast<std::intptr_t>(rva + 5) + displacement == static_cast<std::intptr_t>(target);
        }
    }

    bool install()
    {
        if (s_attempted) return s_installed;
        s_attempted = true;
        // These three call paths were checked in raw FO4VR disassembly:
        // drop factory 3E2AC1 -> 11BD80, factory pose 11C23D -> 3F4370,
        // and reference initialization 3EA980 (locked attachment at 3EAB75).
        constexpr std::array<std::uint8_t, 9> prefix{ 0x40,0x53,0x48,0x81,0xEC,0xD0,0,0,0 };
        if (!REL::Module::IsVR() || !native_object_placement::available() ||
            !callMatches(0x3E2AC1, 0x11BD80) || !callMatches(0x11C23D, 0x3F4370) ||
            !matches(0x3EA980, prefix) ||
            !matches(0x3EA820, std::array<std::uint8_t, 9>{0x40,0x55,0x48,0x83,0xEC,0x20,0x48,0x8B,0x01})) {
            ROCK_LOG_ERROR(Init, "Bipod frozen retirement unavailable: native creation/initialization qualification failed");
            return false;
        }
        try {
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < 128) {
                ROCK_LOG_ERROR(Init, "Bipod frozen retirement unavailable: trampoline capacity");
                return false;
            }
            // Nine complete position-independent prologue bytes; the relative call
            // beginning at +12 is deliberately left in its original location.
            auto* original = static_cast<std::uint8_t*>(trampoline.allocate(prefix.size() + 14));
            std::memcpy(original, prefix.data(), prefix.size());
            const std::array<std::uint8_t, 6> jump{0xFF,0x25,0,0,0,0};
            std::memcpy(original + prefix.size(), jump.data(), jump.size());
            const auto resume = REL::Offset(0x3EA980).address() + prefix.size();
            std::memcpy(original + prefix.size() + jump.size(), &resume, sizeof(resume));
            s_initialize = reinterpret_cast<Initialize>(original);
            s_detach = reinterpret_cast<Detach>(REL::Offset(0x3EA820).address());
            s_lock = reinterpret_cast<Lock>(REL::Offset(0x1DF5FB0).address());
            s_unlock = reinterpret_cast<Lock>(REL::Offset(0x1DF5FD0).address());
            s_factory = reinterpret_cast<Factory>(REL::Offset(0x11BD80).address());
            s_position = reinterpret_cast<Position>(REL::Offset(0x3F4370).address());
            (void)trampoline.write_call<5>(REL::Offset(0x3E2AC1).address(), &factoryHook);
            (void)trampoline.write_call<5>(REL::Offset(0x11C23D).address(), &positionHook);
            FlushInstructionCache(GetCurrentProcess(), original, prefix.size() + 14);
            (void)trampoline.write_branch<5>(REL::Offset(0x3EA980).address(), &initializeHook);
            s_installed = true;
            ROCK_LOG_INFO(Init, "Bipod frozen retirement native creation guard installed");
            return true;
        } catch (...) {
            // Any installed observer remains a passive original-call forwarder;
            // arming stays disabled unless all three hooks were installed.
            ROCK_LOG_ERROR(Init, "Bipod frozen retirement hook installation failed");
            return false;
        }
    }

    bool available() { return s_installed; }
    bool pending() { return s_stage.load(std::memory_order_acquire) != Stage::Empty; }
    bool frozen(const std::uint32_t reference) {
        return reference && s_stage.load(std::memory_order_acquire) == Stage::Frozen &&
            s_reference.load(std::memory_order_acquire) == reference;
    }
    void cancelForWorldLoss()
    {
        s_cancelled.store(true, std::memory_order_release);
        auto stage = s_stage.load(std::memory_order_acquire);
        if (stage == Stage::Initializing) return;
        if (!s_stage.compare_exchange_strong(stage, Stage::Finishing, std::memory_order_acq_rel)) return;
        s_reference.store(0, std::memory_order_release);
        s_cancelled.store(false, std::memory_order_release);
        s_stage.store(Stage::Empty, std::memory_order_release);
    }

    bool arm(std::uint32_t weapon, RE::bhkWorld* world, const RE::NiTransform& pose)
    {
        if (!available() || pending() || !weapon || !world ||
            !dynamic_weapon_collision_policy::isFiniteTransform(pose)) return false;
        s_weapon = weapon;
        s_world = reinterpret_cast<std::uintptr_t>(world);
        s_pose = pose;
        s_scriptStarted = false;
        s_recoveryAttempted = false;
        s_cancelled.store(false, std::memory_order_relaxed);
        s_reference.store(0, std::memory_order_relaxed);
        s_stage.store(Stage::Armed, std::memory_order_release);
        s_removing = true;
        return true;
    }

    void finishRemoval(bool committed)
    {
        s_removing = false;
        if (!committed && s_stage.load(std::memory_order_acquire) == Stage::Armed)
            s_stage.store(Stage::Empty, std::memory_order_release);
        else if (committed && !s_reference.load(std::memory_order_acquire))
            ROCK_LOG_CRITICAL(Weapon, "Bipod native removal committed without the qualified reference publication");
    }

    void service()
    {
        if (s_cancelled.load(std::memory_order_acquire)) cancelForWorldLoss();
        auto stage = s_stage.load(std::memory_order_acquire);
        if (stage != Stage::Frozen && stage != Stage::Quarantined) return;
        if (!s_stage.compare_exchange_strong(stage, Stage::Finishing, std::memory_order_acq_rel)) return;
        const auto form = s_reference.load(std::memory_order_acquire);
        auto* ref = RE::TESForm::GetFormByID<RE::TESObjectREFR>(form);
        if (!ref || ref->IsDeleted()) {
            s_reference.store(0, std::memory_order_release);
            s_stage.store(Stage::Empty, std::memory_order_release);
            return;
        }
        if (stage == Stage::Quarantined) {
            if (s_recoveryAttempted) { s_stage.store(stage, std::memory_order_release); return; }
            s_recoveryAttempted = true;
            auto* player = RE::PlayerCharacter::GetSingleton();
            const bool recovered = player && ref->ActivateRef(player, nullptr, 1, false, false, false);
            ROCK_LOG_ERROR(Weapon, "Bipod retirement rejected ref={:08X} exact-item-inventory-recovery={} physics=guarded", form, recovered);
            f4vr::showNotification(recovered ? "ROCK: Bipod placement failed; weapon returned to inventory." :
                "ROCK: Bipod placement failed; the weapon remains at its placement position.");
            if (!recovered) { s_stage.store(stage, std::memory_order_release); return; }
        } else {
            const auto script = native_object_placement::prepareLoadScript(ref, !s_scriptStarted);
            s_scriptStarted = true;
            if (script != native_object_placement::ScriptPreparation::Ready) {
                s_stage.store(script == native_object_placement::ScriptPreparation::Rejected ? Stage::Quarantined : stage,
                    std::memory_order_release);
                return;
            }
            ROCK_LOG_INFO(Weapon, "Bipod frozen retirement completed ref={:08X} save=native", form);
        }
        s_reference.store(0, std::memory_order_release);
        s_stage.store(Stage::Empty, std::memory_order_release);
    }
}
