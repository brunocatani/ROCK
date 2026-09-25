#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/weapon/NativeCarriedWeaponContext.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiAVObject.h"

#include <array>
#include <atomic>

namespace rock::carried_weapon_projectile
{
    namespace
    {
        struct Owner
        {
            std::uintptr_t weapon{}, instance{};
            std::uint32_t reference{}, shooter{}, index{}, bodyCount{};
            std::array<std::uint32_t, 257> bodies{};
        };
        struct Publication
        {
            std::atomic<std::uint64_t> sequence{};
            std::atomic<std::uintptr_t> weapon{}, instance{};
            std::atomic<std::uint32_t> reference{}, shooter{}, index{}, bodyCount{};
            std::array<std::atomic<std::uint32_t>, 257> bodies{};
            bool read(Owner& out) const noexcept
            {
                const auto before = sequence.load(std::memory_order_acquire);
                if (before & 1) return false;
                out = {weapon.load(), instance.load(), reference.load(), shooter.load(), index.load()};
                out.bodyCount = (std::min)(bodyCount.load(), static_cast<std::uint32_t>(out.bodies.size()));
                for (unsigned i = 0; i < out.bodyCount; ++i) out.bodies[i] = bodies[i].load();
                return out.reference && sequence.load(std::memory_order_acquire) == before;
            }
        };
        std::array<Publication, 2> owners{};
        struct ProjectileContext
        {
            RE::AIProcess* process{};
            RE::EquippedItem item{RE::BGSObjectInstance(nullptr, nullptr)};
            RE::NiPointer<RE::TESObjectREFR> reference{};
            RE::NiPointer<RE::NiAVObject> muzzle{};
        };
        PhysicsCallbackQuiescenceGate contextGate;
        std::array<ProjectileContext, 2> contexts;
        using Initialize = void (*)(void*);
        Initialize originalInitialize{};
        std::array<std::atomic<unsigned>, 2> initializeLogs{};
        using AdmitHit = bool (*)(void*, std::uint32_t, void*, std::uint32_t);
        using AddImpact = std::uint32_t (*)(void*, const void*);
        AdmitHit originalAdmit{};
        AddImpact originalImpact{};
        bool installed{};
        std::array<std::atomic<std::uint32_t>, 2> candidateLogs{}, impactLogs{};

        bool ownedProjectile(const void* projectile, Owner& snapshot, Identity& identity) noexcept
        {
            if (!projectile) return false;
            // 1057B00 copies these fields from launch data; 104E020 initializes
            // them and 10585A0 independently reads the retained weapon pair.
            return native_memory::tryReadField(projectile, 0x1A8, identity.shooter) &&
                native_memory::tryReadField(projectile, 0x238, identity.weapon) &&
                native_memory::tryReadField(projectile, 0x240, identity.instance) &&
                native_memory::tryReadField(projectile, 0x250, identity.index) && identity.index < owners.size() &&
                owners[identity.index].read(snapshot) && identity == Identity{snapshot.weapon, snapshot.instance, snapshot.shooter, snapshot.index};
        }

        void initializeProjectile(void* projectile)
        {
            Owner owner{};
            Identity identity{};
            ProjectileContext context;
            if (ownedProjectile(projectile, owner, identity)) {
                // Do not hold the publication gate across native work or its
                // callbacks. Each copied record pins the data, exact item and
                // muzzle through initialization, including concurrent withdrawal.
                auto lease = contextGate.tryEnterCallback();
                if (lease) {
                    const auto& current = contexts[identity.index];
                    if (current.reference && current.reference->formID == owner.reference &&
                        reinterpret_cast<std::uintptr_t>(current.item.item.object) == identity.weapon &&
                        reinterpret_cast<std::uintptr_t>(current.item.item.instanceData.get()) == identity.instance &&
                        current.item.equipIndex.index == identity.index && current.muzzle) context = current;
                }
            }
            if (!context.reference || !context.process || !context.item.data) {
                originalInitialize(projectile);
                return;
            }
            // DB0E00 dispatches 1057700 after launch. Its 105A6C0 -> 104C1D0
            // -> ECC570 path resolves the indexed muzzle/effect data again.
            native_carried_weapon_context::Scope scope(context.process, context.item);
            originalInitialize(projectile);
            if (initializeLogs[identity.index].fetch_add(1, std::memory_order_relaxed) < 4) {
                const auto* data = static_cast<const RE::EquippedWeaponData*>(context.item.data.get());
                try { ROCK_LOG_DEBUG(Weapon, "AKIMBO_PROJECTILE_INITIALIZED slot={} ref={:08X} privateContext=true muzzleFlash={:p} fireNode={:p}",
                    identity.index, owner.reference, static_cast<void*>(data->muzzleFlash), static_cast<void*>(data->fireNode)); } catch (...) {}
            }
        }

        bool admitHit(void* target, std::uint32_t filter, void* projectile, std::uint32_t body)
        {
            const bool admitted = originalAdmit(target, filter, projectile, body);
            Owner snapshot{};
            Identity projectileIdentity{};
            if (!ownedProjectile(projectile, snapshot, projectileIdentity)) return admitted;
            std::uint32_t targetForm{}, projectileForm{};
            if (target) (void)native_memory::tryReadField(target, 0x14, targetForm);
            (void)native_memory::tryReadField(projectile, 0x14, projectileForm);
            const Identity identity{snapshot.weapon, snapshot.instance, snapshot.shooter, snapshot.index};
            const bool ownBody = body != UINT32_MAX && body != 0x7FFFFFFF &&
                std::find(snapshot.bodies.begin(), snapshot.bodies.begin() + snapshot.bodyCount, body) != snapshot.bodies.begin() + snapshot.bodyCount;
            const bool self = isOwnHeldWeapon(snapshot.reference, identity, targetForm, projectileIdentity) || ownBody;
            if (candidateLogs[projectileIdentity.index].fetch_add(1, std::memory_order_relaxed) < 32) {
                try {
                    ROCK_LOG_INFO(Weapon, "Akimbo projectile candidate slot={} ref={:08X} projectile={:08X} target={:08X} body={} layer={} nativeAccepted={} ownWeapon={} accepted={}",
                        projectileIdentity.index, snapshot.reference, projectileForm, targetForm, body, filter & 0x7F, admitted, self, admitted && !self);
                } catch (...) {}
            }
            // Match the exact held reference, not a form family or layer.
            // Its physical contacts and every other projectile stay native.
            return admitted && !self;
        }

        std::uint32_t addImpact(void* projectile, const void* input)
        {
            const auto result = originalImpact(projectile, input);
            Owner snapshot{};
            Identity projectileIdentity{};
            if (result == UINT32_MAX || !input || !ownedProjectile(projectile, snapshot, projectileIdentity) ||
                impactLogs[projectileIdentity.index].fetch_add(1, std::memory_order_relaxed) >= 32) return result;
            void* target{};
            std::uint32_t targetForm{}, projectileForm{}, body{}, shape{}, filter{};
            std::array<float, 3> position{};
            if (native_memory::tryReadField(input, 0x20, target) && target)
                (void)native_memory::tryReadField(target, 0x14, targetForm);
            (void)native_memory::tryReadField(projectile, 0x14, projectileForm);
            (void)native_memory::tryReadField(input, 0x28, filter);
            (void)native_memory::tryReadField(input, 0x2C, body);
            (void)native_memory::tryReadField(input, 0x30, shape);
            (void)native_memory::guardedCopyFromMemory(input, position.data(), sizeof(position));
            try {
                ROCK_LOG_INFO(Weapon, "Akimbo projectile impact slot={} ref={:08X} projectile={:08X} target={:08X} body={} shape={:08X} layer={} impact={} position=({:.3f},{:.3f},{:.3f})",
                    projectileIdentity.index, snapshot.reference, projectileForm, targetForm, body, shape, filter & 0x7F, result, position[0], position[1], position[2]);
            } catch (...) {}
            return result;
        }
    }

    bool install() noexcept try
    {
        if (installed) return true;
        // The signatures are independently established by 1048B20 and
        // 1054170/1058A80; both hooks preserve unknown/native candidates.
        constexpr std::array<std::uint8_t, 17> hitBytes{
            0x44,0x89,0x4C,0x24,0x20,0x89,0x54,0x24,0x10,0x53,0x55,0x56,0x57,0x41,0x55,0x41,0x56};
        constexpr std::array<std::uint8_t, 17> impactBytes{
            0x40,0x53,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57,0x48,0x83,0xEC,0x78};
        void* original{};
        if (!originalAdmit) {
            if (!entry_trampoline_hook::install("carried-weapon projectile ownership", 0x10582F0,
                    hitBytes.data(), hitBytes.size(), reinterpret_cast<void*>(&admitHit), original)) return false;
            originalAdmit = reinterpret_cast<AdmitHit>(original);
        }
        if (!originalImpact) {
            if (!entry_trampoline_hook::install("carried-weapon projectile impact", 0x10585A0,
                    impactBytes.data(), impactBytes.size(), reinterpret_cast<void*>(&addImpact), original)) return false;
            originalImpact = reinterpret_cast<AddImpact>(original);
        }
        constexpr std::array<std::uint8_t, 15> initializeBytes{
            0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18};
        if (!entry_trampoline_hook::install("carried-weapon projectile initialization", 0x1057700,
                initializeBytes.data(), initializeBytes.size(), reinterpret_cast<void*>(&initializeProjectile), original)) return false;
        originalInitialize = reinterpret_cast<Initialize>(original);
        installed = true;
        return true;
    }
    catch (...) { return false; }

    void publish(std::uint32_t reference, std::uint32_t shooter, std::uintptr_t weapon,
        std::uintptr_t instance, std::uint32_t index) noexcept
    {
        if (index >= owners.size()) return;
        auto& owner = owners[index];
        // Single frame-thread publisher; readers use only atomics and never wait.
        owner.sequence.fetch_add(1, std::memory_order_acq_rel);
        owner.reference.store(reference); owner.shooter.store(shooter);
        owner.weapon.store(weapon); owner.instance.store(instance); owner.index.store(index);
        owner.sequence.fetch_add(1, std::memory_order_release);
        candidateLogs[index].store(0); impactLogs[index].store(0);
        initializeLogs[index].store(0);
    }

    void publishContext(std::uint32_t slot, RE::AIProcess* process, const RE::EquippedItem& item,
        RE::TESObjectREFR* reference, RE::NiAVObject* muzzle)
    {
        if (slot >= contexts.size()) return;
        auto& current = contexts[slot];
        // Sole frame-thread writer. Avoid refcount churn when the pinned scene
        // and data are unchanged. Readers copy under their nonblocking gate.
        if (current.process == process && current.item.data == item.data &&
            current.reference.get() == reference && current.muzzle.get() == muzzle) return;
        {
            auto mutation = contextGate.pauseForMutation();
            current.process = process;
            current.item = item;
            current.reference.reset(reference);
            current.muzzle.reset(muzzle);
        }
        contextGate.resumeCallbacks();
    }

    void publishBodies(std::uint32_t slot, std::span<const std::uint32_t> bodies) noexcept
    {
        if (slot >= owners.size()) return;
        auto& owner = owners[slot];
        owner.sequence.fetch_add(1, std::memory_order_acq_rel);
        const auto count = (std::min)(bodies.size(), owner.bodies.size());
        for (std::size_t i = 0; i < count; ++i) owner.bodies[i].store(bodies[i]);
        owner.bodyCount.store(static_cast<std::uint32_t>(count));
        owner.sequence.fetch_add(1, std::memory_order_release);
    }
    void clear(std::uint32_t slot) noexcept
    {
        if (slot >= contexts.size()) return;
        publish(0, 0, 0, 0, slot);
        publishBodies(slot, {});
        withdrawContext(slot);
    }
    void withdrawContext(std::uint32_t slot) noexcept
    {
        if (slot >= contexts.size()) return;
        auto mutation = contextGate.pauseForMutation();
        contexts[slot] = ProjectileContext{};
    }
}
