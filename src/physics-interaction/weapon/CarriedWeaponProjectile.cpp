#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <array>
#include <atomic>

namespace rock::carried_weapon_projectile
{
    namespace
    {
        struct Owner
        {
            std::uintptr_t weapon{}, instance{};
            std::uint32_t reference{}, shooter{}, index{};
        };
        struct Publication
        {
            std::atomic<std::uint64_t> sequence{};
            std::atomic<std::uintptr_t> weapon{}, instance{};
            std::atomic<std::uint32_t> reference{}, shooter{}, index{};
            bool read(Owner& out) const noexcept
            {
                const auto before = sequence.load(std::memory_order_acquire);
                if (before & 1) return false;
                out = {weapon.load(), instance.load(), reference.load(), shooter.load(), index.load()};
                return out.reference && sequence.load(std::memory_order_acquire) == before;
            }
        } owner;
        using AdmitHit = bool (*)(void*, std::uint32_t, void*, std::uint32_t);
        using AddImpact = std::uint32_t (*)(void*, const void*);
        AdmitHit originalAdmit{};
        AddImpact originalImpact{};
        bool installed{};
        std::atomic<std::uint32_t> candidateLogs{}, impactLogs{};

        bool ownedProjectile(const void* projectile, Owner& snapshot, Identity& identity) noexcept
        {
            if (!projectile || !owner.read(snapshot)) return false;
            // 1057B00 copies these fields from launch data; 104E020 initializes
            // them and 10585A0 independently reads the retained weapon pair.
            return native_memory::tryReadField(projectile, 0x1A8, identity.shooter) &&
                native_memory::tryReadField(projectile, 0x238, identity.weapon) &&
                native_memory::tryReadField(projectile, 0x240, identity.instance) &&
                native_memory::tryReadField(projectile, 0x250, identity.index) &&
                identity == Identity{snapshot.weapon, snapshot.instance, snapshot.shooter, snapshot.index};
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
            const bool self = isOwnHeldWeapon(snapshot.reference, identity, targetForm, projectileIdentity);
            if (candidateLogs.fetch_add(1, std::memory_order_relaxed) < 32) {
                try {
                    ROCK_LOG_INFO(Weapon, "Akimbo projectile candidate projectile={:08X} target={:08X} body={} layer={} nativeAccepted={} ownWeapon={} accepted={}",
                        projectileForm, targetForm, body, filter & 0x7F, admitted, self, admitted && !self);
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
                impactLogs.fetch_add(1, std::memory_order_relaxed) >= 32) return result;
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
                ROCK_LOG_INFO(Weapon, "Akimbo projectile impact projectile={:08X} target={:08X} body={} shape={:08X} layer={} impact={} position=({:.3f},{:.3f},{:.3f})",
                    projectileForm, targetForm, body, shape, filter & 0x7F, result, position[0], position[1], position[2]);
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
        if (!entry_trampoline_hook::install("carried-weapon projectile impact", 0x10585A0,
                impactBytes.data(), impactBytes.size(), reinterpret_cast<void*>(&addImpact), original)) return false;
        originalImpact = reinterpret_cast<AddImpact>(original);
        installed = true;
        return true;
    }
    catch (...) { return false; }

    void publish(std::uint32_t reference, std::uint32_t shooter, std::uintptr_t weapon,
        std::uintptr_t instance, std::uint32_t index) noexcept
    {
        // Single frame-thread publisher; readers use only atomics and never wait.
        owner.sequence.fetch_add(1, std::memory_order_acq_rel);
        owner.reference.store(reference); owner.shooter.store(shooter);
        owner.weapon.store(weapon); owner.instance.store(instance); owner.index.store(index);
        owner.sequence.fetch_add(1, std::memory_order_release);
        candidateLogs.store(0); impactLogs.store(0);
    }

    void clear() noexcept { publish(0, 0, 0, 0, 0); }
}
