#include "physics-interaction/weapon/AkimboSessionPolicy.h"
#include "physics-interaction/weapon/WeaponCyclePolicy.h"
#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include <array>
#include <iostream>
#include <limits>

using namespace rock::akimbo;

int main()
{
    bool ok = true;
    const auto check = [&](bool value, const char* what) {
        if (!value) { std::cerr << what << '\n'; ok = false; }
    };
    using namespace rock::carried_weapon_projectile;
    constexpr Identity own{0x1000, 0x2000, 0x3000, 1};
    check(isOwnHeldWeapon(42, own, 42, own), "The carried gun cannot intercept its own projectile");
    check(!isOwnHeldWeapon(42, own, 43, own), "Another physical copy of the same gun remains hittable");
    check(!isOwnHeldWeapon(0, own, 42, own), "Releasing the gun ends its self exclusion");
    for (const Identity other : {Identity{0x1001,0x2000,0x3000,1}, Identity{0x1000,0x2001,0x3000,1},
            Identity{0x1000,0x2000,0x3001,1}, Identity{0x1000,0x2000,0x3000,0}})
        check(!isOwnHeldWeapon(42, own, 42, other), "Other weapon/instance/actor/index projectiles keep native collision");
    using namespace rock::weapon_cycle_policy;
    constexpr std::array<std::int16_t, 7> parents{-1,0,1,2,0,6,5};
    check(isWeaponPart(3, 1, parents), "Nested bolt bones belong to the weapon branch");
    for (const int bone : {0, 1, 4, 5, 6, 7, -1})
        check(!isWeaponPart(bone, 1, parents), "Root/body/malformed/cyclic bones must never receive firing transforms");
    check(fireClipPriority("Animations\\Glock19xAnims\\WPNFireSingleReady.hkx") == 3 &&
        fireClipPriority("Animations/Glock19xAnims/WPNFireSingleReady.hkt") == 3,
        "The exact subgraph's firing stroke accepts native resource extensions");
    for (const auto path : {"WPNReload.hkx", "WPNFireAutoReadyBack.hkx", "WPNFireSingleReadySlave.hkt", "WPNAfterJiggleFireSingleAdd.hkx"})
        check(!fireClipPriority(path), "Reload, blend, slave and additive arm clips cannot become a mechanical stroke");
    OperationState rifle, pistol;
    rifle.begin(1); pistol.begin(2);
    rifle.bind(Hand::Left, Grip::Support);
    pistol.bind(Hand::Right, Grip::Firing);
    check(!rifle.requestFire(true, true, true, true, 8).operation, "Support carry cannot fire its loaded rifle");
    check(!pistol.requestFire(true, true, false, true, 3).operation, "Drawing with a held trigger must require release");
    (void)pistol.requestFire(true, false, false, true, 3);
    auto shot = pistol.requestFire(true, true, false, true, 3);
    check(pistol.current(shot), "The pistol firing grip enables its own session");
    check(!rifle.current(shot), "A firing ticket must not match another weapon");
    pistol.completeFire(shot, 0.25f);

    rifle.bind(Hand::Left, Grip::Firing);
    (void)rifle.requestFire(true, false, true, true, 8);
    auto rifleShot = rifle.requestFire(true, true, true, true, 8);
    check(rifle.current(rifleShot), "Pistol cooldown must not delay the rifle");
    rifle.completeFire(rifleShot, 0.1f);
    rifle.advance(0.1f);
    check(rifle.requestFire(true, true, true, true, 7).operation != 0, "Automatic fire follows the weapon cadence");

    pistol.bind(Hand::Left, Grip::Firing);
    (void)pistol.requestFire(true, false, false, true, 2);
    check(!pistol.requestFire(true, true, false, true, 2).operation, "Hand changes must not reset cooldown");
    pistol.advance(0.25f);
    check(!pistol.requestFire(true, true, false, true, 2).operation, "Semiautomatic fire requires a fresh press");
    (void)pistol.requestFire(true, false, false, true, 2);
    auto beforeHandoff = pistol.requestFire(true, true, false, true, 2);
    pistol.bind(Hand::Right, Grip::Firing);
    check(!pistol.current(beforeHandoff), "A queued operation must not survive a changed grip binding");
    (void)pistol.requestFire(true, false, false, true, 2);
    auto afterHandoff = pistol.requestFire(true, true, false, true, 2);
    check(pistol.current(afterHandoff), "Cancelled old tickets must not permanently block the weapon");
    pistol.completeFire(afterHandoff, 0.25f);

    check(pistol.beginReload(1.0f), "A firing grip can start its weapon reload");
    pistol.advance(0.4f);
    pistol.bind(Hand::Left, Grip::Firing);
    check(!pistol.reloadDue(), "Hand changes must preserve remaining reload time");
    pistol.advance(0.6f);
    check(pistol.reloadDue(), "Reload completion belongs to the weapon");
    pistol.completeReload();
    (void)pistol.requestFire(true, false, false, true, 3);
    check(!pistol.requestFire(true, true, false, false, 3).operation, "Unknown ammo is not a loaded magazine");
    (void)pistol.requestFire(true, false, false, true, 0);
    check(!pistol.requestFire(true, true, false, true, 0).operation, "An empty weapon cannot dispatch a shot");

    check(clampReload(17, 100, 17) == 17, "Ample shared reserve must not subtract the other magazine from capacity");
    check(clampReload(17, 20, 8) == 12, "Shared reload cannot allocate rounds already loaded in the other weapon");
    check(clampReload(17, 8, 8) == 0, "Exhausted shared reserve is distinct from loaded rounds elsewhere");
    check(clampReload(17, 2, 8) == 0, "External ammo loss must not underflow the reservation");
    check(clampReload(17, std::numeric_limits<std::uint32_t>::max(), 8) == 17,
        "Large inventory totals must not overflow capacity accounting");

    OperationState first, second;
    first.begin(40); second.begin(41);
    first.bind(Hand::Left, Grip::Firing); second.bind(Hand::Right, Grip::Firing);
    (void)first.requestFire(true, false, false, true, 8);
    (void)second.requestFire(true, false, false, true, 3);
    const auto leftShot = first.requestFire(true, true, false, true, 8);
    const auto rightShot = second.requestFire(true, true, false, true, 3);
    check(first.current(leftShot) && second.current(rightShot), "Both weapons can dispatch in the same frame");
    first.completeFire(leftShot, 0.1f); second.completeFire(rightShot, 0.4f);
    check(first.beginReload(2.0f) && second.beginReload(1.0f), "Both weapons can reload independently");
    first.advance(1.0f); second.advance(1.0f);
    check(!first.reloadDue() && second.reloadDue(), "Reload duration belongs to each weapon");

    OperationState restored;
    restored.begin(42); restored.bind(Hand::Left, Grip::Firing);
    restored.restore(0.3f, 0.6f, true);
    check(restored.reloading() && restored.cooldown() == 0.3f && restored.reloadRemaining() == 0.6f,
        "Restoring a session preserves cooldown and reload progress");
    restored.bind(Hand::Right, Grip::Support);
    restored.advance(0.2f);
    check(restored.reloading() && restored.reloadRemaining() > 0.39f && restored.cooldown() > 0.09f,
        "A support grip does not reset restored operation progress");
    restored.restore(std::numeric_limits<float>::quiet_NaN(), 0.0f, false);
    check(restored.reloading(), "Invalid serialized timing cannot replace a valid operation");
    restored.advance(std::numeric_limits<float>::infinity());
    check(!restored.reloadDue(), "Invalid frame time cannot complete a reload");
    restored.advance(0.5f); restored.completeReload();
    restored.bind(Hand::Left, Grip::Firing);
    check(!restored.requestFire(true, true, false, true, 8).operation,
        "Restored or rebound firing grips still require trigger release");
    return ok ? 0 : 1;
}
