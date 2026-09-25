#include "physics-interaction/weapon/AkimboSessionPolicy.h"
#include "physics-interaction/weapon/WeaponCyclePolicy.h"
#include "physics-interaction/weapon/EquippedWeaponPairPolicy.h"
#include "physics-interaction/weapon/PhysicalWeaponShotPolicy.h"
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
    check(rock::weapon_cycle_policy::playbackRate(0.5f, 0.1f) == 5.0f &&
        rock::weapon_cycle_policy::playbackRate(0.1f, 0.5f) == 1.0f,
        "Automatic mechanics finish a stroke before the next shot without slowing shorter authored strokes");
    struct Point { float x{},y{},z{}; };
    struct Matrix { float entry[3][3]{}; };
    struct Transform { Matrix rotate{}; Point translate{}; float scale{1}; };
    Transform muzzle{{{{0,1,0},{-1,0,0},{0,0,1}}},{7,8,9},1};
    auto aim = rock::physical_weapon_shot_policy::muzzleAim(muzzle);
    check(aim.ray.valid && std::fabs(aim.ray.direction.x+1)<0.0001f && std::fabs(aim.ray.direction.y)<0.0001f,
        "A quarter-turn muzzle points along its stored +Y row, not the inverse column");
    auto launched = rock::native_scope_shot_policy::launchRay(aim.ray.origin, aim.yaw, aim.pitch);
    check(rock::native_scope_shot_policy::angleDegrees(aim.ray, launched)<0.001f && launched.origin.x==7,
        "Native yaw/pitch reproduce the visible muzzle direction and position");
    muzzle.rotate = {{{1,0,0},{0,0,1},{0,-1,0}}};
    aim = rock::physical_weapon_shot_policy::muzzleAim(muzzle);
    launched = rock::native_scope_shot_policy::launchRay(aim.ray.origin, aim.yaw, aim.pitch);
    check(aim.ray.direction.z>0.999f && launched.direction.z>0.999f, "An upward barrel must not fire downward");
    muzzle.scale = 0;
    check(!rock::physical_weapon_shot_policy::muzzleAim(muzzle).ray.valid, "Collapsed muzzle frames cannot supply a shot direction");
    namespace entry = rock::equipped_weapon_pair_policy;
    const auto nativeOwners = entry::collisionOwners({100,3}, {});
    check(nativeOwners[0].body == 100 && nativeOwners[1].body == 100 && nativeOwners[0].hands && nativeOwners[1].hands,
        "Two grips on one native weapon preserve their existing collision ownership");
    const auto dualOwners = entry::collisionOwners({}, {{{200,1},{300,2}}});
    check(dualOwners[0].body == 200 && dualOwners[1].body == 300 && dualOwners[0].hands && dualOwners[1].hands,
        "Both physical gun owners suppress their own hands with independent body identities");
    const auto transitionOwners = entry::collisionOwners({100,3}, {{{200,1},{0x7FFFFFFFu,2}}});
    check(transitionOwners[0].body == 200 && transitionOwners[1].body == 0x7FFFFFFFu && transitionOwners[1].hands,
        "A preparing physical owner suppresses its hand without borrowing the old native weapon body");
    const auto handedOff = entry::collisionOwners({}, {{{200,2},{}}});
    check(!handedOff[0].hands && handedOff[1].body == 200 && handedOff[1].hands,
        "Handoff releases the former hand collision claim and preserves the weapon body on its new hand");
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
