#include "physics-interaction/weapon/AkimboSessionPolicy.h"
#include "physics-interaction/weapon/WeaponCyclePolicy.h"
#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include "physics-interaction/weapon/PhysicalWeaponPairPolicy.h"
#include "physics-interaction/weapon/PhysicalWeaponGripPolicy.h"
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
    check(transferCapture(true, true, false, false, false) == TransferCapture::NotApplicable,
        "Ammo-less native items use ordinary equip instead of consuming the request");
    check(transferCapture(true, false, false, false, false) == TransferCapture::NotApplicable,
        "Native melee replacement does not require a firearm magazine");
    check(transferCapture(false, false, false, false, false) == TransferCapture::Unavailable &&
        transferCapture(true, true, true, false, true) == TransferCapture::Unavailable,
        "An unreadable or temporarily unready firearm remains owned during admission");
    check(transferCapture(true, true, true, true, true) == TransferCapture::Ready,
        "Two eligible firearms retain magazine capture");
    check(keepFiringSound(true, Grip::Firing, true, true, true, 10, false), "Automatic burst retains its own firing sound");
    for (unsigned cause = 0; cause < 7; ++cause) check(!keepFiringSound(cause != 0,
        cause == 1 ? Grip::Support : Grip::Firing, cause != 2, cause != 3, cause != 4, cause == 5 ? 0 : 10, cause == 6),
        "Detach, support carry, blocked input, release, unknown/empty ammo and reload end the firing loop");
    check(rock::weapon_cycle_policy::playbackRate(0.5f, 0.1f) == 5.0f &&
        rock::weapon_cycle_policy::playbackRate(0.1f, 0.5f) == 1.0f,
        "Automatic mechanics finish a stroke before the next shot without slowing shorter authored strokes");
    check(restoreArchiveFlags(1, 0) == 2u && restoreArchiveFlags(1, 1) == 3u, "Old co-saves restore the formerly always-active weapon state");
    for (const bool active : {false,true}) for (const bool reloading : {false,true}) {
        const auto flags = archiveFlags(reloading, active);
        check(restoreArchiveFlags(kArchiveVersion, flags) == flags, "Retained/active and reload state survive the new private co-save format");
    }
    check(!restoreArchiveFlags(1, 2) && !restoreArchiveFlags(2, 4) && !restoreArchiveFlags(3, 0), "Unknown co-save flags and versions are rejected");
    namespace grips = rock::physical_weapon_grip_policy;
    for (const auto mode : {grips::GrabMode::ToggleBoth, grips::GrabMode::ToggleFiringOnly, grips::GrabMode::HoldBoth}) {
        for (const bool firing : {false, true}) {
            std::array<grips::Grip, 2> hands{};
            for (auto& hand : hands) {
                const bool toggle = rock::equipped_weapon_toggle_grab_policy::usesToggleForRole(mode, firing);
                check(!hand.observe(1, true, mode, firing, {true,true,false}, true, false), "Acquisition press cannot release either physical gun");
                check(hand.observe(1, true, mode, firing, {false,false,true}, true, false) == !toggle,
                    "Both physical guns honor firing/support toggle and hold modes");
                if (toggle) check(hand.observe(1, true, mode, firing, {true,true,false}, true, false), "A fresh toggle press releases the existing grip");
            }
            for (const auto drop : {grips::DropMode::Off, grips::DropMode::ToggleDrop, grips::DropMode::AutoDrop}) {
                const auto expected = drop == grips::DropMode::Off ? grips::Release::KeepAttached :
                    drop == grips::DropMode::ToggleDrop ? grips::Release::RetainLoose : grips::Release::Drop;
                check(grips::releaseAction(true, false, drop) == expected, "Last-grip release follows the configured drop mode");
                check(grips::releaseAction(true, true, drop) == grips::Release::DetachHand, "A support/firing peer keeps the gun when one grip detaches");
                check(grips::releaseAction(false, true, drop) == grips::Release::KeepAttached, "Disabled firing-hand detachment retains that grip");
            }
        }
    }
    grips::Grip retained;
    retained.retainLoose(2);
    check(!retained.observe(2, false, grips::GrabMode::ToggleBoth, true, {true,false,false}, true, false) &&
        !retained.observe(2, false, grips::GrabMode::ToggleBoth, true, {false,false,true}, true, false),
        "Toggle Drop retains the object through the release of the spent weapon gesture");
    check(!retained.observe(2, false, grips::GrabMode::ToggleBoth, true, {true,true,false}, true, false) &&
        retained.observe(2, false, grips::GrabMode::ToggleBoth, true, {false,false,true}, true, false),
        "A subsequent complete click physically drops the retained inactive weapon");
    check(!retained.observe(3, true, grips::GrabMode::ToggleBoth, true, {true,true,false}, true, false), "A new grab cannot inherit an old completed release");
    check(!retained.observe(3, true, grips::GrabMode::ToggleBoth, true, {false,false,true}, false, false), "Provider-owned input cannot release the weapon");
    check(!retained.observe(3, true, grips::GrabMode::ToggleBoth, true, {false,false,false}, true, false), "Yielding provider ownership does not replay its release");
    check(retained.observe(3, true, grips::GrabMode::ToggleBoth, true, {true,true,false}, true, false), "A fresh press after UI capture still releases a toggle grip");
    grips::Grip off;
    (void)off.observe(4, true, grips::GrabMode::HoldBoth, true, {true,true,false}, true, false);
    check(off.observe(4, true, grips::GrabMode::HoldBoth, true, {false,false,true}, true, false), "Hold release reaches drop policy");
    off.keepAttached();
    for (int frame=0; frame<5; ++frame) check(!off.observe(4, true, grips::GrabMode::HoldBoth, true, {}, true, false), "Drop Off must not repeat rejected releases while the button stays open");
    (void)off.observe(4, true, grips::GrabMode::HoldBoth, true, {true,true,false}, true, false);
    check(off.observe(4, true, grips::GrabMode::HoldBoth, true, {false,false,true}, true, false), "Holding again rearms the next valid release");
    grips::Grip resumed;
    (void)resumed.observe(5, true, grips::GrabMode::HoldBoth, false, {true,true,false}, true, false);
    resumed.suspend();
    check(!resumed.observe(5, true, grips::GrabMode::HoldBoth, false, {false,false,true}, true, false), "Menu release cannot drop a resumed hold-mode weapon");
    (void)resumed.observe(5, true, grips::GrabMode::HoldBoth, false, {true,true,false}, true, false);
    check(resumed.observe(5, true, grips::GrabMode::HoldBoth, false, {false,false,true}, true, false), "Hold mode rearms after menu resume");

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
    using namespace rock::carried_weapon_projectile;
    namespace entry = rock::physical_weapon_pair_policy;
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
    check(entry::advance({.incomingHeld=true, .nativeOriginalPresent=true, .allReady=true}) == entry::EntryAction::Wait,
        "Preparing the second gun cannot admit a mixed native/physical pair");
    check(entry::advance({.incomingHeld=false, .nativeOriginalPresent=true}) == entry::EntryAction::Cancel,
        "Cancellation before conversion preserves the original native gun");
    check(entry::advance({.converted=true, .incomingHeld=false, .placementPending=true}) == entry::EntryAction::Wait,
        "An interrupted conversion must retain its receipt until original placement completes");
    check(entry::advance({.converted=true, .incomingHeld=false, .outgoingHeld=true}) == entry::EntryAction::RestoreOriginal,
        "Interrupted entry restores the original weapon instead of leaving a half-completed replacement");
    check(entry::advance({.converted=true, .incomingHeld=true, .outgoingHeld=true, .allReady=true}) == entry::EntryAction::Complete,
        "Both physical weapon backends must be ready before dual entry completes");
    check(entry::advance({.converted=true, .incomingHeld=false, .outgoingHeld=false}) == entry::EntryAction::Cancel,
        "Dropping both guns must not pick either one up again");
    check(entry::advance({.converted=true, .incomingHeld=true}) == entry::EntryAction::RestoreIncoming,
        "Losing the original hold during conversion returns the remaining gun to single handling");
    check(entry::advance({.incomingHeld=true, .nativeOriginalPresent=true, .failed=true}) == entry::EntryAction::Cancel,
        "A failed preflight cannot convert the original weapon");
    check(entry::advance({.converted=true, .incomingHeld=true, .outgoingHeld=true, .failed=true}) == entry::EntryAction::RestoreOriginal,
        "A mechanical failure after conversion restores the original exact item");
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
