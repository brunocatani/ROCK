#include "physics-interaction/weapon/AkimboSessionPolicy.h"
#include "physics-interaction/weapon/WeaponCyclePolicy.h"
#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include "physics-interaction/weapon/LooseWeaponExperimentPolicy.h"
#include "physics-interaction/weapon/LooseWeaponRecoilPolicy.h"
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
    using namespace rock::loose_weapon_experiment;
    check(selectReference(42,0) == 42 && selectReference(0,42) == 42,
        "Either physical hand can own the single loose firearm");
    check(selectReference(42,42) == 42, "Two hands on one reference still mean one weapon");
    check(selectReference(42,43) == 0 && selectReference(0,0) == 0,
        "The experiment cannot admit two guns or an empty hold");
    check(canAdmit(true,false,42) && !canAdmit(true,true,42) && !canAdmit(false,false,42) && !canAdmit(true,false,0),
        "A loose shot requires its own valid reference and no native equipped weapon");
    check(suppressNativePress(true,true,true,false) && !suppressNativePress(true,false,true,false) &&
        !suppressNativePress(false,true,true,false) && !suppressNativePress(true,true,true,true),
        "Loose triggers cannot also punch, while releases, other hands and UI retain native handling");
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
    check(reloadClipPriority("Animations/MWVictor/WPNReload.hkx") &&
        emptyReloadClipPriority("Animations/M249/WPNReloadEmpty.hkt") &&
        reserveReloadClipPriority("Animations/spas12/WPNReloadReserve.hkx"),
        "Authored normal, empty and tactical reload clips remain distinct");
    check(!reloadClipPriority("WPNReloadReserve.hkx") && !reloadClipPriority("WPNReloadSlave.hkx") &&
        !emptyReloadClipPriority("WPNReload.hkx"), "Reload selection cannot pick a different operation or slave clip");
    for (const auto tag : {"weaponFire", "ReloadComplete", "reloadEnd", "Equip", "Attach", "AnimationDriven"})
        check(presentationEvent(tag) == PresentationEvent::Ignore,
            "Weapon presentation cannot replay actor gameplay, ammo, equip or arm-animation events");
    check(presentationEvent("SoundPlay") == PresentationEvent::Sound && presentationEvent("CullBone") == PresentationEvent::HidePart &&
        presentationEvent("UnCullBone") == PresentationEvent::ShowPart, "Authored sound and weapon visibility markers retain their separate meanings");
    struct Point { float x{}, y{}, z{}; };
    struct Matrix { float entry[3][4]{}; };
    struct Transform { Matrix rotate; Point translate; float scale{1}; };
    auto modelRest = rock::transform_math::makeIdentityTransform<Transform>();
    modelRest.rotate.entry[0][0] = 0; modelRest.rotate.entry[0][1] = -1;
    modelRest.rotate.entry[1][0] = 1; modelRest.rotate.entry[1][1] = 0;
    modelRest.translate = {20,30,40};
    auto clipRest = rock::transform_math::makeIdentityTransform<Transform>();
    clipRest.translate = {2,3,4};
    auto slide = clipRest; slide.translate.y += 5;
    const auto mapped = retargetPart(modelRest, rock::transform_math::invertTransform(clipRest), slide);
    check(std::abs(mapped.translate.x - 25) < 0.0001f && std::abs(mapped.translate.y - 30) < 0.0001f &&
        std::abs(mapped.translate.z - 40) < 0.0001f, "Slide displacement follows the model's rotated bind basis without moving its rest offset");
    const auto stationary = retargetPart(modelRest, rock::transform_math::invertTransform(clipRest), clipRest);
    check(std::abs(stationary.translate.x - 20) < 0.0001f && std::abs(stationary.translate.y - 30) < 0.0001f,
        "A neutral clip sample preserves the assembled model's exact rest pose");
    rock::loose_weapon_recoil::Kick kick;
    kick.fire(0.3926990817f, 0, 1, 5, 0.1f, 0.3f);
    check(std::abs(kick.offset - 3) < 0.0001f && std::abs(kick.duration - 0.2f) < 0.0001f,
        "Native recoil uses the verified 4/pi normalization and native setting ranges");
    const float halfKick = kick.duration * 0.5f;
    check(std::abs(kick.advance(halfKick) - 0.75f) < 0.0001f && kick.advance(halfKick) == 0,
        "Native kickback returns quadratically to neutral without accumulating a held offset");
    kick.fire(50,0,1,5,0.1f,0.3f);
    check(kick.offset == 5 && kick.duration == 0.3f, "Large recoil remains within the native setting limits");
    kick.fire(std::numeric_limits<float>::quiet_NaN(),0,1,5,0.1f,0.3f);
    check(kick.advance(0.01f) == 0, "Invalid recoil input cannot poison the physical grab target");
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

    OperationState burst;
    burst.begin(50); burst.bind(Hand::Right, Grip::Firing);
    check(!burst.canSustainAttack(true, true, true, 8), "A held trigger on acquisition cannot keep an attack loop alive");
    (void)burst.requestFire(true, false, true, true, 8);
    const auto burstShot = burst.requestFire(true, true, true, true, 8);
    burst.completeFire(burstShot, 0.1f);
    check(burst.canSustainAttack(true, true, true, 7), "An automatic burst keeps its sound through inter-shot cooldown");
    check(!burst.canSustainAttack(true, false, true, 7) && !burst.canSustainAttack(false, true, true, 7),
        "Trigger release and blocked gameplay end the sound while the weapon stays held");
    check(!burst.canSustainAttack(true, true, true, 0) && !burst.canSustainAttack(true, true, false, 7),
        "An empty or unknown magazine cannot sustain firing sound");
    burst.cancelInput();
    check(!burst.canSustainAttack(true, true, true, 7), "Suspension ends the burst until release-to-rearm");
    (void)burst.requestFire(true, false, true, true, 7);
    burst.bind(Hand::Left, Grip::Firing);
    check(!burst.canSustainAttack(true, true, true, 7), "Hand transfer cannot inherit an active sound loop");
    (void)burst.requestFire(true, false, true, true, 7);
    check(burst.beginReload(1.0f) && !burst.canSustainAttack(true, true, true, 7), "Reload interrupts firing sound");
    burst.advance(1.0f); burst.completeReload();
    check(!burst.canSustainAttack(true, true, true, 7), "Reload completion alone does not restart an attack loop");
    (void)burst.requestFire(true, false, true, true, 7);
    burst.bind(Hand::Left, Grip::Support);
    check(!burst.canSustainAttack(true, true, true, 7), "Support-only carry cannot sustain a firing loop");
    burst.begin(0);
    check(!burst.canSustainAttack(true, true, true, 7), "A retired session cannot sustain firing sound");
    return ok ? 0 : 1;
}
