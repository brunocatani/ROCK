#include "physics-interaction/weapon/WeaponSurfaceSupport.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    bool check(bool condition, const char* message)
    {
        if (!condition) std::printf("FAIL: %s\n", message);
        return condition;
    }

    bool near(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return rock::weaponSolverLength(rock::weaponSolverSub(a, b)) < 0.002f;
    }

    bool sameRotation(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (std::abs(a.entry[row][column] - b.entry[row][column]) > 0.000001f) return false;
            }
        }
        return true;
    }

    rock::weapon_surface_support::Contact contactAt(const RE::NiPoint3& local, float scale = 1.0f)
    {
        rock::weapon_surface_support::Contact contact{};
        contact.world = 1;
        contact.shape = 2;
        contact.generation = 3;
        contact.proxyBodyId = 4;
        contact.surfaceBodyId = 5;
        contact.sampledAtMilliseconds = 1000;
        contact.weaponWorld = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        contact.weaponWorld.translate = { 4200.0f, -7300.0f, 1200.0f };
        contact.weaponWorld.scale = scale;
        contact.surfaceWorld = rock::transform_math::makeIdentityTransform<RE::NiTransform>();
        contact.weaponPointLocal = local;
        contact.surfacePointLocal = rock::transform_math::localPointToWorld(contact.weaponWorld, local);
        contact.valid = true;
        return contact;
    }
}

int main()
{
    using namespace rock;
    using namespace weapon_surface_support;
    using namespace dynamic_weapon_collision_policy;
    bool ok = true;

    Toggle toggle{};
    ok &= check(!toggle.consume({ true, true, true, 0 }), "held-on-entry requires release");
    ok &= check(!toggle.consume({ true, false, false, 0 }), "release arms without toggling");
    ok &= check(toggle.consume({ true, true, true, 0 }), "first distinct click toggles");
    for (int i = 0; i < 100; ++i) ok &= check(!toggle.consume({ true, true, false, 0 }), "holding never repeats");
    ok &= check(!toggle.consume({ true, false, false, 0 }), "button release keeps latch");
    ok &= check(toggle.consume({ true, true, true, 0 }), "second distinct click toggles");
    toggle.consume({ false, false, false, 0 });
    ok &= check(!toggle.consume({ true, true, true, 0 }), "menu-held click cannot replay");
    toggle.consume({ true, false, false, 0 });
    ok &= check(!toggle.consume({ true, true, true, 101 }), "stale click rejected");
    toggle.consume({ true, false, false, 0 });
    ok &= check(toggle.consume({ true, false, true, 0 }), "short press and release between frames counts once");

    auto contact = contactAt({ 0.0f, 30.0f, -4.0f });
    ok &= check(isFresh(contact, 1, 3, 4, 1001), "fresh contact admitted");
    ok &= check(!isFresh(contact, 1, 3, 4, 1101), "old contact rejected");
    ok &= check(!isFresh(contact, 1, 3, 4, 999), "future contact rejected");
    ok &= check(!isFresh(contact, 2, 3, 4, 1001), "world change rejected");
    ok &= check(!isFresh(contact, 1, 6, 4, 1001), "weapon generation change rejected");
    ok &= check(!isFresh(contact, 1, 3, 7, 1001), "retired proxy contact rejected");
    ok &= check(contactStillTouches(contact, contact.weaponWorld, contact.surfaceWorld), "resting contact remains eligible");
    auto movedWeapon = contact.weaponWorld;
    movedWeapon.translate.x += 10.0f;
    ok &= check(!contactStillTouches(contact, movedWeapon, contact.surfaceWorld), "recent contact cannot latch after weapon leaves it");
    ContactChannel channel;
    Contact copied{};
    ok &= check(!channel.read(copied), "empty contact unavailable");
    channel.publish(contact);
    ok &= check(channel.read(copied) && near(copied.weaponPointLocal, contact.weaponPointLocal), "contact payload retained coherently");
    channel.clear();
    ok &= check(!channel.read(copied), "retirement clears acquisition evidence");

    // No semantic part discriminator exists. Exercise arbitrary contact
    // positions including either side, underside, rear, and non-unit scale.
    for (const auto local : { RE::NiPoint3{0, 30, -4}, RE::NiPoint3{7, 0, 0}, RE::NiPoint3{0, -20, 5} }) {
        for (const float scale : { 0.7f, 1.0f, 1.8f }) {
            auto sample = contactAt(local, scale);
            State state{};
            const RE::NiPoint3 grip{ 0, -4, 0 };
            auto requested = sample.weaponWorld;
            ok &= check(capture(state, sample, requested, grip), "arbitrary contact captures");
            RE::NiTransform supported{};
            ok &= check(solve(state, requested, sample.surfaceWorld, supported), "capture solves");
            ok &= check(near(supported.translate, sample.weaponWorld.translate) && rotationDeltaDegrees(supported, sample.weaponWorld) < 0.01f, "acquisition does not snap");
            const auto anchor = sample.surfacePointLocal;
            for (int frame = 1; frame <= 300; ++frame) {
                requested.translate.x += 0.3f;
                requested.translate.z += 0.04f;
                ok &= check(solve(state, requested, sample.surfaceWorld, supported), "aim stays finite during translation");
                ok &= check(near(transform_math::localPointToWorld(supported, local), anchor), "moving hands cannot drag planted pivot");
            }
            ok &= check(state.latched(), "elapsed frames and absent contacts never unlatch");
            ok &= check(rotationDeltaDegrees(supported, sample.weaponWorld) > 1.0f, "latched weapon can still aim");

            // Same pivot must be used by the Havok authority, the contact
            // body, and the rendered weapon, including rotation and VR scale.
            const RE::NiPoint3 center{ 2, 8, 3 };
            const auto authority = makeGripAuthorityTarget(supported, local);
            const auto body = makeContactBodyTargetFromGripAuthority(authority, center, scale, local);
            const auto restored = reconstructWeaponRoot(body, center, scale);
            ok &= check(near(authority.translate, anchor), "physics authority is at the planted point");
            ok &= check(near(restored.translate, supported.translate) && sameRotation(restored.rotate, supported.rotate), "pivot/body roundtrip preserves visible pose");
            ok &= check(!evaluateGripRecovery(body, authority, center, scale, 1.0f, local).resetNow, "correct surface pivot never triggers grip recovery");

            release(state);
            ok &= check(!state.latched() && state.ownsPose(), "second click begins controlled return");
            auto returning = advanceReturn(state, requested, 0.0f);
            ok &= check(near(returning.translate, supported.translate), "release begins at presented pose");
            for (int frame = 0; frame < 30; ++frame) returning = advanceReturn(state, requested, 1.0f / 90.0f);
            ok &= check(!state.ownsPose() && near(returning.translate, requested.translate), "return hands control back to current grip");
        }
    }

    State invalid{};
    auto invalidContact = contact;
    invalidContact.weaponPointLocal.x = std::numeric_limits<float>::quiet_NaN();
    ok &= check(!capture(invalid, invalidContact, contact.weaponWorld, {}), "nonfinite contact cannot capture");
    ok &= check(!capture(invalid, contact, contact.weaponWorld, contact.weaponPointLocal), "zero aiming lever rejected");
    std::printf("Weapon surface support: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
