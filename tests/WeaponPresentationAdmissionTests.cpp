#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/visual/HandWorldAuthorityRegistryPolicy.h"
#include "physics-interaction/weapon/collision/WeaponIntentStabilityPolicy.h"
#include "physics-interaction/weapon/presentation/WeaponPresentationWarmUpPolicy.h"

#include <cstdio>

namespace
{
    namespace warmUp = rock::weapon_presentation_warm_up_policy;
    namespace stability = rock::weapon_intent_stability_policy;
    using BlockReason = warmUp::BlockReason;

    bool expectTrue(const char* label, const bool actual)
    {
        if (actual) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, const bool actual)
    {
        if (!actual) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectReason(const char* label, const BlockReason actual, const BlockReason expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %s got %s\n", label, warmUp::blockReasonName(expected), warmUp::blockReasonName(actual));
        return false;
    }

    // A settled two-hand carry with everything ready.
    warmUp::Inputs readyTwoHandInputs()
    {
        return warmUp::Inputs{
            .handAttached = { true, true },
            .handPublicationReady = { true, true },
            .handWeaponPairSuppressed = { true, true },
            .nativeRenderable = true,
            .handPoseHandoffComplete = true,
            .pairFilterReady = true,
        };
    }

    bool testWarmedUpCarryPasses()
    {
        bool ok = true;
        ok &= expectTrue("a settled carry is warmed up", warmUp::isWarmedUp(readyTwoHandInputs()));
        ok &= expectReason("a settled carry has no block reason", warmUp::blockReason(readyTwoHandInputs()), BlockReason::None);
        return ok;
    }

    // Case 6: the hand calibration has not opened its publication channel yet
    // while the weapon is already touching something.
    bool testDelayedCalibrationBlocksPresentation()
    {
        bool ok = true;
        auto inputs = readyTwoHandInputs();
        inputs.handPublicationReady[0] = false;
        ok &= expectFalse("an unpublishable hand blocks presentation", warmUp::isWarmedUp(inputs));
        ok &= expectReason("the reason names the closed channel", warmUp::blockReason(inputs), BlockReason::HandPublicationClosed);

        // A hand that is not part of the carry cannot block it.
        inputs.handAttached[0] = false;
        ok &= expectTrue("an unattached hand does not block", warmUp::isWarmedUp(inputs));
        return ok;
    }

    /*
     * Case 7: the off hand rests on the weapon at equip. Its proxy and the
     * weapon proxy share the graph until the per-pair suppression lease is
     * live, so any correction in that window is manufactured.
     */
    bool testUnsuppressedHandPairBlocksPresentation()
    {
        bool ok = true;
        auto inputs = readyTwoHandInputs();
        inputs.handWeaponPairSuppressed[0] = false;
        ok &= expectFalse("an unsuppressed owning hand blocks presentation", warmUp::isWarmedUp(inputs));
        ok &= expectReason("the reason names the pair", warmUp::blockReason(inputs), BlockReason::HandWeaponPairNotSuppressed);

        inputs = readyTwoHandInputs();
        inputs.pairFilterReady = false;
        ok &= expectFalse("a missing pair filter blocks presentation", warmUp::isWarmedUp(inputs));
        ok &= expectReason("the reason names the filter", warmUp::blockReason(inputs), BlockReason::PairFilterUnavailable);
        return ok;
    }

    bool testEquipHandoverBlocksPresentation()
    {
        bool ok = true;
        auto inputs = readyTwoHandInputs();
        inputs.nativeRenderable = false;
        ok &= expectReason("an unrendered weapon blocks first", warmUp::blockReason(inputs), BlockReason::WeaponNotRenderable);

        inputs = readyTwoHandInputs();
        inputs.handPoseHandoffComplete = false;
        ok &= expectReason("a pending handoff blocks", warmUp::blockReason(inputs), BlockReason::HandPoseHandoffPending);

        // A detached weapon still waits for the handover: the weapon itself is
        // not yet where the correction claims it is.
        auto detached = readyTwoHandInputs();
        detached.handAttached = { false, false };
        detached.nativeRenderable = false;
        ok &= expectFalse("a detached weapon still waits for the handover", warmUp::isWarmedUp(detached));
        return ok;
    }

    /*
     * Case 16: the equip transient. The weapon flies hundreds of game units to
     * its attach point; a collision body driven along that flight sweeps the
     * world and manufactures a large correction against a static surface.
     */
    bool testEquipFlightIsNotAdmissibleCollisionIntent()
    {
        bool ok = true;
        stability::State state{};
        RE::NiTransform driver{};
        driver.rotate.MakeIdentity();
        driver.scale = 1.0f;

        auto weaponAt = [&](const float x) {
            RE::NiTransform weapon = driver;
            weapon.translate = RE::NiPoint3{ x, 0.0f, 0.0f };
            return weapon;
        };

        // The attach flight: the weapon closes on the hand over many frames.
        float distance = 19600.0f;
        stability::Sample sample{};
        for (int frame = 0; frame < 30; ++frame) {
            sample = stability::update(state, true, driver, true, weaponAt(distance), 0x77u);
            ok &= expectFalse(
                "an in-flight weapon is never admissible collision intent",
                stability::isAdmissibleCollisionIntent(sample));
            distance *= 0.5f;
        }

        // It settles. Admission needs several consecutive steady frames.
        const RE::NiTransform settled = weaponAt(3.0f);
        for (std::uint32_t frame = 0; frame < stability::kRequiredStableFrameCount; ++frame) {
            sample = stability::update(state, true, driver, true, settled, 0x77u);
            ok &= expectFalse(
                "admission waits for the full stable window",
                stability::isAdmissibleCollisionIntent(sample));
        }
        sample = stability::update(state, true, driver, true, settled, 0x77u);
        ok &= expectTrue("a settled weapon is admissible", stability::isAdmissibleCollisionIntent(sample));

        // Case 8: a re-equip restarts the window, so no snapshot from the old
        // topology can be replayed against the new one.
        sample = stability::update(state, true, driver, true, settled, 0x88u);
        ok &= expectFalse("a new generation is not immediately admissible", stability::isAdmissibleCollisionIntent(sample));
        return ok;
    }

    /*
     * Case 17: the role gate. DynamicContact keeps weapon-follow status on
     * purpose. A hand deviated by its own contact solve is still holding the
     * weapon, and removing it would silently change recoil eligibility while
     * the firing hand is deviated. The dangerous window is closed by the
     * warm-up gate above, not by demoting the role.
     */
    bool testWeaponFollowRoleTable()
    {
        bool ok = true;
        using Role = rock::hand_world_authority_registry_policy::Role;
        const auto follows = rock::hand_world_authority_registry_policy::weaponPresentationFollowsRole;

        ok &= expectTrue("dynamic contact keeps weapon follow", follows(Role::DynamicContact));
        ok &= expectTrue("weapon collision follows", follows(Role::WeaponCollision));
        ok &= expectTrue("primary grip follows", follows(Role::PrimaryGrip));
        ok &= expectTrue("support grip follows", follows(Role::SupportGrip));
        ok &= expectTrue("gunstock follows", follows(Role::Gunstock));
        ok &= expectTrue("equip handoff follows", follows(Role::EquipHandoff));
        ok &= expectTrue("weapon return follows", follows(Role::WeaponReturn));
        ok &= expectTrue("a weapon-coupled provider follows", follows(Role::ProviderWeaponCoupled));

        // A hand that is holding something else must never move the weapon.
        ok &= expectFalse("grab held does not follow", follows(Role::GrabHeld));
        ok &= expectFalse("grab return does not follow", follows(Role::GrabReturn));
        ok &= expectFalse("primary detach does not follow", follows(Role::PrimaryDetach));
        ok &= expectFalse("a plain provider does not follow", follows(Role::Provider));
        ok &= expectFalse("an unknown owner does not follow", follows(Role::Unknown));
        return ok;
    }

    /*
     * The layer-51 comment used to claim the weapon proxy solved only against
     * world surfaces and tagged cars. It also solves against both dynamic hand
     * proxy rows, which is precisely why the pair suppression must be live
     * before a correction is believed.
     */
    bool testWeaponProxyReallySeesHandProxies()
    {
        bool ok = true;
        namespace layers = rock::collision_layer_policy;
        const auto mask = layers::buildRockDynamicWeaponProxyExpectedMask(true, true, true);
        ok &= expectTrue(
            "the weapon proxy sees the right hand proxy row",
            layers::maskEnablesLayer(mask, layers::ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY));
        ok &= expectTrue(
            "the weapon proxy sees the left hand proxy row",
            layers::maskEnablesLayer(mask, layers::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY));

        const auto disabledMask = layers::buildRockDynamicWeaponProxyExpectedMask(false, true, true);
        ok &= expectFalse(
            "disabled interactions drop the right hand proxy row",
            layers::maskEnablesLayer(disabledMask, layers::ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY));
        ok &= expectFalse(
            "disabled interactions drop the left hand proxy row",
            layers::maskEnablesLayer(disabledMask, layers::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY));
        return ok;
    }
}

int main()
{
    bool ok = true;
    ok &= testWarmedUpCarryPasses();
    ok &= testDelayedCalibrationBlocksPresentation();
    ok &= testUnsuppressedHandPairBlocksPresentation();
    ok &= testEquipHandoverBlocksPresentation();
    ok &= testEquipFlightIsNotAdmissibleCollisionIntent();
    ok &= testWeaponFollowRoleTable();
    ok &= testWeaponProxyReallySeesHandProxies();
    return ok ? 0 : 1;
}
