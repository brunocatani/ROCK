#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/WeaponSupport.h"

#include <cstdio>
#include <cstdint>

namespace
{
    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }

        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }

        std::printf("%s expected false\n", label);
        return false;
    }

    template <class T>
    bool expectEqual(const char* label, T actual, T expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %llu got %llu\n", label, static_cast<unsigned long long>(expected), static_cast<unsigned long long>(actual));
        return false;
    }
}

int main()
{
    bool ok = true;

    using namespace rock::contact_pipeline_policy;

    const ContactEndpoint weapon{
        .bodyId = 100,
        .layer = 44,
        .kind = ContactEndpointKind::Weapon,
    };

    const auto leftWeapon = classifyContact(
        ContactEndpoint{ .bodyId = 10, .layer = 43, .kind = ContactEndpointKind::LeftHand },
        weapon);
    ok &= expectEqual("left hand weapon contact routes as hand weapon", leftWeapon.route, ContactRoute::HandWeapon);
    ok &= expectTrue("left hand weapon contact drives support evidence", leftWeapon.drivesWeaponSupportContact);
    ok &= expectEqual("left hand remains contact source", leftWeapon.source.kind, ContactEndpointKind::LeftHand);

    const auto rightWeapon = classifyContact(
        ContactEndpoint{ .bodyId = 20, .layer = 43, .kind = ContactEndpointKind::RightHand },
        weapon);
    ok &= expectEqual("right hand weapon contact routes as hand weapon", rightWeapon.route, ContactRoute::HandWeapon);
    ok &= expectTrue("right hand weapon contact drives support evidence", rightWeapon.drivesWeaponSupportContact);
    ok &= expectEqual("right hand remains contact source", rightWeapon.source.kind, ContactEndpointKind::RightHand);

    using rock::weapon_two_handed_grip_math::canProcessNormalGrabInput;
    ok &= expectFalse("left normal grab is blocked while support grip owns weapon", canProcessNormalGrabInput(true, true, true, false));
    ok &= expectFalse("right normal grab is blocked while firing hand owns equipped weapon", canProcessNormalGrabInput(false, false, true, false));
    ok &= expectTrue("right normal grab is restored while primary hand is detached", canProcessNormalGrabInput(false, false, true, true));
    ok &= expectTrue("right normal grab stays available without equipped weapon", canProcessNormalGrabInput(false, false, false, false));

    using namespace rock::equipped_weapon_manual_ownership_policy;
    ok &= expectTrue("manual grip feature is available for active full-solver weapon", featureAvailable(true, true, true, true, 10));
    ok &= expectFalse("manual grip feature is unavailable without active weapon node", featureAvailable(true, true, true, false, 10));
    ok &= expectFalse("manual grip feature is unavailable without weapon generation", featureAvailable(true, true, true, true, 0));
    ok &= expectTrue("pending trigger-equip grip waits while runtime weapon is not ready",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .configEnabled = true,
            .fullTwoHandedSolverMode = true,
            .primaryPoseBlockerAvailable = true,
            .virtualHolstersOwnsInput = false,
        }));
    ok &= expectFalse("pending trigger-equip grip clears on release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .configEnabled = true,
            .fullTwoHandedSolverMode = true,
            .primaryPoseBlockerAvailable = true,
            .virtualHolstersOwnsInput = false,
        }));
    ok &= expectFalse("pending trigger-equip grip clears for visual-only sidearm mode",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .configEnabled = true,
            .fullTwoHandedSolverMode = false,
            .primaryPoseBlockerAvailable = true,
            .virtualHolstersOwnsInput = false,
        }));
    ok &= expectFalse("pending trigger-equip grip clears when virtual holsters owns input",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .configEnabled = true,
            .fullTwoHandedSolverMode = true,
            .primaryPoseBlockerAvailable = true,
            .virtualHolstersOwnsInput = true,
        }));

    RuntimeState manualState{};
    auto manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .weaponGenerationKey = 10,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectFalse("native equip alone does not start manual ownership", manualDecision.active);
    ok &= expectFalse("native equip alone does not request drop", manualDecision.dropRequested);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .weaponGenerationKey = 10,
            .startRequested = true,
            .primaryGripRetained = true,
            .supportGripRetained = false,
        });
    ok &= expectTrue("first retained grip starts manual ownership", manualDecision.started);
    ok &= expectTrue("manual ownership remains active while primary grip retained", manualDecision.active);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .weaponGenerationKey = 10,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectTrue("manual ownership requests drop when all grips release", manualDecision.dropRequested);
    ok &= expectFalse("drop request clears manual ownership state", manualState.active);

    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .weaponGenerationKey = 11,
            .startRequested = true,
            .primaryGripRetained = false,
            .supportGripRetained = true,
        });
    ok &= expectTrue("support grip can start manual ownership", manualDecision.started);
    manualDecision = update(manualState,
        Input{
            .weaponEquipped = true,
            .weaponGenerationKey = 12,
            .startRequested = false,
            .primaryGripRetained = false,
            .supportGripRetained = false,
        });
    ok &= expectTrue("weapon generation change clears manual ownership", manualDecision.cleared);
    ok &= expectFalse("weapon generation change does not drop old equipped weapon", manualDecision.dropRequested);

    using namespace rock::equipped_weapon_drop_policy;
    ok &= expectEqual("support release normally drops from left hand", sourceForSupportRelease(false), SourceHand::Left);
    ok &= expectEqual("same-frame primary release drops from right hand", sourceForSupportRelease(true), SourceHand::Right);
    ok &= expectTrue("right-hand release surrenders to VirtualHolsters when source hand owns input",
        shouldSurrenderReleaseToVirtualHolsters(SourceHand::Right, true));
    ok &= expectTrue("left-hand release surrenders to VirtualHolsters when source hand owns input",
        shouldSurrenderReleaseToVirtualHolsters(SourceHand::Left, true));
    ok &= expectFalse("release does not surrender without VirtualHolsters source-hand ownership",
        shouldSurrenderReleaseToVirtualHolsters(SourceHand::Right, false));
    ok &= expectFalse("unknown release source never surrenders to VirtualHolsters",
        shouldSurrenderReleaseToVirtualHolsters(SourceHand::None, true));

    SuppressionFrameState postDropSuppression{};
    beginPostDropSuppression(postDropSuppression);
    ok &= expectTrue("post-drop suppression starts active", postDropSuppression.active());
    ok &= expectEqual("post-drop suppression frame count", postDropSuppression.framesRemaining, kPostDropHandCollisionSuppressionFrames);
    for (int frame = 1; frame < kPostDropHandCollisionSuppressionFrames; ++frame) {
        ok &= expectFalse("post-drop suppression remains active before final frame", advancePostDropSuppression(postDropSuppression));
        ok &= expectTrue("post-drop suppression countdown remains active", postDropSuppression.active());
    }
    ok &= expectTrue("post-drop suppression expires on final frame", advancePostDropSuppression(postDropSuppression));
    ok &= expectFalse("post-drop suppression inactive after expiry", postDropSuppression.active());

    return ok ? 0 : 1;
}
