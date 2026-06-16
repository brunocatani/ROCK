#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/weapon/WeaponSupport.h"

#include <array>
#include <cstdio>
#include <cstdint>
#include <cstring>

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
    using rock::weapon_two_handed_grip_math::resolveSupportReleaseManualAction;
    using rock::weapon_two_handed_grip_math::SupportReleaseManualAction;
    ok &= expectFalse("left normal grab is blocked while support grip owns weapon", canProcessNormalGrabInput(true, true, true, false));
    ok &= expectFalse("right normal grab is blocked while firing hand owns equipped weapon", canProcessNormalGrabInput(false, false, true, false));
    ok &= expectTrue("right normal grab is restored while primary hand is detached", canProcessNormalGrabInput(false, false, true, true));
    ok &= expectTrue("right normal grab stays available without equipped weapon", canProcessNormalGrabInput(false, false, false, false));
    ok &= expectTrue("full two-handed support still owns weapon transform",
        rock::weapon_support_authority_policy::supportGripOwnsWeaponTransform(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("full two-handed support applies primary hand authority while active",
        rock::weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectTrue("support grip continues to apply offhand visual authority",
        rock::weapon_support_authority_policy::supportGripAppliesSupportHandAuthority(rock::weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver));
    ok &= expectEqual("support release keeps primary ownership when primary grip is held",
        resolveSupportReleaseManualAction(true, true),
        SupportReleaseManualAction::KeepPrimaryOwnership);
    ok &= expectEqual("support release drops equipped weapon when primary grip is not held",
        resolveSupportReleaseManualAction(true, false),
        SupportReleaseManualAction::DropEquippedWeapon);
    ok &= expectEqual("support release ends support only when manual detach is disabled",
        resolveSupportReleaseManualAction(false, true),
        SupportReleaseManualAction::EndSupportOnly);

    using namespace rock::equipped_weapon_manual_ownership_policy;
    ok &= expectTrue("manual grip feature is available for active equipped weapon", featureAvailable(true, true, true, 10));
    ok &= expectFalse("manual grip feature is unavailable without active weapon node", featureAvailable(true, true, false, 10));
    ok &= expectFalse("manual grip feature is unavailable without weapon generation", featureAvailable(true, true, true, 0));
    ok &= expectTrue("pending trigger-equip grip waits while runtime weapon is not ready",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .configEnabled = true,
            .primaryPoseBlockerAvailable = true,
            .virtualHolstersOwnsInput = false,
        }));
    ok &= expectFalse("pending trigger-equip grip clears on release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = false,
            .configEnabled = true,
            .primaryPoseBlockerAvailable = true,
            .virtualHolstersOwnsInput = false,
        }));
    ok &= expectTrue("pending trigger-equip grip is retained for visual-only sidearm release",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .configEnabled = true,
            .primaryPoseBlockerAvailable = true,
            .virtualHolstersOwnsInput = false,
        }));
    ok &= expectFalse("pending trigger-equip grip clears when virtual holsters owns input",
        shouldKeepPendingPrimaryOnlyStart(PendingPrimaryOnlyStartInput{
            .pending = true,
            .gripHeld = true,
            .configEnabled = true,
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

    using namespace rock::weapon_part_runtime;
    std::array<Target, 3> weaponPartTargets{};
    weaponPartTargets[0].active = true;
    weaponPartTargets[0].ownerToken = 10;
    weaponPartTargets[0].weaponGenerationKey = 0xABC;
    weaponPartTargets[0].flags = MatchBodyId;
    weaponPartTargets[0].grabMode = GrabMode::AttachOnly;
    weaponPartTargets[0].bodyId = 42;
    weaponPartTargets[0].priority = 1;
    weaponPartTargets[0].groupId = 7;
    weaponPartTargets[1].active = true;
    weaponPartTargets[1].ownerToken = 11;
    weaponPartTargets[1].weaponGenerationKey = 0xABC;
    weaponPartTargets[1].flags = MatchBodyId;
    weaponPartTargets[1].grabMode = GrabMode::FullTwoHandAuthority;
    weaponPartTargets[1].bodyId = 42;
    weaponPartTargets[1].priority = 2;
    weaponPartTargets[2].active = true;
    weaponPartTargets[2].ownerToken = 12;
    weaponPartTargets[2].weaponGenerationKey = 0xABC;
    weaponPartTargets[2].flags = MatchSourceName;
    weaponPartTargets[2].grabMode = GrabMode::AttachOnly;
    std::memcpy(weaponPartTargets[2].sourceName.data(), "BoltNode", 8);
    weaponPartTargets[2].priority = 3;

    const auto unmatchedWhitelist = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("weapon part whitelist becomes active for matching generation", unmatchedWhitelist.whitelistActive);
    ok &= expectFalse("weapon part whitelist fails closed for unregistered contact", unmatchedWhitelist.matched);

    const auto matchedBody = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("weapon part body target matches", matchedBody.matched);
    ok &= expectEqual("higher priority matching target selects full authority",
        matchedBody.grabMode,
        GrabMode::FullTwoHandAuthority);
    ok &= expectEqual("matching target owner is preserved",
        matchedBody.ownerToken,
        static_cast<std::uint64_t>(11));

    const auto matchedName = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "BoltNode",
        });
    ok &= expectTrue("weapon part source-name target matches", matchedName.matched);
    ok &= expectEqual("source-name target selects attach-only mode",
        matchedName.grabMode,
        GrabMode::AttachOnly);

    const auto otherGeneration = resolveTarget(weaponPartTargets,
        Contact{
            .weaponGenerationKey = 0xDEF,
            .bodyId = 100,
            .sourceRoot = 0x900,
            .sourceName = "Receiver",
        });
    ok &= expectFalse("weapon part whitelist does not apply to other generation", otherGeneration.whitelistActive);

    std::array<Target, 1> strictWeaponPartTarget{};
    strictWeaponPartTarget[0].active = true;
    strictWeaponPartTarget[0].ownerToken = 20;
    strictWeaponPartTarget[0].weaponGenerationKey = 0xABC;
    strictWeaponPartTarget[0].flags = MatchBodyId | MatchSourceName | MatchPartKind;
    strictWeaponPartTarget[0].grabMode = GrabMode::AttachOnly;
    strictWeaponPartTarget[0].bodyId = 42;
    strictWeaponPartTarget[0].partKind = rock::WeaponPartKind::Bolt;
    std::memcpy(strictWeaponPartTarget[0].sourceName.data(), "BoltNode", 8);

    const auto strictMatched = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .partKind = rock::WeaponPartKind::Bolt,
        });
    ok &= expectTrue("weapon part target requires and accepts all requested match fields", strictMatched.matched);

    const auto strictWrongName = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "Receiver",
            .partKind = rock::WeaponPartKind::Bolt,
        });
    ok &= expectFalse("weapon part target rejects partial match with wrong source name", strictWrongName.matched);

    const auto strictWrongPart = resolveTarget(strictWeaponPartTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .partKind = rock::WeaponPartKind::Receiver,
        });
    ok &= expectFalse("weapon part target rejects partial match with wrong semantic part", strictWrongPart.matched);

    using namespace rock::hand_collision_suppression_math;
    SuppressionSet<2> postDropSuppression{};
    const auto postDropSuppressionResult = beginSuppression(postDropSuppression, 42, 0);
    ok &= expectTrue("post-drop suppression stores release body", postDropSuppressionResult.stored);
    DelayedRestoreState postDropRestore{};
    ok &= expectTrue("post-drop suppression uses grab release delay seconds", beginDelayedRestore(postDropRestore, postDropSuppression, 0.8f));
    ok &= expectFalse("post-drop suppression remains active before configured delay", advanceDelayedRestore(postDropRestore, postDropSuppression, 0.79f));
    ok &= expectTrue("post-drop suppression expires at configured delay", advanceDelayedRestore(postDropRestore, postDropSuppression, 0.01f));

    return ok ? 0 : 1;
}
