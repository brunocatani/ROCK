#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/weapon/EquippedWeaponDropPolicy.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponPartGripReportPolicy.h"
#include "physics-interaction/weapon/WeaponPartMotionPathPolicy.h"
#include "physics-interaction/weapon/WeaponPartMotionScrubPolicy.h"
#include "physics-interaction/weapon/WeaponPartRecordIdentityPolicy.h"
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

    using rock::weapon_support_authority_policy::canApplySidearmHybridAuthority;
    using rock::weapon_support_authority_policy::resolveSidearmHybridSupportAuthorityMode;
    using rock::weapon_support_authority_policy::WeaponSupportAuthorityMode;
    ok &= expectTrue("sidearm hybrid applies to class-resolved visual-only support",
        canApplySidearmHybridAuthority(WeaponSupportAuthorityMode::VisualOnlySupport, false));
    ok &= expectFalse("sidearm hybrid never applies to full-authority resolution",
        canApplySidearmHybridAuthority(WeaponSupportAuthorityMode::FullTwoHandedSolver, false));
    ok &= expectFalse("sidearm hybrid never upgrades a provider-mandated grab mode",
        canApplySidearmHybridAuthority(WeaponSupportAuthorityMode::VisualOnlySupport, true));
    ok &= expectEqual("sidearm grab near the firing grip stays visual-only",
        resolveSidearmHybridSupportAuthorityMode(5.5f, 6.0f),
        WeaponSupportAuthorityMode::VisualOnlySupport);
    ok &= expectEqual("sidearm grab away from the firing grip takes full authority",
        resolveSidearmHybridSupportAuthorityMode(6.5f, 6.0f),
        WeaponSupportAuthorityMode::FullTwoHandedSolver);

    using rock::weapon_interaction_probe_math::isBetterProbeCandidate;
    using rock::weapon_interaction_probe_math::ProbeCandidateRank;
    ok &= expectTrue("closer weapon part wins outside dual containment",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 },
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 }));
    ok &= expectFalse("farther weapon part loses outside dual containment",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 9.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 4.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("contained tiny part beats the engulfing receiver hull",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("containment tolerance lets a near-miss tiny part beat the engulfing hull",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.81f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 1225.0f, .semanticPriority = 62 }));
    ok &= expectTrue("semantic priority breaks equal-size containment ties",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 95 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 }));
    ok &= expectFalse("equal candidates keep the current best",
        isBetterProbeCandidate(
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 },
            ProbeCandidateRank{ .distanceSquaredGame = 0.0f, .aabbDiagonalSquaredGame = 82.0f, .semanticPriority = 62 }));
    ok &= expectEqual("support release keeps primary ownership when primary grip is held",
        resolveSupportReleaseManualAction(true, true),
        SupportReleaseManualAction::KeepPrimaryOwnership);
    ok &= expectEqual("support release drops equipped weapon when primary grip is not held",
        resolveSupportReleaseManualAction(true, false),
        SupportReleaseManualAction::DropEquippedWeapon);
    ok &= expectEqual("support release ends support only when manual detach is disabled",
        resolveSupportReleaseManualAction(false, true),
        SupportReleaseManualAction::EndSupportOnly);

    using rock::weapon_two_handed_grip_math::canStartFreeHandPartGrip;
    using rock::weapon_two_handed_grip_math::canAttemptFiringGripReattach;
    using rock::weapon_two_handed_grip_math::FiringGripReattachInput;
    using rock::weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab;
    ok &= expectTrue("firing grip reattach is eligible during part carry",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
        }));
    ok &= expectFalse("firing grip reattach requires part-carry state",
        canAttemptFiringGripReattach(FiringGripReattachInput{}));
    ok &= expectFalse("firing grip reattach is blocked while a menu owns input",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
            .menuInputActive = true,
        }));
    ok &= expectFalse("firing grip reattach is blocked while the hand holds an object",
        canAttemptFiringGripReattach(FiringGripReattachInput{
            .partCarryActive = true,
            .handHoldingObject = true,
        }));

    ok &= expectTrue("held grab with the palm on the grip re-takes the firing grip",
        shouldReattachFiringGripOnGrab(true, 2.9f, 3.0f));
    ok &= expectFalse("grab reattach requires the palm inside the radius",
        shouldReattachFiringGripOnGrab(true, 3.5f, 3.0f));
    ok &= expectFalse("an open hand never re-takes the firing grip",
        shouldReattachFiringGripOnGrab(false, 0.1f, 3.0f));

    ok &= expectTrue("free hand part grip starts on grab press over a routed support part",
        canStartFreeHandPartGrip(true, true, false, false));
    ok &= expectFalse("free hand part grip requires a routed support-grip contact",
        canStartFreeHandPartGrip(false, true, false, false));
    ok &= expectFalse("free hand part grip requires a grab press edge",
        canStartFreeHandPartGrip(true, false, false, false));
    ok &= expectFalse("free hand part grip is blocked while the hand holds an object",
        canStartFreeHandPartGrip(true, true, true, false));
    ok &= expectFalse("free hand part grip does not restart while already gripping",
        canStartFreeHandPartGrip(true, true, false, true));

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

    ok &= expectEqual("primary-only carry stashes from the firing hand",
        resolveEquippedWeaponStashCarryHand(true, false, false, false, false),
        SourceHand::Right);
    ok &= expectEqual("primary-only carry follows a left firing hand",
        resolveEquippedWeaponStashCarryHand(true, false, false, false, true),
        SourceHand::Left);
    ok &= expectEqual("part carry with only the left grip stashes from the left hand",
        resolveEquippedWeaponStashCarryHand(false, true, true, false, false),
        SourceHand::Left);
    ok &= expectEqual("part carry with only the right grip stashes from the right hand",
        resolveEquippedWeaponStashCarryHand(false, true, false, true, false),
        SourceHand::Right);
    ok &= expectEqual("part carry with both grips has no stash carry hand",
        resolveEquippedWeaponStashCarryHand(false, true, true, true, false),
        SourceHand::None);
    ok &= expectEqual("inactive grip states have no stash carry hand",
        resolveEquippedWeaponStashCarryHand(false, false, false, false, false),
        SourceHand::None);

    {
        using namespace rock::weapon_part_grip_report_policy;
        ok &= expectTrue("active non-attach part grip counts as carry", partGripCountsAsCarry(true, false));
        ok &= expectFalse("attach-only part grip never counts as carry", partGripCountsAsCarry(true, true));
        ok &= expectFalse("inactive part grip never counts as carry", partGripCountsAsCarry(false, false));

        ok &= expectTrue("provider AttachOnly grab mode resolves attach-only",
            providerGrabModeIsAttachOnly(true, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::AttachOnly)));
        ok &= expectFalse("provider full-authority grab mode is not attach-only",
            providerGrabModeIsAttachOnly(true, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::FullTwoHandAuthority)));
        ok &= expectFalse("attach-only requires an active provider authority",
            providerGrabModeIsAttachOnly(false, static_cast<std::uint32_t>(rock::weapon_part_runtime::GrabMode::AttachOnly)));

        ok &= expectEqual("firing hand in gripping state reports the firing grip",
            resolveHandGripKind(true, false, false, true, false, false, false),
            HandGripKind::FiringGrip);
        ok &= expectEqual("firing hand in primary-only state reports the firing grip",
            resolveHandGripKind(false, false, true, true, false, false, false),
            HandGripKind::FiringGrip);
        ok &= expectEqual("offhand full-authority support grip reports full authority",
            resolveHandGripKind(true, false, false, false, true, false, false),
            HandGripKind::SupportFullAuthority);
        ok &= expectEqual("offhand visual-only support grip reports visual-only",
            resolveHandGripKind(true, false, false, false, true, false, true),
            HandGripKind::SupportVisualOnly);
        ok &= expectEqual("attach-only grip reports attach-only in gripping state",
            resolveHandGripKind(true, false, false, false, true, true, true),
            HandGripKind::AttachOnly);
        ok &= expectEqual("carry part grip in part-carry reports part carry",
            resolveHandGripKind(false, true, false, false, true, false, false),
            HandGripKind::PartCarry);
        ok &= expectEqual("detached firing hand attach-only grip reports attach-only",
            resolveHandGripKind(false, true, false, true, true, true, false),
            HandGripKind::AttachOnly);
        ok &= expectEqual("detached firing hand carry grip reports part carry",
            resolveHandGripKind(false, true, false, true, true, false, false),
            HandGripKind::PartCarry);
        ok &= expectEqual("idle hand reports no grip",
            resolveHandGripKind(false, false, false, false, false, false, false),
            HandGripKind::None);
        ok &= expectEqual("firing hand without part grip in part-carry reports no grip",
            resolveHandGripKind(false, true, false, true, false, false, false),
            HandGripKind::None);
    }

    {
        using namespace rock::weapon_part_record_identity_policy;
        ok &= expectEqual("P-Mag resolves the magazine slot anchor",
            resolveStructureAnchor("P-Mag"), StructureAnchor::SlotMagazine);
        ok &= expectEqual("P-Barrel resolves the barrel slot anchor",
            resolveStructureAnchor("P-Barrel"), StructureAnchor::SlotBarrel);
        ok &= expectEqual("P-Compensator resolves the muzzle slot anchor",
            resolveStructureAnchor("P-Compensator"), StructureAnchor::SlotMuzzle);
        ok &= expectEqual("WeaponBolt resolves the bolt rig anchor",
            resolveStructureAnchor("WeaponBolt"), StructureAnchor::RigBolt);
        ok &= expectEqual("WeaponMagazineChild3 resolves the magazine display rig anchor",
            resolveStructureAnchor("WeaponMagazineChild3"), StructureAnchor::RigMagazineDisplay);
        ok &= expectEqual("unknown mod-added connect point resolves no anchor",
            resolveStructureAnchor("P-CustomThing"), StructureAnchor::None);
        ok &= expectEqual("plain mesh name resolves no anchor",
            resolveStructureAnchor("AK74M_Body"), StructureAnchor::None);

        const auto otherByName = rock::classifyWeaponPartKind(rock::WeaponPartKind::Other);
        const auto magFromSlot = applyStructureAnchor(otherByName, StructureAnchor::SlotMagazine);
        ok &= expectEqual("magazine slot classifies an unnamed part as magazine",
            magFromSlot.partKind, rock::WeaponPartKind::Magazine);
        ok &= expectEqual("magazine slot classification is slot-sourced",
            magFromSlot.classificationSource, rock::WeaponPartClassificationSource::SlotAnchor);
        ok &= expectEqual("magazine slot carries the vanilla attach-point form id",
            magFromSlot.attachPointFormId, kAttachPointMagazine);

        const auto receiverByWeakToken = rock::classifyWeaponPartKind(rock::WeaponPartKind::Receiver);
        const auto barrelOverride = applyStructureAnchor(receiverByWeakToken, StructureAnchor::SlotBarrel);
        ok &= expectEqual("barrel slot overrides a weak receiver name match",
            barrelOverride.partKind, rock::WeaponPartKind::Barrel);

        const auto slideByName = rock::classifyWeaponPartKind(rock::WeaponPartKind::Slide);
        const auto slideKept = applyStructureAnchor(slideByName, StructureAnchor::RigBolt);
        ok &= expectEqual("action-named part keeps its name under the bolt rig",
            slideKept.partKind, rock::WeaponPartKind::Slide);
        ok &= expectEqual("kept action name stays name-sourced",
            slideKept.classificationSource, rock::WeaponPartClassificationSource::NameToken);
        const auto pumpKept = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Pump), StructureAnchor::SlotHandguard);
        ok &= expectEqual("pump keeps its action role inside the handguard slot",
            pumpKept.partKind, rock::WeaponPartKind::Pump);

        const auto receiverFill = applyStructureAnchor(otherByName, StructureAnchor::SlotReceiver);
        ok &= expectEqual("receiver slot fills unclassified parts",
            receiverFill.partKind, rock::WeaponPartKind::Receiver);
        const auto stockKeptOverReceiver = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Stock), StructureAnchor::SlotReceiver);
        ok &= expectEqual("receiver slot never overrides a critical name match",
            stockKeptOverReceiver.partKind, rock::WeaponPartKind::Stock);

        const auto roundKept = applyStructureAnchor(
            rock::classifyWeaponPartKind(rock::WeaponPartKind::Round), StructureAnchor::RigMagazineDisplay);
        ok &= expectEqual("named ammo round keeps its reload role under the magazine rig",
            roundKept.partKind, rock::WeaponPartKind::Round);
        const auto followerFill = applyStructureAnchor(otherByName, StructureAnchor::RigMagazineDisplay);
        ok &= expectEqual("unnamed magazine-rig part fills as cosmetic ammo",
            followerFill.partKind, rock::WeaponPartKind::CosmeticAmmo);

        const auto noAnchor = applyStructureAnchor(otherByName, StructureAnchor::None);
        ok &= expectEqual("no anchor keeps the name classification",
            noAnchor.partKind, rock::WeaponPartKind::Other);
        ok &= expectEqual("no anchor keeps the name source",
            noAnchor.classificationSource, rock::WeaponPartClassificationSource::NameToken);
    }

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

    // Non-exclusive whitelist targets grant grab modes without activating
    // whitelist gating for everything else.
    std::array<Target, 2> mixedExclusivityTargets{};
    mixedExclusivityTargets[0].active = true;
    mixedExclusivityTargets[0].ownerToken = 30;
    mixedExclusivityTargets[0].flags = MatchActionRole | NonExclusive;
    mixedExclusivityTargets[0].grabMode = GrabMode::AttachOnly;
    mixedExclusivityTargets[0].actionRole = rock::WeaponActionRole::Bolt;

    const auto nonExclusiveBolt = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
            .actionRole = rock::WeaponActionRole::Bolt,
        });
    ok &= expectTrue("non-exclusive bolt target matches bolt contact", nonExclusiveBolt.matched);
    ok &= expectEqual("non-exclusive bolt target grants attach-only", nonExclusiveBolt.grabMode, GrabMode::AttachOnly);
    ok &= expectFalse("non-exclusive target does not activate whitelist gating", nonExclusiveBolt.whitelistActive);

    const auto nonExclusiveMiss = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 43,
            .sourceName = "Receiver",
        });
    ok &= expectFalse("non-bolt contact stays unmatched under non-exclusive target", nonExclusiveMiss.matched);
    ok &= expectFalse("non-bolt contact is not whitelist-gated by non-exclusive target", nonExclusiveMiss.whitelistActive);

    mixedExclusivityTargets[1].active = true;
    mixedExclusivityTargets[1].ownerToken = 31;
    mixedExclusivityTargets[1].flags = MatchBodyId;
    mixedExclusivityTargets[1].grabMode = GrabMode::FullTwoHandAuthority;
    mixedExclusivityTargets[1].bodyId = 77;
    const auto mixedUnmatched = resolveTarget(mixedExclusivityTargets,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 43,
            .sourceName = "Receiver",
        });
    ok &= expectTrue("exclusive target still activates whitelist gating alongside non-exclusive", mixedUnmatched.whitelistActive);
    ok &= expectFalse("mixed whitelist still fails closed for unmatched contact", mixedUnmatched.matched);

    std::array<Target, 1> semanticsOnlyTarget{};
    semanticsOnlyTarget[0].active = true;
    semanticsOnlyTarget[0].ownerToken = 32;
    semanticsOnlyTarget[0].flags = NonExclusive;
    semanticsOnlyTarget[0].grabMode = GrabMode::AttachOnly;
    const auto semanticsOnly = resolveTarget(semanticsOnlyTarget,
        Contact{
            .weaponGenerationKey = 0xABC,
            .bodyId = 42,
            .sourceName = "BoltNode",
        });
    ok &= expectFalse("NonExclusive without a matcher is unusable", semanticsOnly.matched);
    ok &= expectFalse("NonExclusive without a matcher activates nothing", semanticsOnly.whitelistActive);

    {
        using namespace rock::weapon_part_motion_path;

        // A synthetic bolt stroke: rest, pull back 6 units along +Y in steps,
        // hold at peak, return to rest, hold still until completion.
        RecorderState recorder{};
        std::array<PoseSample, kMaxRecordingSamples> buffer{};
        PoseSample rest{};
        rest.translate = Vec3{ 1.0f, 2.0f, 3.0f };

        StepResult lastResult = StepResult::Idle;
        for (std::uint32_t i = 0; i <= kRestStableFramesToArm; ++i) {
            lastResult = step(recorder, buffer.data(), rest, true);
        }
        ok &= expectEqual("motion recorder arms after stable rest", lastResult, StepResult::Armed);

        for (int i = 1; i <= 12; ++i) {
            PoseSample moving = rest;
            moving.translate.y = rest.translate.y + 0.5f * static_cast<float>(i);
            lastResult = step(recorder, buffer.data(), moving, true);
            ok &= expectEqual("motion recorder records the stroke", lastResult, StepResult::RecordingActive);
        }
        PoseSample peak = rest;
        peak.translate.y = rest.translate.y + 6.0f;
        for (std::uint32_t i = 0; i < 3; ++i) {
            lastResult = step(recorder, buffer.data(), peak, true);
        }
        PoseSample returned = rest;
        for (int i = 11; i >= 0; --i) {
            returned.translate.y = rest.translate.y + 0.5f * static_cast<float>(i);
            lastResult = step(recorder, buffer.data(), returned, true);
        }
        for (std::uint32_t i = 0; i < kRestReturnFramesToComplete && lastResult != StepResult::RecordingComplete; ++i) {
            lastResult = step(recorder, buffer.data(), rest, true);
        }
        ok &= expectEqual("motion recorder completes when the part is still again", lastResult, StepResult::RecordingComplete);

        MotionPath path{};
        ok &= expectTrue("completed stroke builds a motion path", buildPathFromRecording(buffer.data(), recorder.sampleCount, path));
        ok &= expectTrue("motion path is valid", path.valid);
        ok &= expectTrue("motion path arc covers the stroke", path.totalArcLength > 5.5f && path.totalArcLength < 6.5f);
        ok &= expectTrue("motion path starts at rest",
            std::abs(path.keys[0].translate.y - rest.translate.y) < 0.05f);
        ok &= expectTrue("motion path is truncated at peak excursion, not the return",
            std::abs(path.keys[kResampledKeyCount - 1].translate.y - peak.translate.y) < 0.30f);

        // Untrusted (driven) frames discard an in-flight recording.
        RecorderState drivenRecorder{};
        for (std::uint32_t i = 0; i <= kRestStableFramesToArm; ++i) {
            (void)step(drivenRecorder, buffer.data(), rest, true);
        }
        PoseSample drivenMove = rest;
        drivenMove.translate.y += 1.0f;
        (void)step(drivenRecorder, buffer.data(), drivenMove, true);
        const auto drivenResult = step(drivenRecorder, buffer.data(), drivenMove, false);
        ok &= expectEqual("driven frame discards in-flight recording", drivenResult, StepResult::RecordingDiscarded);

        // Micro-jitter strokes never become paths.
        std::array<PoseSample, 4> jitter{};
        jitter[0] = rest;
        jitter[1] = rest;
        jitter[1].translate.y += 0.15f;
        jitter[2] = rest;
        jitter[3] = rest;
        MotionPath jitterPath{};
        ok &= expectFalse("stroke below the noise floor builds no path",
            buildPathFromRecording(jitter.data(), static_cast<std::uint32_t>(jitter.size()), jitterPath));

        MotionPath shorterPath = path;
        shorterPath.totalArcLength = path.totalArcLength * 0.5f;
        ok &= expectFalse("shorter stroke does not replace a longer stored path", shouldReplacePath(path, shorterPath));
        MotionPath longerPath = path;
        longerPath.totalArcLength = path.totalArcLength * 1.5f;
        ok &= expectTrue("longer stroke replaces the stored path", shouldReplacePath(path, longerPath));
        ok &= expectTrue("any valid stroke replaces an empty slot", shouldReplacePath(MotionPath{}, path));

        using namespace rock::weapon_part_motion_scrub;
        const auto seededAtRest = initialScrubPosition(path, rest.translate);
        ok &= expectTrue("scrub seeds from the part pose", seededAtRest.valid);
        ok &= expectTrue("scrub seeded at rest starts near arc zero", seededAtRest.arcPosition < 0.5f);
        const auto seededAtPeak = initialScrubPosition(path, peak.translate);
        ok &= expectTrue("scrub seeded at peak lands near full arc",
            seededAtPeak.valid && seededAtPeak.arcPosition > path.totalArcLength - 0.5f);

        // Pulling the hand along the stroke advances the scrub and the target
        // follows the path; the per-frame clamp bounds each step.
        float arc = seededAtRest.arcPosition;
        Vec3 desired = rest.translate;
        desired.y += 3.0f;
        for (int i = 0; i < 8; ++i) {
            const auto result = scrub(path, arc, desired);
            ok &= expectTrue("scrub result stays valid", result.valid);
            ok &= expectTrue("scrub advance respects the per-frame clamp",
                result.arcPosition - arc <= kMaxScrubAdvancePerFrame + 0.001f);
            arc = result.arcPosition;
        }
        ok &= expectTrue("scrub converges to the hand's point on the stroke", std::abs(arc - 3.0f) < 0.35f);
        const auto midTarget = scrub(path, arc, desired);
        ok &= expectTrue("scrub target tracks the path translation",
            std::abs(midTarget.target.translate.y - desired.y) < 0.35f &&
            std::abs(midTarget.target.translate.x - rest.translate.x) < 0.10f);

        // Overshooting the stroke clamps at the path end.
        Vec3 beyond = peak.translate;
        beyond.y += 10.0f;
        for (int i = 0; i < 16; ++i) {
            arc = scrub(path, arc, beyond).arcPosition;
        }
        ok &= expectTrue("scrub clamps at the end of the stroke", arc <= path.totalArcLength + 0.001f);
        ok &= expectTrue("scrub reaches the end of the stroke", arc > path.totalArcLength - 0.35f);
    }

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
