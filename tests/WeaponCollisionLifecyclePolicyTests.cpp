#include <cstdio>

#include "physics-interaction/weapon/WeaponAuthority.h"

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
}

int main()
{
    using namespace rock::weapon_authority_lifecycle_policy;
    using namespace rock::weapon_generated_source_completeness_policy;
    using namespace rock::weapon_instance_visual_witness_policy;
    using namespace rock::weapon_native_visual_remap_policy;
    using rock::WeaponPartKind;

    bool ok = true;

    const auto visualGrace = evaluateEquippedVisualMissing(EquippedVisualMissingInput{
        .hasEquippedIdentity = true,
        .hasExistingBodies = true,
        .missingSeconds = 0.25f,
        .graceSeconds = 0.75f,
    });
    ok &= expectTrue("equipped missing visual remains pending during grace", visualGrace.keepPending);
    ok &= expectTrue("existing bodies disable immediately when visual drive root is absent", visualGrace.disableExistingBodies);
    ok &= expectFalse("visual grace does not warn", visualGrace.warnPastGrace);

    const auto visualExpired = evaluateEquippedVisualMissing(EquippedVisualMissingInput{
        .hasEquippedIdentity = true,
        .hasExistingBodies = true,
        .missingSeconds = 0.80f,
        .graceSeconds = 0.75f,
    });
    ok &= expectTrue("equipped missing visual remains pending after grace", visualExpired.keepPending);
    ok &= expectTrue("existing bodies remain disabled after visual grace", visualExpired.disableExistingBodies);
    ok &= expectFalse("equipped visual absence does not destroy bodies", visualExpired.destroyExistingBodies);

    const auto noIdentity = evaluateEquippedVisualMissing(EquippedVisualMissingInput{
        .hasEquippedIdentity = false,
        .hasExistingBodies = true,
        .missingSeconds = 0.0f,
        .graceSeconds = 0.75f,
    });
    ok &= expectFalse("no equipped identity is not pending", noIdentity.keepPending);
    ok &= expectTrue("no equipped identity destroys existing weapon bodies", noIdentity.destroyExistingBodies);

    GeneratedSourceCompleteness cached{};
    cached.signature = 0x100;
    cached.geometryHash = 0x200;
    cached.boundsExtentScore = 10000;
    cached.sourceCount = 10;
    cached.pointCount = 1000;
    cached.childClusterCount = 10;
    cached.semanticPartMask = partMask(WeaponPartKind::Receiver) | partMask(WeaponPartKind::Barrel);
    cached.gameplayCriticalCount = 2;
    cached.durableSourceCount = 10;
    cached.durableChildClusterCount = 10;
    cached.durablePointCount = 1000;
    cached.durableBoundsExtentScore = 10000;
    cached.durableGeometryHash = 0x300;

    const auto staleVisualSource = evaluateInstanceVisualSync(InstanceVisualSyncInput{
        .hasExistingBodies = true,
        .equippedInstanceChanged = true,
        .settingsChanged = false,
        .hasBuildableSource = true,
        .cachedSource = cached,
        .observedSource = cached,
    });
    ok &= expectTrue("instance change with unchanged visual source is stale", staleVisualSource.staleVisualSource);
    ok &= expectTrue("stale visual source keeps existing bodies", staleVisualSource.keepExistingBodies);
    ok &= expectTrue("stale visual source remains pending", staleVisualSource.pending);
    ok &= expectFalse("unchanged visual source does not force immediate observation loop", staleVisualSource.requestNextObservation);

    const auto amcarBarrelRemap = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xB001,
        .lastRequestedInstanceSignature = 0,
    });
    ok &= expectTrue("AMCAR barrel instance with unchanged visible source requests native visual remap", amcarBarrelRemap.requestRemap);
    ok &= expectTrue("AMCAR stale barrel keeps old colliders active while remap is pending", staleVisualSource.keepExistingBodies);

    const auto amcarBarrelDuplicateFrame = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xB001,
        .lastRequestedInstanceSignature = 0xB001,
    });
    ok &= expectFalse("duplicate stale AMCAR barrel frames do not spam native remap requests", amcarBarrelDuplicateFrame.requestRemap);

    auto queuedBarrelAttempt = recordNativeVisualRemapQueued(NativeVisualRemapAttemptState{}, 0xB001);
    const auto amcarBarrelAwaitingNativeResult = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xB001,
        .attemptState = queuedBarrelAttempt,
    });
    ok &= expectFalse("queued AMCAR barrel remap waits for the native task result before retrying", amcarBarrelAwaitingNativeResult.requestRemap);

    queuedBarrelAttempt = observeNativeVisualRemapStillStale(queuedBarrelAttempt, 0xB001);
    queuedBarrelAttempt = observeNativeVisualRemapStillStale(queuedBarrelAttempt, 0xB001);
    const auto amcarBarrelQueuedRetry = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xB001,
        .attemptState = queuedBarrelAttempt,
    });
    ok &= expectTrue("stale AMCAR barrel after queued observation window gets one bounded native retry", amcarBarrelQueuedRetry.requestRemap);

    auto failedAcquireAttempt = recordNativeVisualRemapAcquireFailed(NativeVisualRemapAttemptState{}, 0xB002);
    const auto amcarAcquireBackoff = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xB002,
        .attemptState = failedAcquireAttempt,
    });
    ok &= expectFalse("failed AMCAR remap candidate acquisition backs off instead of retrying every frame", amcarAcquireBackoff.requestRemap);

    for (std::uint32_t i = 0; i < 4; ++i) {
        failedAcquireAttempt = observeNativeVisualRemapStillStale(failedAcquireAttempt, 0xB002);
    }
    const auto amcarAcquireRetry = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xB002,
        .attemptState = failedAcquireAttempt,
    });
    ok &= expectTrue("failed AMCAR remap candidate acquisition retries after a bounded evidence window", amcarAcquireRetry.requestRemap);

    const auto amcarStockRemap = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xC001,
        .lastRequestedInstanceSignature = 0xB001,
    });
    ok &= expectTrue("AMCAR stock instance after barrel requests a new native visual remap", amcarStockRemap.requestRemap);

    const auto amcarSightBank20Remap = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xD001,
        .lastRequestedInstanceSignature = 0xC001,
    });
    ok &= expectTrue("AMCAR sight as last manual mod triggers remap instead of publishing stale bank 20", amcarSightBank20Remap.requestRemap);

    const auto pipboyMissingVisualNoRemap = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = false,
        .staleVisualSource = false,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xD101,
        .lastRequestedInstanceSignature = 0xD001,
    });
    ok &= expectFalse("Pip-Boy changed instance with missing first-person visual node does not trigger native visual remap", pipboyMissingVisualNoRemap.requestRemap);

    auto priorStaleVisibleAttempt = recordNativeVisualRemapQueued(NativeVisualRemapAttemptState{}, 0xD001);
    priorStaleVisibleAttempt = observeNativeVisualRemapStillStale(priorStaleVisibleAttempt, 0xD001);
    priorStaleVisibleAttempt = observeNativeVisualRemapStillStale(priorStaleVisibleAttempt, 0xD001);
    const auto pipboyMissingVisualDoesNotRetry = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = false,
        .staleVisualSource = false,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xD202,
        .lastRequestedInstanceSignature = 0xD001,
        .attemptState = priorStaleVisibleAttempt,
    });
    ok &= expectFalse(
        "Pip-Boy missing first-person visual root cannot reuse stale-visible retry state as native remap evidence",
        pipboyMissingVisualDoesNotRetry.requestRemap);

    const auto sameFormPointerOnlyRemapWitnessChanged = sameFormEquippedInstanceRemapWitnessChanged(EquippedInstanceRemapWitnessInput{
        .cachedInstanceSignature = 0xC001,
        .pendingInstanceSignature = 0xC001,
        .cachedFormID = 0x06033B60,
        .pendingFormID = 0x06033B60,
        .cachedInstanceDataAddress = 0x2000,
        .pendingInstanceDataAddress = 0x2100,
        .cachedObjectInstanceExtraAddress = 0x3000,
        .pendingObjectInstanceExtraAddress = 0x3100,
    });
    ok &= expectTrue(
        "same-form native remap witness changes when instance data or object extra changes even if content signature is stable",
        sameFormPointerOnlyRemapWitnessChanged);

    const auto differentFormPointerOnlyRemapWitnessChanged = sameFormEquippedInstanceRemapWitnessChanged(EquippedInstanceRemapWitnessInput{
        .cachedInstanceSignature = 0xC001,
        .pendingInstanceSignature = 0xC001,
        .cachedFormID = 0x06033B60,
        .pendingFormID = 0x000DF42E,
        .cachedInstanceDataAddress = 0x2000,
        .pendingInstanceDataAddress = 0x2100,
        .cachedObjectInstanceExtraAddress = 0x3000,
        .pendingObjectInstanceExtraAddress = 0x3100,
    });
    ok &= expectFalse(
        "different-form native remap witness changes do not enter same-form stale-visible remap",
        differentFormPointerOnlyRemapWitnessChanged);

    const auto amcarMissingVisualReturnedCached = evaluateReturnedCachedVisualPending(ReturnedCachedVisualPendingInput{
        .hasExistingBodies = true,
        .settingsChanged = false,
        .cachedKey = 0xA001,
        .currentKey = 0xA001,
        .pendingKey = 0xB001,
        .cachedInstanceSignature = 0xC001,
        .pendingInstanceSignature = 0xC001,
        .cachedFormID = 0x06033B60,
        .pendingFormID = 0x06033B60,
        .cachedInstanceDataAddress = 0x2000,
        .pendingInstanceDataAddress = 0x2100,
        .cachedObjectInstanceExtraAddress = 0x3000,
        .pendingObjectInstanceExtraAddress = 0x3100,
    });
    ok &= expectFalse("AMCAR missing visual return does not cancel same-form changed native remap witness", amcarMissingVisualReturnedCached.cancelPending);
    ok &= expectTrue(
        "AMCAR missing visual return keeps pending native remap witness so stale-visible remap can run",
        amcarMissingVisualReturnedCached.keepPendingForStaleVisibleCheck);

    const auto pipboyDifferentWeaponReturnedCached = evaluateReturnedCachedVisualPending(ReturnedCachedVisualPendingInput{
        .hasExistingBodies = true,
        .settingsChanged = false,
        .cachedKey = 0xA001,
        .currentKey = 0xA001,
        .pendingKey = 0xB001,
        .cachedInstanceSignature = 0xC001,
        .pendingInstanceSignature = 0xC002,
        .cachedFormID = 0x06033B60,
        .pendingFormID = 0x000DF42E,
    });
    ok &= expectTrue("Pip-Boy different weapon returning cached visual cancels pending ROCK remap state", pipboyDifferentWeaponReturnedCached.cancelPending);
    ok &= expectFalse(
        "Pip-Boy different weapon returning cached visual does not enter stale-visible remap",
        pipboyDifferentWeaponReturnedCached.keepPendingForStaleVisibleCheck);

    const auto transientMissingVisualReturnedCached = evaluateReturnedCachedVisualPending(ReturnedCachedVisualPendingInput{
        .hasExistingBodies = true,
        .settingsChanged = false,
        .cachedKey = 0xA001,
        .currentKey = 0xA001,
        .pendingKey = 0xB001,
        .cachedInstanceSignature = 0xC001,
        .pendingInstanceSignature = 0xC001,
        .cachedFormID = 0x06033B60,
        .pendingFormID = 0x06033B60,
    });
    ok &= expectTrue("same equipped instance returning cached visual remains a transient cancel", transientMissingVisualReturnedCached.cancelPending);
    ok &= expectFalse(
        "same equipped instance returning cached visual does not request stale-visible remap",
        transientMissingVisualReturnedCached.keepPendingForStaleVisibleCheck);

    auto exhaustedSightAttempt = recordNativeVisualRemapQueued(NativeVisualRemapAttemptState{}, 0xD001);
    exhaustedSightAttempt = observeNativeVisualRemapStillStale(exhaustedSightAttempt, 0xD001);
    exhaustedSightAttempt = observeNativeVisualRemapStillStale(exhaustedSightAttempt, 0xD001);
    exhaustedSightAttempt = recordNativeVisualRemapQueued(exhaustedSightAttempt, 0xD001);
    exhaustedSightAttempt = observeNativeVisualRemapStillStale(exhaustedSightAttempt, 0xD001);
    exhaustedSightAttempt = observeNativeVisualRemapStillStale(exhaustedSightAttempt, 0xD001);
    const auto amcarSightExhausted = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = true,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xD001,
        .attemptState = exhaustedSightAttempt,
    });
    ok &= expectFalse("AMCAR sight remap stops after bounded queued attempts instead of looping forever", amcarSightExhausted.requestRemap);

    const auto amcarRemapDisabled = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = false,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xE001,
        .lastRequestedInstanceSignature = 0,
    });
    ok &= expectFalse("native visual remap kill switch suppresses queued remap", amcarRemapDisabled.requestRemap);

    const auto inventoryEquipWitnessBlocked = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = false,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xF001,
        .lastRequestedInstanceSignature = 0,
    });
    ok &= expectFalse("inventory equip witness cannot authorize native visual remap", inventoryEquipWitnessBlocked.requestRemap);

    const auto workbenchWitnessAuthorized = evaluateNativeVisualRemap(NativeVisualRemapInput{
        .enabled = true,
        .authorizedWitness = true,
        .staleVisualSource = staleVisualSource.staleVisualSource,
        .equippedInstanceChanged = true,
        .pendingInstanceSignature = 0xF002,
        .lastRequestedInstanceSignature = 0,
    });
    ok &= expectTrue("workbench direct equipped-stack witness authorizes native visual remap", workbenchWitnessAuthorized.requestRemap);

    const NativeVisualRemapTargetWitness expectedRemapTarget{
        .formID = 0x01020304,
        .formAddress = 0x1000,
        .instanceDataAddress = 0x2000,
        .objectInstanceExtraAddress = 0x3000,
    };
    const auto exactRemapTarget = evaluateNativeVisualRemapTargetMatch(expectedRemapTarget, expectedRemapTarget);
    ok &= expectTrue("native visual remap queues only the matching equipped instance witness", exactRemapTarget.matches);

    auto mismatchedFormTarget = expectedRemapTarget;
    mismatchedFormTarget.formID = 0x01020305;
    ok &= expectFalse("native visual remap rejects a different equipped weapon form",
        evaluateNativeVisualRemapTargetMatch(expectedRemapTarget, mismatchedFormTarget).matches);

    auto mismatchedInstanceTarget = expectedRemapTarget;
    mismatchedInstanceTarget.instanceDataAddress = 0x2008;
    ok &= expectFalse("native visual remap rejects a different equipped instance-data pointer",
        evaluateNativeVisualRemapTargetMatch(expectedRemapTarget, mismatchedInstanceTarget).matches);

    auto mismatchedExtraTarget = expectedRemapTarget;
    mismatchedExtraTarget.objectInstanceExtraAddress = 0x3008;
    ok &= expectFalse("native visual remap rejects a different object-instance extra pointer",
        evaluateNativeVisualRemapTargetMatch(expectedRemapTarget, mismatchedExtraTarget).matches);

    auto expectedWithoutExtraEvidence = expectedRemapTarget;
    expectedWithoutExtraEvidence.objectInstanceExtraAddress = 0;
    ok &= expectTrue("native visual remap does not reject when expected object-instance extra evidence is unavailable",
        evaluateNativeVisualRemapTargetMatch(expectedWithoutExtraEvidence, expectedRemapTarget).matches);

    auto jitteredVisualSource = cached;
    jitteredVisualSource.geometryHash = 0x211;
    jitteredVisualSource.durableGeometryHash = 0x311;
    jitteredVisualSource.pointCount = cached.pointCount - 10;
    jitteredVisualSource.durablePointCount = cached.durablePointCount - 10;
    jitteredVisualSource.boundsExtentScore = cached.boundsExtentScore + 50;
    jitteredVisualSource.durableBoundsExtentScore = cached.durableBoundsExtentScore + 50;
    const auto staleJitteredVisualSource = evaluateInstanceVisualSync(InstanceVisualSyncInput{
        .hasExistingBodies = true,
        .equippedInstanceChanged = true,
        .settingsChanged = false,
        .hasBuildableSource = true,
        .cachedSource = cached,
        .observedSource = jitteredVisualSource,
    });
    ok &= expectTrue("same visual structure with geometry jitter is still stale", staleJitteredVisualSource.staleVisualSource);

    auto materialVisualSource = cached;
    materialVisualSource.geometryHash = 0x212;
    materialVisualSource.durableGeometryHash = 0x312;
    materialVisualSource.pointCount = cached.pointCount + 120;
    materialVisualSource.durablePointCount = cached.durablePointCount + 120;
    materialVisualSource.boundsExtentScore = cached.boundsExtentScore + 400;
    materialVisualSource.durableBoundsExtentScore = cached.durableBoundsExtentScore + 400;
    const auto freshMaterialVisualSource = evaluateInstanceVisualSync(InstanceVisualSyncInput{
        .hasExistingBodies = true,
        .equippedInstanceChanged = true,
        .settingsChanged = false,
        .hasBuildableSource = true,
        .cachedSource = cached,
        .observedSource = materialVisualSource,
    });
    ok &= expectFalse("same structure with material geometry growth is not stale", freshMaterialVisualSource.staleVisualSource);
    ok &= expectTrue("same structure material growth can replace", sourceSetHasMaterialDurableGeometryChange(cached, materialVisualSource));
    ok &= expectTrue("same structure material growth is a durable geometry difference",
        sourceSetHasMaterialDurableGeometryDifference(cached, materialVisualSource));

    auto smallerMaterialVisualSource = cached;
    smallerMaterialVisualSource.geometryHash = 0x213;
    smallerMaterialVisualSource.durableGeometryHash = 0x313;
    smallerMaterialVisualSource.pointCount = cached.pointCount - 120;
    smallerMaterialVisualSource.durablePointCount = cached.durablePointCount - 120;
    smallerMaterialVisualSource.boundsExtentScore = cached.boundsExtentScore - 400;
    smallerMaterialVisualSource.durableBoundsExtentScore = cached.durableBoundsExtentScore - 400;
    const auto freshSmallerVisualSource = evaluateInstanceVisualSync(InstanceVisualSyncInput{
        .hasExistingBodies = true,
        .equippedInstanceChanged = true,
        .settingsChanged = false,
        .hasBuildableSource = true,
        .cachedSource = cached,
        .observedSource = smallerMaterialVisualSource,
    });
    ok &= expectFalse("same structure with smaller material geometry is not stale", freshSmallerVisualSource.staleVisualSource);
    ok &= expectFalse("same structure material shrink is not an enrichment growth", sourceSetHasMaterialDurableGeometryChange(cached, smallerMaterialVisualSource));
    ok &= expectTrue("same structure material shrink is a durable geometry difference",
        sourceSetHasMaterialDurableGeometryDifference(cached, smallerMaterialVisualSource));
    ok &= expectTrue("same structure material shrink uses geometry settle key",
        makeGeneratedSourceReplacementSettleKey(cached, smallerMaterialVisualSource) != smallerMaterialVisualSource.signature);
    ok &= expectTrue("same-owner pending material replacement keeps geometry settle key",
        makeGeneratedSourcePendingSettleKey(cached, smallerMaterialVisualSource, false, false) != smallerMaterialVisualSource.signature);

    auto ownerChangedHeavySource = cached;
    ownerChangedHeavySource.signature = 0x711;
    ownerChangedHeavySource.geometryHash = 0x811;
    ownerChangedHeavySource.durableGeometryHash = 0x911;
    ownerChangedHeavySource.sourceCount = 17;
    ownerChangedHeavySource.pointCount = 886;
    ownerChangedHeavySource.childClusterCount = 42;
    ownerChangedHeavySource.durableSourceCount = 17;
    ownerChangedHeavySource.durablePointCount = 886;
    ownerChangedHeavySource.durableChildClusterCount = 42;
    auto ownerChangedHeavySourceJitter = ownerChangedHeavySource;
    ownerChangedHeavySourceJitter.geometryHash = 0x812;
    ownerChangedHeavySourceJitter.durableGeometryHash = 0x912;
    ownerChangedHeavySourceJitter.pointCount = 899;
    ownerChangedHeavySourceJitter.durablePointCount = 899;
    ownerChangedHeavySourceJitter.boundsExtentScore += 25;
    ownerChangedHeavySourceJitter.durableBoundsExtentScore += 25;
    ok &= expectTrue("owner-changed pending source uses structural settle key",
        makeGeneratedSourcePendingSettleKey(cached, ownerChangedHeavySource, true, false) == ownerChangedHeavySource.signature);
    ok &= expectTrue("owner-changed pending source ignores skinned geometry jitter in settle key",
        makeGeneratedSourcePendingSettleKey(cached, ownerChangedHeavySource, true, false) ==
            makeGeneratedSourcePendingSettleKey(cached, ownerChangedHeavySourceJitter, true, false));

    auto grownVisualSource = cached;
    grownVisualSource.signature = 0x101;
    grownVisualSource.geometryHash = 0x201;
    grownVisualSource.durableGeometryHash = 0x301;
    grownVisualSource.sourceCount = cached.sourceCount + 1;
    grownVisualSource.pointCount = cached.pointCount + 160;
    grownVisualSource.durableSourceCount = cached.durableSourceCount + 1;
    grownVisualSource.durablePointCount = cached.durablePointCount + 160;
    grownVisualSource.boundsExtentScore = cached.boundsExtentScore + 500;
    grownVisualSource.durableBoundsExtentScore = cached.durableBoundsExtentScore + 500;
    const auto freshVisualSource = evaluateInstanceVisualSync(InstanceVisualSyncInput{
        .hasExistingBodies = true,
        .equippedInstanceChanged = true,
        .settingsChanged = false,
        .hasBuildableSource = true,
        .cachedSource = cached,
        .observedSource = grownVisualSource,
    });
    ok &= expectFalse("instance change with grown visual source is not stale", freshVisualSource.staleVisualSource);
    ok &= expectTrue("grown visual source can still drive replacement", sourceSetImproved(cached, grownVisualSource));

    const auto missingVisualSource = evaluateInstanceVisualSync(InstanceVisualSyncInput{
        .hasExistingBodies = true,
        .equippedInstanceChanged = true,
        .settingsChanged = false,
        .hasBuildableSource = false,
        .cachedSource = cached,
        .observedSource = GeneratedSourceCompleteness{},
    });
    ok &= expectTrue("instance change with no buildable visual source stays pending", missingVisualSource.staleVisualSource);
    ok &= expectTrue("no buildable source requests another observation", missingVisualSource.requestNextObservation);

    const auto settingsOverride = evaluateInstanceVisualSync(InstanceVisualSyncInput{
        .hasExistingBodies = true,
        .equippedInstanceChanged = true,
        .settingsChanged = true,
        .hasBuildableSource = true,
        .cachedSource = cached,
        .observedSource = cached,
    });
    ok &= expectFalse("settings rebuild is not blocked by stale visual witness", settingsOverride.staleVisualSource);

    auto observed = cached;
    observed.geometryHash = 0x201;
    observed.durableGeometryHash = 0x301;
    observed.boundsExtentScore = 10300;
    observed.durableBoundsExtentScore = 10300;
    observed.pointCount = 980;
    observed.durablePointCount = 980;
    ok &= expectTrue("same-structure material bounds growth triggers late enrichment", sourceSetImproved(cached, observed));
    const auto boundsReplacement = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = true,
        .ownerIdentityChanged = false,
        .settingsChanged = false,
        .cached = cached,
        .observed = observed,
    });
    ok &= expectTrue("same-signature material bounds growth reaches explicit replacement policy", boundsReplacement.allowed);
    ok &= expectTrue("same-signature material replacement uses geometry settle key",
        makeGeneratedSourceReplacementSettleKey(cached, observed) != observed.signature);

    observed = cached;
    observed.geometryHash = 0x202;
    observed.durableGeometryHash = 0x302;
    observed.boundsExtentScore = 10100;
    observed.durableBoundsExtentScore = 10100;
    observed.pointCount = 990;
    observed.durablePointCount = 990;
    ok &= expectFalse("small skinned-geometry jitter does not force enrichment", sourceSetImproved(cached, observed));

    observed = cached;
    observed.geometryHash = 0x203;
    observed.durableGeometryHash = 0x303;
    observed.boundsExtentScore = 10000;
    observed.durableBoundsExtentScore = 10000;
    observed.pointCount = 1100;
    observed.durablePointCount = 1100;
    ok &= expectTrue("same-structure material point growth triggers late enrichment", sourceSetImproved(cached, observed));

    observed = cached;
    observed.geometryHash = 0x204;
    observed.durableGeometryHash = 0x304;
    observed.boundsExtentScore = 12000;
    observed.durableBoundsExtentScore = 12000;
    observed.pointCount = 1200;
    observed.durablePointCount = 1200;
    observed.semanticPartMask = partMask(WeaponPartKind::Receiver);
    observed.gameplayCriticalCount = 1;
    ok &= expectTrue("same-structure material geometry change is not blocked by semantic loss", sourceSetImproved(cached, observed));

    observed = cached;
    observed.signature = 0x305;
    observed.geometryHash = 0x405;
    observed.durableGeometryHash = 0x505;
    observed.boundsExtentScore = 12000;
    observed.durableBoundsExtentScore = 12000;
    observed.pointCount = 1200;
    observed.durablePointCount = 1200;
    observed.semanticPartMask = partMask(WeaponPartKind::Receiver) | partMask(WeaponPartKind::Barrel) | partMask(WeaponPartKind::Stock);
    observed.gameplayCriticalCount = 3;
    auto cachedWithMagazine = cached;
    cachedWithMagazine.semanticPartMask |= partMask(WeaponPartKind::Magazine);
    cachedWithMagazine.gameplayCriticalCount = 3;
    ok &= expectTrue("visible source package growth can replace without semantic permanence gate", sourceSetImproved(cachedWithMagazine, observed));
    const auto swappedPermanentPartReplacement = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = true,
        .ownerIdentityChanged = false,
        .settingsChanged = false,
        .cached = cachedWithMagazine,
        .observed = observed,
    });
    ok &= expectTrue("same-owner swapped visible parts are allowed", swappedPermanentPartReplacement.allowed);
    ok &= expectTrue("same-owner swapped permanent parts keep existing bodies", swappedPermanentPartReplacement.keepExistingBodies);
    ok &= expectFalse("same-owner swapped visible parts do not request another observation", swappedPermanentPartReplacement.pending);

    observed = cached;
    observed.signature = 0x400;
    observed.geometryHash = 0x401;
    observed.boundsExtentScore = 12000;
    observed.pointCount = 1200;
    observed.transientReloadSourceCount = 2;
    observed.durableSourceCount = cached.durableSourceCount;
    observed.durableChildClusterCount = cached.durableChildClusterCount;
    observed.durablePointCount = cached.durablePointCount;
    observed.durableBoundsExtentScore = cached.durableBoundsExtentScore;
    observed.durableGeometryHash = cached.durableGeometryHash;
    ok &= expectFalse("transient-only reload source growth does not trigger late enrichment", sourceSetImproved(cached, observed));
    const auto transientReplacement = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = true,
        .ownerIdentityChanged = false,
        .settingsChanged = false,
        .cached = cached,
        .observed = observed,
    });
    ok &= expectTrue("transient-only reload geometry growth can trigger explicit replacement", transientReplacement.allowed);
    ok &= expectFalse("transient-only reload geometry growth does not adopt cached bodies", transientReplacement.adoptExistingBodies);

    GeneratedSourceCompleteness amcarStockNoBarrel{};
    amcarStockNoBarrel.signature = 0xA100;
    amcarStockNoBarrel.sourceCount = 9;
    amcarStockNoBarrel.pointCount = 3845;
    amcarStockNoBarrel.childClusterCount = 28;
    amcarStockNoBarrel.semanticPartMask =
        partMask(WeaponPartKind::Receiver) |
        partMask(WeaponPartKind::Stock) |
        partMask(WeaponPartKind::Grip) |
        partMask(WeaponPartKind::Magazine) |
        partMask(WeaponPartKind::Bolt) |
        partMask(WeaponPartKind::ChargingHandle) |
        partMask(WeaponPartKind::Sight);
    amcarStockNoBarrel.gameplayCriticalCount = 6;
    amcarStockNoBarrel.durableSourceCount = 9;
    amcarStockNoBarrel.durableChildClusterCount = 28;
    amcarStockNoBarrel.durablePointCount = 3845;
    amcarStockNoBarrel = withDerivedPackageCoverage(amcarStockNoBarrel);

    GeneratedSourceCompleteness amcarBarrelNoStock = amcarStockNoBarrel;
    amcarBarrelNoStock.signature = 0xA200;
    amcarBarrelNoStock.sourceCount = 7;
    amcarBarrelNoStock.semanticPartMask =
        partMask(WeaponPartKind::Receiver) |
        partMask(WeaponPartKind::Barrel) |
        partMask(WeaponPartKind::Magazine) |
        partMask(WeaponPartKind::Bolt) |
        partMask(WeaponPartKind::ChargingHandle) |
        partMask(WeaponPartKind::Sight);
    amcarBarrelNoStock.gameplayCriticalCount = 5;
    amcarBarrelNoStock = withDerivedPackageCoverage(amcarBarrelNoStock);

    ok &= expectTrue("AMCAR stock/no-barrel package is firearm-like", amcarStockNoBarrel.firearmLikePackage);
    ok &= expectFalse("AMCAR stock/no-barrel package does not block on front coverage",
        (amcarStockNoBarrel.missingRequiredPackageCoverageMask & kMissingFrontPackageCoverage) != 0);
    ok &= expectTrue("AMCAR barrel/no-stock package is firearm-like", amcarBarrelNoStock.firearmLikePackage);
    ok &= expectFalse("AMCAR barrel/no-stock package does not block on rear coverage",
        (amcarBarrelNoStock.missingRequiredPackageCoverageMask & kMissingRearPackageCoverage) != 0);
    ok &= expectFalse("AMCAR barrel/no-stock package does not accept pistol grip as long-gun rear coverage",
        amcarBarrelNoStock.hasRequiredRearCoverage);

    const auto amcarOwnerChangedMissingRear = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = true,
        .ownerIdentityChanged = true,
        .settingsChanged = false,
        .cached = amcarStockNoBarrel,
        .observed = amcarBarrelNoStock,
    });
    ok &= expectTrue("owner change publishes AMCAR barrel/no-stock replacement", amcarOwnerChangedMissingRear.allowed);
    ok &= expectTrue("AMCAR missing rear replacement keeps old bodies active", amcarOwnerChangedMissingRear.keepExistingBodies);
    ok &= expectFalse("AMCAR missing rear replacement does not remain pending", amcarOwnerChangedMissingRear.pending);

    const auto amcarOwnerChangedMissingFront = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = true,
        .ownerIdentityChanged = true,
        .settingsChanged = false,
        .cached = amcarBarrelNoStock,
        .observed = amcarStockNoBarrel,
    });
    ok &= expectTrue("owner change publishes AMCAR stock/no-barrel replacement", amcarOwnerChangedMissingFront.allowed);
    ok &= expectTrue("AMCAR missing front replacement keeps old bodies active", amcarOwnerChangedMissingFront.keepExistingBodies);
    ok &= expectFalse("AMCAR missing front replacement does not remain pending", amcarOwnerChangedMissingFront.pending);

    const auto amcarInitialIncompleteWaiting = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = false,
        .ownerIdentityChanged = false,
        .settingsChanged = false,
        .allowIncompletePackageWithoutExistingBodies = false,
        .observed = amcarStockNoBarrel,
    });
    ok &= expectTrue("initial AMCAR package publishes without failed-open fallback", amcarInitialIncompleteWaiting.allowed);
    ok &= expectFalse("initial AMCAR package does not remain pending for semantic coverage", amcarInitialIncompleteWaiting.pending);

    const auto amcarInitialIncompleteFailedOpen = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = false,
        .ownerIdentityChanged = false,
        .settingsChanged = false,
        .allowIncompletePackageWithoutExistingBodies = true,
        .observed = amcarStockNoBarrel,
    });
    ok &= expectTrue("initial AMCAR package publishes regardless of old failed-open flag", amcarInitialIncompleteFailedOpen.allowed);

    GeneratedSourceCompleteness fullLongGun = amcarStockNoBarrel;
    fullLongGun.signature = 0xA300;
    fullLongGun.semanticPartMask |= partMask(WeaponPartKind::Barrel);
    fullLongGun.gameplayCriticalCount = 7;
    fullLongGun = withDerivedPackageCoverage(fullLongGun);

    GeneratedSourceCompleteness longGunGripNoStock = fullLongGun;
    longGunGripNoStock.signature = 0xA400;
    longGunGripNoStock.semanticPartMask &= ~partMask(WeaponPartKind::Stock);
    longGunGripNoStock = withDerivedPackageCoverage(longGunGripNoStock);
    ok &= expectFalse("long-gun package with pistol grip but no stock does not need semantic rear coverage",
        sourcePackageNeedsMoreCoverage(longGunGripNoStock));

    const auto expectedStockRegression = evaluateGeneratedSourceReplacement(GeneratedSourceReplacementInput{
        .hasExistingBodies = true,
        .ownerIdentityChanged = false,
        .settingsChanged = false,
        .expected = fullLongGun,
        .cached = fullLongGun,
        .observed = longGunGripNoStock,
    });
    ok &= expectTrue("expected long-gun stock coverage does not block visible grip-only package", expectedStockRegression.allowed);
    ok &= expectFalse("expected stock regression reports no blocking missing rear coverage",
        (expectedStockRegression.missingRequiredPackageCoverageMask & kMissingRearPackageCoverage) != 0);

    GeneratedSourceCompleteness compactPistol{};
    compactPistol.signature = 0xB100;
    compactPistol.sourceCount = 5;
    compactPistol.pointCount = 900;
    compactPistol.semanticPartMask =
        partMask(WeaponPartKind::Receiver) |
        partMask(WeaponPartKind::Barrel) |
        partMask(WeaponPartKind::Magazine) |
        partMask(WeaponPartKind::Slide) |
        partMask(WeaponPartKind::Sight);
    compactPistol = withDerivedPackageCoverage(compactPistol);
    ok &= expectTrue("compact pistol package is firearm-like", compactPistol.firearmLikePackage);
    ok &= expectFalse("compact pistol package does not require stock/grip rear coherence",
        sourcePackageNeedsMoreCoverage(compactPistol));

    GeneratedSourceSettleState settle{};
    ok &= expectFalse("first source observation is not settled", advanceGeneratedSourceSettle(settle, 0xAA).settled);
    ok &= expectFalse("second source observation is not settled", advanceGeneratedSourceSettle(settle, 0xAA).settled);
    ok &= expectTrue("third source observation is settled", advanceGeneratedSourceSettle(settle, 0xAA).settled);
    ok &= expectFalse("zero source observation resets settle", advanceGeneratedSourceSettle(settle, 0).settled);

    return ok ? 0 : 1;
}
