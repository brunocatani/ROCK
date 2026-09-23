#include <cstdint>
#include <cstdio>
#include <limits>

#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponEffectGeometryPolicy.h"
#include "physics-interaction/weapon/WeaponMaterialVisibilityPolicy.h"
#include "physics-interaction/weapon/WeaponEmitterPolicy.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"

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

    bool expectNonZero(const char* label, std::uint64_t value)
    {
        if (value != 0) {
            return true;
        }

        std::printf("%s expected nonzero\n", label);
        return false;
    }

    bool expectDifferent(const char* label, std::uint64_t lhs, std::uint64_t rhs)
    {
        if (lhs != rhs) {
            return true;
        }

        std::printf("%s expected different values\n", label);
        return false;
    }

    bool expectSame(const char* label, std::uint64_t lhs, std::uint64_t rhs)
    {
        if (lhs == rhs) {
            return true;
        }

        std::printf("%s expected same values\n", label);
        return false;
    }
}

int main()
{
    using namespace rock::weapon_authority_lifecycle_policy;
    using namespace rock::weapon_generated_source_completeness_policy;
    using namespace rock::weapon_generation_identity_policy;
    using namespace rock::weapon_effect_geometry_policy;
    namespace emitter = rock::weapon_emitter_policy;
    using rock::WeaponPartKind;

    bool ok = true;

    ok &= expectFalse("normal update keeps weapon authority", shouldClearWeaponAuthorityForUpdateInterruption(false, false, false));
    ok &= expectTrue("menu interruption clears weapon authority", shouldClearWeaponAuthorityForUpdateInterruption(true, false, false));
    ok &= expectTrue("ROCK disable clears weapon authority", shouldClearWeaponAuthorityForUpdateInterruption(false, true, false));
    ok &= expectTrue("missing skeleton clears weapon authority", shouldClearWeaponAuthorityForUpdateInterruption(false, false, true));

    ok &= expectTrue("legacy zero contact generation remains usable", isWeaponContactGenerationCurrent(0, 0x10));
    ok &= expectTrue("current generation contact remains usable", isWeaponContactGenerationCurrent(0x10, 0x10));
    ok &= expectFalse("stale generated contact is rejected", isWeaponContactGenerationCurrent(0x10, 0x20));

    ok &= expectNonZero("receiver part mask is valid", partMask(WeaponPartKind::Receiver));
    ok &= expectNonZero("stock part mask is valid", partMask(WeaponPartKind::Stock));
    ok &= expectFalse("out-of-range part mask is zero", partMask(static_cast<WeaponPartKind>(31)) != 0);

    GeneratedSourceCompleteness receiverBarrelGrip{};
    receiverBarrelGrip.signature = 0x100;
    receiverBarrelGrip.geometryHash = 0x200;
    receiverBarrelGrip.boundsExtentScore = 1000;
    receiverBarrelGrip.sourceCount = 3;
    receiverBarrelGrip.pointCount = 300;
    receiverBarrelGrip.childClusterCount = 3;
    receiverBarrelGrip.semanticPartMask =
        partMask(WeaponPartKind::Receiver) |
        partMask(WeaponPartKind::Barrel) |
        partMask(WeaponPartKind::Grip);
    receiverBarrelGrip.gameplayCriticalCount = 3;

    const auto derivedCompact = withDerivedPackageCoverage(receiverBarrelGrip);
    ok &= expectTrue("front coverage derives from barrel", derivedCompact.hasRequiredFrontCoverage);
    ok &= expectTrue("compact rear coverage accepts grip", derivedCompact.hasRequiredRearCoverage);
    ok &= expectTrue("receiver/action package is firearm-like telemetry", derivedCompact.firearmLikePackage);
    ok &= expectFalse("package coverage telemetry no longer gates collision", derivedCompact.missingRequiredPackageCoverageMask != 0);

    GeneratedSourceCompleteness longGunWithoutStock = receiverBarrelGrip;
    longGunWithoutStock.semanticPartMask |= partMask(WeaponPartKind::Handguard);
    const auto longGunCoverage = withDerivedPackageCoverage(longGunWithoutStock);
    ok &= expectFalse("long-gun rear coverage requires stock", hasLongGunRearPackageCoverage(longGunCoverage));
    ok &= expectFalse("long-gun package with only grip lacks required rear telemetry", longGunCoverage.hasRequiredRearCoverage);

    GeneratedSourceCompleteness receiverMuzzleGrip = receiverBarrelGrip;
    receiverMuzzleGrip.semanticPartMask =
        partMask(WeaponPartKind::Receiver) |
        partMask(WeaponPartKind::MuzzleDevice) |
        partMask(WeaponPartKind::Grip);
    ok &= expectTrue("muzzle device preserves structural front-coverage telemetry",
        withDerivedPackageCoverage(receiverMuzzleGrip).hasRequiredFrontCoverage);

    ok &= expectTrue("shell is transient reload geometry", isTransientReloadPart(WeaponPartKind::Shell));
    ok &= expectTrue("round is transient reload geometry", isTransientReloadPart(WeaponPartKind::Round));
    ok &= expectTrue("cosmetic ammo is transient reload geometry", isTransientReloadPart(WeaponPartKind::CosmeticAmmo));
    ok &= expectFalse("magazine is durable weapon structure", isTransientReloadPart(WeaponPartKind::Magazine));
    ok &= expectTrue("receiver is permanent gameplay-critical structure", (permanentGameplayCriticalPartMask() & partMask(WeaponPartKind::Receiver)) != 0);
    ok &= expectTrue("muzzle device preserves permanent barrel-like structure", (permanentGameplayCriticalPartMask() & partMask(WeaponPartKind::MuzzleDevice)) != 0);
    ok &= expectTrue("bipod remains durable weapon structure independent of deployment state", (permanentGameplayCriticalPartMask() & partMask(WeaponPartKind::Bipod)) != 0);
    ok &= expectFalse("shell is not permanent gameplay-critical structure", (permanentGameplayCriticalPartMask() & partMask(WeaponPartKind::Shell)) != 0);

    using rock::native_scope_sight_anchor_policy::PublicationIdentity;
    using rock::native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon;
    const PublicationIdentity currentScopeIdentity{
        .weaponGenerationKey = 0x10,
        .equippedWeaponOwnershipKey = 0x20,
        .weaponFormID = 0x30,
    };
    ok &= expectTrue("scope sight publication accepts its exact equipped weapon owner",
        matchesCurrentEquippedWeapon(currentScopeIdentity, currentScopeIdentity));
    ok &= expectFalse("scope sight publication rejects a previous weapon instance",
        matchesCurrentEquippedWeapon(PublicationIdentity{ 0x10, 0x21, 0x30 }, currentScopeIdentity));
    ok &= expectFalse("scope sight publication rejects a previous weapon form",
        matchesCurrentEquippedWeapon(PublicationIdentity{ 0x10, 0x20, 0x31 }, currentScopeIdentity));
    ok &= expectFalse("scope sight publication rejects a previous collision generation",
        matchesCurrentEquippedWeapon(PublicationIdentity{ 0x11, 0x20, 0x30 }, currentScopeIdentity));
    ok &= expectFalse("scope sight publication fails closed without ownership",
        matchesCurrentEquippedWeapon(PublicationIdentity{ 0x10, 0, 0x30 }, currentScopeIdentity));

    ok &= expectTrue("effect-shader geometry is excluded regardless of author name",
        classify(Evidence{ .hasEffectShaderProperty = true, .geometryName = "Glass:0" }) == ExclusionReason::EffectShaderProperty);
    ok &= expectTrue("billboard geometry is excluded without relying on author name",
        classify(Evidence{ .hasBillboardAncestor = true, .geometryName = "Plane01" }) == ExclusionReason::BillboardAncestor);
    ok &= expectTrue("laser beam fallback name is excluded",
        classify(Evidence{ .geometryName = "LaserSightBeam:0" }) == ExclusionReason::KnownEffectName);
    ok &= expectTrue("laser dot fallback name is excluded",
        classify(Evidence{ .geometryName = "LaserSightDot:0" }) == ExclusionReason::KnownEffectName);
    ok &= expectTrue("custom laser ray fallback name is excluded",
        classify(Evidence{ .geometryName = "AA12LaserRay" }) == ExclusionReason::KnownEffectName);
    ok &= expectTrue("flashlight glow fallback name is excluded",
        classify(Evidence{ .geometryName = "ScreenGlowEffect01:0" }) == ExclusionReason::KnownEffectName);
    ok &= expectTrue("reticle fallback name is excluded",
        classify(Evidence{ .geometryName = "deltapoint_reticle" }) == ExclusionReason::KnownEffectName);
    ok &= expectFalse("physical flashlight module remains collidable",
        classify(Evidence{ .geometryName = "Flashlight:0" }) != ExclusionReason::None);
    ok &= expectFalse("physical laser module remains collidable",
        classify(Evidence{ .geometryName = "LaserSight:0" }) != ExclusionReason::None);
    ok &= expectFalse("combined physical laser and light housing remains collidable",
        classify(Evidence{ .geometryName = "laser_light_Viridian_C5L:0" }) != ExclusionReason::None);
    ok &= expectFalse("scope housing remains collidable",
        classify(Evidence{ .geometryName = "Sight:0" }) != ExclusionReason::None);

    ok &= expectTrue("laser beam is classified as a laser emitter",
        emitter::classify({ .nodeName = "LaserSightBeam:0", .effectGeometry = true, .sightContext = true }) == emitter::Kind::Laser);
    ok &= expectTrue("reticle name outranks a neighboring laser context",
        emitter::classify({ .nodeName = "Reticle:0", .effectGeometry = true, .laserContext = true, .sightContext = true }) == emitter::Kind::Reticle);
    ok &= expectTrue("generic sight effect is classified as a reticle emitter",
        emitter::classify({ .nodeName = "Glass:0", .effectGeometry = true, .sightContext = true }) == emitter::Kind::Reticle);
    ok &= expectTrue("flashlight AddOnNode uses physical attachment context",
        emitter::classify({ .nodeName = "AddOnNode130", .valueNode = true, .flashlightContext = true }) == emitter::Kind::Flashlight);
    ok &= expectTrue("unowned AddOnNode fails closed",
        emitter::classify({ .nodeName = "AddOnNode130", .valueNode = true }) == emitter::Kind::Unknown);
    const auto addOnNode130 = emitter::parseAddOnNodeValue("AddOnNode130");
    ok &= expectTrue("AddOnNode decimal suffix is available", addOnNode130.valid && addOnNode130.value == 130);
    ok &= expectFalse("AddOnNode without numeric suffix is unavailable", emitter::parseAddOnNodeValue("AddOnNode").valid);
    ok &= expectTrue("AddOnNode transform outranks laser dot geometry",
        emitter::transformPriority(emitter::Kind::Laser, emitter::Source::AddOnNode, "AddOnNode130") >
            emitter::transformPriority(emitter::Kind::Laser, emitter::Source::EffectGeometry, "LaserSightDot:0"));

    using rock::manual_scope_target_policy::hasExplicitScopeIdentity;
    ok &= expectTrue("Watchman scope path recovers manual native-scope eligibility",
        hasExplicitScopeIdentity("[Sights] S&B PM II 5-25x56 - Black", "Weapons\\SalientPhantom\\OMEN\\Sights\\Scopes\\5x25.nif"));
    ok &= expectTrue("explicit scope record recovers eligibility with a generic model path",
        hasExplicitScopeIdentity("Recon Scope", "Weapons\\Example\\Optic.nif"));
    ok &= expectFalse("generic red-dot sight does not acquire native-scope eligibility",
        hasExplicitScopeIdentity("Trijicon MRO", "Weapons\\Example\\Sight_MRO.nif"));
    ok &= expectFalse("scope-named record without a physical model fails closed",
        hasExplicitScopeIdentity("Recon Scope", ""));
    rock::manual_scope_target_policy::StructuralMarkerEvidence mike24Structure{};
    rock::manual_scope_target_policy::observeStructuralNodeName(mike24Structure, "ScopeAiming");
    rock::manual_scope_target_policy::observeStructuralNodeName(mike24Structure, "ScopeViewParts");
    rock::manual_scope_target_policy::observeStructuralNodeName(mike24Structure, "ScopeFade:0");
    ok &= expectTrue("Mike24 native scope hierarchy is recognized without scope text in its OMOD",
        rock::manual_scope_target_policy::hasMagnifiedScopeStructure(mike24Structure));
    rock::manual_scope_target_policy::StructuralMarkerEvidence redDotStructure{};
    rock::manual_scope_target_policy::observeStructuralNodeName(redDotStructure, "Sight");
    rock::manual_scope_target_policy::observeStructuralNodeName(redDotStructure, "Reticle:0");
    ok &= expectFalse("generic red-dot hierarchy is not promoted to a magnified native scope",
        rock::manual_scope_target_policy::hasMagnifiedScopeStructure(redDotStructure));
    ok &= expectFalse("native overlay zero does not provide a reticle",
        rock::manual_scope_target_policy::isValidNativeOverlayIndex(0));
    ok &= expectTrue("native overlay upper bound is accepted",
        rock::manual_scope_target_policy::isValidNativeOverlayIndex(16));
    ok &= expectFalse("out-of-range native overlay fails closed",
        rock::manual_scope_target_policy::isValidNativeOverlayIndex(17));
    {
        namespace scope = rock::manual_scope_target_policy;
        ok &= expectTrue("native admission matches the published weapon instance", scope::matchesNativeIdentity(0x100, 0x200, 0x100, 0x200));
        ok &= expectFalse("native admission rejects another weapon before publication", scope::matchesNativeIdentity(0x100, 0x200, 0x101, 0x200));
        ok &= expectFalse("native admission rejects another instance of the same weapon", scope::matchesNativeIdentity(0x100, 0x200, 0x100, 0x201));
        ok &= expectFalse("empty publication cannot admit a native scope", scope::matchesNativeIdentity(0, 0, 0, 0));
        ok &= expectTrue("base-only weapon identity remains representable", scope::matchesNativeIdentity(0x100, 0, 0x100, 0));
        ok &= expectFalse("an instance may remove its base scope flag", scope::nativeHasScope(true, false, true));
        ok &= expectTrue("an instance may add a scope", scope::nativeHasScope(true, true, false));
        ok &= expectTrue("base scope remains usable without extra OMOD data", scope::nativeHasScope(false, false, true));
        ok &= expectFalse("a generic sight is not a forced scope", scope::isScopeEligible(false, false, false));
        ok &= expectTrue("structural scope survives missing flags and presentation data", scope::isScopeEligible(false, false, true));
        ok &= expectFalse("scope text in weapon directory cannot classify its barrel",
            scope::hasExplicitScopeIdentity("Heavy Barrel", "Weapons/ScopeRifle/Barrel.nif"));
        ok &= expectTrue("generically named optic in an explicit scope directory is retained",
            scope::hasExplicitScopeIdentity("ATACR", "Weapons/MCPR/Scope/ATACR416.nif"));
        ok &= expectTrue("MCPR keeps authored 8x and receives standard overlay",
            scope::resolveMagnification(8.0f) == 8.0f && scope::resolveOverlay(0) == 6);
        ok &= expectTrue("working native zoom and overlay remain independent",
            scope::resolveMagnification(10.0f) == 10.0f && scope::resolveOverlay(8) == 8);
        ok &= expectTrue("recon overlay is retained when zoom needs a default",
            scope::resolveMagnification(0.0f) == 4.0f && scope::resolveOverlay(16) == 16);
        ok &= expectTrue("missing both fields selects the standard 4x presentation",
            scope::resolveMagnification(0.0f) == 4.0f && scope::resolveOverlay(0) == 6);
        for (float invalid : { -1.0f, 0.0f, 1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() }) {
            ok &= expectTrue("invalid or unmagnified scope zoom uses 4x", scope::resolveMagnification(invalid) == 4.0f);
        }
        ok &= expectTrue("out-of-range overlay receives a usable default", scope::resolveOverlay(0xFFFFFFFFu) == 6);
        // Independent optical expectation: a 90-degree camera at 4x has
        // tan(FOV/2)=1/4; it must not use the linear 90/4 approximation.
        ok &= expectTrue("4x scope projection is 28.0725 degrees",
            std::abs(scope::magnifiedFieldOfView(90.0f, 4.0f) - 28.072487f) < 0.0001f);
        ok &= expectTrue("8x scope projection is 14.25 degrees",
            std::abs(scope::magnifiedFieldOfView(90.0f, 8.0f) - 14.250033f) < 0.0001f);
    }
    ok &= expectTrue("unflagged explicit scope requires the direct native transition",
        rock::manual_scope_target_policy::requiresDirectNativeTransition(false, true, false, true));
    ok &= expectTrue("unflagged structurally verified scope requires the direct native transition",
        rock::manual_scope_target_policy::requiresDirectNativeTransition(false, false, true, true));
    ok &= expectFalse("authored native scope keeps the normal hooked transition path",
        rock::manual_scope_target_policy::requiresDirectNativeTransition(true, true, true, true));
    ok &= expectFalse("unflagged scope with invalid overlay metadata fails closed",
        rock::manual_scope_target_policy::requiresDirectNativeTransition(false, true, true, false));

    const std::uint64_t bodySetKey = makeGeneratedWeaponBodySetKey(0xABC, derivedCompact, 1);
    ok &= expectNonZero("body-set key is created for equipped source and epoch", bodySetKey);
    ok &= expectDifferent("body-set key changes with epoch", bodySetKey, makeGeneratedWeaponBodySetKey(0xABC, derivedCompact, 2));
    ok &= expectDifferent("body-set key changes with equipped weapon key", bodySetKey, makeGeneratedWeaponBodySetKey(0xDEF, derivedCompact, 1));

    auto changedSource = derivedCompact;
    changedSource.geometryHash ^= 0x55;
    ok &= expectDifferent("body-set key changes with visible geometry", bodySetKey, makeGeneratedWeaponBodySetKey(0xABC, changedSource, 1));
    ok &= expectFalse("body-set key requires equipped weapon key", makeGeneratedWeaponBodySetKey(0, derivedCompact, 1) != 0);
    ok &= expectFalse("body-set key requires source signature", makeGeneratedWeaponBodySetKey(0xABC, GeneratedSourceCompleteness{}, 1) != 0);
    ok &= expectFalse("body-set key requires epoch", makeGeneratedWeaponBodySetKey(0xABC, derivedCompact, 0) != 0);

    EquippedWeaponGenerationIdentity identity{};
    ok &= expectFalse("empty equipped identity produces no content key", makeEquippedWeaponIdentityKey(identity) != 0);
    identity.hasEquippedWeapon = true;
    identity.formID = 0x1234;
    identity.formAddress = 0x2222;
    identity.instanceDataAddress = 0x2223;
    identity.instanceKeywordDataAddress = 0x2224;
    identity.instanceContentKey = 0x3333;
    identity.objectInstanceExtraAddress = 0x2225;
    identity.equippedDataAddress = 0x2226;
    identity.equippedObjectAddress = 0x2227;
    identity.objectIndexDataSignature = 0xAAAA;
    identity.objectIndexDataCount = 2;
    identity.activeModCount = 2;
    identity.displayName = "Test Weapon";
    const std::uint64_t contentKey = makeEquippedWeaponIdentityKey(identity);
    const std::uint64_t ownershipKey = makeEquippedWeaponOwnershipKey(identity);
    ok &= expectNonZero("equipped identity creates content key", contentKey);
    ok &= expectNonZero("equipped instance witnesses create ownership key", ownershipKey);
    auto pointerChurnIdentity = identity;
    pointerChurnIdentity.formAddress = 0x9000;
    pointerChurnIdentity.instanceDataAddress = 0x9001;
    pointerChurnIdentity.instanceKeywordDataAddress = 0x9002;
    pointerChurnIdentity.objectInstanceExtraAddress = 0x9003;
    pointerChurnIdentity.equippedDataAddress = 0x9004;
    pointerChurnIdentity.equippedObjectAddress = 0x9005;
    ok &= expectSame("runtime pointer churn does not change content key", contentKey, makeEquippedWeaponIdentityKey(pointerChurnIdentity));
    ok &= expectDifferent("different equipped instance witnesses change ownership key", ownershipKey, makeEquippedWeaponOwnershipKey(pointerChurnIdentity));
    auto transientEquipWrapperChurn = identity;
    transientEquipWrapperChurn.objectInstanceExtraAddress = 0xA001;
    transientEquipWrapperChurn.equippedDataAddress = 0xA002;
    transientEquipWrapperChurn.equippedObjectAddress = 0xA003;
    ok &= expectSame("stable instance data preserves ownership across transient equip-wrapper churn", ownershipKey,
        makeEquippedWeaponOwnershipKey(transientEquipWrapperChurn));
    auto sameInstanceContentChange = identity;
    sameInstanceContentChange.instanceContentKey ^= 0x55;
    ok &= expectSame("same equipped instance keeps ownership across content-key rebuilds", ownershipKey, makeEquippedWeaponOwnershipKey(sameInstanceContentChange));
    auto objectExtraFallback = identity;
    objectExtraFallback.instanceDataAddress = 0;
    objectExtraFallback.equippedDataAddress = 0;
    objectExtraFallback.equippedObjectAddress = 0;
    const std::uint64_t objectExtraFallbackKey = makeEquippedWeaponOwnershipKey(objectExtraFallback);
    ok &= expectNonZero("object instance extra provides ownership fallback", objectExtraFallbackKey);
    objectExtraFallback.objectInstanceExtraAddress ^= 0x10;
    ok &= expectDifferent("object instance fallback remains instance-bound", objectExtraFallbackKey, makeEquippedWeaponOwnershipKey(objectExtraFallback));
    auto equippedObjectFallback = identity;
    equippedObjectFallback.instanceDataAddress = 0;
    equippedObjectFallback.objectInstanceExtraAddress = 0;
    equippedObjectFallback.equippedDataAddress = 0;
    ok &= expectNonZero("equipped object provides ownership fallback", makeEquippedWeaponOwnershipKey(equippedObjectFallback));
    auto equippedDataFallback = equippedObjectFallback;
    equippedDataFallback.equippedObjectAddress = 0;
    equippedDataFallback.equippedDataAddress = identity.equippedDataAddress;
    ok &= expectNonZero("equipped data provides final ownership fallback", makeEquippedWeaponOwnershipKey(equippedDataFallback));
    auto missingInstanceWitness = identity;
    missingInstanceWitness.instanceDataAddress = 0;
    missingInstanceWitness.objectInstanceExtraAddress = 0;
    missingInstanceWitness.equippedDataAddress = 0;
    missingInstanceWitness.equippedObjectAddress = 0;
    ok &= expectFalse("manual ownership fails closed without an instance witness", makeEquippedWeaponOwnershipKey(missingInstanceWitness) != 0);
    identity.instanceContentKey = 0x3334;
    ok &= expectDifferent("content key changes with equipped instance content", contentKey, makeEquippedWeaponIdentityKey(identity));
    identity.instanceContentKey = 0x3333;
    identity.objectIndexDataSignature = 0xAAAB;
    ok &= expectDifferent("content key changes with equipped mod index content", contentKey, makeEquippedWeaponIdentityKey(identity));

    using namespace rock::weapon_material_visibility;
    ok &= expectTrue("Makarov hidden rail/sticker texture is excluded", isInvisibleTexture("AKMnv/Invisible.dds"));
    ok &= expectTrue("live texture names accept resource prefix and Windows case/separators",
        isInvisibleTexture("Textures\\AKMNV\\INVISIBLE.DDS"));
    ok &= expectFalse("installed sight rail stays collidable",
        isInvisibleTexture("SPAS12nv/Attachments/Rail/Rail LP_Rail_d.DDS"));
    ok &= expectFalse("unrelated invisible basename does not identify an absent part", isInvisibleTexture("Other/Invisible.dds"));
    ok &= expectFalse("no substring match", isInvisibleTexture("AKMnv/Invisible.dds.extra"));
    ok &= expectFalse("missing texture is not proof of absence", isInvisibleTexture(""));
    const auto hidden = decideCull(true, false, false);
    ok &= expectTrue("absent part acquires a rendering cull", hidden.culled && hidden.owned);
    const auto reasserted = decideCull(true, false, hidden.owned);
    ok &= expectTrue("foreign uncull cannot expose an absent part", reasserted.culled && reasserted.owned);
    const auto installed = decideCull(false, true, hidden.owned);
    ok &= expectFalse("installing attachment restores its rendering", installed.culled || installed.owned);
    const auto originallyHidden = decideCull(true, true, false);
    ok &= expectTrue("engine cull is preserved without claiming ownership", originallyHidden.culled && !originallyHidden.owned);
    const auto restoredNative = decideCull(false, originallyHidden.culled, originallyHidden.owned);
    ok &= expectTrue("material swap does not clear an engine-owned cull", restoredNative.culled && !restoredNative.owned);

    return ok ? 0 : 1;
}
