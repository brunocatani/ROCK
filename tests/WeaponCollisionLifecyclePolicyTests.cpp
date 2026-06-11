#include <cstdint>
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

    ok &= expectTrue("shell is transient reload geometry", isTransientReloadPart(WeaponPartKind::Shell));
    ok &= expectTrue("round is transient reload geometry", isTransientReloadPart(WeaponPartKind::Round));
    ok &= expectTrue("cosmetic ammo is transient reload geometry", isTransientReloadPart(WeaponPartKind::CosmeticAmmo));
    ok &= expectFalse("magazine is durable weapon structure", isTransientReloadPart(WeaponPartKind::Magazine));
    ok &= expectTrue("receiver is permanent gameplay-critical structure", (permanentGameplayCriticalPartMask() & partMask(WeaponPartKind::Receiver)) != 0);
    ok &= expectFalse("shell is not permanent gameplay-critical structure", (permanentGameplayCriticalPartMask() & partMask(WeaponPartKind::Shell)) != 0);

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
    ok &= expectFalse("empty equipped identity and empty visual key produce no generation key", makeEquippedWeaponGenerationKey(0, identity) != 0);
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
    const std::uint64_t generationKey = makeEquippedWeaponGenerationKey(0x4444, identity);
    ok &= expectNonZero("equipped identity creates generation key", generationKey);
    ok &= expectSame("visual-only witness changes do not change generation key", generationKey, makeEquippedWeaponGenerationKey(0x5555, identity));
    auto pointerChurnIdentity = identity;
    pointerChurnIdentity.formAddress = 0x9000;
    pointerChurnIdentity.instanceDataAddress = 0x9001;
    pointerChurnIdentity.instanceKeywordDataAddress = 0x9002;
    pointerChurnIdentity.objectInstanceExtraAddress = 0x9003;
    pointerChurnIdentity.equippedDataAddress = 0x9004;
    pointerChurnIdentity.equippedObjectAddress = 0x9005;
    ok &= expectSame("runtime pointer churn does not change generation key", generationKey, makeEquippedWeaponGenerationKey(0x4444, pointerChurnIdentity));
    identity.instanceContentKey = 0x3334;
    ok &= expectDifferent("generation key changes with equipped instance content", generationKey, makeEquippedWeaponGenerationKey(0x4444, identity));
    identity.instanceContentKey = 0x3333;
    identity.objectIndexDataSignature = 0xAAAB;
    ok &= expectDifferent("generation key changes with equipped mod index content", generationKey, makeEquippedWeaponGenerationKey(0x4444, identity));

    return ok ? 0 : 1;
}
