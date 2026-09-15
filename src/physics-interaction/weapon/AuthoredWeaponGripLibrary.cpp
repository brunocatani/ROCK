#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"
#include "physics-interaction/weapon/PipeFiringGripPolicy.h"
#include "physics-interaction/grab/FrikWeaponOffsetCache.h"
#include "physics-interaction/TransformMath.h"

#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiAVObject.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace rock::authored_weapon_grip_library
{
    namespace
    {
        constexpr std::size_t kCapacity = 128;
        constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ull;
        constexpr std::uint64_t kFnvPrime = 1099511628211ull;

        struct Entry
        {
            std::uint32_t weaponFormId{ 0 };
            std::uint64_t variantKey{ 0 };
            std::uint64_t instanceContentKey{ 0 };
            RE::NiTransform rightHandWeaponLocal{};
            RE::NiTransform rightPositionOnlyHandWeaponLocal{};
            FiringFingerPose rightFiringFingerPose{};
            RE::NiTransform supportHandWeaponLocal{};
            FiringFingerPose supportFingerPose{};
            std::uint64_t supportCaptureSequence{ 0 };
            std::uint64_t captureSequence{ 0 };
            std::uint64_t positionOnlyFrikOffsetRevision{ 0 };
            std::uint64_t publicationOrdinal{ 0 };
            CaptureSource source{ CaptureSource::Unknown };
            CaptureSource supportSource{ CaptureSource::Unknown };
            bool inPowerArmor{ false };
            bool instanceContentKnown{ false };
            bool hasSupportRelation{ false };
            bool hasRightPositionOnlyHandWeaponLocal{ false };
            bool occupied{ false };
            bool vanillaPipePose{ false };
        };

        std::array<Entry, kCapacity> s_entries{};
        std::uint64_t s_publicationOrdinal = 0;

        [[nodiscard]] bool finiteTransform(const RE::NiTransform& transform)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) && std::isfinite(transform.translate.z) && std::isfinite(transform.scale) &&
                std::abs(transform.scale) > 0.000001f;
        }

        [[nodiscard]] bool validCompleteFingerPose(const FiringFingerPose& pose)
        {
            if (!pose.complete()) {
                return false;
            }
            for (const auto& transform : pose.localTransforms) {
                if (!finiteTransform(transform)) {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] std::uint64_t hashName(std::string_view name)
        {
            if (name.empty()) {
                return 0;
            }

            std::uint64_t hash = kFnvOffsetBasis;
            for (const unsigned char value : name) {
                hash ^= value;
                hash *= kFnvPrime;
            }
            return hash;
        }

        [[nodiscard]] bool sameIdentity(
            const Entry& entry,
            const std::uint32_t weaponFormId,
            const WeaponVariantIdentity variant,
            const bool inPowerArmor)
        {
            return entry.occupied &&
                   entry.weaponFormId == weaponFormId &&
                   entry.variantKey == variant.key &&
                   entry.instanceContentKey == variant.instanceContentKey &&
                   entry.instanceContentKnown == variant.instanceContentKnown &&
                   entry.inPowerArmor == inPowerArmor;
        }

        [[nodiscard]] authored_weapon_grip_authority_policy::PublicationAuthority publicationAuthority(const CaptureSource source) noexcept
        {
            using Authority = authored_weapon_grip_authority_policy::PublicationAuthority;
            switch (source) {
            case CaptureSource::LiveEquippedGraph:
                return Authority::LiveEquippedGraph;
            case CaptureSource::PersistedNativeIdle:
                return Authority::PersistedNativeIdle;
            case CaptureSource::NativeIdlePreharvest:
                return Authority::FreshNativeIdle;
            case CaptureSource::Unknown:
            default:
                return Authority::Unknown;
            }
        }

        [[nodiscard]] const char* captureSourceName(const CaptureSource source)
        {
            switch (source) {
            case CaptureSource::LiveEquippedGraph:
                return "liveEquippedGraph";
            case CaptureSource::PersistedNativeIdle:
                return "persistedNativeIdle";
            case CaptureSource::NativeIdlePreharvest:
                return "nativeIdlePreharvest";
            case CaptureSource::Unknown:
            default:
                return "unknown";
            }
        }

        [[nodiscard]] LookupResult makeResult(const Entry& entry, const bool usedVariantFallback)
        {
            return LookupResult{
                .found = true,
                .rightHandWeaponLocal = entry.rightHandWeaponLocal,
                .rightPositionOnlyHandWeaponLocal =
                    entry.rightPositionOnlyHandWeaponLocal,
                .rightFiringFingerPose = entry.rightFiringFingerPose,
                .supportHandWeaponLocal = entry.supportHandWeaponLocal,
                .supportFingerPose = entry.supportFingerPose,
                .supportCaptureSequence = entry.supportCaptureSequence,
                .captureSequence = entry.captureSequence,
                .positionOnlyFrikOffsetRevision =
                    entry.positionOnlyFrikOffsetRevision,
                .source = entry.source,
                .supportSource = entry.supportSource,
                .hasSupportRelation = entry.hasSupportRelation,
                .hasRightPositionOnlyHandWeaponLocal =
                    entry.hasRightPositionOnlyHandWeaponLocal,
                .usedVariantFallback = usedVariantFallback,
                .vanillaPipePose = entry.vanillaPipePose,
                .reason = usedVariantFallback ? "authoredAnimationFormFallback" : "authoredAnimationExactVariant",
            };
        }
    }

    WeaponVariantIdentity identifyWeaponVariant(
        const RE::NiAVObject* weaponRoot,
        const std::uint64_t instanceContentKey,
        const bool instanceContentKnown) noexcept
    {
        auto* mutableRoot = const_cast<RE::NiAVObject*>(weaponRoot);
        auto* grip = mutableRoot ? f4vr::findNode(mutableRoot, "P-Grip") : nullptr;
        auto* gripChild = grip ? f4vr::getFirstChild(grip) : nullptr;
        const char* childName = gripChild ? gripChild->name.c_str() : nullptr;
        return WeaponVariantIdentity{
            .key = childName ? hashName(childName) : 0,
            .instanceContentKey = instanceContentKey,
            .instanceContentKnown = instanceContentKnown,
        };
    }

    bool publish(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, const bool inPowerArmor, const RE::NiTransform& rightHandWeaponLocal,
        const std::uint64_t captureSequence, const CaptureSource source, const FiringFingerPose* rightFiringFingerPose)
    {
        return publishResolvedVariant(
            weapon,
            identifyWeaponVariant(weaponRoot),
            inPowerArmor,
            rightHandWeaponLocal,
            captureSequence,
            source,
            rightFiringFingerPose);
    }

    bool publishResolvedVariant(const RE::TESObjectWEAP* weapon, const WeaponVariantIdentity variant, const bool inPowerArmor,
        const RE::NiTransform& rightHandWeaponLocal, const std::uint64_t captureSequence, const CaptureSource source,
        const FiringFingerPose* rightFiringFingerPose, const bool vanillaPipePose)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        const bool validFingerPose = rightFiringFingerPose && validCompleteFingerPose(*rightFiringFingerPose);
        if (weaponFormId == 0 || captureSequence == 0 || source == CaptureSource::Unknown || !finiteTransform(rightHandWeaponLocal) ||
            (rightFiringFingerPose && !validFingerPose) ||
            !authored_weapon_grip_authority_policy::publicationHasRequiredFingerPose(isNativeIdleAuthority(source), validFingerPose)) {
            return false;
        }

        Entry* destination = nullptr;
        Entry* oldest = nullptr;
        for (auto& entry : s_entries) {
            if (sameIdentity(entry, weaponFormId, variant, inPowerArmor)) {
                destination = &entry;
                break;
            }
            if (!entry.occupied) {
                if (!destination) {
                    destination = &entry;
                }
                continue;
            }
            if (!oldest || entry.publicationOrdinal < oldest->publicationOrdinal) {
                oldest = &entry;
            }
        }
        if (!destination) {
            destination = oldest;
        }
        if (!destination) {
            return false;
        }

        const bool newIdentity = !sameIdentity(*destination, weaponFormId, variant, inPowerArmor);
        if (newIdentity) {
            *destination = {};
            destination->weaponFormId = weaponFormId;
            destination->variantKey = variant.key;
            destination->instanceContentKey = variant.instanceContentKey;
            destination->instanceContentKnown = variant.instanceContentKnown;
            destination->inPowerArmor = inPowerArmor;
            destination->occupied = true;
        }

        /*
         * A successful off-screen idle sample is the asset-authored source of
         * truth. Live graph capture remains a compatibility fallback for an
         * asset that cannot be harvested, but it must never replace a
         * preharvested relation or make the exact finger pose disappear.
         */
        if (!authored_weapon_grip_authority_policy::shouldAcceptPublication(
                !newIdentity,
                publicationAuthority(destination->source),
                publicationAuthority(source))) {
            return true;
        }

        if (destination->captureSequence != captureSequence) {
            destination->rightPositionOnlyHandWeaponLocal = {};
            destination->positionOnlyFrikOffsetRevision = 0;
            destination->hasRightPositionOnlyHandWeaponLocal = false;
            // The paired support relation belongs to the authored pose, not
            // to one capture instance: the same idle republished from a new
            // source (live -> preharvest -> disk cache) keeps it. A
            // materially different canonical invalidates it.
            if (destination->hasSupportRelation &&
                !authored_weapon_grip_authority_policy::
                    handRelationValueMatches(
                        destination->rightHandWeaponLocal,
                        rightHandWeaponLocal)) {
                destination->supportHandWeaponLocal = {};
                destination->supportFingerPose = {};
                destination->supportCaptureSequence = 0;
                destination->supportSource = CaptureSource::Unknown;
                destination->hasSupportRelation = false;
            }
        }
        destination->rightHandWeaponLocal = rightHandWeaponLocal;
        destination->rightFiringFingerPose = rightFiringFingerPose ? *rightFiringFingerPose : FiringFingerPose{};
        destination->captureSequence = captureSequence;
        destination->publicationOrdinal = ++s_publicationOrdinal;
        destination->source = source;
        destination->vanillaPipePose = vanillaPipePose && source == CaptureSource::NativeIdlePreharvest;

        if (newIdentity || isNativeIdleAuthority(source)) {
            ROCK_LOG_INFO(Animation,
                "Learned authored weapon grip formID={:08X} pGripVariant={:016X} instanceContent={:016X} instanceKnown={} powerArmor={} capture={} source={} firingFingerMask=0x{:04X}",
                weaponFormId,
                variant.key,
                variant.instanceContentKey,
                variant.instanceContentKnown ? "yes" : "no",
                inPowerArmor ? "yes" : "no",
                captureSequence,
                captureSourceName(source),
                destination->rightFiringFingerPose.enabledMask);
        }
        return true;
    }

    bool publishPositionOnlyHold(
        const RE::TESObjectWEAP* weapon,
        const bool inPowerArmor,
        const std::uint64_t authoredCaptureSequence,
        const std::uint64_t frikOffsetRevision,
        const RE::NiTransform& rightPositionOnlyHandWeaponLocal)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        if (weaponFormId == 0 || authoredCaptureSequence == 0 ||
            frikOffsetRevision == 0 ||
            !finiteTransform(rightPositionOnlyHandWeaponLocal)) {
            return false;
        }

        for (auto& entry : s_entries) {
            if (!entry.occupied ||
                entry.weaponFormId != weaponFormId ||
                entry.inPowerArmor != inPowerArmor ||
                entry.captureSequence != authoredCaptureSequence) {
                continue;
            }

            entry.rightPositionOnlyHandWeaponLocal =
                rightPositionOnlyHandWeaponLocal;
            entry.positionOnlyFrikOffsetRevision = frikOffsetRevision;
            entry.hasRightPositionOnlyHandWeaponLocal = true;
            return true;
        }
        return false;
    }

    bool publishSupportRelation(
        const RE::TESObjectWEAP* weapon,
        const WeaponVariantIdentity variant,
        const bool inPowerArmor,
        const RE::NiTransform& supportHandWeaponLocal,
        const FiringFingerPose& supportFingerPose,
        const std::uint64_t supportCaptureSequence,
        const CaptureSource source)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        if (weaponFormId == 0 || supportCaptureSequence == 0 ||
            source == CaptureSource::Unknown ||
            !finiteTransform(supportHandWeaponLocal) ||
            !validCompleteFingerPose(supportFingerPose)) {
            return false;
        }

        for (auto& entry : s_entries) {
            if (!sameIdentity(entry, weaponFormId, variant, inPowerArmor)) {
                continue;
            }
            if (entry.captureSequence == 0) {
                return false;
            }
            // A live equipped-graph capture is a per-frame read that can
            // land mid-blend; it never displaces the clip-sampled relation.
            if (!authored_weapon_grip_authority_policy::shouldAcceptPublication(
                    entry.hasSupportRelation,
                    publicationAuthority(entry.supportSource),
                    publicationAuthority(source))) {
                return true;
            }
            if (entry.hasSupportRelation &&
                entry.supportSource == source &&
                authored_weapon_grip_authority_policy::
                    handRelationValueMatches(
                        entry.supportHandWeaponLocal,
                        supportHandWeaponLocal)) {
                // Same authored pose within idle-sway tolerance: keep the
                // stored value and sequence.
                return true;
            }
            const bool firstRelation = !entry.hasSupportRelation;
            entry.supportHandWeaponLocal = supportHandWeaponLocal;
            entry.supportFingerPose = supportFingerPose;
            entry.supportCaptureSequence = supportCaptureSequence;
            entry.supportSource = source;
            entry.hasSupportRelation = true;
            ROCK_LOG_INFO(Animation,
                "{} authored support relation formID={:08X} pGripVariant={:016X} instanceContent={:016X} powerArmor={} capture={} source={} supportHandT=({:.3f},{:.3f},{:.3f})",
                firstRelation ? "Learned" : "Updated",
                weaponFormId,
                variant.key,
                variant.instanceContentKey,
                inPowerArmor ? "yes" : "no",
                supportCaptureSequence,
                captureSourceName(source),
                supportHandWeaponLocal.translate.x,
                supportHandWeaponLocal.translate.y,
                supportHandWeaponLocal.translate.z);
            return true;
        }
        return false;
    }

    void applyPipeDefaultOffset(LookupResult& result, const frik_weapon_offset_cache::LookupResult& offset, const bool isLeft) noexcept
    {
        if (!pipe_firing_grip_policy::useFrikDefault(result.found, isLeft, result.vanillaPipePose,
                offset.found && offset.source == frik_weapon_offset_cache::OffsetSource::EmbeddedResource)) return;
        // FRIK stores Weapon-in-Hand; authored consumers need Hand-in-Weapon.
        const auto handInWeapon = transform_math::invertTransform(offset.offset);
        if (!finiteTransform(handInWeapon)) return;
        result.rightHandWeaponLocal = handInWeapon;
        result.reason = "pipeDefaultFrikOffset";
    }

    LookupResult find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, const bool inPowerArmor)
    {
        auto result = findResolvedVariant(weapon, identifyWeaponVariant(weaponRoot), inPowerArmor);
        if (!result.found) return result;
        RE::NiPoint3 displacement{};
        if (!vanilla_weapon_grip_frame::resolveModelTranslation(weapon->formID, weaponRoot, displacement)) {
            return LookupResult{ .reason = "invalidVanillaModelFrame" };
        }
        result.rightHandWeaponLocal = vanilla_weapon_grip_frame::translateGrip(result.rightHandWeaponLocal, displacement);
        if (result.hasSupportRelation) {
            result.supportHandWeaponLocal = vanilla_weapon_grip_frame::translateGrip(result.supportHandWeaponLocal, displacement);
        }
        return result;
    }

    LookupResult findResolvedVariant(const RE::TESObjectWEAP* weapon, const WeaponVariantIdentity variant, const bool inPowerArmor)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        if (weaponFormId == 0) {
            return LookupResult{ .reason = "missingWeaponForm" };
        }

        const Entry* exactVariantMatch = nullptr;
        const Entry* soleFormMatch = nullptr;
        const Entry* soleNativeIdleMatch = nullptr;
        std::size_t formMatchCount = 0;
        std::size_t nativeIdleMatchCount = 0;
        for (const auto& entry : s_entries) {
            if (!entry.occupied || entry.weaponFormId != weaponFormId || entry.inPowerArmor != inPowerArmor) {
                continue;
            }
            if (variant.instanceContentKnown &&
                (!entry.instanceContentKnown || entry.instanceContentKey != variant.instanceContentKey)) {
                continue;
            }
            if (entry.variantKey == variant.key &&
                (!variant.instanceContentKnown || entry.instanceContentKnown == variant.instanceContentKnown)) {
                exactVariantMatch = &entry;
            }
            soleFormMatch = &entry;
            ++formMatchCount;
            if (isNativeIdleAuthority(entry.source)) {
                soleNativeIdleMatch = &entry;
                ++nativeIdleMatchCount;
            }
        }

        // Equipped and loose scene graphs occasionally omit P-Grip at
        // different wrapper depths. Prefer the sole native-idle authority over
        // an exact-key live fallback: that generic live entry is commonly
        // created while the equipped subtree is incomplete and must not erase
        // the exact loose pose. Multiple native-idle variants remain ambiguous.
        switch (authored_weapon_grip_authority_policy::selectLookup(
            exactVariantMatch != nullptr,
            exactVariantMatch && isNativeIdleAuthority(exactVariantMatch->source),
            exactVariantMatch && exactVariantMatch->variantKey == 0,
            nativeIdleMatchCount,
            formMatchCount)) {
        case authored_weapon_grip_authority_policy::LookupSelection::ExactVariant:
            return makeResult(*exactVariantMatch, false);
        case authored_weapon_grip_authority_policy::LookupSelection::SoleNativeIdleVariant:
            return makeResult(*soleNativeIdleMatch,
                soleNativeIdleMatch->variantKey != variant.key ||
                    soleNativeIdleMatch->instanceContentKnown != variant.instanceContentKnown);
        case authored_weapon_grip_authority_policy::LookupSelection::SoleFormVariant:
            return makeResult(*soleFormMatch, true);
        case authored_weapon_grip_authority_policy::LookupSelection::None:
        default:
            break;
        }
        return LookupResult{
            .reason = nativeIdleMatchCount > 1 ? "authoredAnimationNativeIdleVariantAmbiguous" :
                                                  (formMatchCount > 1 ? "authoredAnimationVariantAmbiguous" : "authoredAnimationNotLearned"),
        };
    }
}
