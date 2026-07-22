#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"

#include "physics-interaction/PhysicsLog.h"

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
            RE::NiTransform rightHandWeaponLocal{};
            FiringFingerPose rightFiringFingerPose{};
            std::uint64_t captureSequence{ 0 };
            std::uint64_t publicationOrdinal{ 0 };
            CaptureSource source{ CaptureSource::Unknown };
            bool inPowerArmor{ false };
            bool occupied{ false };
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

        [[nodiscard]] bool sameIdentity(const Entry& entry, const std::uint32_t weaponFormId, const std::uint64_t variantKey, const bool inPowerArmor)
        {
            return entry.occupied && entry.weaponFormId == weaponFormId && entry.variantKey == variantKey && entry.inPowerArmor == inPowerArmor;
        }

        [[nodiscard]] const char* captureSourceName(const CaptureSource source)
        {
            switch (source) {
            case CaptureSource::LiveEquippedGraph:
                return "liveEquippedGraph";
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
                .rightFiringFingerPose = entry.rightFiringFingerPose,
                .captureSequence = entry.captureSequence,
                .source = entry.source,
                .usedVariantFallback = usedVariantFallback,
                .reason = usedVariantFallback ? "authoredAnimationFormFallback" : "authoredAnimationExactVariant",
            };
        }
    }

    WeaponVariantIdentity identifyWeaponVariant(const RE::NiAVObject* weaponRoot) noexcept
    {
        auto* mutableRoot = const_cast<RE::NiAVObject*>(weaponRoot);
        auto* grip = mutableRoot ? f4vr::findNode(mutableRoot, "P-Grip") : nullptr;
        auto* gripChild = grip ? f4vr::getFirstChild(grip) : nullptr;
        const char* childName = gripChild ? gripChild->name.c_str() : nullptr;
        return WeaponVariantIdentity{ .key = childName ? hashName(childName) : 0 };
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
        const FiringFingerPose* rightFiringFingerPose)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        const bool validFingerPose = rightFiringFingerPose && validCompleteFingerPose(*rightFiringFingerPose);
        if (weaponFormId == 0 || captureSequence == 0 || source == CaptureSource::Unknown || !finiteTransform(rightHandWeaponLocal) ||
            (rightFiringFingerPose && !validFingerPose) ||
            !authored_weapon_grip_authority_policy::publicationHasRequiredFingerPose(source == CaptureSource::NativeIdlePreharvest, validFingerPose)) {
            return false;
        }

        const std::uint64_t variantKey = variant.key;
        Entry* destination = nullptr;
        Entry* oldest = nullptr;
        for (auto& entry : s_entries) {
            if (sameIdentity(entry, weaponFormId, variantKey, inPowerArmor)) {
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

        const bool newIdentity = !sameIdentity(*destination, weaponFormId, variantKey, inPowerArmor);
        if (newIdentity) {
            *destination = {};
            destination->weaponFormId = weaponFormId;
            destination->variantKey = variantKey;
            destination->inPowerArmor = inPowerArmor;
            destination->occupied = true;
        }

        /*
         * A successful off-screen idle sample is the asset-authored source of
         * truth. Live graph capture remains a compatibility fallback for an
         * asset that cannot be harvested, but it must never replace a
         * preharvested relation or make the exact finger pose disappear.
         */
        if (!authored_weapon_grip_authority_policy::shouldAcceptPublication(!newIdentity, destination->source == CaptureSource::NativeIdlePreharvest,
                source == CaptureSource::NativeIdlePreharvest)) {
            return true;
        }

        destination->rightHandWeaponLocal = rightHandWeaponLocal;
        destination->rightFiringFingerPose = rightFiringFingerPose ? *rightFiringFingerPose : FiringFingerPose{};
        destination->captureSequence = captureSequence;
        destination->publicationOrdinal = ++s_publicationOrdinal;
        destination->source = source;

        if (newIdentity || source == CaptureSource::NativeIdlePreharvest) {
            ROCK_LOG_INFO(Animation, "Learned authored loose-weapon grip formID={:08X} variant={:016X} powerArmor={} capture={} source={} firingFingerMask=0x{:04X}", weaponFormId, variantKey,
                inPowerArmor ? "yes" : "no", captureSequence, captureSourceName(source), destination->rightFiringFingerPose.enabledMask);
        }
        return true;
    }

    LookupResult find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, const bool inPowerArmor)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        if (weaponFormId == 0) {
            return LookupResult{ .reason = "missingWeaponForm" };
        }

        const std::uint64_t variantKey = identifyWeaponVariant(weaponRoot).key;
        const Entry* exactVariantMatch = nullptr;
        const Entry* soleFormMatch = nullptr;
        const Entry* soleNativeIdleMatch = nullptr;
        std::size_t formMatchCount = 0;
        std::size_t nativeIdleMatchCount = 0;
        for (const auto& entry : s_entries) {
            if (!entry.occupied || entry.weaponFormId != weaponFormId || entry.inPowerArmor != inPowerArmor) {
                continue;
            }
            if (entry.variantKey == variantKey) {
                exactVariantMatch = &entry;
            }
            soleFormMatch = &entry;
            ++formMatchCount;
            if (entry.source == CaptureSource::NativeIdlePreharvest) {
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
            exactVariantMatch && exactVariantMatch->source == CaptureSource::NativeIdlePreharvest,
            exactVariantMatch && exactVariantMatch->variantKey == 0,
            nativeIdleMatchCount,
            formMatchCount)) {
        case authored_weapon_grip_authority_policy::LookupSelection::ExactVariant:
            return makeResult(*exactVariantMatch, false);
        case authored_weapon_grip_authority_policy::LookupSelection::SoleNativeIdleVariant:
            return makeResult(*soleNativeIdleMatch, soleNativeIdleMatch->variantKey != variantKey);
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
