#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/weapon/AuthoredWeaponGripEquivalencePolicy.h"

#include "f4vr/F4VRUtils.h"

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

        struct PreharvestBaseline
        {
            RE::NiTransform rightHandWeaponLocal{};
            std::uint64_t captureSequence{ 0 };
            bool available{ false };
            bool initialComparisonReported{ false };
            bool stableComparisonReported{ false };
        };

        struct LiveStability
        {
            RE::NiTransform previousRightHandWeaponLocal{};
            std::uint8_t stableSamples{ 0 };
            bool previousSampleAvailable{ false };
        };

        struct Entry
        {
            std::uint32_t weaponFormId{ 0 };
            std::uint64_t variantKey{ 0 };
            RE::NiTransform rightHandWeaponLocal{};
            std::uint64_t captureSequence{ 0 };
            std::uint64_t publicationOrdinal{ 0 };
            PreharvestBaseline preharvest{};
            LiveStability liveStability{};
            CaptureSource source{ CaptureSource::Unknown };
            bool inPowerArmor{ false };
            bool occupied{ false };
        };

        struct PreharvestMatch
        {
            Entry* entry{ nullptr };
            bool usedVariantFallback{ false };
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

        [[nodiscard]] std::uint64_t weaponVariantKey(const RE::NiAVObject* weaponRoot)
        {
            auto* mutableRoot = const_cast<RE::NiAVObject*>(weaponRoot);
            auto* grip = mutableRoot ? f4vr::findNode(mutableRoot, "P-Grip") : nullptr;
            auto* gripChild = grip ? f4vr::getFirstChild(grip) : nullptr;
            const char* childName = gripChild ? gripChild->name.c_str() : nullptr;
            return childName ? hashName(childName) : 0;
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

        void resetLiveStability(const std::uint32_t weaponFormId, const bool inPowerArmor)
        {
            for (auto& entry : s_entries) {
                if (entry.occupied && entry.weaponFormId == weaponFormId && entry.inPowerArmor == inPowerArmor) {
                    entry.liveStability = {};
                }
            }
        }

        [[nodiscard]] PreharvestMatch findPreharvestBaseline(
            Entry& liveEntry,
            const std::uint32_t weaponFormId,
            const std::uint64_t variantKey,
            const bool inPowerArmor)
        {
            if (sameIdentity(liveEntry, weaponFormId, variantKey, inPowerArmor) && liveEntry.preharvest.available) {
                return PreharvestMatch{ .entry = &liveEntry };
            }

            Entry* soleFormMatch = nullptr;
            std::size_t formMatchCount = 0;
            for (auto& entry : s_entries) {
                if (!entry.occupied || !entry.preharvest.available || entry.weaponFormId != weaponFormId || entry.inPowerArmor != inPowerArmor) {
                    continue;
                }
                soleFormMatch = &entry;
                ++formMatchCount;
            }
            if (formMatchCount == 1) {
                return PreharvestMatch{
                    .entry = soleFormMatch,
                    .usedVariantFallback = true,
                };
            }
            return {};
        }

        void logEquivalence(
            const char* phase,
            const Entry& preharvestEntry,
            const std::uint64_t liveVariantKey,
            const std::uint64_t liveCaptureSequence,
            const RE::NiTransform& liveRightHandWeaponLocal,
            const bool usedVariantFallback,
            const std::uint8_t stableSamples)
        {
            const auto& preharvest = preharvestEntry.preharvest.rightHandWeaponLocal;
            const float translationDelta = hand_visual_lerp_math::distanceGameUnits(preharvest.translate, liveRightHandWeaponLocal.translate);
            const float rotationDelta = hand_visual_lerp_math::rotationDistanceDegrees(preharvest, liveRightHandWeaponLocal);
            const float scaleDelta = std::abs(preharvest.scale - liveRightHandWeaponLocal.scale);
            ROCK_LOG_INFO(Animation,
                "Authored grip equivalence phase={} formID={:08X} match={} preharvestVariant={:016X} liveVariant={:016X} powerArmor={} preharvestCapture={} liveCapture={} "
                "stableSamples={} deltaT={:.6f}gu deltaR={:.6f}deg deltaS={:.7f} preharvestT=({:.6f},{:.6f},{:.6f}) liveT=({:.6f},{:.6f},{:.6f}) "
                "preharvestScale={:.7f} liveScale={:.7f}",
                phase, preharvestEntry.weaponFormId, usedVariantFallback ? "uniqueForm" : "exactVariant", preharvestEntry.variantKey, liveVariantKey,
                preharvestEntry.inPowerArmor ? "yes" : "no", preharvestEntry.preharvest.captureSequence, liveCaptureSequence, stableSamples, translationDelta, rotationDelta,
                scaleDelta, preharvest.translate.x, preharvest.translate.y, preharvest.translate.z, liveRightHandWeaponLocal.translate.x,
                liveRightHandWeaponLocal.translate.y, liveRightHandWeaponLocal.translate.z, preharvest.scale, liveRightHandWeaponLocal.scale);
            ROCK_LOG_INFO(Animation,
                "Authored grip equivalence matrices phase={} formID={:08X} preharvestR=({:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f}) "
                "liveR=({:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f};{:.7f},{:.7f},{:.7f})",
                phase, preharvestEntry.weaponFormId, preharvest.rotate.entry[0][0], preharvest.rotate.entry[0][1], preharvest.rotate.entry[0][2],
                preharvest.rotate.entry[1][0], preharvest.rotate.entry[1][1], preharvest.rotate.entry[1][2], preharvest.rotate.entry[2][0], preharvest.rotate.entry[2][1],
                preharvest.rotate.entry[2][2], liveRightHandWeaponLocal.rotate.entry[0][0], liveRightHandWeaponLocal.rotate.entry[0][1],
                liveRightHandWeaponLocal.rotate.entry[0][2], liveRightHandWeaponLocal.rotate.entry[1][0], liveRightHandWeaponLocal.rotate.entry[1][1],
                liveRightHandWeaponLocal.rotate.entry[1][2], liveRightHandWeaponLocal.rotate.entry[2][0], liveRightHandWeaponLocal.rotate.entry[2][1],
                liveRightHandWeaponLocal.rotate.entry[2][2]);
        }

        void observeLiveEquivalence(
            Entry& liveEntry,
            const std::uint32_t weaponFormId,
            const std::uint64_t liveVariantKey,
            const bool inPowerArmor,
            const RE::NiTransform& liveRightHandWeaponLocal,
            const std::uint64_t liveCaptureSequence)
        {
            auto match = findPreharvestBaseline(liveEntry, weaponFormId, liveVariantKey, inPowerArmor);
            if (!match.entry) {
                return;
            }

            auto& preharvestBaseline = match.entry->preharvest;
            if (!preharvestBaseline.initialComparisonReported) {
                logEquivalence("initial", *match.entry, liveVariantKey, liveCaptureSequence, liveRightHandWeaponLocal, match.usedVariantFallback, 1);
                preharvestBaseline.initialComparisonReported = true;
            }

            auto& stability = liveEntry.liveStability;
            float translationDelta = 0.0f;
            float rotationDelta = 0.0f;
            float scaleDelta = 0.0f;
            if (stability.previousSampleAvailable) {
                translationDelta = hand_visual_lerp_math::distanceGameUnits(stability.previousRightHandWeaponLocal.translate, liveRightHandWeaponLocal.translate);
                rotationDelta = hand_visual_lerp_math::rotationDistanceDegrees(stability.previousRightHandWeaponLocal, liveRightHandWeaponLocal);
                scaleDelta = std::abs(stability.previousRightHandWeaponLocal.scale - liveRightHandWeaponLocal.scale);
            }
            stability.stableSamples = authored_weapon_grip_equivalence_policy::advanceStableSampleCount(
                stability.previousSampleAvailable,
                stability.stableSamples,
                translationDelta,
                rotationDelta,
                scaleDelta);
            stability.previousRightHandWeaponLocal = liveRightHandWeaponLocal;
            stability.previousSampleAvailable = true;

            if (!preharvestBaseline.stableComparisonReported &&
                authored_weapon_grip_equivalence_policy::readyForStableReport(stability.stableSamples)) {
                logEquivalence("stable", *match.entry, liveVariantKey, liveCaptureSequence, liveRightHandWeaponLocal, match.usedVariantFallback, stability.stableSamples);
                preharvestBaseline.stableComparisonReported = true;
            }
        }

        [[nodiscard]] LookupResult makeResult(const Entry& entry, const bool usedVariantFallback)
        {
            return LookupResult{
                .found = true,
                .rightHandWeaponLocal = entry.rightHandWeaponLocal,
                .captureSequence = entry.captureSequence,
                .source = entry.source,
                .usedVariantFallback = usedVariantFallback,
                .reason = usedVariantFallback ? "authoredAnimationFormFallback" : "authoredAnimationExactVariant",
            };
        }
    }

    bool publish(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, const bool inPowerArmor, const RE::NiTransform& rightHandWeaponLocal,
        const std::uint64_t captureSequence, const CaptureSource source)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        if (weaponFormId == 0 || captureSequence == 0 || source == CaptureSource::Unknown || !finiteTransform(rightHandWeaponLocal)) {
            return false;
        }

        const std::uint64_t variantKey = weaponVariantKey(weaponRoot);
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

        if (source == CaptureSource::NativeIdlePreharvest) {
            resetLiveStability(weaponFormId, inPowerArmor);
            destination->preharvest = PreharvestBaseline{
                .rightHandWeaponLocal = rightHandWeaponLocal,
                .captureSequence = captureSequence,
                .available = true,
            };
        } else {
            observeLiveEquivalence(*destination, weaponFormId, variantKey, inPowerArmor, rightHandWeaponLocal, captureSequence);
        }

        destination->rightHandWeaponLocal = rightHandWeaponLocal;
        destination->captureSequence = captureSequence;
        destination->publicationOrdinal = ++s_publicationOrdinal;
        destination->source = source;

        if (newIdentity) {
            ROCK_LOG_INFO(Animation, "Learned authored loose-weapon grip formID={:08X} variant={:016X} powerArmor={} capture={} source={}", weaponFormId, variantKey,
                inPowerArmor ? "yes" : "no", captureSequence, captureSourceName(source));
        }
        return true;
    }

    LookupResult find(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, const bool inPowerArmor)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        if (weaponFormId == 0) {
            return LookupResult{ .reason = "missingWeaponForm" };
        }

        const std::uint64_t variantKey = weaponVariantKey(weaponRoot);
        const Entry* soleFormMatch = nullptr;
        std::size_t formMatchCount = 0;
        for (const auto& entry : s_entries) {
            if (!entry.occupied || entry.weaponFormId != weaponFormId || entry.inPowerArmor != inPowerArmor) {
                continue;
            }
            if (entry.variantKey == variantKey) {
                return makeResult(entry, false);
            }
            soleFormMatch = &entry;
            ++formMatchCount;
        }

        // Equipped and loose scene graphs occasionally omit P-Grip at
        // different wrapper depths. A single known form variant is
        // unambiguous; multiple known variants fail closed instead of applying
        // the wrong stock's grip.
        if (formMatchCount == 1 && soleFormMatch) {
            return makeResult(*soleFormMatch, true);
        }
        return LookupResult{
            .reason = formMatchCount > 1 ? "authoredAnimationVariantAmbiguous" : "authoredAnimationNotLearned",
        };
    }
}
