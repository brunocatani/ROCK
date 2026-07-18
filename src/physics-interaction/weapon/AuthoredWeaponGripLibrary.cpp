#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"

#include "physics-interaction/PhysicsLog.h"

#include "f4vr/F4VRUtils.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiAVObject.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
            std::uint64_t captureSequence{ 0 };
            std::uint64_t publicationOrdinal{ 0 };
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

        [[nodiscard]] LookupResult makeResult(const Entry& entry, const bool usedVariantFallback)
        {
            return LookupResult{
                .found = true,
                .rightHandWeaponLocal = entry.rightHandWeaponLocal,
                .captureSequence = entry.captureSequence,
                .usedVariantFallback = usedVariantFallback,
                .reason = usedVariantFallback ? "authoredAnimationFormFallback" : "authoredAnimationExactVariant",
            };
        }
    }

    bool publish(const RE::TESObjectWEAP* weapon, const RE::NiAVObject* weaponRoot, const bool inPowerArmor, const RE::NiTransform& rightHandWeaponLocal,
        const std::uint64_t captureSequence)
    {
        const std::uint32_t weaponFormId = weapon ? weapon->formID : 0;
        if (weaponFormId == 0 || captureSequence == 0 || !finiteTransform(rightHandWeaponLocal)) {
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
        destination->weaponFormId = weaponFormId;
        destination->variantKey = variantKey;
        destination->rightHandWeaponLocal = rightHandWeaponLocal;
        destination->captureSequence = captureSequence;
        destination->publicationOrdinal = ++s_publicationOrdinal;
        destination->inPowerArmor = inPowerArmor;
        destination->occupied = true;

        if (newIdentity) {
            ROCK_LOG_INFO(Animation, "Learned authored loose-weapon grip formID={:08X} variant={:016X} powerArmor={} capture={}", weaponFormId, variantKey,
                inPowerArmor ? "yes" : "no", captureSequence);
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
