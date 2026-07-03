#include "physics-interaction/weapon/LooseWeaponGripProbe.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <format>

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiNode.h"

#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "f4sevr/Forms.h"

namespace rock::loose_weapon_grip_probe
{
    namespace
    {
        /*
         * One learned firing-grip frame. All geometry is weapon-node-local:
         * the equipped first-person weapon node at capture time, assumed to
         * correspond to the dropped world-model root at resolve time. That
         * correspondence is exactly what this spike exists to confirm.
         */
        struct GripEntry
        {
            std::uint32_t weaponFormID{ 0 };
            bool inPowerArmor{ false };
            bool capturedHandIsLeft{ false };
            std::uint32_t captureCount{ 0 };
            float maxGripLocalDrift{ 0.0f };
            RE::NiPoint3 gripLocal{};
            RE::NiPoint3 firstGripLocal{};
            RE::NiTransform handWeaponLocal{};
            char weaponName[64]{};
        };

        constexpr std::size_t MAX_ENTRIES = 16;
        constexpr int CAPTURE_INTERVAL_FRAMES = 15;
        constexpr std::uint32_t DRIFT_LOG_EVERY_REFRESHES = 40;
        constexpr int PROBE_LOG_INTERVAL_FRAMES = 180;
        constexpr int PROBE_MISSING_LOG_INTERVAL_FRAMES = 600;

        std::array<GripEntry, MAX_ENTRIES> s_entries{};
        std::size_t s_nextEvictIndex = 0;
        int s_captureFrameCounter = 0;

        std::array<ResolvedGripDebug, 2> s_resolvedDebug{};
        std::array<int, 2> s_probeLogCounters{};
        std::array<int, 2> s_probeMissingLogCounters{};

        std::size_t handIndex(const bool isLeft) { return isLeft ? 0u : 1u; }

        bool isFinitePoint(const RE::NiPoint3& point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        bool isUsableWorldTransform(const RE::NiTransform& transform)
        {
            return isFinitePoint(transform.translate) && std::isfinite(transform.scale) && std::fabs(transform.scale) > 0.0001f;
        }

        float pointDistance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            const float dx = lhs.x - rhs.x;
            const float dy = lhs.y - rhs.y;
            const float dz = lhs.z - rhs.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        [[nodiscard]] RE::TESObjectWEAP* readEquippedWeaponForm() noexcept
        {
            auto* player = f4vr::getPlayer();
            auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
            auto* equipData = processData ? processData->equipData : nullptr;
            const auto* weaponForm = equipData ? equipData->item : nullptr;
            if (!weaponForm || weaponForm->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
                return nullptr;
            }

            auto* reForm = reinterpret_cast<RE::TESForm*>(const_cast<F4SEVR::TESForm*>(weaponForm));
            return reForm ? reForm->As<RE::TESObjectWEAP>() : nullptr;
        }

        [[nodiscard]] GripEntry* findEntry(const std::uint32_t weaponFormID, const bool inPowerArmor)
        {
            for (auto& entry : s_entries) {
                if (entry.weaponFormID == weaponFormID && entry.inPowerArmor == inPowerArmor) {
                    return &entry;
                }
            }
            return nullptr;
        }

        [[nodiscard]] GripEntry& claimEntry(const std::uint32_t weaponFormID, const bool inPowerArmor)
        {
            for (auto& entry : s_entries) {
                if (entry.weaponFormID == 0) {
                    return entry;
                }
            }
            auto& evicted = s_entries[s_nextEvictIndex];
            s_nextEvictIndex = (s_nextEvictIndex + 1) % MAX_ENTRIES;
            ROCK_LOG_DEBUG(Weapon,
                "LooseWeaponGripProbe: evicting learned grip formID={:08X} inPA={} for formID={:08X} inPA={}",
                evicted.weaponFormID,
                evicted.inPowerArmor ? "yes" : "no",
                weaponFormID,
                inPowerArmor ? "yes" : "no");
            evicted = {};
            return evicted;
        }
    }

    void captureFromEquippedWeapon(RE::NiNode* weaponNode, const bool firingHandIsLeft, const bool weaponTransformOwnedByRock)
    {
        if (!weaponNode || weaponTransformOwnedByRock) {
            return;
        }
        if (++s_captureFrameCounter < CAPTURE_INTERVAL_FRAMES) {
            return;
        }
        s_captureFrameCounter = 0;

        if (!isUsableWorldTransform(weaponNode->world)) {
            return;
        }

        auto* weapon = readEquippedWeaponForm();
        const std::uint32_t weaponFormID = weapon ? weapon->GetFormID() : 0u;
        if (weaponFormID == 0) {
            return;
        }

        RE::NiPoint3 palmWorld{};
        RE::NiTransform handWorld{};
        if (!TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(firingHandIsLeft, palmWorld, handWorld) || !isFinitePoint(palmWorld)) {
            return;
        }

        const RE::NiPoint3 gripLocal = transform_math::worldPointToLocal(weaponNode->world, palmWorld);
        const RE::NiTransform handWeaponLocal =
            transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), handWorld);
        if (!isFinitePoint(gripLocal)) {
            return;
        }

        const bool inPowerArmor = f4vr::isInPowerArmor();
        GripEntry* entry = findEntry(weaponFormID, inPowerArmor);
        const bool isNewEntry = entry == nullptr;
        if (isNewEntry) {
            entry = &claimEntry(weaponFormID, inPowerArmor);
            entry->weaponFormID = weaponFormID;
            entry->inPowerArmor = inPowerArmor;
            entry->firstGripLocal = gripLocal;
            const std::string weaponName = f4vr::getEquippedWeaponName();
            std::snprintf(entry->weaponName, sizeof(entry->weaponName), "%s", weaponName.c_str());
        }

        entry->capturedHandIsLeft = firingHandIsLeft;
        entry->gripLocal = gripLocal;
        entry->handWeaponLocal = handWeaponLocal;
        entry->maxGripLocalDrift = (std::max)(entry->maxGripLocalDrift, pointDistance(gripLocal, entry->firstGripLocal));
        ++entry->captureCount;

        if (isNewEntry) {
            const auto* firstChild = f4vr::getFirstChild(weaponNode);
            ROCK_LOG_INFO(Weapon,
                "LooseWeaponGripProbe: learned firing grip weapon='{}' formID={:08X} inPA={} hand={} "
                "gripLocal=({:.3f},{:.3f},{:.3f}) weaponNode='{}' nodeScale={:.3f} firstChild='{}' childLocal=({:.3f},{:.3f},{:.3f}) childScale={:.3f}",
                entry->weaponName,
                weaponFormID,
                inPowerArmor ? "yes" : "no",
                firingHandIsLeft ? "left" : "right",
                gripLocal.x,
                gripLocal.y,
                gripLocal.z,
                weaponNode->name.c_str(),
                weaponNode->world.scale,
                firstChild ? firstChild->name.c_str() : "<none>",
                firstChild ? firstChild->local.translate.x : 0.0f,
                firstChild ? firstChild->local.translate.y : 0.0f,
                firstChild ? firstChild->local.translate.z : 0.0f,
                firstChild ? firstChild->local.scale : 0.0f);
        } else if (entry->captureCount % DRIFT_LOG_EVERY_REFRESHES == 0) {
            ROCK_LOG_DEBUG(Weapon,
                "LooseWeaponGripProbe: grip refresh weapon='{}' formID={:08X} captures={} gripLocal=({:.3f},{:.3f},{:.3f}) maxDrift={:.3f}gu",
                entry->weaponName,
                weaponFormID,
                entry->captureCount,
                gripLocal.x,
                gripLocal.y,
                gripLocal.z,
                entry->maxGripLocalDrift);
        }
    }

    void updateHeldLooseWeaponProbe(const bool isLeft, const bool holdingLooseWeapon, RE::TESObjectREFR* heldRef)
    {
        auto& resolved = s_resolvedDebug[handIndex(isLeft)];
        if (!holdingLooseWeapon || !heldRef) {
            resolved = {};
            return;
        }

        auto* baseForm = heldRef->GetObjectReference();
        auto* weapon = baseForm ? baseForm->As<RE::TESObjectWEAP>() : nullptr;
        const std::uint32_t weaponFormID = weapon ? weapon->GetFormID() : 0u;
        if (weaponFormID == 0) {
            resolved = {};
            return;
        }

        const GripEntry* entry = findEntry(weaponFormID, f4vr::isInPowerArmor());
        if (!entry) {
            resolved = {};
            if (++s_probeMissingLogCounters[handIndex(isLeft)] >= PROBE_MISSING_LOG_INTERVAL_FRAMES) {
                s_probeMissingLogCounters[handIndex(isLeft)] = 0;
                ROCK_LOG_INFO(Weapon,
                    "LooseWeaponGripProbe: {} hand holds loose weapon formID={:08X} with no learned grip (equip it once to learn)",
                    isLeft ? "left" : "right",
                    weaponFormID);
            }
            return;
        }

        auto* looseRoot = heldRef->Get3D();
        if (!looseRoot || !isUsableWorldTransform(looseRoot->world)) {
            resolved = {};
            return;
        }

        ResolvedGripDebug next{};
        next.gripWorld = transform_math::localPointToWorld(looseRoot->world, entry->gripLocal);
        next.handTargetWorld = transform_math::composeTransforms(looseRoot->world, entry->handWeaponLocal);
        if (!isFinitePoint(next.gripWorld)) {
            resolved = {};
            return;
        }
        next.valid = true;

        RE::NiTransform palmHandWorld{};
        if (TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(isLeft, next.palmWorld, palmHandWorld) && isFinitePoint(next.palmWorld)) {
            next.palmValid = true;
            next.palmToGripDistance = pointDistance(next.palmWorld, next.gripWorld);
        }
        resolved = next;

        if (++s_probeLogCounters[handIndex(isLeft)] >= PROBE_LOG_INTERVAL_FRAMES) {
            s_probeLogCounters[handIndex(isLeft)] = 0;
            ROCK_LOG_INFO(Weapon,
                "LooseWeaponGripProbe: {} hand loose weapon='{}' formID={:08X} rootNode='{}' rootScale={:.3f} "
                "gripLocal=({:.3f},{:.3f},{:.3f}) gripWorld=({:.2f},{:.2f},{:.2f}) palmDist={} capturedHand={} captures={} maxDrift={:.3f}gu",
                isLeft ? "left" : "right",
                entry->weaponName,
                weaponFormID,
                looseRoot->name.c_str(),
                looseRoot->world.scale,
                entry->gripLocal.x,
                entry->gripLocal.y,
                entry->gripLocal.z,
                next.gripWorld.x,
                next.gripWorld.y,
                next.gripWorld.z,
                next.palmValid ? std::format("{:.2f}gu", next.palmToGripDistance) : "<no-palm>",
                entry->capturedHandIsLeft ? "left" : "right",
                entry->captureCount,
                entry->maxGripLocalDrift);
        }
    }

    bool tryGetResolvedGripDebug(const bool isLeft, ResolvedGripDebug& out)
    {
        const auto& resolved = s_resolvedDebug[handIndex(isLeft)];
        if (!resolved.valid) {
            return false;
        }
        out = resolved;
        return true;
    }
}
