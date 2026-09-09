#include "physics-interaction/grenade/LooseGrenadeRuntime.h"

#include "RockConfig.h"

#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/ProcessLists.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESDataHandler.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include <atomic>
#include <cmath>

namespace rock::loose_grenade_runtime
{
    namespace
    {
        constexpr std::uint32_t kInvalidStackId = 0xFFFF'FFFFu;
        constexpr std::uint32_t kMaximumProximityActorHandlesScanned = 2048;

        enum class GrenadeKind : std::uint8_t
        {
            NotGrenade,
            Generic,
            Molotov,
        };

        struct InventoryStackMatch
        {
            bool found{ false };
            bool exactInstanceData{ false };
            bool equipped{ false };
            std::uint32_t stackId{ kInvalidStackId };
            std::uint32_t count{ 0 };
        };

        std::atomic<std::uint64_t> s_nextRequestId{ 1 };

        [[nodiscard]] RE::TESObjectWEAP::InstanceData* weaponInstanceData(
            RE::TESObjectWEAP* weapon,
            RE::TBO_InstanceData* instanceData) noexcept
        {
            if (instanceData) {
                return static_cast<RE::TESObjectWEAP::InstanceData*>(instanceData);
            }
            return weapon ? &weapon->weaponData : nullptr;
        }

        [[nodiscard]] RE::BGSProjectile* resolveProjectile(
            RE::TESObjectWEAP* weapon,
            RE::TBO_InstanceData* instanceData) noexcept
        {
            auto* data = weaponInstanceData(weapon, instanceData);
            if (!weapon || !data) {
                return nullptr;
            }

            if (data->rangedData && data->rangedData->overrideProjectile) {
                return data->rangedData->overrideProjectile;
            }
            if (weapon->weaponData.rangedData && weapon->weaponData.rangedData->overrideProjectile) {
                return weapon->weaponData.rangedData->overrideProjectile;
            }
            if (data->ammo && data->ammo->data.projectile) {
                return data->ammo->data.projectile;
            }
            if (weapon->weaponData.ammo && weapon->weaponData.ammo->data.projectile) {
                return weapon->weaponData.ammo->data.projectile;
            }
            return nullptr;
        }

        [[nodiscard]] bool playObjectPickupSoundAtReference(RE::TESObjectREFR* ref)
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* object = ref ? ref->GetObjectReference() : nullptr;
            if (!player || !object) {
                return false;
            }

            player->PlayPickUpSound(object, false, true);
            return true;
        }

        [[nodiscard]] RE::BSTSmartPointer<RE::TBO_InstanceData> resolveReferenceInstanceData(RE::TESObjectREFR* ref) noexcept
        {
            if (!ref || !ref->extraList) {
                return {};
            }

            const auto* instanceExtra = ref->extraList->GetByType<RE::ExtraInstanceData>();
            return instanceExtra ? instanceExtra->data : RE::BSTSmartPointer<RE::TBO_InstanceData>{};
        }

        [[nodiscard]] const RE::BGSObjectInstanceExtra* resolveReferenceObjectInstanceExtra(RE::TESObjectREFR* ref) noexcept
        {
            if (!ref || !ref->extraList) {
                return nullptr;
            }

            return ref->extraList->GetByType<RE::BGSObjectInstanceExtra>();
        }

        [[nodiscard]] char toLowerAscii(char value) noexcept
        {
            return value >= 'A' && value <= 'Z' ? static_cast<char>(value - 'A' + 'a') : value;
        }

        [[nodiscard]] bool containsMolotovToken(const char* text) noexcept
        {
            constexpr char kMolotovToken[] = "molotov";
            if (!text || text[0] == '\0') {
                return false;
            }

            for (const char* cursor = text; *cursor != '\0'; ++cursor) {
                const char* haystack = cursor;
                const char* needle = kMolotovToken;
                while (*needle != '\0' && *haystack != '\0' && toLowerAscii(*haystack) == *needle) {
                    ++haystack;
                    ++needle;
                }
                if (*needle == '\0') {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool keywordFormHasMolotovToken(const RE::BGSKeywordForm* keywordForm) noexcept
        {
            if (!keywordForm || !keywordForm->keywords) {
                return false;
            }

            for (std::uint32_t index = 0; index < keywordForm->numKeywords; ++index) {
                const auto* keyword = keywordForm->keywords[index];
                if (keyword && containsMolotovToken(keyword->formEditorID.c_str())) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool objectInstanceExtraHasMolotovOmod(const RE::BGSObjectInstanceExtra* objectInstanceExtra) noexcept
        {
            if (!objectInstanceExtra || !objectInstanceExtra->values) {
                return false;
            }

            for (const auto& modIndex : objectInstanceExtra->GetIndexData()) {
                if (modIndex.disabled) {
                    continue;
                }

                const auto* omod = RE::TESForm::GetFormByID<RE::BGSMod::Attachment::Mod>(modIndex.objectID);
                if (!omod) {
                    continue;
                }
                if (containsMolotovToken(omod->fullName.c_str()) || containsMolotovToken(omod->model.c_str())) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool isMolotovGrenade(
            RE::TESObjectWEAP* weapon,
            RE::TBO_InstanceData* instanceData,
            RE::BGSProjectile* projectile,
            const RE::BGSObjectInstanceExtra* objectInstanceExtra) noexcept
        {
            if (!weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGrenade) {
                return false;
            }

            /*
             * WEAPON_TYPE only says "grenade". Molotov identity is authored on
             * the weapon/instance records and, for modded variants, the active
             * OMOD list. Keep the check local and token-based so unknown
             * records fail closed to normal timed-fuse behavior.
             */
            if (containsMolotovToken(weapon->fullName.c_str()) || keywordFormHasMolotovToken(weapon)) {
                return true;
            }
            if (instanceData && keywordFormHasMolotovToken(instanceData->GetKeywordData())) {
                return true;
            }
            if (projectile && (containsMolotovToken(projectile->fullName.c_str()) || containsMolotovToken(projectile->model.c_str()))) {
                return true;
            }
            return objectInstanceExtraHasMolotovOmod(objectInstanceExtra);
        }

        [[nodiscard]] GrenadeKind classifyGrenadeSources(
            RE::TESObjectWEAP* weapon,
            RE::TBO_InstanceData* instanceData,
            RE::BGSProjectile* projectile,
            const RE::BGSObjectInstanceExtra* objectInstanceExtra) noexcept
        {
            if (!weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGrenade) {
                return GrenadeKind::NotGrenade;
            }
            return isMolotovGrenade(weapon, instanceData, projectile, objectInstanceExtra) ?
                       GrenadeKind::Molotov :
                       GrenadeKind::Generic;
        }

        [[nodiscard]] bool resolveGrenadeRuntimeDataForSources(
            RE::TESObjectWEAP* weapon,
            RE::TBO_InstanceData* instanceData,
            const RE::BGSObjectInstanceExtra* objectInstanceExtra,
            GrenadeRuntimeData& outRuntime) noexcept
        {
            outRuntime = {};
            if (!isThrowableWeapon(weapon)) {
                return false;
            }

            auto* projectile = resolveProjectile(weapon, instanceData);
            if (!projectile || !projectile->data.explosionType) {
                return false;
            }

            const auto weaponType = weapon->weaponData.type.get();
            const GrenadeKind kind = classifyGrenadeSources(
                weapon,
                instanceData,
                projectile,
                objectInstanceExtra);
            const auto policyMode = loose_throwable_policy::classifyDetonationMode(
                weaponType,
                RE::WEAPON_TYPE::kGrenade,
                RE::WEAPON_TYPE::kMine,
                kind == GrenadeKind::Molotov,
                true,
                projectile->data.explosionProximity);
            const GrenadeDetonationMode mode = [&]() {
                switch (policyMode) {
                case loose_throwable_policy::DetonationMode::TimedFuse:
                    return GrenadeDetonationMode::TimedFuse;
                case loose_throwable_policy::DetonationMode::Impact:
                    return GrenadeDetonationMode::Impact;
                case loose_throwable_policy::DetonationMode::Proximity:
                    return GrenadeDetonationMode::Proximity;
                case loose_throwable_policy::DetonationMode::Unsupported:
                default:
                    return GrenadeDetonationMode::Unsupported;
                }
            }();
            if (mode == GrenadeDetonationMode::Unsupported) {
                return false;
            }

            float fuseSeconds = 0.0f;
            if (mode == GrenadeDetonationMode::TimedFuse) {
                const float configuredFuseSeconds = g_rockConfig.rockRealisticGrenadeFuseSeconds;
                fuseSeconds = std::isfinite(configuredFuseSeconds) && configuredFuseSeconds > 0.0f ?
                                  configuredFuseSeconds :
                                  projectile->data.explosionTimer;
            } else if (mode == GrenadeDetonationMode::Proximity) {
                // For placed mines this is an arming delay, not a detonation deadline.
                fuseSeconds = std::isfinite(projectile->data.explosionTimer) && projectile->data.explosionTimer > 0.0f ?
                                  projectile->data.explosionTimer :
                                  0.0f;
            }
            if (mode == GrenadeDetonationMode::TimedFuse && (!std::isfinite(fuseSeconds) || fuseSeconds <= 0.0f)) {
                return false;
            }

            outRuntime = GrenadeRuntimeData{
                .projectile = projectile,
                .explosion = projectile->data.explosionType,
                .fuseSeconds = fuseSeconds,
                .proximityRadiusGameUnits = mode == GrenadeDetonationMode::Proximity ?
                                                projectile->data.explosionProximity :
                                                0.0f,
                .directImpactDamage =
                    mode == GrenadeDetonationMode::Impact && weaponType == RE::WEAPON_TYPE::kMine ?
                        static_cast<float>(weaponInstanceData(weapon, instanceData)->attackDamage) :
                        0.0f,
                .preserveReferenceAfterDetonation =
                    loose_throwable_policy::preservesReferenceAfterDetonation(
                        policyMode,
                        projectile->data.flags,
                        projectile->data.explosionType->data.impactPlacedObject != nullptr),
                .detonationMode = mode,
            };
            return true;
        }

        [[nodiscard]] InventoryStackMatch findExactInventoryStack(
            RE::PlayerCharacter* player,
            RE::TESObjectWEAP* weapon,
            const RE::BSTSmartPointer<RE::TBO_InstanceData>& instanceData,
            std::uint32_t requestedStackId) noexcept
        {
            if (!player || !weapon || requestedStackId == kInvalidStackId || !player->inventoryList) {
                return {};
            }

            const RE::BSAutoReadLock inventoryLock{ player->inventoryList->rwLock };
            for (auto& inventoryItem : player->inventoryList->data) {
                if (inventoryItem.object != weapon) {
                    continue;
                }

                std::uint32_t stackId = 0;
                for (auto* stack = inventoryItem.stackData.get(); stack; stack = stack->nextStack.get(), ++stackId) {
                    if (stackId != requestedStackId) {
                        continue;
                    }

                    RE::BSTSmartPointer<RE::TBO_InstanceData> stackInstanceData{};
                    if (stack->extra) {
                        if (const auto* instanceExtra = stack->extra->GetByType<RE::ExtraInstanceData>()) {
                            stackInstanceData = instanceExtra->data;
                        }
                    }

                    return InventoryStackMatch{
                        .found = true,
                        .exactInstanceData = stackInstanceData.get() == instanceData.get(),
                        .equipped = stack->IsEquipped(),
                        .stackId = stackId,
                        .count = stack->GetCount(),
                    };
                }
            }
            return {};
        }
    }

    bool isThrowableWeapon(const RE::TESObjectWEAP* weapon) noexcept
    {
        if (!weapon) {
            return false;
        }
        // WEAPON_TYPE is single-valued. Explicit equality keeps guns from aliasing thrown types.
        return loose_throwable_policy::isSupportedWeaponType(
            weapon->weaponData.type.get(),
            RE::WEAPON_TYPE::kGrenade,
            RE::WEAPON_TYPE::kMine);
    }

    bool isThrowableRef(RE::TESObjectREFR* ref) noexcept
    {
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
        return isThrowableWeapon(weapon);
    }

    bool resolveGrenadeRuntimeData(RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instanceData, GrenadeRuntimeData& outRuntime) noexcept
    {
        return resolveGrenadeRuntimeDataForSources(weapon, instanceData, nullptr, outRuntime);
    }

    bool resolveGrenadeRuntimeDataForReference(RE::TESObjectREFR* ref, GrenadeRuntimeData& outRuntime) noexcept
    {
        outRuntime = {};
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
        const auto instanceData = resolveReferenceInstanceData(ref);
        const auto* objectInstanceExtra = resolveReferenceObjectInstanceExtra(ref);
        return resolveGrenadeRuntimeDataForSources(weapon, instanceData.get(), objectInstanceExtra, outRuntime);
    }

    EquippedGrenadeSelectionStatus resolveEquippedGrenadeSelection(
        EquippedGrenadeSelection& outSelection) noexcept
    {
        outSelection = {};
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player || !player->inventoryList) {
            return EquippedGrenadeSelectionStatus::PlayerUnavailable;
        }

        RE::TESObjectWEAP* selectedWeapon = nullptr;
        RE::BSTSmartPointer<RE::TBO_InstanceData> selectedInstanceData{};
        RE::BSTSmartPointer<RE::ExtraDataList> selectedExtraList{};
        std::uint32_t selectedStackId = kInvalidStackId;
        std::uint32_t equippedGrenadeStackCount = 0;

        {
            const RE::BSAutoReadLock inventoryLock{ player->inventoryList->rwLock };
            for (auto& inventoryItem : player->inventoryList->data) {
                auto* weapon = inventoryItem.object ? inventoryItem.object->As<RE::TESObjectWEAP>() : nullptr;
                if (!isThrowableWeapon(weapon)) {
                    continue;
                }

                std::uint32_t stackId = 0;
                for (auto* stack = inventoryItem.stackData.get(); stack; stack = stack->nextStack.get(), ++stackId) {
                    if (stack->GetCount() == 0 || !stack->IsEquipped()) {
                        continue;
                    }

                    ++equippedGrenadeStackCount;
                    if (equippedGrenadeStackCount > 1) {
                        continue;
                    }

                    selectedWeapon = weapon;
                    selectedExtraList = stack->extra;
                    selectedStackId = stackId;
                    if (selectedExtraList) {
                        if (const auto* instanceExtra = selectedExtraList->GetByType<RE::ExtraInstanceData>()) {
                            selectedInstanceData = instanceExtra->data;
                        }
                    }
                }
            }
        }

        if (equippedGrenadeStackCount == 0) {
            return EquippedGrenadeSelectionStatus::NoneEquipped;
        }
        if (equippedGrenadeStackCount != 1 || !selectedWeapon || selectedStackId == kInvalidStackId) {
            return EquippedGrenadeSelectionStatus::AmbiguousEquipped;
        }

        const auto* objectInstanceExtra =
            selectedExtraList ? selectedExtraList->GetByType<RE::BGSObjectInstanceExtra>() : nullptr;
        GrenadeRuntimeData runtime{};
        if (!resolveGrenadeRuntimeDataForSources(
                selectedWeapon,
                selectedInstanceData.get(),
                objectInstanceExtra,
                runtime)) {
            return EquippedGrenadeSelectionStatus::InvalidRuntimeData;
        }

        /*
         * FO4VR 1.2.72 raw disassembly establishes the native selection
         * contract used here: BGSInventoryItem::Stack stores next/extra/count/
         * flags at +0x10/+0x18/+0x20/+0x24 (constructor 0x1401AD6A0), and both
         * Pip-Boy equip state (0x140C1E9A0) and native equipped-stack traversal
         * (0x1401B1740) test flags & 7. Resolve that exact selected stack only
         * on the B-button edge; no persistent cache can become stale.
         */
        outSelection = EquippedGrenadeSelection{
            .requestId = s_nextRequestId.fetch_add(1, std::memory_order_relaxed),
            .weapon = selectedWeapon,
            .instanceData = selectedInstanceData,
            .stackId = selectedStackId,
            .runtime = runtime,
        };
        return EquippedGrenadeSelectionStatus::Selected;
    }

    const char* selectionStatusName(EquippedGrenadeSelectionStatus status) noexcept
    {
        switch (status) {
        case EquippedGrenadeSelectionStatus::Selected:
            return "selected";
        case EquippedGrenadeSelectionStatus::PlayerUnavailable:
            return "player-unavailable";
        case EquippedGrenadeSelectionStatus::NoneEquipped:
            return "none-equipped";
        case EquippedGrenadeSelectionStatus::AmbiguousEquipped:
            return "ambiguous-equipped";
        case EquippedGrenadeSelectionStatus::InvalidRuntimeData:
            return "invalid-runtime-data";
        default:
            return "unknown";
        }
    }

    DropResult dropEquippedGrenadeSelectionToWorld(
        const EquippedGrenadeSelection& selection,
        const RE::NiPoint3& dropLocation)
    {
        DropResult result{};
        result.stackId = selection.stackId;
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = "missing-player";
            return result;
        }
        if (selection.requestId == 0 || !selection.weapon) {
            result.reason = "missing-selection";
            return result;
        }
        if (!player->inventoryList) {
            result.reason = "missing-inventory-list";
            return result;
        }

        const auto stack = findExactInventoryStack(
            player,
            selection.weapon,
            selection.instanceData,
            selection.stackId);
        if (!stack.found || !stack.exactInstanceData || !stack.equipped ||
            stack.count == 0 || stack.stackId == kInvalidStackId) {
            result.reason = "selection-changed";
            return result;
        }

        /*
         * The native count modifier at 0x1401B01E0 changes only stack+0x20;
         * it does not clear the equipped bits at +0x24 while the stack still
         * has items. KDropping exactly one therefore preserves Bethesda's
         * selected grenade until its final inventory count is consumed.
         */
        RE::TESObjectREFR::RemoveItemData removeData(selection.weapon, 1);
        removeData.reason = RE::ITEM_REMOVE_REASON::KDropping;
        removeData.dropLoc = &dropLocation;
        removeData.stackData.push_back(stack.stackId);

        result.stackId = stack.stackId;
        result.handle = player->RemoveItem(removeData);
        if (!result.handle) {
            result.reason = "remove-item-failed";
            return result;
        }

        const auto droppedRef = result.handle.get();
        result.droppedRef = droppedRef.get();
        if (!result.droppedRef) {
            /*
             * RemoveItem already committed the inventory-to-world transfer.
             * The reference/3D can resolve asynchronously, so retain the
             * handle and let the force-grab transaction wait for it.
             */
            result.success = true;
            result.reason = "dropped-reference-pending";
            return result;
        }

        result.success = true;
        result.reason = "dropped";
        return result;
    }

    DropResult dropInventoryItemToWorld(std::uint32_t baseFormId, const RE::NiPoint3& dropLocation)
    {
        DropResult result{};
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* form = RE::TESForm::GetFormByID(baseFormId);
        auto* object = form ? form->As<RE::TESBoundObject>() : nullptr;
        if (!player || !player->inventoryList || !object ||
            !object->GetPlayable(object->GetBaseInstanceData()) ||
            (!object->Is(RE::ENUM_FORM_ID::kALCH) &&
                !(object->Is(RE::ENUM_FORM_ID::kWEAP) && isThrowableWeapon(static_cast<RE::TESObjectWEAP*>(object))))) {
            result.reason = "unsupported-inventory-item";
            return result;
        }
        std::uint32_t chosenStack = kInvalidStackId;
        {
            const RE::BSAutoReadLock lock{ player->inventoryList->rwLock };
            std::uint32_t scanned = 0;
            for (const auto& entry : player->inventoryList->data) {
                if (++scanned > 16384) {
                    break;
                }
                if (entry.object != object) {
                    continue;
                }
                std::uint32_t stackId = 0;
                for (auto* stack = entry.stackData.get(); stack && stackId < 4096; stack = stack->nextStack.get(), ++stackId) {
                    if (stack->GetCount() > 0) {
                        chosenStack = stackId;
                        break;
                    }
                }
                break;
            }
        }
        if (chosenStack == kInvalidStackId) {
            result.reason = "item-no-longer-owned";
            return result;
        }
        // Same native, instance-preserving transfer as grenade quick draw.
        // Release the inventory read lock before the engine mutates its stacks.
        RE::TESObjectREFR::RemoveItemData removeData(object, 1);
        removeData.reason = RE::ITEM_REMOVE_REASON::KDropping;
        removeData.dropLoc = &dropLocation;
        removeData.stackData.push_back(chosenStack);
        result.stackId = chosenStack;
        result.handle = player->RemoveItem(removeData);
        result.success = static_cast<bool>(result.handle);
        const auto dropped = result.handle.get();
        result.droppedRef = dropped.get();
        result.reason = result.success ? "dropped" : "remove-item-failed";
        return result;
    }

    bool createExplosionAtReference(RE::TESObjectREFR* ref, RE::BGSExplosion* explosion)
    {
        if (!ref || !explosion) {
            return false;
        }

        auto* dataHandler = RE::TESDataHandler::GetSingleton();
        auto* cell = ref->GetParentCell();
        if (!dataHandler || !cell) {
            return false;
        }

        RE::NEW_REFR_DATA data{};
        data.location = ref->Get3D() ? ref->Get3D()->world.translate : ref->data.location;
        data.direction = ref->data.angle;
        data.object = explosion;
        data.interior = cell->IsInterior() ? cell : nullptr;
        data.world = cell->IsExterior() ? cell->worldSpace : nullptr;
        data.clearStillLoadingFlag = true;
        data.initializeScripts = true;

        const auto handle = dataHandler->CreateReferenceAtLocation(data);
        return handle.get() != nullptr;
    }

    const char* detonationModeName(GrenadeDetonationMode mode) noexcept
    {
        switch (mode) {
        case GrenadeDetonationMode::Unsupported:
            return "unsupported";
        case GrenadeDetonationMode::TimedFuse:
            return "timed-fuse";
        case GrenadeDetonationMode::Impact:
            return "impact";
        case GrenadeDetonationMode::Proximity:
            return "proximity";
        default:
            return "unknown";
        }
    }

    ProximityScanResult scanHostileActorsWithinProximity(
        RE::TESObjectREFR* ref,
        float radiusGameUnits) noexcept
    {
        ProximityScanResult result{};
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* processLists = RE::ProcessLists::GetSingleton();
        auto* refCell = ref ? ref->GetParentCell() : nullptr;
        if (!ref || ref->IsDeleted() || ref->IsDisabled() ||
            !player || !processLists || !refCell ||
            !std::isfinite(radiusGameUnits) || radiusGameUnits <= 0.0f) {
            return result;
        }

        const RE::NiPoint3 source = ref->GetPosition();
        auto scanHandles = [&](const RE::BSTArray<RE::ActorHandle>& handles) {
            for (const auto& handle : handles) {
                if (result.actorHandlesScanned >= kMaximumProximityActorHandlesScanned) {
                    result.truncated = true;
                    return true;
                }
                ++result.actorHandlesScanned;

                const auto actorPtr = handle.get();
                auto* actor = actorPtr.get();
                if (!actor || actor == player || actor->IsDeleted() || actor->IsDisabled() || actor->IsDead(false)) {
                    continue;
                }

                auto* actorCell = actor->GetParentCell();
                if (!actorCell) {
                    continue;
                }
                if (refCell->IsInterior() || actorCell->IsInterior()) {
                    if (actorCell != refCell) {
                        continue;
                    }
                } else if (actorCell->worldSpace != refCell->worldSpace) {
                    continue;
                }

                const RE::NiPoint3 target = actor->GetPosition();
                if (!loose_throwable_policy::isWithinProximity(
                        radiusGameUnits,
                        target.x - source.x,
                        target.y - source.y,
                        target.z - source.z)) {
                    continue;
                }
                if (!player->GetHostileToActor(actor) && !actor->GetHostileToActor(player)) {
                    continue;
                }

                result.targetFound = true;
                result.targetActorFormID = actor->GetFormID();
                return true;
            }
            return false;
        };

        if (scanHandles(processLists->highActorHandles) ||
            scanHandles(processLists->middleHighActorHandles) ||
            scanHandles(processLists->middleLowActorHandles) ||
            scanHandles(processLists->lowActorHandles)) {
            return result;
        }
        return result;
    }

    bool playPinPulledFeedbackAtReference(RE::TESObjectREFR* ref)
    {
        return playObjectPickupSoundAtReference(ref);
    }

    bool returnDroppedReferenceToInventory(RE::TESObjectREFR* ref)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player || !ref || ref->IsDeleted() || ref->IsDisabled()) {
            return false;
        }

        /*
         * Native activation is the same locally established loose-weapon
         * pickup transfer used by ROCK's equip path. It moves this exact
         * reference (including its instance data) back into player inventory.
         */
        return ref->ActivateRef(player, nullptr, 1, false, false, false);
    }

    void disableAndDeleteReference(RE::TESObjectREFR* ref)
    {
        if (!ref || ref->IsDeleted()) {
            return;
        }
        if (!ref->IsDisabled()) {
            ref->Disable();
        }
        ref->SetWantsDelete(true);
    }
}
