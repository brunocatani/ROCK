#include "physics-interaction/grenade/LooseGrenadeRuntime.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"

#include "RockConfig.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BGSMod.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESDataHandler.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include <REL/Relocation.h>
#include <windows.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <mutex>

namespace rock::loose_grenade_runtime
{
    namespace
    {
        using EquipObject_t = bool (*)(
            RE::ActorEquipManager*,
            RE::Actor*,
            const RE::BGSObjectInstance&,
            std::uint32_t,
            std::uint32_t,
            const RE::BGSEquipSlot*,
            bool,
            bool,
            bool,
            bool,
            bool);

        constexpr std::uintptr_t kFuncActorEquipManagerEquipObject = 0x0E6FEA0;
        constexpr std::uint32_t kInvalidStackId = 0xFFFF'FFFFu;
        constexpr std::array<std::uint8_t, 17> kActorEquipManagerEquipObjectExpectedPrefix{
            0x4C, 0x8B, 0xDC,
            0x49, 0x89, 0x53, 0x10,
            0x55,
            0x56,
            0x41, 0x54,
            0x41, 0x57,
            0x49, 0x8D, 0x6B, 0xD9,
        };

        struct InventoryStackMatch
        {
            bool found{ false };
            bool exactRequestedStack{ false };
            bool exactInstanceData{ false };
            std::uint32_t stackId{ kInvalidStackId };
            std::uint32_t count{ 0 };
        };

        EquipObject_t s_originalEquipObject = nullptr;
        std::atomic<bool> s_equipHookInstalled{ false };
        std::mutex s_pendingEquipMutex;
        PendingEquipRequest s_pendingEquipRequest{};
        std::uint64_t s_nextRequestId{ 1 };
        thread_local bool t_insideEquipHook = false;

        class EquipHookReentryGuard
        {
        public:
            EquipHookReentryGuard() noexcept
            {
                t_insideEquipHook = true;
            }

            ~EquipHookReentryGuard() noexcept
            {
                t_insideEquipHook = false;
            }

            EquipHookReentryGuard(const EquipHookReentryGuard&) = delete;
            EquipHookReentryGuard& operator=(const EquipHookReentryGuard&) = delete;
        };

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
            if (!weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGrenade) {
                return false;
            }

            auto* projectile = resolveProjectile(weapon, instanceData);
            if (!projectile || !projectile->data.explosionType) {
                return false;
            }

            const GrenadeKind kind = classifyGrenadeSources(weapon, instanceData, projectile, objectInstanceExtra);
            const GrenadeDetonationMode mode =
                kind == GrenadeKind::Molotov ?
                    GrenadeDetonationMode::Impact :
                    GrenadeDetonationMode::TimedFuse;
            const float configuredFuseSeconds = g_rockConfig.rockRealisticGrenadeFuseSeconds;
            const float fuseSeconds = std::isfinite(configuredFuseSeconds) && configuredFuseSeconds > 0.0f ?
                configuredFuseSeconds :
                projectile->data.explosionTimer;
            if (mode == GrenadeDetonationMode::TimedFuse && (!std::isfinite(fuseSeconds) || fuseSeconds <= 0.0f)) {
                return false;
            }

            outRuntime = GrenadeRuntimeData{
                .projectile = projectile,
                .explosion = projectile->data.explosionType,
                .fuseSeconds = fuseSeconds,
                .detonationMode = mode,
            };
            return true;
        }

        [[nodiscard]] InventoryStackMatch findInventoryStack(
            RE::PlayerCharacter* player,
            RE::TESObjectWEAP* weapon,
            const RE::BSTSmartPointer<RE::TBO_InstanceData>& instanceData,
            std::uint32_t requestedStackId) noexcept
        {
            InventoryStackMatch fallback{};
            InventoryStackMatch firstCandidate{};
            std::uint32_t candidateCount = 0;
            if (!player || !weapon || !player->inventoryList) {
                return {};
            }

            const RE::BSAutoReadLock inventoryLock{ player->inventoryList->rwLock };
            for (auto& inventoryItem : player->inventoryList->data) {
                if (inventoryItem.object != weapon) {
                    continue;
                }

                std::uint32_t stackId = 0;
                for (auto* stack = inventoryItem.stackData.get(); stack; stack = stack->nextStack.get(), ++stackId) {
                    const auto count = stack->GetCount();
                    if (count == 0) {
                        continue;
                    }

                    RE::BSTSmartPointer<RE::TBO_InstanceData> stackInstanceData{};
                    if (stack->extra) {
                        if (const auto* instanceExtra = stack->extra->GetByType<RE::ExtraInstanceData>()) {
                            stackInstanceData = instanceExtra->data;
                        }
                    }

                    InventoryStackMatch candidate{
                        .found = true,
                        .exactRequestedStack = requestedStackId != kInvalidStackId && stackId == requestedStackId,
                        .exactInstanceData = instanceData && stackInstanceData.get() == instanceData.get(),
                        .stackId = stackId,
                        .count = count,
                    };

                    if (candidate.exactRequestedStack) {
                        return candidate;
                    }
                    if (candidate.exactInstanceData) {
                        fallback = candidate;
                    }
                    if (!firstCandidate.found) {
                        firstCandidate = candidate;
                    }
                    ++candidateCount;
                }
            }

            if (fallback.found) {
                return fallback;
            }
            if (!instanceData && candidateCount == 1) {
                return firstCandidate;
            }
            return {};
        }

        [[nodiscard]] bool enqueuePendingEquipRequest(
            RE::TESObjectWEAP* weapon,
            const RE::BSTSmartPointer<RE::TBO_InstanceData>& instanceData,
            std::uint32_t stackId,
            const GrenadeRuntimeData& runtime)
        {
            std::scoped_lock lock(s_pendingEquipMutex);
            if (s_pendingEquipRequest.active) {
                return false;
            }

            s_pendingEquipRequest = PendingEquipRequest{
                .active = true,
                .requestId = s_nextRequestId++,
                .weapon = weapon,
                .instanceData = instanceData,
                .stackId = stackId,
                .runtime = runtime,
            };
            return true;
        }

        [[nodiscard]] bool shouldInterceptEquip(
            RE::Actor* actor,
            const RE::BGSObjectInstance& object,
            std::uint32_t number,
            RE::TESObjectWEAP*& outWeapon,
            GrenadeRuntimeData& outRuntime) noexcept
        {
            outWeapon = nullptr;
            outRuntime = {};
            if (t_insideEquipHook || !g_rockConfig.rockEnabled || number == 0) {
                return false;
            }

            auto* player = RE::PlayerCharacter::GetSingleton();
            if (!player || actor != player || !object.object) {
                return false;
            }

            auto* weapon = object.object->As<RE::TESObjectWEAP>();
            if (!isGrenadeWeapon(weapon)) {
                return false;
            }

            outWeapon = weapon;
            static_cast<void>(resolveGrenadeRuntimeData(weapon, object.instanceData.get(), outRuntime));
            return true;
        }

        bool hookedEquipObject(
            RE::ActorEquipManager* manager,
            RE::Actor* actor,
            const RE::BGSObjectInstance& object,
            std::uint32_t stackId,
            std::uint32_t number,
            const RE::BGSEquipSlot* slot,
            bool queueEquip,
            bool forceEquip,
            bool playSounds,
            bool applyNow,
            bool locked)
        {
            RE::TESObjectWEAP* weapon = nullptr;
            GrenadeRuntimeData runtime{};
            if (shouldInterceptEquip(actor, object, number, weapon, runtime)) {
                if (!runtime.projectile || !runtime.explosion ||
                    (runtime.detonationMode == GrenadeDetonationMode::TimedFuse &&
                        (!std::isfinite(runtime.fuseSeconds) || runtime.fuseSeconds <= 0.0f))) {
                    ROCK_LOG_WARN(Hand,
                        "Blocked grenade equip because ROCK could not resolve projectile/explosion/fuse data: weapon={:08X} stack={}",
                        weapon ? weapon->GetFormID() : 0,
                        stackId);
                    return false;
                }

                if (enqueuePendingEquipRequest(weapon, object.instanceData, stackId, runtime)) {
                    ROCK_LOG_INFO(Hand,
                        "Queued loose grenade equip interception: weapon={:08X} projectile={:08X} explosion={:08X} stack={} mode={} fuse={:.3f}s",
                        weapon ? weapon->GetFormID() : 0,
                        runtime.projectile ? runtime.projectile->GetFormID() : 0,
                        runtime.explosion ? runtime.explosion->GetFormID() : 0,
                        stackId,
                        detonationModeName(runtime.detonationMode),
                        runtime.fuseSeconds);
                    return true;
                }

                /*
                 * The first request remains authoritative through its attach
                 * terminal state. Report duplicate menu presses as handled so
                 * native equip cannot run, but never preserve them for replay.
                 */
                ROCK_LOG_INFO(Hand,
                    "Ignored duplicate loose grenade equip while one transaction is active: weapon={:08X} stack={}",
                    weapon ? weapon->GetFormID() : 0,
                    stackId);
                return true;
            }

            if (!s_originalEquipObject) {
                return false;
            }

            const EquipHookReentryGuard reentryGuard;
            const bool result = s_originalEquipObject(manager, actor, object, stackId, number, slot, queueEquip, forceEquip, playSounds, applyNow, locked);
            return result;
        }

    }

    bool installEquipHook()
    {
        if (s_equipHookInstalled.load(std::memory_order_acquire)) {
            return true;
        }

        void* original = reinterpret_cast<void*>(s_originalEquipObject);
        const bool installed = entry_trampoline_hook::install(
            "ActorEquipManager::EquipObject loose grenade interception",
            kFuncActorEquipManagerEquipObject,
            kActorEquipManagerEquipObjectExpectedPrefix.data(),
            kActorEquipManagerEquipObjectExpectedPrefix.size(),
            reinterpret_cast<void*>(&hookedEquipObject),
            original);
        s_originalEquipObject = reinterpret_cast<EquipObject_t>(original);
        s_equipHookInstalled.store(installed && s_originalEquipObject != nullptr, std::memory_order_release);
        return s_equipHookInstalled.load(std::memory_order_acquire);
    }

    bool isGrenadeWeapon(const RE::TESObjectWEAP* weapon) noexcept
    {
        // WEAPON_TYPE is a single-valued enum stored in EnumSet; bitmask any() makes guns/mines alias grenades.
        return weapon && weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade;
    }

    bool isGrenadeRef(RE::TESObjectREFR* ref) noexcept
    {
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
        return isGrenadeWeapon(weapon);
    }

    GrenadeKind classifyGrenadeRef(RE::TESObjectREFR* ref) noexcept
    {
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
        if (!isGrenadeWeapon(weapon)) {
            return GrenadeKind::NotGrenade;
        }

        const auto instanceData = resolveReferenceInstanceData(ref);
        auto* projectile = resolveProjectile(weapon, instanceData.get());
        const auto* objectInstanceExtra = resolveReferenceObjectInstanceExtra(ref);
        return classifyGrenadeSources(weapon, instanceData.get(), projectile, objectInstanceExtra);
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

    bool copyPendingEquipRequest(PendingEquipRequest& outRequest)
    {
        std::scoped_lock lock(s_pendingEquipMutex);
        if (!s_pendingEquipRequest.active) {
            outRequest = {};
            return false;
        }

        outRequest = s_pendingEquipRequest;
        return true;
    }

    bool hasPendingEquipRequest()
    {
        std::scoped_lock lock(s_pendingEquipMutex);
        return s_pendingEquipRequest.active;
    }

    void discardPendingEquipRequest(std::uint64_t requestId)
    {
        std::scoped_lock lock(s_pendingEquipMutex);
        if (s_pendingEquipRequest.active && s_pendingEquipRequest.requestId == requestId) {
            s_pendingEquipRequest = {};
        }
    }

    void clearPendingEquipRequest()
    {
        std::scoped_lock lock(s_pendingEquipMutex);
        s_pendingEquipRequest = {};
    }

    DropResult dropPendingEquipRequestToWorld(
        const PendingEquipRequest& request,
        const RE::NiPoint3& dropLocation)
    {
        DropResult result{};
        result.stackId = request.stackId;
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = "missing-player";
            return result;
        }
        if (!request.active || !request.weapon) {
            result.reason = "missing-request";
            return result;
        }
        if (!player->inventoryList) {
            result.reason = "missing-inventory-list";
            return result;
        }

        const auto stack = findInventoryStack(player, request.weapon, request.instanceData, request.stackId);
        if (!stack.found || stack.count == 0 || stack.stackId == kInvalidStackId) {
            result.reason = "inventory-stack-not-found";
            return result;
        }

        RE::TESObjectREFR::RemoveItemData removeData(request.weapon, 1);
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
        case GrenadeDetonationMode::TimedFuse:
            return "timed-fuse";
        case GrenadeDetonationMode::Impact:
            return "impact";
        default:
            return "unknown";
        }
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
