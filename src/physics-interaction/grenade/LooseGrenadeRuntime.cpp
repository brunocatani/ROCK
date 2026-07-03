#include "physics-interaction/grenade/LooseGrenadeRuntime.h"

#include "physics-interaction/PhysicsLog.h"

#include "RockConfig.h"

#include "RE/Bethesda/Actor.h"
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
        constexpr std::size_t kPendingEquipCapacity = 4;
        constexpr DWORD kPageExecuteRead = 0x20u;
        constexpr DWORD kPageExecuteReadWrite = 0x40u;
        constexpr DWORD kVirtualMemoryCommitReserve = MEM_COMMIT | MEM_RESERVE;
        constexpr DWORD kVirtualMemoryRelease = MEM_RELEASE;
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
        std::array<PendingEquipRequest, kPendingEquipCapacity> s_pendingEquipRequests{};
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
            for (auto& request : s_pendingEquipRequests) {
                if (request.active) {
                    continue;
                }

                request = PendingEquipRequest{
                    .active = true,
                    .requestId = s_nextRequestId++,
                    .weapon = weapon,
                    .instanceData = instanceData,
                    .stackId = stackId,
                    .runtime = runtime,
                };
                return true;
            }
            return false;
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
                if (!runtime.projectile || !runtime.explosion || !std::isfinite(runtime.fuseSeconds) || runtime.fuseSeconds <= 0.0f) {
                    ROCK_LOG_WARN(Hand,
                        "Blocked grenade equip because ROCK could not resolve projectile/explosion/fuse data: weapon={:08X} stack={}",
                        weapon ? weapon->GetFormID() : 0,
                        stackId);
                    return false;
                }

                if (enqueuePendingEquipRequest(weapon, object.instanceData, stackId, runtime)) {
                    ROCK_LOG_INFO(Hand,
                        "Queued loose grenade equip interception: weapon={:08X} projectile={:08X} explosion={:08X} stack={} fuse={:.3f}s",
                        weapon ? weapon->GetFormID() : 0,
                        runtime.projectile ? runtime.projectile->GetFormID() : 0,
                        runtime.explosion ? runtime.explosion->GetFormID() : 0,
                        stackId,
                        runtime.fuseSeconds);
                    return true;
                }

                ROCK_LOG_WARN(Hand,
                    "Blocked grenade equip because pending loose-grenade queue is full: weapon={:08X} stack={}",
                    weapon ? weapon->GetFormID() : 0,
                    stackId);
                return false;
            }

            if (!s_originalEquipObject) {
                return false;
            }

            const EquipHookReentryGuard reentryGuard;
            const bool result = s_originalEquipObject(manager, actor, object, stackId, number, slot, queueEquip, forceEquip, playSounds, applyNow, locked);
            return result;
        }

        void writeAbsoluteJump(std::uint8_t* target, std::uintptr_t destination)
        {
            target[0] = 0xFF;
            target[1] = 0x25;
            target[2] = 0x00;
            target[3] = 0x00;
            target[4] = 0x00;
            target[5] = 0x00;
            *reinterpret_cast<std::uintptr_t*>(target + 6) = destination;
        }

        [[nodiscard]] bool installEntryTrampolineHook(const char* label,
            std::uintptr_t targetOffset,
            const std::uint8_t* expectedPrefix,
            std::size_t stolenBytes,
            void* hook,
            void*& original)
        {
            if (stolenBytes < 14) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: stolen byte count {} cannot hold an absolute jump", label, stolenBytes);
                return false;
            }

            REL::Relocation<std::uintptr_t> target{ REL::Offset(targetOffset) };
            auto* targetAddr = reinterpret_cast<std::uint8_t*>(target.address());
            if (!targetAddr || !expectedPrefix) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: target or validation bytes are null", label);
                return false;
            }

            if (std::memcmp(targetAddr, expectedPrefix, stolenBytes) != 0) {
                ROCK_LOG_ERROR(Init, "{} hook validation failed at 0x{:X}; native bytes changed, hook not installed", label, target.address());
                return false;
            }

            constexpr std::size_t kJumpBytes = 14;
            const std::size_t trampolineBytes = stolenBytes + kJumpBytes;
            auto* trampolineMem = reinterpret_cast<std::uint8_t*>(VirtualAlloc(nullptr, trampolineBytes, kVirtualMemoryCommitReserve, kPageExecuteReadWrite));
            if (!trampolineMem) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline allocation failed", label);
                return false;
            }

            std::memcpy(trampolineMem, targetAddr, stolenBytes);
            writeAbsoluteJump(trampolineMem + stolenBytes, target.address() + stolenBytes);

            DWORD oldTrampolineProtect = 0;
            if (!VirtualProtect(trampolineMem, trampolineBytes, kPageExecuteRead, &oldTrampolineProtect)) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline protection failed", label);
                VirtualFree(trampolineMem, 0, kVirtualMemoryRelease);
                return false;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(targetAddr, stolenBytes, kPageExecuteReadWrite, &oldProtect)) {
                ROCK_LOG_ERROR(Init, "{} hook install failed at 0x{:X}: target protection failed", label, target.address());
                VirtualFree(trampolineMem, 0, kVirtualMemoryRelease);
                return false;
            }

            writeAbsoluteJump(targetAddr, reinterpret_cast<std::uintptr_t>(hook));
            for (std::size_t i = kJumpBytes; i < stolenBytes; ++i) {
                targetAddr[i] = 0x90;
            }

            FlushInstructionCache(GetCurrentProcess(), targetAddr, stolenBytes);
            VirtualProtect(targetAddr, stolenBytes, oldProtect, &oldProtect);

            original = trampolineMem;
            ROCK_LOG_INFO(Init, "Installed {} hook at 0x{:X}, original trampoline=0x{:X}", label, target.address(), reinterpret_cast<std::uintptr_t>(trampolineMem));
            return true;
        }
    }

    bool installEquipHook()
    {
        if (s_equipHookInstalled.load(std::memory_order_acquire)) {
            return true;
        }

        void* original = reinterpret_cast<void*>(s_originalEquipObject);
        const bool installed = installEntryTrampolineHook(
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

    bool resolveGrenadeRuntimeData(RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instanceData, GrenadeRuntimeData& outRuntime) noexcept
    {
        outRuntime = {};
        if (!isGrenadeWeapon(weapon)) {
            return false;
        }

        auto* projectile = resolveProjectile(weapon, instanceData);
        if (!projectile || !projectile->data.explosionType) {
            return false;
        }

        const float configuredFuseSeconds = g_rockConfig.rockRealisticGrenadeFuseSeconds;
        const float fuseSeconds = std::isfinite(configuredFuseSeconds) && configuredFuseSeconds > 0.0f ?
            configuredFuseSeconds :
            projectile->data.explosionTimer;
        if (!std::isfinite(fuseSeconds) || fuseSeconds <= 0.0f) {
            return false;
        }

        outRuntime = GrenadeRuntimeData{
            .projectile = projectile,
            .explosion = projectile->data.explosionType,
            .fuseSeconds = fuseSeconds,
        };
        return true;
    }

    bool resolveGrenadeRuntimeDataForReference(RE::TESObjectREFR* ref, GrenadeRuntimeData& outRuntime) noexcept
    {
        outRuntime = {};
        auto* base = ref ? ref->GetObjectReference() : nullptr;
        auto* weapon = base ? base->As<RE::TESObjectWEAP>() : nullptr;
        const auto instanceData = resolveReferenceInstanceData(ref);
        return resolveGrenadeRuntimeData(weapon, instanceData.get(), outRuntime);
    }

    bool copyOldestPendingEquipRequest(PendingEquipRequest& outRequest)
    {
        std::scoped_lock lock(s_pendingEquipMutex);
        PendingEquipRequest* oldest = nullptr;
        for (auto& request : s_pendingEquipRequests) {
            if (!request.active) {
                continue;
            }
            if (!oldest || request.requestId < oldest->requestId) {
                oldest = &request;
            }
        }
        if (!oldest) {
            outRequest = {};
            return false;
        }

        outRequest = *oldest;
        return true;
    }

    void discardPendingEquipRequest(std::uint64_t requestId)
    {
        std::scoped_lock lock(s_pendingEquipMutex);
        for (auto& request : s_pendingEquipRequests) {
            if (request.active && request.requestId == requestId) {
                request = {};
                return;
            }
        }
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
            result.reason = "dropped-reference-unavailable";
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

    bool playPinPulledFeedbackAtReference(RE::TESObjectREFR* ref)
    {
        return playObjectPickupSoundAtReference(ref);
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
