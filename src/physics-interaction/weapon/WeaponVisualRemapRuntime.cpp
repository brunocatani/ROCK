#include "physics-interaction/weapon/WeaponVisualRemapRuntime.h"

#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponInstanceWitnessRuntime.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TaskQueueInterface.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include "f4sevr/Forms.h"

#include <algorithm>
#include <optional>
#include <type_traits>
#include <vector>

namespace rock::weapon_visual_remap_runtime
{
    namespace
    {
        struct EquippedWeaponSnapshot
        {
            /*
             * Equipped snapshots are deliberately non-owning. BGSObjectInstance
             * owns its instance-data reference through BSTSmartPointer; copying
             * it into remap candidate storage makes ROCK run destructors over
             * engine-managed equip state. Keep only witness pointers here and
             * build a borrowed queue view at the native call boundary.
             */
            RE::TESForm* weaponForm = nullptr;
            RE::TESObjectWEAP* weapon = nullptr;
            RE::TBO_InstanceData* instanceData = nullptr;
            const RE::BGSObjectInstanceExtra* objectInstanceExtra = nullptr;
            RE::BGSEquipIndex equipIndex{};
            std::uint32_t weaponFormID = 0;
            std::uintptr_t weaponFormAddress = 0;
            std::uintptr_t instanceDataAddress = 0;
            std::uintptr_t objectInstanceExtraAddress = 0;
            std::uint32_t sourceSlotIndex = 0;
            const char* source = "";
        };

        struct BorrowedWeaponInstance
        {
            RE::TESForm* object = nullptr;
            RE::TBO_InstanceData* instanceData = nullptr;
        };

        static_assert(std::is_standard_layout_v<BorrowedWeaponInstance>);
        static_assert(sizeof(BorrowedWeaponInstance) == sizeof(RE::BGSObjectInstanceT<RE::TESObjectWEAP>));
        static_assert(alignof(BorrowedWeaponInstance) == alignof(RE::BGSObjectInstanceT<RE::TESObjectWEAP>));
        static_assert(sizeof(RE::BGSObjectInstanceT<RE::TESObjectWEAP>) == sizeof(RE::BGSObjectInstance));

        RequestOutcome makeOutcome(
            RequestResult result,
            std::uint64_t pendingInstanceSignature,
            const char* detail,
            const EquippedWeaponSnapshot* snapshot = nullptr)
        {
            return RequestOutcome{
                .result = result,
                .pendingInstanceSignature = pendingInstanceSignature,
                .weaponFormID = snapshot ? snapshot->weaponFormID : 0,
                .weaponFormAddress = snapshot ? snapshot->weaponFormAddress : 0,
                .instanceDataAddress = snapshot ? snapshot->instanceDataAddress : 0,
                .objectInstanceExtraAddress = snapshot ? snapshot->objectInstanceExtraAddress : 0,
                .equipIndex = snapshot ? snapshot->equipIndex.index : 0,
                .sourceSlotIndex = snapshot ? snapshot->sourceSlotIndex : 0,
                .source = snapshot ? snapshot->source : "",
                .detail = detail,
            };
        }

        std::uintptr_t findObjectInstanceExtraAddress(
            const F4SEVR::PlayerCharacter* player,
            const RE::TESForm* weaponForm,
            const RE::TBO_InstanceData* instanceData)
        {
            if (!player || !weaponForm) {
                return 0;
            }

            auto scanEquipData = [&](const F4SEVR::ActorEquipData* equipData) -> std::uintptr_t {
                if (!equipData) {
                    return 0;
                }

                for (std::uint32_t slotIndex = 0; slotIndex < F4SEVR::ActorEquipData::kMaxSlots; ++slotIndex) {
                    const auto& slot = equipData->slots[slotIndex];
                    if (slot.item != weaponForm) {
                        continue;
                    }
                    if (instanceData && slot.instanceData && slot.instanceData != instanceData) {
                        continue;
                    }
                    if (slot.extraData) {
                        return reinterpret_cast<std::uintptr_t>(slot.extraData);
                    }
                }

                return 0;
            };

            if (const auto firstPersonExtra = scanEquipData(player->playerEquipData)) {
                return firstPersonExtra;
            }
            return scanEquipData(player->equipData);
        }

        RE::BGSEquipIndex findCompatibleEquipIndex(
            const std::vector<EquippedWeaponSnapshot>& snapshots,
            const RE::TESForm* weaponForm)
        {
            RE::BGSEquipIndex equipIndex{};
            if (!weaponForm) {
                return equipIndex;
            }

            for (const auto& snapshot : snapshots) {
                if (snapshot.weaponForm == weaponForm) {
                    return snapshot.equipIndex;
                }
            }

            return equipIndex;
        }

        std::optional<RE::BGSEquipIndex> findLiveCompatibleEquipIndex(
            const std::vector<EquippedWeaponSnapshot>& snapshots,
            const RE::TESForm* weaponForm)
        {
            if (!weaponForm) {
                return std::nullopt;
            }

            for (const auto& snapshot : snapshots) {
                if (snapshot.weaponForm == weaponForm) {
                    return snapshot.equipIndex;
                }
            }

            return std::nullopt;
        }

        bool snapshotAlreadyCaptured(
            const std::vector<EquippedWeaponSnapshot>& snapshots,
            const RE::TESForm* weaponForm,
            const RE::TBO_InstanceData* instanceData,
            const RE::BGSObjectInstanceExtra* objectInstanceExtra)
        {
            for (const auto& snapshot : snapshots) {
                if (snapshot.weaponForm == weaponForm &&
                    snapshot.instanceData == instanceData &&
                    snapshot.objectInstanceExtra == objectInstanceExtra) {
                    return true;
                }
            }

            return false;
        }

        void appendMiddleHighEquippedSnapshots(
            std::vector<EquippedWeaponSnapshot>& snapshots,
            RE::Actor& actor)
        {
            auto* process = actor.currentProcess;
            auto* middleHigh = process ? process->middleHigh : nullptr;
            if (!middleHigh) {
                return;
            }

            RE::BSAutoLock lock{ middleHigh->equippedItemsLock };
            const auto itemCount = middleHigh->equippedItems.size();
            const auto* items = middleHigh->equippedItems.data();
            if (itemCount == 0 || !items) {
                return;
            }

            /*
             * The equipped item array can be observed during first-person
             * skeleton teardown/rebuild. Avoid range iteration because a
             * transient null data pointer with stale size turns begin/end into
             * an invalid dereference before ROCK can fall back to equip data.
             */
            constexpr std::uint32_t kMaxQueueableEquippedItems = 8;
            const auto scanCount = (std::min)(itemCount, kMaxQueueableEquippedItems);
            for (std::uint32_t i = 0; i < scanCount; ++i) {
                const auto& equipped = items[i];
                auto* form = equipped.item.object;
                if (!form || !form->IsWeapon()) {
                    continue;
                }

                auto* weapon = form->As<RE::TESObjectWEAP>();
                if (!weapon) {
                    continue;
                }

                EquippedWeaponSnapshot snapshot{};
                snapshot.weaponForm = form;
                snapshot.weapon = weapon;
                snapshot.instanceData = equipped.item.instanceData.get();
                snapshot.equipIndex = equipped.equipIndex;
                snapshot.weaponFormID = weapon->GetFormID();
                snapshot.weaponFormAddress = reinterpret_cast<std::uintptr_t>(form);
                snapshot.instanceDataAddress = reinterpret_cast<std::uintptr_t>(equipped.item.instanceData.get());
                snapshot.source = "middleHighEquippedItems";
                snapshots.push_back(snapshot);
            }
        }

        void appendEquipDataSnapshots(
            std::vector<EquippedWeaponSnapshot>& snapshots,
            const F4SEVR::ActorEquipData* equipData,
            const char* source)
        {
            if (!equipData) {
                return;
            }

            for (std::uint32_t slotIndex = 0; slotIndex < F4SEVR::ActorEquipData::kMaxSlots; ++slotIndex) {
                const auto& slot = equipData->slots[slotIndex];
                auto* form = slot.item;
                if (!form || !form->IsWeapon()) {
                    continue;
                }

                auto* weapon = form->As<RE::TESObjectWEAP>();
                if (!weapon) {
                    continue;
                }

                if (snapshotAlreadyCaptured(snapshots, form, slot.instanceData, slot.extraData)) {
                    continue;
                }

                EquippedWeaponSnapshot snapshot{};
                snapshot.weaponForm = form;
                snapshot.weapon = weapon;
                snapshot.instanceData = slot.instanceData;
                snapshot.objectInstanceExtra = slot.extraData;
                snapshot.equipIndex = findCompatibleEquipIndex(snapshots, form);
                snapshot.weaponFormID = weapon->GetFormID();
                snapshot.weaponFormAddress = reinterpret_cast<std::uintptr_t>(form);
                snapshot.instanceDataAddress = reinterpret_cast<std::uintptr_t>(slot.instanceData);
                snapshot.objectInstanceExtraAddress = reinterpret_cast<std::uintptr_t>(slot.extraData);
                snapshot.sourceSlotIndex = slotIndex;
                snapshot.source = source;
                snapshots.push_back(snapshot);
            }
        }

        std::vector<EquippedWeaponSnapshot> collectCurrentEquippedWeapons(
            RE::Actor& actor,
            const F4SEVR::PlayerCharacter* player)
        {
            std::vector<EquippedWeaponSnapshot> snapshots{};

            appendMiddleHighEquippedSnapshots(snapshots, actor);

            for (auto& snapshot : snapshots) {
                snapshot.objectInstanceExtraAddress = findObjectInstanceExtraAddress(
                    player,
                    snapshot.weaponForm,
                    snapshot.instanceData);
                snapshot.objectInstanceExtra = reinterpret_cast<const RE::BGSObjectInstanceExtra*>(snapshot.objectInstanceExtraAddress);
            }

            /*
             * The pending equipped witness comes from the player equip data path,
             * while middleHigh->equippedItems can be one manual-mod transaction
             * behind. These fallback snapshots do not broaden the native call:
             * they are still matched against the exact pending form, instance
             * data, and object-instance-extra witness before QueueAttachWeapon is
             * allowed to run.
             */
            appendEquipDataSnapshots(snapshots, player ? player->playerEquipData : nullptr, "firstPersonEquipData");
            appendEquipDataSnapshots(snapshots, player ? player->equipData : nullptr, "actorEquipData");

            return snapshots;
        }

        std::optional<EquippedWeaponSnapshot> findMatchingCurrentEquippedWeapon(
            RE::Actor& actor,
            const F4SEVR::PlayerCharacter* player,
            const RequestInput& input,
            const char*& mismatchReason,
            EquippedWeaponSnapshot& firstObserved)
        {
            const auto snapshots = collectCurrentEquippedWeapons(actor, player);
            const bool hasCurrentSnapshots = !snapshots.empty();
            if (hasCurrentSnapshots) {
                firstObserved = snapshots.front();
            }
            mismatchReason = "targetMissing";
            const weapon_native_visual_remap_policy::NativeVisualRemapTargetWitness expected{
                .formID = input.expectedWeaponFormID,
                .formAddress = input.expectedWeaponFormAddress,
                .instanceDataAddress = input.expectedInstanceDataAddress,
                .objectInstanceExtraAddress = input.expectedObjectInstanceExtraAddress,
            };

            bool mismatchReasonCaptured = false;
            for (const auto& snapshot : snapshots) {
                const weapon_native_visual_remap_policy::NativeVisualRemapTargetWitness observed{
                    .formID = snapshot.weaponFormID,
                    .formAddress = snapshot.weaponFormAddress,
                    .instanceDataAddress = snapshot.instanceDataAddress,
                    .objectInstanceExtraAddress = snapshot.objectInstanceExtraAddress,
                };
                const auto decision = weapon_native_visual_remap_policy::evaluateNativeVisualRemapTargetMatch(
                    expected,
                    observed);
                if (decision.matches) {
                    mismatchReason = decision.reason;
                    return snapshot;
                }

                if (!mismatchReasonCaptured) {
                    mismatchReason = decision.reason;
                    mismatchReasonCaptured = true;
                }
            }

            weapon_instance_witness_runtime::AuthoritativeEquippedWeaponWitness authoritativeWitness{};
            const bool authoritativeFallbackRequiresLiveEquipSnapshot =
                actor.currentProcess &&
                actor.currentProcess->middleHigh &&
                hasCurrentSnapshots;
            if (authoritativeFallbackRequiresLiveEquipSnapshot &&
                weapon_instance_witness_runtime::tryGetAuthoritativeEquippedWeaponWitness(
                    input.pendingInstanceSignature,
                    authoritativeWitness)) {
                auto* authoritativeForm = reinterpret_cast<RE::TESForm*>(authoritativeWitness.weaponFormAddress);
                auto* authoritativeWeapon = authoritativeForm ? authoritativeForm->As<RE::TESObjectWEAP>() : nullptr;
                const auto authoritativeEquipIndex = findLiveCompatibleEquipIndex(snapshots, authoritativeForm);
                if (!authoritativeEquipIndex) {
                    if (!mismatchReasonCaptured) {
                        mismatchReason = "authoritativeWitnessNoLiveEquipSlot";
                        mismatchReasonCaptured = true;
                    }
                    if (!hasCurrentSnapshots) {
                        mismatchReason = "currentEquippedWeaponMissing";
                    }
                    return std::nullopt;
                }

                EquippedWeaponSnapshot authoritativeSnapshot{};
                authoritativeSnapshot.weaponForm = authoritativeForm;
                authoritativeSnapshot.weapon = authoritativeWeapon;
                authoritativeSnapshot.instanceData = reinterpret_cast<RE::TBO_InstanceData*>(authoritativeWitness.instanceDataAddress);
                authoritativeSnapshot.objectInstanceExtra = reinterpret_cast<const RE::BGSObjectInstanceExtra*>(authoritativeWitness.objectInstanceExtraAddress);
                authoritativeSnapshot.equipIndex = *authoritativeEquipIndex;
                authoritativeSnapshot.weaponFormID = authoritativeWitness.weaponFormID;
                authoritativeSnapshot.weaponFormAddress = authoritativeWitness.weaponFormAddress;
                authoritativeSnapshot.instanceDataAddress = authoritativeWitness.instanceDataAddress;
                authoritativeSnapshot.objectInstanceExtraAddress = authoritativeWitness.objectInstanceExtraAddress;
                authoritativeSnapshot.sourceSlotIndex = authoritativeWitness.stackIndex;
                authoritativeSnapshot.source = weapon_instance_witness_runtime::sourceName(authoritativeWitness.source);

                const weapon_native_visual_remap_policy::NativeVisualRemapTargetWitness observed{
                    .formID = authoritativeSnapshot.weaponFormID,
                    .formAddress = authoritativeSnapshot.weaponFormAddress,
                    .instanceDataAddress = authoritativeSnapshot.instanceDataAddress,
                    .objectInstanceExtraAddress = authoritativeSnapshot.objectInstanceExtraAddress,
                };
                const auto decision = weapon_native_visual_remap_policy::evaluateNativeVisualRemapTargetMatch(
                    expected,
                    observed);
                if (decision.matches) {
                    mismatchReason = decision.reason;
                    return authoritativeSnapshot;
                }

                if (!mismatchReasonCaptured) {
                    firstObserved = authoritativeSnapshot;
                    mismatchReason = decision.reason;
                    mismatchReasonCaptured = true;
                }
            }

            if (!hasCurrentSnapshots) {
                mismatchReason = "currentEquippedWeaponMissing";
            }
            return std::nullopt;
        }

        RequestResult missingProcessResult(RE::Actor& actor)
        {
            if (!actor.currentProcess) {
                return RequestResult::MissingCurrentProcess;
            }
            if (!actor.currentProcess->middleHigh) {
                return RequestResult::MissingMiddleHighProcess;
            }
            return RequestResult::MissingEquippedWeapon;
        }

        void queueAttachWeapon(
            RE::TaskQueueInterface& taskQueue,
            RE::Actor& actor,
            RE::BGSObjectInstanceT<RE::TESObjectWEAP>& weapon,
            RE::BGSEquipIndex equipIndex)
        {
            // TaskQueueInterface::QueueAttachWeapon, verified in FO4VR at REL::ID(916430).
            using func_t = void (RE::TaskQueueInterface::*)(
                RE::Actor*,
                RE::BGSObjectInstanceT<RE::TESObjectWEAP>&,
                RE::BGSEquipIndex);
            static REL::Relocation<func_t> func{ REL::ID(916430) };
            return func(&taskQueue, &actor, weapon, equipIndex);
        }
    }

    const char* resultName(RequestResult result) noexcept
    {
        switch (result) {
        case RequestResult::Queued:
            return "queued";
        case RequestResult::Disabled:
            return "disabled";
        case RequestResult::MissingPendingInstance:
            return "missingPendingInstance";
        case RequestResult::MissingPlayer:
            return "missingPlayer";
        case RequestResult::MissingCurrentProcess:
            return "missingCurrentProcess";
        case RequestResult::MissingMiddleHighProcess:
            return "missingMiddleHighProcess";
        case RequestResult::MissingEquippedWeapon:
            return "missingEquippedWeapon";
        case RequestResult::MissingTaskQueue:
            return "missingTaskQueue";
        case RequestResult::InvalidEquippedWeapon:
            return "invalidEquippedWeapon";
        case RequestResult::MismatchedEquippedWeapon:
            return "mismatchedEquippedWeapon";
        default:
            return "unknown";
        }
    }

    RequestOutcome requestCurrentFirstPersonWeaponVisualRemap(const RequestInput& input)
    {
        if (!input.enabled) {
            return makeOutcome(RequestResult::Disabled, input.pendingInstanceSignature, "configDisabled");
        }

        if (input.pendingInstanceSignature == 0) {
            return makeOutcome(RequestResult::MissingPendingInstance, input.pendingInstanceSignature, "missingPendingWitness");
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            return makeOutcome(RequestResult::MissingPlayer, input.pendingInstanceSignature, "playerSingletonMissing");
        }

        auto* vrPlayer = reinterpret_cast<const F4SEVR::PlayerCharacter*>(player);
        const char* mismatchReason = "";
        EquippedWeaponSnapshot firstObserved{};
        auto snapshot = findMatchingCurrentEquippedWeapon(*player, vrPlayer, input, mismatchReason, firstObserved);
        if (!snapshot || !snapshot->weapon) {
            if (firstObserved.weapon) {
                return makeOutcome(
                    RequestResult::MismatchedEquippedWeapon,
                    input.pendingInstanceSignature,
                    mismatchReason,
                    &firstObserved);
            }
            return makeOutcome(missingProcessResult(*player), input.pendingInstanceSignature, "currentEquippedWeaponMissing");
        }

        auto* form = snapshot->weaponForm;
        if (!form || !form->IsWeapon() || !snapshot->weapon) {
            return makeOutcome(RequestResult::InvalidEquippedWeapon, input.pendingInstanceSignature, "equippedObjectIsNotWeapon", &*snapshot);
        }

        auto* taskQueue = RE::TaskQueueInterface::GetSingleton();
        if (!taskQueue) {
            return makeOutcome(RequestResult::MissingTaskQueue, input.pendingInstanceSignature, "taskQueueMissing", &*snapshot);
        }

        BorrowedWeaponInstance borrowedWeaponInstance{ snapshot->weapon, snapshot->instanceData };
        auto& weaponInstance = reinterpret_cast<RE::BGSObjectInstanceT<RE::TESObjectWEAP>&>(borrowedWeaponInstance);
        queueAttachWeapon(*taskQueue, *player, weaponInstance, snapshot->equipIndex);
        return makeOutcome(RequestResult::Queued, input.pendingInstanceSignature, input.reason, &*snapshot);
    }
}
