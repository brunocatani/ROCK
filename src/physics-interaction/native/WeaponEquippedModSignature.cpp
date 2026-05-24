#include "physics-interaction/native/WeaponEquippedModSignature.h"

#include "physics-interaction/weapon/WeaponAuthority.h"

#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/TESForms.h"

#include "f4vr/PlayerNodes.h"
#include "f4sevr/Forms.h"

#include <algorithm>
#include <limits>

namespace rock
{
    namespace
    {
        struct ObjectInstanceExtraWitness
        {
            std::uint64_t signature{ 0 };
            std::uint32_t count{ 0 };
            std::uint32_t activeCount{ 0 };
            std::uint32_t disabledCount{ 0 };
        };

        ObjectInstanceExtraWitness makeObjectInstanceExtraWitness(const RE::BGSObjectInstanceExtra* extra)
        {
            ObjectInstanceExtraWitness witness{};
            if (!extra || !extra->values) {
                return witness;
            }

            const auto indexData = extra->GetIndexData();
            std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
            weapon_visual_composition_policy::mixString(key, "ROCKObjectInstanceExtraIndexWitnessV1");
            weapon_visual_composition_policy::mixValue(key, indexData.size());
            witness.count = static_cast<std::uint32_t>(
                (std::min)(indexData.size(), static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
            for (const auto& modIndex : indexData) {
                weapon_visual_composition_policy::mixValue(key, modIndex.objectID);
                weapon_visual_composition_policy::mixValue(key, modIndex.index);
                weapon_visual_composition_policy::mixValue(key, modIndex.rank);
                weapon_visual_composition_policy::mixValue(key, modIndex.disabled);
                if (modIndex.disabled) {
                    ++witness.disabledCount;
                } else {
                    ++witness.activeCount;
                }
            }
            witness.signature = key;
            return witness;
        }

        const RE::BGSObjectInstanceExtra* findEquippedWeaponObjectInstanceExtra(
            const F4SEVR::PlayerCharacter* player,
            const F4SEVR::TESForm* weaponForm,
            const RE::TBO_InstanceData* instanceData)
        {
            if (!player || !weaponForm) {
                return nullptr;
            }

            const auto* reWeaponForm = reinterpret_cast<const RE::TESForm*>(weaponForm);
            auto scanEquipData = [&](const F4SEVR::ActorEquipData* equipData) -> const RE::BGSObjectInstanceExtra* {
                if (!equipData) {
                    return nullptr;
                }
                for (std::uint32_t slotIndex = 0; slotIndex < F4SEVR::ActorEquipData::kMaxSlots; ++slotIndex) {
                    const auto& slot = equipData->slots[slotIndex];
                    if (slot.item != reWeaponForm) {
                        continue;
                    }
                    if (instanceData && slot.instanceData && slot.instanceData != instanceData) {
                        continue;
                    }
                    if (slot.extraData) {
                        return slot.extraData;
                    }
                }
                return nullptr;
            };

            if (const auto* firstPersonExtra = scanEquipData(player->playerEquipData)) {
                return firstPersonExtra;
            }
            return scanEquipData(player->equipData);
        }
    }

    EquippedWeaponModSignature readEquippedWeaponModSignature()
    {
        EquippedWeaponModSignature signature{};

        auto* player = f4vr::getPlayer();
        auto* processData = player && player->middleProcess ? player->middleProcess->unk08 : nullptr;
        auto* equipData = processData ? processData->equipData : nullptr;
        auto* weaponForm = equipData ? equipData->item : nullptr;
        if (!weaponForm || weaponForm->formType != static_cast<std::uint8_t>(RE::ENUM_FORM_ID::kWEAP)) {
            return signature;
        }

        const auto* objectInstanceExtra = findEquippedWeaponObjectInstanceExtra(player, weaponForm, equipData->instanceData);
        const auto objectInstanceWitness = makeObjectInstanceExtraWitness(objectInstanceExtra);
        signature.hasEquippedWeapon = true;
        signature.formID = weaponForm->formID;
        signature.instanceDataAddress = reinterpret_cast<std::uintptr_t>(equipData->instanceData);
        signature.objectInstanceExtraAddress = reinterpret_cast<std::uintptr_t>(objectInstanceExtra);
        signature.objectIndexDataSignature = objectInstanceWitness.signature;
        signature.objectIndexDataCount = objectInstanceWitness.count;
        signature.activeModCount = objectInstanceWitness.activeCount;
        signature.disabledModCount = objectInstanceWitness.disabledCount;
        signature.equippedDataAddress = reinterpret_cast<std::uintptr_t>(equipData->equippedData);
        signature.equippedObjectAddress = reinterpret_cast<std::uintptr_t>(equipData->equippedData ? equipData->equippedData->object : nullptr);

        std::uint64_t key = weapon_visual_composition_policy::kWeaponVisualCompositionOffset;
        weapon_visual_composition_policy::mixString(key, "ROCKEquippedWeaponModGraphRefreshSignatureV2");
        weapon_visual_composition_policy::mixValue(key, signature.formID);
        weapon_visual_composition_policy::mixValue(key, signature.instanceDataAddress);
        weapon_visual_composition_policy::mixValue(key, signature.objectInstanceExtraAddress);
        weapon_visual_composition_policy::mixValue(key, signature.objectIndexDataSignature);
        weapon_visual_composition_policy::mixValue(key, signature.objectIndexDataCount);
        weapon_visual_composition_policy::mixValue(key, signature.activeModCount);
        weapon_visual_composition_policy::mixValue(key, signature.disabledModCount);
        weapon_visual_composition_policy::mixValue(key, signature.equippedDataAddress);
        weapon_visual_composition_policy::mixValue(key, signature.equippedObjectAddress);
        signature.key = key;
        return signature;
    }
}
