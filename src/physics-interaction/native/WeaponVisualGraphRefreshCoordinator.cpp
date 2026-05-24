#include "physics-interaction/native/WeaponVisualGraphRefreshCoordinator.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/WeaponAuthority.h"

#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSFixedString.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/UI.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

#include "f4vr/PlayerNodes.h"
#include "f4sevr/Forms.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kWeaponReferenceRefreshFlags =
            static_cast<std::uint32_t>(RE::RESET_3D_FLAGS::kModel) |
            static_cast<std::uint32_t>(RE::RESET_3D_FLAGS::kSkin) |
            static_cast<std::uint32_t>(RE::RESET_3D_FLAGS::kHead) |
            static_cast<std::uint32_t>(RE::RESET_3D_FLAGS::kScale) |
            static_cast<std::uint32_t>(RE::RESET_3D_FLAGS::kSkeleton);

        static_assert(kWeaponReferenceRefreshFlags == 0x37);

        struct ObjectInstanceExtraWitness
        {
            std::uint64_t signature{ 0 };
            std::uint32_t count{ 0 };
            std::uint32_t activeCount{ 0 };
            std::uint32_t disabledCount{ 0 };
        };

        struct WeaponNodeStats
        {
            std::uint32_t childCount{ 0 };
            std::uint32_t nodeCount{ 0 };
            std::uint32_t triShapeCount{ 0 };
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

        WeaponVisualGraphRefreshCoordinator::EquippedWeaponModSignature readEquippedWeaponModSignature()
        {
            WeaponVisualGraphRefreshCoordinator::EquippedWeaponModSignature signature{};

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
            weapon_visual_composition_policy::mixString(key, "ROCKEquippedWeaponModGraphRefreshSignatureV1");
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

        void accumulateWeaponNodeStats(RE::NiAVObject* node, WeaponNodeStats& stats, std::uint32_t depth = 0)
        {
            if (!node || depth > 32 || stats.nodeCount >= 4096) {
                return;
            }

            ++stats.nodeCount;
            if (node->IsTriShape()) {
                ++stats.triShapeCount;
                return;
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            const auto& children = niNode->children;
            if (depth == 0) {
                const auto childCount = static_cast<std::size_t>(children.size());
                stats.childCount = static_cast<std::uint32_t>(
                    (std::min)(childCount, static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
            }
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* child = children[i].get()) {
                    accumulateWeaponNodeStats(child, stats, depth + 1);
                }
            }
        }

        WeaponNodeStats summarizeWeaponNode(RE::NiAVObject* node)
        {
            WeaponNodeStats stats{};
            accumulateWeaponNodeStats(node, stats);
            return stats;
        }

        const char* safeNodeName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "";
            }
            const char* name = node->name.c_str();
            return name ? name : "";
        }

        bool weaponGraphRefreshMenuOpen()
        {
            auto* ui = RE::UI::GetSingleton();
            if (!ui) {
                return false;
            }

            static const RE::BSFixedString craftingMenu{ "Crafting Menu" };
            static const RE::BSFixedString examineMenu{ "ExamineMenu" };
            static const RE::BSFixedString examineConfirmMenu{ "ExamineConfirmMenu" };
            return ui->GetMenuOpen(craftingMenu) || ui->GetMenuOpen(examineMenu) || ui->GetMenuOpen(examineConfirmMenu);
        }
    }

    void WeaponVisualGraphRefreshCoordinator::reset()
    {
        _lastSignature = {};
        _pendingSignature = {};
        _refreshWindowStartSignature = {};
        _refreshWindowSignature = {};
        _hasLastSignature = false;
        _hasRefreshWindowStartSignature = false;
        _hasRefreshWindowSignature = false;
        _refreshWindowOpenLastFrame = false;
        _refreshWindowSignatureChanged = false;
        _pendingRefresh = false;
        _pendingReason = "unknown";
        _pendingApplyFrames = 0;
        _postRefreshCollisionHoldFrames = 0;
    }

    void WeaponVisualGraphRefreshCoordinator::clearRefreshWindowState()
    {
        _refreshWindowStartSignature = {};
        _refreshWindowSignature = {};
        _hasRefreshWindowStartSignature = false;
        _hasRefreshWindowSignature = false;
        _refreshWindowOpenLastFrame = false;
        _refreshWindowSignatureChanged = false;
    }

    void WeaponVisualGraphRefreshCoordinator::scheduleRefresh(const EquippedWeaponModSignature& signature, const char* reason)
    {
        _pendingSignature = signature;
        _pendingRefresh = true;
        _pendingReason = reason ? reason : "unknown";
        _pendingApplyFrames = 0;

        ROCK_LOG_INFO(Weapon,
            "Equipped weapon graph refresh scheduled reason={} formID={:08X} signature={:016X} instance=0x{:016X} extra=0x{:016X} activeMods={} disabledMods={}",
            _pendingReason,
            signature.formID,
            signature.key,
            signature.instanceDataAddress,
            signature.objectInstanceExtraAddress,
            signature.activeModCount,
            signature.disabledModCount);
    }

    bool WeaponVisualGraphRefreshCoordinator::tryApplyRefresh(const UpdateInput& input)
    {
        if (!_pendingRefresh) {
            return false;
        }

        ++_pendingApplyFrames;
        if (input.menuBlocking || !input.weaponDrawn || !input.weaponNode) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                1000,
                "Equipped weapon graph refresh pending reason={} formID={:08X} signature={:016X} menuBlocking={} weaponDrawn={} weaponNode={} pendingFrames={}",
                _pendingReason,
                _pendingSignature.formID,
                _pendingSignature.key,
                input.menuBlocking,
                input.weaponDrawn,
                input.weaponNode != nullptr,
                _pendingApplyFrames);
            return false;
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            ROCK_LOG_SAMPLE_WARN(Weapon,
                1000,
                "Equipped weapon graph refresh could not apply: PlayerCharacter unavailable reason={} formID={:08X} signature={:016X}",
                _pendingReason,
                _pendingSignature.formID,
                _pendingSignature.key);
            return false;
        }

        const auto beforeStats = summarizeWeaponNode(input.weaponNode);
        player->Set3DUpdateFlag(static_cast<RE::RESET_3D_FLAGS>(kWeaponReferenceRefreshFlags));
        _pendingRefresh = false;
        _postRefreshCollisionHoldFrames = 1;

        ROCK_LOG_INFO(Weapon,
            "Equipped weapon graph refresh applied reason={} flags=0x{:X} formID={:08X} signature={:016X} node='{}' nodePtr=0x{:016X} children={} nodes={} triShapes={} pendingFrames={}",
            _pendingReason,
            kWeaponReferenceRefreshFlags,
            _pendingSignature.formID,
            _pendingSignature.key,
            safeNodeName(input.weaponNode),
            reinterpret_cast<std::uintptr_t>(input.weaponNode),
            beforeStats.childCount,
            beforeStats.nodeCount,
            beforeStats.triShapeCount,
            _pendingApplyFrames);
        return true;
    }

    WeaponVisualGraphRefreshCoordinator::UpdateResult WeaponVisualGraphRefreshCoordinator::update(const UpdateInput& input)
    {
        UpdateResult result{};

        if (!input.enabled || !input.visualAuthorityAvailable || !input.skeletonReady) {
            reset();
            return result;
        }

        if (_postRefreshCollisionHoldFrames > 0) {
            result.deferWeaponCollision = true;
            --_postRefreshCollisionHoldFrames;
        }

        const auto signature = readEquippedWeaponModSignature();
        result.signatureKey = signature.key;
        const bool refreshMenuOpen = weaponGraphRefreshMenuOpen();

        if (!signature.hasEquippedWeapon) {
            if (_hasLastSignature || _pendingRefresh || _refreshWindowOpenLastFrame) {
                ROCK_LOG_INFO(Weapon, "Equipped weapon graph refresh state cleared: no equipped weapon");
            }
            reset();
            return result;
        }

        if (!_hasLastSignature) {
            _lastSignature = signature;
            _hasLastSignature = true;
        } else if (signature.key != _lastSignature.key) {
            _lastSignature = signature;
            result.signatureChanged = true;
        }

        if (refreshMenuOpen) {
            if (!_refreshWindowOpenLastFrame) {
                _refreshWindowStartSignature = signature;
                _refreshWindowSignature = signature;
                _hasRefreshWindowStartSignature = true;
                _hasRefreshWindowSignature = true;
                _refreshWindowSignatureChanged = false;
                ROCK_LOG_DEBUG(Weapon,
                    "Equipped weapon graph refresh window opened formID={:08X} signature={:016X} activeMods={} disabledMods={}",
                    signature.formID,
                    signature.key,
                    signature.activeModCount,
                    signature.disabledModCount);
            } else if (!_hasRefreshWindowSignature) {
                _refreshWindowSignature = signature;
                _hasRefreshWindowSignature = true;
            } else if (signature.key != _refreshWindowSignature.key) {
                _refreshWindowSignature = signature;
                _refreshWindowSignatureChanged = !_hasRefreshWindowStartSignature || signature.key != _refreshWindowStartSignature.key;
                ROCK_LOG_INFO(Weapon,
                    "Equipped weapon mod signature changed inside workbench/menu window formID={:08X} signature={:016X} activeMods={} disabledMods={}",
                    signature.formID,
                    signature.key,
                    signature.activeModCount,
                    signature.disabledModCount);
            }
            _refreshWindowOpenLastFrame = true;
            return result;
        }

        if (_refreshWindowOpenLastFrame) {
            if (!_hasRefreshWindowSignature) {
                _refreshWindowStartSignature = signature;
                _refreshWindowSignature = signature;
                _hasRefreshWindowStartSignature = true;
                _hasRefreshWindowSignature = true;
            } else if (signature.key != _refreshWindowSignature.key) {
                _refreshWindowSignature = signature;
                _refreshWindowSignatureChanged = !_hasRefreshWindowStartSignature || signature.key != _refreshWindowStartSignature.key;
                result.signatureChanged = true;
            }

            if (_refreshWindowSignatureChanged && _hasRefreshWindowSignature) {
                scheduleRefresh(_refreshWindowSignature, "workbench-menu-closed-mod-signature-changed");
            } else {
                ROCK_LOG_DEBUG(Weapon,
                    "Equipped weapon graph refresh window closed without mod signature change formID={:08X} signature={:016X}",
                    signature.formID,
                    signature.key);
            }
            clearRefreshWindowState();
        }

        if (_pendingRefresh && signature.key != _pendingSignature.key) {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon graph refresh canceled: equipped signature diverged before apply pending={:016X} current={:016X} pendingFormID={:08X} currentFormID={:08X}",
                _pendingSignature.key,
                signature.key,
                _pendingSignature.formID,
                signature.formID);
            _pendingRefresh = false;
            _pendingSignature = {};
            _pendingReason = "unknown";
            _pendingApplyFrames = 0;
        }

        if (tryApplyRefresh(input)) {
            result.refreshApplied = true;
            result.deferWeaponCollision = true;
        }

        return result;
    }
}
