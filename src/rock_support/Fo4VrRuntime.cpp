#include "rock_support/Fo4VrRuntime.h"

#include "rock_support/Fo4VrActorStatePolicy.h"
#include "rock_support/Logger.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/weapon/CarriedWeaponRuntime.h"

#include <RE/Bethesda/SendPapyrusEvent.h>

#include <algorithm>
#include <cstring>
#include <ranges>

namespace rock::fo4vr
{
    namespace
    {
        [[nodiscard]] bool hasKeyword(const RE::BGSKeywordForm* keywordForm, const std::uint32_t keywordFormId) noexcept
        {
            if (!keywordForm || !keywordForm->keywords) {
                return false;
            }
            for (std::uint32_t index = 0; index < keywordForm->numKeywords; ++index) {
                const auto* keyword = keywordForm->keywords[index];
                if (keyword && keyword->formID == keywordFormId) {
                    return true;
                }
            }
            return false;
        }

    }

    RE::PlayerCharacter* getPlayer() noexcept
    {
        return RE::PlayerCharacter::GetSingleton();
    }

    PlayerNodes* getPlayerNodes() noexcept
    {
        auto* player = getPlayer();
        return player ? reinterpret_cast<PlayerNodes*>(reinterpret_cast<std::uintptr_t>(player) + 0x6E0) : nullptr;
    }

    RE::NiNode* getWorldRootNode() noexcept
    {
        auto* player = getPlayer();
        if (!player || !player->loadedData || !player->loadedData->data3D) {
            return nullptr;
        }
        return player->loadedData->data3D->IsNode();
    }

    RE::NiNode* getRootNode() noexcept
    {
        auto* worldRoot = getWorldRootNode();
        if (!worldRoot || worldRoot->children.empty() || !worldRoot->children[0]) {
            return nullptr;
        }
        return worldRoot->children[0]->IsNode();
    }

    BSFlattenedBoneTree* getFlattenedBoneTree() noexcept
    {
        return reinterpret_cast<BSFlattenedBoneTree*>(getRootNode());
    }

    RE::NiNode* getFirstPersonSkeleton() noexcept
    {
        auto* player = getPlayer();
        return player ? player->firstPerson3D.get() : nullptr;
    }

    BSFlattenedBoneTree* getFirstPersonBoneTree() noexcept
    {
        auto* skeleton = getFirstPersonSkeleton();
        if (!skeleton || skeleton->children.empty() || !skeleton->children[0]) {
            return nullptr;
        }
        return reinterpret_cast<BSFlattenedBoneTree*>(skeleton->children[0]->IsNode());
    }

    RE::EquippedItem* getEquippedWeaponItem() noexcept
    {
        auto* player = getPlayer();
        auto* middleHigh = player && player->currentProcess ? player->currentProcess->middleHigh : nullptr;
        if (!middleHigh || middleHigh->equippedItems.empty()) {
            return nullptr;
        }

        // Preserve the legacy native-equipment view. Only ROCK's explicitly
        // owned carried context is excluded; other native indices retain
        // their previous behavior for existing consumers.
        RE::EquippedItem* equippedItem = nullptr;
        for (auto& item : middleHigh->equippedItems) {
            if (!rock::CarriedWeaponRuntime::ownsNativeContext(item)) { equippedItem = &item; break; }
        }
        if (!equippedItem) return nullptr;
        auto* object = equippedItem->item.object;
        if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP) {
            return nullptr;
        }

        const auto* weapon = static_cast<const RE::TESObjectWEAP*>(object);
        const auto weaponType = weapon->weaponData.type.get();
        if (weaponType == RE::WEAPON_TYPE::kGrenade ||
            weaponType == RE::WEAPON_TYPE::kMine) {
            // FO4VR keeps the selected throwable in equippedItems[0] when no
            // hand weapon is equipped. It is inventory selection state, not a
            // weapon presentation/occupancy identity for ROCK's hand systems.
            return nullptr;
        }

        return equippedItem;
    }

    RE::EquippedWeaponData* getEquippedWeaponData() noexcept
    {
        auto* equippedItem = getEquippedWeaponItem();
        return equippedItem && equippedItem->data ?
            static_cast<RE::EquippedWeaponData*>(equippedItem->data.get()) : nullptr;
    }

    RE::NiNode* getWeaponNode() noexcept
    {
        return findNode(getFirstPersonSkeleton(), "Weapon");
    }

    RE::PlayerCamera* getPlayerCamera() noexcept
    {
        return RE::PlayerCamera::GetSingleton();
    }

    RE::NiPoint3 getCameraPosition() noexcept
    {
        auto* camera = getPlayerCamera();
        return camera && camera->cameraRoot ? camera->cameraRoot->world.translate : RE::NiPoint3{};
    }

    RE::NiNode* getLeftHandNode() noexcept
    {
        auto* nodes = getPlayerNodes();
        if (!nodes) {
            return nullptr;
        }
        return nodes->SecondaryWandNode;
    }

    RE::NiNode* getRightHandNode() noexcept
    {
        auto* nodes = getPlayerNodes();
        if (!nodes) {
            return nullptr;
        }
        return nodes->primaryWandNode;
    }

    std::uint32_t getNativeWeaponState(const RE::Actor* actor) noexcept
    {
        if (!actor) {
            return fo4vr_actor_state_policy::kInvalidWeaponState;
        }

        const auto* actorState = static_cast<const RE::ActorState*>(actor);
        std::uint32_t actorStateStorage = 0;
        std::memcpy(
            &actorStateStorage,
            reinterpret_cast<const std::byte*>(actorState) +
                fo4vr_actor_state_policy::kWeaponStateStorageOffset,
            sizeof(actorStateStorage));
        return fo4vr_actor_state_policy::decodeWeaponState(actorStateStorage);
    }

    std::uint32_t getNativeGunState(const RE::Actor* actor) noexcept
    {
        if (!actor) {
            return fo4vr_actor_state_policy::kInvalidGunState;
        }

        const auto* actorState = static_cast<const RE::ActorState*>(actor);
        std::uint32_t actorStateStorage = 0;
        std::memcpy(
            &actorStateStorage,
            reinterpret_cast<const std::byte*>(actorState) +
                fo4vr_actor_state_policy::kWeaponStateStorageOffset,
            sizeof(actorStateStorage));
        return fo4vr_actor_state_policy::decodeGunState(actorStateStorage);
    }

    bool IsWeaponDrawn() noexcept
    {
        return fo4vr_actor_state_policy::isWeaponMagicDrawn(
            getNativeWeaponState(getPlayer()));
    }

    bool isMeleeWeaponEquipped() noexcept
    {
        auto* player = getPlayer();
        if (!player || !CombatUtilities_IsActorUsingMelee(player) || !player->inventoryList) {
            return false;
        }

        return std::ranges::any_of(player->inventoryList->data, [](const RE::BGSInventoryItem& item) {
            const auto* weapon = item.object ? item.object->As<RE::TESObjectWEAP>() : nullptr;
            return weapon && weapon_type_policy::isEquippedMelee(weapon->weaponData.type.get()) &&
                item.stackData &&
                item.stackData->IsEquipped();
        });
    }

    namespace
    {
        PowerArmorStateProvider s_powerArmorStateProvider = nullptr;
    }

    void setPowerArmorStateProvider(const PowerArmorStateProvider provider) noexcept
    {
        s_powerArmorStateProvider = provider;
    }

    bool isInPowerArmor() noexcept
    {
        return s_powerArmorStateProvider ? s_powerArmorStateProvider() : isInPowerArmorFromBiped();
    }

    bool isInPowerArmorFromBiped() noexcept
    {
        auto* player = getPlayer();
        const auto biped = player ? player->biped.get() : nullptr;
        if (!biped) {
            return false;
        }

        const auto* equippedForm = biped->object[3].parent.object;
        if (!equippedForm || equippedForm->formType != RE::ENUM_FORM_ID::kARMO) {
            return false;
        }
        const auto* armor = static_cast<const RE::TESObjectARMO*>(equippedForm);
        return hasKeyword(armor, kPowerArmorKeywordFormId) || hasKeyword(armor, kPowerArmorFrameKeywordFormId);
    }

    bool isExplodableCar(const RE::TESBoundObject* baseForm) noexcept
    {
        if (!baseForm || !baseForm->Is(RE::ENUM_FORM_ID::kMSTT)) {
            return false;
        }

        const auto* movableStatic = static_cast<const RE::BGSMovableStatic*>(baseForm);
        return hasKeyword(movableStatic, car_interaction_policy::kExplodableCarKeywordFormId);
    }

    RE::Setting* getIniSetting(const char* name) noexcept
    {
        if (!name || !*name) {
            return nullptr;
        }
        if (auto* preferences = RE::INIPrefSettingCollection::GetSingleton()) {
            if (auto* setting = preferences->GetSetting(name)) {
                return setting;
            }
        }
        if (auto* settings = RE::INISettingCollection::GetSingleton()) {
            if (auto* setting = settings->GetSetting(name)) {
                return setting;
            }
        }
        logger::warn("Setting '{}' not found in Fallout 4 VR INI collections", name);
        return nullptr;
    }

    void showNotification(const std::string& text)
    {
        logger::info("Show notification: '{}'", text);
        auto* vm = RE::BSScript::Internal::VirtualMachine::GetSingleton();
        if (!vm) {
            logger::warn("Cannot show notification because the Papyrus VM is unavailable");
            return;
        }

        const RE::BSFixedString scriptName{ "Debug" };
        const RE::BSFixedString functionName{ "Notification" };
        const RE::BSFixedString message{ text };
        const RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> callback{};
        if (!Papyrus::detail::DispatchStaticCall(vm, scriptName, functionName, callback, message)) {
            logger::warn("Papyrus rejected Debug.Notification dispatch");
        }
    }

    RE::NiAVObject* getFirstChild(RE::NiAVObject* object) noexcept
    {
        auto* node = object ? object->IsNode() : nullptr;
        return node && !node->children.empty() ? node->children[0].get() : nullptr;
    }

    RE::NiNode* findNode(RE::NiAVObject* root, const char* name, const int maxDepth) noexcept
    {
        if (!root || !name || maxDepth < 0) {
            return nullptr;
        }
        if (_stricmp(name, root->name.c_str()) == 0) {
            return root->IsNode();
        }
        if (maxDepth == 0) {
            return nullptr;
        }

        auto* node = root->IsNode();
        if (!node) {
            return nullptr;
        }
        for (const auto& child : node->children) {
            if (child) {
                if (auto* result = findNode(child.get(), name, maxDepth - 1)) {
                    return result;
                }
            }
        }
        return nullptr;
    }

    RE::NiNode* find1StChildNode(RE::NiAVObject* root, const char* name) noexcept
    {
        return findNode(root, name, 1);
    }

    bool isNodeVisible(const RE::NiAVObject* node) noexcept
    {
        return node && (node->flags.flags & 0x1) == 0;
    }

    void updateDown(RE::NiAVObject* node, const bool updateSelf, const char* ignoredNode) noexcept
    {
        if (!node) {
            return;
        }
        RE::NiUpdateData* updateData = nullptr;
        if (updateSelf) {
            node->UpdateWorldData(updateData);
        }

        auto* parent = node->IsNode();
        if (!parent) {
            return;
        }
        for (const auto& child : parent->children) {
            if (!child || (ignoredNode && _stricmp(child->name.c_str(), ignoredNode) == 0)) {
                continue;
            }
            if (auto* childNode = child->IsNode()) {
                updateDown(childNode, true);
            } else if (auto* geometry = child->IsGeometry()) {
                geometry->UpdateWorldData(updateData);
            }
        }
    }

    void updateTransforms(RE::NiAVObject* node) noexcept
    {
        if (!node || !node->parent) {
            return;
        }
        const auto& parentTransform = node->parent->world;
        const auto& localTransform = node->local;
        const RE::NiPoint3 translated = parentTransform.rotate.Transpose() * (localTransform.translate * parentTransform.scale);
        node->world.translate = parentTransform.translate + translated;
        node->world.rotate = localTransform.rotate * parentTransform.rotate;
        node->world.scale = parentTransform.scale * localTransform.scale;
    }

    void updateTransformsDown(RE::NiAVObject* node, const bool updateSelf, const char* ignoredNode) noexcept
    {
        if (!node) {
            return;
        }
        if (updateSelf) {
            updateTransforms(node);
        }

        auto* parent = node->IsNode();
        if (!parent) {
            return;
        }
        for (const auto& child : parent->children) {
            if (!child || (ignoredNode && _stricmp(child->name.c_str(), ignoredNode) == 0)) {
                continue;
            }
            if (auto* childNode = child->IsNode()) {
                updateTransformsDown(childNode, true);
            } else if (auto* triShape = child->IsTriShape()) {
                updateTransforms(triShape);
            }
        }
    }
}
