#include "physics-interaction/actor/ActorEquipmentGrab.h"

#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/PhysicsLog.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSGeometry.h"
#include "RE/Bethesda/BSSkin.h"
#include "RE/Bethesda/FormUtil.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiNode.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace rock::actor_equipment_grab
{
    namespace
    {
        constexpr int kMaxNodeSearchDepth = 48;
        constexpr std::uint32_t kMaxSkinBoneCount = 512;

        struct EquippedStackMatch
        {
            bool found = false;
            std::uint32_t stackIndex = kInvalidStackIndex;
            std::uint32_t count = 0;
            RE::TBO_InstanceData* instanceData = nullptr;
            RE::ExtraDataList* extraList = nullptr;
        };

        [[nodiscard]] std::uint32_t slotIndex(RE::BIPED_OBJECT slot) noexcept
        {
            if (slot == RE::BIPED_OBJECT::kNone) {
                return kInvalidStackIndex;
            }
            return static_cast<std::uint32_t>(std::to_underlying(slot));
        }

        [[nodiscard]] bool validBipedSlot(std::uint32_t index) noexcept
        {
            return index < static_cast<std::uint32_t>(std::to_underlying(RE::BIPED_OBJECT::kTotal));
        }

        [[nodiscard]] bool isActorRef(RE::TESObjectREFR* ref) noexcept
        {
            auto* base = ref ? ref->GetObjectReference() : nullptr;
            return base && base->Is(RE::ENUM_FORM_ID::kNPC_);
        }

        [[nodiscard]] RE::Actor* asDeadActor(RE::TESObjectREFR* ref) noexcept
        {
            if (!isActorRef(ref) || !ref->IsDead(false)) {
                return nullptr;
            }
            return static_cast<RE::Actor*>(ref);
        }

        [[nodiscard]] bool playableActorWearableForm(RE::TESBoundObject* item, RE::TBO_InstanceData* instanceData)
        {
            if (!item || !item->Is(RE::ENUM_FORM_ID::kARMO) || !item->GetPlayable(instanceData)) {
                return false;
            }

            const auto fullName = RE::TESFullName::GetFullName(*item, false);
            if (fullName.empty()) {
                return false;
            }

            return true;
        }

        [[nodiscard]] int bipedSlotPriority(std::uint32_t index) noexcept
        {
            /*
             * ROCK resolves overlapping skinned body slots with deterministic
             * biped priority. FO4VR uses a different biped enum past the 32
             * authored armor slots, so weapons and utility slots are treated
             * after armor while retaining deterministic ordering.
             */
            constexpr std::array<int, 32> armorPriorities{
                4, 3, 10, 6, 7, 2, 5, 8,
                9, 0, 13, 11, 1, 12, 84, 85,
                14, 86, 87, 88, 89, 90, 91, 92,
                93, 94, 95, 96, 97, 98, 99, 100,
            };

            if (index < armorPriorities.size()) {
                return armorPriorities[index];
            }
            return 200 + static_cast<int>(index);
        }

        [[nodiscard]] const char* nodeName(RE::NiAVObject* node) noexcept
        {
            return node && node->name.c_str() ? node->name.c_str() : "(unnamed)";
        }

        [[nodiscard]] bool sameOrPreferredInstance(RE::TBO_InstanceData* preferred, RE::TBO_InstanceData* actual) noexcept
        {
            return !preferred || preferred == actual;
        }

        [[nodiscard]] EquippedStackMatch findEquippedStack(RE::TESObjectREFR* actorRef, RE::TESBoundObject* item, RE::TBO_InstanceData* preferredInstanceData)
        {
            EquippedStackMatch fallback{};
            std::uint32_t equippedMatchCount = 0;
            if (!actorRef || !item || !actorRef->inventoryList) {
                return fallback;
            }

            for (auto& inventoryItem : actorRef->inventoryList->data) {
                if (inventoryItem.object != item) {
                    continue;
                }

                std::uint32_t stackIndex = 0;
                for (auto* stack = inventoryItem.stackData.get(); stack; stack = stack->nextStack.get(), ++stackIndex) {
                    if (!stack->IsEquipped()) {
                        continue;
                    }
                    ++equippedMatchCount;

                    auto* instanceData = inventoryItem.GetInstanceData(stackIndex);
                    EquippedStackMatch match{
                        .found = true,
                        .stackIndex = stackIndex,
                        .count = stack->GetCount(),
                        .instanceData = instanceData,
                        .extraList = stack->extra.get(),
                    };

                    if (sameOrPreferredInstance(preferredInstanceData, instanceData)) {
                        return match;
                    }
                    if (!fallback.found) {
                        fallback = match;
                    }
                }
            }

            if (preferredInstanceData && equippedMatchCount != 1) {
                return {};
            }
            return fallback;
        }

        void collectDownstreamNodesNoCollision(RE::NiAVObject* root, std::unordered_set<RE::NiAVObject*>& targets, int depth)
        {
            if (!root || depth < 0) {
                return;
            }

            targets.insert(root);
            auto* node = root->IsNode();
            if (!node) {
                return;
            }

            auto& children = node->GetRuntimeData().children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                auto* child = children[i].get();
                if (!child || child->collisionObject.get()) {
                    continue;
                }
                collectDownstreamNodesNoCollision(child, targets, depth - 1);
            }
        }

        [[nodiscard]] bool skinnedGeometryUsesTargetBones(RE::BSGeometry* geometry, const std::unordered_set<RE::NiAVObject*>& targetNodes)
        {
            if (!geometry || targetNodes.empty()) {
                return false;
            }

            auto* skinInstance = geometry->GetRuntimeData().skinInstance.get();
            if (!skinInstance || !skinInstance->bonesData || skinInstance->bonesCount == 0 || skinInstance->bonesCount > kMaxSkinBoneCount) {
                return false;
            }

            auto** boneNodes = reinterpret_cast<RE::NiAVObject**>(skinInstance->bonesData);
            for (std::uint32_t i = 0; i < skinInstance->bonesCount; ++i) {
                auto* bone = boneNodes[i];
                if (bone && targetNodes.find(bone) != targetNodes.end()) {
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] bool isSkinnedToTargetNodes(RE::NiAVObject* visualRoot, const std::unordered_set<RE::NiAVObject*>& targetNodes, int depth)
        {
            if (!visualRoot || depth < 0) {
                return false;
            }

            if (auto* geometry = visualRoot->IsGeometry()) {
                return skinnedGeometryUsesTargetBones(geometry, targetNodes);
            }

            auto* node = visualRoot->IsNode();
            if (!node) {
                return false;
            }

            auto& children = node->GetRuntimeData().children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                auto* child = children[i].get();
                if (child && isSkinnedToTargetNodes(child, targetNodes, depth - 1)) {
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]] bool isSkinnedToNode(RE::NiAVObject* visualRoot, RE::NiAVObject* hitNode)
        {
            if (!visualRoot || !hitNode) {
                return false;
            }

            std::unordered_set<RE::NiAVObject*> targetNodes;
            targetNodes.reserve(32);
            collectDownstreamNodesNoCollision(hitNode, targetNodes, kMaxNodeSearchDepth);
            return isSkinnedToTargetNodes(visualRoot, targetNodes, kMaxNodeSearchDepth);
        }

        [[nodiscard]] bool betterEquipmentCandidate(const ActorEquipmentSelection& candidate, const ActorEquipmentSelection& current) noexcept
        {
            if (!current.valid) {
                return true;
            }

            const auto candidateSlot = slotIndex(candidate.slot);
            const auto currentSlot = slotIndex(current.slot);
            const int candidatePriority = bipedSlotPriority(candidateSlot);
            const int currentPriority = bipedSlotPriority(currentSlot);
            if (candidatePriority != currentPriority) {
                return candidatePriority < currentPriority;
            }

            if (candidate.disconnected != current.disconnected) {
                return candidate.disconnected;
            }

            if (candidate.skinned != current.skinned) {
                return candidate.skinned;
            }

            return candidateSlot < currentSlot;
        }
    }

    const char* dropStatusName(DropStatus status) noexcept
    {
        switch (status) {
        case DropStatus::Success:
            return "success";
        case DropStatus::InvalidActor:
            return "invalid-actor";
        case DropStatus::NotDeadActor:
            return "not-dead-actor";
        case DropStatus::MissingSelection:
            return "missing-selection";
        case DropStatus::MissingInventoryStack:
            return "missing-inventory-stack";
        case DropStatus::RemoveItemFailed:
            return "remove-item-failed";
        case DropStatus::DroppedReferenceUnavailable:
            return "dropped-reference-unavailable";
        default:
            return "unknown";
        }
    }

    bool nodeContainsNode(RE::NiAVObject* root, RE::NiAVObject* target, int maxDepth)
    {
        if (!root || !target || maxDepth < 0) {
            return false;
        }
        if (root == target) {
            return true;
        }

        auto* node = root->IsNode();
        if (!node) {
            return false;
        }

        auto& children = node->GetRuntimeData().children;
        for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
            auto* child = children[i].get();
            if (child && nodeContainsNode(child, target, maxDepth - 1)) {
                return true;
            }
        }
        return false;
    }

    ActorEquipmentSelection resolveFarActorEquipmentSelection(
        RE::TESObjectREFR* ref,
        RE::NiAVObject* hitNode,
        const RE::NiPoint3& hitPointWorld,
        bool hasHitPoint)
    {
        ActorEquipmentSelection best{};
        best.actorFormId = ref ? ref->GetFormID() : 0;
        best.hitNode = hitNode;
        best.hitPointWorld = hitPointWorld;
        best.hasHitPoint = hasHitPoint;

        auto* actor = asDeadActor(ref);
        if (!actor || !hitNode) {
            return best;
        }

        auto* biped = actor->GetBiped().get();
        if (!biped) {
            return best;
        }

        const auto totalSlots = static_cast<std::uint32_t>(std::to_underlying(RE::BIPED_OBJECT::kEditorCount));
        for (std::uint32_t i = 0; i < totalSlots; ++i) {
            if (!validBipedSlot(i)) {
                continue;
            }

            auto& bipObject = biped->object[i];
            auto* visualNode = bipObject.partClone.get();
            auto* itemForm = bipObject.parent.object;
            auto* item = itemForm ? itemForm->As<RE::TESBoundObject>() : nullptr;
            auto* instanceData = bipObject.parent.instanceData.get();
            if (!visualNode || !playableActorWearableForm(item, instanceData)) {
                continue;
            }

            const bool disconnected = nodeContainsNode(visualNode, hitNode, kMaxNodeSearchDepth);
            const bool skinned = !disconnected && isSkinnedToNode(visualNode, hitNode);
            if (!disconnected && !skinned) {
                continue;
            }

            const auto stack = findEquippedStack(actor, item, instanceData);
            if (!stack.found || stack.count == 0) {
                continue;
            }

            ActorEquipmentSelection candidate{};
            candidate.valid = true;
            candidate.slot = static_cast<RE::BIPED_OBJECT>(i);
            candidate.actorFormId = actor->GetFormID();
            candidate.itemFormId = item->GetFormID();
            candidate.item = item;
            candidate.instanceData = stack.instanceData ? stack.instanceData : instanceData;
            candidate.extraList = stack.extraList;
            candidate.hitNode = hitNode;
            candidate.visualNode = visualNode;
            candidate.hitPointWorld = hitPointWorld;
            candidate.stackIndex = stack.stackIndex;
            candidate.stackCount = stack.count;
            candidate.hasHitPoint = hasHitPoint;
            candidate.disconnected = disconnected;
            candidate.skinned = skinned;

            if (betterEquipmentCandidate(candidate, best)) {
                best = candidate;
            }
        }

        if (best.valid) {
            ROCK_LOG_DEBUG(Hand,
                "Resolved far actor equipment: actor={:08X} item={:08X} slot={} stack={} count={} visual='{}' hit='{}' disconnected={} skinned={}",
                best.actorFormId,
                best.itemFormId,
                slotIndex(best.slot),
                best.stackIndex,
                best.stackCount,
                nodeName(best.visualNode),
                nodeName(best.hitNode),
                best.disconnected ? "yes" : "no",
                best.skinned ? "yes" : "no");
        }

        return best;
    }

    DropResult dropFarActorEquipmentSelection(RE::TESObjectREFR* actorRef, const ActorEquipmentSelection& selection, float attachedDropZOffsetGameUnits)
    {
        DropResult result{};
        result.actorFormId = actorRef ? actorRef->GetFormID() : 0;
        result.itemFormId = selection.itemFormId;

        if (!isActorRef(actorRef)) {
            result.status = DropStatus::InvalidActor;
            return result;
        }
        if (!actorRef->IsDead(false)) {
            result.status = DropStatus::NotDeadActor;
            return result;
        }
        if (!selection.isUsable()) {
            result.status = DropStatus::MissingSelection;
            return result;
        }

        const auto stack = findEquippedStack(actorRef, selection.item, selection.instanceData);
        if (!stack.found || stack.count == 0 || stack.stackIndex == kInvalidStackIndex) {
            result.status = DropStatus::MissingInventoryStack;
            return result;
        }

        RE::NiPoint3 dropLoc = selection.hasHitPoint ? selection.hitPointWorld :
                               selection.hitNode ? selection.hitNode->world.translate :
                               RE::NiPoint3{ actorRef->data.location.x, actorRef->data.location.y, actorRef->data.location.z };
        if (selection.disconnected && selection.visualNode) {
            dropLoc = selection.visualNode->world.translate;
        } else {
            dropLoc.z += std::isfinite(attachedDropZOffsetGameUnits) ? attachedDropZOffsetGameUnits : kDefaultAttachedDropZOffsetGameUnits;
        }

        RE::NiPoint3 dropRot{};
        const RE::NiPoint3* dropRotPtr = nullptr;
        if (selection.disconnected && selection.visualNode) {
            dropRot = grab_node_info_math::nifskopeMatrixToEulerRadians<RE::NiMatrix3, RE::NiPoint3>(selection.visualNode->world.rotate);
            dropRotPtr = &dropRot;
        }

        auto removeCount = static_cast<std::int32_t>((std::min)(stack.count, static_cast<std::uint32_t>((std::numeric_limits<std::int32_t>::max)())));
        if (!selection.item->Is(RE::ENUM_FORM_ID::kAMMO)) {
            removeCount = 1;
        }

        RE::TESObjectREFR::RemoveItemData removeData(selection.item, removeCount);
        removeData.reason = RE::ITEM_REMOVE_REASON::KDropping;
        removeData.dropLoc = &dropLoc;
        removeData.rotate = dropRotPtr;
        removeData.stackData.push_back(stack.stackIndex);

        result.handle = actorRef->RemoveItem(removeData);
        if (!result.handle) {
            result.status = DropStatus::RemoveItemFailed;
            return result;
        }

        result.droppedRef = result.handle.get();
        if (!result.droppedRef) {
            result.status = DropStatus::DroppedReferenceUnavailable;
            return result;
        }

        result.status = DropStatus::Success;
        result.droppedFormId = result.droppedRef->GetFormID();
        ROCK_LOG_INFO(Hand,
            "Dropped far actor equipment: actor={:08X} item={:08X} dropped={:08X} stack={} count={} loc=({:.1f},{:.1f},{:.1f}) disconnected={}",
            result.actorFormId,
            result.itemFormId,
            result.droppedFormId,
            stack.stackIndex,
            removeCount,
            dropLoc.x,
            dropLoc.y,
            dropLoc.z,
            selection.disconnected ? "yes" : "no");
        return result;
    }
}
