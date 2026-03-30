#pragma once

#include <cstdint>

#include "physics-interaction/object/GrabTargetKind.h"

#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiSmartPointer.h"

namespace rock::actor_equipment_grab_policy
{
    /*
     * ROCK pulls worn gear by removing the equipped item through the actor
     * equipment path, spawning the dropped reference, then handing that reference
     * to the normal pull/grab runtime. Keeping this path separate from physical
     * actor-body grabbing prevents dead NPC selections from falling through to
     * whole-body pull. The pure policy remains a gate for call sites that need
     * explicit verification state; the verified runtime executor below is only
     * entered by the far-selection pre-pass.
     */
    enum class BeginPullDecision : std::uint8_t
    {
        Allow,
        NotActorEquipment,
        LiveActorDisabled,
        NativeExecutorNotVerified,
        MissingEquippedItem,
    };

    struct BeginPullInput
    {
        grab_target::Kind targetKind{ grab_target::Kind::None };
        bool actorIsLive{ false };
        bool liveActorLootEnabled{ false };
        bool nativeExecutorVerified{ false };
        bool hasEquippedItem{ false };
    };

    [[nodiscard]] inline constexpr BeginPullDecision decideBeginPull(BeginPullInput input) noexcept
    {
        if (input.targetKind != grab_target::Kind::ActorEquipment) {
            return BeginPullDecision::NotActorEquipment;
        }

        if (input.actorIsLive && !input.liveActorLootEnabled) {
            return BeginPullDecision::LiveActorDisabled;
        }

        if (!input.nativeExecutorVerified) {
            return BeginPullDecision::NativeExecutorNotVerified;
        }

        if (!input.hasEquippedItem) {
            return BeginPullDecision::MissingEquippedItem;
        }

        return BeginPullDecision::Allow;
    }

    [[nodiscard]] inline constexpr const char* decisionName(BeginPullDecision decision) noexcept
    {
        switch (decision) {
        case BeginPullDecision::Allow:
            return "allow";
        case BeginPullDecision::NotActorEquipment:
            return "not-actor-equipment";
        case BeginPullDecision::LiveActorDisabled:
            return "live-actor-disabled";
        case BeginPullDecision::NativeExecutorNotVerified:
            return "native-executor-not-verified";
        case BeginPullDecision::MissingEquippedItem:
            return "missing-equipped-item";
        default:
            return "unknown";
        }
    }
}

namespace rock::actor_equipment_grab
{
    /*
     * ROCK resolves actor clothing from far-hit biped visuals, removes the
     * equipped inventory stack from the actor, and then pulls the newly dropped
     * reference as a normal physics object. This authority split prevents
     * close/body grabs from becoming clothing-strips while still letting the
     * standard dynamic pull/grab code own the spawned item. Whole NPC bodies and
     * live actor control stay out of this module.
     */
    inline constexpr float kDefaultAttachedDropZOffsetGameUnits = 20.0f;
    inline constexpr std::uint32_t kInvalidStackIndex = 0xFFFF'FFFFu;

    struct ActorEquipmentSelection
    {
        bool valid = false;
        RE::BIPED_OBJECT slot = RE::BIPED_OBJECT::kNone;
        std::uint32_t actorFormId = 0;
        std::uint32_t itemFormId = 0;
        RE::TESBoundObject* item = nullptr;
        RE::TBO_InstanceData* instanceData = nullptr;
        RE::ExtraDataList* extraList = nullptr;
        RE::NiAVObject* hitNode = nullptr;
        RE::NiAVObject* visualNode = nullptr;
        RE::NiPoint3 hitPointWorld{};
        std::uint32_t stackIndex = kInvalidStackIndex;
        std::uint32_t stackCount = 0;
        bool hasHitPoint = false;
        bool disconnected = false;
        bool skinned = false;

        [[nodiscard]] bool isUsable() const noexcept
        {
            return valid && item != nullptr && stackIndex != kInvalidStackIndex && stackCount > 0;
        }
    };

    enum class DropStatus : std::uint8_t
    {
        Success,
        InvalidActor,
        NotDeadActor,
        MissingSelection,
        MissingInventoryStack,
        RemoveItemFailed,
        DroppedReferenceUnavailable,
    };

    struct DropResult
    {
        DropStatus status = DropStatus::MissingSelection;
        RE::ObjectRefHandle handle{};
        RE::NiPointer<RE::TESObjectREFR> droppedRef;
        std::uint32_t actorFormId = 0;
        std::uint32_t itemFormId = 0;
        std::uint32_t droppedFormId = 0;
    };

    [[nodiscard]] const char* dropStatusName(DropStatus status) noexcept;

    [[nodiscard]] bool nodeContainsNode(RE::NiAVObject* root, RE::NiAVObject* target, int maxDepth = 32);

    [[nodiscard]] ActorEquipmentSelection resolveFarActorEquipmentSelection(
        RE::TESObjectREFR* ref,
        RE::NiAVObject* hitNode,
        const RE::NiPoint3& hitPointWorld,
        bool hasHitPoint);

    [[nodiscard]] DropResult dropFarActorEquipmentSelection(
        RE::TESObjectREFR* actorRef,
        const ActorEquipmentSelection& selection,
        float attachedDropZOffsetGameUnits = kDefaultAttachedDropZOffsetGameUnits);
}
