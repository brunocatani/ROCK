#pragma once

#include "physics-interaction/weapon/PipboyEquipPolicy.h"

#include <cstdint>

namespace rock::pipboy_equip_runtime
{
    struct SelectionEvent
    {
        std::uint64_t sequence{ 0 };
        std::uint32_t handleId{ 0 };
        std::uint32_t stackId{ 0 };
        std::uint32_t formId{ 0 };
        pipboy_equip_policy::Hand requestedHand{ pipboy_equip_policy::Hand::Right };
        pipboy_equip_policy::TriggerSource triggerSource{ pipboy_equip_policy::TriggerSource::FallbackRight };
        bool equipped{ false };
    };

    struct AssignmentSnapshot
    {
        std::uint64_t sequence{ 0 };
        std::uint32_t handleId{ 0 };
        std::uint32_t stackId{ 0 };
        std::uint32_t formId{ 0 };
        pipboy_equip_policy::Hand hand{ pipboy_equip_policy::Hand::Right };
        bool active{ false };
    };

    struct StackSnapshot
    {
        std::uint32_t formId{ 0 };
        bool resolved{ false };
        bool weapon{ false };
        bool equipped{ false };
    };

    bool installHooks();
    bool consumeSelectionEvent(std::uint64_t& lastSequence, SelectionEvent& outEvent);
    bool getAssignment(AssignmentSnapshot& outAssignment);
    bool inspectStack(std::uint32_t handleId, std::uint32_t stackId, StackSnapshot& outSnapshot);

    void setEquipMode(pipboy_equip_policy::EquipMode mode);
    void setLeftHandEquipAvailable(bool available);
    void publishWeaponAssignment(
        std::uint32_t handleId,
        std::uint32_t stackId,
        std::uint32_t formId,
        pipboy_equip_policy::Hand hand);
    void clearWeaponAssignment(std::uint32_t handleId = 0, std::uint32_t stackId = 0);
    void resetRuntimeState();
}
