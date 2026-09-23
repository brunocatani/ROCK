#pragma once
#include <ROCK/WeaponV1_1.h>
#include "physics-interaction/weapon/WeaponEquipTransfer.h"
#include <array>

namespace rock {
    // Owned by PhysicsInteraction; only the ROCK frame thread accesses it.
    // The single native equipped slot admits one switch at a time.
    struct InventoryWeaponEquipRuntime {
        struct Completed {
            api::OwnerToken owner{};
            api::weapon::v1_1::EquipResult result{};
        };
        api::OwnerToken owner{};
        api::weapon::v1_1::EquipRequest request{};
        api::weapon::v1_1::EquipResult result{};
        weapon_equip_transfer::InventorySelection selection{};
        std::uint64_t nextCommandId{1};
        std::uintptr_t previousNativeNode{}; // Comparison only; never dereferenced.
        bool cancelRequested{};
        std::array<Completed, 64> completed{};
        std::size_t nextCompleted{};

        [[nodiscard]] bool active() const noexcept { return owner != 0; }
        void finish(api::weapon::v1_1::EquipState state, api::weapon::v1_1::EquipFailure failure) noexcept {
            result.state = state;
            result.failure = failure;
            completed[nextCompleted++ % completed.size()] = {owner, result};
            selection = {};
            owner = 0;
            cancelRequested = false;
            previousNativeNode = 0;
        }
    };
}
