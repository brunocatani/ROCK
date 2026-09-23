#pragma once
#include <ROCK/WeaponV1_1.h>

namespace rock::inventory_weapon_equip_policy {
    using api::Status;
    using api::weapon::v1_1::InventoryWeapon;
    [[nodiscard]] constexpr Status validateCapture(const InventoryWeapon& requested, const InventoryWeapon& current) noexcept {
        if (!requested.worldGeneration || !requested.skeletonGeneration || !requested.providerGeneration ||
            requested.frameIndex != current.frameIndex || requested.worldGeneration != current.worldGeneration ||
            requested.skeletonGeneration != current.skeletonGeneration || requested.providerGeneration != current.providerGeneration)
            return Status::GenerationMismatch;
        if (!requested.stackKey || !requested.count || requested.baseFormId != current.baseFormId ||
            requested.stackIndex != current.stackIndex || requested.stackKey != current.stackKey ||
            requested.instanceKey != current.instanceKey || requested.count != current.count) return Status::TargetUnavailable;
        return Status::Ok;
    }
    enum class Destination { Unavailable, ReplaceUncarried, RetainOtherHand };
    [[nodiscard]] constexpr Destination destination(bool left, bool destinationBusy,
        bool rightCarries, bool leftCarries, bool equippedOccupiesHand) noexcept {
        if (destinationBusy || (left ? leftCarries : rightCarries)) return Destination::Unavailable;
        if (left ? rightCarries : leftCarries) return Destination::RetainOtherHand;
        return equippedOccupiesHand ? Destination::Unavailable : Destination::ReplaceUncarried;
    }
}
