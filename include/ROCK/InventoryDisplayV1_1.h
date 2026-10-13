#pragma once
#include "InventoryDisplay.h"
#pragma pack(push,8)
namespace rock::api::inventory_display::v1_1 {
    inline constexpr char kExport[]="ROCK_InventoryDisplay_QueryV1_1";
    struct Api {
        inventory_display::Api v1;
        // Same validation, ownership, lease, and authored poses as setDisplay.
        // Creates no collision geometry/body and reports no collision contacts.
        // Switching modes retires the old body at the next safe physics update.
        // The v1.setDisplay entry always retains its original colliding behavior.
        Status(ROCK_CALL* setDisplayWithoutCollision)(OwnerToken,const Display*) noexcept;
    };
    using Query=const Api*(ROCK_CALL*)(std::uint32_t major,std::uint32_t minor,std::uint32_t bytes) noexcept;
    static_assert(sizeof(Api)==56 && offsetof(Api,setDisplayWithoutCollision)==48);
}
#pragma pack(pop)
