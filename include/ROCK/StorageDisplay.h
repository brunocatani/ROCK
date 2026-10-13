#pragma once
#include "Abi.h"
#pragma pack(push,8)
namespace rock::api::storage_display
{
    inline constexpr char kExport[] = "ROCK_StorageDisplay_QueryV1";
    struct SelectionV1 {
        std::uint32_t size{sizeof(SelectionV1)}, version{1};
        Hand hand{Hand::None};
        std::uint32_t parentReference{}, storedReference{}, worldGeneration{}, skeletonGeneration{}, providerGeneration{};
        char nodeName[64]{};
    };
    struct ApiV1 {
        std::uint32_t size{sizeof(ApiV1)}, version{1};
        // Owner frame thread, Grab Read|Control. storedReference must have this
        // owner's committed Storage claim; parentReference must be held by the
        // opposite hand, while hand is empty. nodeName names a loaded child of
        // that parent. No engine pointer is accepted or borrowed from a caller.
        // Steady green selection, three-frame refresh lease. Yields to held-item
        // pulsing and other native selection owners (Busy). Clears on detach,
        // ownership/tracking/world loss. Provider retains the node during its lease;
        // caller clears when no longer selected. Detached nodes expire safely.
        Status(ROCK_CALL* setSelection)(OwnerToken,const SelectionV1*) noexcept;
        Status(ROCK_CALL* clearSelection)(OwnerToken,Hand) noexcept;
    };
    using Query = const ApiV1*(ROCK_CALL*)(std::uint32_t major,std::uint32_t bytes) noexcept;
    static_assert(sizeof(SelectionV1)==96 && sizeof(ApiV1)==24);
}
#pragma pack(pop)
