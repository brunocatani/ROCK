#pragma once

namespace RE
{
    class TESForm;
    class TBO_InstanceData;
    class EquippedWeaponData;
    class BGSObjectInstanceExtra;
}

namespace rock
{
    // Borrowed for one synchronous read. The collision owner pins a physical
    // item and its data; the legacy owner resolves its normal equipped item.
    struct WeaponCollisionSource
    {
        RE::TESForm* form{};
        RE::TBO_InstanceData* instance{};
        RE::EquippedWeaponData* data{};
        const RE::BGSObjectInstanceExtra* mods{};
    };
}
