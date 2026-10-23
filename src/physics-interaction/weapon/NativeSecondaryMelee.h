#pragma once

namespace RE
{
    class NiAVObject;
}

namespace rock::native_secondary_melee
{
    /*
     * Native VR melee for a melee weapon at equipment index 1.
     *
     * FO4VR attaches its melee contact callback (140EFF000) to the index-0
     * weapon node only (140F0A1B0), and that callback resolves the weapon
     * (E50DA0), the object impact (E56FA0) and the actor hit (E56B10) for the
     * default equip index (1459444C8). ROCK attaches the same native melee
     * collision (F0AC60/F0AD90) to the presented index-1 melee model and, for
     * a contact from one of that model's bodies, gives those four indexed
     * calls index 1. Cooldown, target filters, actions, impulse, damage, perks
     * and hit events stay native. native_melee_wand gives such a contact the
     * wand of the index-1 hand, and native_secondary_model keeps the model's
     * collision, which the first-person builder strips from every weapon
     * outside the race weapon slot.
     */
    [[nodiscard]] bool install() noexcept;

    // Frame owner. The presented index-1 melee model, or nullptr when there is
    // none (holstered, unequipped, a firearm, not presented). Attaches native
    // melee collision to a new model and detaches it from a replaced one.
    void present(RE::NiAVObject* model) noexcept;

    // Native model retirement, before the record is released: detaches the
    // attached model when this record retires it.
    void beforeModelRetired(const RE::NiAVObject* root) noexcept;

    // Any thread, around one native melee contact callback: a contact from a
    // body of the attached index-1 model routes the callback to index 1.
    class ImpactRoute
    {
    public:
        ImpactRoute(const void* contactEvent, const void* collisionEvent) noexcept;
        ~ImpactRoute();
        ImpactRoute(const ImpactRoute&) = delete;
        ImpactRoute& operator=(const ImpactRoute&) = delete;
        [[nodiscard]] bool routed() const noexcept { return _routed; }

    private:
        bool _routed = false;
        bool _previous = false;
    };

    // Inside one native melee contact callback: the index-1 route claimed it.
    [[nodiscard]] bool routedContact() noexcept;
}
