// LooseItemModelService: WeaponParts 2.1 loose item models. A consumer names,
// per misc base form, the object mod whose model every loose reference of that
// base shows instead of its own geometry. The model is put on the incoming 3D
// in ROCK's reference 3D attach hook (LooseWeaponStateService.cpp), before the engine
// publishes it; LooseWeaponCollision gives such references hulls generated
// from that model and keeps them while they rest.
#pragma once
#include <ROCK/WeaponParts.h>
#include <span>

namespace RE
{
    class TESObjectREFR;
    class NiAVObject;
}

namespace rock::loose_item_model
{
    namespace contract = api::weaponparts;

    // Any thread (provider). Replaces the owner's set; count 0 clears it.
    api::Status set(api::OwnerToken owner, std::span<const contract::LooseItemModel> models);
    api::Status clear(api::OwnerToken owner);

    // 3D attach hook, any loading thread: shows the registered model on a detached
    // incoming model. Never touches a model that already has a parent. A tree that
    // already carries another part model (a reference the engine recycled with its
    // 3D) has that part replaced.
    void present(RE::TESObjectREFR* reference, RE::NiAVObject* model) noexcept;

    // Frame thread: the same for the reference's live 3D, before a hand takes it.
    // A dropped item can come back as a recycled reference whose 3D still shows
    // the part model of the item that reference held before.
    void refresh(RE::TESObjectREFR* reference) noexcept;

    // Frame thread. Presented references whose current 3D still carries the
    // presented model and whose base is still registered.
    std::size_t references(std::span<std::uint32_t> output);
    bool presents(std::uint32_t reference);
}
