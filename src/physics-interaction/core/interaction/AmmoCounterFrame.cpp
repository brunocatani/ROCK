#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/native/NativeAmmoCounterVisibility.h"
#include "physics-interaction/visual/AmmoCounters.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"

// Ammo counters: one per held weapon at its firing grip, on the palm side of
// its firing hand, from this frame's publication (hand roles, per-slot rounds).

namespace rock
{
    namespace
    {
        using Point = ammo_counter_pose::Point;

        ammo_counter_pose::Point toPoint(const RE::NiPoint3& value) noexcept
        {
            return { value.x, value.y, value.z };
        }

        RE::NiPoint3 toNiPoint(const ammo_counter_pose::Point& value) noexcept
        {
            return { value[0], value[1], value[2] };
        }
    }

    void PhysicsInteraction::publishAmmoCounters()
    {
        std::array<ammo_counters::HandCounter, 2> hands{};
        std::array<bool, 2> anchored{};
        const auto& runtime = runtime_state::currentFrame();
        provider::Publication publication{};
        auto* nodes = f4vr::getPlayerNodes();
        if (_lifecycle.initialized.load(std::memory_order_acquire) && runtime.localSkeletonReady &&
            !runtime.localMenuBlocking && runtime.weaponDrawn && nodes && nodes->HmdNode &&
            provider::runtime::copyPublication(publication) && publication.frameIndex == runtime.frameIndex) {
            const auto head = toPoint(nodes->HmdNode->world.translate);
            using WeaponFlag = api::weapon::WeaponStateFlag;
            constexpr auto required = static_cast<std::uint32_t>(WeaponFlag::AmmoKnown) |
                static_cast<std::uint32_t>(WeaponFlag::WeaponDrawn);
            for (std::uint32_t slot = 0; slot < api::kMaxWeaponSlots; ++slot) {
                api::weapon::WeaponState state{};
                if (provider::runtime::copyPublishedWeaponState(state, slot) != api::Status::Ok ||
                    (state.flags & required) != required ||
                    (state.firingHand != api::Hand::Right && state.firingHand != api::Hand::Left)) {
                    continue;
                }
                // The counter sits at the weapon's firing grip whichever hands
                // hold the weapon, on the palm side of its firing hand.
                const bool isLeft = state.firingHand == api::Hand::Left;
                const auto index = isLeft ? 1u : 0u;
                const auto holds = [&](std::size_t hand) {
                    const auto& role = publication.handRoles[hand];
                    return role.role != api::hands::HandRole::None && role.weaponSlot == slot;
                };
                if (!holds(0) && !holds(1)) continue;
                // The slot's presented weapon and the rendered wrist, both
                // final after AfterWorldFinal.
                const RE::NiAVObject* weapon = slot == 0 ?
                    static_cast<RE::NiAVObject*>(resolveEquippedWeaponInteractionNode()) :
                    native_secondary_presentation::presentedModel();
                if (!weapon || !finiteNiTransform(weapon->world)) continue;
                const auto& weaponWorld = weapon->world;
                const ammo_counter_pose::WeaponAnchor::Identity identity{ slot, state.weaponFormId, state.weaponGenerationKey };
                auto& weaponAnchor = _ammoCounterAnchors[slot];
                anchored[slot] = true;
                // The firing palm is captured while the firing hand holds the
                // firing grip (authored seating puts it on the grip) and kept
                // for this weapon after that hand lets go.
                const Point* palmInWeapon = weaponAnchor.find(identity);
                if (!palmInWeapon) {
                    const auto& firingRole = publication.handRoles[index];
                    RE::NiTransform hand{};
                    if (firingRole.role != api::hands::HandRole::Firing || firingRole.weaponSlot != slot ||
                        !frik_visual_authority::tryGetBoneWorldTransform(isLeft ? "LArm_Hand" : "RArm_Hand", hand)) {
                        continue;
                    }
                    palmInWeapon = &weaponAnchor.update(identity, toPoint(transform_math::worldPointToLocal(
                        weaponWorld, computeGrabPalmPointWorldFromHandBasis(hand, isLeft))));
                }
                const auto palm = transform_math::localPointToWorld(weaponWorld, toNiPoint(*palmInWeapon));
                const auto weaponRight = transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const auto weaponUp = transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                auto& counter = hands[index];
                counter.pose = ammo_counter_pose::resolve(toPoint(palm), toPoint(weaponRight), toPoint(weaponUp), isLeft, head);
                counter.shown = counter.pose.valid;
                counter.loaded = state.loadedRounds;
                counter.reserve = state.reserveRounds;
            }
        }
        // A slot whose weapon is no longer drawn recaptures on its next weapon.
        for (std::size_t slot = 0; slot < anchored.size(); ++slot) {
            if (!anchored[slot]) _ammoCounterAnchors[slot].clear();
        }
        // The counters replace the vanilla one only while RPS UI shows them.
        const bool replacing = ammo_counters::update(hands) && native_ammo_counter_visibility::install();
        native_ammo_counter_visibility::replace(replacing);
    }
}
