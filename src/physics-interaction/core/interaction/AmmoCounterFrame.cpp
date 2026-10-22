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
        ammo_counter_pose::Point toPoint(const RE::NiPoint3& value) noexcept
        {
            return { value.x, value.y, value.z };
        }
    }

    void PhysicsInteraction::publishAmmoCounters()
    {
        std::array<ammo_counters::HandCounter, 2> hands{};
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
                // The slot's firing seat in its weapon's frame, final after
                // AfterWorldFinal. It needs no hand on the weapon.
                RE::NiTransform seat{}, weaponWorld{};
                if (slot == 0) {
                    const RE::NiNode* weaponNode = nullptr;
                    if (!_twoHandedGrip.tryGetFiringSeat(isLeft, weaponNode, seat) || !weaponNode) continue;
                    weaponWorld = weaponNode->world;
                } else if (!native_secondary_presentation::tryGetFiringSeat(seat, weaponWorld)) {
                    continue;
                }
                if (!finiteNiTransform(weaponWorld) || !finiteNiTransform(seat)) continue;
                const auto palmInWeapon = computeGrabPalmPointWorldFromHandBasis(seat, isLeft);
                const auto palm = transform_math::localPointToWorld(weaponWorld, palmInWeapon);
                const auto weaponRight = transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const auto weaponUp = transform_math::rotateLocalVectorToWorld(weaponWorld.rotate, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                auto& counter = hands[index];
                counter.pose = ammo_counter_pose::resolve(toPoint(palm), toPoint(weaponRight), toPoint(weaponUp), isLeft, head);
                counter.shown = counter.pose.valid;
                counter.loaded = state.loadedRounds;
                counter.reserve = state.reserveRounds;
            }
        }
        // The counters replace the vanilla one only while RPS UI shows them.
        const bool replacing = ammo_counters::update(hands) && native_ammo_counter_visibility::install();
        native_ammo_counter_visibility::replace(replacing);
    }
}
