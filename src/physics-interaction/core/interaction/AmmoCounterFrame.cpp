#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "api/ProviderRuntimeServices.h"
#include "api/ProviderStatePolicy.h"
#include "physics-interaction/native/NativeAmmoCounterVisibility.h"
#include "physics-interaction/visual/AmmoCounters.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/HoldingGripRecords.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"
#include "physics-interaction/weapon/WeaponGripSeat.h"
#include "physics-interaction/weapon/WeaponInstanceClassification.h"

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
                if (provider::runtime::copyPublishedWeaponState(state, slot) != api::Status::Ok) continue;
                // Slot 0's hands are applied at read time, as getWeaponState
                // does; slot 1 carries its own in the record.
                if (slot == 0) {
                    provider_state_policy::WeaponHandling handling{};
                    if (queryProviderWeaponHandling(handling)) provider_state_policy::applyHandling(state, handling);
                }
                if ((state.flags & required) != required ||
                    (state.firingHand != api::Hand::Right && state.firingHand != api::Hand::Left)) {
                    continue;
                }
                // handRoles: [0] right, [1] left.
                const auto holds = [&](std::size_t hand, bool firingOnly) {
                    const auto& role = publication.handRoles[hand];
                    return role.weaponSlot == slot &&
                        (firingOnly ? role.role == api::hands::HandRole::Firing : role.role != api::hands::HandRole::None);
                };
                if (!holds(0, false) && !holds(1, false)) continue;
                // The counter sits at the weapon's firing grip on the palm side
                // of the hand that holds it: the firing-grip hand, else the only
                // hand still on the weapon. A detached firing grip keeps its
                // firingHand record, so with akimbo both weapons could name the
                // same hand and one counter would overwrite the other.
                bool isLeft = state.firingHand == api::Hand::Left;
                if (holds(1, true) || holds(0, true)) {
                    isLeft = holds(1, true);
                } else if (holds(1, false) != holds(0, false)) {
                    isLeft = holds(1, false);
                }
                const auto index = isLeft ? 1u : 0u;
                // The slot's firing seat in its weapon's frame, final after
                // AfterWorldFinal. It needs no hand on the weapon.
                RE::NiTransform seat{}, weaponWorld{};
                const auto hidden = [&](const char* stage) {
                    ROCK_LOG_SAMPLE_INFO(Weapon, 2000, "Ammo counter hidden slot={} stage={} firingHand={} form={:08X}",
                        slot, stage, isLeft ? "left" : "right", state.weaponFormId);
                };
                if (slot == 0) {
                    // The same authored seat the firing grip places the hand on.
                    auto* node = resolveEquippedWeaponInteractionNode();
                    auto* weaponForm = currentEquippedWeaponForm();
                    if (!node || !weaponForm) {
                        hidden("weapon-node");
                        continue;
                    }
                    const bool powerArmor = f4vr::isInPowerArmor();
                    const auto native = weapon_grip_seat::resolveNative({
                        .weapon = weaponForm,
                        .weaponRoot = node,
                        .variant = authored_weapon_grip_library::identifyWeaponVariant(node),
                        .keywordFlags = weapon_instance_classification::computeWeaponKeywordFlags(
                            weaponForm, currentEquippedWeaponInstanceData(weaponForm)),
                        .inPowerArmor = powerArmor,
                    });
                    weapon_grip_seat::FiringSeat firing{};
                    if (!weapon_grip_seat::firingSeat(native,
                            holding_grips::find(weaponForm, node, isLeft, state.weaponGenerationKey, powerArmor),
                            isLeft, _weaponCollision.holdingMirrorOffsets().plane(false), firing)) {
                        hidden(native.modelRegistered ? "authored-seat" : "model-frame");
                        continue;
                    }
                    seat = firing.handWeaponLocal;
                    weaponWorld = node->world;
                } else if (!native_secondary_presentation::tryGetFiringSeat(seat, weaponWorld)) {
                    hidden("second-weapon-placement");
                    continue;
                }
                if (!finiteNiTransform(weaponWorld) || !finiteNiTransform(seat)) {
                    hidden("non-finite");
                    continue;
                }
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
