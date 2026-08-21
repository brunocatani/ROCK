#include "rock_support/Fo4VrActorStatePolicy.h"
#include "physics-interaction/weapon/NativeReloadHandAuthorityPolicy.h"

#include <cstdio>

namespace
{
    bool expect(const char* message, const bool condition)
    {
        if (!condition) {
            std::fprintf(stderr, "FAILED: %s\n", message);
        }
        return condition;
    }
}

int main()
{
    using namespace rock::fo4vr_actor_state_policy;

    bool ok = true;
    ok &= expect("the verified actor-state storage offset must remain 0x0C",
        kWeaponStateStorageOffset == 0x0C);
    ok &= expect("the verified weapon-state field must begin at bit two",
        kWeaponStateShift == 2);
    ok &= expect("the verified weapon-state field must remain three bits wide",
        kWeaponStateValueMask == 0x7);
    ok &= expect("the verified gun-state field must begin at bit fifteen",
        kGunStateShift == 15);
    ok &= expect("the verified gun-state field must remain four bits wide",
        kGunStateValueMask == 0xF);
    ok &= expect("the verified native reload state must remain four",
        kReloadingGunState == 4);

    for (std::uint32_t state = 0; state <= 7; ++state) {
        constexpr std::uint32_t unrelatedBits = 0xA5A5A5A5u;
        const std::uint32_t storage =
            (unrelatedBits & ~(kWeaponStateValueMask << kWeaponStateShift)) |
            (state << kWeaponStateShift);
        ok &= expect("weapon-state decoding must ignore every unrelated bit",
            decodeWeaponState(storage) == state);
    }

    ok &= expect("native Drawing storage must decode as state two",
        decodeWeaponState(0x08u) == 2);
    ok &= expect("native Drawn storage must decode as state three",
        decodeWeaponState(0x0Cu) == 3);

    for (std::uint32_t state = 0; state <= 15; ++state) {
        constexpr std::uint32_t unrelatedBits = 0x5A5A5A5Au;
        const std::uint32_t storage =
            (unrelatedBits & ~(kGunStateValueMask << kGunStateShift)) |
            (state << kGunStateShift);
        ok &= expect("gun-state decoding must ignore every unrelated bit",
            decodeGunState(storage) == state);
    }
    ok &= expect("native Reloading storage must decode as gun state four",
        decodeGunState(4u << kGunStateShift) == 4);
    ok &= expect("native gun state four must report reload authority",
        isNativeReloading(4));
    ok &= expect("non-reload and invalid gun states must fail closed",
        !isNativeReloading(0) &&
            !isNativeReloading(3) &&
            !isNativeReloading(5) &&
            !isNativeReloading(kInvalidGunState));

    ok &= expect("Sheathed must not report weapon magic drawn",
        !isWeaponMagicDrawn(0));
    ok &= expect("Drawing must not report weapon magic drawn",
        !isWeaponMagicDrawn(2));
    ok &= expect("Drawn must report weapon magic drawn",
        isWeaponMagicDrawn(3));
    ok &= expect("WantToSheathe must remain drawn until native holster completes",
        isWeaponMagicDrawn(4));
    ok &= expect("Sheathing must remain drawn until native holster completes",
        isWeaponMagicDrawn(5));
    ok &= expect("invalid bit patterns must fail closed",
        !isWeaponMagicDrawn(6) &&
            !isWeaponMagicDrawn(7) &&
            !isWeaponMagicDrawn(kInvalidWeaponState));

    using ReloadInput =
        rock::native_reload_hand_authority_policy::Input;
    using ReloadState =
        rock::native_reload_hand_authority_policy::State;
    using rock::native_reload_hand_authority_policy::update;

    ReloadState postFire{};
    ok &= expect("a normal firing state must not reserve the support hand",
        !update(postFire, ReloadInput{
            .gunState = 7,
            .frameIndex = 100,
            .firingTriggerHeld = true,
        }));
    ok &= expect("post-fire state four must not become reload hand authority",
        !update(postFire, ReloadInput{
            .gunState = 4,
            .frameIndex = 101,
            .firingTriggerHeld = false,
            .magazineCountKnown = true,
            .magazineEmpty = false,
        }));
    ok &= expect("a post-fire state-four interval must remain excluded",
        !update(postFire, ReloadInput{
            .gunState = 4,
            .frameIndex = 140,
            .magazineCountKnown = true,
            .magazineEmpty = false,
        }));

    ReloadState routedReload{};
    ok &= expect("a routed reload request must reserve the support hand",
        update(routedReload, ReloadInput{
            .gunState = 4,
            .frameIndex = 200,
            .reloadDispatchSequence = 1,
            .magazineCountKnown = true,
            .magazineEmpty = false,
        }));

    ReloadState emptyReload{};
    ok &= expect("an empty-magazine reload must reserve the support hand",
        update(emptyReload, ReloadInput{
            .gunState = 4,
            .frameIndex = 300,
            .firingTriggerHeld = true,
            .magazineCountKnown = true,
            .magazineEmpty = true,
        }));

    ReloadState providerReload{};
    ok &= expect("provider arms or hands authority must always win",
        update(providerReload, ReloadInput{
            .gunState = 7,
            .frameIndex = 400,
            .providerOwnsArmsOrHands = true,
            .firingTriggerHeld = true,
        }));

    return ok ? 0 : 1;
}
