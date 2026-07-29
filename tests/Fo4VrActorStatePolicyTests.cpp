#include "rock_support/Fo4VrActorStatePolicy.h"

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

    return ok ? 0 : 1;
}
