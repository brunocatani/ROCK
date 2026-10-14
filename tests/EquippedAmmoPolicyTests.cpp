#include "physics-interaction/weapon/EquippedAmmoPolicy.h"

#include <cstdio>

namespace
{
    using namespace rock::equipped_ammo;

    bool expect(const char* message, const bool condition)
    {
        if (!condition) {
            std::fprintf(stderr, "FAILED: %s\n", message);
        }
        return condition;
    }

    bool shows(const Counts& counts, std::uint32_t loaded, std::uint32_t reserve)
    {
        return counts.known && counts.loaded == loaded && counts.reserve == reserve;
    }
}

int main()
{
    bool ok = true;
    const Facts gun{ .weapon = true, .hasAmmo = true, .loaded = 8, .inventoryCount = 50 };

    ok &= expect("no ranged weapon shows no counter", !resolve({}).known);
    auto notPlayable = gun;
    notPlayable.notPlayable = true;
    ok &= expect("a not-playable weapon shows no counter", !resolve(notPlayable).known);
    ok &= expect("a weapon without ammo shows zero", shows(resolve({ .weapon = true }), 0, 0));

    ok &= expect("reserve is the inventory count less the loaded rounds", shows(resolve(gun), 8, 42));
    auto shared = gun;
    shared.otherLoadedSameAmmo = 12;
    ok &= expect("rounds loaded in the other slot's weapon are not reserve", shows(resolve(shared), 8, 30));
    auto exhausted = shared;
    exhausted.inventoryCount = 15;
    ok &= expect("a shared count never goes below zero", shows(resolve(exhausted), 8, 0));

    auto charged = gun;
    charged.charge = 100;
    charged.inventoryCount = 3;
    charged.loaded = 60;
    ok &= expect("charged ammo shows its count as reserve", shows(resolve(charged), 60, 3));

    auto special = shared;
    special.countAsLoaded = true;
    ok &= expect("special ammo counts only the loaded rounds", shows(resolve(special), 8, 0));
    auto nonPlayable = shared;
    nonPlayable.nonPlayableAmmo = true;
    nonPlayable.capacity = 20;
    ok &= expect("non-playable ammo counts the capacity and shares nothing", shows(resolve(nonPlayable), 8, 12));
    return ok ? 0 : 1;
}
