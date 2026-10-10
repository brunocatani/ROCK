#include "api/LooseWeaponStatePath.h"
#include <cassert>
int main() {
    using rock::loose_weapon_state::relativeItemPath;
    constexpr auto path="Weapon  (0D01EC2E)/P-Receiver/WEAPON/WeaponExtra4";
    assert(relativeItemPath("Weapon",path)==path);
    assert(relativeItemPath("Weapon  (0D01EC2E)",path)=="P-Receiver/WEAPON/WeaponExtra4");
    assert(relativeItemPath("Weapon  (0D01EC2E)","Weapon  (0D01EC2E)").empty());
    assert(relativeItemPath("Weapon  (0D01EC2E)","Weapon  (0D01EC2E) extra/WeaponBolt")=="Weapon  (0D01EC2E) extra/WeaponBolt");
    assert(relativeItemPath("Weapon  (2400D8A5)",path)==path);
    assert(relativeItemPath("",path)==path);
    assert(relativeItemPath("WeaponMagazine","WeaponMagazine/WeaponMagazineChild1")=="WeaponMagazineChild1");
}
