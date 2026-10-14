#pragma pack(push,1)
#include <ROCK/Storage.h>
static_assert(sizeof(rock::api::storage::Api)==152 && sizeof(rock::api::storage::Display)==160);
#include <ROCK/Weapon.h>
static_assert(sizeof(rock::api::weapon::Api)==136 && sizeof(rock::api::weapon::WeaponState)==112);
struct StorageCallerPacking { char c; std::uint64_t value; };
static_assert(alignof(StorageCallerPacking)==1);
#pragma pack(pop)
#include "ModularAbiLayoutChecks.h"
#pragma pack(push,1)
#include <ROCK/WeaponParts.h>
struct ItemStateCallerPacking {char c; std::uint64_t value;};
static_assert(alignof(ItemStateCallerPacking)==1);
#pragma pack(pop)
static_assert(sizeof(rock::api::weaponparts::Api)==176 && sizeof(rock::api::weaponparts::ItemState)==29964);
#pragma pack(push, 1)
#include <ROCK/Touch.h>
struct TouchCallerPacking {char c; std::uint64_t value;};
static_assert(alignof(TouchCallerPacking)==1);
#pragma pack(pop)
int main() {}
