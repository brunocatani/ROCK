#pragma pack(push,1)
#include <ROCK/Storage.h>
static_assert(sizeof(rock::api::storage::Api)==152 && sizeof(rock::api::storage::Display)==152);
#include <ROCK/EquipPickup.h>
static_assert(sizeof(rock::api::equip_pickup::Api)==16 && alignof(rock::api::equip_pickup::Api)==8);
struct StorageCallerPacking { char c; std::uint64_t value; };
static_assert(alignof(StorageCallerPacking)==1);
#pragma pack(pop)
#include "ModularAbiLayoutChecks.h"
#pragma pack(push,1)
#include <ROCK/WeaponPartsV1_7.h>
struct ItemStateCallerPacking {char c; std::uint64_t value;};
static_assert(alignof(ItemStateCallerPacking)==1);
#pragma pack(pop)
static_assert(sizeof(rock::api::weaponparts::v1_6::Api)==200);
static_assert(sizeof(rock::api::weaponparts::v1_7::Api)==224);
static_assert(offsetof(rock::api::weaponparts::v1_7::Api,setItemState)==200);
static_assert(offsetof(rock::api::weaponparts::v1_7::Api,moveItemState)==208);
static_assert(offsetof(rock::api::weaponparts::v1_7::Api,clearItemStates)==216);
static_assert(sizeof(rock::api::weaponparts::v1_7::ItemState)==29960);
#pragma pack(push, 1)
#include <ROCK/Touch.h>
struct TouchCallerPacking {char c; std::uint64_t value;};
static_assert(alignof(TouchCallerPacking)==1);
#pragma pack(pop)
#include <ROCK/WeaponPartsV1_2.h>
static_assert(sizeof(rock::api::weaponparts::ApiV1)==144);
static_assert(sizeof(rock::api::weaponparts::v1_2::Api)==160);
static_assert(offsetof(rock::api::weaponparts::v1_2::Api,setCollisionSuppression)==144);
static_assert(offsetof(rock::api::weaponparts::v1_2::Api,clearCollisionSuppression)==152);
static_assert(sizeof(rock::api::weaponparts::v1_2::CollisionSuppression)==184);
static_assert(offsetof(rock::api::weaponparts::v1_2::CollisionSuppression,sample)==8);
static_assert(offsetof(rock::api::weaponparts::v1_2::CollisionSuppression,sourceKeys)==56);
int main() {}
