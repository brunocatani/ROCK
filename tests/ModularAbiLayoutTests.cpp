#pragma pack(push,1)
#include <ROCK/InventoryDisplayV1_1.h>
static_assert(sizeof(rock::api::inventory_display::v1_1::Api)==56);
#include <ROCK/EquipPickup.h>
static_assert(sizeof(rock::api::equip_pickup::Api)==16 && alignof(rock::api::equip_pickup::Api)==8);
struct InventoryDisplayCallerPacking { char c; std::uint64_t value; };
static_assert(alignof(InventoryDisplayCallerPacking)==1);
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
#include <ROCK/GrabV1_1.h>
static_assert(sizeof(rock::api::grab::ApiV1)==136);
static_assert(sizeof(rock::api::grab::v1_1::Api)==160);
static_assert(offsetof(rock::api::grab::v1_1::Api,getHeldPlacementState)==136);
static_assert(sizeof(rock::api::grab::v1_1::HeldPlacementState)==96);
static_assert(sizeof(rock::api::grab::v1_1::HeldPlacementIntent)==56);
#include <ROCK/GrabV1_2.h>
#include <ROCK/GrabV1_3.h>
static_assert(sizeof(rock::api::grab::v1_3::Api)==200);
static_assert(offsetof(rock::api::grab::v1_3::Api,v1_2)==0);
static_assert(offsetof(rock::api::grab::v1_3::Api,setHeldObjectTarget)==176);
static_assert(offsetof(rock::api::grab::v1_3::Api,clearHeldObjectTarget)==184);
static_assert(offsetof(rock::api::grab::v1_3::Api,getHeldObjectPose)==192);
static_assert(sizeof(rock::api::grab::v1_3::HeldObjectPose)==112);
static_assert(sizeof(rock::api::grab::v1_3::HeldObjectTarget)==128);
static_assert(offsetof(rock::api::grab::v1_3::HeldObjectTarget,rootWorld)==72);
static_assert(sizeof(rock::api::grab::v1_2::Api)==176);
static_assert(offsetof(rock::api::grab::v1_2::Api,v1_1)==0);
static_assert(offsetof(rock::api::grab::v1_2::Api,setDefaultGrabPoses)==160);
static_assert(offsetof(rock::api::grab::v1_2::Api,clearDefaultGrabPoses)==168);
static_assert(sizeof(rock::api::grab::v1_2::DefaultHandPose)==132);
static_assert(sizeof(rock::api::grab::v1_2::DefaultGrabPoses)==272);
static_assert(offsetof(rock::api::grab::v1_2::DefaultGrabPoses,right)==8);
static_assert(offsetof(rock::api::grab::v1_2::DefaultGrabPoses,left)==140);
#include <ROCK/Storage.h>
static_assert(sizeof(rock::api::storage::ApiV1) == 72);
static_assert(offsetof(rock::api::storage::ApiV1, claim) == 8);
static_assert(offsetof(rock::api::storage::ApiV1, draw) == 64);
static_assert(offsetof(rock::api::storage::TargetV1, reference) == 8);
static_assert(offsetof(rock::api::storage::TargetV1, hand) == 28);
static_assert(offsetof(rock::api::storage::TargetV1, gripSequence) == 32);
#include <ROCK/StorageFeedback.h>
static_assert(sizeof(rock::api::storage_feedback::CandidateV1)==40);
static_assert(offsetof(rock::api::storage_feedback::CandidateV1,hand)==8);
static_assert(offsetof(rock::api::storage_feedback::CandidateV1,reference)==12);
static_assert(offsetof(rock::api::storage_feedback::CandidateV1,worldGeneration)==20);
static_assert(offsetof(rock::api::storage_feedback::CandidateV1,gripSequence)==32);
static_assert(sizeof(rock::api::storage_feedback::ApiV1)==24);
static_assert(offsetof(rock::api::storage_feedback::ApiV1,setCandidate)==8);
static_assert(offsetof(rock::api::storage_feedback::ApiV1,clearCandidate)==16);
#include <ROCK/StorageDisplay.h>
static_assert(sizeof(rock::api::storage_display::SelectionV1)==96);
static_assert(offsetof(rock::api::storage_display::SelectionV1,hand)==8);
static_assert(offsetof(rock::api::storage_display::SelectionV1,parentReference)==12);
static_assert(offsetof(rock::api::storage_display::SelectionV1,storedReference)==16);
static_assert(offsetof(rock::api::storage_display::SelectionV1,worldGeneration)==20);
static_assert(offsetof(rock::api::storage_display::SelectionV1,nodeName)==32);
static_assert(sizeof(rock::api::storage_display::ApiV1)==24);
static_assert(offsetof(rock::api::storage_display::ApiV1,setSelection)==8);
static_assert(offsetof(rock::api::storage_display::ApiV1,clearSelection)==16);
#include <ROCK/StorageInteraction.h>
static_assert(sizeof(rock::api::storage_interaction::DisplayV1)==104);
static_assert(offsetof(rock::api::storage_interaction::DisplayV1,displayId)==8);
static_assert(offsetof(rock::api::storage_interaction::DisplayV1,nodeName)==40);
static_assert(sizeof(rock::api::storage_interaction::ShoulderRoutingV1)==32);
static_assert(sizeof(rock::api::storage_interaction::ContactV1)==72);
static_assert(offsetof(rock::api::storage_interaction::ContactV1,displayId)==40);
static_assert(sizeof(rock::api::storage_interaction::ApiV1)==48);
static_assert(offsetof(rock::api::storage_interaction::ApiV1,setDisplay)==8);
static_assert(offsetof(rock::api::storage_interaction::ApiV1,clearOwner)==40);
#include <ROCK/StorageInteractionV1_1.h>

static_assert(sizeof(rock::api::inventory_display::Api)==48);
static_assert(alignof(rock::api::inventory_display::Api)==8);
static_assert(offsetof(rock::api::inventory_display::Api,setDisplay)==8);
static_assert(offsetof(rock::api::inventory_display::Api,clearOwner)==40);
static_assert(sizeof(rock::api::inventory_display::Display)==144);
static_assert(offsetof(rock::api::inventory_display::Display,item)==16);
static_assert(offsetof(rock::api::inventory_display::Display,nodeName)==80);
static_assert(sizeof(rock::api::inventory_display::Contact)==120);
static_assert(sizeof(rock::api::inventory_display::GripPose)==1056);
static_assert(offsetof(rock::api::inventory_display::GripPose,item)==56);
static_assert(offsetof(rock::api::inventory_display::GripPose,displayWorld)==128);
static_assert(sizeof(rock::api::storage_interaction::ApiV1)==48);
static_assert(sizeof(rock::api::storage_interaction::v1_1::Api)==56);
static_assert(offsetof(rock::api::storage_interaction::v1_1::Api,getAuthoredGrip)==48);
static_assert(sizeof(rock::api::storage_interaction::v1_1::GripRequest)==56);
static_assert(sizeof(rock::api::storage_interaction::v1_1::GripPose)==1008);
static_assert(alignof(rock::api::storage_interaction::v1_1::GripPose)==8);
int main() {}
