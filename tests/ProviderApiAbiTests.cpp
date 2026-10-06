#include <ModularAbiLayoutChecks.h>
#include <ROCK/InventoryDisplay.h>
#include <ROCK/StorageInteractionV1_1.h>
#include "api/StorageInteractionPolicy.h"
using namespace rock::storage_interaction_policy;
static_assert(supports(1,48) && !supports(2,48) && !supports(1,56));
static_assert(supportsGripMinor(1,0,48) && supportsGripMinor(1,1,48) && supportsGripMinor(1,1,56));
static_assert(!supportsGripMinor(2,0,56) && !supportsGripMinor(1,2,56) && !supportsGripMinor(1,1,47));
static_assert(supportsInventory(1,0,48) && !supportsInventory(1,1,48) && !supportsInventory(2,0,48));
static_assert(!supportsInventory(1,0,40) && !supportsInventory(1,0,56));
static_assert(sizeof(rock::api::storage_interaction::ApiV1)==48);
static_assert(sizeof(rock::api::storage_interaction::v1_1::Api)==56);
int main() {}
