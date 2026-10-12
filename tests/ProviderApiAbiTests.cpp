#include <ModularAbiLayoutChecks.h>
#include <ROCK/InventoryDisplayV1_1.h>
static_assert(sizeof(rock::api::inventory_display::Api)==48 && sizeof(rock::api::inventory_display::v1_1::Api)==56);
#include <ROCK/StorageInteractionV1_1.h>
#include "api/StorageInteractionPolicy.h"
using namespace rock::storage_interaction_policy;
static_assert(supportsInventoryMinor(1,0,48) && supportsInventoryMinor(1,1,56));
static_assert(!supportsInventoryMinor(2,1,56) && !supportsInventoryMinor(1,2,56) && !supportsInventoryMinor(1,1,64));
static_assert(buildsCollision(false,false) && buildsCollision(true,true) && !buildsCollision(true,false));
static_assert(supports(1,48) && !supports(2,48) && !supports(1,56));
static_assert(supportsGripMinor(1,0,48) && supportsGripMinor(1,1,48) && supportsGripMinor(1,1,56));
static_assert(!supportsGripMinor(2,0,56) && !supportsGripMinor(1,2,56) && !supportsGripMinor(1,1,47));
static_assert(supportsInventory(1,0,48) && !supportsInventory(1,1,48) && !supportsInventory(2,0,48));
static_assert(!supportsInventory(1,0,40) && !supportsInventory(1,0,56));
static_assert(sizeof(rock::api::storage_interaction::ApiV1)==48);
static_assert(sizeof(rock::api::storage_interaction::v1_1::Api)==56);
// Adding the inventory range must not stretch the old 16-display rotation
// into a 48-frame cycle or repeatedly favor legacy slot zero.
static_assert([] {
    constexpr auto legacy=rock::api::storage_interaction::kMaximumDisplays;
    constexpr auto inventory=rock::api::inventory_display::kMaximumDisplays;
    for(std::uint64_t frame=0;frame<2*inventory;++frame) {
        if(preparationIndex(0,frame,legacy)!=frame%legacy) return false;
        for(std::size_t i=0;i<legacy;++i)
            if(preparationIndex(i,frame,legacy)!=(i+frame)%legacy) return false;
        for(std::size_t i=0;i<inventory;++i)
            if(preparationIndex(i,frame,inventory)!=(i+frame)%inventory) return false;
    }
    return true;
}());
#include "physics-interaction/weapon/EquipPickupScope.h"
#include <cstdlib>
#include <thread>
int main() {
    using namespace rock::equip_pickup;
    auto check=[](bool value){if(!value) std::abort();};
    check(query(1,0,16) && !query(1,1,16) && !query(2,0,16) && !query(1,0,8) && !query(1,0,24));
    check(!matches(100,1));
    {
        Scope outer(100,1); check(matches(100,1) && !matches(101,1) && !matches(100,2));
        std::thread other([&]{check(!matches(100,1));}); other.join();
        { Scope inner(200,2); check(matches(200,2) && !matches(100,1)); }
        check(matches(100,1));
    }
    check(!matches(100,1));
    try { Scope failing(300,1); throw 1; } catch(...) {}
    check(!matches(300,1) && !matches(0,0));
}
