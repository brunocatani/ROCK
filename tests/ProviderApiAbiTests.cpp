#include <ModularAbiLayoutChecks.h>
#include <ROCK/Storage.h>
#include "api/StorageInteractionPolicy.h"
using namespace rock::storage_interaction_policy;
static_assert(validRouting(3,3) && validRouting(0,1) && !validRouting(4,3) && !validRouting(1,0) && !validRouting(1,4));
// The display rotation visits every slot once per kMaxDisplays frames.
static_assert([] {
    constexpr auto count=rock::api::storage::kMaxDisplays;
    for(std::uint64_t frame=0;frame<2*count;++frame)
        for(std::size_t i=0;i<count;++i)
            if(preparationIndex(i,frame,count)!=(i+frame)%count) return false;
    return true;
}());
#include "physics-interaction/weapon/EquipPickupScope.h"
#include <cstdlib>
#include <thread>
int main() {
    using namespace rock::equip_pickup;
    auto check=[](bool value){if(!value) std::abort();};
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
