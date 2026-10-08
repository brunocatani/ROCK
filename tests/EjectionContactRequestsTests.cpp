#include "api/EjectionContactRequests.h"
#include <cassert>
#include "physics-interaction/native/MagazineEjectionSnapshot.h"
int main() {
    using namespace rock;using namespace api;
    EjectionContactRequests requests;
    ejection_contacts::RequestV1 target;target.reference={41,88,2,3,4};target.weaponGenerationKey=99;target.weaponForm=7;target.expectedBase=8;
    assert(requests.request(1,target)==Status::RequestQueued);
    requests.entries[0].waiting=.7f;
    assert(requests.request(1,target)==Status::RequestQueued&&requests.entries[0].waiting==.7f);
    requests.entries[0].active=true;requests.entries[0].age=.9f;
    assert(requests.request(1,target)==Status::Ok&&requests.entries[0].age==.9f);
    assert(requests.request(2,target)==Status::OwnerConflict);
    ++target.reference.referenceNativeHandle;assert(requests.request(1,target)==Status::GenerationMismatch);--target.reference.referenceNativeHandle;
    ++target.reference.skeletonGeneration;assert(requests.request(1,target)==Status::GenerationMismatch);--target.reference.skeletonGeneration;
    requests.cancel(2,0);assert(!requests.entries[0].cancelled);
    requests.cancel(1,41);assert(requests.request(1,target)==Status::TargetUnavailable);
    requests={};
    for(unsigned i=0;i<8;++i){target.reference.referenceFormId=i+1;assert(requests.request(1,target)==Status::RequestQueued);}
    target.reference.referenceFormId=100;assert(requests.request(1,target)==Status::CapacityFull);
    requests.entries[0].finished=true;assert(requests.request(1,target)==Status::RequestQueued);
    requests.entries[0].active=true;requests.entries[0].finished=true;
    assert(requests.request(1,target)==Status::Ok&&requests.entries[0].finished);
    requests.cancel(1,target.reference.referenceFormId);
    assert(requests.request(1,target)==Status::RequestQueued&&!requests.entries[0].finished);
    requests.entries[0].finished=true;++target.reference.referenceNativeHandle;
    assert(requests.request(1,target)==Status::RequestQueued&&!requests.entries[0].finished);
    using namespace native_player_collision;
    MagazineEjectionSnapshot contacts;contacts.world=123;
    const BodyIdentity magazine{100,5,1000,2000}, weapon{200,6,3000,4000}, floor{300,0,5000,6000};
    contacts.magazines[0]=magazine;contacts.magazineCount=1;
    contacts.weapons[0]=weapon;contacts.weaponCount=1;
    assert(contacts.containsPair(100,200)&&contacts.containsPair(200,100));
    assert(contacts.suppresses(123,magazine,weapon)&&contacts.suppresses(123,weapon,magazine));
    assert(!contacts.suppresses(123,magazine,floor)&&!contacts.suppresses(123,weapon,floor));
    assert(!contacts.suppresses(124,magazine,weapon));
    auto recycled=magazine;++recycled.collisionObject;
    assert(!contacts.suppresses(123,recycled,weapon));
    recycled=weapon;++recycled.ownerNode;
    assert(!contacts.suppresses(123,magazine,recycled));
    contacts={};assert(!contacts.suppresses(123,magazine,weapon));
}
