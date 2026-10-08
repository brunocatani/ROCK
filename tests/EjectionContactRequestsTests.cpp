#include "api/EjectionContactRequests.h"
#include <cassert>
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
}
