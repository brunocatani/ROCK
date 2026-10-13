#include "api/ObjectRestRequests.h"
#include <cassert>
int main() {
    using namespace rock;using namespace api;
    object_rest_service::Requests requests;
    collision::ObjectRestRequest target;target.reference.referenceFormId=44;target.expectedBase=77;
    collision::ObjectRestState state;
    assert(requests.watch(1,target,state)==Status::RequestQueued&&state==collision::ObjectRestState::Waiting);
    assert(requests.watch(1,target,state)==Status::RequestQueued);
    assert(requests.watch(2,target,state)==Status::OwnerConflict);
    ++target.expectedBase;assert(requests.watch(1,target,state)==Status::InvalidArgument);--target.expectedBase;
    ++target.reference.worldGeneration;assert(requests.watch(1,target,state)==Status::GenerationMismatch);--target.reference.worldGeneration;
    requests.entries[0].status=Status::Ok;requests.entries[0].state=collision::ObjectRestState::Watching;
    assert(requests.watch(1,target,state)==Status::Ok&&state==collision::ObjectRestState::Watching);
    requests.cancel(2,0);assert(!requests.entries[0].cancelled);
    requests.cancel(1,45);assert(!requests.entries[0].cancelled);
    requests.entries[0].state=collision::ObjectRestState::Resting;
    assert(requests.watch(1,target,state)==Status::Ok&&state==collision::ObjectRestState::Resting&&!requests.entries[0].owner);
    for(unsigned i=0;i<collision::kMaxObjectRestWatches;++i){target.reference.referenceFormId=i+1;assert(requests.watch(1,target,state)==Status::RequestQueued);}
    target.reference.referenceFormId=100;assert(requests.watch(1,target,state)==Status::CapacityFull);
    requests.cancel(1,0);for(const auto& entry:requests.entries)assert(entry.cancelled);
    target.reference.referenceFormId=1;assert(requests.watch(1,target,state)==Status::Busy);
    requests={};assert(requests.watch(1,target,state)==Status::RequestQueued);
    requests.entries[0].status=Status::TargetUnavailable;
    assert(requests.watch(1,target,state)==Status::TargetUnavailable&&!requests.entries[0].owner);
}
