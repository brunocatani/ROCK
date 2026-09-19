#include "api/OwnedEventStream.h"
#include <cassert>

struct Event { std::uint64_t sequence{}; std::uint32_t value{}; };
int main() {
    rock::provider::OwnedEventStream<Event,3,2> stream;
    using rock::api::Status;
    assert(stream.bind(0)==Status::InvalidArgument);
    assert(stream.bind(1)==Status::Ok && stream.bind(2)==Status::Ok);
    assert(stream.bind(3)==Status::CapacityFull);
    for (std::uint32_t i=0;i<9;++i) stream.publish({0,i},1);
    stream.publish({0,99},2);
    Event output[3]{}; rock::api::StreamV1 state{};
    assert(stream.copy(2,0,output,3,state)==Status::Ok);
    assert(state.lostCount==0 && state.copiedCount==1 && output[0].value==99);
    assert(stream.copy(1,0,output,1,state)==Status::Ok);
    assert(state.lostCount==6 && state.copiedCount==1 && state.remainingCount==2 && state.nextSequence==7);
    assert(output[0].value==6);
    assert(stream.copy(1,state.nextSequence,output,3,state)==Status::Ok);
    assert(state.lostCount==0 && state.copiedCount==2 && state.nextSequence==9);
    stream.publish({0,123});
    assert(stream.copy(2,1,output,3,state)==Status::Ok && output[0].value==123);
    stream.bind(2); // Binding upgrade must retain the same stream.
    assert(stream.copy(2,0,output,3,state)==Status::Ok && state.copiedCount==2);
    stream.remove(1);assert(stream.bind(3)==Status::Ok);
    assert(stream.copy(1,0,output,3,state)==Status::PermissionDenied);
    assert(stream.copy(3,0,nullptr,0,state)==Status::Ok && !state.copiedCount);
    assert(stream.copy(3,99,output,3,state)==Status::InvalidArgument);
}
