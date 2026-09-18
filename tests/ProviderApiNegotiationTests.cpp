#include "api/InterfaceNegotiation.h"
#include "api/OwnerBindingPolicy.h"
#include <array>
#include <cassert>
int main() {
    using namespace rock::api;
    constexpr std::uint64_t core=1, collision1=2, collision2=3, grab=4;
    const std::array descriptors{
        InterfaceDescriptorV1{40,InterfaceId::Core,1,0,8,1,0,0,&core},
        InterfaceDescriptorV1{40,InterfaceId::Collision,1,2,8,1,0,0,&collision1},
        InterfaceDescriptorV1{40,InterfaceId::Collision,2,0,8,1,0,0,&collision2},
        InterfaceDescriptorV1{40,InterfaceId::Grab,1,0,8,1,0,0,&grab}
    };
    const InterfaceDescriptorV1* result=nullptr;
    const auto query=[&](InterfaceId id,std::uint32_t major,std::uint32_t minor=0,std::uint32_t bytes=8) {
        result=&descriptors[0];
        const auto status=discovery::query(descriptors,id,major,minor,bytes,&result);
        if (status!=Status::Ok) assert(!result);
        return status;
    };
    assert(query(InterfaceId::Collision,2)==Status::Ok && result->table==&collision2);
    assert(query(InterfaceId::Grab,1)==Status::Ok && result->table==&grab);
    assert(query(InterfaceId::Core,1)==Status::Ok && result->table==&core);
    assert(query(InterfaceId::Collision,1,2)==Status::Ok && result->table==&collision1);
    assert(query(InterfaceId::Collision,1,3)==Status::UnsupportedMinor);
    assert(query(InterfaceId::Collision,3)==Status::UnsupportedMajor);
    assert(query(InterfaceId::Hands,1)==Status::UnknownInterface);
    assert(query(InterfaceId::Core,1,0,16)==Status::TableTooSmall);
    assert(query(InterfaceId::Core,0)==Status::InvalidArgument);
    assert(discovery::query(descriptors,InterfaceId::Core,1,0,8,nullptr)==Status::InvalidArgument);
    rock::provider::InterfaceBinding collision{},grabbing{};
    assert(rock::provider::bindInterface(collision,2,1,3)==Status::Ok);
    assert(rock::provider::bindInterface(grabbing,1,3,3)==Status::Ok);
    assert(rock::provider::bindInterface(collision,1,1,3)==Status::Busy);
    assert(collision.major==2 && grabbing.major==1);
    assert(rock::provider::bindInterface(collision,2,3,3)==Status::Ok);
    assert(rock::provider::bindInterface(collision,2,1,3)==Status::Busy);
    assert(rock::provider::bindInterface(collision,2,7,3)==Status::InvalidArgument);
}
