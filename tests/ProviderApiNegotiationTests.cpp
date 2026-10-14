#include "api/InterfaceNegotiation.h"
#include "api/OwnerBindingPolicy.h"
#include <array>
#include <cassert>
#include "ModularTestTables.h"
int main() {
    using namespace rock::api;
    constexpr std::uint64_t coreTable=1, collision1=2, collision2=3, grabTable=4;
    const std::array<discovery::RegisteredInterface,4> descriptors{
        discovery::RegisteredInterface{{40,InterfaceId::Core,1,0,8,1,0,0,&coreTable},5},
        discovery::RegisteredInterface{{40,InterfaceId::Collision,1,2,8,1,0,0,&collision1},3},
        discovery::RegisteredInterface{{40,InterfaceId::Collision,2,0,8,1,0,0,&collision2},1},
        discovery::RegisteredInterface{{40,InterfaceId::Grab,1,0,8,1,0,0,&grabTable},3}
    };
    const InterfaceDescriptorV1* result=nullptr;
    const auto query=[&](InterfaceId id,std::uint32_t major,std::uint32_t minor=0,std::uint32_t bytes=8) {
        result=&descriptors[0].descriptor;
        const auto status=discovery::query(descriptors,id,major,minor,bytes,&result);
        if (status!=Status::Ok) assert(!result);
        return status;
    };
    assert(query(InterfaceId::Collision,2)==Status::Ok && result->table==&collision2);
    assert(query(InterfaceId::Grab,1)==Status::Ok && result->table==&grabTable);
    assert(query(InterfaceId::Core,1)==Status::Ok && result->table==&coreTable);
    assert(query(InterfaceId::Collision,1,2)==Status::Ok && result->table==&collision1);
    assert(query(InterfaceId::Collision,1,3)==Status::UnsupportedMinor);
    assert(query(InterfaceId::Collision,3)==Status::UnsupportedMajor);
    assert(query(InterfaceId::Hands,1)==Status::UnknownInterface);
    assert(query(InterfaceId::Core,1,0,16)==Status::TableTooSmall);
    assert(query(InterfaceId::Core,0)==Status::InvalidArgument);
    assert(discovery::query(descriptors,InterfaceId::Core,1,0,8,nullptr)==Status::InvalidArgument);
    const auto* collision2Registration=discovery::findRegistration(descriptors,InterfaceId::Collision,2);
    assert(collision2Registration && collision2Registration->permissions==1);
    assert(!discovery::findRegistration(descriptors,InterfaceId::Collision,3));

    // Exercise the production descriptor list/export, not just synthetic metadata.
    const auto verify=[]<class Table>(const Table& table,std::uint32_t permissions) {
        const InterfaceDescriptorV1* descriptor{};
        assert(ROCKAPI_QueryInterfaceV1(Table::interfaceId,Table::majorVersion,Table::minorVersion,sizeof(Table),&descriptor)==Status::Ok);
        assert(descriptor && descriptor->table==&table && descriptor->tableByteSize==sizeof(Table));
        assert(descriptor->major==Table::majorVersion && descriptor->minor==Table::minorVersion);
        assert(descriptor->requiredCoreMajor==core::kMajor && descriptor->requiredCoreMinor==core::kMinor);
        const auto* entry=discovery::findRegistration(discovery::registeredInterfaces(),Table::interfaceId,Table::majorVersion);
        assert(entry && entry->permissions==permissions);
        assert(ROCKAPI_QueryInterfaceV1(Table::interfaceId,Table::majorVersion,Table::minorVersion+1,sizeof(Table),&descriptor)==Status::UnsupportedMinor);
        assert(!descriptor);
        assert(ROCKAPI_QueryInterfaceV1(Table::interfaceId,Table::majorVersion,0,sizeof(Table)+1,&descriptor)==Status::TableTooSmall);
        assert(!descriptor);
        if constexpr (Table::majorVersion>1) {
            // A major-1 consumer is refused cleanly, never handed the major-2 table.
            descriptor=reinterpret_cast<const InterfaceDescriptorV1*>(&table);
            assert(ROCKAPI_QueryInterfaceV1(Table::interfaceId,1,0,8,&descriptor)==Status::UnsupportedMajor);
            assert(!descriptor);
        }
    };
    verify(core::table(),core::kSupportedPermissions);
    verify(rock::native_interaction_service::table(),native_interaction::kSupportedPermissions);
    verify(hands::table(),hands::kSupportedPermissions);
    verify(collision::table(),collision::kSupportedPermissions);
    verify(storage::table(),storage::kSupportedPermissions);
    verify(grab::table(),grab::kSupportedPermissions);
    {
        const auto& api=grab::table();
        grab::DefaultGrabPoses poses;
        poses.baseFormId=100; poses.left.flags=poses.right.flags=1;
        assert(api.setDefaultGrabPoses(78,&poses)==Status::Ok);
        assert(api.setDefaultGrabPoses(79,&poses)==Status::OwnerConflict);
        assert(api.clearDefaultGrabPoses(79,100)==Status::OwnerConflict);
        assert(api.clearDefaultGrabPoses(78,100)==Status::Ok);
        // References (10) is retired; its queries are part of Grab.
        const InterfaceDescriptorV1* retired{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::References,1,0,8,&retired)!=Status::Ok && !retired);
    }
    verify(touch::table(),touch::kSupportedPermissions);
    {
        const InterfaceDescriptorV1* current{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::Touch,2,0,sizeof(touch::Api),&current)==Status::Ok);
        const auto* api=static_cast<const touch::Api*>(current->table);
        assert(api->requestTouchGrabYield(77,5,6,7)==Status::Ok);
        touch::AuthoredHand request{};request.hand=Hand::Right;request.referenceFormId=42;request.flags=3;
        assert(api->setAuthoredHand(78,&request)==Status::Ok);
        assert(api->setAuthoredHand(79,&request)==Status::OwnerConflict);
        assert(api->clearAuthoredHand(79,Hand::Right)==Status::OwnerConflict);
        assert(api->clearAuthoredHand(78,Hand::Right)==Status::Ok);
    }
    verify(weapon::table(),weapon::kSupportedPermissions);
    verify(weaponparts::table(),weaponparts::kSupportedPermissions);
    verify(animation::table(),animation::kSupportedPermissions);
    verify(input::table(),input::kSupportedPermissions);
    verify(playercontroller::table(),playercontroller::kSupportedPermissions);
    {
        // Diagnostics (12) is retired: other mods draw through RPS UI Framework.
        const InterfaceDescriptorV1* retired{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::Diagnostics,1,0,8,&retired)!=Status::Ok && !retired);
    }
    verify(configuration::table(),configuration::kSupportedPermissions);
    // Weapon major 1 and its 1.1 minor are refused.
    const InterfaceDescriptorV1* oldWeapon{};
    assert(ROCKAPI_QueryInterfaceV1(InterfaceId::Weapon, 1, 0, 8, &oldWeapon) == Status::UnsupportedMajor && !oldWeapon);
    rock::provider::InterfaceBinding collision{},grabbing{};
    assert(rock::provider::bindInterface(collision,2,1,3)==Status::Ok);
    assert(rock::provider::bindInterface(grabbing,1,3,3)==Status::Ok);
    assert(rock::provider::bindInterface(collision,1,1,3)==Status::Busy);
    assert(collision.major==2 && grabbing.major==1);
    assert(rock::provider::bindInterface(collision,2,3,3)==Status::Ok);
    assert(rock::provider::bindInterface(collision,2,1,3)==Status::Busy);
    assert(rock::provider::bindInterface(collision,2,7,3)==Status::InvalidArgument);
}
