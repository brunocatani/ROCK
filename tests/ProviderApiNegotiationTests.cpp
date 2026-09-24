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
    };
    verify(core::table(),core::kSupportedPermissions);
    verify(hands::table(),hands::kSupportedPermissions);
    verify(collision::table(),collision::kSupportedPermissions);
    verify(grab::table(),grab::kSupportedPermissions);
    verify(touch::table(),touch::kSupportedPermissions);
    verify(weapon::tableV1_1(),weapon::kSupportedPermissions);
    verify(weaponparts::table(),weaponparts::kSupportedPermissions);
    verify(animation::table(),animation::kSupportedPermissions);
    verify(input::tableV1_1(),input::kSupportedPermissions);
    {
        const InterfaceDescriptorV1* oldInput{};
        const InterfaceDescriptorV1* newInput{};
        assert(ROCKAPI_QueryInterfaceV1(input::kInterfaceId,1,0,sizeof(input::ApiV1),&oldInput)==Status::Ok);
        assert(ROCKAPI_QueryInterfaceV1(input::kInterfaceId,1,1,sizeof(input::v1_1::Api),&newInput)==Status::Ok);
        assert(oldInput->table==newInput->table && oldInput->requiredCoreMinor==0);
        const auto* oldCalls=static_cast<const input::ApiV1*>(oldInput->table);
        assert(oldCalls->clearHandInputSuppressionV1(77,Hand::Left)==Status::Ok);
        input::v1_1::DecorationState state;
        assert(static_cast<const input::v1_1::Api*>(newInput->table)->getDecorationState(77,&state)==Status::Ok);
        assert(state.flags==static_cast<std::uint32_t>(input::v1_1::DecorationFlag::InputReserved));
        assert(oldCalls->clearHandInputSuppressionV1(77,Hand::Left)==Status::Ok);
    }
    verify(references::table(),references::kSupportedPermissions);
    verify(playercontroller::table(),playercontroller::kSupportedPermissions);
    verify(diagnostics::table(),diagnostics::kSupportedPermissions);
    verify(configuration::table(),configuration::kSupportedPermissions);
    // The unchanged Weapon.h is the released 1.0 consumer. Discover its old
    // extent, bind the old major, and call it alongside an opted-in 1.1 caller.
    const InterfaceDescriptorV1* oldWeapon{};
    const InterfaceDescriptorV1* newWeapon{};
    assert(ROCKAPI_QueryInterfaceV1(InterfaceId::Weapon, 1, 0, sizeof(weapon::ApiV1), &oldWeapon) == Status::Ok);
    assert(ROCKAPI_QueryInterfaceV1(InterfaceId::Weapon, 1, 1, sizeof(weapon::v1_1::Api), &newWeapon) == Status::Ok);
    assert(oldWeapon == newWeapon && oldWeapon->minor == 1 && oldWeapon->requiredCoreMinor == 0);
    const auto* oldApi = static_cast<const weapon::ApiV1*>(oldWeapon->table);
    const auto* newApi = static_cast<const weapon::v1_1::Api*>(newWeapon->table);
    assert(oldApi == &newApi->v1 && oldApi->getPrimaryHandV1 == newApi->v1.getPrimaryHandV1);
    Hand primary{};
    assert(oldApi->getPrimaryHandV1(77, &primary) == Status::Ok && primary == Hand::Left);
    for (const auto hand : {Hand::Left, Hand::Right}) {
        weapon::v1_1::EquipRequest request{};
        request.hand = hand;
        std::uint64_t command{};
        assert(newApi->requestInventoryEquip(77, &request, &command) == Status::RequestQueued);
        assert(command == (hand == Hand::Left ? 101 : 102));
        assert(oldApi->getPrimaryHandV1(77, &primary) == Status::Ok);
    }
    rock::provider::InterfaceBinding oldBinding{}, newBinding{};
    assert(rock::provider::bindInterface(oldBinding, 1, 1, weapon::kSupportedPermissions) == Status::Ok);
    assert(rock::provider::bindInterface(newBinding, 1, 3, weapon::kSupportedPermissions) == Status::Ok);
    const std::array oldOnly{discovery::RegisteredInterface{
        {40, InterfaceId::Weapon, 1, 0, sizeof(weapon::ApiV1), 1, 0, 0, oldApi}, 3}};
    assert(discovery::query(oldOnly, InterfaceId::Weapon, 1, 1, sizeof(weapon::v1_1::Api), &result) == Status::UnsupportedMinor);
    assert(!result);
    rock::provider::InterfaceBinding collision{},grabbing{};
    assert(rock::provider::bindInterface(collision,2,1,3)==Status::Ok);
    assert(rock::provider::bindInterface(grabbing,1,3,3)==Status::Ok);
    assert(rock::provider::bindInterface(collision,1,1,3)==Status::Busy);
    assert(collision.major==2 && grabbing.major==1);
    assert(rock::provider::bindInterface(collision,2,3,3)==Status::Ok);
    assert(rock::provider::bindInterface(collision,2,1,3)==Status::Busy);
    assert(rock::provider::bindInterface(collision,2,7,3)==Status::InvalidArgument);
}
