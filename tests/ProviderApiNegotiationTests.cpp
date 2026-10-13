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
    verify(rock::native_interaction_service::tableV1_1(),native_interaction::kSupportedPermissions);
    {
        const InterfaceDescriptorV1 *legacy{},*current{};
        assert(ROCKAPI_QueryInterfaceV1(native_interaction::kInterfaceId,1,0,sizeof(native_interaction::ApiV1),&legacy)==Status::Ok);
        assert(ROCKAPI_QueryInterfaceV1(native_interaction::kInterfaceId,1,1,sizeof(native_interaction::v1_1::Api),&current)==Status::Ok);
        assert(legacy==current&&legacy->requiredCoreMinor==0);
        const auto* oldTable=static_cast<const native_interaction::ApiV1*>(legacy->table);
        assert(oldTable->registerExtension==rock::native_interaction_service::table().registerExtension);
        assert(oldTable->publishVisuals==rock::native_interaction_service::table().publishVisuals);
    }
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
    verify(weapon::tableV1_1(),weapon::kSupportedPermissions);
    verify(weaponparts::tableV1_7(),weaponparts::kSupportedPermissions);
    {
        const InterfaceDescriptorV1 *oldParts{},*newParts{};
        for(const auto minor:{0u,1u}) {
            assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,minor,sizeof(weaponparts::ApiV1),&oldParts)==Status::Ok);
            assert(oldParts->requiredCoreMinor==0);
            assert(static_cast<const weaponparts::ApiV1*>(oldParts->table)->clearWeaponPartTargetsV1(77)==Status::Ok);
        }
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,2,sizeof(weaponparts::v1_2::Api),&newParts)==Status::Ok&&newParts==oldParts);
        const auto* api=static_cast<const weaponparts::v1_2::Api*>(newParts->table);
        weaponparts::v1_2::CollisionSuppression suppression;
        assert(api->setCollisionSuppression(78,&suppression)==Status::Ok);
        assert(api->clearCollisionSuppression(78)==Status::Ok);
        assert(api->v1.clearWeaponPartTargetsV1(77)==Status::Ok);
        const InterfaceDescriptorV1* pointed{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,3,sizeof(weaponparts::v1_3::Api),&pointed)==Status::Ok&&pointed==newParts);
        const auto* points=static_cast<const weaponparts::v1_3::Api*>(pointed->table);weaponparts::v1_3::GrabPoint point;
        assert(points->setGrabPoints(79,&point,1)==Status::Ok);
        assert(points->v1_2.clearCollisionSuppression(78)==Status::Ok&&points->v1_2.v1.clearWeaponPartTargetsV1(77)==Status::Ok);
        const InterfaceDescriptorV1* operating{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,4,sizeof(weaponparts::v1_4::Api),&operating)==Status::Ok&&operating==pointed);
        const auto* operatingPoints=static_cast<const weaponparts::v1_4::Api*>(operating->table);
        assert(operatingPoints->setOperatingGrabPoints(79,&point,1)==Status::Ok);
        assert(operatingPoints->v1_3.setGrabPoints(79,&point,1)==Status::Ok&&operatingPoints->v1_3.clearGrabPoints(77)==Status::Ok);
        const InterfaceDescriptorV1* pinky{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,5,sizeof(weaponparts::v1_5::Api),&pinky)==Status::Ok&&pinky==operating);
        const auto* pinkyPoints=static_cast<const weaponparts::v1_5::Api*>(pinky->table);
        assert(pinkyPoints->setPinkyOperatingGrabPoints(79,&point,1)==Status::Ok);
        assert(pinkyPoints->v1_4.setOperatingGrabPoints(79,&point,1)==Status::Ok);
        const std::array previousOperating{discovery::RegisteredInterface{{40,InterfaceId::WeaponParts,1,4,sizeof(weaponparts::v1_4::Api),1,0,0,&pinkyPoints->v1_4},3}};
        assert(discovery::query(previousOperating,InterfaceId::WeaponParts,1,5,sizeof(*pinkyPoints),&result)==Status::UnsupportedMinor);
        const InterfaceDescriptorV1* anchored{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,6,sizeof(weaponparts::v1_6::Api),&anchored)==Status::Ok&&anchored==pinky);
        const auto* anchoredPoints=static_cast<const weaponparts::v1_6::Api*>(anchored->table);
        assert(anchoredPoints->setAnchoredOperatingGrabPoints(79,&point,1)==Status::Ok);
        assert(anchoredPoints->v1_5.setPinkyOperatingGrabPoints(79,&point,1)==Status::Ok);
        const std::array previousPinky{discovery::RegisteredInterface{{40,InterfaceId::WeaponParts,1,5,sizeof(*pinkyPoints),1,0,0,pinkyPoints},3}};
        assert(discovery::query(previousPinky,InterfaceId::WeaponParts,1,6,sizeof(*anchoredPoints),&result)==Status::UnsupportedMinor);
        const InterfaceDescriptorV1* itemDescriptor{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,7,sizeof(weaponparts::v1_7::Api),&itemDescriptor)==Status::Ok&&itemDescriptor==anchored);
        const auto* items=static_cast<const weaponparts::v1_7::Api*>(itemDescriptor->table);
        weaponparts::v1_7::ItemKey itemKey{123,0x14,7},worldKey{123,0xff001000,0};
        weaponparts::v1_7::ItemState itemState;
        assert(items->setItemState(80,&itemKey,&itemState)==Status::Ok);
        assert(items->moveItemState(80,&itemKey,&worldKey)==Status::Ok);
        assert(anchoredPoints->setAnchoredOperatingGrabPoints(79,&point,1)==Status::Ok);
        assert(items->v1_6.v1_5.v1_4.v1_3.v1_2.v1.clearWeaponPartTargetsV1(77)==Status::Ok);
        const std::array previousItems{discovery::RegisteredInterface{{40,InterfaceId::WeaponParts,1,6,sizeof(*anchoredPoints),1,0,0,anchoredPoints},3}};
        assert(discovery::query(previousItems,InterfaceId::WeaponParts,1,7,sizeof(*items),&result)==Status::UnsupportedMinor);
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,8,sizeof(*items),&result)==Status::UnsupportedMinor);
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,2,0,sizeof(*points),&result)==Status::UnsupportedMajor);
        const std::array previous{discovery::RegisteredInterface{{40,InterfaceId::WeaponParts,1,1,sizeof(weaponparts::ApiV1),1,0,0,&api->v1},3}};
        assert(discovery::query(previous,InterfaceId::WeaponParts,1,2,sizeof(*api),&result)==Status::UnsupportedMinor);
        rock::provider::InterfaceBinding oldBinding{},newBinding{};
        assert(rock::provider::bindInterface(oldBinding,1,3,weaponparts::kSupportedPermissions)==Status::Ok);
        assert(rock::provider::bindInterface(newBinding,1,3,weaponparts::kSupportedPermissions)==Status::Ok);
    }
    verify(animation::table(),animation::kSupportedPermissions);
    verify(input::table(),input::kSupportedPermissions);
    verify(playercontroller::table(),playercontroller::kSupportedPermissions);
    {
        // Diagnostics (12) is retired: other mods draw through RPS UI Framework.
        const InterfaceDescriptorV1* retired{};
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::Diagnostics,1,0,8,&retired)!=Status::Ok && !retired);
    }
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
