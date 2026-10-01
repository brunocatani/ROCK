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
    verify(grab::tableV1_3(),grab::kSupportedPermissions);
    {
        const InterfaceDescriptorV1 *oldGrab{}, *newGrab{};
        assert(ROCKAPI_QueryInterfaceV1(grab::kInterfaceId,1,0,sizeof(grab::ApiV1),&oldGrab)==Status::Ok);
        assert(ROCKAPI_QueryInterfaceV1(grab::kInterfaceId,1,1,sizeof(grab::v1_1::Api),&newGrab)==Status::Ok);
        assert(oldGrab==newGrab && oldGrab->requiredCoreMinor==0);
        const InterfaceDescriptorV1* defaults{};
        assert(ROCKAPI_QueryInterfaceV1(grab::kInterfaceId,1,2,sizeof(grab::v1_2::Api),&defaults)==Status::Ok);
        assert(defaults==oldGrab && defaults->requiredCoreMinor==0);
        const InterfaceDescriptorV1* guided{};
        assert(ROCKAPI_QueryInterfaceV1(grab::kInterfaceId,1,3,sizeof(grab::v1_3::Api),&guided)==Status::Ok);
        assert(guided==oldGrab && guided->requiredCoreMinor==0);
        const auto* defaultsApi=static_cast<const grab::v1_2::Api*>(defaults->table);
        grab::v1_2::DefaultGrabPoses poses;
        poses.baseFormId=100; poses.left.flags=poses.right.flags=1;
        assert(defaultsApi->setDefaultGrabPoses(78,&poses)==Status::Ok);
        assert(defaultsApi->setDefaultGrabPoses(79,&poses)==Status::OwnerConflict);
        assert(defaultsApi->clearDefaultGrabPoses(79,100)==Status::OwnerConflict);
        assert(defaultsApi->clearDefaultGrabPoses(78,100)==Status::Ok);
        rock::provider::InterfaceBinding oldGrabBinding{},newGrabBinding{};
        assert(rock::provider::bindInterface(oldGrabBinding,1,3,grab::kSupportedPermissions)==Status::Ok);
        assert(rock::provider::bindInterface(newGrabBinding,1,3,grab::kSupportedPermissions)==Status::Ok);
        const std::array priorGrab{discovery::RegisteredInterface{
            {40,InterfaceId::Grab,1,1,sizeof(grab::v1_1::Api),1,0,0,&defaultsApi->v1_1},3}};
        assert(discovery::query(priorGrab,InterfaceId::Grab,1,2,sizeof(grab::v1_2::Api),&result)==Status::UnsupportedMinor);
        assert(!result);
        const auto* oldApi=static_cast<const grab::ApiV1*>(oldGrab->table);
        const auto* newApi=static_cast<const grab::v1_1::Api*>(newGrab->table);
        assert(oldApi->cancelInteractionCommandV1(77,9)==Status::Ok);
        grab::v1_1::HeldPlacementState state;
        assert(newApi->getHeldPlacementState(77,&state)==Status::Ok && state.frameToken==42);
        grab::v1_1::HeldPlacementIntent request;request.frameToken=state.frameToken;
        assert(newApi->submitHeldPlacementIntent(77,&request)==Status::RequestQueued);
        assert(newApi->clearHeldPlacementIntent(77)==Status::Ok);
        assert(oldApi->cancelInteractionCommandV1(77,9)==Status::Ok);
    }
    verify(touch::table(),touch::kSupportedPermissions);
    verify(weapon::tableV1_1(),weapon::kSupportedPermissions);
    verify(weaponparts::tableV1_5(),weaponparts::kSupportedPermissions);
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
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,1,6,sizeof(*pinkyPoints),&result)==Status::UnsupportedMinor);
        assert(ROCKAPI_QueryInterfaceV1(InterfaceId::WeaponParts,2,0,sizeof(*points),&result)==Status::UnsupportedMajor);
        const std::array previous{discovery::RegisteredInterface{{40,InterfaceId::WeaponParts,1,1,sizeof(weaponparts::ApiV1),1,0,0,&api->v1},3}};
        assert(discovery::query(previous,InterfaceId::WeaponParts,1,2,sizeof(*api),&result)==Status::UnsupportedMinor);
        rock::provider::InterfaceBinding oldBinding{},newBinding{};
        assert(rock::provider::bindInterface(oldBinding,1,3,weaponparts::kSupportedPermissions)==Status::Ok);
        assert(rock::provider::bindInterface(newBinding,1,3,weaponparts::kSupportedPermissions)==Status::Ok);
    }
    verify(animation::table(),animation::kSupportedPermissions);
    verify(input::tableV1_4(),input::kSupportedPermissions);
    {
        const InterfaceDescriptorV1* oldInput{};
        const InterfaceDescriptorV1* newInput{};
        assert(ROCKAPI_QueryInterfaceV1(input::kInterfaceId,1,0,sizeof(input::ApiV1),&oldInput)==Status::Ok);
        assert(ROCKAPI_QueryInterfaceV1(input::kInterfaceId,1,1,sizeof(input::v1_1::Api),&newInput)==Status::Ok);
        assert(oldInput->table==newInput->table && oldInput->requiredCoreMinor==0);
        const InterfaceDescriptorV1* physicalInput{};
        assert(ROCKAPI_QueryInterfaceV1(input::kInterfaceId,1,2,sizeof(input::v1_2::Api),&physicalInput)==Status::Ok);
        assert(physicalInput==oldInput && physicalInput->requiredCoreMinor==0);
        const InterfaceDescriptorV1* gestureInput{};
        assert(ROCKAPI_QueryInterfaceV1(input::kInterfaceId,1,3,sizeof(input::v1_3::Api),&gestureInput)==Status::Ok);
        assert(gestureInput==oldInput&&gestureInput->requiredCoreMinor==0);
        const InterfaceDescriptorV1* gameplayInput{};
        assert(ROCKAPI_QueryInterfaceV1(input::kInterfaceId,1,4,sizeof(input::v1_4::Api),&gameplayInput)==Status::Ok);
        assert(gameplayInput==oldInput&&gameplayInput->requiredCoreMinor==0);
        rock::provider::InterfaceBinding oldInputBinding{},newInputBinding{},gameplayBinding{};
        assert(rock::provider::bindInterface(oldInputBinding,1,1,input::kSupportedPermissions)==Status::Ok);
        assert(rock::provider::bindInterface(newInputBinding,1,3,input::kSupportedPermissions)==Status::Ok);
        assert(rock::provider::bindInterface(gameplayBinding,gameplayInput->major,3,input::kSupportedPermissions)==Status::Ok);
        const std::array priorInput{discovery::RegisteredInterface{
            {40,InterfaceId::Input,1,1,sizeof(input::v1_1::Api),1,0,0,&input::tableV1_1()},3}};
        assert(discovery::query(priorInput,InterfaceId::Input,1,2,sizeof(input::v1_2::Api),&result)==Status::UnsupportedMinor);
        const std::array exclusiveInput{discovery::RegisteredInterface{
            {40,InterfaceId::Input,1,2,sizeof(input::v1_2::Api),1,0,0,&input::tableV1_2()},3}};
        assert(discovery::query(exclusiveInput,InterfaceId::Input,1,3,sizeof(input::v1_3::Api),&result)==Status::UnsupportedMinor);
        const std::array oldGesture{discovery::RegisteredInterface{
            {40,InterfaceId::Input,1,3,sizeof(input::v1_3::Api),1,0,0,&input::tableV1_3()},3}};
        assert(discovery::query(oldGesture,InterfaceId::Input,1,4,sizeof(input::v1_4::Api),&result)==Status::UnsupportedMinor);
        const auto* oldCalls=static_cast<const input::ApiV1*>(oldInput->table);
        assert(oldCalls->clearHandInputSuppressionV1(77,Hand::Left)==Status::Ok);
        input::v1_1::PlacementClickState state;
        assert(static_cast<const input::v1_1::Api*>(newInput->table)->getPlacementClickState(77,&state)==Status::Ok);
        assert(state.flags==static_cast<std::uint32_t>(input::v1_1::PlacementClickFlag::InputReserved));
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
