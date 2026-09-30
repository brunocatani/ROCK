#include <ROCK/Discovery.h>
#include "ModularTestTables.h"
#include "physics-interaction/grab/DefaultGrabPoseRegistry.h"
namespace rock::api::core { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::hands { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::collision { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::grab {
namespace {
saved_grab_offset::DefaultGrabPoseRegistry poses;
Status ROCK_CALL setPoses(OwnerToken owner,const v1_2::DefaultGrabPoses* request) noexcept {return request ? poses.set(owner,*request) : Status::InvalidArgument;}
Status ROCK_CALL clearPoses(OwnerToken owner,std::uint32_t form) noexcept {return poses.clear(owner,form);}
Status ROCK_CALL oldCancel(OwnerToken owner,std::uint64_t id) noexcept {return owner==77 && id==9 ? Status::Ok : Status::InvalidArgument;}
Status ROCK_CALL placementRead(OwnerToken owner,v1_1::HeldPlacementState* out) noexcept {if(owner!=77 || !out)return Status::InvalidArgument;*out={};out->frameToken=42;return Status::Ok;}
Status ROCK_CALL placementSubmit(OwnerToken owner,const v1_1::HeldPlacementIntent* request) noexcept {return owner==77 && request && request->frameToken==42 ? Status::RequestQueued : Status::InvalidArgument;}
Status ROCK_CALL placementClear(OwnerToken owner) noexcept {return owner==77 ? Status::Ok : Status::InvalidArgument;}
}
const v1_1::Api& tableV1_1() noexcept {static const v1_1::Api value{[] {ApiV1 old{};old.cancelInteractionCommandV1=&oldCancel;return old;}(),&placementRead,&placementSubmit,&placementClear};return value;}
const v1_2::Api& tableV1_2() noexcept {static const v1_2::Api value{tableV1_1(),&setPoses,&clearPoses};return value;}
const v1_3::Api& tableV1_3() noexcept {static const v1_3::Api value{tableV1_2()};return value;}
const ApiV1& table() noexcept {return tableV1_2().v1_1.v1;}
}
namespace rock::api::touch { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::weapon {
namespace {
Status ROCK_CALL oldPrimary(OwnerToken owner, Hand* hand) noexcept {
    if (owner != 77 || !hand) return Status::InvalidArgument;
    *hand = Hand::Left;
    return Status::Ok;
}
Status ROCK_CALL newEquip(OwnerToken owner, const v1_1::EquipRequest* request, std::uint64_t* command) noexcept {
    if (owner != 77 || !request || !command) return Status::InvalidArgument;
    *command = request->hand == Hand::Left ? 101 : 102;
    return Status::RequestQueued;
}
}
const v1_1::Api& tableV1_1() noexcept {
    static const v1_1::Api value{{&oldPrimary}, nullptr, &newEquip, nullptr, nullptr};
    return value;
}
const ApiV1& table() noexcept { return tableV1_1().v1; }
}
namespace rock::api::weaponparts {
namespace {
Status ROCK_CALL oldClear(OwnerToken owner) noexcept {return owner==77?Status::Ok:Status::OwnerNotRegistered;}
Status ROCK_CALL suppress(OwnerToken owner,const v1_2::CollisionSuppression* request) noexcept {return owner==78&&request?Status::Ok:Status::InvalidArgument;}
Status ROCK_CALL clearSuppression(OwnerToken owner) noexcept {return owner==78?Status::Ok:Status::OwnerNotRegistered;}
}
const ApiV1& table() noexcept { static const ApiV1 value=[] {ApiV1 api{};api.clearWeaponPartTargetsV1=&oldClear;return api;}(); return value; }
const v1_2::Api& tableV1_2() noexcept {static const v1_2::Api value{table(),&suppress,&clearSuppression};return value;}
Status ROCK_CALL points(OwnerToken owner,const v1_3::GrabPoint* p,std::uint32_t n) noexcept {return owner==79&&p&&n?Status::Ok:Status::InvalidArgument;}
const v1_3::Api& tableV1_3() noexcept {static const v1_3::Api value{tableV1_2(),&points,&oldClear};return value;}
}
namespace rock::api::animation { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::input {
namespace {
Status ROCK_CALL oldClear(OwnerToken owner,Hand hand) noexcept { return owner==77 && hand==Hand::Left ? Status::Ok : Status::InvalidArgument; }
Status ROCK_CALL placement(OwnerToken owner,v1_1::PlacementClickState* out) noexcept {
    if(owner!=77 || !out) return Status::InvalidArgument;
    *out={};out->flags=static_cast<std::uint32_t>(v1_1::PlacementClickFlag::InputReserved);return Status::Ok;
}
}
const v1_1::Api& tableV1_1() noexcept { static const v1_1::Api value{{nullptr,&oldClear},&placement}; return value; }
const v1_2::Api& tableV1_2() noexcept { static const v1_2::Api value{tableV1_1()}; return value; }
const v1_3::Api& tableV1_3() noexcept { static const v1_3::Api value{tableV1_2()}; return value; }
const ApiV1& table() noexcept { return tableV1_1().v1; }
}
namespace rock::api::references { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::playercontroller { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::diagnostics { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::configuration { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
