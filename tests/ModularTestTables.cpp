#include <ROCK/Discovery.h>
#include "ModularTestTables.h"
namespace rock::api::core { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::hands { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::collision { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::grab {
namespace {
Status ROCK_CALL oldCancel(OwnerToken owner,std::uint64_t id) noexcept {return owner==77 && id==9 ? Status::Ok : Status::InvalidArgument;}
Status ROCK_CALL placementRead(OwnerToken owner,v1_1::HeldPlacementState* out) noexcept {if(owner!=77 || !out)return Status::InvalidArgument;*out={};out->frameToken=42;return Status::Ok;}
Status ROCK_CALL placementSubmit(OwnerToken owner,const v1_1::HeldPlacementIntent* request) noexcept {return owner==77 && request && request->frameToken==42 ? Status::RequestQueued : Status::InvalidArgument;}
Status ROCK_CALL placementClear(OwnerToken owner) noexcept {return owner==77 ? Status::Ok : Status::InvalidArgument;}
}
const v1_1::Api& tableV1_1() noexcept {static const v1_1::Api value{[] {ApiV1 old{};old.cancelInteractionCommandV1=&oldCancel;return old;}(),&placementRead,&placementSubmit,&placementClear};return value;}
const ApiV1& table() noexcept {return tableV1_1().v1;}
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
namespace rock::api::weaponparts { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
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
const ApiV1& table() noexcept { return tableV1_1().v1; }
}
namespace rock::api::references { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::playercontroller { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::diagnostics { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::configuration { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
