#include <ROCK/Discovery.h>
#include "ModularTestTables.h"
#include "api/AuthoredInteractionLease.h"
#include "physics-interaction/grab/DefaultGrabPoseRegistry.h"
namespace rock::api::core { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::hands { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::collision { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::storage { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::grab {
namespace {
saved_grab_offset::DefaultGrabPoseRegistry poses;
Status ROCK_CALL setPoses(OwnerToken owner,const DefaultGrabPoses* request) noexcept {return request ? poses.set(owner,*request) : Status::InvalidArgument;}
Status ROCK_CALL clearPoses(OwnerToken owner,std::uint32_t form) noexcept {return poses.clear(owner,form);}
}
const Api& table() noexcept {static const Api value=[] {Api api{};api.setDefaultGrabPoses=&setPoses;api.clearDefaultGrabPoses=&clearPoses;return api;}();return value;}
}
namespace rock::api::touch {
namespace {
Status ROCK_CALL oldYield(OwnerToken owner,std::uint64_t scope,std::uint64_t target,std::uint32_t generation) noexcept {
    return owner==77&&scope==5&&target==6&&generation==7?Status::Ok:Status::InvalidArgument;
}
authored_interaction::Lease authored;
Sample sample{};
Status ROCK_CALL setAuthored(OwnerToken owner,const AuthoredHand* request) noexcept {return request?authored.set(owner,*request,sample):Status::InvalidArgument;}
Status ROCK_CALL clearAuthored(OwnerToken owner,Hand) noexcept {return authored.clear(owner,sample);}
}
const Api& table() noexcept {
    static const Api value=[] {Api v{};v.requestTouchGrabYield=&oldYield;v.setAuthoredHand=&setAuthored;v.clearAuthoredHand=&clearAuthored;return v;}();
    return value;
}
}
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
const v1_4::Api& tableV1_4() noexcept {static const v1_4::Api value{tableV1_3(),&points};return value;}
const v1_5::Api& tableV1_5() noexcept {static const v1_5::Api value{tableV1_4(),&points};return value;}
const v1_6::Api& tableV1_6() noexcept {static const v1_6::Api value{tableV1_5(),&points};return value;}
Status ROCK_CALL itemState(OwnerToken owner,const v1_7::ItemKey* key,const v1_7::ItemState* state) noexcept {return owner==80&&key&&state?Status::Ok:Status::InvalidArgument;}
Status ROCK_CALL itemMove(OwnerToken owner,const v1_7::ItemKey* before,const v1_7::ItemKey* after) noexcept {return owner==80&&before&&after?Status::Ok:Status::InvalidArgument;}
const v1_7::Api& tableV1_7() noexcept {static const v1_7::Api value{tableV1_6(),&itemState,&itemMove,&oldClear};return value;}
}
namespace rock::api::animation { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::input { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::playercontroller { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::configuration { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::native_interaction_service { const api::native_interaction::ApiV1& table() noexcept { static const api::native_interaction::ApiV1 value{};return value; } const api::native_interaction::v1_1::Api& tableV1_1() noexcept { static const api::native_interaction::v1_1::Api value{table()};return value; } }
