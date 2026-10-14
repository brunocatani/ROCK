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
namespace rock::api::weapon { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::weaponparts { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::animation { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::input { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::playercontroller { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::api::configuration { const Api& table() noexcept { static const Api value{}; return value; } }
namespace rock::native_interaction_service { const api::native_interaction::Api& table() noexcept { static const api::native_interaction::Api value{};return value; } }
