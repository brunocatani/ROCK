#include <ROCK/Discovery.h>
#include "ModularTestTables.h"
namespace rock::api::core { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::hands { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::collision { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::grab { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
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
namespace rock::api::input { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::references { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::playercontroller { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::diagnostics { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
namespace rock::api::configuration { const ApiV1& table() noexcept { static const ApiV1 value{}; return value; } }
