#pragma once
#include <ROCK/EquipPickup.h>

namespace rock::equip_pickup
{
    struct Intent { std::uint32_t reference{},count{}; };
    inline thread_local Intent current;
    class Scope {
        Intent previous;
    public:
        Scope(std::uint32_t reference,std::uint32_t count) noexcept : previous(current) { current={reference,count}; }
        ~Scope() { current=previous; }
        Scope(const Scope&)=delete;
        Scope& operator=(const Scope&)=delete;
    };
    inline std::uint32_t ROCK_CALL matches(std::uint32_t reference,std::uint32_t count) noexcept
    { return reference && count && current.reference==reference && current.count==count; }
    inline const api::equip_pickup::Api table{sizeof(api::equip_pickup::Api),1,matches};
    inline const api::equip_pickup::Api* query(std::uint32_t major,std::uint32_t minor,std::uint32_t bytes) noexcept
    { return major==1 && minor==0 && bytes==sizeof(table)?&table:nullptr; }
}
