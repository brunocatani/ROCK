#pragma once

#include <cstdint>
#include <xbyak/xbyak.h>

namespace rock::native_inventory_hand_slots
{
    // E202ED is a complete native CALL with RCX=item, EDX=worn bit ordinal,
    // RBX=the native callback retaining the actor. Tail-call the replacement
    // with that owner as argument three; reuse the caller's shadow space and
    // return address. No nonvolatile register or stack adjustment is needed.
    class OrdinalStub final : public Xbyak::CodeGenerator
    {
    public:
        explicit OrdinalStub(std::uintptr_t resolver):CodeGenerator(64)
        {
            mov(r8,rbx);
            mov(rax,resolver);
            jmp(rax);
            ready();
        }
    };
}
