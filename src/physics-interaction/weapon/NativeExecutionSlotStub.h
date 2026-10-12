#pragma once

#include <cstdint>
#include <xbyak/xbyak.h>

namespace rock::native_execution_slot
{
    // E72E19 replaces a complete virtual getter call. Native RSP is already
    // aligned and has outgoing shadow/argument space; its first local is +30.
    // The original call may clobber all volatile registers/XMM0-5. R14 is the
    // equip component, RBX the instance; RBP+6F/+77 retain actor/item arguments
    // and R13+8 is the requested slot. RAX must survive into E72E25.
    class Stub final : public Xbyak::CodeGenerator
    {
    public:
        Stub(std::uintptr_t resolver, std::uintptr_t continuation) : CodeGenerator(256)
        {
            mov(rcx, r14);
            mov(rdx, rbx);
            mov(r8, ptr[rbp + 0x6F]);
            mov(r9, ptr[rbp + 0x77]);
            mov(rax, ptr[r13 + 8]);
            mov(ptr[rsp + 0x20], rax);
            mov(rax, resolver);
            call(rax);
            Xbyak::Label resume;
            jmp(ptr[rip + resume]);
            L(resume);
            dq(continuation);
            ready();
        }
    };
}
