#pragma once

#include <cstdint>
#include <xbyak/xbyak.h>

namespace rock::native_secondary_model
{
    // Three 1C9F70 classification sites have the same displaced instructions:
    // TEST AL,AL; JZ skip; CMP R12D,20h. The real biped index in R12 and the
    // object record in R13 must NEVER change. Only the gun-role branch changes.
    class ModelBranch final : public Xbyak::CodeGenerator
    {
    public:
        ModelBranch(std::uintptr_t resolver, std::uintptr_t gunRole,
            std::uintptr_t nativeSkip, std::uintptr_t nativeCompare) : CodeGenerator(512)
        {
            // Native RSP is 16-byte aligned. Preserve all volatile values:
            // this inserts a call where the original code only tested flags.
            sub(rsp, 0xC0);
            int offset = 0x20;
            for (auto reg : {rax, rcx, rdx, r8, r9, r10, r11}) {
                mov(ptr[rsp + offset], reg);
                offset += 8;
            }
            for (int i = 0; i < 6; ++i) movdqu(ptr[rsp + 0x60 + i * 16], Xbyak::Xmm(i));
            mov(rcx, ptr[rbp + 0x98]); // pointer to the retained biped argument
            mov(edx, r12d);
            mov(r8, r13);             // exact native BIPOBJECT record
            movzx(r9d, al);           // actual IsFirstPersonBiped result
            mov(rax, resolver);
            call(rax);
            test(al, al);
            Xbyak::Label ordinary, skip, forcedTarget, skipTarget, compareTarget;
            jz(ordinary, T_NEAR);
            restore();
            jmp(ptr[rip + forcedTarget]);
            L(ordinary);
            restore();
            test(al, al);
            jz(skip);
            cmp(r12d, 0x20);
            jmp(ptr[rip + compareTarget]); // original following JL consumes flags
            L(skip);
            jmp(ptr[rip + skipTarget]);
            L(forcedTarget); dq(gunRole);
            L(skipTarget); dq(nativeSkip);
            L(compareTarget); dq(nativeCompare);
            ready();
        }
    private:
        void restore()
        {
            for (int i = 0; i < 6; ++i) movdqu(Xbyak::Xmm(i), ptr[rsp + 0x60 + i * 16]);
            int offset = 0x20;
            for (auto reg : {rax, rcx, rdx, r8, r9, r10, r11}) {
                mov(reg, ptr[rsp + offset]);
                offset += 8;
            }
            add(rsp, 0xC0);
        }
    };
}
