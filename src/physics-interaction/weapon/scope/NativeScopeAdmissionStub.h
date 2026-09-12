#pragma once

#include <cstdint>
#include <xbyak/xbyak.h>

namespace rock::native_scope_admission
{
    enum class Site : std::uint32_t { Geometry, Menu };

    // Both patched instructions are inside native functions with RSP aligned
    // to 16 bytes. Their next instruction tests BL/AL, so incoming arithmetic
    // flags are dead. All other volatile registers remain live.
    class Stub final : public Xbyak::CodeGenerator
    {
    public:
        Stub(Site site, std::uintptr_t predicate, std::uintptr_t continuation) : CodeGenerator(512)
        {
            push(rax);
            push(rcx);
            push(rdx);
            push(r8);
            push(r9);
            push(r10);
            push(r11);
            sub(rsp, 0x88); // shadow space + six XMM registers + alignment
            for (int i = 0; i < 6; ++i) {
                movdqu(ptr[rsp + 0x20 + i * 16], Xbyak::Xmm(i));
            }

            mov(ecx, site == Site::Geometry ? ebx : eax);
            shr(ecx, 0x15);
            and_(ecx, 1);
            mov(rdx, ptr[rsp + 0xB0]); // original RCX: current native weapon
            // Native GetEquippedWeapon returned a held instance in this
            // caller's stack. Addresses never outlive this predicate call.
            mov(r8, ptr[rsp + 0xC0 + (site == Site::Geometry ? 0x38 : 0x28)]);
            mov(r9d, static_cast<std::uint32_t>(site));
            mov(rax, predicate);
            call(rax);

            if (site == Site::Geometry) {
                shr(ebx, 0x15);
                and_(bl, 1);
                or_(bl, al);
            } else {
                mov(r11d, dword[rsp + 0xB8]); // original EAX: native flags
                shr(r11d, 0x15);
                and_(r11b, 1);
                or_(r11b, al);
                mov(ptr[rsp + 0xB8], r11);
            }
            for (int i = 0; i < 6; ++i) {
                movdqu(Xbyak::Xmm(i), ptr[rsp + 0x20 + i * 16]);
            }
            add(rsp, 0x88);
            pop(r11);
            pop(r10);
            pop(r9);
            pop(r8);
            pop(rdx);
            pop(rcx);
            pop(rax);
            Xbyak::Label resume;
            jmp(ptr[rip + resume]);
            L(resume);
            dq(continuation);
            ready();
        }
    };
}
