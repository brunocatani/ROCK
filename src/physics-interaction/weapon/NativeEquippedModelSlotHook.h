#pragma once
#include <cstdint>
#include <xbyak/xbyak.h>

namespace rock::native_equipped_model_slot
{
    enum class Arguments { EquipmentIndex, Attach, Rebuild, Classification, BodyFilter, Cleanup };

    class Stub final : public Xbyak::CodeGenerator
    {
    public:
        Stub(Arguments arguments, std::uintptr_t resolve, std::uintptr_t compare, std::uintptr_t cleanup) : CodeGenerator(256)
        {
            // These replace calls to known leaf functions. Preserve the
            // caller's volatile state as well as the ordinary Windows ABI.
            pushfq();
            push(rcx); push(rdx); push(r8); push(r9); push(r10); push(r11);
            sub(rsp, 0x80); // aligned call, shadow space and XMM0..5
            for (int i = 0; i < 6; ++i) movdqu(ptr[rsp + 0x20 + 16 * i], Xbyak::Xmm(i));
            auto target = compare;
            switch (arguments) {
            case Arguments::EquipmentIndex: target = resolve; break;
            case Arguments::Attach: mov(edx, esi); break;
            case Arguments::Rebuild: mov(edx, r15d); break;
            case Arguments::Classification: mov(edx, ebx); break;
            case Arguments::BodyFilter: mov(edx, edi); break;
            case Arguments::Cleanup:
                mov(rdx, rsi);
                mov(r8, ptr[rbp + 0x67]);
                target = cleanup;
                break;
            }
            mov(rax, target); call(rax);
            for (int i = 0; i < 6; ++i) movdqu(Xbyak::Xmm(i), ptr[rsp + 0x20 + 16 * i]);
            add(rsp, 0x80);
            pop(r11); pop(r10); pop(r9); pop(r8); pop(rdx); pop(rcx);
            popfq(); ret(); ready();
        }
    };
}
