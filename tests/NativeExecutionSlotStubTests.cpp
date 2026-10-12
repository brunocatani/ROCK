#include "physics-interaction/weapon/NativeExecutionSlotStub.h"
#include "physics-interaction/weapon/NativeInventorySlotStub.h"
#include <array>
#include <cassert>
#include <cstdint>

namespace
{
    std::array<std::uintptr_t, 5> arguments{};
    std::uintptr_t callbackAlignment = 0;
    constexpr std::array<std::uintptr_t, 5> expected{0x11112222,0x33334444,0x55556666,0x77778888,0x9999AAAA};
    constexpr std::uintptr_t selected = 0xABCDABCD;
    struct Observation
    {
        std::uintptr_t result, instance, component, alignment;
    };

    class Resolver final : public Xbyak::CodeGenerator
    {
    public:
        Resolver() : CodeGenerator(256)
        {
            mov(r10, reinterpret_cast<std::uintptr_t>(arguments.data()));
            mov(ptr[r10], rcx);
            mov(ptr[r10 + 8], rdx);
            mov(ptr[r10 + 16], r8);
            mov(ptr[r10 + 24], r9);
            mov(rax, ptr[rsp + 0x28]);
            mov(ptr[r10 + 32], rax);
            mov(rax, rsp);
            and_(eax, 15);
            mov(r10, reinterpret_cast<std::uintptr_t>(&callbackAlignment));
            mov(ptr[r10], rax);
            // Exercise permitted getter clobbers. The stub may not depend on
            // any volatile value surviving the resolver call.
            for (auto reg : {rcx, rdx, r8, r9, r10, r11}) mov(reg, 0xBAD0BAD0BAD0BAD0ull);
            for (int i = 0; i < 6; ++i) pcmpeqd(Xbyak::Xmm(i), Xbyak::Xmm(i));
            mov(rax, selected);
            ret();
            ready();
        }
    };
    class Resume final : public Xbyak::CodeGenerator
    {
    public:
        Resume() : CodeGenerator(256)
        {
            mov(ptr[r12], rax);
            mov(ptr[r12 + 8], rbx);
            mov(ptr[r12 + 16], r14);
            mov(rax, rsp);
            and_(eax, 15);
            mov(ptr[r12 + 24], rax);
            add(rsp, 0x108);
            pop(r15); pop(r14); pop(r13); pop(r12); pop(rbx); pop(rbp);
            ret();
            ready();
        }
    };
    class Caller final : public Xbyak::CodeGenerator
    {
    public:
        explicit Caller(const void* stub) : CodeGenerator(256)
        {
            push(rbp); push(rbx); push(r12); push(r13); push(r14); push(r15);
            sub(rsp, 0x108);
            mov(r12, rcx);
            lea(rbp, ptr[rsp + 0x80]);
            mov(rax, expected[2]); mov(ptr[rbp + 0x6F], rax);
            mov(rax, expected[3]); mov(ptr[rbp + 0x77], rax);
            lea(r13, ptr[rsp + 0x40]);
            mov(rax, expected[4]); mov(ptr[r13 + 8], rax);
            mov(r14, expected[0]); mov(rbx, expected[1]);
            Xbyak::Label target;
            jmp(ptr[rip + target]);
            L(target); dq(reinterpret_cast<std::uintptr_t>(stub));
            ready();
        }
    };
}

int main()
{
    Resolver resolver;
    Resume resume;
    rock::native_execution_slot::Stub stub(reinterpret_cast<std::uintptr_t>(resolver.getCode()),
        reinterpret_cast<std::uintptr_t>(resume.getCode()));
    Caller caller(stub.getCode());
    Observation observed{};
    caller.getCode<void (*)(Observation*)>()(&observed);
    assert(arguments == expected);
    assert(callbackAlignment == 8 && observed.alignment == 0);
    assert(observed.result == selected);
    assert(observed.instance == expected[1] && observed.component == expected[0]);

    // E202ED retains its owner callback in RBX, with an already aligned
    // native call frame. Verify the actual generated tail-call ABI, including
    // volatile clobbers and the surviving nonvolatile owner register.
    class OrdinalResolver final : public Xbyak::CodeGenerator
    {
    public:
        OrdinalResolver():CodeGenerator(256)
        {
            mov(r10,reinterpret_cast<std::uintptr_t>(arguments.data()));
            mov(ptr[r10],rcx); mov(ptr[r10+8],rdx); mov(ptr[r10+16],r8);
            mov(rax,rsp); and_(eax,15); mov(ptr[r10+24],rax);
            for (auto reg:{rcx,rdx,r8,r9,r10,r11}) mov(reg,0xBAD0BAD0BAD0BAD0ull);
            mov(rax,selected); ret(); ready();
        }
    } ordinalResolver;
    rock::native_inventory_hand_slots::OrdinalStub ordinalStub(reinterpret_cast<std::uintptr_t>(ordinalResolver.getCode()));
    class OrdinalCaller final : public Xbyak::CodeGenerator
    {
    public:
        explicit OrdinalCaller(const void* target):CodeGenerator(256)
        {
            push(rbx); push(rsi); sub(rsp,0x28);
            mov(rsi,rcx); mov(rcx,expected[0]); mov(rdx,expected[1]); mov(rbx,expected[2]);
            mov(rax,reinterpret_cast<std::uintptr_t>(target)); call(rax);
            mov(ptr[rsi],rax); mov(ptr[rsi+8],rbx);
            add(rsp,0x28); pop(rsi); pop(rbx); ret(); ready();
        }
    } ordinalCaller(ordinalStub.getCode());
    arguments={}; observed={};
    ordinalCaller.getCode<void(*)(Observation*)>()(&observed);
    assert(arguments[0]==expected[0] && arguments[1]==expected[1] && arguments[2]==expected[2]);
    assert(arguments[3]==8 && observed.result==selected && observed.instance==expected[2]);
}
