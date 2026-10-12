#include "physics-interaction/weapon/NativeSecondaryModelBranch.h"
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace
{
    std::array<std::uint64_t, 4> arguments{};
    std::uint64_t callbackAlignment = 0;
    bool forceGunRole = false;
    constexpr std::uint64_t kBipedArgument = 0x123456789ABC0000ull;
    constexpr std::uint64_t kRecord = 0x3141592653580000ull;
    constexpr std::uint64_t kSentinel = 0x2122232425262728ull;
    constexpr std::array<std::uint64_t, 7> kVolatile{
        0xAABBCCDDEEFF0000ull, 0x1111222233334444ull, 0x2222333344445555ull,
        0x3333444455556666ull, 0x4444555566667777ull, 0x5555666677778888ull, 0x6666777788889999ull};
    constexpr std::array<std::array<std::uint64_t, 2>, 6> kXmm{{
        {1,2},{3,4},{5,6},{7,8},{9,10},{11,12}
    }};
    struct Observation
    {
        std::array<std::uint64_t, 7> registers{};
        std::array<std::array<std::uint64_t, 2>, 6> xmm{};
        std::uint64_t slot = 0, record = 0, alignment = 0, route = 0, localIntact = 0;
    };
    struct Input { std::int32_t slot; bool firstPerson; };

    class Resolver final : public Xbyak::CodeGenerator
    {
    public:
        Resolver() : CodeGenerator(512)
        {
            mov(r10, reinterpret_cast<std::uintptr_t>(arguments.data()));
            mov(ptr[r10], rcx); mov(ptr[r10 + 8], rdx);
            mov(ptr[r10 + 16], r8); mov(ptr[r10 + 24], r9);
            mov(rax, rsp); and_(eax, 15);
            mov(r10, reinterpret_cast<std::uintptr_t>(&callbackAlignment)); mov(ptr[r10], rax);
            // A valid Win64 callee may overwrite every volatile GPR/XMM and
            // all four home slots. None of those changes may leak into native.
            for (int i = 8; i <= 32; i += 8) mov(qword[rsp + i], 0xBAD);
            for (auto reg : {rcx, rdx, r8, r9, r10, r11}) mov(reg, 0xBADBADBADBADBADBull);
            for (int i = 0; i < 6; ++i) pcmpeqd(Xbyak::Xmm(i), Xbyak::Xmm(i));
            mov(rax, reinterpret_cast<std::uintptr_t>(&forceGunRole)); movzx(eax, byte[rax]);
            ret(); ready();
        }
    };
    class Continuations final : public Xbyak::CodeGenerator
    {
    public:
        const void *forced = nullptr, *skip = nullptr, *compare = nullptr;
        Continuations() : CodeGenerator(2048)
        {
            Xbyak::Label record, less;
            forced = getCurr(); mov(qword[r15 + offsetof(Observation, route)], 1); jmp(record, T_NEAR);
            skip = getCurr(); mov(qword[r15 + offsetof(Observation, route)], 2); jmp(record, T_NEAR);
            compare = getCurr();
            jl(less); // The original native JL immediately following each patch.
            mov(qword[r15 + offsetof(Observation, route)], 3); jmp(record, T_NEAR);
            L(less); mov(qword[r15 + offsetof(Observation, route)], 4);
            L(record);
            int offset = 0;
            for (auto reg : {rax, rcx, rdx, r8, r9, r10, r11}) {
                mov(ptr[r15 + offset], reg); offset += 8;
            }
            for (int i = 0; i < 6; ++i)
                movdqu(ptr[r15 + offsetof(Observation, xmm) + i * 16], Xbyak::Xmm(i));
            mov(ptr[r15 + offsetof(Observation, slot)], r12);
            mov(ptr[r15 + offsetof(Observation, record)], r13);
            mov(rax, rsp); and_(eax, 15); mov(ptr[r15 + offsetof(Observation, alignment)], rax);
            mov(rcx, kSentinel); mov(qword[r15 + offsetof(Observation, localIntact)], 1);
            Xbyak::Label damaged, finish;
            for (int i = 0x20; i < 0xA0; i += 8) { cmp(ptr[rsp + i], rcx); jne(damaged); }
            jmp(finish);
            L(damaged); mov(qword[r15 + offsetof(Observation, localIntact)], 0);
            L(finish);
            add(rsp, 0x208);
            pop(r15); pop(r14); pop(r13); pop(r12); pop(rdi); pop(rsi); pop(rbx); pop(rbp);
            ret(); ready();
        }
    };
    class Caller final : public Xbyak::CodeGenerator
    {
    public:
        explicit Caller(const void* code) : CodeGenerator(1024)
        {
            push(rbp); push(rbx); push(rsi); push(rdi); push(r12); push(r13); push(r14); push(r15);
            sub(rsp, 0x208);
            mov(r14, rcx); mov(r15, rdx);
            // Reproduce 1C9F70's RBP-relative incoming biped argument and the
            // real slot/record nonvolatile registers from the captured slot29.
            lea(rbp, ptr[rsp + 0x100]);
            mov(rax, kBipedArgument); mov(ptr[rbp + 0x98], rax);
            mov(r13, kRecord); movsxd(r12, dword[r14 + offsetof(Input, slot)]);
            mov(rax, kSentinel);
            for (int i = 0x20; i < 0xA0; i += 8) mov(ptr[rsp + i], rax);
            mov(rax, reinterpret_cast<std::uintptr_t>(kXmm.data()));
            for (int i = 0; i < 6; ++i) movdqu(Xbyak::Xmm(i), ptr[rax + i * 16]);
            int index = 0;
            for (auto reg : {rax, rcx, rdx, r8, r9, r10, r11}) mov(reg, kVolatile[index++]);
            mov(al, byte[r14 + offsetof(Input, firstPerson)]);
            Xbyak::Label target;
            jmp(ptr[rip + target]); L(target); dq(reinterpret_cast<std::uintptr_t>(code));
            ready();
        }
    };
}

int main()
{
    Resolver resolver;
    Continuations tails;
    rock::native_secondary_model::ModelBranch stub(reinterpret_cast<std::uintptr_t>(resolver.getCode()),
        reinterpret_cast<std::uintptr_t>(tails.forced), reinterpret_cast<std::uintptr_t>(tails.skip),
        reinterpret_cast<std::uintptr_t>(tails.compare));
    Caller caller(stub.getCode());
    for (bool forced : {false, true}) for (bool firstPerson : {false, true})
        for (std::int32_t slot : {-1, 0, 29, 31, 32, 39, 40, 41, 43}) {
            forceGunRole = forced;
            const Input input{slot, firstPerson};
            Observation observed{};
            caller.getCode<void (*)(const Input*, Observation*)>()(&input, &observed);
            auto expectedRegisters = kVolatile;
            expectedRegisters[0] |= firstPerson ? 1u : 0u;
            assert(observed.registers == expectedRegisters && observed.xmm == kXmm);
            assert(observed.slot == static_cast<std::uint64_t>(static_cast<std::int64_t>(slot)) && observed.record == kRecord);
            assert(observed.alignment == 0 && callbackAlignment == 8 && observed.localIntact == 1);
            assert(arguments[0] == kBipedArgument && arguments[1] == static_cast<std::uint32_t>(slot) &&
                arguments[2] == kRecord && arguments[3] == static_cast<std::uint64_t>(firstPerson));
            assert(observed.route == (forced ? 1u : !firstPerson ? 2u : slot < 32 ? 4u : 3u));
        }
}
