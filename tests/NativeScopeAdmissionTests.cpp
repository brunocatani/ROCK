#include "physics-interaction/weapon/scope/NativeScopeAdmissionStub.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace
{
    using Site = rock::native_scope_admission::Site;
    struct Observation
    {
        std::array<std::uint64_t, 8> registers{};
        std::array<std::array<std::uint64_t, 2>, 6> xmm{};
        std::uint64_t stackAlignment{};
    };
    std::array<std::uint64_t, 4> arguments{};
    std::uint32_t callbackAlignment = 0;
    constexpr std::uintptr_t weapon = 0x12345678;
    constexpr std::uintptr_t instance = 0x76543210;
    constexpr std::array<std::uint64_t, 8> sentinels{
        0xAA11AA11AA11AA11, 0xBB22BB22BB22BB22, weapon, 0xDD44DD44DD44DD44,
        0x8811881188118811, 0x9922992299229922, 0xAA33AA33AA33AA33, 0xBB44BB44BB44BB44
    };
    constexpr std::array<std::array<std::uint64_t, 2>, 6> vectors{{
        { 0x1020304050607080, 0x9080706050403020 }, { 11, 22 }, { 33, 44 },
        { 55, 66 }, { 77, 88 }, { 99, 111 }
    }};

    void capture(std::uint32_t native, const void* form, const void* data, Site site)
    {
        arguments = { native, reinterpret_cast<std::uintptr_t>(form),
            reinterpret_cast<std::uintptr_t>(data), static_cast<std::uint32_t>(site) };
    }

    class Predicate final : public Xbyak::CodeGenerator
    {
    public:
        explicit Predicate(bool result) : CodeGenerator(512)
        {
            mov(rax, rsp);
            and_(eax, 15);
            mov(r10, reinterpret_cast<std::uintptr_t>(&callbackAlignment));
            mov(dword[r10], eax);
            sub(rsp, 0x28);
            mov(rax, reinterpret_cast<std::uintptr_t>(&capture));
            call(rax);
            add(rsp, 0x28);
            // Exercise everything a conforming C++ callback may clobber.
            for (auto reg : { rcx, rdx, r8, r9, r10, r11 }) {
                mov(reg, 0xBAD0BAD0BAD0BAD0ull);
            }
            for (int i = 0; i < 6; ++i) {
                pcmpeqd(Xbyak::Xmm(i), Xbyak::Xmm(i));
            }
            mov(eax, result ? 1 : 0);
            ret();
            ready();
        }
    };

    class Resume final : public Xbyak::CodeGenerator
    {
    public:
        Resume() : CodeGenerator(512)
        {
            const std::array<Xbyak::Reg64, 8> regs{ rax, rbx, rcx, rdx, r8, r9, r10, r11 };
            for (std::size_t i = 0; i < regs.size(); ++i) {
                mov(ptr[r12 + offsetof(Observation, registers) + i * 8], regs[i]);
            }
            for (int i = 0; i < 6; ++i) {
                movdqu(ptr[r12 + offsetof(Observation, xmm) + i * 16], Xbyak::Xmm(i));
            }
            mov(rax, rsp);
            and_(eax, 15);
            mov(ptr[r12 + offsetof(Observation, stackAlignment)], rax);
            add(rsp, 0x58);
            pop(r12);
            pop(rbx);
            ret();
            ready();
        }
    };

    class NativeCaller final : public Xbyak::CodeGenerator
    {
    public:
        NativeCaller(Site site, std::uint32_t flags, const void* stub) : CodeGenerator(512)
        {
            push(rbx);
            push(r12);
            sub(rsp, 0x58);
            mov(r12, rcx); // fixture output; not a volatile native register
            mov(qword[rsp + 0x28], instance);
            mov(qword[rsp + 0x38], instance);
            mov(rax, reinterpret_cast<std::uintptr_t>(vectors.data()));
            for (int i = 0; i < 6; ++i) {
                movdqu(Xbyak::Xmm(i), ptr[rax + i * 16]);
            }
            const std::array<Xbyak::Reg64, 8> regs{ rax, rbx, rcx, rdx, r8, r9, r10, r11 };
            for (std::size_t i = 0; i < regs.size(); ++i) {
                mov(regs[i], sentinels[i]);
            }
            if (site == Site::Geometry) {
                mov(ebx, flags);
            } else {
                mov(eax, flags);
            }
            Xbyak::Label destination;
            jmp(ptr[rip + destination]);
            L(destination);
            dq(reinterpret_cast<std::uintptr_t>(stub));
            ready();
        }
    };
}

int main()
{
    bool ok = true;
    for (Site site : { Site::Geometry, Site::Menu }) {
        for (std::uint32_t flags : { 0u, 0x00200000u, 0x80000000u, 0x80200000u }) {
            for (bool admit : { false, true }) {
                Resume resume;
                Predicate predicate(admit);
                rock::native_scope_admission::Stub stub(site,
                    reinterpret_cast<std::uintptr_t>(predicate.getCode()), reinterpret_cast<std::uintptr_t>(resume.getCode()));
                NativeCaller caller(site, flags, stub.getCode());
                Observation observed{};
                caller.getCode<void (*)(Observation*)>()(&observed);
                auto expected = sentinels;
                expected[site == Site::Geometry ? 1 : 0] = ((flags >> 21) & 0xFFFFFF01u) | (admit ? 1u : 0u);
                const std::array<std::uint64_t, 4> expectedArguments{
                    (flags >> 21) & 1u, weapon, instance, static_cast<std::uint32_t>(site)
                };
                const bool pass = observed.registers == expected && observed.xmm == vectors &&
                    observed.stackAlignment == 0 && callbackAlignment == 8 && arguments == expectedArguments;
                if (!pass) {
                    std::printf("Scope admission ABI failed site=%u flags=%08X admit=%u registers=%u xmm=%u stack=%llu callback=%u arguments=%u\n",
                        static_cast<unsigned>(site), flags, admit, observed.registers == expected, observed.xmm == vectors,
                        observed.stackAlignment, callbackAlignment, arguments == expectedArguments);
                }
                ok &= pass;
            }
        }
    }
    return ok ? 0 : 1;
}
