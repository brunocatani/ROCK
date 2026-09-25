#include "physics-interaction/weapon/NativeEquippedModelSlotHook.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace
{
    using rock::native_equipped_model_slot::Arguments;
    struct Observation {
        std::array<std::uint64_t, 7> registers{};
        std::array<std::array<std::uint64_t, 2>, 6> xmm{};
        std::uint64_t flags{};
    };
    std::array<std::uint64_t, 3> received{};
    std::uint64_t callbackAlignment{};
    constexpr std::array<std::uint64_t, 6> sentinels{0x1010101010,0x2020202020,0x3030303030,0x4040404040,0x5050505050,0x6060606060};
    constexpr std::array<std::array<std::uint64_t, 2>, 6> vectors{{{11,22},{33,44},{55,66},{77,88},{99,111},{123,456}}};
    constexpr std::uint64_t fromRsi=0x1234567800000023, fromRdi=0x2345678900000024,
        fromRbx=0x3456789000000025, fromR15=0x4567890100000026, biped=0x5678901200001000;

    class Predicate : public Xbyak::CodeGenerator {
    public:
        explicit Predicate(std::uint32_t result) : CodeGenerator(512) {
            mov(rax, reinterpret_cast<std::uintptr_t>(received.data()));
            mov(ptr[rax], rcx); mov(ptr[rax+8], rdx); mov(ptr[rax+16], r8);
            mov(rax, rsp); and_(eax, 15);
            mov(r10, reinterpret_cast<std::uintptr_t>(&callbackAlignment)); mov(ptr[r10], rax);
            // Prove the wrapper preserves even legal callback clobbers.
            for (auto reg : {rcx,rdx,r8,r9,r10,r11}) mov(reg, 0xBAD0BAD0BAD0BAD0ull);
            for (int i=0;i<6;++i) pcmpeqd(Xbyak::Xmm(i), Xbyak::Xmm(i));
            xor_(eax,eax); clc(); mov(eax,result); ret(); ready();
        }
    };
    class Caller : public Xbyak::CodeGenerator {
    public:
        explicit Caller(const void* stub) : CodeGenerator(1024) {
            push(rbx); push(rbp); push(rsi); push(rdi); push(r12); push(r15);
            sub(rsp,0xA8); // aligned Windows call and a simulated cleanup frame
            mov(r12,rcx);
            lea(rbp,ptr[rsp+0x20]); mov(rax,biped); mov(ptr[rbp+0x67],rax);
            mov(rsi,fromRsi); mov(rdi,fromRdi); mov(rbx,fromRbx); mov(r15,fromR15);
            mov(rax,reinterpret_cast<std::uintptr_t>(vectors.data()));
            for(int i=0;i<6;++i) movdqu(Xbyak::Xmm(i),ptr[rax+i*16]);
            const std::array<Xbyak::Reg64,6> regs{rcx,rdx,r8,r9,r10,r11};
            for(std::size_t i=0;i<regs.size();++i) mov(regs[i],sentinels[i]);
            xor_(eax,eax); stc(); // ZF and CF must survive the replacement CALL
            mov(rax,reinterpret_cast<std::uintptr_t>(stub)); call(rax);
            mov(ptr[r12+offsetof(Observation,registers)],rax);
            for(std::size_t i=0;i<regs.size();++i) mov(ptr[r12+offsetof(Observation,registers)+(i+1)*8],regs[i]);
            for(int i=0;i<6;++i) movdqu(ptr[r12+offsetof(Observation,xmm)+i*16],Xbyak::Xmm(i));
            pushfq(); pop(rax); mov(ptr[r12+offsetof(Observation,flags)],rax);
            add(rsp,0xA8);
            pop(r15); pop(r12); pop(rdi); pop(rsi); pop(rbp); pop(rbx); ret(); ready();
        }
    };
}

int main()
{
    bool ok=true;
    Predicate resolve(41), compare(35), cleanup(36);
    for (auto kind : {Arguments::EquipmentIndex,Arguments::Attach,Arguments::Rebuild,
            Arguments::Classification,Arguments::BodyFilter,Arguments::Cleanup}) {
        rock::native_equipped_model_slot::Stub stub(kind,
            reinterpret_cast<std::uintptr_t>(resolve.getCode()), reinterpret_cast<std::uintptr_t>(compare.getCode()),
            reinterpret_cast<std::uintptr_t>(cleanup.getCode()));
        Caller caller(stub.getCode());
        Observation out{};
        caller.getCode<void(*)(Observation*)>()(&out);
        std::array<std::uint64_t,3> args{sentinels[0],sentinels[1],sentinels[2]};
        switch(kind) {
        case Arguments::EquipmentIndex: break;
        case Arguments::Attach: args[1]=static_cast<std::uint32_t>(fromRsi); break;
        case Arguments::Rebuild: args[1]=static_cast<std::uint32_t>(fromR15); break;
        case Arguments::Classification: args[1]=static_cast<std::uint32_t>(fromRbx); break;
        case Arguments::BodyFilter: args[1]=static_cast<std::uint32_t>(fromRdi); break;
        case Arguments::Cleanup: args[1]=fromRsi; args[2]=biped; break;
        }
        bool registers=true;
        for(std::size_t i=0;i<sentinels.size();++i) registers &= out.registers[i+1]==sentinels[i];
        const auto result=kind==Arguments::EquipmentIndex?41u:kind==Arguments::Cleanup?36u:35u;
        const bool pass=registers && out.registers[0]==result && received==args &&
            callbackAlignment==8 && out.xmm==vectors && (out.flags&0x41)==0x41;
        if(!pass) std::printf("Model slot ABI failed kind=%u registers=%u result=%llu args=%u alignment=%llu vectors=%u flags=%llx\n",
            static_cast<unsigned>(kind), registers, out.registers[0], received==args, callbackAlignment, out.xmm==vectors,out.flags);
        ok &= pass;
    }
    return ok?0:1;
}
