// Patches native VR melee to read the wand of the hand that holds each weapon
// instead of always the right wand.
#include "physics-interaction/weapon/native/NativeMeleeWand.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <xbyak/xbyak.h>
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryMelee.h"

namespace rock::native_melee_wand
{
    namespace
    {
        // FO4VR 1.2.72, verified in ghidra_fo4vr and checked live before patching.
        constexpr std::uintptr_t kPrimaryWand = 0x5AC8EB0; // right wand singleton
        constexpr std::uintptr_t kSecondaryWand = 0x5AC72B8; // left wand singleton
        constexpr std::ptrdiff_t kAveragedLinearSpeed = 0x28;

        // mov reg, [rip+disp32] loads of the primary wand.
        enum class Reader
        {
            Gate,
            Contact
        };

        struct Load
        {
            std::uintptr_t site;
            std::array<std::uint8_t, 3> opcode;
            bool intoR14;
            Reader reader;
        };

        constexpr std::array<Load, 4> kLoads{ {
            { 0xEFED63, { 0x48, 0x8B, 0x05 }, false, Reader::Gate }, // velocity gate
            { 0xEFF333, { 0x48, 0x8B, 0x05 }, false, Reader::Contact }, // contact gun-bash speed
            { 0xEFF50A, { 0x4C, 0x8B, 0x35 }, true, Reader::Contact }, // contact hit wand
            { 0xEFF6F8, { 0x48, 0x8B, 0x05 }, false, Reader::Contact }, // contact hit velocity
        } };
        constexpr std::size_t kLoadSize = 7;

        bool s_installed = false; // Plugin load owner; immutable afterwards.
        // Frame thread writes, any thread reads. Zero means "native right wand".
        std::atomic<std::uintptr_t> s_gateWand{ 0 };
        std::atomic<std::uintptr_t> s_indexZeroWand{ 0 };
        std::atomic<std::uintptr_t> s_indexOneWand{ 0 };
        Holders s_last{};

        std::uintptr_t wand(std::uintptr_t rva) noexcept
        {
            std::uintptr_t value = 0;
            (void)native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(REL::Offset(rva).address()),
                value);
            return value;
        }

        std::uintptr_t orNative(std::uintptr_t published) noexcept
        {
            return published ? published
                             : *reinterpret_cast<const std::uintptr_t*>(REL::Offset(kPrimaryWand).address());
        }

        std::uintptr_t gateWand() noexcept { return orNative(s_gateWand.load(std::memory_order_acquire)); }

        std::uintptr_t contactWand() noexcept
        {
            return orNative((native_secondary_melee::routedContact() ? s_indexOneWand : s_indexZeroWand)
                    .load(std::memory_order_acquire));
        }

        // Replaces one 7-byte load with CALL + 2 NOPs. The native code keeps
        // every register but the load target and reads no flags after it.
        class WandLoad final : public Xbyak::CodeGenerator
        {
        public:
            WandLoad(std::uintptr_t resolver, bool intoR14) : CodeGenerator(256)
            {
                // Entry RSP is 8 mod 16 (native call sites are aligned).
                sub(rsp, 0xC8);
                save(intoR14);
                mov(rax, resolver);
                call(rax);
                if (intoR14)
                    mov(r14, rax);
                restore(intoR14);
                add(rsp, 0xC8);
                ret();
                ready();
            }

        private:
            void save(bool keepRax)
            {
                int offset = 0x20;
                for (auto reg : { rcx, rdx, r8, r9, r10, r11 }) {
                    mov(ptr[rsp + offset], reg);
                    offset += 8;
                }
                if (keepRax)
                    mov(ptr[rsp + offset], rax);
                for (int i = 0; i < 6; ++i)
                    movdqu(ptr[rsp + 0x60 + i * 16], Xbyak::Xmm(i));
            }

            void restore(bool keepRax)
            {
                for (int i = 0; i < 6; ++i)
                    movdqu(Xbyak::Xmm(i), ptr[rsp + 0x60 + i * 16]);
                int offset = 0x20;
                for (auto reg : { rcx, rdx, r8, r9, r10, r11 }) {
                    mov(reg, ptr[rsp + offset]);
                    offset += 8;
                }
                if (keepRax)
                    mov(rax, ptr[rsp + offset]);
            }
        };

        bool loadMatches(const Load& load) noexcept
        {
            std::array<std::uint8_t, kLoadSize> live{};
            const auto address = REL::Offset(load.site).address();
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), live.data(),
                    live.size()) ||
                std::memcmp(live.data(), load.opcode.data(), load.opcode.size()) != 0)
                return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, live.data() + load.opcode.size(), sizeof(displacement));
            return address + kLoadSize + displacement == REL::Offset(kPrimaryWand).address();
        }

        float linearSpeed(std::uintptr_t wandObject) noexcept
        {
            float speed = 0.0f;
            (void)native_memory::tryReadField(reinterpret_cast<const void*>(wandObject), kAveragedLinearSpeed, speed);
            return speed;
        }
    }

    bool install() noexcept
    {
        if (s_installed)
            return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72)
            return false;
        std::array<std::array<std::uint8_t, kLoadSize>, kLoads.size()> originals{};
        for (std::size_t i = 0; i < kLoads.size(); ++i) {
            if (!loadMatches(kLoads[i])) {
                ROCK_LOG_ERROR(Init, "Melee wand routing refused: load at {:#x} is not the primary wand",
                    kLoads[i].site);
                return false;
            }
            (void)native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(kLoads[i].site).address()), originals[i].data(), kLoadSize);
        }
        try {
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < 1024)
                throw std::runtime_error("trampoline space");
            for (const auto& load : kLoads) {
                const auto resolver = load.reader == Reader::Gate ? reinterpret_cast<std::uintptr_t>(&gateWand)
                                                                  : reinterpret_cast<std::uintptr_t>(&contactWand);
                WandLoad stub(resolver, load.intoR14);
                auto* code = trampoline.allocate(stub.getSize());
                std::memcpy(code, stub.getCode(), stub.getSize());
                const auto address = REL::Offset(load.site).address();
                constexpr std::array<std::uint8_t, 2> nops{ 0x90, 0x90 };
                REL::safe_write(address + 5, nops.data(), nops.size());
                (void)trampoline.write_call<5>(address, reinterpret_cast<std::uintptr_t>(code));
            }
            if (!FlushInstructionCache(GetCurrentProcess(), nullptr, 0))
                throw std::runtime_error("instruction cache");
        } catch (...) {
            for (std::size_t i = 0; i < kLoads.size(); ++i)
                REL::safe_write(REL::Offset(kLoads[i].site).address(), originals[i].data(), kLoadSize);
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_ERROR(Init, "Melee wand routing install failed; original loads restored");
            return false;
        }
        s_installed = true;
        ROCK_LOG_INFO(Init,
            "Melee wand routing installed: swing gate and contacts read the wand of the hand holding the weapon");
        return true;
    }

    void update(const Holders& holders) noexcept
    {
        if (!s_installed)
            return;
        const auto right = wand(kPrimaryWand);
        const auto left = wand(kSecondaryWand);
        if (!right || !left) {
            s_gateWand.store(0, std::memory_order_release);
            s_indexZeroWand.store(0, std::memory_order_release);
            s_indexOneWand.store(0, std::memory_order_release);
            return;
        }
        const auto zero = holders.indexZeroLeft ? left : right;
        const auto one = holders.indexOneHeld ? (holders.indexOneLeft ? left : right) : zero;
        // A weapon in both hands: the faster hand starts the swing or bash
        // (speeds are the wand's own 12-frame averages).
        const bool oneLeads =
            holders.indexZeroHeld && holders.indexOneHeld && one != zero && linearSpeed(one) > linearSpeed(zero);
        s_indexZeroWand.store(zero, std::memory_order_release);
        s_indexOneWand.store(one, std::memory_order_release);
        s_gateWand.store(oneLeads ? one : zero, std::memory_order_release);
        if (holders.indexZeroLeft != s_last.indexZeroLeft || holders.indexOneHeld != s_last.indexOneHeld ||
            holders.indexOneLeft != s_last.indexOneLeft) {
            ROCK_LOG_INFO(Weapon, "Native melee wands: index 0 {} wand, index 1 {}",
                holders.indexZeroLeft ? "left" : "right",
                holders.indexOneHeld ? (holders.indexOneLeft ? "left wand" : "right wand") : "none");
        }
        s_last = holders;
    }
}
