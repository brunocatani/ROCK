#include "physics-interaction/native/CharacterProxyFaultPolicy.h"

#include <cstdio>

namespace
{
    constexpr std::uintptr_t instruction = 0x141E4DEED;
    constexpr std::uintptr_t destructorCaller = 0x141E4B4CA;
}

int main()
{
    using namespace rock::character_proxy_fault_policy;
    bool passed = true;
    const auto check = [&](const char* label, Evidence evidence,
                           Disposition expected,
                           std::uintptr_t approvedCaller = destructorCaller) {
        if (classify(evidence, instruction, approvedCaller) != expected) {
            std::printf("FAILED: %s\n", label);
            passed = false;
        }
    };

    // Replay both active callsites and the body/read addresses from the
    // 2026-09-10 crash. Neither caller can accept a successful null result.
    Evidence observed{ instruction, 0x6237C, 0x62310, 0x141E4B8DE, true };
    check("constraint listener preserves original exception", observed,
        Disposition::PreserveNativeException);
    observed.caller = 0x141E9E20D;
    check("gravity caller preserves original exception", observed,
        Disposition::PreserveNativeException);
    observed.insidePhysicsStep = false;
    check("active caller is not teardown just because TLS is clear", observed,
        Disposition::PreserveNativeException);
    observed.caller = destructorCaller;
    check("verified destructor can recover", observed, Disposition::RecoverDestructor);
    observed.insidePhysicsStep = true;
    check("destructor during physics does not recover", observed,
        Disposition::PreserveNativeException);
    observed.insidePhysicsStep = false;
    check("unvalidated destructor does not recover", observed,
        Disposition::PreserveNativeException, 0);
    observed.instruction += 1;
    check("different fault instruction is not swallowed", observed, Disposition::Unrelated);
    observed.instruction = instruction;
    observed.readAddress += 4;
    check("different failed field is not swallowed", observed, Disposition::Unrelated);
    observed.readAddress = 0x6237C;
    observed.bodyAddress += 1;
    check("non-body-aligned address is not swallowed", observed, Disposition::Unrelated);
    observed.bodyAddress = 0x1000'0080;
    observed.readAddress = observed.bodyAddress + 0x6C;
    check("unrelated pointer corruption is not swallowed", observed, Disposition::Unrelated);

    return passed ? 0 : 1;
}
