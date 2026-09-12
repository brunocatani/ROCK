#pragma once

#include <cstdint>

namespace rock::character_proxy_fault_policy
{
    enum class Disposition
    {
        Unrelated,
        PreserveNativeException,
        RecoverDestructor,
    };

    struct Evidence
    {
        std::uintptr_t instruction = 0;
        std::uintptr_t readAddress = 0;
        std::uintptr_t bodyAddress = 0;
        std::uintptr_t caller = 0;
        bool insidePhysicsStep = false;
    };

    // Applies only after the SEH boundary has identified a read access violation.
    [[nodiscard]] constexpr Disposition classify(const Evidence& evidence,
        std::uintptr_t verifiedInstruction, std::uintptr_t verifiedDestructorCaller)
    {
        if (verifiedInstruction == 0 || evidence.instruction != verifiedInstruction ||
            evidence.bodyAddress >= 0x1000'0000 || evidence.bodyAddress % 0x90 != 0 ||
            evidence.readAddress != evidence.bodyAddress + 0x6C) {
            return Disposition::Unrelated;
        }
        if (verifiedDestructorCaller != 0 && evidence.caller == verifiedDestructorCaller &&
            !evidence.insidePhysicsStep) {
            return Disposition::RecoverDestructor;
        }
        return Disposition::PreserveNativeException;
    }
}
