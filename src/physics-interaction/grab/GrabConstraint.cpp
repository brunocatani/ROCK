#include "physics-interaction/grab/GrabConstraint.h"

#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_set>
#include <windows.h>

namespace rock
{
    namespace
    {
        struct RetiredGrabConstraintPayload
        {
            void* constraintData = nullptr;
            HkPositionMotor* angularMotor = nullptr;
            HkPositionMotor* linearMotor = nullptr;
            std::uint32_t constraintId = 0x7FFF'FFFFu;
            std::uint32_t remainingPhysicsSteps = 0;

            [[nodiscard]] bool occupied() const
            {
                return constraintData || angularMotor || linearMotor;
            }
        };

        inline constexpr std::uint32_t kRetiredGrabConstraintPayloadGraceSteps = 8;
        inline constexpr std::size_t kMaxRetiredGrabConstraintPayloads = 512;

        std::mutex s_retiredGrabConstraintPayloadMutex;
        std::array<RetiredGrabConstraintPayload, kMaxRetiredGrabConstraintPayloads> s_retiredGrabConstraintPayloads{};
        std::uint32_t s_retiredGrabConstraintPayloadCount = 0;

        float safePositiveMotorValue(float value, float fallback)
        {
            return (std::isfinite(value) && value > 0.0f) ? value : fallback;
        }

        void freeRetiredGrabConstraintPayload(RetiredGrabConstraintPayload& payload)
        {
            if (payload.angularMotor) {
                havok_runtime::freeHavok(payload.angularMotor, HK_POSITION_MOTOR_SIZE);
            }
            if (payload.linearMotor && payload.linearMotor != payload.angularMotor) {
                havok_runtime::freeHavok(payload.linearMotor, HK_POSITION_MOTOR_SIZE);
            }
            if (payload.constraintData) {
                havok_runtime::freeHavok(payload.constraintData, GRAB_CONSTRAINT_SIZE);
            }
            payload = {};
        }

        void retireGrabConstraintPayload(ActiveConstraint& constraint)
        {
            RetiredGrabConstraintPayload payload{
                .constraintData = constraint.constraintData,
                .angularMotor = constraint.angularMotor,
                .linearMotor = constraint.linearMotor,
                .constraintId = constraint.constraintId,
                .remainingPhysicsSteps = kRetiredGrabConstraintPayloadGraceSteps,
            };
            if (!payload.occupied()) {
                return;
            }

            std::scoped_lock lock(s_retiredGrabConstraintPayloadMutex);
            for (auto& retired : s_retiredGrabConstraintPayloads) {
                if (!retired.occupied()) {
                    retired = payload;
                    ++s_retiredGrabConstraintPayloadCount;
                    ROCK_LOG_SAMPLE_DEBUG(GrabConstraint,
                        1000,
                        "Constraint {} payload retired for {} physics steps activeRetired={}",
                        payload.constraintId,
                        kRetiredGrabConstraintPayloadGraceSteps,
                        s_retiredGrabConstraintPayloadCount);
                    return;
                }
            }

            /*
             * If this ever happens, leaking one retired payload is safer than
             * freeing memory that the native solver may still read.
             */
            ROCK_LOG_ERROR(GrabConstraint,
                "Retired grab constraint payload queue full; intentionally leaking constraint {} payload to avoid native use-after-free",
                payload.constraintId);
        }
    }

    HkPositionMotor* createPositionMotor(float tau, float damping, float proportionalRecoveryVelocity, float constantRecoveryVelocity, float minForce, float maxForce)
    {
        auto* motor = static_cast<HkPositionMotor*>(havok_runtime::allocateHavok(HK_POSITION_MOTOR_SIZE));
        if (!motor) {
            ROCK_LOG_ERROR(GrabConstraint, "Failed to allocate position motor");
            return nullptr;
        }

        auto baseAddr = REL::Module::get().base();
        motor->vtable = reinterpret_cast<void*>(baseAddr + (MOTOR_VTABLE_POSITION - 0x140000000));
        motor->referenceCount = static_cast<std::int16_t>(HK_REFERENCED_OBJECT_INITIAL_REFCOUNT);
        motor->memSizeAndFlags = HK_REFERENCED_OBJECT_DEFAULT_MEM_FLAGS;
        motor->pad0C = 0;
        motor->type = 1;
        std::memset(motor->pad11, 0, sizeof(motor->pad11));
        motor->minForce = minForce;
        motor->maxForce = maxForce;
        motor->tau = tau;
        motor->damping = damping;

        motor->proportionalRecoveryVelocity = proportionalRecoveryVelocity;
        motor->constantRecoveryVelocity = constantRecoveryVelocity;

        ROCK_LOG_DEBUG(GrabConstraint,
            "Motor created: tau={:.3f} damping={:.2f} "
            "force=[{:.0f},{:.0f}] propRecov={:.1f} constRecov={:.1f}",
            tau, damping, minForce, maxForce, proportionalRecoveryVelocity, constantRecoveryVelocity);

        return motor;
    }

    using GetConstraintInfoUtil_t = void(const void*, int, void*);
    static REL::Relocation<GetConstraintInfoUtil_t> s_getConstraintInfoUtil{ REL::Offset(offsets::kFunc_GetConstraintInfoUtil) };

    static void __cdecl getRuntimeInfoCallback(void* thisPtr, void* param2, void* param3)
    {
        (void)thisPtr;
        (void)param2;

        if (param3) {
            auto* info = reinterpret_cast<std::int32_t*>(param3);
            info[0] = RUNTIME_REPORTED_SIZE;
            info[1] = RUNTIME_SOLVER_RESULTS;
        }
    }

    static void __cdecl getConstraintInfoCallback(void* thisPtr, void* infoOut)
    {
        auto* atoms = static_cast<char*>(thisPtr) + ATOMS_START;
        s_getConstraintInfoUtil(atoms, ATOMS_SIZE, infoOut);
    }

    static void __cdecl addInstanceCallback(void* thisPtr, void* constraint, void* runtime, int sizeOfRuntime)
    {
        ROCK_LOG_TRACE(GrabConstraint, "addInstance CALLED: this={:p} constraint={:p} runtime={:p} size={}", thisPtr, constraint, runtime, sizeOfRuntime);
        if (runtime) {
            std::memset(runtime, 0, sizeOfRuntime);
            ROCK_LOG_TRACE(GrabConstraint, "  memset runtime OK");
        }
    }

    static void* __cdecl destructorCallback(void* thisPtr, std::uint32_t flags)
    {
        (void)flags;
        return thisPtr;
    }

    static void __cdecl noOpIntCallback(void* thisPtr, int value)
    {
        (void)thisPtr;
        (void)value;
    }

    static float __cdecl returnZeroFloatCallback(void* thisPtr)
    {
        (void)thisPtr;
        return 0.0f;
    }

    static std::uint8_t __cdecl returnZeroByteCallback(void* thisPtr)
    {
        (void)thisPtr;
        return 0;
    }

    static void* __cdecl outStatusFloatCallback(void* thisPtr, void* statusOut, float value)
    {
        (void)thisPtr;
        (void)value;
        if (statusOut) {
            *static_cast<std::uint32_t*>(statusOut) = 0;
        }
        return statusOut;
    }

    static void* __cdecl outStatusValueCallback(void* thisPtr, void* statusOut, void* valueOut)
    {
        (void)thisPtr;
        if (statusOut) {
            *static_cast<std::uint32_t*>(statusOut) = 0;
        }
        if (valueOut) {
            *static_cast<std::uint32_t*>(valueOut) = 0;
        }
        return statusOut;
    }

    static void* s_customVtable[64] = { nullptr };
    static bool s_vtableBuilt = false;

    static constexpr int MAX_SHELLCODE_BLOCKS = 4;
    static void* s_shellcodeBlocks[MAX_SHELLCODE_BLOCKS] = { nullptr };
    static int s_shellcodeCount = 0;
    static constexpr DWORD kVirtualMemoryCommitReserve = 0x00001000u | 0x00002000u;
    static constexpr DWORD kVirtualMemoryRelease = 0x00008000u;
    static constexpr DWORD kPageExecuteRead = 0x00000020u;
    static constexpr DWORD kPageExecuteReadWrite = 0x00000040u;

    static void buildCustomVtable()
    {
        if (s_vtableBuilt)
            return;

        auto baseAddr = REL::Module::get().base();
        auto* ragdollVtable = reinterpret_cast<void**>(baseAddr + (RAGDOLL_VTABLE - 0x140000000));
        for (int i = 0; i < 64; i++) {
            s_customVtable[i] = reinterpret_cast<void*>(&returnZeroByteCallback);
        }
        constexpr int kFo4VrRagdollFunctionSlots = 24;
        for (int i = 0; i < kFo4VrRagdollFunctionSlots; i++) {
            s_customVtable[i] = ragdollVtable[i];
        }

        /*
         * FO4VR's stock hkpRagdollConstraintData vtable contains several
         * methods that read/write stock-only fields at this+0x190..0x198 or
         * release motors from this+0x100..0x110 in the destructor. ROCK owns a
         * smaller custom atom contract, so those slots must not fall through to
         * stock code.
         */
        s_customVtable[VTABLE_SLOT_DESTRUCTOR] = reinterpret_cast<void*>(&destructorCallback);
        s_customVtable[VTABLE_SLOT_SET_SOLVING_METHOD] = reinterpret_cast<void*>(&noOpIntCallback);
        s_customVtable[VTABLE_SLOT_GET_INERTIA_STABILIZATION] = reinterpret_cast<void*>(&returnZeroFloatCallback);
        s_customVtable[VTABLE_SLOT_SET_DISPLAY_FLAGS] = reinterpret_cast<void*>(&noOpIntCallback);
        s_customVtable[VTABLE_SLOT_GET_DISPLAY_FLAGS] = reinterpret_cast<void*>(&returnZeroByteCallback);
        s_customVtable[VTABLE_SLOT_SET_MOTOR_MODE] = reinterpret_cast<void*>(&noOpIntCallback);
        s_customVtable[VTABLE_SLOT_SET_MAX_FRICTION] = reinterpret_cast<void*>(&outStatusFloatCallback);
        s_customVtable[VTABLE_SLOT_GET_MAX_FRICTION] = reinterpret_cast<void*>(&outStatusValueCallback);

        auto allocShellcode = [](std::size_t size) -> std::uint8_t* {
            auto* code = static_cast<std::uint8_t*>(VirtualAlloc(nullptr, size, kVirtualMemoryCommitReserve, kPageExecuteReadWrite));
            if (code && s_shellcodeCount < MAX_SHELLCODE_BLOCKS) {
                s_shellcodeBlocks[s_shellcodeCount++] = code;
            }
            return code;
        };

        {
            auto* code = allocShellcode(16);
            if (code) {
                int off = 0;

                code[off++] = 0xB8;
                *reinterpret_cast<std::int32_t*>(code + off) = 100;
                off += 4;

                code[off++] = 0xC3;
                s_customVtable[VTABLE_SLOT_GET_TYPE] = code;
            }
        }

        {
            auto* code = allocShellcode(16);
            if (code) {
                int off = 0;

                code[off++] = 0xB0;
                code[off++] = 0x01;

                code[off++] = 0xC3;
                s_customVtable[VTABLE_SLOT_IS_VALID] = code;
            }
        }

        s_customVtable[VTABLE_SLOT_GET_CONSTRAINT_INFO] = reinterpret_cast<void*>(&getConstraintInfoCallback);

        s_customVtable[VTABLE_SLOT_GET_RUNTIME_INFO] = reinterpret_cast<void*>(&getRuntimeInfoCallback);

        s_customVtable[VTABLE_SLOT_ADD_INSTANCE] = reinterpret_cast<void*>(&addInstanceCallback);

        s_vtableBuilt = true;

        DWORD oldProtect;
        for (int i = 0; i < s_shellcodeCount; i++) {
            if (s_shellcodeBlocks[i]) {
                VirtualProtect(s_shellcodeBlocks[i], 16, kPageExecuteRead, &oldProtect);
            }
        }

        ROCK_LOG_DEBUG(GrabConstraint, "Custom vtable built at {:p} (FO4VR custom grab contract overrides active)",
            (void*)s_customVtable);
    }

    static void setGrabMotorAtomsActive(char* header, bool linearActive, bool angularActive)
    {
        if (!header) {
            return;
        }

        *(header + ATOM_RAGDOLL_MOT + 0x02) = static_cast<char>(angularActive ? 1 : 0);

        const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };
        for (int axis = 0; axis < 3; axis++) {
            *(header + linAtomOffsets[axis] + 0x02) = static_cast<char>(linearActive ? 1 : 0);
        }
    }

    ActiveConstraint createGrabConstraint(RE::hknpWorld* world, RE::hknpBodyId authorityBodyId, RE::hknpBodyId objectBodyId,
        const RE::NiTransform& authorityBodyWorld, const RE::NiPoint3& pivotAWorldGame, const float* pivotBConstraintLocalHk,
        const RE::NiTransform& desiredBodyTransformAuthoritySpace, const GrabConstraintMotorTuning& tuning)
    {
        ActiveConstraint result;
        if (!world) {
            ROCK_LOG_ERROR(GrabConstraint, "createGrabConstraint: world is null");
            return result;
        }

        buildCustomVtable();

        void* cd = havok_runtime::allocateHavok(GRAB_CONSTRAINT_SIZE);
        if (!cd) {
            ROCK_LOG_ERROR(GrabConstraint, "Failed to allocate constraint data ({:#x} bytes)", GRAB_CONSTRAINT_SIZE);
            return result;
        }
        std::memset(cd, 0, GRAB_CONSTRAINT_SIZE);

        *reinterpret_cast<void**>(cd) = s_customVtable;
        auto* header = static_cast<char*>(cd);

        *reinterpret_cast<std::uint32_t*>(header + 0x08) = HK_REFERENCED_OBJECT_HEADER_DWORD;

        *reinterpret_cast<std::uint16_t*>(header + ATOM_TRANSFORMS) = ATOM_TYPE_SET_LOCAL_TRANSFORMS;
        *reinterpret_cast<std::uint16_t*>(header + ATOM_STABILIZE) = ATOM_TYPE_SETUP_STABILIZATION;
        *reinterpret_cast<std::uint16_t*>(header + ATOM_RAGDOLL_MOT) = ATOM_TYPE_RAGDOLL_MOTOR;
        *reinterpret_cast<std::uint16_t*>(header + ATOM_LIN_MOTOR_0) = ATOM_TYPE_LIN_MOTOR;
        *reinterpret_cast<std::uint16_t*>(header + ATOM_LIN_MOTOR_1) = ATOM_TYPE_LIN_MOTOR;
        *reinterpret_cast<std::uint16_t*>(header + ATOM_LIN_MOTOR_2) = ATOM_TYPE_LIN_MOTOR;
        grab_constraint_math::writeSetupStabilizationDefaults(header + ATOM_STABILIZE);

        {
            auto* ragAtom = header + ATOM_RAGDOLL_MOT;
            *(ragAtom + 0x02) = 0;

            *reinterpret_cast<std::int16_t*>(ragAtom + 0x04) = static_cast<std::int16_t>(RT_RAGDOLL_INIT_OFFSET);
            *reinterpret_cast<std::int16_t*>(ragAtom + 0x06) = static_cast<std::int16_t>(RT_RAGDOLL_PREV_ANG_OFFSET);

            auto* target = reinterpret_cast<float*>(ragAtom + RAGDOLL_MOTOR_TARGET_B_RELATIVE_TO_A);
            target[0] = 1.0f;
            target[1] = 0.0f;
            target[2] = 0.0f;
            target[3] = 0.0f;
            target[4] = 0.0f;
            target[5] = 1.0f;
            target[6] = 0.0f;
            target[7] = 0.0f;
            target[8] = 0.0f;
            target[9] = 0.0f;
            target[10] = 1.0f;
            target[11] = 0.0f;
        }

        {
            const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };

            const std::int16_t initOffsets[3] = {
                static_cast<std::int16_t>(RT_LINEAR_INIT_BASE + 0),
                static_cast<std::int16_t>(RT_LINEAR_INIT_BASE + 1),
                static_cast<std::int16_t>(RT_LINEAR_INIT_BASE + 2),
            };
            const std::int16_t prevTargetOffsets[3] = {
                static_cast<std::int16_t>(RT_LINEAR_PREV_POS_BASE + 0x00),
                static_cast<std::int16_t>(RT_LINEAR_PREV_POS_BASE + 0x04),
                static_cast<std::int16_t>(RT_LINEAR_PREV_POS_BASE + 0x08),
            };

            for (int axis = 0; axis < 3; axis++) {
                auto* atom = header + linAtomOffsets[axis];

                *(atom + 0x02) = 0;
                *(atom + 0x03) = static_cast<char>(axis);

                *reinterpret_cast<std::int16_t*>(atom + 0x04) = initOffsets[axis];
                *reinterpret_cast<std::int16_t*>(atom + 0x06) = prevTargetOffsets[axis];

                *reinterpret_cast<float*>(atom + 0x08) = 0.0f;
            }
        }

        ROCK_LOG_DEBUG(GrabConstraint, "Custom constraint data at {:p} ({:#x} bytes, atomBase={:#x}, 6 atoms, {} solver results, runtime {:#x})",
            cd,
            GRAB_CONSTRAINT_SIZE,
            ATOMS_START,
            RUNTIME_SOLVER_RESULTS,
            RUNTIME_REPORTED_SIZE);
        ROCK_LOG_TRACE(GrabConstraint, "RagdollMotor offsets: init={:#x} prevAng={:#x} (absolute runtime offsets)", RT_RAGDOLL_INIT_OFFSET, RT_RAGDOLL_PREV_ANG_OFFSET);
        ROCK_LOG_TRACE(GrabConstraint,
            "LinMotor offsets: [{:#x},{:#x}] [{:#x},{:#x}] [{:#x},{:#x}] (absolute runtime offsets)",
            RT_LINEAR_INIT_BASE + 0,
            RT_LINEAR_PREV_POS_BASE + 0x00,
            RT_LINEAR_INIT_BASE + 1,
            RT_LINEAR_PREV_POS_BASE + 0x04,
            RT_LINEAR_INIT_BASE + 2,
            RT_LINEAR_PREV_POS_BASE + 0x08);

        {
            auto* tA_col0 = reinterpret_cast<float*>(header + GRAB_TRANSFORM_A_COL0);
            auto* tA_col1 = reinterpret_cast<float*>(header + GRAB_TRANSFORM_A_COL1);
            auto* tA_col2 = reinterpret_cast<float*>(header + GRAB_TRANSFORM_A_COL2);
            auto* tA_pos = reinterpret_cast<float*>(header + GRAB_TRANSFORM_A_POS);

            tA_col0[0] = 1.0f;
            tA_col0[1] = 0.0f;
            tA_col0[2] = 0.0f;
            tA_col0[3] = 0.0f;
            tA_col1[0] = 0.0f;
            tA_col1[1] = 1.0f;
            tA_col1[2] = 0.0f;
            tA_col1[3] = 0.0f;
            tA_col2[0] = 0.0f;
            tA_col2[1] = 0.0f;
            tA_col2[2] = 1.0f;
            tA_col2[3] = 0.0f;

            grab_constraint_math::writeConstraintPivotLocalTranslation(tA_pos, authorityBodyWorld, pivotAWorldGame, gameToHavokScale());

            auto* tB_col0 = reinterpret_cast<float*>(header + GRAB_TRANSFORM_B_COL0);
            auto* tB_pos = reinterpret_cast<float*>(header + GRAB_TRANSFORM_B_POS);
            auto* targetBRelativeToA = reinterpret_cast<float*>(header + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_B_RELATIVE_TO_A);

            grab_constraint_math::writeInitialGrabAngularFrame(tB_col0, targetBRelativeToA, desiredBodyTransformAuthoritySpace);

            tB_pos[0] = pivotBConstraintLocalHk[0];
            tB_pos[1] = pivotBConstraintLocalHk[1];
            tB_pos[2] = pivotBConstraintLocalHk[2];
            tB_pos[3] = 0.0f;

            ROCK_LOG_TRACE(GrabConstraint,
                "setInBodySpace: pivotA=({:.3f},{:.3f},{:.3f}) [palm] "
                "pivotB=({:.3f},{:.3f},{:.3f}) [active-pivot-b] "
                "tB_inverse_col0=({:.3f},{:.3f},{:.3f})",
                tA_pos[0], tA_pos[1], tA_pos[2], tB_pos[0], tB_pos[1], tB_pos[2], tB_col0[0], tB_col0[1], tB_col0[2]);

            ROCK_LOG_TRACE(GrabConstraint, "targetBRelativeToA initial inverse solver rows: row0=[{:.3f},{:.3f},{:.3f}] row1=[{:.3f},{:.3f},{:.3f}]", targetBRelativeToA[0], targetBRelativeToA[1],
                targetBRelativeToA[2], targetBRelativeToA[4], targetBRelativeToA[5], targetBRelativeToA[6]);
        }

        const float linearTau = safePositiveMotorValue(tuning.linearTau, 0.03f);
        const float linearDamping = safePositiveMotorValue(tuning.linearDamping, 0.8f);
        const float linearProportionalRecovery = safePositiveMotorValue(tuning.linearProportionalRecovery, 2.0f);
        const float linearConstantRecovery = safePositiveMotorValue(tuning.linearConstantRecovery, 1.0f);
        const float linearMaxForce = (std::max)(0.0f, std::isfinite(tuning.linearMaxForce) ? tuning.linearMaxForce : 0.0f);

        const float angularTau = safePositiveMotorValue(tuning.angularTau, linearTau);
        const float angularDamping = safePositiveMotorValue(tuning.angularDamping, linearDamping);
        const float angularProportionalRecovery = safePositiveMotorValue(tuning.angularProportionalRecovery, linearProportionalRecovery);
        const float angularConstantRecovery = safePositiveMotorValue(tuning.angularConstantRecovery, linearConstantRecovery);
        const float angularMaxForce = (std::max)(0.0f, std::isfinite(tuning.angularMaxForce) ? tuning.angularMaxForce : 0.0f);

        auto* angMotor = createPositionMotor(
            angularTau,
            angularDamping,
            angularProportionalRecovery,
            angularConstantRecovery,
            -angularMaxForce,
            angularMaxForce);

        auto* linMotor = createPositionMotor(
            linearTau,
            linearDamping,
            linearProportionalRecovery,
            linearConstantRecovery,
            -linearMaxForce,
            linearMaxForce);

        if (!angMotor || !linMotor) {
            ROCK_LOG_ERROR(GrabConstraint, "Motor creation failed — aborting");
            if (angMotor)
                havok_runtime::freeHavok(angMotor, HK_POSITION_MOTOR_SIZE);
            if (linMotor)
                havok_runtime::freeHavok(linMotor, HK_POSITION_MOTOR_SIZE);
            havok_runtime::freeHavok(cd, GRAB_CONSTRAINT_SIZE);
            return result;
        }

        {
            auto* ragAtom = header + ATOM_RAGDOLL_MOT;
            for (int i = 0; i < 3; i++) {
                *reinterpret_cast<void**>(ragAtom + RAGDOLL_MOTOR_MOTORS + i * 8) = angMotor;
            }
        }
        {
            const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };
            for (int axis = 0; axis < 3; axis++) {
                *reinterpret_cast<void**>(header + linAtomOffsets[axis] + 0x10) = linMotor;
            }
        }

        /*
         * Dynamic grab rotation is solver-owned. The type-19 ragdoll motor atom
         * is always active so there is no alternate direct angular velocity
         * writer competing with the constraint target.
         */
        setGrabMotorAtomsActive(header, true, true);

        ROCK_LOG_DEBUG(GrabConstraint,
            "Motors attached before CreateConstraint: angularBudget={:.0f} authority={} ragdollAtom=enabled linear={:.0f}",
            angularMaxForce,
            grabAngularAuthorityName(tuning.angularAuthority),
            linearMaxForce);

        RE::hknpConstraintCinfo cinfo{};
        cinfo.constraintData = reinterpret_cast<RE::hkpConstraintData*>(cd);
        cinfo.bodyIdA = authorityBodyId.value;
        cinfo.bodyIdB = objectBodyId.value;

        std::uint32_t outId = 0x7FFF'FFFF;
        world->CreateConstraint(&outId, cinfo);

        if (outId == 0x7FFF'FFFF) {
            ROCK_LOG_ERROR(GrabConstraint, "CreateConstraint returned invalid ID");
            if (angMotor) {
                havok_runtime::freeHavok(angMotor, HK_POSITION_MOTOR_SIZE);
            }
            if (linMotor && linMotor != angMotor) {
                havok_runtime::freeHavok(linMotor, HK_POSITION_MOTOR_SIZE);
            }
            havok_runtime::freeHavok(cd, GRAB_CONSTRAINT_SIZE);
            return result;
        }

        ROCK_LOG_DEBUG(GrabConstraint, "Constraint created: id={}, authorityBody={}, heldBody={}", outId, authorityBodyId.value, objectBodyId.value);

        result.constraintId = outId;
        result.constraintData = cd;
        result.angularMotor = angMotor;
        result.linearMotor = linMotor;
        result.angularAuthority = tuning.angularAuthority;
        result.currentTau = linearTau;
        result.currentMaxForce = linearMaxForce;
        result.targetMaxForce = linearMaxForce;

        return result;
    }

    ActiveConstraint createGrabConstraint(RE::hknpWorld* world, RE::hknpBodyId authorityBodyId, RE::hknpBodyId objectBodyId,
        const RE::NiTransform& authorityBodyWorld, const RE::NiPoint3& pivotAWorldGame, const float* pivotBConstraintLocalHk,
        const RE::NiTransform& desiredBodyTransformAuthoritySpace, float tau, float damping, float maxForce, float proportionalRecovery, float constantRecovery)
    {
        const float linearMaxForce = (std::max)(0.0f, std::isfinite(maxForce) ? maxForce : 0.0f);
        return createGrabConstraint(
            world,
            authorityBodyId,
            objectBodyId,
            authorityBodyWorld,
            pivotAWorldGame,
            pivotBConstraintLocalHk,
            desiredBodyTransformAuthoritySpace,
            GrabConstraintMotorTuning{
                .linearTau = tau,
                .linearDamping = damping,
                .linearProportionalRecovery = proportionalRecovery,
                .linearConstantRecovery = constantRecovery,
                .linearMaxForce = linearMaxForce,
                .angularTau = g_rockConfig.rockGrabAngularTau,
                .angularDamping = g_rockConfig.rockGrabAngularDamping,
                .angularProportionalRecovery = g_rockConfig.rockGrabAngularProportionalRecovery,
                .angularConstantRecovery = g_rockConfig.rockGrabAngularConstantRecovery,
                .angularMaxForce = linearMaxForce,
            });
    }

    void destroyGrabConstraint(RE::hknpWorld* world, ActiveConstraint& constraint)
    {
        if (!constraint.isValid())
            return;

        if (constraint.constraintData) {
            setGrabMotorAtomsActive(static_cast<char*>(constraint.constraintData), false, false);
        }

        if (world) {
            std::uint32_t ids[1] = { constraint.constraintId };
            world->DestroyConstraints(ids, 1);
            ROCK_LOG_DEBUG(GrabConstraint, "Constraint {} destroyed", constraint.constraintId);
        } else {
            ROCK_LOG_WARN(GrabConstraint, "Constraint {} destroy requested without world; payload retired but native constraint removal could not be issued",
                constraint.constraintId);
        }

        retireGrabConstraintPayload(constraint);
        constraint.clear();
    }

    void serviceRetiredGrabConstraintPayloads(std::uint32_t completedPhysicsSteps)
    {
        if (completedPhysicsSteps == 0) {
            return;
        }

        std::scoped_lock lock(s_retiredGrabConstraintPayloadMutex);
        for (auto& payload : s_retiredGrabConstraintPayloads) {
            if (!payload.occupied()) {
                continue;
            }

            payload.remainingPhysicsSteps =
                payload.remainingPhysicsSteps > completedPhysicsSteps ? payload.remainingPhysicsSteps - completedPhysicsSteps : 0;
            if (payload.remainingPhysicsSteps == 0) {
                const auto constraintId = payload.constraintId;
                freeRetiredGrabConstraintPayload(payload);
                if (s_retiredGrabConstraintPayloadCount > 0) {
                    --s_retiredGrabConstraintPayloadCount;
                }
                ROCK_LOG_SAMPLE_DEBUG(GrabConstraint,
                    1000,
                    "Retired constraint {} payload reclaimed activeRetired={}",
                    constraintId,
                    s_retiredGrabConstraintPayloadCount);
            }
        }
    }

    namespace
    {
        bool normalizeGrabbedInertiaMotion(RE::hknpWorld* world, RE::hknpBodyId bodyId, SavedObjectState::SavedMotionInertiaState& savedMotionState)
        {
            if (!world)
                return false;

            auto* body = havok_runtime::getBody(world, bodyId);
            if (!body)
                return false;

            auto motionIndex = body->motionIndex;
            auto* motion = havok_runtime::getMotion(world, motionIndex);
            if (!motion)
                return false;

            auto* packed = reinterpret_cast<std::int16_t*>(reinterpret_cast<char*>(motion) + MOTION_PACKED_INERTIA_OFFSET);

            savedMotionState.bodyId = bodyId;
            savedMotionState.motionIndex = motionIndex;
            savedMotionState.savedPackedInertia[0] = packed[0];
            savedMotionState.savedPackedInertia[1] = packed[1];
            savedMotionState.savedPackedInertia[2] = packed[2];

            if (packed[0] <= 0 || packed[1] <= 0 || packed[2] <= 0) {
                ROCK_LOG_WARN(GrabConstraint, "Skipping inertia normalization: zero/negative packed value");
                return false;
            }

            float invI[3] = { unpackBfloat16(packed[0]), unpackBfloat16(packed[1]), unpackBfloat16(packed[2]) };

            ROCK_LOG_DEBUG(GrabConstraint,
                "Inertia pre-normalize: body={} motion={} packed=[{},{},{}] "
                "float=[{:.6e},{:.6e},{:.6e}] mass_packed={}",
                bodyId.value, motionIndex, packed[0], packed[1], packed[2], invI[0], invI[1], invI[2], packed[3]);

            if (invI[0] <= 0.0f || invI[1] <= 0.0f || invI[2] <= 0.0f) {
                ROCK_LOG_WARN(GrabConstraint, "Skipping inertia normalization: zero unpacked float value");
                return false;
            }

            const auto normalizedInertia = grab_inertia_policy::normalizeInverseInertiaAxesForGrab(
                invI[0],
                invI[1],
                invI[2],
                g_rockConfig.rockGrabMaxInertiaRatio,
                g_rockConfig.rockGrabMinInertia);
            if (!normalizedInertia.valid) {
                ROCK_LOG_WARN(GrabConstraint, "Skipping inertia normalization: invalid unpacked values");
                return false;
            }

            if (normalizedInertia.modified) {
                invI[0] = normalizedInertia.normalized[0];
                invI[1] = normalizedInertia.normalized[1];
                invI[2] = normalizedInertia.normalized[2];

                packed[0] = repackBfloat16(invI[0]);
                packed[1] = repackBfloat16(invI[1]);
                packed[2] = repackBfloat16(invI[2]);

                ROCK_LOG_DEBUG(GrabConstraint,
                    "Inertia clamped: ratio {:.1f}x -> {:.1f}x limit={:.1f} minInertiaMaxInv={:.6e} "
                    "packed [{},{},{}] -> [{},{},{}] float [{:.6e},{:.6e},{:.6e}]",
                    normalizedInertia.originalRatio,
                    normalizedInertia.normalizedRatio,
                    normalizedInertia.ratioLimit,
                    normalizedInertia.maxInverseInertiaFromMinimum,
                    savedMotionState.savedPackedInertia[0],
                    savedMotionState.savedPackedInertia[1],
                    savedMotionState.savedPackedInertia[2],
                    packed[0],
                    packed[1],
                    packed[2],
                    invI[0],
                    invI[1],
                    invI[2]);
            } else {
                ROCK_LOG_DEBUG(GrabConstraint,
                    "Inertia OK: ratio {:.1f}x limit={:.1f} minInertiaMaxInv={:.6e}, no clamping needed",
                    normalizedInertia.originalRatio,
                    normalizedInertia.ratioLimit,
                    normalizedInertia.maxInverseInertiaFromMinimum);
            }

            savedMotionState.inertiaModified = true;

            havok_runtime::rebuildMotionMassProperties(world, motionIndex);
            ROCK_LOG_TRACE(GrabConstraint, "rebuildMotionMassProperties called for motionIndex={}", motionIndex);
            return true;
        }
    }

    void normalizeGrabbedInertia(RE::hknpWorld* world, RE::hknpBodyId bodyId, SavedObjectState& savedState)
    {
        savedState.motionInertiaStates.clear();
        savedState.inertiaModified = false;
        SavedObjectState::SavedMotionInertiaState savedMotion{};
        if (normalizeGrabbedInertiaMotion(world, bodyId, savedMotion)) {
            savedState.savedPackedInertia[0] = savedMotion.savedPackedInertia[0];
            savedState.savedPackedInertia[1] = savedMotion.savedPackedInertia[1];
            savedState.savedPackedInertia[2] = savedMotion.savedPackedInertia[2];
            savedState.inertiaModified = true;
            savedState.motionInertiaStates.push_back(savedMotion);
        }
    }

    void normalizeGrabbedInertiaForBodies(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId, const std::vector<std::uint32_t>& heldBodyIds, SavedObjectState& savedState)
    {
        savedState.motionInertiaStates.clear();
        savedState.inertiaModified = false;

        if (!world) {
            return;
        }

        std::vector<std::uint32_t> bodyIds;
        bodyIds.reserve(heldBodyIds.size() + 1);
        bodyIds.push_back(primaryBodyId.value);
        for (const auto bodyId : heldBodyIds) {
            bodyIds.push_back(bodyId);
        }

        std::unordered_set<std::uint32_t> seenMotionIds;
        for (const auto rawBodyId : bodyIds) {
            if (rawBodyId == 0x7FFF'FFFF) {
                continue;
            }
            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ rawBodyId });
            if (!body) {
                continue;
            }
            if (!seenMotionIds.insert(body->motionIndex).second) {
                continue;
            }
            SavedObjectState::SavedMotionInertiaState savedMotion{};
            if (normalizeGrabbedInertiaMotion(world, RE::hknpBodyId{ rawBodyId }, savedMotion)) {
                savedState.motionInertiaStates.push_back(savedMotion);
            }
        }

        if (!savedState.motionInertiaStates.empty()) {
            const auto& primarySaved = savedState.motionInertiaStates.front();
            savedState.savedPackedInertia[0] = primarySaved.savedPackedInertia[0];
            savedState.savedPackedInertia[1] = primarySaved.savedPackedInertia[1];
            savedState.savedPackedInertia[2] = primarySaved.savedPackedInertia[2];
            savedState.inertiaModified = true;
        }
    }

    void restoreGrabbedInertia(RE::hknpWorld* world, SavedObjectState& savedState)
    {
        if (!world || !savedState.inertiaModified)
            return;

        if (savedState.motionInertiaStates.empty()) {
            SavedObjectState::SavedMotionInertiaState legacyState{};
            legacyState.bodyId = savedState.bodyId;
            legacyState.savedPackedInertia[0] = savedState.savedPackedInertia[0];
            legacyState.savedPackedInertia[1] = savedState.savedPackedInertia[1];
            legacyState.savedPackedInertia[2] = savedState.savedPackedInertia[2];
            legacyState.inertiaModified = savedState.inertiaModified;
            if (auto* body = havok_runtime::getBody(world, savedState.bodyId)) {
                legacyState.motionIndex = body->motionIndex;
            }
            savedState.motionInertiaStates.push_back(legacyState);
        }

        for (auto& savedMotion : savedState.motionInertiaStates) {
            if (!savedMotion.inertiaModified) {
                continue;
            }
            auto* motion = havok_runtime::getMotion(world, savedMotion.motionIndex);
            if (!motion) {
                continue;
            }
            auto* packed = reinterpret_cast<std::int16_t*>(reinterpret_cast<char*>(motion) + MOTION_PACKED_INERTIA_OFFSET);

            packed[0] = savedMotion.savedPackedInertia[0];
            packed[1] = savedMotion.savedPackedInertia[1];
            packed[2] = savedMotion.savedPackedInertia[2];

            ROCK_LOG_DEBUG(
                GrabConstraint, "Inertia restored: body={} motion={} -> packed=[{},{},{}]", savedMotion.bodyId.value, savedMotion.motionIndex, packed[0], packed[1], packed[2]);
            havok_runtime::rebuildMotionMassProperties(world, savedMotion.motionIndex);
            savedMotion.inertiaModified = false;
        }
        savedState.inertiaModified = false;
        savedState.motionInertiaStates.clear();
    }

    void cleanupGrabConstraintVtable()
    {
        for (int i = 0; i < s_shellcodeCount; i++) {
            if (s_shellcodeBlocks[i]) {
                VirtualFree(s_shellcodeBlocks[i], 0, kVirtualMemoryRelease);
                s_shellcodeBlocks[i] = nullptr;
            }
        }
        s_shellcodeCount = 0;
        s_vtableBuilt = false;
        ROCK_LOG_DEBUG(GrabConstraint, "Vtable shellcode freed");
    }
}
