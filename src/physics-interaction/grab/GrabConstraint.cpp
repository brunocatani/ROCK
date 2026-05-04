#include "physics-interaction/grab/GrabConstraint.h"

#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "RockConfig.h"

#include <limits>
#include <unordered_set>
#include <windows.h>

namespace rock
{
    HkPositionMotor* createPositionMotor(float tau, float damping, float proportionalRecoveryVelocity, float constantRecoveryVelocity, float minForce, float maxForce)
    {
        auto* motor = static_cast<HkPositionMotor*>(havok_runtime::allocateHavok(HK_POSITION_MOTOR_SIZE));
        if (!motor) {
            ROCK_LOG_ERROR(GrabConstraint, "Failed to allocate position motor");
            return nullptr;
        }

        auto baseAddr = REL::Module::get().base();
        motor->vtable = reinterpret_cast<void*>(baseAddr + (MOTOR_VTABLE_POSITION - 0x140000000));
        motor->memSizeAndFlags = 0x0001;
        motor->referenceCount = -1;
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

    static void* s_customVtable[64] = { nullptr };
    static bool s_vtableBuilt = false;

    static constexpr int MAX_SHELLCODE_BLOCKS = 4;
    static void* s_shellcodeBlocks[MAX_SHELLCODE_BLOCKS] = { nullptr };
    static int s_shellcodeCount = 0;

    static void buildCustomVtable()
    {
        if (s_vtableBuilt)
            return;

        auto baseAddr = REL::Module::get().base();
        auto* ragdollVtable = reinterpret_cast<void**>(baseAddr + (RAGDOLL_VTABLE - 0x140000000));
        for (int i = 0; i < 64; i++) {
            s_customVtable[i] = ragdollVtable[i];
        }

        auto allocShellcode = [](std::size_t size) -> std::uint8_t* {
            auto* code = static_cast<std::uint8_t*>(VirtualAlloc(nullptr, size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));
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

        {
            auto* code = allocShellcode(16);
            if (code) {
                code[0] = 0xC3;
                s_customVtable[15] = code;
            }
        }

        s_vtableBuilt = true;

        DWORD oldProtect;
        for (int i = 0; i < s_shellcodeCount; i++) {
            if (s_shellcodeBlocks[i]) {
                VirtualProtect(s_shellcodeBlocks[i], 16, PAGE_EXECUTE_READ, &oldProtect);
            }
        }

        ROCK_LOG_DEBUG(GrabConstraint, "Custom vtable built at {:p} (6 overrides: getType/isValid/getConstraintInfo/setSolvingMethod/getRuntimeInfo/addInstance)",
            (void*)s_customVtable);
    }

    static void setGrabMotorAtomsActive(char* header, bool active)
    {
        if (!header) {
            return;
        }

        const auto enabled = active ? 1 : 0;
        *(header + ATOM_RAGDOLL_MOT + 0x02) = static_cast<char>(enabled);

        const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };
        for (int axis = 0; axis < 3; axis++) {
            *(header + linAtomOffsets[axis] + 0x02) = static_cast<char>(enabled);
        }
    }

    ActiveConstraint createGrabConstraint(RE::hknpWorld* world, RE::hknpBodyId handBodyId, RE::hknpBodyId objectBodyId,
        const RE::NiTransform& handBodyWorld, const RE::NiPoint3& palmWorldGame, const float* pivotBBodyLocalHk,
        const RE::NiTransform& desiredBodyTransformHandSpace, float tau, float damping, float maxForce, float proportionalRecovery, float constantRecovery)
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

        *reinterpret_cast<std::int16_t*>(header + 0x08) = -1;
        *reinterpret_cast<std::uint16_t*>(header + 0x0A) = 0x0001;

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

            auto* target = reinterpret_cast<float*>(ragAtom + 0x10);
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

            const std::int16_t initOffsets[3] = { 0x40, 0x31, 0x22 };
            const std::int16_t prevTargetOffsets[3] = { 0x44, 0x38, 0x2C };

            for (int axis = 0; axis < 3; axis++) {
                auto* atom = header + linAtomOffsets[axis];

                *(atom + 0x02) = 0;
                *(atom + 0x03) = static_cast<char>(axis);

                *reinterpret_cast<std::int16_t*>(atom + 0x04) = initOffsets[axis];
                *reinterpret_cast<std::int16_t*>(atom + 0x06) = prevTargetOffsets[axis];

                *reinterpret_cast<float*>(atom + 0x08) = 0.0f;
            }
        }

        ROCK_LOG_DEBUG(GrabConstraint, "Custom constraint data at {:p} ({:#x} bytes, 6 atoms, {} solver results, runtime {:#x})", cd, GRAB_CONSTRAINT_SIZE, RUNTIME_SOLVER_RESULTS,
            RUNTIME_REPORTED_SIZE);
        ROCK_LOG_TRACE(GrabConstraint, "RagdollMotor offsets: init={:#x} prevAng={:#x} (relative to ptr at 0x00)", RT_RAGDOLL_INIT_OFFSET, RT_RAGDOLL_PREV_ANG_OFFSET);
        ROCK_LOG_TRACE(GrabConstraint, "LinMotor offsets: [0x40,0x44] [0x31,0x38] [0x22,0x2C] (relative to per-atom ptrs at 0x30,0x40,0x50)");

        {
            auto* tA_col0 = reinterpret_cast<float*>(header + offsets::kTransformA_Col0);
            auto* tA_col1 = reinterpret_cast<float*>(header + offsets::kTransformA_Col1);
            auto* tA_col2 = reinterpret_cast<float*>(header + offsets::kTransformA_Col2);
            auto* tA_pos = reinterpret_cast<float*>(header + offsets::kTransformA_Pos);

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

            grab_constraint_math::writeConstraintPivotLocalTranslation(tA_pos, handBodyWorld, palmWorldGame, gameToHavokScale());

            auto* tB_col0 = reinterpret_cast<float*>(header + offsets::kTransformB_Col0);
            auto* tB_pos = reinterpret_cast<float*>(header + offsets::kTransformB_Pos);
            auto* targetBRca = reinterpret_cast<float*>(header + ATOM_RAGDOLL_MOT + 0x10);

            grab_constraint_math::writeInitialGrabAngularFrame(tB_col0, targetBRca, desiredBodyTransformHandSpace);

            tB_pos[0] = pivotBBodyLocalHk[0];
            tB_pos[1] = pivotBBodyLocalHk[1];
            tB_pos[2] = pivotBBodyLocalHk[2];
            tB_pos[3] = 0.0f;

            ROCK_LOG_TRACE(GrabConstraint,
                "setInBodySpace: pivotA=({:.3f},{:.3f},{:.3f}) [palm] "
                "pivotB=({:.3f},{:.3f},{:.3f}) [surface] "
                "tB_inverse_col0=({:.3f},{:.3f},{:.3f})",
                tA_pos[0], tA_pos[1], tA_pos[2], tB_pos[0], tB_pos[1], tB_pos[2], tB_col0[0], tB_col0[1], tB_col0[2]);

            ROCK_LOG_TRACE(GrabConstraint, "target_bRca initial inverse: col0=[{:.3f},{:.3f},{:.3f}] col1=[{:.3f},{:.3f},{:.3f}]", targetBRca[0], targetBRca[1],
                targetBRca[2], targetBRca[4], targetBRca[5], targetBRca[6]);
        }

        float angularForceRatio = g_rockConfig.rockGrabAngularToLinearForceRatio;

        auto* angMotor = createPositionMotor(g_rockConfig.rockGrabAngularTau, g_rockConfig.rockGrabAngularDamping, g_rockConfig.rockGrabAngularProportionalRecovery,
            g_rockConfig.rockGrabAngularConstantRecovery, -maxForce / angularForceRatio, maxForce / angularForceRatio);

        auto* linMotor = createPositionMotor(tau, damping, proportionalRecovery, constantRecovery, -maxForce, maxForce);

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
                *reinterpret_cast<void**>(ragAtom + 0x40 + i * 8) = angMotor;
            }
        }
        {
            const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };
            for (int axis = 0; axis < 3; axis++) {
                *reinterpret_cast<void**>(header + linAtomOffsets[axis] + 0x10) = linMotor;
            }
        }

        /*
         * HIGGS enables grab motors after building the constraint instance but
         * before adding it to the Havok world. FO4VR hknp's CreateConstraint
         * performs world insertion in one call, so the atom enabled flags must
         * be live before CreateConstraint builds solver runtime for the grab.
         */
        setGrabMotorAtomsActive(header, true);

        ROCK_LOG_DEBUG(GrabConstraint, "Motors attached and enabled before CreateConstraint: angular={:.0f}, linear={:.0f}", maxForce / angularForceRatio, maxForce);

        RE::hknpConstraintCinfo cinfo{};
        cinfo.constraintData = reinterpret_cast<RE::hkpConstraintData*>(cd);
        cinfo.bodyIdA = handBodyId.value;
        cinfo.bodyIdB = objectBodyId.value;

        std::uint32_t outId = 0x7FFF'FFFF;
        world->CreateConstraint(&outId, cinfo);

        if (outId == 0x7FFF'FFFF) {
            ROCK_LOG_ERROR(GrabConstraint, "CreateConstraint returned invalid ID");
            return result;
        }

        ROCK_LOG_DEBUG(GrabConstraint, "Constraint created: id={}, hand={}, obj={}", outId, handBodyId.value, objectBodyId.value);

        result.constraintId = outId;
        result.constraintData = cd;
        result.angularMotor = angMotor;
        result.linearMotor = linMotor;
        result.currentTau = tau;
        result.currentMaxForce = 0.0f;
        result.targetMaxForce = maxForce;

        return result;
    }

    void destroyGrabConstraint(RE::hknpWorld* world, ActiveConstraint& constraint)
    {
        if (!constraint.isValid())
            return;

        if (world) {
            std::uint32_t ids[1] = { constraint.constraintId };
            world->DestroyConstraints(ids, 1);
            ROCK_LOG_DEBUG(GrabConstraint, "Constraint {} destroyed", constraint.constraintId);
        }

        if (constraint.angularMotor) {
            havok_runtime::freeHavok(constraint.angularMotor, HK_POSITION_MOTOR_SIZE);
        }
        if (constraint.linearMotor && constraint.linearMotor != constraint.angularMotor) {
            havok_runtime::freeHavok(constraint.linearMotor, HK_POSITION_MOTOR_SIZE);
        }
        if (constraint.constraintData) {
            havok_runtime::freeHavok(constraint.constraintData, GRAB_CONSTRAINT_SIZE);
        }

        constraint.clear();
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

            float minI = (std::min)({ invI[0], invI[1], invI[2] });
            float maxI = (std::max)({ invI[0], invI[1], invI[2] });

            float MAX_INERTIA_RATIO = g_rockConfig.rockGrabMaxInertiaRatio;
            float ratio = maxI / minI;

            if (ratio > MAX_INERTIA_RATIO) {
                float maxAllowed = minI * MAX_INERTIA_RATIO;
                for (int i = 0; i < 3; i++) {
                    if (invI[i] > maxAllowed) {
                        invI[i] = maxAllowed;
                    }
                }

                packed[0] = repackBfloat16(invI[0]);
                packed[1] = repackBfloat16(invI[1]);
                packed[2] = repackBfloat16(invI[2]);

                ROCK_LOG_DEBUG(GrabConstraint,
                    "Inertia ratio clamped: {:.1f}x -> {:.1f}x "
                    "packed [{},{},{}] -> [{},{},{}] float [{:.6e},{:.6e},{:.6e}]",
                    ratio, MAX_INERTIA_RATIO, savedMotionState.savedPackedInertia[0], savedMotionState.savedPackedInertia[1], savedMotionState.savedPackedInertia[2], packed[0],
                    packed[1], packed[2], invI[0], invI[1], invI[2]);
            } else {
                ROCK_LOG_DEBUG(GrabConstraint, "Inertia ratio OK ({:.1f}x), no clamping needed", ratio);
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
                VirtualFree(s_shellcodeBlocks[i], 0, MEM_RELEASE);
                s_shellcodeBlocks[i] = nullptr;
            }
        }
        s_shellcodeCount = 0;
        s_vtableBuilt = false;
        ROCK_LOG_DEBUG(GrabConstraint, "Vtable shellcode freed");
    }
}
