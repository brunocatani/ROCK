#include "physics-interaction/grab/GrabMotorTelemetry.h"

#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/telemetry/DynamicColliderTrace.h"

#include <REL/Relocation.h>
#include <algorithm>
#include <cstring>
#include <limits>
#include <spdlog/fmt/ranges.h>

namespace rock::grab_motor_telemetry
{
    namespace
    {
        // Native allocation 1417E39C0 and destruction 141555AC0 agree on
        // runtime +28 / size +26. World create/destroy and filter callbacks
        // independently use +128 and stride 38. No engine pointer is retained.
        struct NativeConstraint
        {
            std::uint32_t bodyA, bodyB;
            std::uintptr_t data;
            std::uint32_t id;
            std::uint16_t group;
            std::uint8_t flags, type;
            std::uintptr_t atoms;
            std::uint16_t atomBytes, schemaBytes;
            std::uint8_t results, temps;
            std::uint16_t runtimeBytes;
            std::uintptr_t runtime, userData;
        };
        static_assert(sizeof(NativeConstraint) == 0x38);
        static_assert(offsetof(NativeConstraint, runtime) == 0x28);
        static_assert(offsetof(NativeConstraint, runtimeBytes) == 0x26);

        struct MotionValues
        {
            std::array<float, 4> linear{}, angular{};
            std::array<std::int16_t, 4> inverseInertiaMass{};
        };

        bool readMotion(const RE::hknpMotion* motion, MotionValues& out)
        {
            return motion &&
                native_memory::guardedCopyFromMemory(&motion->linearVelocity, out.linear.data(), sizeof(out.linear)) &&
                native_memory::guardedCopyFromMemory(&motion->angularVelocity, out.angular.data(), sizeof(out.angular)) &&
                native_memory::guardedCopyFromMemory(motion->packedInverseInertia,
                    out.inverseInertiaMass.data(), sizeof(out.inverseInertiaMass));
        }

        const char* readConstraint(RE::hknpWorld* world, const ActiveConstraint& constraint,
            const Command& command, NativeConstraint& out)
        {
            if (!world || reinterpret_cast<std::uintptr_t>(world) != command.world || !constraint.isValid())
                return "world-or-owner";
            std::uintptr_t table = 0;
            std::int32_t count = 0;
            if (!native_memory::tryReadField(world, 0x128, table) ||
                !native_memory::tryReadField(world, 0x130, count)) return "constraint-table";
            if (table < 0x10000 || count <= 0 || constraint.constraintId >= static_cast<std::uint32_t>(count))
                return "constraint-range";
            const auto offset = static_cast<std::uintptr_t>(constraint.constraintId) * sizeof(NativeConstraint);
            if (table > std::numeric_limits<std::uintptr_t>::max() - offset - sizeof(NativeConstraint) ||
                !native_memory::tryReadValue(reinterpret_cast<const NativeConstraint*>(table + offset), out))
                return "constraint-record";
            if (out.id != constraint.constraintId || out.bodyA != command.bodyA || out.bodyB != command.bodyB ||
                out.data != reinterpret_cast<std::uintptr_t>(constraint.constraintData)) return "constraint-identity";
            if ((out.flags & 4u) != 0) return "constraint-disabled";
            if (out.atoms != out.data + ATOMS_START) return "atom-identity";
            if (out.atomBytes != ATOMS_SIZE) return "atom-size";
            if (out.results != RUNTIME_SOLVER_RESULTS) return "result-count";
            if (out.runtimeBytes != RUNTIME_REPORTED_SIZE) return "runtime-size";
            if (out.runtime < 0x10000 || (out.runtime & 0xfu) != 0) return "runtime-pointer";
            return nullptr;
        }

        const char* ownerName(Owner owner)
        {
            switch (owner) {
            case Owner::LeftHand: return "held-left";
            case Owner::Weapon: return "equipped";
            default: return "held-right";
            }
        }

        void flush(ActiveConstraint& constraint)
        {
            auto& state = constraint.motorTelemetry;
            if (state.samples == 0 && state.invalid == 0) return;
            auto* log = state.command.owner == Owner::Weapon ?
                dynamic_collider_trace::activeWeaponLogger() : dynamic_collider_trace::activeLogger();
            if (log) {
                try {
                    const auto& peak = state.peakGrip;
                    const auto& cmd = state.samples ? peak.command : state.command;
                    if (state.samples == 0) {
                        log->info("MOTOR_LOAD_UNAVAILABLE owner={} tag={:016X} constraint={} bodies={}/{} invalid={} failure={} windowDt={:.6f} source={} beforeSolve={}",
                            ownerName(cmd.owner), cmd.tag, constraint.constraintId, cmd.bodyA, cmd.bodyB,
                            state.invalid, state.failure, state.elapsed, cmd.source, cmd.beforeSolve);
                    } else {
                        log->info("MOTOR_LOAD owner={} tag={:016X} constraint={} bodies={}/{} samples={} invalid={} failure={} windowDt={:.6f} peakGrip={:.4f}gu peakRotation={:.4f}deg peakNetUtil={} nearNetLimitSeconds={} peakGripSolve={} source={} solverDt={:.6f} steps={}/{} mass={:.5f} invInertiaMass={} limits={} tau={} damping={} proportional={} constant={} impulse={} recoveryState={} averageEffort={} netUtil={} enabledAxes=0x{:02X} atPeakRotation={:.4f}deg proxyError={:.5f}gu commandContact={} contact={} teleported={} beforeLinear={} afterLinear={} beforeAngularLocal={} afterAngularLocal={}",
                        ownerName(cmd.owner), cmd.tag, constraint.constraintId, cmd.bodyA, cmd.bodyB,
                        state.samples, state.invalid, state.failure, state.elapsed,
                        peak.gripError, state.peakRotation, state.peakUtilization, state.nearLimitSeconds,
                        peak.solve, cmd.source, peak.solverSeconds, peak.solverSteps, peak.microSteps,
                        cmd.mass, cmd.inverseInertiaMass, cmd.limits, cmd.tau, cmd.damping,
                        cmd.proportional, cmd.constant, peak.load.impulse, peak.load.recoveryState,
                        peak.load.averageEffort, peak.load.netUtilization, peak.load.enabledAxes,
                        peak.rotationError, peak.proxyError, cmd.contact, peak.contact, peak.teleported,
                        cmd.beforeLinear, peak.afterLinear, cmd.beforeAngular, peak.afterAngular);
                    }
                } catch (...) {
                    dynamic_collider_trace::suppressAfterError();
                }
            }
            const auto command = state.command;
            state = {};
            state.command = command;
        }

        void reject(ActiveConstraint& constraint, const char* failure)
        {
            auto& state = constraint.motorTelemetry;
            ++state.invalid;
            state.failure = failure;
            if (state.elapsed >= 0.1f) flush(constraint);
        }
    }

    bool verifyLayout() noexcept
    {
        struct Signature { std::uintptr_t rva; std::array<std::uint8_t, 8> bytes; };
        constexpr Signature signatures[] = {
            { 0x17e3b2b, {0x49, 0x89, 0x5f, 0x28, 0x48, 0x8b, 0x5c, 0x24} },
            { 0x1555ad0, {0x48, 0x8b, 0x71, 0x28, 0x8b, 0xea, 0x48, 0x8b} },
            { 0x1a58f13, {0x49, 0x83, 0xc6, 0x10, 0x48, 0x83, 0xc7, 0x04} },
            { 0x18251a5, {0xf3, 0x41, 0x0f, 0x10, 0x45, 0x00, 0xf3, 0x41} },
            { 0x17e75c7, {0xf3, 0x0f, 0x11, 0x99, 0xb8, 0x00, 0x00, 0x00} },
            { 0x1afd998, {0xf3, 0x0f, 0x10, 0x41, 0x08, 0xf3, 0x0f, 0x59} },
        };
        const auto base = REL::Module::get().base();
        for (const auto& signature : signatures) {
            std::array<std::uint8_t, 8> live{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(base + signature.rva),
                    live.data(), live.size()) || live != signature.bytes) return false;
        }
        return true;
    }

    void capture(RE::hknpWorld* world, ActiveConstraint& constraint,
        std::uint32_t bodyA, std::uint32_t bodyB,
        const havok_physics_timing::PhysicsTimingSample& timing,
        std::uint64_t source, std::uint64_t tag, Owner owner, float mass, bool contact) noexcept
    {
        auto& state = constraint.motorTelemetry;
        if (!dynamic_collider_trace::motorOutputEnabled()) { state = {}; return; }
        auto& cmd = state.command;
        cmd = {};
        cmd.world = reinterpret_cast<std::uintptr_t>(world);
        cmd.bodyA = bodyA;
        cmd.bodyB = bodyB;
        cmd.beforeSolve = timing.solveSequence;
        cmd.source = source;
        cmd.tag = tag;
        cmd.owner = owner;
        cmd.mass = mass;
        cmd.contact = contact;
        if (!havok_physics_timing::tryGetDriveDeltaSeconds(timing, cmd.seconds)) {
            cmd.failure = "command-timing";
            return;
        }
        NativeConstraint native{};
        if (const char* failure = readConstraint(world, constraint, cmd, native)) {
            cmd.failure = failure;
            return;
        }
        cmd.runtimeIdentity = native.runtime;
        const auto body = havok_runtime::snapshotBody(world, RE::hknpBodyId{bodyB});
        MotionValues motion{};
        if (!body.valid || !readMotion(body.motion, motion)) {
            cmd.failure = "command-motion";
            return;
        }
        cmd.motion = body.motionIndex;
        std::copy_n(motion.linear.begin(), 3, cmd.beforeLinear.begin());
        std::copy_n(motion.angular.begin(), 3, cmd.beforeAngular.begin());
        for (std::size_t i = 0; i < 4; ++i) cmd.inverseInertiaMass[i] = unpackBfloat16(motion.inverseInertiaMass[i]);
        const HkPositionMotor* motors[] = {constraint.angularMotor, constraint.linearMotor};
        for (std::size_t i = 0; i < 2; ++i) {
            if (!motors[i] || motors[i]->type != 1) { cmd.failure = "command-motor"; return; }
            cmd.tau[i] = motors[i]->tau;
            cmd.damping[i] = motors[i]->damping;
            cmd.proportional[i] = motors[i]->proportionalRecoveryVelocity;
            cmd.constant[i] = motors[i]->constantRecoveryVelocity;
            for (std::size_t axis = 0; axis < 3; ++axis) cmd.limits[i * 3 + axis] = motors[i]->maxForce;
        }
        cmd.failure = "none";
        cmd.pending = true;
    }

    void record(RE::hknpWorld* world, ActiveConstraint& constraint,
        const havok_physics_timing::PhysicsTimingSample& timing,
        float gripError, float rotationError, float proxyError, bool contact, bool teleported) noexcept
    {
        auto& state = constraint.motorTelemetry;
        if (!dynamic_collider_trace::motorOutputEnabled()) { state = {}; return; }
        float seconds = 0.0f;
        if (!havok_physics_timing::tryGetDriveDeltaSeconds(timing, seconds)) {
            state.command.pending = false;
            reject(constraint, "solve-timing");
            return;
        }
        state.elapsed += seconds;
        const auto cmd = state.command;
        state.command.pending = false;
        if (!cmd.pending) {
            reject(constraint, std::strcmp(cmd.failure, "none") == 0 ?
                "no-pending-command" : cmd.failure);
            return;
        }
        if (!grab_motor_load::matchingSolve(cmd.beforeSolve, timing.solveSequence) ||
            std::abs(cmd.seconds - seconds) > 0.00001f) { reject(constraint, "solve-pair"); return; }
        NativeConstraint native{};
        if (const char* failure = readConstraint(world, constraint, cmd, native)) { reject(constraint, failure); return; }
        if (native.runtime != cmd.runtimeIdentity) { reject(constraint, "runtime-replaced"); return; }

        Sample sample{};
        sample.command = cmd;
        sample.solve = timing.solveSequence;
        sample.gripError = gripError;
        sample.rotationError = rotationError;
        sample.proxyError = proxyError;
        sample.contact = contact;
        sample.teleported = teleported;
        // setStepInfo 1417E7530 stores total T at solverInfo+C8, N at B4,
        // micro-count at BC. The builder uses T/(N*micro) for each bound;
        // solve resets scratch at the first iteration and exports after last.
        if (!native_memory::tryReadField(world, 0x3f8, sample.solverSeconds) ||
            !native_memory::tryReadField(world, 0x3e4, sample.solverSteps) ||
            !native_memory::tryReadField(world, 0x3ec, sample.microSteps) ||
            !std::isfinite(sample.solverSeconds) || std::abs(sample.solverSeconds - seconds) > 0.00001f ||
            sample.solverSteps == 0 || sample.microSteps == 0) { reject(constraint, "native-solver-timing"); return; }
        std::array<float, 24> runtime{};
        if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(native.runtime),
                runtime.data(), sizeof(runtime))) { reject(constraint, "runtime-results"); return; }
        sample.load = grab_motor_load::decode(runtime, cmd.limits, sample.solverSeconds);
        if (!sample.load.valid) { reject(constraint, "nonfinite-or-stale-results"); return; }
        const auto body = havok_runtime::snapshotBody(world, RE::hknpBodyId{cmd.bodyB});
        MotionValues motion{};
        if (!body.valid || body.motionIndex != cmd.motion || !readMotion(body.motion, motion)) {
            reject(constraint, "solve-motion");
            return;
        }
        std::copy_n(motion.linear.begin(), 3, sample.afterLinear.begin());
        std::copy_n(motion.angular.begin(), 3, sample.afterAngular.begin());
        if (!std::isfinite(gripError) || gripError < 0.0f || !std::isfinite(rotationError) || rotationError < 0.0f ||
            !std::isfinite(proxyError) || proxyError < 0.0f) { reject(constraint, "pose-readback"); return; }
        if (state.samples == 0 || gripError > state.peakGrip.gripError) state.peakGrip = sample;
        ++state.samples;
        state.peakRotation = (std::max)(state.peakRotation, rotationError);
        for (std::size_t axis = 0; axis < grab_motor_load::kAxisCount; ++axis) {
            state.peakUtilization[axis] = (std::max)(state.peakUtilization[axis], sample.load.netUtilization[axis]);
            // Near-unity net utilization is evidence of sustained load. Low
            // net utilization cannot rule out opposite impulses cancelling.
            if (sample.load.netUtilization[axis] >= 0.95f) state.nearLimitSeconds[axis] += seconds;
        }
        if (state.elapsed >= 0.1f) flush(constraint);
    }

    void finish(ActiveConstraint& constraint) noexcept
    {
        if (dynamic_collider_trace::motorOutputEnabled()) flush(constraint);
        constraint.motorTelemetry = {};
    }
}
