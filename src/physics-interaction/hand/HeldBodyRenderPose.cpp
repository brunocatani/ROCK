#include "physics-interaction/hand/HeldBodyRenderPose.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"

#include "RE/Havok/hknpBody.h"
#include "REL/Relocation.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>

namespace rock::held_body_render_pose
{
    namespace
    {
        /*
         * hknpWorld::setBodyTransform (immediate) — blind-verified 2026-08-17
         * (0x1415395E0): (world, bodyId, const hkTransform*, activation).
         * Copies the XYZ lanes of four aligned columns into body+0x00..0x3F,
         * derives/normalizes the quaternion, propagates into motion/spatial
         * state and the attached-body chain, dispatches body-change
         * listeners, preserves velocities, and no-ops inside a global
         * tolerance. Caller invariants: world-write execution context and a
         * valid live body id.
         */
        using SetBodyTransform_t = void (*)(RE::hknpWorld*, std::uint32_t, const float*, std::int32_t);

        // hkTransform: rows of the Ni-stored axis matrix map directly onto
        // the body's 4-float axis blocks (see TransformMath.h note); no
        // transpose. W lanes are ignored by the setter (body W preserved).
        struct alignas(16) HkTransformBlocks
        {
            float f[16]{};
        };

        constexpr std::int32_t kActivationBehaviorNone = 0;
        // Anchor vs solver pose gate: the measured stick-sprint gap is
        // ~0.03 havok; a full havok unit (~70 gu) means teleport/recenter or
        // a stale target — fail closed for that step.
        constexpr float kMaxTargetVsSolverHavok = 1.0f;
        // Restore verification: the body must still hold ROCK's masquerade
        // write. Engine attached-body regeneration can nudge within noise;
        // anything larger means another writer owned the body (keyframe on
        // deactivate, teleport) and the restore must fail closed.
        constexpr float kMaxRestoreDriftHavok = 0.05f;

        struct HandSlot
        {
            std::mutex mutex;
            // target (game thread writer)
            bool targetValid = false;
            std::uint32_t targetBodyId = 0;
            HkTransformBlocks targetPose{};
            // masquerade (physics thread owner)
            bool active = false;
            std::uint32_t activeBodyId = 0;
            HkTransformBlocks writtenPose{};
            HkTransformBlocks savedSolverPose{};
        };

        HandSlot s_hands[2];
        std::atomic<std::uint32_t> s_applied{ 0 };
        std::atomic<std::uint32_t> s_restored{ 0 };
        std::atomic<std::uint32_t> s_skippedContended{ 0 };
        std::atomic<std::uint32_t> s_skippedInvalidBody{ 0 };
        std::atomic<std::uint32_t> s_skippedImplausible{ 0 };
        std::atomic<std::uint32_t> s_restoreMismatch{ 0 };

        HandSlot& slotFor(const bool isLeft)
        {
            return s_hands[isLeft ? 1 : 0];
        }

        RE::hknpBody* resolveLiveBody(RE::hknpWorld* world, const std::uint32_t bodyId)
        {
            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            if (!body || body->bodyId.value != bodyId) {
                return nullptr;
            }
            return body;
        }

        float translationDeltaHavok(const float* a, const float* b)
        {
            const float dx = a[12] - b[12];
            const float dy = a[13] - b[13];
            const float dz = a[14] - b[14];
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        void callSetBodyTransform(RE::hknpWorld* world, const std::uint32_t bodyId, const HkTransformBlocks& pose)
        {
            static REL::Relocation<SetBodyTransform_t> setBodyTransform{ REL::Offset(offsets::kFunc_HknpWorld_SetBodyTransform) };
            setBodyTransform(world, bodyId, pose.f, kActivationBehaviorNone);
        }

        void applyHandMasquerade(HandSlot& slot, RE::hknpWorld* world)
        {
            std::unique_lock lock(slot.mutex, std::try_to_lock);
            if (!lock.owns_lock()) {
                s_skippedContended.fetch_add(1, std::memory_order_relaxed);
                return;
            }
            if (!slot.targetValid) {
                return;
            }

            auto* body = resolveLiveBody(world, slot.targetBodyId);
            if (!body) {
                s_skippedInvalidBody.fetch_add(1, std::memory_order_relaxed);
                slot.targetValid = false;
                return;
            }

            HkTransformBlocks solverPose{};
            std::memcpy(solverPose.f, body, sizeof(solverPose.f));
            if (translationDeltaHavok(slot.targetPose.f, solverPose.f) > kMaxTargetVsSolverHavok) {
                s_skippedImplausible.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            slot.savedSolverPose = solverPose;
            callSetBodyTransform(world, slot.targetBodyId, slot.targetPose);
            slot.writtenPose = slot.targetPose;
            slot.activeBodyId = slot.targetBodyId;
            slot.active = true;
            s_applied.fetch_add(1, std::memory_order_relaxed);
        }

        void restoreHandMasquerade(HandSlot& slot, RE::hknpWorld* world)
        {
            std::unique_lock lock(slot.mutex, std::try_to_lock);
            if (!lock.owns_lock()) {
                // The restore must not be silently lost: without it the next
                // solve consumes the anchor pose. Contention here is a
                // game-thread publish/clear racing the step start; block
                // briefly rather than corrupt the solver input.
                lock = std::unique_lock(slot.mutex);
            }
            if (!slot.active) {
                return;
            }
            slot.active = false;

            auto* body = resolveLiveBody(world, slot.activeBodyId);
            if (!body) {
                s_skippedInvalidBody.fetch_add(1, std::memory_order_relaxed);
                return;
            }

            HkTransformBlocks currentPose{};
            std::memcpy(currentPose.f, body, sizeof(currentPose.f));
            if (translationDeltaHavok(currentPose.f, slot.writtenPose.f) > kMaxRestoreDriftHavok) {
                // Another writer moved the body inside the masquerade window;
                // its pose is now authoritative. Restoring the stale solver
                // pose would fight it — fail closed and drop.
                s_restoreMismatch.fetch_add(1, std::memory_order_relaxed);
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "held-body render pose: restore skipped, body {} rewritten inside masquerade window (drift {:.3f} havok)",
                    slot.activeBodyId,
                    translationDeltaHavok(currentPose.f, slot.writtenPose.f));
                return;
            }

            callSetBodyTransform(world, slot.activeBodyId, slot.savedSolverPose);
            s_restored.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void publishTarget(const bool isLeft, const std::uint32_t bodyId, const RE::NiTransform& anchorWorldGame, const float gameToHavok) noexcept
    {
        if (!std::isfinite(gameToHavok) || gameToHavok <= 0.0f) {
            return;
        }
        HkTransformBlocks pose{};
        pose.f[0] = anchorWorldGame.rotate.entry[0][0];
        pose.f[1] = anchorWorldGame.rotate.entry[0][1];
        pose.f[2] = anchorWorldGame.rotate.entry[0][2];
        pose.f[4] = anchorWorldGame.rotate.entry[1][0];
        pose.f[5] = anchorWorldGame.rotate.entry[1][1];
        pose.f[6] = anchorWorldGame.rotate.entry[1][2];
        pose.f[8] = anchorWorldGame.rotate.entry[2][0];
        pose.f[9] = anchorWorldGame.rotate.entry[2][1];
        pose.f[10] = anchorWorldGame.rotate.entry[2][2];
        pose.f[12] = anchorWorldGame.translate.x * gameToHavok;
        pose.f[13] = anchorWorldGame.translate.y * gameToHavok;
        pose.f[14] = anchorWorldGame.translate.z * gameToHavok;
        for (int i = 0; i < 15; ++i) {
            if (!std::isfinite(pose.f[i])) {
                return;
            }
        }

        auto& slot = slotFor(isLeft);
        std::scoped_lock lock(slot.mutex);
        slot.targetBodyId = bodyId;
        slot.targetPose = pose;
        slot.targetValid = true;
    }

    void invalidateTarget(const bool isLeft) noexcept
    {
        auto& slot = slotFor(isLeft);
        std::scoped_lock lock(slot.mutex);
        slot.targetValid = false;
    }

    void clearWithoutRestore(const bool isLeft) noexcept
    {
        auto& slot = slotFor(isLeft);
        std::scoped_lock lock(slot.mutex);
        slot.targetValid = false;
        slot.active = false;
    }

    void applyAtFinalSubstepPostSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing) noexcept
    {
        if (!world || timing.substepCount == 0 || timing.substepIndex + 1 != timing.substepCount) {
            return;
        }
        applyHandMasquerade(s_hands[0], world);
        applyHandMasquerade(s_hands[1], world);
    }

    void restoreBeforeWholeStep(RE::hknpWorld* world) noexcept
    {
        if (!world) {
            return;
        }
        restoreHandMasquerade(s_hands[0], world);
        restoreHandMasquerade(s_hands[1], world);
    }

    bool tryGetSolverPoseOverride(const std::uint32_t bodyId, const float havokToGame, RE::NiTransform& outBodyWorldGame) noexcept
    {
        if (!std::isfinite(havokToGame) || havokToGame <= 0.0f) {
            return false;
        }
        for (auto& slot : s_hands) {
            std::unique_lock lock(slot.mutex, std::try_to_lock);
            if (!lock.owns_lock()) {
                continue;
            }
            if (!slot.active || slot.activeBodyId != bodyId) {
                continue;
            }
            const float* f = slot.savedSolverPose.f;
            outBodyWorldGame.rotate = transform_math::hknpBodyColumnsToNiStoredAxes<RE::NiMatrix3>(f);
            outBodyWorldGame.translate = { f[12] * havokToGame, f[13] * havokToGame, f[14] * havokToGame };
            outBodyWorldGame.scale = 1.0f;
            return true;
        }
        return false;
    }

    void copyStatus(MasqueradeStatus& out) noexcept
    {
        out.appliedSteps = s_applied.load(std::memory_order_relaxed);
        out.restoredSteps = s_restored.load(std::memory_order_relaxed);
        out.skippedContended = s_skippedContended.load(std::memory_order_relaxed);
        out.skippedInvalidBody = s_skippedInvalidBody.load(std::memory_order_relaxed);
        out.skippedImplausible = s_skippedImplausible.load(std::memory_order_relaxed);
        out.restoreMismatch = s_restoreMismatch.load(std::memory_order_relaxed);
        out.activeNow = 0;
        for (int i = 0; i < 2; ++i) {
            std::unique_lock lock(s_hands[i].mutex, std::try_to_lock);
            if (lock.owns_lock() && s_hands[i].active) {
                out.activeNow |= i == 0 ? 1u : 2u;
            }
        }
    }
}
