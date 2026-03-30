#pragma once

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <mutex>

namespace RE
{
    class hknpWorld;
}

namespace rock
{
    namespace native_mouse_spring_grab
    {
        /*
         * The native FO4VR mouse spring is used only for the close held-object
         * phase. ROCK freezes the visible object relation in root-flattened hand
         * space, then crosses the native action boundary with the BODY frame that
         * the action and held visual node actually follow. MOTION/COM stays
         * outside this wrapper as diagnostics; native rotation packing stays
         * isolated here.
         */
        inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
        inline constexpr std::uint32_t kNativeGrabBodyFlags = 0x08000000u;

        inline float sanitizeTuningScale(float value)
        {
            return std::isfinite(value) ? std::clamp(value, 0.05f, 4.0f) : 1.0f;
        }

        inline float composeTuningScale(float baseScale, float multiplier)
        {
            return sanitizeTuningScale(baseScale * multiplier);
        }

        struct Tuning
        {
            float angularClampScale = 1.0f;
            float linearResponseScale = 1.0f;
            float angularResponseScale = 1.0f;
            std::uint32_t bodyFlags = kNativeGrabBodyFlags;
        };

        struct DebugState
        {
            void* action = nullptr;
            RE::hknpBodyId bodyId{ kInvalidBodyId };
            RE::NiTransform targetBodyWorldGame{};
            RE::NiPoint3 targetPointWorldGame{};
            RE::NiPoint3 localGrabPointBodyGame{};
            float lastFlushDeltaSeconds = 0.0f;
            std::uint64_t queuedTargets = 0;
            std::uint64_t flushedTargets = 0;
            std::uint64_t failedFlushes = 0;
            bool hasTarget = false;
        };

        RE::NiPoint3 computeTargetPointWorldGame(const RE::NiTransform& targetBodyWorldGame, const RE::NiPoint3& localGrabPointBodyGame);
    }

    class NativeMouseSpringGrab
    {
    public:
        NativeMouseSpringGrab() = default;
        NativeMouseSpringGrab(const NativeMouseSpringGrab&) = delete;
        NativeMouseSpringGrab& operator=(const NativeMouseSpringGrab&) = delete;
        ~NativeMouseSpringGrab();

        bool create(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            const RE::NiTransform& initialTargetBodyWorldGame,
            const RE::NiPoint3& localGrabPointBodyGame,
            const native_mouse_spring_grab::Tuning& tuning = {});

        void destroy(RE::hknpWorld* world = nullptr, bool restoreBodyFlags = true);
        void clear();

        bool queueTarget(const RE::NiTransform& targetBodyWorldGame);
        bool flush(RE::hknpWorld* world, float deltaSeconds);

        bool isValid() const;
        RE::hknpBodyId bodyId() const;
        void* action() const;
        bool hasTarget() const;
        RE::NiPoint3 targetPointWorldGame() const;
        native_mouse_spring_grab::DebugState debugState() const;

    private:
        bool isValidUnlocked() const;
        void destroyUnlocked(RE::hknpWorld* world, bool restoreBodyFlags);
        void clearUnlocked();

        mutable std::mutex _mutex;
        void* _action = nullptr;
        RE::hknpBodyId _bodyId{ native_mouse_spring_grab::kInvalidBodyId };
        RE::NiTransform _targetBodyWorldGame{};
        RE::NiPoint3 _targetPointWorldGame{};
        RE::NiPoint3 _localGrabPointBodyGame{};
        std::uint32_t _bodyFlags = 0;
        float _lastFlushDeltaSeconds = 0.0f;
        std::uint64_t _queuedTargets = 0;
        std::uint64_t _flushedTargets = 0;
        std::uint64_t _failedFlushes = 0;
        bool _hasTarget = false;
    };
}
