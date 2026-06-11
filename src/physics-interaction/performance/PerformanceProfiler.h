/*
 * ROCK performance profiling is diagnostics-only because the interaction stack
 * must not let wall-clock timing influence gameplay state. The alternative was
 * to scatter timer calls directly through hand, body, weapon, and overlay code;
 * centralizing the boundary here keeps the high-resolution clock isolated in
 * the implementation file while still giving future optimization passes
 * measured subsystem costs.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::performance_profiler
{
    enum class Scope : std::uint8_t
    {
        FrameUpdate = 0,
        HandColliderUpdate,
        BodyColliderUpdate,
        GeneratedColliderPhysicsFlush,
        WeaponCollision,
        WeaponCollisionTransforms,
        GeneratedBodyContactRegistry,
        WeaponColliderBuild,
        WeaponColliderCreate,
        TwoHandedGripStart,
        SupportGripSuppression,
        SelectionCasts,
        SoftContact,
        DebugOverlayPublish,
        DebugOverlayRender,
        ContactResolve,
        NativeContactCallback,
        Count
    };

    enum class Counter : std::uint8_t
    {
        WeaponRebuildQueued = 0,
        WeaponRebuildCanceled,
        WeaponRebuildCompleted,
        WeaponRebuildVisualRootDeferred,
        WeaponRebuildVisualStableWait,
        WeaponRebuildVisualSourceUnavailableRetained,
        WeaponRebuildReasonSettingsChanged,
        WeaponRebuildReasonDriveRequested,
        WeaponRebuildReasonKeyChanged,
        WeaponRebuildReasonMissingBodies,
        WeaponKeyChangeVisualOnly,
        WeaponKeyChangeIdentityOnly,
        WeaponKeyChangeVisualAndIdentity,
        Count
    };

    enum class ValueMetric : std::uint8_t
    {
        WeaponBuildVisibleTriShapes = 0,
        WeaponBuildGeneratedSources,
        WeaponBuildBodiesCreated,
        WeaponBuildTransientReloadSources,
        WeaponBuildBodyCount,
        Count
    };

    inline constexpr std::size_t kOverlayMaxLines = 8;
    inline constexpr std::size_t kOverlayLineLength = 128;
    using OverlayLines = std::array<std::array<char, kOverlayLineLength>, kOverlayMaxLines>;

    void refreshSettings(bool enabled, int logIntervalFrames, int warmupFrames, bool overlayTextEnabled) noexcept;
    void beginFrame() noexcept;
    void endFrame() noexcept;
    void addEventCount(Scope scope, std::uint64_t count = 1) noexcept;
    void addCounter(Counter counter, std::uint64_t count = 1) noexcept;
    void observeValue(ValueMetric metric, std::uint64_t value) noexcept;
    bool overlayTextEnabled() noexcept;
    std::uint32_t copyOverlayLines(OverlayLines& outLines) noexcept;

    class ScopedTimer
    {
    public:
        explicit ScopedTimer(Scope scope) noexcept;
        ~ScopedTimer();

        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;
        ScopedTimer(ScopedTimer&&) = delete;
        ScopedTimer& operator=(ScopedTimer&&) = delete;

        void stop() noexcept;

    private:
        Scope _scope{ Scope::Count };
        std::uint64_t _startTicks{ 0 };
        bool _active{ false };
    };

    class FrameScope
    {
    public:
        FrameScope() noexcept;
        ~FrameScope();

        FrameScope(const FrameScope&) = delete;
        FrameScope& operator=(const FrameScope&) = delete;
        FrameScope(FrameScope&&) = delete;
        FrameScope& operator=(FrameScope&&) = delete;

    private:
        ScopedTimer _timer;
    };
}
