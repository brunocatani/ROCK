#pragma once

#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiTransform.h"

#include <cstdint>

namespace RE
{
    class bhkWorld;
}

namespace rock::grab_authority_phase0
{
    /*
     * Phase 0 exists to validate the future custom dynamic grab authority
     * surface without changing active gameplay grabs. It owns isolated hidden
     * no-contact test bodies, records the actual physics-step setter behavior,
     * and can be deleted without touching HandGrab or the current native
     * mouse-spring held-object path.
     */
    enum class ProxyFilterPolicy : int
    {
        NonCollidableLayerPlusNoCollideBit = 0,
        RockBodyLayerPlusNoCollideBit = 1,
        RockHandLayerPlusNoCollideBit = 2,
    };

    struct Config
    {
        bool enabled = false;
        bool solverProbeEnabled = true;
        int proxyFilterPolicy = static_cast<int>(ProxyFilterPolicy::NonCollidableLayerPlusNoCollideBit);
        int logIntervalFrames = 30;
        float motionAmplitudeGameUnits = 8.0f;
    };

    class Probe
    {
    public:
        void updateGameFrame(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const Config& config);
        void driveBetweenCollideAndSolve(RE::hknpWorld* hknpWorld, const havok_physics_timing::PhysicsTimingSample& timing, const Config& config);
        void observeAfterAny(RE::hknpWorld* hknpWorld, const havok_physics_timing::PhysicsTimingSample& timing, const Config& config);
        void noteSemanticContactBodyIds(std::uint32_t rightHandBodyId, std::uint32_t leftHandBodyId, std::uint32_t weaponBodyId);
        void shutdown(RE::bhkWorld* fallbackBhkWorld);
        void abandon();

        [[nodiscard]] bool hasBodies() const noexcept { return _proxyBody.isValid() || _receiverBody.isValid(); }
        [[nodiscard]] std::uint32_t proxyBodyId() const noexcept { return _proxyBody.isValid() ? _proxyBody.getBodyId().value : kInvalidBodyId; }
        [[nodiscard]] std::uint32_t receiverBodyId() const noexcept { return _receiverBody.isValid() ? _receiverBody.getBodyId().value : kInvalidBodyId; }

    private:
        static constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

        bool createBodies(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const Config& config);
        void destroyBodies(RE::bhkWorld* fallbackBhkWorld);
        bool bodyIdMatchesSemanticContact(std::uint32_t bodyId) const noexcept;

        BethesdaPhysicsBody _proxyBody;
        BethesdaPhysicsBody _receiverBody;
        ActiveConstraint _solverConstraint;
        RE::bhkWorld* _bhkWorld = nullptr;
        RE::hknpWorld* _hknpWorld = nullptr;
        RE::NiTransform _previousProxyTarget{};
        RE::NiTransform _lastProxyTarget{};
        bool _hasPreviousProxyTarget = false;
        bool _activeSolverProbeEnabled = true;
        int _activeFilterPolicy = static_cast<int>(ProxyFilterPolicy::NonCollidableLayerPlusNoCollideBit);
        int _createRetryFrames = 0;
        std::uint64_t _driveSequence = 0;
        int _betweenLogCounter = 0;
        int _afterSolveLogCounter = 0;
        std::uint32_t _lastSemanticRightHandBodyId = 0xFFFF'FFFFu;
        std::uint32_t _lastSemanticLeftHandBodyId = 0xFFFF'FFFFu;
        std::uint32_t _lastSemanticWeaponBodyId = 0xFFFF'FFFFu;
    };
}
