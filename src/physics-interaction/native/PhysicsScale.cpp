#include "physics-interaction/native/PhysicsScale.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"
#include "f4vr/F4VRUtils.h"

#include <REL/Relocation.h>

#include <atomic>
#include <cmath>
#include <mutex>

namespace frik::rock::physics_scale
{
    namespace
    {
        std::mutex s_scaleMutex;
        std::atomic<float> s_gameToHavok{ kFallbackGameToHavok };
        std::atomic<float> s_havokToGame{ kFallbackHavokToGame };
        std::atomic<float> s_vrScale{ 0.0f };
        std::atomic<float> s_vrGlobalScale{ 0.0f };
        std::atomic<float> s_raycastScale{ 0.0f };
        std::atomic<std::uint32_t> s_revision{ 0 };
        std::atomic<bool> s_initialized{ false };

        float readGameToHavokScale()
        {
            static REL::Relocation<float*> scale{ REL::Offset(offsets::kData_HavokGameToHavokScale) };
            return scale.address() ? *scale : kFallbackGameToHavok;
        }

        float readHavokToGameScale()
        {
            static REL::Relocation<float*> scale{ REL::Offset(offsets::kData_HavokToGameScale) };
            return scale.address() ? *scale : kFallbackHavokToGame;
        }

        float readVrGlobalScale()
        {
            static REL::Relocation<float*> scale{ REL::Offset(offsets::kData_VRScalePrimary) };
            return scale.address() ? *scale : 0.0f;
        }

        float readRaycastScale()
        {
            static REL::Relocation<float*> scale{ REL::Offset(offsets::kData_RaycastResultScale) };
            return scale.address() ? *scale : 0.0f;
        }

        float readVrScaleSetting()
        {
            if (auto* setting = f4vr::getIniSetting("fVrScale:VR")) {
                return setting->GetFloat();
            }
            return 0.0f;
        }

        Snapshot readRuntimeSnapshot(std::uint32_t revision)
        {
            return makeSnapshot(readGameToHavokScale(), readHavokToGameScale(), readVrScaleSetting(), revision, readVrGlobalScale(), readRaycastScale());
        }

        bool vrDiagnosticsChanged(const Snapshot& a, const Snapshot& b)
        {
            return std::fabs(a.vrScale - b.vrScale) > 0.001f || std::fabs(a.vrGlobalScale - b.vrGlobalScale) > 0.001f ||
                   std::fabs(a.raycastScale - b.raycastScale) > 0.001f;
        }

        Snapshot loadCurrentSnapshot()
        {
            Snapshot snapshot{};
            snapshot.gameToHavok = s_gameToHavok.load(std::memory_order_relaxed);
            snapshot.havokToGame = s_havokToGame.load(std::memory_order_relaxed);
            snapshot.vrScale = s_vrScale.load(std::memory_order_relaxed);
            snapshot.vrGlobalScale = s_vrGlobalScale.load(std::memory_order_relaxed);
            snapshot.raycastScale = s_raycastScale.load(std::memory_order_relaxed);
            snapshot.revision = s_revision.load(std::memory_order_acquire);
            return snapshot;
        }

        void storeCurrentSnapshot(const Snapshot& snapshot)
        {
            s_gameToHavok.store(snapshot.gameToHavok, std::memory_order_relaxed);
            s_havokToGame.store(snapshot.havokToGame, std::memory_order_relaxed);
            s_vrScale.store(snapshot.vrScale, std::memory_order_relaxed);
            s_vrGlobalScale.store(snapshot.vrGlobalScale, std::memory_order_relaxed);
            s_raycastScale.store(snapshot.raycastScale, std::memory_order_relaxed);
            s_revision.store(snapshot.revision, std::memory_order_release);
            s_initialized.store(true, std::memory_order_release);
        }
    }

    Snapshot current()
    {
        return loadCurrentSnapshot();
    }

    float gameToHavok()
    {
        return s_gameToHavok.load(std::memory_order_relaxed);
    }

    float havokToGame()
    {
        return s_havokToGame.load(std::memory_order_relaxed);
    }

    std::uint32_t revision()
    {
        return s_revision.load(std::memory_order_acquire);
    }

    bool refreshAndLogIfChanged()
    {
        std::scoped_lock lock(s_scaleMutex);

        const bool hadSnapshot = s_initialized.load(std::memory_order_acquire);
        const Snapshot previous = loadCurrentSnapshot();
        Snapshot next = readRuntimeSnapshot(hadSnapshot ? previous.revision : 1);
        const bool conversionChanged = !hadSnapshot || !sameConversionScale(previous, next);
        const bool diagnosticsChanged = hadSnapshot && vrDiagnosticsChanged(previous, next);

        if (conversionChanged && hadSnapshot) {
            next.revision = previous.revision + 1;
        } else if (hadSnapshot) {
            next.revision = previous.revision;
        }

        storeCurrentSnapshot(next);

        if (conversionChanged) {
            const float reciprocal = isUsableScale(next.gameToHavok) ? (1.0f / next.gameToHavok) : 0.0f;
            const float reciprocalDrift = reciprocalDriftGameUnits(next);
            ROCK_LOG_INFO(Config,
                "Physics scale {}: gameToHavok={:.8f} havokToGame={:.5f} reciprocal={:.5f} reciprocalDrift={:.6f} fVrScale={:.3f} vrGlobal={:.3f} raycastScale={:.3f} revision={}",
                hadSnapshot ? "changed" : "initialized",
                next.gameToHavok,
                next.havokToGame,
                reciprocal,
                reciprocalDrift,
                next.vrScale,
                next.vrGlobalScale,
                next.raycastScale,
                next.revision);
            if (hasReciprocalMismatch(next, 0.001f)) {
                ROCK_LOG_WARN(Config,
                    "Physics scale globals disagree: gameToHavok reciprocal={:.6f}, havokToGame={:.6f}, drift={:.6f}. Check FO4VR scale offsets before trusting world-space collision diagnostics.",
                    reciprocal,
                    next.havokToGame,
                    reciprocalDrift);
            }
        } else if (diagnosticsChanged) {
            ROCK_LOG_DEBUG(Config,
                "Physics scale diagnostics changed without Havok conversion change: fVrScale={:.3f} vrGlobal={:.3f} raycastScale={:.3f} revision={}",
                next.vrScale,
                next.vrGlobalScale,
                next.raycastScale,
                next.revision);
        }

        return hadSnapshot && conversionChanged;
    }
}
