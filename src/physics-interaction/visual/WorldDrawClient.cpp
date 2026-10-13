#include "physics-interaction/visual/WorldDrawClient.h"

#include <atomic>
#include <cstdio>
#include <mutex>

#include "physics-interaction/PhysicsLog.h"
#include "RPSUIWorldDrawApi.h"

namespace rock::world_draw_client
{
    namespace
    {
        using rpsui::sdk::ResultV1;

        // About one attempt per second at 90 Hz while RPS UI is absent.
        constexpr std::uint32_t kBindRetryCalls = 90;

        std::mutex g_bindMutex;
        std::atomic<const rpsui::sdk::WorldDrawApiV1*> g_api{ nullptr };
        std::atomic<std::uint64_t> g_owner{ 0 };
        std::atomic<FrameHandler> g_handler{ nullptr };
        std::uint32_t g_retryCountdown{ 0 }; // Guarded by g_bindMutex.
        bool g_absentReported{ false };      // Guarded by g_bindMutex.
        bool g_refusalReported{ false };     // Guarded by g_bindMutex.

        void RPSUI_CALL onFrame(void*, std::uint64_t) noexcept
        {
            if (const auto handler = g_handler.load(std::memory_order_acquire)) handler();
        }

        void reportRefusal(const char* step, ResultV1 result) noexcept
        {
            if (g_refusalReported) return;
            g_refusalReported = true;
            ROCK_LOG_WARN(Hand, "RPS UI world drawing: {} refused ({}); ROCK draws nothing until it succeeds",
                step, static_cast<std::uint32_t>(result));
        }
    }

    bool ensureBound(const FrameHandler handler) noexcept
    {
        if (g_api.load(std::memory_order_acquire)) return true;
        std::scoped_lock lock(g_bindMutex);
        if (g_api.load(std::memory_order_acquire)) return true;
        if (g_retryCountdown > 0) {
            --g_retryCountdown;
            return false;
        }
        g_retryCountdown = kBindRetryCalls;

        const auto* registry = rpsui::sdk::RequestApiV1();
        const auto* draw = registry ? rpsui::sdk::RequestWorldDrawApiV1() : nullptr;
        if (!draw) {
            if (!g_absentReported) {
                g_absentReported = true;
                if (registry) {
                    ROCK_LOG_WARN(Hand, "RPS UI Framework does not offer world drawing v1; ROCK indicators and debug drawing stay hidden");
                } else {
                    ROCK_LOG_INFO(Hand, "RPS UI Framework is not loaded; ROCK indicators and debug drawing stay hidden until it is");
                }
            }
            return false;
        }

        auto owner = g_owner.load(std::memory_order_acquire);
        if (owner == 0) {
            rpsui::sdk::ConsumerRegistrationV1 registration{};
            std::snprintf(registration.consumerId, sizeof(registration.consumerId), "%s", "rock.world-draw");
            std::snprintf(registration.displayName, sizeof(registration.displayName), "%s", "ROCK world drawing");
            rpsui::sdk::ConsumerHandleV1 handle{};
            if (const auto result = registry->registerConsumer(&registration, &handle);
                result != ResultV1::Ok || handle.ownerToken == 0) {
                reportRefusal("consumer registration", result);
                return false;
            }
            owner = handle.ownerToken;
            g_owner.store(owner, std::memory_order_release);
        }

        // The owner stays registered for the process lifetime: the callback and
        // its handler are static, so RPS UI never calls into released state.
        g_handler.store(handler, std::memory_order_release);
        if (const auto result = draw->setFrameCallback(owner, &onFrame, nullptr); result != ResultV1::Ok) {
            reportRefusal("frame callback registration", result);
            return false;
        }
        g_api.store(draw, std::memory_order_release);
        ROCK_LOG_INFO(Hand, "RPS UI world drawing connected; ROCK indicators and debug drawing render through it");
        return true;
    }

    bool submit(const rpsui::sdk::WorldDrawPublicationV1& publication) noexcept
    {
        const auto* api = g_api.load(std::memory_order_acquire);
        return api && api->submit(g_owner.load(std::memory_order_acquire), &publication) == ResultV1::Ok;
    }

    bool registerMesh(const std::uint64_t meshId, const float* positions, const std::uint32_t vertexCount,
        const std::uint32_t* indices, const std::uint32_t indexCount) noexcept
    {
        const auto* api = g_api.load(std::memory_order_acquire);
        if (!api) return false;
        rpsui::sdk::WorldMeshV1 mesh{};
        mesh.positions = positions;
        mesh.vertexCount = vertexCount;
        mesh.indices = indices;
        mesh.indexCount = indexCount;
        return api->registerMesh(g_owner.load(std::memory_order_acquire), meshId, &mesh) == ResultV1::Ok;
    }

    void releaseMesh(const std::uint64_t meshId) noexcept
    {
        if (const auto* api = g_api.load(std::memory_order_acquire)) {
            (void)api->releaseMesh(g_owner.load(std::memory_order_acquire), meshId);
        }
    }
}
