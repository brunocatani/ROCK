#include "physics-interaction/visual/AmmoCounters.h"

#include <cstdio>

#include "physics-interaction/PhysicsLog.h"
#include "RPSUINumberOverlayApi.h"

namespace rock::ammo_counters
{
    namespace
    {
        using rpsui::sdk::ResultV1;

        // About one attempt per second at 90 Hz while RPS UI is absent or not ready.
        constexpr std::uint32_t kBindRetryCalls = 90;

        struct Overlay
        {
            const char* id;
            const char* name;
            std::uint64_t handle{ 0 };
            std::uint64_t sequence{ 0 };
            bool open{ false };
        };

        // [hand * 2 + 0] loaded, [hand * 2 + 1] reserve; hand 0 right, 1 left.
        std::array<Overlay, 4> g_overlays{ {
            { "rock.ammo.right.loaded", "Right hand loaded rounds" },
            { "rock.ammo.right.reserve", "Right hand reserve rounds" },
            { "rock.ammo.left.loaded", "Left hand loaded rounds" },
            { "rock.ammo.left.reserve", "Left hand reserve rounds" },
        } };
        const rpsui::sdk::ApiV1* g_base{ nullptr };
        const rpsui::sdk::NumberOverlayApiV1* g_numbers{ nullptr };
        std::uint64_t g_owner{ 0 };
        bool g_bound{ false };
        std::uint32_t g_retryCountdown{ 0 };
        bool g_absentReported{ false };
        ResultV1 g_reported{ ResultV1::Ok };

        void report(const char* step, ResultV1 result) noexcept
        {
            if (result == g_reported) return;
            g_reported = result;
            ROCK_LOG_WARN(Weapon, "Ammo counters: RPS UI {} returned {}; the vanilla counter stays until it succeeds",
                step, static_cast<std::uint32_t>(result));
        }

        bool bind() noexcept
        {
            if (g_bound) return true;
            if (g_retryCountdown > 0) {
                --g_retryCountdown;
                return false;
            }
            g_retryCountdown = kBindRetryCalls;
            if (!g_base) g_base = rpsui::sdk::RequestApiV1();
            if (!g_numbers && g_base) g_numbers = rpsui::sdk::RequestNumberOverlayApiV1();
            if (!g_base || !g_numbers) {
                if (!g_absentReported) {
                    g_absentReported = true;
                    ROCK_LOG_INFO(Weapon, "Ammo counters: RPS UI number overlays unavailable; the vanilla counter stays");
                }
                return false;
            }
            if (!g_base->isFrameworkReady || !g_base->isFrameworkReady()) return false;
            if (g_owner == 0) {
                rpsui::sdk::ConsumerRegistrationV1 registration{};
                std::snprintf(registration.consumerId, sizeof(registration.consumerId), "%s", "rock.ammo-counters");
                std::snprintf(registration.displayName, sizeof(registration.displayName), "%s", "ROCK ammo counters");
                rpsui::sdk::ConsumerHandleV1 handle{};
                if (const auto result = g_base->registerConsumer(&registration, &handle);
                    result != ResultV1::Ok || handle.ownerToken == 0) {
                    report("consumer registration", result);
                    return false;
                }
                g_owner = handle.ownerToken;
            }
            for (auto& overlay : g_overlays) {
                if (overlay.handle) continue;
                rpsui::sdk::NumberOverlayRegistrationV1 registration{};
                std::snprintf(registration.overlayId, sizeof(registration.overlayId), "%s", overlay.id);
                std::snprintf(registration.displayName, sizeof(registration.displayName), "%s", overlay.name);
                if (const auto result = g_numbers->registerOverlay(g_owner, &registration, &overlay.handle);
                    result != ResultV1::Ok || overlay.handle == 0) {
                    overlay.handle = 0;
                    report("overlay registration", result);
                    return false;
                }
            }
            g_bound = true;
            ROCK_LOG_INFO(Weapon, "Ammo counters connected to RPS UI; they replace the vanilla counter");
            return true;
        }

        bool close(Overlay& overlay) noexcept
        {
            if (!overlay.open) return true;
            rpsui::sdk::NumberOverlayPresentationV1 presentation{};
            presentation.sequence = ++overlay.sequence;
            const auto result = g_numbers->submit(g_owner, overlay.handle, &presentation);
            if (result == ResultV1::Ok || result == ResultV1::PanelNotRegistered) {
                overlay.open = false;
                return true;
            }
            report("close", result);
            return false;
        }

        bool show(Overlay& overlay, const ammo_counter_pose::Pose& pose, const ammo_counter_pose::Panel& panel,
            std::uint32_t value) noexcept
        {
            rpsui::sdk::NumberOverlayPresentationV1 presentation{};
            presentation.sequence = ++overlay.sequence;
            presentation.open = 1;
            presentation.value = value;
            presentation.opacity = pose.opacity;
            for (unsigned i = 0; i < 3; ++i) {
                presentation.pose.center[i] = panel.center[i];
                presentation.pose.right[i] = pose.right[i];
                presentation.pose.up[i] = pose.up[i];
                presentation.pose.front[i] = pose.front[i];
            }
            presentation.pose.physicalWidth = panel.width;
            presentation.pose.physicalHeight = panel.height;
            const auto result = g_numbers->submit(g_owner, overlay.handle, &presentation);
            if (result != ResultV1::Ok) {
                report("presentation", result);
                return false;
            }
            overlay.open = true;
            return true;
        }
    }

    bool update(const std::array<HandCounter, 2>& hands) noexcept
    {
        if (!bind()) return false;
        bool healthy = true;
        for (std::size_t hand = 0; hand < hands.size(); ++hand) {
            const auto& counter = hands[hand];
            auto& loaded = g_overlays[hand * 2];
            auto& reserve = g_overlays[hand * 2 + 1];
            if (counter.shown && counter.pose.valid && counter.pose.opacity > 0.0f) {
                healthy &= show(loaded, counter.pose, counter.pose.loaded, counter.loaded);
                healthy &= show(reserve, counter.pose, counter.pose.reserve, counter.reserve);
            } else {
                healthy &= close(loaded);
                healthy &= close(reserve);
            }
        }
        if (healthy) g_reported = ResultV1::Ok;
        return healthy;
    }

    void hide() noexcept
    {
        if (!g_bound) return;
        for (auto& overlay : g_overlays) (void)close(overlay);
    }
}
