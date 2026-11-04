#include "physics-interaction/visual/AmmoCounters.h"

#include <cstdio>

#include <RPS_UI_Framework/NumberOverlay.h>
#include "physics-interaction/visual/RpsUiQuery.h"
#include "physics-interaction/PhysicsLog.h"

namespace rock::ammo_counters
{
    namespace
    {
        using rpsui::api::Result;

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
        // The HUD styles draw the whole counter on the loaded overlay.
        std::array<Overlay, 4> g_overlays{ {
            { "rock.ammo.right.loaded", "Right hand ammo counter" },
            { "rock.ammo.right.reserve", "Right hand reserve rounds" },
            { "rock.ammo.left.loaded", "Left hand ammo counter" },
            { "rock.ammo.left.reserve", "Left hand reserve rounds" },
        } };
        const rpsui::api::framework::Api* g_base{ nullptr };
        const rpsui::api::number_overlay::Api* g_numbers{ nullptr };
        std::uint64_t g_owner{ 0 };
        bool g_bound{ false };
        std::uint32_t g_retryCountdown{ 0 };
        bool g_absentReported{ false };
        Result g_reported{ Result::Ok };

        void report(const char* step, Result result) noexcept
        {
            if (result == g_reported)
                return;
            g_reported = result;
            ROCK_LOG_WARN(Weapon, "Ammo counters: RPS UI {} returned {}; the vanilla counter stays until it succeeds",
                step, static_cast<std::uint32_t>(result));
        }

        bool bind() noexcept
        {
            if (g_bound)
                return true;
            if (g_retryCountdown > 0) {
                --g_retryCountdown;
                return false;
            }
            g_retryCountdown = kBindRetryCalls;
            const auto query = rps_ui::query();
            if (!g_base)
                (void)rpsui::api::acquire(query, g_base);
            if (!g_numbers && g_base)
                (void)rpsui::api::acquire(query, g_numbers);
            if (!g_base || !g_numbers) {
                if (!g_absentReported) {
                    g_absentReported = true;
                    ROCK_LOG_INFO(Weapon,
                        "Ammo counters: RPS UI number overlays unavailable; the vanilla counter stays");
                }
                return false;
            }
            if (!g_base->isFrameworkReady())
                return false;
            if (g_owner == 0) {
                rpsui::api::framework::ConsumerRegistration registration{};
                std::snprintf(registration.consumerId, sizeof(registration.consumerId), "%s", "rock.ammo-counters");
                std::snprintf(registration.displayName, sizeof(registration.displayName), "%s", "ROCK ammo counters");
                rpsui::api::framework::ConsumerHandle handle{};
                if (const auto result = g_base->registerConsumer(&registration, &handle);
                    result != Result::Ok || handle.ownerToken == 0) {
                    report("consumer registration", result);
                    return false;
                }
                g_owner = handle.ownerToken;
            }
            for (auto& overlay : g_overlays) {
                if (overlay.handle)
                    continue;
                rpsui::api::number_overlay::Registration registration{};
                std::snprintf(registration.overlayId, sizeof(registration.overlayId), "%s", overlay.id);
                std::snprintf(registration.displayName, sizeof(registration.displayName), "%s", overlay.name);
                if (const auto result = g_numbers->registerOverlay(g_owner, &registration, &overlay.handle);
                    result != Result::Ok || overlay.handle == 0) {
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
            if (!overlay.open)
                return true;
            rpsui::api::number_overlay::Presentation presentation{};
            presentation.sequence = ++overlay.sequence;
            const auto result = g_numbers->submit(g_owner, overlay.handle, &presentation);
            if (result == Result::Ok || result == Result::PanelNotRegistered) {
                overlay.open = false;
                return true;
            }
            report("close", result);
            return false;
        }

        struct Look
        {
            rpsui::api::number_overlay::Style style{ rpsui::api::number_overlay::Style::Plain };
            std::array<float, 3> color{ 1.0f, 1.0f, 1.0f };
            std::uint32_t secondaryValue{ 0 };
        };

        bool show(Overlay& overlay, const ammo_counter_pose::Pose& pose, const ammo_counter_pose::Panel& panel,
            std::uint32_t value, const Look& look) noexcept
        {
            rpsui::api::number_overlay::Presentation presentation{};
            presentation.sequence = ++overlay.sequence;
            presentation.open = 1;
            presentation.value = value;
            presentation.opacity = pose.opacity;
            presentation.style = look.style;
            presentation.secondaryValue = look.secondaryValue;
            for (unsigned i = 0; i < 3; ++i)
                presentation.color[i] = look.color[i];
            for (unsigned i = 0; i < 3; ++i) {
                presentation.pose.center[i] = panel.center[i];
                presentation.pose.right[i] = pose.right[i];
                presentation.pose.up[i] = pose.up[i];
                presentation.pose.front[i] = pose.front[i];
            }
            presentation.pose.physicalWidth = panel.width;
            presentation.pose.physicalHeight = panel.height;
            const auto result = g_numbers->submit(g_owner, overlay.handle, &presentation);
            if (result != Result::Ok) {
                report("presentation", result);
                return false;
            }
            overlay.open = true;
            return true;
        }
    }

    bool update(const std::array<HandCounter, 2>& hands, const Style style,
        const std::array<float, 3>& hudColor) noexcept
    {
        if (!bind())
            return false;
        bool healthy = true;
        for (std::size_t hand = 0; hand < hands.size(); ++hand) {
            const auto& counter = hands[hand];
            auto& loaded = g_overlays[hand * 2];
            auto& reserve = g_overlays[hand * 2 + 1];
            if (counter.shown && counter.pose.valid && counter.pose.opacity > 0.0f) {
                if (style == Style::Minimal) {
                    healthy &= show(loaded, counter.pose, counter.pose.loaded, counter.loaded, {});
                    healthy &= show(reserve, counter.pose, counter.pose.reserve, counter.reserve, {});
                } else {
                    const Look look{
                        .style = style == Style::Immersive ? rpsui::api::number_overlay::Style::HudBracketed
                                                           : rpsui::api::number_overlay::Style::Hud,
                        .color = hudColor,
                        .secondaryValue = counter.reserve,
                    };
                    healthy &= show(loaded, counter.pose, counter.pose.counter, counter.loaded, look);
                    healthy &= close(reserve);
                }
            } else {
                healthy &= close(loaded);
                healthy &= close(reserve);
            }
        }
        if (healthy)
            g_reported = Result::Ok;
        return healthy;
    }

    void hide() noexcept
    {
        if (!g_bound)
            return;
        for (auto& overlay : g_overlays)
            (void)close(overlay);
    }
}
