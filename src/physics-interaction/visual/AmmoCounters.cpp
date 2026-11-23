#include "physics-interaction/visual/AmmoCounters.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <RPS_UI_Framework/NumberOverlay.h>
#include "physics-interaction/visual/RpsUiQuery.h"
#include "physics-interaction/PhysicsLog.h"

static_assert(rock::ammo_counter_pose::kLabelNumberFill == rpsui::api::number_overlay::kPlainLabelNumberFill);

namespace rock::ammo_counters
{
    namespace
    {
        using rpsui::api::Result;

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
        // The bound NumberOverlay draws labels (minor 1 or later).
        bool g_labels{ false };
        std::uint64_t g_owner{ 0 };
        bool g_bound{ false };
        rps_ui::BindGate g_bindGate;
        bool g_absentReported{ false };
        Result g_reported{ Result::Ok };

        void report(const char* step, Result result) noexcept
        {
            g_bindGate.refused();
            if (result == g_reported)
                return;
            g_reported = result;
            ROCK_LOG_WARN(Weapon, "Ammo counters: RPS UI {} returned {}; the vanilla counter stays until it succeeds",
                step, static_cast<std::uint32_t>(result));
        }

        /*
         * NumberOverlay at minor 0, so the counters also run with RPS UI 2.0;
         * the minor the host reports decides whether labels are sent. The
         * checks are Client.h acquire()'s with the requested minor.
         */
        const rpsui::api::number_overlay::Api* acquireNumbers(const rpsui::api::QueryInterfaceV1 query) noexcept
        {
            using Table = rpsui::api::number_overlay::Api;
            const rpsui::api::InterfaceDescriptorV1* descriptor{};
            if (query(Table::interfaceId, Table::majorVersion, 0, sizeof(Table), &descriptor) != Result::Ok || !descriptor ||
                descriptor->size != sizeof(rpsui::api::InterfaceDescriptorV1) || descriptor->interfaceId != Table::interfaceId ||
                descriptor->major != Table::majorVersion || descriptor->tableByteSize < sizeof(Table) ||
                descriptor->requiredFrameworkMajor != rpsui::api::framework::kMajor ||
                descriptor->requiredFrameworkMinor > rpsui::api::framework::kMinor || !descriptor->table)
                return nullptr;
            g_labels = descriptor->minor >= 1;
            return static_cast<const Table*>(descriptor->table);
        }

        bool bind() noexcept
        {
            if (g_bound)
                return true;
            const auto reportUnavailable = []() noexcept {
                if (!g_absentReported) {
                    g_absentReported = true;
                    ROCK_LOG_INFO(Weapon,
                        "Ammo counters: RPS UI number overlays unavailable; the vanilla counter stays");
                }
            };
            const auto query = g_bindGate.due();
            if (!query) {
                if (g_bindGate.absent())
                    reportUnavailable();
                return false;
            }
            if (!g_base)
                (void)rpsui::api::acquire(query, g_base);
            if (!g_numbers && g_base)
                g_numbers = acquireNumbers(query);
            if (!g_base || !g_numbers) {
                g_bindGate.refused();
                reportUnavailable();
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
            ROCK_LOG_INFO(Weapon, "Ammo counters connected to RPS UI; they replace the vanilla counter{}",
                g_labels ? " and name the caliber" : ", without the caliber (it needs RPS UI NumberOverlay 2.1)");
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
            rpsui::api::number_overlay::ChamberMarker chamber{ rpsui::api::number_overlay::ChamberMarker::None };
        };

        [[nodiscard]] rpsui::api::number_overlay::ChamberMarker overlayMarker(const api::weapon::ChamberMarker marker) noexcept
        {
            using Overlay = rpsui::api::number_overlay::ChamberMarker;
            switch (marker) {
            case api::weapon::ChamberMarker::Empty:
                return Overlay::Empty;
            case api::weapon::ChamberMarker::Chambered:
                return Overlay::Chambered;
            default:
                return Overlay::None;
            }
        }

        bool show(Overlay& overlay, const ammo_counter_pose::Pose& pose, const ammo_counter_pose::Panel& panel,
            std::uint32_t value, const Look& look, const char* label = nullptr) noexcept
        {
            rpsui::api::number_overlay::Presentation presentation{};
            presentation.sequence = ++overlay.sequence;
            presentation.open = 1;
            presentation.value = value;
            presentation.opacity = pose.opacity;
            presentation.style = look.style;
            presentation.secondaryValue = look.secondaryValue;
            presentation.chamber = look.chamber;
            if (g_labels && label) {
                const auto length = std::min(std::strlen(label), sizeof(presentation.label) - 1);
                std::memcpy(presentation.label, label, length);
            }
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

    Caliber caliberLabel(std::string_view name) noexcept
    {
        const auto isSpace = [](const char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; };
        const auto trim = [&](std::string_view value) {
            while (!value.empty() && isSpace(value.front()))
                value.remove_prefix(1);
            while (!value.empty() && isSpace(value.back()))
                value.remove_suffix(1);
            return value;
        };
        const auto lower = [](const char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c; };
        name = trim(name);
        for (const std::string_view word : { std::string_view{ "round" }, std::string_view{ "rounds" },
                 std::string_view{ "ammo" } }) {
            if (name.size() <= word.size() + 1 || !isSpace(name[name.size() - word.size() - 1]))
                continue;
            const auto tail = name.substr(name.size() - word.size());
            if (std::equal(tail.begin(), tail.end(), word.begin(), [&](char a, char b) { return lower(a) == b; })) {
                name = trim(name.substr(0, name.size() - word.size()));
                break;
            }
        }
        if (name.size() >= kCaliberCapacity) {
            // The last space among the first kCaliberCapacity characters ends
            // the last whole word that fits; without one the name is cut.
            const auto space = name.substr(0, kCaliberCapacity).find_last_of(" \t");
            name = space == std::string_view::npos || space == 0 ? name.substr(0, kCaliberCapacity - 1)
                                                                 : trim(name.substr(0, space));
        }
        Caliber label{};
        std::memcpy(label.data(), name.data(), name.size());
        return label;
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
                const auto chamber = overlayMarker(counter.chamber);
                if (style == Style::Minimal) {
                    const std::array<float, 3> color = counter.colorOverride ? counter.color : std::array{ 1.0f, 1.0f, 1.0f };
                    healthy &= show(loaded, counter.pose, counter.pose.loaded, counter.loaded, Look{ .color = color, .chamber = chamber });
                    healthy &= show(reserve, counter.pose, counter.pose.reserve, counter.reserve, Look{ .color = color }, counter.caliber);
                } else {
                    const Look look{
                        .style = style == Style::Immersive ? rpsui::api::number_overlay::Style::HudBracketed
                                                           : rpsui::api::number_overlay::Style::Hud,
                        .color = counter.colorOverride ? counter.color : hudColor,
                        .secondaryValue = counter.reserve,
                        .chamber = chamber,
                    };
                    healthy &= show(loaded, counter.pose, counter.pose.counter, counter.loaded, look, counter.caliber);
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

    bool labels() noexcept { return g_bound && g_labels; }
}
