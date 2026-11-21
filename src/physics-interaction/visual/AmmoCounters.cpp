#include "physics-interaction/visual/AmmoCounters.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include <RPS_UI_Framework/NumberOverlay.h>
#include <RPS_UI_Framework/WorldDraw.h>
#include "physics-interaction/visual/RpsUiQuery.h"
#include "physics-interaction/PhysicsLog.h"

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
        // The caliber labels: world text under the counters' own owner, so
        // ROCK's indicator and debug publication is never replaced by them.
        const rpsui::api::world_draw::Api* g_text{ nullptr };
        bool g_captionsPublished{ false };
        Result g_captionReported{ Result::Ok };
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
                (void)rpsui::api::acquire(query, g_numbers);
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
            // The caliber labels are optional: without world drawing the
            // counters still replace the vanilla one.
            if (rpsui::api::acquire(query, g_text) != Result::Ok) {
                g_text = nullptr;
                ROCK_LOG_WARN(Weapon, "Ammo counters: RPS UI world drawing unavailable; the counters show no caliber");
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
            std::uint32_t value, const Look& look) noexcept
        {
            rpsui::api::number_overlay::Presentation presentation{};
            presentation.sequence = ++overlay.sequence;
            presentation.open = 1;
            presentation.value = value;
            presentation.opacity = pose.opacity;
            presentation.style = look.style;
            presentation.secondaryValue = look.secondaryValue;
            presentation.chamber = look.chamber;
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

        // Block-glyph cell size in eye pixels. Minimal's and Immersive's label
        // matches their smaller numbers at arm's length; the Vanilla screen's
        // numbers are larger.
        constexpr float kCaptionPixelSize = 2.5f;
        constexpr float kVanillaCaptionPixelSize = 3.0f;
        // A glyph is 5 cells wide with a 6-cell advance and 7 cells tall.
        constexpr float kGlyphAdvanceCells = 6.0f;
        // Republished every frame; two frames bridge a late game frame.
        constexpr std::uint32_t kCaptionLeaseFrames = 2;

        // Writes the hand's caliber label centered below its numbers. False
        // when the hand shows none.
        bool caption(const HandCounter& counter, const Style style, const std::array<float, 3>& hudColor,
            rpsui::api::world_draw::Text& out) noexcept
        {
            if (!counter.caliber || counter.caliber[0] == '\0' || !counter.captionInView)
                return false;
            const auto length = std::min(std::strlen(counter.caliber), sizeof(out.text) - 1);
            std::memcpy(out.text, counter.caliber, length);
            out.text[length] = '\0';
            const float pixel = style == Style::Vanilla ? kVanillaCaptionPixelSize : kCaptionPixelSize;
            out.pixelSize = pixel;
            out.offsetPixels[0] = -(static_cast<float>(length) * kGlyphAdvanceCells - 1.0f) * pixel * 0.5f;
            out.offsetPixels[1] = 0.0f;
            for (unsigned i = 0; i < 3; ++i) {
                out.anchor[i] = counter.pose.caption[i];
                out.color[i] = style == Style::Minimal ? 1.0f : hudColor[i];
            }
            out.color[3] = counter.pose.opacity;
            return true;
        }

        void publishCaptions(const std::array<rpsui::api::world_draw::Text, 2>& texts, const std::uint32_t count) noexcept
        {
            if (!g_text || (count == 0 && !g_captionsPublished))
                return;
            rpsui::api::world_draw::Publication publication{};
            publication.leaseFrames = kCaptionLeaseFrames;
            publication.textCount = count;
            publication.texts = count ? texts.data() : nullptr;
            if (const auto result = g_text->submit(g_owner, &publication); result != Result::Ok) {
                if (result != g_captionReported) {
                    g_captionReported = result;
                    ROCK_LOG_WARN(Weapon, "Ammo counters: RPS UI world text returned {}; the caliber stays hidden",
                        static_cast<std::uint32_t>(result));
                }
                return;
            }
            g_captionReported = Result::Ok;
            g_captionsPublished = count != 0;
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
        std::array<rpsui::api::world_draw::Text, 2> captions{};
        std::uint32_t captionCount = 0;
        for (std::size_t hand = 0; hand < hands.size(); ++hand) {
            const auto& counter = hands[hand];
            auto& loaded = g_overlays[hand * 2];
            auto& reserve = g_overlays[hand * 2 + 1];
            if (counter.shown && counter.pose.valid && counter.pose.opacity > 0.0f) {
                if (caption(counter, style, hudColor, captions[captionCount]))
                    ++captionCount;
                const auto chamber = overlayMarker(counter.chamber);
                if (style == Style::Minimal) {
                    healthy &= show(loaded, counter.pose, counter.pose.loaded, counter.loaded, Look{ .chamber = chamber });
                    healthy &= show(reserve, counter.pose, counter.pose.reserve, counter.reserve, {});
                } else {
                    const Look look{
                        .style = style == Style::Immersive ? rpsui::api::number_overlay::Style::HudBracketed
                                                           : rpsui::api::number_overlay::Style::Hud,
                        .color = hudColor,
                        .secondaryValue = counter.reserve,
                        .chamber = chamber,
                    };
                    healthy &= show(loaded, counter.pose, counter.pose.counter, counter.loaded, look);
                    healthy &= close(reserve);
                }
            } else {
                healthy &= close(loaded);
                healthy &= close(reserve);
            }
        }
        // The label goes with its numbers: hidden while the vanilla counter shows.
        publishCaptions(captions, healthy ? captionCount : 0);
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
        publishCaptions({}, 0);
    }
}
