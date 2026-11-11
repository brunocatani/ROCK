#pragma once

#include <string>

#include "setup-wizard/SteamVrBindingPolicy.h"

/*
 * ROCK's SteamVR binding for Fallout 4 VR (steamvr_bindings/, embedded in ROCK.dll):
 * inspects what SteamVR uses now and, on the player's choice in the setup wizard, installs
 * ROCK's template beside ROCK's settings and points SteamVR's Fallout 4 VR setting at it.
 * Everything goes through SteamVR's own settings interface; ROCK never edits SteamVR's files.
 *
 * Frame thread only. Both calls talk to vrserver, so they run when the wizard opens, after
 * a game load and on the player's choice, never per frame.
 */
namespace rock::steamvr_binding
{
    struct Inspection
    {
        steamvr_binding_policy::Status status{ steamvr_binding_policy::Status::NotApplicable };
        std::optional<steamvr_binding_policy::ControllerInfo> controller;
        std::string appKey;
        std::string currentUrl;
    };

    // What SteamVR uses for Fallout 4 VR: the controllers it reports and the binding set for
    // each controller ROCK has a binding for.
    struct State
    {
        // False under OpenComposite or without OpenVR: ROCK can set nothing.
        bool steamVr{ false };
        // Prop_ControllerType_String of the controllers on now; empty when none is on.
        std::string reportedType;
        Inspection index;
        Inspection vive;

        [[nodiscard]] const Inspection& of(steamvr_binding_policy::Controller controller) const noexcept
        {
            return controller == steamvr_binding_policy::Controller::Index ? index : vive;
        }

        [[nodiscard]] std::optional<steamvr_binding_policy::Headset> detected() const noexcept
        {
            return steamvr_binding_policy::detectHeadset(steamVr, reportedType, index.status, vive.status);
        }
    };

    [[nodiscard]] State read() noexcept;

    // The detected controllers' inspection (State::detected); NotApplicable when they need none.
    [[nodiscard]] Inspection inspect() noexcept;

    // Installs the template and sets it for the inspected controller, then reads the
    // setting back. On failure, error says what failed.
    [[nodiscard]] bool apply(const Inspection& inspection, std::string& error) noexcept;
}
