#pragma once

#include "api/OutdatedConsumers.h"

#include <RPS_UI_Framework/Client.h>

#include <Windows.h>

namespace rock::rps_ui
{
    // Null when RPS UI Framework is not loaded (ROCK then runs without it), or
    // when the loaded one has no 2.0 discovery, which also stops the game at the
    // main menu. A refusal after this comes from RPS UI, which reports it itself.
    [[nodiscard]] inline rpsui::api::QueryInterfaceV1 query() noexcept
    {
        const auto module = GetModuleHandleW(rpsui::api::kModuleName);
        if (!module)
            return nullptr;
        const auto function = reinterpret_cast<rpsui::api::QueryInterfaceV1>(
            GetProcAddress(module, rpsui::api::kQueryExportName));
        if (!function)
            rock::api::discovery::noteRpsUiWithoutDiscovery();
        return function;
    }
}
