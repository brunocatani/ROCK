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

    /*
     * When a ROCK client of RPS UI tries to bind. Clients bind from the frame,
     * after every F4SE plugin has loaded, so a missing RPS UI is final for the
     * session and is never looked for again. A framework that is loaded but
     * not ready is asked again on the next call, which costs one call into it.
     * Only a refusal by RPS UI is paused, by elapsed time. Each client owns one
     * gate and guards it like the rest of its bind state.
     */
    class BindGate
    {
    public:
        // The query to bind through, or null when no attempt is due.
        [[nodiscard]] rpsui::api::QueryInterfaceV1 due() noexcept
        {
            if (_absent || (_retryAtMilliseconds != 0 && GetTickCount64() < _retryAtMilliseconds))
                return nullptr;
            _retryAtMilliseconds = 0;
            if (!_query) {
                _query = query();
                _absent = !_query;
            }
            return _query;
        }

        [[nodiscard]] bool absent() const noexcept { return _absent; }

        void refused() noexcept { _retryAtMilliseconds = GetTickCount64() + kRefusalPauseMilliseconds; }

    private:
        static constexpr ULONGLONG kRefusalPauseMilliseconds = 1000;

        rpsui::api::QueryInterfaceV1 _query{ nullptr };
        ULONGLONG _retryAtMilliseconds{ 0 };
        bool _absent{ false };
    };
}
