#pragma once

/*
 * Shoulder stash notifications are formatted outside the runtime transfer path
 * so UI text stays edge-triggered and inventory-success driven. The transfer
 * owner supplies the resolved item name and count; this policy only shapes the
 * user-visible confirmation and fallback text.
 */

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>

namespace rock::shoulder_stash_notification_policy
{
    [[nodiscard]] inline std::string fallbackItemName(std::uint32_t formID)
    {
        if (formID == 0) {
            return "item";
        }

        char buffer[16]{};
        std::snprintf(buffer, sizeof(buffer), "%08X", formID);
        std::string name = "item ";
        name += buffer;
        return name;
    }

    [[nodiscard]] inline std::string formatCollectedNotification(std::string_view itemName, int count, std::uint32_t formID)
    {
        std::string message = "[ROCK] Collected ";
        message += itemName.empty() ? fallbackItemName(formID) : std::string(itemName);

        const int safeCount = (std::max)(1, count);
        if (safeCount > 1) {
            message += " x";
            message += std::to_string(safeCount);
        }

        return message;
    }
}
