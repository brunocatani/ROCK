#pragma once

#include <charconv>
#include <cstdint>
#include <string_view>

#include "physics-interaction/collision/CollisionLayerPolicy.h"

namespace rock::far_selection_blacklist_policy
{
    struct FarSelectionBlacklistInput
    {
        bool isFarSelection = false;
        std::uint32_t referenceFormId = 0;
        std::uint32_t baseFormId = 0;
        std::uint32_t collisionLayer = 0;
        std::string_view formType;
        std::string_view blockedReferenceFormIds;
        std::string_view blockedBaseFormIds;
        std::string_view blockedFormTypes;
        std::string_view blockedLayers;
    };

    struct FarSelectionBlacklistDecision
    {
        bool blocked = false;
        const char* reason = "none";
    };

    [[nodiscard]] inline bool isDelimiter(char c) noexcept
    {
        return c == ',' || c == ';' || c == '|' || c == ' ' || c == '\t' || c == '\r' || c == '\n';
    }

    [[nodiscard]] inline std::string_view trimToken(std::string_view value) noexcept
    {
        while (!value.empty() && isDelimiter(value.front())) {
            value.remove_prefix(1);
        }
        while (!value.empty() && isDelimiter(value.back())) {
            value.remove_suffix(1);
        }
        return value;
    }

    [[nodiscard]] inline bool nextToken(std::string_view list, std::size_t& offset, std::string_view& outToken) noexcept
    {
        while (offset < list.size() && isDelimiter(list[offset])) {
            ++offset;
        }
        const std::size_t start = offset;
        while (offset < list.size() && !isDelimiter(list[offset])) {
            ++offset;
        }
        outToken = trimToken(list.substr(start, offset - start));
        return !outToken.empty();
    }

    [[nodiscard]] inline char upperAscii(char c) noexcept
    {
        return c >= 'a' && c <= 'z' ? static_cast<char>(c - ('a' - 'A')) : c;
    }

    [[nodiscard]] inline bool equalsIgnoreCase(std::string_view lhs, std::string_view rhs) noexcept
    {
        if (lhs.size() != rhs.size()) {
            return false;
        }
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            if (upperAscii(lhs[i]) != upperAscii(rhs[i])) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] inline bool tokenLooksHex(std::string_view token) noexcept
    {
        if (token.size() > 2 && token[0] == '0' && (token[1] == 'x' || token[1] == 'X')) {
            return true;
        }
        for (const char c : token) {
            if ((c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline bool parseUnsignedToken(std::string_view token, std::uint32_t& outValue, bool preferHex) noexcept
    {
        token = trimToken(token);
        if (token.empty()) {
            return false;
        }
        int base = preferHex || tokenLooksHex(token) ? 16 : 10;
        if (token.size() > 2 && token[0] == '0' && (token[1] == 'x' || token[1] == 'X')) {
            token.remove_prefix(2);
            base = 16;
        }
        if (token.empty()) {
            return false;
        }
        std::uint32_t parsed = 0;
        const auto* begin = token.data();
        const auto* end = begin + token.size();
        const auto result = std::from_chars(begin, end, parsed, base);
        if (result.ec != std::errc{} || result.ptr != end) {
            return false;
        }
        outValue = parsed;
        return true;
    }

    [[nodiscard]] inline bool listContainsFormId(std::string_view list, std::uint32_t formId) noexcept
    {
        if (list.empty() || formId == 0) {
            return false;
        }
        std::size_t offset = 0;
        std::string_view token;
        while (nextToken(list, offset, token)) {
            std::uint32_t parsed = 0;
            if (parseUnsignedToken(token, parsed, true) && parsed == formId) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline bool listContainsText(std::string_view list, std::string_view value) noexcept
    {
        if (list.empty() || value.empty()) {
            return false;
        }
        std::size_t offset = 0;
        std::string_view token;
        while (nextToken(list, offset, token)) {
            if (equalsIgnoreCase(token, value)) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline bool tryParseLayerName(std::string_view token, std::uint32_t& outLayer) noexcept
    {
        if (equalsIgnoreCase(token, "STATIC")) {
            outLayer = collision_layer_policy::FO4_LAYER_STATIC;
            return true;
        }
        if (equalsIgnoreCase(token, "ANIMSTATIC") || equalsIgnoreCase(token, "ANIM_STATIC")) {
            outLayer = collision_layer_policy::FO4_LAYER_ANIMSTATIC;
            return true;
        }
        if (equalsIgnoreCase(token, "CLUTTER")) {
            outLayer = collision_layer_policy::FO4_LAYER_CLUTTER;
            return true;
        }
        if (equalsIgnoreCase(token, "BIPED")) {
            outLayer = collision_layer_policy::FO4_LAYER_BIPED;
            return true;
        }
        if (equalsIgnoreCase(token, "DEADBIP")) {
            outLayer = collision_layer_policy::FO4_LAYER_DEADBIP;
            return true;
        }
        if (equalsIgnoreCase(token, "NONCOLLIDABLE")) {
            outLayer = collision_layer_policy::FO4_LAYER_NONCOLLIDABLE;
            return true;
        }
        return false;
    }

    [[nodiscard]] inline bool listContainsLayer(std::string_view list, std::uint32_t layer) noexcept
    {
        if (list.empty()) {
            return false;
        }
        std::size_t offset = 0;
        std::string_view token;
        while (nextToken(list, offset, token)) {
            std::uint32_t parsed = 0;
            if ((tryParseLayerName(token, parsed) || parseUnsignedToken(token, parsed, false)) && parsed == layer) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline FarSelectionBlacklistDecision evaluateFarSelectionBlacklist(const FarSelectionBlacklistInput& input) noexcept
    {
        if (!input.isFarSelection) {
            return {};
        }
        if (listContainsFormId(input.blockedReferenceFormIds, input.referenceFormId)) {
            return { .blocked = true, .reason = "far-blacklist-ref-formid" };
        }
        if (listContainsFormId(input.blockedBaseFormIds, input.baseFormId)) {
            return { .blocked = true, .reason = "far-blacklist-base-formid" };
        }
        if (listContainsText(input.blockedFormTypes, input.formType)) {
            return { .blocked = true, .reason = "far-blacklist-form-type" };
        }
        if (listContainsLayer(input.blockedLayers, input.collisionLayer)) {
            return { .blocked = true, .reason = "far-blacklist-layer" };
        }
        return {};
    }
}
