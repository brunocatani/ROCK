#include "physics-interaction/collision/ColliderTuning.h"
#include "RockConfig.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <string_view>

namespace rock::collider_tuning
{
    namespace hand_detail
    {
        using hand_collider_semantics::HandColliderRole;
        float sanitizeHandColliderScale(float value)
        {
            if (!std::isfinite(value)) {
                return 1.0f;
            }
            return std::clamp(value, 0.05f, 8.0f);
        }

        std::string_view trimOverrideToken(std::string_view value)
        {
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
                value.remove_prefix(1);
            }
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
                value.remove_suffix(1);
            }
            return value;
        }

        char lowerAscii(char ch)
        {
            return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }

        bool equalsAsciiInsensitive(std::string_view lhs, std::string_view rhs)
        {
            if (lhs.size() != rhs.size()) {
                return false;
            }
            for (std::size_t i = 0; i < lhs.size(); ++i) {
                if (lowerAscii(lhs[i]) != lowerAscii(rhs[i])) {
                    return false;
                }
            }
            return true;
        }

        bool parseOverrideFloat(std::string_view token, float& outValue)
        {
            token = trimOverrideToken(token);
            if (token.empty()) {
                return false;
            }

            const std::string buffer{ token };
            char* end = nullptr;
            const float parsed = std::strtof(buffer.c_str(), &end);
            if (end == buffer.c_str() || !std::isfinite(parsed)) {
                return false;
            }
            while (end && *end != '\0') {
                if (!std::isspace(static_cast<unsigned char>(*end))) {
                    return false;
                }
                ++end;
            }
            outValue = parsed;
            return true;
        }

        bool handOverrideProfileMatches(std::string_view profile, bool powerArmor)
        {
            profile = trimOverrideToken(profile);
            if (profile.empty()) {
                return true;
            }
            if (equalsAsciiInsensitive(profile, "PowerArmor")) {
                return powerArmor;
            }
            if (equalsAsciiInsensitive(profile, "Standard")) {
                return !powerArmor;
            }
            return false;
        }

        enum class HandOverrideMatch : int
        {
            None = 0,
            Generic = 1,
            ProfileSpecific = 2,
        };

        HandOverrideMatch handRoleOverrideKeyMatch(std::string_view key, HandColliderRole role, bool powerArmor)
        {
            key = trimOverrideToken(key);
            const auto dot = key.find('.');
            std::string_view roleNameToken = key;
            bool profileSpecific = false;
            if (dot != std::string_view::npos) {
                if (!handOverrideProfileMatches(key.substr(0, dot), powerArmor)) {
                    return HandOverrideMatch::None;
                }
                roleNameToken = trimOverrideToken(key.substr(dot + 1));
                profileSpecific = true;
            }

            return equalsAsciiInsensitive(roleNameToken, hand_collider_semantics::roleName(role)) ?
                       (profileSpecific ? HandOverrideMatch::ProfileSpecific : HandOverrideMatch::Generic) :
                       HandOverrideMatch::None;
        }

        float handRoleRadiusScaleOverride(const RockConfigValues& config, HandColliderRole role, bool powerArmor)
        {
            if (hand_collider_semantics::isPalmRole(role) || config.rockHandBoneColliderRadiusScaleOverrides.empty()) {
                return 1.0f;
            }

            std::string_view overrides{ config.rockHandBoneColliderRadiusScaleOverrides };
            HandOverrideMatch bestMatch = HandOverrideMatch::None;
            float bestScale = 1.0f;
            while (!overrides.empty()) {
                const auto semicolon = overrides.find(';');
                const auto entry = trimOverrideToken(overrides.substr(0, semicolon));
                if (!entry.empty()) {
                    const auto equals = entry.find('=');
                    const auto matchType = equals != std::string_view::npos ? handRoleOverrideKeyMatch(entry.substr(0, equals), role, powerArmor) :
                                                                              HandOverrideMatch::None;
                    if (matchType != HandOverrideMatch::None && static_cast<int>(matchType) >= static_cast<int>(bestMatch)) {
                        float parsed = 1.0f;
                        if (parseOverrideFloat(entry.substr(equals + 1), parsed)) {
                            bestMatch = matchType;
                            bestScale = sanitizeHandColliderScale(parsed);
                        }
                    }
                }
                if (semicolon == std::string_view::npos) {
                    break;
                }
                overrides.remove_prefix(semicolon + 1);
            }
            return bestScale;
        }

        struct PalmDimensionScale
        {
            float x = 1.0f;
            float y = 1.0f;
            float z = 1.0f;
        };

        bool parsePalmDimensionScale(std::string_view value, PalmDimensionScale& outScale)
        {
            outScale = {};
            float parsed[3]{};
            std::uint32_t count = 0;
            value = trimOverrideToken(value);
            while (!value.empty() && count < 3) {
                const auto comma = value.find(',');
                const auto token = trimOverrideToken(value.substr(0, comma));
                if (!parseOverrideFloat(token, parsed[count])) {
                    return false;
                }
                ++count;
                if (comma == std::string_view::npos) {
                    break;
                }
                if (count >= 3) {
                    return false;
                }
                value.remove_prefix(comma + 1);
            }

            if (count != 3 || value.find(',') != std::string_view::npos) {
                return false;
            }

            outScale.x = sanitizeHandColliderScale(parsed[0]);
            outScale.y = sanitizeHandColliderScale(parsed[1]);
            outScale.z = sanitizeHandColliderScale(parsed[2]);
            return true;
        }

        PalmDimensionScale palmDimensionScaleOverride(const RockConfigValues& config, HandColliderRole role, bool powerArmor)
        {
            PalmDimensionScale result{};
            if (!hand_collider_semantics::isPalmRole(role) || config.rockHandPalmColliderDimensionScaleOverrides.empty()) {
                return result;
            }

            std::string_view overrides{ config.rockHandPalmColliderDimensionScaleOverrides };
            HandOverrideMatch bestMatch = HandOverrideMatch::None;
            while (!overrides.empty()) {
                const auto semicolon = overrides.find(';');
                const auto entry = trimOverrideToken(overrides.substr(0, semicolon));
                if (!entry.empty()) {
                    const auto equals = entry.find('=');
                    const auto matchType = equals != std::string_view::npos ? handRoleOverrideKeyMatch(entry.substr(0, equals), role, powerArmor) :
                                                                              HandOverrideMatch::None;
                    if (matchType != HandOverrideMatch::None && static_cast<int>(matchType) >= static_cast<int>(bestMatch)) {
                        PalmDimensionScale parsed{};
                        if (parsePalmDimensionScale(entry.substr(equals + 1), parsed)) {
                            bestMatch = matchType;
                            result = parsed;
                        }
                    }
                }
                if (semicolon == std::string_view::npos) {
                    break;
                }
                overrides.remove_prefix(semicolon + 1);
            }
            return result;
        }

        void mixHandColliderSignatureString(std::uint64_t& signature, const std::string& value)
        {
            for (unsigned char ch : value) {
                signature ^= static_cast<std::uint64_t>(ch) + 0x9E37'79B9'7F4A'7C15ull + (signature << 6) + (signature >> 2);
            }
        }

        std::uint64_t handColliderTuningSignature(const RockConfigValues& config, bool powerArmor)
        {
            std::uint64_t signature = powerArmor ? 0x4841'4E44'5041ull : 0x4841'4E44'5354ull;
            mixHandColliderSignatureString(signature, config.rockHandBoneColliderRadiusScaleOverrides);
            mixHandColliderSignatureString(signature, config.rockHandPalmColliderDimensionScaleOverrides);
            return signature;
        }

    }
    HandProfile prepareHand(const RockConfigValues& config, bool powerArmor)
    {
        HandProfile result{};
        result.signature = hand_detail::handColliderTuningSignature(config, powerArmor);
        for (const auto role : hand_collider_semantics::kHandColliderRoles) {
            auto& tuning = result.roles[static_cast<std::size_t>(role)];
            tuning.radiusScale = hand_detail::handRoleRadiusScaleOverride(config, role, powerArmor);
            const auto palm = hand_detail::palmDimensionScaleOverride(config, role, powerArmor);
            tuning.palmDimensionScale = { palm.x, palm.y, palm.z };
        }
        return result;
    }
}
