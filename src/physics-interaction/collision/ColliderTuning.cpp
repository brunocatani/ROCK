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
    namespace body_detail
    {
        using skeleton_bone_debug_math::BoneColliderDescriptor;
        using skeleton_bone_debug_math::BoneColliderRole;
        float sanitizeBodyColliderScale(float value)
        {
            if (!std::isfinite(value)) {
                return 1.0f;
            }
            return std::clamp(value, 0.05f, 8.0f);
        }

        float roleRadiusScale(const RockConfigValues& config, BoneColliderRole role)
        {
            switch (role) {
            case BoneColliderRole::TorsoSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderTorsoRadiusScale);
            case BoneColliderRole::UpperArmSegment:
            case BoneColliderRole::ForearmSegment:
            case BoneColliderRole::HandSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderArmRadiusScale);
            case BoneColliderRole::LegSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderLegRadiusScale);
            case BoneColliderRole::FootSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderFootRadiusScale);
            case BoneColliderRole::FingerSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderArmRadiusScale);
            }
            return 1.0f;
        }

        float roleLengthScale(const RockConfigValues& config, BoneColliderRole role)
        {
            switch (role) {
            case BoneColliderRole::TorsoSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderTorsoLengthScale);
            case BoneColliderRole::UpperArmSegment:
            case BoneColliderRole::ForearmSegment:
            case BoneColliderRole::HandSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderArmLengthScale);
            case BoneColliderRole::LegSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderLegLengthScale);
            case BoneColliderRole::FootSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderFootLengthScale);
            case BoneColliderRole::FingerSegment:
                return sanitizeBodyColliderScale(config.rockBodyBoneColliderArmLengthScale);
            }
            return 1.0f;
        }

        float profileRadiusScale(const RockConfigValues& config, bool inPowerArmor)
        {
            return sanitizeBodyColliderScale(inPowerArmor ? config.rockBodyBoneColliderPowerArmorRadiusScale :
                                                            config.rockBodyBoneColliderStandardRadiusScale);
        }

        float profileLengthScale(const RockConfigValues& config, bool inPowerArmor)
        {
            return sanitizeBodyColliderScale(inPowerArmor ? config.rockBodyBoneColliderPowerArmorLengthScale :
                                                            config.rockBodyBoneColliderStandardLengthScale);
        }

        float profileConvexRadiusScale(const RockConfigValues& config, bool inPowerArmor)
        {
            return sanitizeBodyColliderScale(inPowerArmor ? config.rockBodyBoneColliderPowerArmorConvexRadiusScale :
                                                            config.rockBodyBoneColliderStandardConvexRadiusScale);
        }

        struct BodyZoneTuningOverride
        {
            bool valid = false;
            float radiusScale = 1.0f;
            float lengthScale = 1.0f;
            float convexRadiusScale = 1.0f;
            RE::NiPoint3 localOffsetGame{};
            bool hasLocalOffset = false;
        };

        std::string_view trimZoneOverrideToken(std::string_view value)
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

        bool parseZoneOverrideFloat(std::string_view token, float& outValue)
        {
            token = trimZoneOverrideToken(token);
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

        bool parseZoneOverrideValue(std::string_view value, BodyZoneTuningOverride& outOverride)
        {
            outOverride = {};
            float parsed[6]{};
            std::uint32_t count = 0;
            value = trimZoneOverrideToken(value);
            while (!value.empty() && count < 6) {
                const auto comma = value.find(',');
                const auto token = trimZoneOverrideToken(value.substr(0, comma));
                if (!parseZoneOverrideFloat(token, parsed[count])) {
                    return false;
                }
                ++count;
                if (comma == std::string_view::npos) {
                    break;
                }
                if (count >= 6) {
                    return false;
                }
                value.remove_prefix(comma + 1);
            }

            if (count != 3 && count != 6) {
                return false;
            }
            if (value.find(',') != std::string_view::npos) {
                return false;
            }

            outOverride.valid = true;
            outOverride.radiusScale = sanitizeBodyColliderScale(parsed[0]);
            outOverride.lengthScale = sanitizeBodyColliderScale(parsed[1]);
            outOverride.convexRadiusScale = sanitizeBodyColliderScale(parsed[2]);
            if (count == 6) {
                outOverride.localOffsetGame = RE::NiPoint3{ parsed[3], parsed[4], parsed[5] };
                outOverride.hasLocalOffset = true;
            }
            return true;
        }

        bool zoneOverrideProfileMatches(std::string_view profile, bool inPowerArmor)
        {
            profile = trimZoneOverrideToken(profile);
            if (profile.empty()) {
                return true;
            }
            if (equalsAsciiInsensitive(profile, "PowerArmor")) {
                return inPowerArmor;
            }
            if (equalsAsciiInsensitive(profile, "Standard")) {
                return !inPowerArmor;
            }
            return false;
        }

        enum class BodyZoneOverrideMatch
        {
            None,
            Generic,
            ProfileSpecific,
        };

        BodyZoneOverrideMatch zoneOverrideKeyMatch(std::string_view key, body_zone::BodyZoneKind zone, bool inPowerArmor)
        {
            key = trimZoneOverrideToken(key);
            const auto dot = key.find('.');
            std::string_view zoneName = key;
            BodyZoneOverrideMatch matchType = BodyZoneOverrideMatch::Generic;
            if (dot != std::string_view::npos) {
                if (!zoneOverrideProfileMatches(key.substr(0, dot), inPowerArmor)) {
                    return BodyZoneOverrideMatch::None;
                }
                zoneName = trimZoneOverrideToken(key.substr(dot + 1));
                matchType = BodyZoneOverrideMatch::ProfileSpecific;
            }
            return equalsAsciiInsensitive(zoneName, body_zone::bodyZoneName(zone)) ? matchType : BodyZoneOverrideMatch::None;
        }

        BodyZoneTuningOverride bodyZoneTuningOverride(const RockConfigValues& config, body_zone::BodyZoneKind zone, bool inPowerArmor)
        {
            if (zone == body_zone::BodyZoneKind::Unknown || config.rockBodyBoneColliderZoneScaleOverrides.empty()) {
                return {};
            }

            std::string_view overrides{ config.rockBodyBoneColliderZoneScaleOverrides };
            BodyZoneTuningOverride genericMatch{};
            while (!overrides.empty()) {
                const auto semicolon = overrides.find(';');
                const auto entry = trimZoneOverrideToken(overrides.substr(0, semicolon));
                if (!entry.empty()) {
                    const auto equals = entry.find('=');
                    const auto matchType = equals != std::string_view::npos ? zoneOverrideKeyMatch(entry.substr(0, equals), zone, inPowerArmor) :
                                                                              BodyZoneOverrideMatch::None;
                    if (matchType != BodyZoneOverrideMatch::None) {
                        BodyZoneTuningOverride parsed{};
                        if (parseZoneOverrideValue(entry.substr(equals + 1), parsed)) {
                            if (matchType == BodyZoneOverrideMatch::ProfileSpecific) {
                                return parsed;
                            }
                            genericMatch = parsed;
                        }
                    }
                }
                if (semicolon == std::string_view::npos) {
                    break;
                }
                overrides.remove_prefix(semicolon + 1);
            }
            return genericMatch;
        }

        enum class BodyRadiusOverrideMatch : int
        {
            None = 0,
            GenericZone = 1,
            ProfileZone = 2,
            GenericBonePair = 3,
            ProfileBonePair = 4,
        };

        bool bonePairOverrideKeyMatches(std::string_view key, const BoneColliderDescriptor& descriptor)
        {
            key = trimZoneOverrideToken(key);
            const auto arrow = key.find("->");
            const auto delimiterOffset = arrow != std::string_view::npos ? arrow : key.find('>');
            const auto delimiterLength = arrow != std::string_view::npos ? 2u : 1u;
            if (delimiterOffset == std::string_view::npos) {
                return false;
            }

            const auto start = trimZoneOverrideToken(key.substr(0, delimiterOffset));
            const auto end = trimZoneOverrideToken(key.substr(delimiterOffset + delimiterLength));
            return equalsAsciiInsensitive(start, descriptor.startBone) && equalsAsciiInsensitive(end, descriptor.endBone);
        }

        BodyRadiusOverrideMatch bodyRadiusOverrideKeyMatch(std::string_view key, const BoneColliderDescriptor& descriptor, bool inPowerArmor)
        {
            key = trimZoneOverrideToken(key);
            const auto dot = key.find('.');
            std::string_view colliderName = key;
            bool profileSpecific = false;
            if (dot != std::string_view::npos) {
                if (!zoneOverrideProfileMatches(key.substr(0, dot), inPowerArmor)) {
                    return BodyRadiusOverrideMatch::None;
                }
                colliderName = trimZoneOverrideToken(key.substr(dot + 1));
                profileSpecific = true;
            }

            if (bonePairOverrideKeyMatches(colliderName, descriptor)) {
                return profileSpecific ? BodyRadiusOverrideMatch::ProfileBonePair : BodyRadiusOverrideMatch::GenericBonePair;
            }
            if (equalsAsciiInsensitive(colliderName, body_zone::bodyZoneName(descriptor.zone))) {
                return profileSpecific ? BodyRadiusOverrideMatch::ProfileZone : BodyRadiusOverrideMatch::GenericZone;
            }
            return BodyRadiusOverrideMatch::None;
        }

        float bodyRadiusScaleOverride(const RockConfigValues& config, const BoneColliderDescriptor& descriptor, bool inPowerArmor)
        {
            if (config.rockBodyBoneColliderRadiusScaleOverrides.empty()) {
                return 1.0f;
            }

            std::string_view overrides{ config.rockBodyBoneColliderRadiusScaleOverrides };
            BodyRadiusOverrideMatch bestMatch = BodyRadiusOverrideMatch::None;
            float bestScale = 1.0f;
            while (!overrides.empty()) {
                const auto semicolon = overrides.find(';');
                const auto entry = trimZoneOverrideToken(overrides.substr(0, semicolon));
                if (!entry.empty()) {
                    const auto equals = entry.find('=');
                    const auto matchType = equals != std::string_view::npos ? bodyRadiusOverrideKeyMatch(entry.substr(0, equals), descriptor, inPowerArmor) :
                                                                              BodyRadiusOverrideMatch::None;
                    if (matchType != BodyRadiusOverrideMatch::None && static_cast<int>(matchType) >= static_cast<int>(bestMatch)) {
                        float parsed = 1.0f;
                        if (parseZoneOverrideFloat(entry.substr(equals + 1), parsed)) {
                            bestMatch = matchType;
                            bestScale = sanitizeBodyColliderScale(parsed);
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

        std::uint32_t quantizeBodyColliderScale(float value)
        {
            return static_cast<std::uint32_t>(std::lround(sanitizeBodyColliderScale(value) * 10000.0f));
        }

        void mixBodyColliderSignature(std::uint64_t& signature, float value)
        {
            signature ^= static_cast<std::uint64_t>(quantizeBodyColliderScale(value)) + 0x9E37'79B9'7F4A'7C15ull + (signature << 6) + (signature >> 2);
        }

        void mixBodyColliderSignatureString(std::uint64_t& signature, const std::string& value)
        {
            for (unsigned char ch : value) {
                signature ^= static_cast<std::uint64_t>(ch) + 0x9E37'79B9'7F4A'7C15ull + (signature << 6) + (signature >> 2);
            }
        }

        std::uint64_t bodyColliderTuningSignature(const RockConfigValues& config, bool inPowerArmor)
        {
            std::uint64_t signature = inPowerArmor ? 0x5041'524D'4F52ull : 0x5354'414E'4444ull;
            mixBodyColliderSignature(signature, config.rockBodyBoneLegAndFootCollidersEnabled ? 1.0f : 0.0f);
            mixBodyColliderSignature(signature, profileRadiusScale(config, inPowerArmor));
            mixBodyColliderSignature(signature, profileLengthScale(config, inPowerArmor));
            mixBodyColliderSignature(signature, profileConvexRadiusScale(config, inPowerArmor));
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderTorsoRadiusScale);
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderArmRadiusScale);
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderLegRadiusScale);
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderFootRadiusScale);
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderTorsoLengthScale);
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderArmLengthScale);
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderLegLengthScale);
            mixBodyColliderSignature(signature, config.rockBodyBoneColliderFootLengthScale);
            mixBodyColliderSignatureString(signature, config.rockBodyBoneColliderZoneScaleOverrides);
            mixBodyColliderSignatureString(signature, config.rockBodyBoneColliderRadiusScaleOverrides);
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

    BodyProfile prepareBody(const RockConfigValues& config, bool powerArmor)
    {
        using namespace body_detail;
        BodyProfile result{};
        result.signature = bodyColliderTuningSignature(config, powerArmor);
        const auto& descriptors = powerArmor ? skeleton_bone_debug_math::kPowerArmorBodyColliderDescriptors :
            skeleton_bone_debug_math::kStandardBodyColliderDescriptors;
        for (std::size_t i = 0; i < descriptors.size(); ++i) {
            const auto& descriptor = descriptors[i];
            const auto zoneOverride = bodyZoneTuningOverride(config, descriptor.zone, powerArmor);
            const float radiusScale = profileRadiusScale(config, powerArmor) * roleRadiusScale(config, descriptor.role) *
                (zoneOverride.valid ? zoneOverride.radiusScale : 1.0f) * bodyRadiusScaleOverride(config, descriptor, powerArmor);
            const float lengthScale = profileLengthScale(config, powerArmor) * roleLengthScale(config, descriptor.role) *
                (zoneOverride.valid ? zoneOverride.lengthScale : 1.0f);
            const float convexRadiusScale = profileConvexRadiusScale(config, powerArmor) *
                (zoneOverride.valid ? zoneOverride.convexRadiusScale : 1.0f);
            result.descriptors[i] = {
                .radius = descriptor.radiusGameUnits * radiusScale,
                .convexRadius = descriptor.convexRadiusGameUnits * convexRadiusScale,
                .lengthScale = lengthScale,
                .localOffsetGame = zoneOverride.localOffsetGame,
                .hasLocalOffset = zoneOverride.valid && zoneOverride.hasLocalOffset,
            };
        }
        return result;
    }
}
