#include "physics-interaction/weapon/AuthoredWeaponGripCacheFormat.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>
#include <utility>

namespace rock::authored_weapon_grip_cache
{
    namespace
    {
        using json = nlohmann::json;

        constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ull;
        constexpr std::uint64_t kFnvPrime = 1099511628211ull;

        void mixBytes(std::uint64_t& hash, const void* data, const std::size_t size) noexcept
        {
            const auto* bytes = static_cast<const std::uint8_t*>(data);
            for (std::size_t index = 0; index < size; ++index) {
                hash ^= bytes[index];
                hash *= kFnvPrime;
            }
        }

        template <class Value>
        void mixValue(std::uint64_t& hash, const Value& value) noexcept
        {
            mixBytes(hash, &value, sizeof(value));
        }

        void mixString(std::uint64_t& hash, const std::string_view value) noexcept
        {
            const auto size = static_cast<std::uint64_t>(value.size());
            mixValue(hash, size);
            mixBytes(hash, value.data(), value.size());
        }

        [[nodiscard]] bool finiteBounded(const float value, const float bound) noexcept
        {
            return std::isfinite(value) && std::abs(value) <= bound;
        }

        [[nodiscard]] float dotRow(const PersistedTransform& transform, const std::size_t left, const std::size_t right) noexcept
        {
            float result = 0.0f;
            for (std::size_t column = 0; column < 3; ++column) {
                result += transform.rotate[left * 3 + column] * transform.rotate[right * 3 + column];
            }
            return result;
        }

        [[nodiscard]] float determinant(const PersistedTransform& transform) noexcept
        {
            const auto& r = transform.rotate;
            return r[0] * (r[4] * r[8] - r[5] * r[7]) -
                   r[1] * (r[3] * r[8] - r[5] * r[6]) +
                   r[2] * (r[3] * r[7] - r[4] * r[6]);
        }

        [[nodiscard]] std::string hex64(const std::uint64_t value)
        {
            char text[17]{};
            std::snprintf(text, sizeof(text), "%016llX", static_cast<unsigned long long>(value));
            return text;
        }

        [[nodiscard]] bool parseHex64(const json& value, std::uint64_t& out)
        {
            if (!value.is_string()) {
                return false;
            }
            const std::string text = value.get<std::string>();
            if (text.empty() || text.size() > 16) {
                return false;
            }
            std::uint64_t parsed = 0;
            for (const char character : text) {
                std::uint8_t digit = 0;
                if (character >= '0' && character <= '9') {
                    digit = static_cast<std::uint8_t>(character - '0');
                } else if (character >= 'a' && character <= 'f') {
                    digit = static_cast<std::uint8_t>(character - 'a' + 10);
                } else if (character >= 'A' && character <= 'F') {
                    digit = static_cast<std::uint8_t>(character - 'A' + 10);
                } else {
                    return false;
                }
                parsed = (parsed << 4) | digit;
            }
            out = parsed;
            return true;
        }

        [[nodiscard]] json transformToJson(const PersistedTransform& transform)
        {
            return json{
                { "rotate", transform.rotate },
                { "translate", transform.translate },
                { "scale", transform.scale },
            };
        }

        [[nodiscard]] bool transformFromJson(const json& value, PersistedTransform& out)
        {
            if (!value.is_object() || !value.contains("rotate") || !value.contains("translate") || !value.contains("scale")) {
                return false;
            }
            const auto& rotate = value.at("rotate");
            const auto& translate = value.at("translate");
            if (!rotate.is_array() || rotate.size() != out.rotate.size() || !translate.is_array() || translate.size() != out.translate.size() ||
                !value.at("scale").is_number()) {
                return false;
            }
            for (std::size_t index = 0; index < out.rotate.size(); ++index) {
                if (!rotate[index].is_number()) {
                    return false;
                }
                out.rotate[index] = rotate[index].get<float>();
            }
            for (std::size_t index = 0; index < out.translate.size(); ++index) {
                if (!translate[index].is_number()) {
                    return false;
                }
                out.translate[index] = translate[index].get<float>();
            }
            out.scale = value.at("scale").get<float>();
            return validTransform(out);
        }

        void setError(std::string* outError, const std::string_view message)
        {
            if (outError) {
                *outError = message;
            }
        }
    }

    bool StableFormIdentity::valid() const noexcept
    {
        return !plugin.empty() && plugin.size() <= 260 && localFormId != 0;
    }

    bool CacheKey::valid() const noexcept
    {
        return weapon.valid() && graphProfileKey != 0;
    }

    std::size_t CacheKeyHash::operator()(const CacheKey& key) const noexcept
    {
        std::uint64_t hash = kFnvOffsetBasis;
        mixString(hash, key.weapon.plugin);
        mixValue(hash, key.weapon.localFormId);
        mixValue(hash, key.pGripVariantKey);
        mixValue(hash, key.instanceContentKey);
        mixValue(hash, key.graphProfileKey);
        mixValue(hash, key.inPowerArmor);
        return static_cast<std::size_t>(hash);
    }

    bool validTransform(const PersistedTransform& transform) noexcept
    {
        for (const float value : transform.rotate) {
            if (!finiteBounded(value, 2.0f)) {
                return false;
            }
        }
        for (const float value : transform.translate) {
            if (!finiteBounded(value, 1000000.0f)) {
                return false;
            }
        }
        if (!std::isfinite(transform.scale) || transform.scale < 0.01f || transform.scale > 100.0f) {
            return false;
        }
        for (std::size_t row = 0; row < 3; ++row) {
            if (std::abs(dotRow(transform, row, row) - 1.0f) > 0.1f) {
                return false;
            }
        }
        if (std::abs(dotRow(transform, 0, 1)) > 0.1f ||
            std::abs(dotRow(transform, 0, 2)) > 0.1f ||
            std::abs(dotRow(transform, 1, 2)) > 0.1f) {
            return false;
        }
        const float det = determinant(transform);
        return std::isfinite(det) && std::abs(det - 1.0f) <= 0.2f;
    }

    std::uint64_t calculateChecksum(const CacheRecord& record) noexcept
    {
        std::uint64_t hash = kFnvOffsetBasis;
        mixValue(hash, record.formatVersion);
        mixValue(hash, record.poseAlgorithmVersion);
        mixString(hash, record.key.weapon.plugin);
        mixValue(hash, record.key.weapon.localFormId);
        mixValue(hash, record.key.pGripVariantKey);
        mixValue(hash, record.key.instanceContentKey);
        mixValue(hash, record.key.graphProfileKey);
        mixValue(hash, record.key.inPowerArmor);
        mixBytes(hash, record.rightHandWeaponLocal.rotate.data(), sizeof(record.rightHandWeaponLocal.rotate));
        mixBytes(hash, record.rightHandWeaponLocal.translate.data(), sizeof(record.rightHandWeaponLocal.translate));
        mixValue(hash, record.rightHandWeaponLocal.scale);
        for (const auto& finger : record.rightFiringFingerLocals) {
            mixBytes(hash, finger.rotate.data(), sizeof(finger.rotate));
            mixBytes(hash, finger.translate.data(), sizeof(finger.translate));
            mixValue(hash, finger.scale);
        }
        mixValue(hash, record.rightFiringFingerMask);
        mixBytes(hash, record.supportHandWeaponLocal.rotate.data(), sizeof(record.supportHandWeaponLocal.rotate));
        mixBytes(hash, record.supportHandWeaponLocal.translate.data(), sizeof(record.supportHandWeaponLocal.translate));
        mixValue(hash, record.supportHandWeaponLocal.scale);
        for (const auto& finger : record.supportFingerLocals) {
            mixBytes(hash, finger.rotate.data(), sizeof(finger.rotate));
            mixBytes(hash, finger.translate.data(), sizeof(finger.translate));
            mixValue(hash, finger.scale);
        }
        mixValue(hash, record.supportFingerMask);
        mixValue(hash, record.supportValid);
        mixString(hash, record.idleClipPath);
        mixValue(hash, record.requestedSubgraphIdentifier);
        mixValue(hash, record.bindingSubgraphIdentifier);
        mixValue(hash, record.quality.sampleCount);
        mixValue(hash, record.quality.selectedTimeSeconds);
        mixValue(hash, record.quality.durationSeconds);
        mixValue(hash, record.quality.maxHandTranslationDelta);
        mixValue(hash, record.quality.maxHandRotationDeltaDegrees);
        mixValue(hash, record.quality.maxFingerTranslationDelta);
        mixValue(hash, record.quality.maxFingerRotationDeltaDegrees);
        mixValue(hash, record.quality.maxScaleDelta);
        mixValue(hash, record.quality.stable);
        return hash;
    }

    bool validRecord(const CacheRecord& record) noexcept
    {
        if (record.formatVersion != kFormatVersion || record.poseAlgorithmVersion != kPoseAlgorithmVersion || !record.key.valid() ||
            !validTransform(record.rightHandWeaponLocal) || record.rightFiringFingerMask != kCompleteFiringFingerMask ||
            record.idleClipPath.empty() || record.idleClipPath.size() >= 260 || record.requestedSubgraphIdentifier == 0 ||
            !record.quality.stable || record.quality.sampleCount != kRequiredPersistenceSamples || !finiteBounded(record.quality.selectedTimeSeconds, 600.0f) ||
            !finiteBounded(record.quality.durationSeconds, 600.0f) || record.quality.durationSeconds <= 0.0f ||
            record.quality.selectedTimeSeconds < 0.0f || record.quality.selectedTimeSeconds > record.quality.durationSeconds ||
            !finiteBounded(record.quality.maxHandTranslationDelta, 1000000.0f) ||
            !finiteBounded(record.quality.maxHandRotationDeltaDegrees, 360.0f) ||
            !finiteBounded(record.quality.maxFingerTranslationDelta, 1000000.0f) ||
            !finiteBounded(record.quality.maxFingerRotationDeltaDegrees, 360.0f) ||
            !finiteBounded(record.quality.maxScaleDelta, 100.0f)) {
            return false;
        }
        for (const auto& finger : record.rightFiringFingerLocals) {
            if (!validTransform(finger)) {
                return false;
            }
        }
        if (record.supportValid) {
            if (record.supportFingerMask != kCompleteFiringFingerMask ||
                !validTransform(record.supportHandWeaponLocal)) {
                return false;
            }
            for (const auto& finger : record.supportFingerLocals) {
                if (!validTransform(finger)) {
                    return false;
                }
            }
        } else if (record.supportFingerMask != 0) {
            return false;
        }
        return record.checksum != 0 && record.checksum == calculateChecksum(record);
    }

    std::string serialize(const CacheRecord& record)
    {
        CacheRecord normalized = record;
        normalized.formatVersion = kFormatVersion;
        normalized.poseAlgorithmVersion = kPoseAlgorithmVersion;
        normalized.checksum = 0;
        normalized.checksum = calculateChecksum(normalized);

        json fingers = json::array();
        for (const auto& finger : normalized.rightFiringFingerLocals) {
            fingers.push_back(transformToJson(finger));
        }
        json supportFingers = json::array();
        for (const auto& finger : normalized.supportFingerLocals) {
            supportFingers.push_back(transformToJson(finger));
        }

        const json root{
            { "formatVersion", normalized.formatVersion },
            { "poseAlgorithmVersion", normalized.poseAlgorithmVersion },
            { "weapon", {
                { "plugin", normalized.key.weapon.plugin },
                { "localFormId", normalized.key.weapon.localFormId },
            } },
            { "identity", {
                { "pGripVariantKey", hex64(normalized.key.pGripVariantKey) },
                { "instanceContentKey", hex64(normalized.key.instanceContentKey) },
                { "graphProfileKey", hex64(normalized.key.graphProfileKey) },
                { "powerArmor", normalized.key.inPowerArmor },
            } },
            { "provenance", {
                { "idleClipPath", normalized.idleClipPath },
                { "requestedSubgraphIdentifier", hex64(normalized.requestedSubgraphIdentifier) },
                { "bindingSubgraphIdentifier", hex64(normalized.bindingSubgraphIdentifier) },
            } },
            { "quality", {
                { "sampleCount", normalized.quality.sampleCount },
                { "selectedTimeSeconds", normalized.quality.selectedTimeSeconds },
                { "durationSeconds", normalized.quality.durationSeconds },
                { "maxHandTranslationDelta", normalized.quality.maxHandTranslationDelta },
                { "maxHandRotationDeltaDegrees", normalized.quality.maxHandRotationDeltaDegrees },
                { "maxFingerTranslationDelta", normalized.quality.maxFingerTranslationDelta },
                { "maxFingerRotationDeltaDegrees", normalized.quality.maxFingerRotationDeltaDegrees },
                { "maxScaleDelta", normalized.quality.maxScaleDelta },
                { "stable", normalized.quality.stable },
            } },
            { "rightHandWeaponLocal", transformToJson(normalized.rightHandWeaponLocal) },
            { "rightFiringFingerMask", normalized.rightFiringFingerMask },
            { "rightFiringFingerLocals", std::move(fingers) },
            { "support", {
                { "valid", normalized.supportValid },
                { "handWeaponLocal", transformToJson(normalized.supportHandWeaponLocal) },
                { "fingerMask", normalized.supportFingerMask },
                { "fingerLocals", std::move(supportFingers) },
            } },
            { "checksum", hex64(normalized.checksum) },
        };
        return root.dump(2);
    }

    bool parse(const std::string_view jsonText, CacheRecord& out, std::string* outError)
    {
        out = {};
        if (outError) {
            outError->clear();
        }
        if (jsonText.empty() || jsonText.size() > kMaximumRecordBytes) {
            setError(outError, "recordSizeInvalid");
            return false;
        }

        try {
            const json root = json::parse(jsonText);
            if (!root.is_object() || !root.contains("formatVersion") || !root.contains("poseAlgorithmVersion") || !root.contains("weapon") ||
                !root.contains("identity") || !root.contains("provenance") || !root.contains("quality") || !root.contains("rightHandWeaponLocal") ||
                !root.contains("rightFiringFingerMask") || !root.contains("rightFiringFingerLocals") || !root.contains("support") ||
                !root.contains("checksum")) {
                setError(outError, "recordFieldsMissing");
                return false;
            }

            CacheRecord parsed{};
            parsed.formatVersion = root.at("formatVersion").get<std::uint32_t>();
            parsed.poseAlgorithmVersion = root.at("poseAlgorithmVersion").get<std::uint32_t>();
            if (parsed.formatVersion != kFormatVersion || parsed.poseAlgorithmVersion != kPoseAlgorithmVersion) {
                setError(outError, "recordVersionUnsupported");
                return false;
            }

            const auto& weapon = root.at("weapon");
            const auto& identity = root.at("identity");
            const auto& provenance = root.at("provenance");
            const auto& quality = root.at("quality");
            parsed.key.weapon.plugin = weapon.at("plugin").get<std::string>();
            parsed.key.weapon.localFormId = weapon.at("localFormId").get<std::uint32_t>();
            parsed.key.inPowerArmor = identity.at("powerArmor").get<bool>();
            if (!parseHex64(identity.at("pGripVariantKey"), parsed.key.pGripVariantKey) ||
                !parseHex64(identity.at("instanceContentKey"), parsed.key.instanceContentKey) ||
                !parseHex64(identity.at("graphProfileKey"), parsed.key.graphProfileKey) ||
                !parseHex64(provenance.at("requestedSubgraphIdentifier"), parsed.requestedSubgraphIdentifier) ||
                !parseHex64(provenance.at("bindingSubgraphIdentifier"), parsed.bindingSubgraphIdentifier) ||
                !parseHex64(root.at("checksum"), parsed.checksum)) {
                setError(outError, "recordHexFieldInvalid");
                return false;
            }

            parsed.idleClipPath = provenance.at("idleClipPath").get<std::string>();
            parsed.quality.sampleCount = quality.at("sampleCount").get<std::uint32_t>();
            parsed.quality.selectedTimeSeconds = quality.at("selectedTimeSeconds").get<float>();
            parsed.quality.durationSeconds = quality.at("durationSeconds").get<float>();
            parsed.quality.maxHandTranslationDelta = quality.at("maxHandTranslationDelta").get<float>();
            parsed.quality.maxHandRotationDeltaDegrees = quality.at("maxHandRotationDeltaDegrees").get<float>();
            parsed.quality.maxFingerTranslationDelta = quality.at("maxFingerTranslationDelta").get<float>();
            parsed.quality.maxFingerRotationDeltaDegrees = quality.at("maxFingerRotationDeltaDegrees").get<float>();
            parsed.quality.maxScaleDelta = quality.at("maxScaleDelta").get<float>();
            parsed.quality.stable = quality.at("stable").get<bool>();
            parsed.rightFiringFingerMask = root.at("rightFiringFingerMask").get<std::uint16_t>();
            if (!transformFromJson(root.at("rightHandWeaponLocal"), parsed.rightHandWeaponLocal)) {
                setError(outError, "recordPrimaryTransformInvalid");
                return false;
            }

            const auto& fingers = root.at("rightFiringFingerLocals");
            if (!fingers.is_array() || fingers.size() != parsed.rightFiringFingerLocals.size()) {
                setError(outError, "recordFingerCountInvalid");
                return false;
            }
            for (std::size_t index = 0; index < parsed.rightFiringFingerLocals.size(); ++index) {
                if (!transformFromJson(fingers[index], parsed.rightFiringFingerLocals[index])) {
                    setError(outError, "recordFingerTransformInvalid");
                    return false;
                }
            }

            const auto& support = root.at("support");
            if (!support.is_object() || !support.contains("valid") || !support.contains("handWeaponLocal") ||
                !support.contains("fingerMask") || !support.contains("fingerLocals")) {
                setError(outError, "recordSupportFieldsMissing");
                return false;
            }
            parsed.supportValid = support.at("valid").get<bool>();
            parsed.supportFingerMask = support.at("fingerMask").get<std::uint16_t>();
            if (!transformFromJson(support.at("handWeaponLocal"), parsed.supportHandWeaponLocal)) {
                setError(outError, "recordSupportTransformInvalid");
                return false;
            }
            const auto& supportFingers = support.at("fingerLocals");
            if (!supportFingers.is_array() || supportFingers.size() != parsed.supportFingerLocals.size()) {
                setError(outError, "recordSupportFingerCountInvalid");
                return false;
            }
            for (std::size_t index = 0; index < parsed.supportFingerLocals.size(); ++index) {
                if (!transformFromJson(supportFingers[index], parsed.supportFingerLocals[index])) {
                    setError(outError, "recordSupportFingerTransformInvalid");
                    return false;
                }
            }
            if (!validRecord(parsed)) {
                setError(outError, "recordValidationFailed");
                return false;
            }
            out = std::move(parsed);
            return true;
        } catch (const std::exception&) {
            setError(outError, "recordParseFailed");
            return false;
        }
    }
}
