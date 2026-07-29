#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

#include <nlohmann/json.hpp>

#include <charconv>
#include <cstdio>

namespace rock::saved_grab_offset
{
    namespace
    {
        using nlohmann::json;

        // Ids serialize as hex strings ("0x0004822D"): human-readable and
        // immune to any JSON integer-precision surprises in external tools.
        std::string formIdToHex(std::uint32_t id)
        {
            char buffer[16]{};
            std::snprintf(buffer, sizeof(buffer), "0x%08X", id);
            return buffer;
        }

        bool hexToFormId(std::string_view text, std::uint32_t& out)
        {
            if (text.size() > 2 && (text[1] == 'x' || text[1] == 'X') && text[0] == '0') {
                text.remove_prefix(2);
            }
            const auto result = std::from_chars(text.data(), text.data() + text.size(), out, 16);
            return result.ec == std::errc{} && result.ptr == text.data() + text.size();
        }

        json formRefToJson(const FormRef& ref)
        {
            return json{ { "plugin", ref.plugin }, { "id", formIdToHex(ref.localFormId) } };
        }

        bool formRefFromJson(const json& j, FormRef& out)
        {
            if (!j.is_object() || !j.contains("plugin") || !j["plugin"].is_string() ||
                !j.contains("id") || !j["id"].is_string()) {
                return false;
            }
            out.plugin = j["plugin"].get<std::string>();
            const auto idText = j["id"].get<std::string>();
            return hexToFormId(idText, out.localFormId);
        }

        json handOffsetToJson(const HandOffset& offset)
        {
            json translate = json::array();
            for (const float value : offset.translateGame) {
                translate.push_back(value);
            }
            json rotate = json::array();
            for (const float value : offset.rotate) {
                rotate.push_back(value);
            }
            json j{ { "translate", std::move(translate) }, { "rotate", std::move(rotate) } };

            if (offset.hasFingerPose) {
                json fingerValues = json::array();
                for (const float value : offset.fingerValues) {
                    fingerValues.push_back(value);
                }
                json finger{ { "values", std::move(fingerValues) } };
                if (offset.hasFingerJointValues) {
                    json jointValues = json::array();
                    for (const float value : offset.fingerJointValues) {
                        jointValues.push_back(value);
                    }
                    finger["jointValues"] = std::move(jointValues);
                }
                j["finger"] = std::move(finger);
            }
            return j;
        }

        // Reads a fixed-size float array field; leaves out untouched and returns
        // false on any missing/wrong-size/non-numeric entry (fail closed).
        template <std::size_t N>
        bool floatArrayFromJson(const json& j, const char* key, float (&out)[N])
        {
            if (!j.contains(key) || !j[key].is_array() || j[key].size() != N) {
                return false;
            }
            for (std::size_t i = 0; i < N; ++i) {
                if (!j[key][i].is_number()) {
                    return false;
                }
            }
            for (std::size_t i = 0; i < N; ++i) {
                out[i] = j[key][i].get<float>();
            }
            return true;
        }

        bool handOffsetFromJson(const json& j, HandOffset& out)
        {
            if (!j.is_object() || !j.contains("translate") || !j["translate"].is_array() || j["translate"].size() != 3 ||
                !j.contains("rotate") || !j["rotate"].is_array() || j["rotate"].size() != 9) {
                return false;
            }
            for (std::size_t i = 0; i < 3; ++i) {
                if (!j["translate"][i].is_number()) {
                    return false;
                }
                out.translateGame[i] = j["translate"][i].get<float>();
            }
            for (std::size_t i = 0; i < 9; ++i) {
                if (!j["rotate"][i].is_number()) {
                    return false;
                }
                out.rotate[i] = j["rotate"][i].get<float>();
            }
            out.present = true;

            out.hasFingerPose = false;
            out.hasFingerJointValues = false;
            if (j.contains("finger") && j["finger"].is_object()) {
                const auto& fingerJson = j["finger"];
                out.hasFingerPose = floatArrayFromJson(fingerJson, "values", out.fingerValues);
                if (out.hasFingerPose && fingerJson.contains("jointValues")) {
                    out.hasFingerJointValues = floatArrayFromJson(fingerJson, "jointValues", out.fingerJointValues);
                }
            }
            return true;
        }
    }

    std::string serialize(const SavedGrabOffsetFile& file)
    {
        json j;
        j["format"] = file.formatVersion;
        j["object"] = formRefToJson(file.object);
        j["objectName"] = file.objectName;
        if (file.right.present) {
            j["right"] = handOffsetToJson(file.right);
        }
        if (file.left.present) {
            j["left"] = handOffsetToJson(file.left);
        }
        // 2-space indent: these files are the hand-tuning surface.
        return j.dump(2);
    }

    bool parse(std::string_view jsonText, SavedGrabOffsetFile& out, std::string* outError)
    {
        out = SavedGrabOffsetFile{};
        if (outError) {
            outError->clear();
        }
        const json j = json::parse(jsonText, nullptr, /*allow_exceptions=*/false);
        if (j.is_discarded() || !j.is_object()) {
            if (outError) {
                *outError = "not valid JSON";
            }
            return false;
        }
        out.formatVersion = j.value("format", 0u);
        if (out.formatVersion == 0 || out.formatVersion > kFormatVersion) {
            if (outError) {
                *outError = "unsupported format version";
            }
            return false;
        }
        if (!j.contains("object") || !formRefFromJson(j["object"], out.object) || out.object.empty()) {
            if (outError) {
                *outError = "missing/invalid object identity";
            }
            return false;
        }
        out.objectName = j.value("objectName", std::string{});
        out.right = {};
        if (j.contains("right") && !handOffsetFromJson(j["right"], out.right)) {
            if (outError) {
                *outError = "invalid right-hand offset (skipped)";
            }
            out.right = {};
        }
        out.left = {};
        if (j.contains("left") && !handOffsetFromJson(j["left"], out.left)) {
            if (outError) {
                *outError = "invalid left-hand offset (skipped)";
            }
            out.left = {};
        }
        return true;
    }
}
