#include "setup-wizard/SteamVrBinding.h"

#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string_view>
#include <system_error>
#include <utility>

#include <openvr.h>

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "setup-wizard/SteamVrBindingResources.h"

namespace rock::steamvr_binding
{
    namespace
    {
        namespace policy = steamvr_binding_policy;

        // Folder beside ROCK's settings that holds the installed templates.
        constexpr std::string_view kFolder = "SteamVR";

        /*
         * True when the interface's first virtual function lives in SteamVR's client runtime.
         * OpenComposite serves the OpenVR interfaces from its own openvr_api.dll, also when its
         * OpenXR runtime is SteamVR's (which loads vrclient_x64.dll as well), so this is what
         * tells that the settings ROCK would write are SteamVR's.
         */
        [[nodiscard]] bool servedBySteamVr(const void* object) noexcept
        {
            const auto client = GetModuleHandleW(L"vrclient_x64.dll");
            if (!object || !client) return false;
            const auto* const* vtable = *static_cast<const void* const* const*>(object);
            if (!vtable || !vtable[0]) return false;
            HMODULE owner = nullptr;
            return GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       static_cast<LPCWSTR>(vtable[0]), &owner) &&
                owner == client;
        }

        // The first controller SteamVR reports a type for, right hand first.
        [[nodiscard]] std::string controllerType(vr::IVRSystem& system)
        {
            for (const auto role : { vr::TrackedControllerRole_RightHand, vr::TrackedControllerRole_LeftHand }) {
                const auto index = system.GetTrackedDeviceIndexForControllerRole(role);
                if (index == vr::k_unTrackedDeviceIndexInvalid) continue;
                std::array<char, 64> buffer{};
                vr::ETrackedPropertyError error{ vr::TrackedProp_Success };
                system.GetStringTrackedDeviceProperty(index, vr::Prop_ControllerType_String, buffer.data(),
                    static_cast<std::uint32_t>(buffer.size()), &error);
                if (error == vr::TrackedProp_Success && buffer[0]) return buffer.data();
            }
            return {};
        }

        [[nodiscard]] std::string_view embeddedTemplate(policy::Controller controller) noexcept
        {
            HMODULE self = nullptr;
            if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                    reinterpret_cast<LPCWSTR>(&embeddedTemplate), &self) ||
                !self) {
                return {};
            }
            const int id = controller == policy::Controller::Index ? ROCK_RC_STEAMVR_BINDING_KNUCKLES
                                                                    : ROCK_RC_STEAMVR_BINDING_VIVE_CONTROLLER;
            const auto info = FindResourceW(self, MAKEINTRESOURCEW(id), MAKEINTRESOURCEW(10));  // RT_RCDATA
            if (!info) return {};
            const auto handle = LoadResource(self, info);
            const auto* data = handle ? static_cast<const char*>(LockResource(handle)) : nullptr;
            return data ? std::string_view{ data, SizeofResource(self, info) } : std::string_view{};
        }

        [[nodiscard]] std::filesystem::path installedPath(const policy::ControllerInfo& controller)
        {
            return g_rockConfig.getConfigDirectory() / kFolder / controller.fileName;
        }

        [[nodiscard]] std::string utf8(const std::filesystem::path& path)
        {
            const auto text = path.u8string();
            return { reinterpret_cast<const char*>(text.data()), text.size() };
        }

        // Writes content to path unless the file already holds exactly that.
        [[nodiscard]] bool writeTemplate(const std::filesystem::path& path, std::string_view content, std::string& error)
        {
            {
                std::ifstream existing(path, std::ios::binary);
                if (existing) {
                    const std::string current{ std::istreambuf_iterator<char>(existing), std::istreambuf_iterator<char>() };
                    if (current == content) return true;
                }
            }
            std::error_code ec;
            std::filesystem::create_directories(path.parent_path(), ec);
            if (ec) {
                error = "could not create " + utf8(path.parent_path()) + ": " + ec.message();
                return false;
            }
            std::ofstream file(path, std::ios::binary | std::ios::trunc);
            file.write(content.data(), static_cast<std::streamsize>(content.size()));
            file.close();
            if (!file) {
                error = "could not write " + utf8(path);
                return false;
            }
            return true;
        }

        /*
         * Once per game start: rewrites an installed template that differs from the one in this
         * ROCK.dll, so a ROCK update reaches players who already use ROCK's binding. Checked by
         * file, not by SteamVR's report: while ROCK's binding is active SteamVR reports the
         * simulated controller type (oculus_touch, bench: Index with the template), so the
         * inspection never sees the knuckles setting then.
         */
        void refreshInstalledTemplates()
        {
            static bool done = false;
            if (std::exchange(done, true)) return;
            for (const auto type : { std::string_view{ "knuckles" }, std::string_view{ "vive_controller" } }) {
                const auto controller = policy::controllerFor(type);
                if (!controller) continue;
                const auto path = installedPath(*controller);
                std::error_code ec;
                if (!std::filesystem::exists(path, ec)) continue;
                const auto content = embeddedTemplate(controller->controller);
                std::string error;
                if (!content.empty() && !writeTemplate(path, content, error))
                    ROCK_LOG_WARN(Config, "SteamVR binding: could not refresh {}: {}", controller->fileName, error);
            }
        }

        // Logs an inspection when it differs from the last one logged.
        void report(const Inspection& inspection, std::string_view note)
        {
            static std::string last;
            std::string line{ note };
            if (inspection.controller) {
                line += "; controller ";
                line += inspection.controller->controllerType;
                line += ", setting \"";
                line += inspection.currentUrl;
                line += '"';
            }
            if (line == last) return;
            last = line;
            ROCK_LOG_INFO(Config, "SteamVR binding: {}", line);
        }
    }

    Inspection inspect() noexcept
    {
        Inspection result{};
        try {
            refreshInstalledTemplates();
            auto* system = vr::VRSystem();
            auto* settings = vr::VRSettings();
            auto* applications = vr::VRApplications();
            if (!system || !settings || !applications || !servedBySteamVr(settings)) {
                report(result, "not SteamVR's settings (OpenComposite or no OpenVR); nothing to offer");
                return result;
            }
            const auto type = controllerType(*system);
            result.controller = policy::controllerFor(type);
            if (!result.controller) {
                report(result, "controller \"" + type + "\" needs no binding");
                return result;
            }
            std::array<char, vr::k_unMaxApplicationKeyLength> key{};
            if (applications->GetApplicationKeyByProcessId(GetCurrentProcessId(), key.data(),
                    static_cast<std::uint32_t>(key.size())) != vr::VRApplicationError_None ||
                !key[0]) {
                report(result, "SteamVR did not name this game's app key; nothing offered");
                return result;
            }
            result.appKey = key.data();
            std::array<char, 1024> url{};
            vr::EVRSettingsError error{ vr::VRSettingsError_None };
            settings->GetString(result.appKey.c_str(), policy::settingKey(result.controller->controllerType).c_str(), url.data(),
                static_cast<std::uint32_t>(url.size()), &error);
            if (error != vr::VRSettingsError_None && error != vr::VRSettingsError_UnsetSettingHasNoDefault) {
                report(result, std::string("SteamVR setting unreadable (") + settings->GetSettingsErrorNameFromEnum(error) + ")");
                return result;
            }
            result.currentUrl = url.data();
            result.status = policy::classify(true, result.controller, result.currentUrl);
            report(result, result.status == policy::Status::Offer ? "SteamVR's default binding; ROCK offers its own"
                    : result.status == policy::Status::RockActive ? "ROCK's binding is set"
                                                                   : "the player's own binding is set; left alone");
        } catch (...) {
            ROCK_LOG_WARN(Config, "SteamVR binding: inspection failed");
            return {};
        }
        return result;
    }

    bool apply(const Inspection& inspection, std::string& error) noexcept
    {
        try {
            if (inspection.status != policy::Status::Offer || !inspection.controller || inspection.appKey.empty()) {
                error = "there is nothing to set";
                return false;
            }
            auto* settings = vr::VRSettings();
            if (!settings || !servedBySteamVr(settings)) {
                error = "SteamVR is not reachable";
                return false;
            }
            const auto content = embeddedTemplate(inspection.controller->controller);
            if (content.empty()) {
                error = "ROCK.dll is missing its SteamVR binding";
                return false;
            }
            const auto path = installedPath(*inspection.controller);
            if (!writeTemplate(path, content, error)) return false;
            const auto url = policy::fileUrl(utf8(path));
            const auto key = policy::settingKey(inspection.controller->controllerType);
            vr::EVRSettingsError settingError{ vr::VRSettingsError_None };
            settings->SetString(inspection.appKey.c_str(), key.c_str(), url.c_str(), &settingError);
            if (settingError != vr::VRSettingsError_None) {
                error = std::string("SteamVR refused the setting (") + settings->GetSettingsErrorNameFromEnum(settingError) + ")";
                return false;
            }
            std::array<char, 1024> readBack{};
            settings->GetString(inspection.appKey.c_str(), key.c_str(), readBack.data(), static_cast<std::uint32_t>(readBack.size()),
                &settingError);
            if (settingError != vr::VRSettingsError_None || url != readBack.data()) {
                error = "SteamVR did not keep the setting";
                return false;
            }
            ROCK_LOG_INFO(Config, "SteamVR binding: set {}.{} = {}", inspection.appKey, key, url);
            return true;
        } catch (...) {
            error = "unexpected failure";
            return false;
        }
    }
}
