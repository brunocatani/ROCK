#include "monitor/GrabClockMonitorConfig.h"

#include <ShlObj.h>
#include <SimpleIni.h>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <condition_variable>
#include <cstdlib>
#include <limits>

#ifndef MAX_PATH
#define ROCK_MONITOR_DEFINED_MAX_PATH_FOR_FILEWATCH 1
#define MAX_PATH 260
#endif
#include <thomasmonkman-filewatch/FileWatch.hpp>
#if defined(ROCK_MONITOR_DEFINED_MAX_PATH_FOR_FILEWATCH)
#undef MAX_PATH
#undef ROCK_MONITOR_DEFINED_MAX_PATH_FOR_FILEWATCH
#endif

namespace rock::monitor
{
    namespace
    {
        constexpr auto kSection = "PanelPose";
        constexpr float kPositionMinimum = -100.0f;
        constexpr float kPositionMaximum = 100.0f;
        constexpr float kRotationMinimumDegrees = -180.0f;
        constexpr float kRotationMaximumDegrees = 180.0f;
        constexpr auto kReloadQuietDelay = std::chrono::milliseconds(60);
        constexpr auto kReloadMaximumDeferral = std::chrono::milliseconds(100);
        constexpr auto kReloadRetryDelay = std::chrono::milliseconds(75);
        constexpr auto kSuccessfulReloadLogInterval = std::chrono::seconds(1);
        constexpr std::uint32_t kReloadAttempts = 3;
        constexpr std::size_t kMaximumIniBytes = 1024 * 1024;
        constexpr std::array<std::string_view, 10> kKnownKeys{
            "fPositionX",
            "fPositionY",
            "fPositionZ",
            "fRotationXDegrees",
            "fRotationYDegrees",
            "fRotationZDegrees",
            "bFlipRotationX",
            "bFlipRotationY",
            "bFlipRotationZ",
            "bEnabled",
        };
        constexpr std::size_t kEnabledKeyIndex = 9;
        constexpr std::array<std::size_t, 4> kOptionalBooleanKeyIndices{
            kEnabledKeyIndex,
            6,
            7,
            8,
        };
        constexpr std::array<bool, 4> kOptionalBooleanDefaults{
            true,
            false,
            false,
            false,
        };

        constexpr std::string_view kDefaultIniTemplate =
            "; ROCK Monitor runtime and panel pose. Changes are hot-reloaded while the game is running.\n"
            "; Set bEnabled to false to suspend the Monitor panel without unloading ROCK.\n"
            "; Position uses right-hand-local game units: +X finger-forward, +Y lateral, +Z up.\n"
            "; Rotation is panel-local in degrees and is applied X, then Y, then Z.\n"
            "[PanelPose]\n"
            "bEnabled = false\n"
            "fPositionX = 7.75\n"
            "fPositionY = 9.0\n"
            "fPositionZ = -16.5\n"
            "fRotationXDegrees = 0.0\n"
            "fRotationYDegrees = 90.0\n"
            "fRotationZDegrees = 6.0\n"
            "; Flips are postmultiplied after the full Euler orientation around final panel-local axes.\n"
            "; When several flips are enabled, their postmultiply order is X, then Y, then Z.\n"
            "bFlipRotationX = true\n"
            "bFlipRotationY = true\n"
            "bFlipRotationZ = false\n";

        [[nodiscard]] std::filesystem::path resolveIniPath()
        {
            char documents[MAX_PATH]{};
            if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_MYDOCUMENTS, nullptr, SHGFP_TYPE_CURRENT, documents))) {
                std::filesystem::path path = documents;
                path /= "My Games";
                path /= "Fallout4VR";
                path /= "ROCK_Config";
                path /= "ROCKMonitor.ini";
                return path;
            }
            return std::filesystem::path("Data") / "F4SE" / "Plugins" / "ROCKMonitor.ini";
        }

        [[nodiscard]] bool ensureDefaultIniExists(const std::filesystem::path& path)
        {
            std::error_code directoryError;
            std::filesystem::create_directories(path.parent_path(), directoryError);
            if (directoryError) {
                logger::warn("ROCK Monitor: could not create config directory '{}' ({}).",
                    path.parent_path().string(), directoryError.message());
                return false;
            }

            const auto pathText = path.string();
            const HANDLE file = CreateFileA(
                pathText.c_str(),
                GENERIC_WRITE,
                FILE_SHARE_READ,
                nullptr,
                CREATE_NEW,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);
            if (file == INVALID_HANDLE_VALUE) {
                const auto error = GetLastError();
                if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS) {
                    return true;
                }
                logger::warn("ROCK Monitor: could not create default config '{}' (Windows error {}).", pathText, error);
                return false;
            }

            DWORD bytesWritten = 0;
            const bool writeSucceeded = WriteFile(
                                            file,
                                            kDefaultIniTemplate.data(),
                                            static_cast<DWORD>(kDefaultIniTemplate.size()),
                                            &bytesWritten,
                                            nullptr) != FALSE;
            const bool completeWrite = writeSucceeded && bytesWritten == static_cast<DWORD>(kDefaultIniTemplate.size());
            const bool flushed = completeWrite && FlushFileBuffers(file) != FALSE;
            auto writeError = flushed ? ERROR_SUCCESS : GetLastError();
            if (!flushed && writeError == ERROR_SUCCESS) {
                writeError = ERROR_WRITE_FAULT;
            }
            CloseHandle(file);
            if (!flushed) {
                (void)DeleteFileA(pathText.c_str());
                logger::warn("ROCK Monitor: default config write failed for '{}' (Windows error {}).", pathText, writeError);
                return false;
            }

            logger::info("ROCK Monitor: created default config at '{}'.", pathText);
            return true;
        }

        [[nodiscard]] bool parseBoundedFloat(
            const CSimpleIniA& ini,
            const char* key,
            float minimum,
            float maximum,
            float& result,
            std::string& error)
        {
            const char* raw = ini.GetValue(kSection, key, nullptr);
            if (!raw) {
                error = std::format("missing [{}] {}", kSection, key);
                return false;
            }

            errno = 0;
            char* end = nullptr;
            const float parsed = std::strtof(raw, &end);
            while (end && *end != '\0' && std::isspace(static_cast<unsigned char>(*end)) != 0) {
                ++end;
            }
            if (errno == ERANGE || end == raw || !end || *end != '\0' || !std::isfinite(parsed)) {
                error = std::format("[{}] {} is not a finite number", kSection, key);
                return false;
            }
            if (parsed < minimum || parsed > maximum) {
                error = std::format("[{}] {} must be in [{:.0f}, {:.0f}]", kSection, key, minimum, maximum);
                return false;
            }

            result = parsed;
            return true;
        }

        [[nodiscard]] std::string_view trimAscii(std::string_view value)
        {
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())) != 0) {
                value.remove_prefix(1);
            }
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
                value.remove_suffix(1);
            }
            return value;
        }

        [[nodiscard]] bool equalsIgnoreCase(std::string_view left, std::string_view right)
        {
            if (left.size() != right.size()) {
                return false;
            }
            for (std::size_t i = 0; i < left.size(); ++i) {
                auto leftCharacter = static_cast<unsigned char>(left[i]);
                auto rightCharacter = static_cast<unsigned char>(right[i]);
                if (leftCharacter >= 'A' && leftCharacter <= 'Z') {
                    leftCharacter = static_cast<unsigned char>(leftCharacter - 'A' + 'a');
                }
                if (rightCharacter >= 'A' && rightCharacter <= 'Z') {
                    rightCharacter = static_cast<unsigned char>(rightCharacter - 'A' + 'a');
                }
                if (leftCharacter != rightCharacter) {
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool parseStrictBool(
            const CSimpleIniA& ini,
            const char* key,
            bool defaultValue,
            bool& result,
            bool& missing,
            std::string& error)
        {
            const char* raw = ini.GetValue(kSection, key, nullptr);
            if (!raw) {
                result = defaultValue;
                missing = true;
                return true;
            }

            missing = false;
            const auto value = trimAscii(raw);
            if (equalsIgnoreCase(value, "true")) {
                result = true;
                return true;
            }
            if (equalsIgnoreCase(value, "false")) {
                result = false;
                return true;
            }

            error = std::format("[{}] {} must be exactly true or false", kSection, key);
            return false;
        }

        [[nodiscard]] bool readFileBytes(
            const std::filesystem::path& path,
            std::string& bytes,
            std::string& error)
        {
            const auto pathText = path.string();
            const HANDLE file = CreateFileA(
                pathText.c_str(),
                GENERIC_READ,
                FILE_SHARE_READ | FILE_SHARE_DELETE,
                nullptr,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);
            if (file == INVALID_HANDLE_VALUE) {
                error = std::format("could not open '{}' for a bounded config snapshot (Windows error {})", pathText, GetLastError());
                return false;
            }

            LARGE_INTEGER fileSize{};
            if (!GetFileSizeEx(file, &fileSize)) {
                const auto readError = GetLastError();
                CloseHandle(file);
                error = std::format("could not size '{}' for a bounded config snapshot (Windows error {})", pathText, readError);
                return false;
            }
            if (fileSize.QuadPart < 0 || static_cast<std::uint64_t>(fileSize.QuadPart) > kMaximumIniBytes) {
                CloseHandle(file);
                error = std::format("'{}' exceeds the {} byte config safety limit", pathText, kMaximumIniBytes);
                return false;
            }

            bytes.assign(static_cast<std::size_t>(fileSize.QuadPart), '\0');
            std::size_t totalRead = 0;
            while (totalRead < bytes.size()) {
                const auto remaining = bytes.size() - totalRead;
                const auto request = static_cast<DWORD>((std::min)(
                    remaining,
                    static_cast<std::size_t>((std::numeric_limits<DWORD>::max)())));
                DWORD bytesRead = 0;
                if (!ReadFile(file, bytes.data() + totalRead, request, &bytesRead, nullptr) || bytesRead == 0) {
                    const auto readError = GetLastError();
                    CloseHandle(file);
                    error = std::format("could not read '{}' for a bounded config snapshot (Windows error {})", pathText, readError);
                    return false;
                }
                totalRead += bytesRead;
            }
            CloseHandle(file);
            return true;
        }

        struct IniTextScan
        {
            std::size_t panelPoseSectionCount{ 0 };
            std::array<std::size_t, kKnownKeys.size()> knownKeyCounts{};
            std::size_t insertionOffset{ 0 };
            bool insertionFollowsNewline{ false };
            std::string newline;
        };

        [[nodiscard]] IniTextScan scanIniText(std::string_view bytes)
        {
            IniTextScan scan{};
            std::string firstNewline;
            bool inPanelPoseSection = false;
            std::size_t cursor = 0;
            while (cursor < bytes.size()) {
                const auto lineBegin = cursor;
                while (cursor < bytes.size() && bytes[cursor] != '\r' && bytes[cursor] != '\n') {
                    ++cursor;
                }
                const auto contentEnd = cursor;
                if (cursor < bytes.size()) {
                    if (bytes[cursor] == '\r' && cursor + 1 < bytes.size() && bytes[cursor + 1] == '\n') {
                        cursor += 2;
                    } else {
                        ++cursor;
                    }
                    if (firstNewline.empty()) {
                        firstNewline.assign(bytes.substr(contentEnd, cursor - contentEnd));
                    }
                }
                const auto lineEnd = cursor;
                auto content = bytes.substr(lineBegin, contentEnd - lineBegin);
                if (lineBegin == 0 && content.starts_with("\xEF\xBB\xBF")) {
                    content.remove_prefix(3);
                }
                const auto trimmed = trimAscii(content);

                const bool sectionLine = trimmed.size() >= 2 && trimmed.front() == '[' && trimmed.back() == ']';
                if (sectionLine) {
                    const auto sectionName = trimAscii(trimmed.substr(1, trimmed.size() - 2));
                    inPanelPoseSection = equalsIgnoreCase(sectionName, kSection);
                    if (inPanelPoseSection) {
                        ++scan.panelPoseSectionCount;
                        scan.insertionOffset = lineEnd;
                        scan.insertionFollowsNewline = lineEnd > contentEnd;
                        if (lineEnd > contentEnd) {
                            scan.newline.assign(bytes.substr(contentEnd, lineEnd - contentEnd));
                        }
                    }
                    continue;
                }
                if (!inPanelPoseSection || trimmed.empty() || trimmed.front() == ';' || trimmed.front() == '#') {
                    continue;
                }

                const auto equals = trimmed.find('=');
                if (equals == std::string_view::npos) {
                    continue;
                }
                const auto key = trimAscii(trimmed.substr(0, equals));
                for (std::size_t i = 0; i < kKnownKeys.size(); ++i) {
                    if (equalsIgnoreCase(key, kKnownKeys[i])) {
                        ++scan.knownKeyCounts[i];
                    }
                }
                // Insert after the final setting in the target section, before any
                // trailing comments or blank lines that may describe the next section.
                scan.insertionOffset = lineEnd;
                scan.insertionFollowsNewline = lineEnd > contentEnd;
                if (lineEnd > contentEnd) {
                    scan.newline.assign(bytes.substr(contentEnd, lineEnd - contentEnd));
                }
            }

            if (scan.newline.empty()) {
                scan.newline = firstNewline.empty() ? "\n" : firstNewline;
            }
            return scan;
        }

        [[nodiscard]] bool validateIniTextStructure(const IniTextScan& scan, std::string& error)
        {
            if (scan.panelPoseSectionCount == 0) {
                error = std::format("missing [{}] section", kSection);
                return false;
            }
            if (scan.panelPoseSectionCount > 1) {
                error = std::format("duplicate [{}] sections are not allowed", kSection);
                return false;
            }
            for (std::size_t i = 0; i < kKnownKeys.size(); ++i) {
                if (scan.knownKeyCounts[i] > 1) {
                    error = std::format(
                        "duplicate [{}] {} entries are not allowed",
                        kSection,
                        kKnownKeys[i]);
                    return false;
                }
            }
            return true;
        }

        [[nodiscard]] bool writeMigratedFileIfUnchanged(
            const std::filesystem::path& path,
            std::string_view originalBytes,
            std::string_view migratedBytes,
            std::string& error)
        {
            auto temporaryPath = path;
            temporaryPath += ".migration.tmp";
            const auto temporaryPathText = temporaryPath.string();
            const auto pathText = path.string();
            const HANDLE file = CreateFileA(
                temporaryPathText.c_str(),
                GENERIC_WRITE,
                0,
                nullptr,
                CREATE_ALWAYS,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);
            if (file == INVALID_HANDLE_VALUE) {
                error = std::format("could not create migration temp file (Windows error {})", GetLastError());
                return false;
            }

            std::size_t totalWritten = 0;
            DWORD writeError = ERROR_SUCCESS;
            while (totalWritten < migratedBytes.size()) {
                const auto remaining = migratedBytes.size() - totalWritten;
                const auto request = static_cast<DWORD>((std::min)(
                    remaining,
                    static_cast<std::size_t>((std::numeric_limits<DWORD>::max)())));
                DWORD bytesWritten = 0;
                if (!WriteFile(file, migratedBytes.data() + totalWritten, request, &bytesWritten, nullptr) || bytesWritten == 0) {
                    writeError = GetLastError();
                    if (writeError == ERROR_SUCCESS) {
                        writeError = ERROR_WRITE_FAULT;
                    }
                    break;
                }
                totalWritten += bytesWritten;
            }
            if (writeError == ERROR_SUCCESS && !FlushFileBuffers(file)) {
                writeError = GetLastError();
            }
            CloseHandle(file);
            if (writeError != ERROR_SUCCESS || totalWritten != migratedBytes.size()) {
                (void)DeleteFileA(temporaryPathText.c_str());
                error = std::format("migration temp write failed (Windows error {})",
                    writeError == ERROR_SUCCESS ? ERROR_WRITE_FAULT : writeError);
                return false;
            }

            // Best-effort lost-update guard: compare the exact bounded snapshot again
            // immediately before the atomic replace. Windows does not provide a
            // cross-process compare-and-swap for file contents, so an external writer
            // can still race in the narrow compare-to-replace window.
            std::string currentBytes;
            if (!readFileBytes(path, currentBytes, error) ||
                std::string_view(currentBytes.data(), currentBytes.size()) != originalBytes) {
                (void)DeleteFileA(temporaryPathText.c_str());
                if (error.empty()) {
                    error = "config changed concurrently; migration deferred";
                }
                return false;
            }

            if (!MoveFileExA(
                    temporaryPathText.c_str(),
                    pathText.c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
                const auto replaceError = GetLastError();
                (void)DeleteFileA(temporaryPathText.c_str());
                error = std::format("atomic migration replace failed (Windows error {})", replaceError);
                return false;
            }
            return true;
        }

        [[nodiscard]] std::optional<std::size_t> migrateMissingBooleanKeys(
            const std::filesystem::path& path,
            const std::string& originalBytes,
            const IniTextScan& scan,
            const std::array<bool, 4>& missingKeys,
            std::string& error)
        {
            if (std::none_of(missingKeys.begin(), missingKeys.end(), [](bool missing) { return missing; })) {
                return std::size_t{ 0 };
            }

            std::array<bool, 4> addKey{};
            std::size_t addCount = 0;
            for (std::size_t i = 0; i < addKey.size(); ++i) {
                addKey[i] = missingKeys[i] &&
                    scan.knownKeyCounts[kOptionalBooleanKeyIndices[i]] == 0;
                addCount += addKey[i] ? 1 : 0;
            }
            if (addCount == 0) {
                error = "missing boolean values already have textual key lines; refusing to add duplicates";
                return std::nullopt;
            }

            std::string insertion;
            if (!scan.insertionFollowsNewline) {
                insertion += scan.newline;
            }
            std::size_t keysWritten = 0;
            const bool suffixFollows = scan.insertionOffset < originalBytes.size();
            const bool originalEndsWithNewline = !originalBytes.empty() &&
                                                 (originalBytes.back() == '\r' || originalBytes.back() == '\n');
            for (std::size_t i = 0; i < addKey.size(); ++i) {
                if (!addKey[i]) {
                    continue;
                }
                insertion += kKnownKeys[kOptionalBooleanKeyIndices[i]];
                insertion += (kOptionalBooleanDefaults[i] ? " = true" : " = false");
                ++keysWritten;
                if (keysWritten < addCount || suffixFollows || originalEndsWithNewline) {
                    insertion += scan.newline;
                }
            }

            std::string migratedBytes;
            migratedBytes.reserve(originalBytes.size() + insertion.size());
            migratedBytes.append(originalBytes, 0, scan.insertionOffset);
            migratedBytes += insertion;
            migratedBytes.append(originalBytes, scan.insertionOffset, std::string::npos);
            if (!writeMigratedFileIfUnchanged(path, originalBytes, migratedBytes, error)) {
                return std::nullopt;
            }
            return addCount;
        }

        [[nodiscard]] std::array<float, 4> multiplyQuaternionsRaw(
            const std::array<float, 4>& left,
            const std::array<float, 4>& right)
        {
            return {
                left[3] * right[0] + left[0] * right[3] + left[1] * right[2] - left[2] * right[1],
                left[3] * right[1] - left[0] * right[2] + left[1] * right[3] + left[2] * right[0],
                left[3] * right[2] + left[0] * right[1] - left[1] * right[0] + left[2] * right[3],
                left[3] * right[3] - left[0] * right[0] - left[1] * right[1] - left[2] * right[2],
            };
        }

        [[nodiscard]] std::optional<std::array<float, 4>> normalizeQuaternionRaw(
            std::array<float, 4> orientation)
        {
            float normSquared = 0.0f;
            for (const float component : orientation) {
                if (!std::isfinite(component)) {
                    return std::nullopt;
                }
                normSquared += component * component;
            }
            if (!std::isfinite(normSquared) || normSquared < 0.000001f) {
                return std::nullopt;
            }
            const float inverseNorm = 1.0f / std::sqrt(normSquared);
            for (float& component : orientation) {
                component *= inverseNorm;
            }
            return orientation;
        }

        [[nodiscard]] std::optional<std::array<float, 4>> localOrientationFromEulerDegrees(
            float xDegrees,
            float yDegrees,
            float zDegrees)
        {
            constexpr float kDegreesToHalfRadians = 0.00872664625997164788f;
            const float halfX = xDegrees * kDegreesToHalfRadians;
            const float halfY = yDegrees * kDegreesToHalfRadians;
            const float halfZ = zDegrees * kDegreesToHalfRadians;
            const std::array<float, 4> xRotation{ std::sin(halfX), 0.0f, 0.0f, std::cos(halfX) };
            const std::array<float, 4> yRotation{ 0.0f, std::sin(halfY), 0.0f, std::cos(halfY) };
            const std::array<float, 4> zRotation{ 0.0f, 0.0f, std::sin(halfZ), std::cos(halfZ) };
            // Column-vector convention: qZ * qY * qX applies the configured
            // panel-local X rotation first, then Y, then Z.
            return normalizeQuaternionRaw(
                multiplyQuaternionsRaw(zRotation, multiplyQuaternionsRaw(yRotation, xRotation)));
        }

        [[nodiscard]] std::optional<std::array<float, 4>> applyFinalPanelLocalFlips(
            std::array<float, 4> configuredOrientation,
            bool flipX,
            bool flipY,
            bool flipZ)
        {
            constexpr std::array<float, 4> kFlipX{ 1.0f, 0.0f, 0.0f, 0.0f };
            constexpr std::array<float, 4> kFlipY{ 0.0f, 1.0f, 0.0f, 0.0f };
            constexpr std::array<float, 4> kFlipZ{ 0.0f, 0.0f, 1.0f, 0.0f };

            // The configured qZ*qY*qX orientation is complete before flips are
            // applied. Right multiplication makes each enabled quaternion an
            // explicit rotation in final panel-local coordinates, independent of
            // where that axis appears in the Euler construction. Multiple flips use
            // the deterministic postmultiply order X, then Y, then Z.
            if (flipX) {
                configuredOrientation = multiplyQuaternionsRaw(configuredOrientation, kFlipX);
            }
            if (flipY) {
                configuredOrientation = multiplyQuaternionsRaw(configuredOrientation, kFlipY);
            }
            if (flipZ) {
                configuredOrientation = multiplyQuaternionsRaw(configuredOrientation, kFlipZ);
            }
            return normalizeQuaternionRaw(configuredOrientation);
        }

        [[nodiscard]] std::optional<MonitorSettings> parseIni(
            const std::filesystem::path& path,
            std::string& error)
        {
            std::string originalBytes;
            if (!readFileBytes(path, originalBytes, error)) {
                return std::nullopt;
            }
            if (originalBytes.find('\0') != std::string::npos) {
                error = "config contains an embedded NUL byte";
                return std::nullopt;
            }
            const auto scan = scanIniText(originalBytes);
            if (!validateIniTextStructure(scan, error)) {
                return std::nullopt;
            }

            CSimpleIniA ini;
            ini.SetUnicode(false);
            const auto result = ini.LoadData(originalBytes.data(), originalBytes.size());
            if (result < 0) {
                error = std::format(
                    "could not parse bounded snapshot of '{}' (SimpleIni error {})",
                    path.string(),
                    static_cast<int>(result));
                return std::nullopt;
            }

            MonitorSettings settings{};
            if (!parseBoundedFloat(ini, "fPositionX", kPositionMinimum, kPositionMaximum, settings.positionX, error) ||
                !parseBoundedFloat(ini, "fPositionY", kPositionMinimum, kPositionMaximum, settings.positionY, error) ||
                !parseBoundedFloat(ini, "fPositionZ", kPositionMinimum, kPositionMaximum, settings.positionZ, error) ||
                !parseBoundedFloat(ini, "fRotationXDegrees", kRotationMinimumDegrees, kRotationMaximumDegrees, settings.rotationXDegrees, error) ||
                !parseBoundedFloat(ini, "fRotationYDegrees", kRotationMinimumDegrees, kRotationMaximumDegrees, settings.rotationYDegrees, error) ||
                !parseBoundedFloat(ini, "fRotationZDegrees", kRotationMinimumDegrees, kRotationMaximumDegrees, settings.rotationZDegrees, error)) {
                return std::nullopt;
            }

            std::array<bool, 4> missingBooleanKeys{};
            std::string boolError;
            bool boolValuesValid = true;
            const std::array<bool*, 4> booleanTargets{
                &settings.enabled,
                &settings.flipRotationX,
                &settings.flipRotationY,
                &settings.flipRotationZ,
            };
            for (std::size_t index = 0; index < booleanTargets.size(); ++index) {
                std::string currentError;
                if (!parseStrictBool(
                        ini,
                        kKnownKeys[kOptionalBooleanKeyIndices[index]].data(),
                        kOptionalBooleanDefaults[index],
                        *booleanTargets[index],
                        missingBooleanKeys[index],
                        currentError)) {
                    boolValuesValid = false;
                    if (boolError.empty()) {
                        boolError = std::move(currentError);
                    }
                }
            }

            if (!boolValuesValid) {
                error = std::move(boolError);
                return std::nullopt;
            }
            if (std::any_of(missingBooleanKeys.begin(), missingBooleanKeys.end(), [](bool missing) { return missing; })) {
                std::string migrationError;
                const auto migrated = migrateMissingBooleanKeys(
                    path,
                    originalBytes,
                    scan,
                    missingBooleanKeys,
                    migrationError);
                if (migrated) {
                    logger::info("ROCK Monitor: migrated '{}' with {} missing boolean setting(s).",
                        path.string(), *migrated);
                } else {
                    logger::warn(
                        "ROCK Monitor: could not persist missing boolean settings; compiled defaults remain active for missing values ({}).",
                        migrationError);
                }
            }

            const auto configuredOrientation = localOrientationFromEulerDegrees(
                settings.rotationXDegrees,
                settings.rotationYDegrees,
                settings.rotationZDegrees);
            if (!configuredOrientation) {
                error = "configured panel-local Euler rotation could not produce a finite normalized quaternion";
                return std::nullopt;
            }
            const auto finalOrientation = applyFinalPanelLocalFlips(
                *configuredOrientation,
                settings.flipRotationX,
                settings.flipRotationY,
                settings.flipRotationZ);
            if (!finalOrientation) {
                error = "final panel-local flip composition could not produce a finite normalized quaternion";
                return std::nullopt;
            }
            settings.localOrientation = *finalOrientation;
            return settings;
        }
    }

    struct MonitorConfig::Impl
    {
        Impl() : published(std::make_shared<const MonitorSettings>()) {}

        void logSuccessfulReloadLocked(const MonitorSettings& latest)
        {
            const auto now = std::chrono::steady_clock::now();
            const bool firstSuccessfulReload = lastSuccessfulReloadAt.time_since_epoch().count() == 0;
            const bool idleGap = !firstSuccessfulReload &&
                                 now - lastSuccessfulReloadAt >= kSuccessfulReloadLogInterval;
            lastSuccessfulReloadAt = now;
            ++successfulReloadsSinceLog;

            const bool firstSuccessfulLog = lastSuccessfulReloadLogAt.time_since_epoch().count() == 0;
            const bool intervalElapsed = firstSuccessfulLog ||
                                         now - lastSuccessfulReloadLogAt >= kSuccessfulReloadLogInterval;
            if (!idleGap && !intervalElapsed) {
                return;
            }

            logger::info(
                "ROCK Monitor: settings loaded (coalescedSuccessfulReloads={}; enabled={}; latest position={:.2f},{:.2f},{:.2f}; configured Euler={:.1f},{:.1f},{:.1f} degrees; final-local-axis flips X/Y/Z={}/{}/{}).",
                successfulReloadsSinceLog,
                latest.enabled,
                latest.positionX,
                latest.positionY,
                latest.positionZ,
                latest.rotationXDegrees,
                latest.rotationYDegrees,
                latest.rotationZDegrees,
                latest.flipRotationX,
                latest.flipRotationY,
                latest.flipRotationZ);
            successfulReloadsSinceLog = 0;
            lastSuccessfulReloadLogAt = now;
        }

        [[nodiscard]] bool reloadFromDisk(std::string& error)
        {
            std::scoped_lock reloadLock(reloadMutex);
            if (stopping.load(std::memory_order_acquire)) {
                error = "config service is stopping";
                return false;
            }

            try {
                const auto parsed = parseIni(iniPath, error);
                if (!parsed) {
                    return false;
                }
                auto nextSnapshot = std::make_shared<const MonitorSettings>(*parsed);
                if (stopping.load(std::memory_order_acquire)) {
                    error = "config service began stopping during reload";
                    return false;
                }
                published.store(std::move(nextSnapshot), std::memory_order_release);
                logSuccessfulReloadLocked(*parsed);
                return true;
            } catch (const std::exception& exception) {
                error = std::format("config reload raised an exception: {}", exception.what());
            } catch (...) {
                error = "config reload raised an unknown exception";
            }
            return false;
        }

        void requestReload() noexcept
        {
            if (stopping.load(std::memory_order_acquire)) {
                return;
            }
            try {
                {
                    std::scoped_lock lock(requestMutex);
                    reloadPending = true;
                }
                requestCv.notify_one();
            } catch (const std::exception& exception) {
                if (!signalFailureLogged.exchange(true, std::memory_order_relaxed)) {
                    try {
                        logger::error("ROCK Monitor: config reload signal failed ({}).", exception.what());
                    } catch (...) {
                    }
                }
            } catch (...) {
                signalFailureLogged.store(true, std::memory_order_relaxed);
            }
        }

        [[nodiscard]] bool waitForSettledRequest()
        {
            std::unique_lock lock(requestMutex);
            requestCv.wait(lock, [this]() {
                return stopping.load(std::memory_order_acquire) || reloadPending;
            });
            if (stopping.load(std::memory_order_acquire)) {
                return false;
            }

            // Quiet stays below the Configurator's 120 ms slider cadence, while the
            // maximum deadline prevents a notification storm from starving publication.
            const auto maximumDeadline = std::chrono::steady_clock::now() + kReloadMaximumDeferral;
            for (;;) {
                reloadPending = false;
                const auto quietDeadline = (std::min)(
                    std::chrono::steady_clock::now() + kReloadQuietDelay,
                    maximumDeadline);
                const bool requestArrived = requestCv.wait_until(lock, quietDeadline, [this]() {
                    return stopping.load(std::memory_order_acquire) || reloadPending;
                });
                if (stopping.load(std::memory_order_acquire)) {
                    return false;
                }
                if (!requestArrived) {
                    return true;
                }
                if (std::chrono::steady_clock::now() >= maximumDeadline) {
                    // This observed event is included by the snapshot taken next; clear
                    // it while holding the mutex so only later events request another pass.
                    reloadPending = false;
                    return true;
                }
            }
        }

        void workerMain() noexcept
        {
            try {
                filewatch::FileWatch<std::string> watch(
                    iniPath.string(),
                    [this](const std::string&, filewatch::Event event) noexcept {
                        if (event == filewatch::Event::modified ||
                            event == filewatch::Event::added ||
                            event == filewatch::Event::renamed_new) {
                            requestReload();
                        }
                    });
                // Close the initial-load/watch-install window: this converges on any
                // edit that landed after startup parsing but before monitoring began.
                requestReload();

                while (waitForSettledRequest()) {
                    bool loaded = false;
                    bool superseded = false;
                    std::string error;
                    for (std::uint32_t attempt = 0;
                         attempt < kReloadAttempts && !stopping.load(std::memory_order_acquire);
                         ++attempt) {
                        error.clear();
                        if (reloadFromDisk(error)) {
                            loaded = true;
                            break;
                        }
                        if (attempt + 1 >= kReloadAttempts) {
                            break;
                        }

                        std::unique_lock lock(requestMutex);
                        (void)requestCv.wait_for(lock, kReloadRetryDelay, [this]() {
                            return stopping.load(std::memory_order_acquire) || reloadPending;
                        });
                        if (stopping.load(std::memory_order_acquire)) {
                            return;
                        }
                        if (reloadPending) {
                            superseded = true;
                            break;
                        }
                    }
                    if (!loaded && !superseded && !stopping.load(std::memory_order_acquire)) {
                        logger::warn("ROCK Monitor: config reload rejected after {} attempts; retaining last valid settings ({}).",
                            kReloadAttempts, error);
                    }
                }
            } catch (const std::exception& exception) {
                if (!stopping.load(std::memory_order_acquire)) {
                    logger::warn("ROCK Monitor: config worker stopped unexpectedly ({}); startup/last-valid settings remain active.",
                        exception.what());
                }
            } catch (...) {
                if (!stopping.load(std::memory_order_acquire)) {
                    logger::warn("ROCK Monitor: config worker stopped after an unknown error; startup/last-valid settings remain active.");
                }
            }
        }

        void startWorker()
        {
            if (worker.joinable() || stopping.load(std::memory_order_acquire)) {
                return;
            }
            worker = std::jthread([this](std::stop_token) { workerMain(); });
        }

        void stopWorker() noexcept
        {
            stopping.store(true, std::memory_order_release);
            requestCv.notify_all();
            if (worker.joinable()) {
                worker.request_stop();
                worker.join();
            }
        }

        std::filesystem::path iniPath;
        std::atomic<std::shared_ptr<const MonitorSettings>> published;
        std::atomic_bool stopping{ false };
        std::mutex reloadMutex;
        // Successful-reload log state is accessed only while reloadMutex is held.
        std::chrono::steady_clock::time_point lastSuccessfulReloadAt{};
        std::chrono::steady_clock::time_point lastSuccessfulReloadLogAt{};
        std::uint64_t successfulReloadsSinceLog{ 0 };
        std::mutex requestMutex;
        std::condition_variable requestCv;
        bool reloadPending{ false };
        std::atomic_bool signalFailureLogged{ false };
        std::jthread worker;
    };

    MonitorConfig::MonitorConfig() : _impl(std::make_unique<Impl>()) {}

    MonitorConfig::~MonitorConfig()
    {
        stop();
    }

    bool MonitorConfig::load()
    {
        _impl->iniPath = resolveIniPath();
        _impl->stopping.store(false, std::memory_order_release);
        const bool defaultReady = ensureDefaultIniExists(_impl->iniPath);
        std::string error;
        if (!defaultReady) {
            error = "default config file is unavailable";
        }
        const bool loaded = defaultReady && _impl->reloadFromDisk(error);
        if (!loaded) {
            logger::warn("ROCK Monitor: initial config load rejected; compiled defaults remain active ({}).", error);
        }
        if (defaultReady) {
            try {
                _impl->startWorker();
            } catch (const std::exception& exception) {
                logger::warn("ROCK Monitor: config worker could not start ({}); startup values remain active.", exception.what());
            }
        }
        return loaded;
    }

    void MonitorConfig::stop() noexcept
    {
        if (!_impl) {
            return;
        }
        _impl->stopWorker();
    }

    std::shared_ptr<const MonitorSettings> MonitorConfig::snapshot() const noexcept
    {
        return _impl->published.load(std::memory_order_acquire);
    }

    const std::filesystem::path& MonitorConfig::path() const noexcept
    {
        return _impl->iniPath;
    }
}
