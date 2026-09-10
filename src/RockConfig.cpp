#include "RockConfig.h"

#include "rock_support/ResourceUtils.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"

#include <filesystem>
#include <thread>

namespace
{
    constexpr auto SECTION = "PhysicsInteraction";
    std::string resolveIniPath()
    {
        try {
            return rock::resources::getPathInDocuments(
                R"(\My Games\Fallout4VR\Mods_Config\ROCK\ROCK.ini)");
        } catch (const std::exception& error) {
            ROCK_LOG_ERROR(Config,
                "Failed to resolve the only supported ROCK.ini path: {}",
                error.what());
            return {};
        }
    }

}

namespace rock
{
    void RockConfig::resetToDefaults()
    {
        static_cast<RockConfigValues&>(*this) = RockConfigValues{};
    }

    bool RockConfig::createDefaultIniIfMissing()
    {
        if (_iniFilePath.empty()) {
            return false;
        }

        const std::filesystem::path targetPath(_iniFilePath);
        std::error_code ec;
        const bool targetExists = std::filesystem::exists(targetPath, ec);
        if (ec) {
            ROCK_LOG_ERROR(Config,
                "Cannot inspect the production ROCK.ini path '{}': {}",
                _iniFilePath,
                ec.message());
            return false;
        }
        if (targetExists) {
            return true;
        }

        try {
            rock::resources::createDirectoryTreeForFile(_iniFilePath);

            CSimpleIniA defaults;
            defaults.SetUnicode(false);
            resetToDefaults();
            readValuesFromIni(defaults, true);

            auto temporaryPath = targetPath;
            temporaryPath += ".creating";
            const SI_Error saveResult =
                defaults.SaveFile(temporaryPath.string().c_str(), false);
            if (saveResult < 0) {
                std::filesystem::remove(temporaryPath, ec);
                ROCK_LOG_ERROR(Config,
                    "Failed to write complete compiled ROCK.ini defaults to temporary file '{}' (code {})",
                    temporaryPath.string(),
                    static_cast<int>(saveResult));
                return false;
            }

            ec.clear();
            std::filesystem::rename(temporaryPath, targetPath, ec);
            if (ec) {
                std::error_code targetCheckError;
                const bool createdElsewhere =
                    std::filesystem::exists(targetPath, targetCheckError) &&
                    !targetCheckError;
                std::error_code cleanupError;
                std::filesystem::remove(temporaryPath, cleanupError);
                if (createdElsewhere) {
                    return true;
                }

                ROCK_LOG_ERROR(Config,
                    "Failed to publish generated ROCK.ini '{}': {}",
                    _iniFilePath,
                    ec.message());
                return false;
            }

            ROCK_LOG_INFO(Config,
                "Created complete production ROCK.ini from compiled C++ defaults: {}",
                _iniFilePath);
            return true;
        } catch (const std::exception& error) {
            ROCK_LOG_ERROR(Config,
                "Failed to create production ROCK.ini '{}': {}",
                _iniFilePath,
                error.what());
            return false;
        }
    }

    void RockConfig::load()
    {
        _iniFilePath = resolveIniPath();
        resetToDefaults();
        if (_iniFilePath.empty()) {
            ROCK_LOG_ERROR(Config,
                "ROCK.ini path is unavailable; using compiled defaults in memory without creating a fallback file");
            return;
        }

        ROCK_LOG_INFO(Config, "Loading the only active ROCK config from: {}", _iniFilePath);
        (void)createDefaultIniIfMissing();

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini not found or unreadable (code {}), using compiled-in defaults", static_cast<int>(rc));
            return;
        }

        resetToDefaults();
        readValuesFromIni(ini);

        ROCK_LOG_INFO(Config,
            "ROCK config loaded (logLevel={} {}, sample={}ms)",
            rockLogLevel,
            logging_policy::logLevelName(rockLogLevel),
            rockLogSampleMilliseconds);

        startFileWatch();
    }

    void RockConfig::reload()
    {
        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "reload() called before load() — delegating to load()");
            load();
            return;
        }

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini reload failed (code {}), retaining current values", static_cast<int>(rc));
            return;
        }

        resetToDefaults();
        readValuesFromIni(ini);
        ROCK_LOG_INFO(Config,
            "ROCK config reloaded (logLevel={} {}, sample={}ms)",
            rockLogLevel,
            logging_policy::logLevelName(rockLogLevel),
            rockLogSampleMilliseconds);
    }

    std::filesystem::path RockConfig::getConfigDirectory() const
    {
        if (_iniFilePath.empty()) {
            return std::filesystem::path(resolveIniPath()).parent_path();
        }
        return std::filesystem::path(_iniFilePath).parent_path();
    }

    bool RockConfig::saveRuntimeIni(CSimpleIniA& ini, const char* reason)
    {
        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        if (path.empty()) {
            ROCK_LOG_WARN(Config,
                "Cannot persist ROCK.ini runtime change '{}': production path is unavailable",
                reason ? reason : "unknown");
            return false;
        }
        _selfIniWriteInProgress.store(true, std::memory_order_release);

        const SI_Error saveRc = ini.SaveFile(path.c_str(), false);
        std::error_code ec;
        const auto writeTime = std::filesystem::last_write_time(path, ec);
        if (!ec) {
            _lastSelfIniWriteTime.store(writeTime, std::memory_order_release);
            _lastIniFileWriteTime.store(writeTime, std::memory_order_release);
        }

        _selfIniWriteInProgress.store(false, std::memory_order_release);

        if (saveRc < 0) {
            ROCK_LOG_WARN(Config, "Failed to persist ROCK.ini runtime change '{}' (code {})", reason ? reason : "unknown", static_cast<int>(saveRc));
            return false;
        }

        ROCK_LOG_DEBUG(Config, "Persisted ROCK.ini runtime change '{}'", reason ? reason : "unknown");
        return true;
    }

    bool RockConfig::persistPhysicsBool(const char* key, bool value)
    {
        if (!key || !key[0]) {
            return false;
        }

        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        if (path.empty()) {
            ROCK_LOG_WARN(Config,
                "Cannot persist ROCK.ini bool '{}': production path is unavailable",
                key);
            return false;
        }
        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error loadRc = ini.LoadFile(path.c_str());
        if (loadRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini bool '{}': load failed with code {}", key, static_cast<int>(loadRc));
            return false;
        }

        const SI_Error setRc = ini.SetBoolValue(SECTION, key, value, nullptr, true);
        if (setRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini bool '{}': set failed with code {}", key, static_cast<int>(setRc));
            return false;
        }

        return saveRuntimeIni(ini, key);
    }

    bool RockConfig::persistGrabLegacyPalmPivotAHandspace(bool isLeft, const RE::NiPoint3& value)
    {
        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        if (path.empty()) {
            ROCK_LOG_WARN(Config,
                "Cannot persist ROCK.ini {} legacy palm pivot A: production path is unavailable",
                isLeft ? "left" : "right");
            return false;
        }
        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error loadRc = ini.LoadFile(path.c_str());
        if (loadRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini {} legacy palm pivot A: load failed with code {}", isLeft ? "left" : "right", static_cast<int>(loadRc));
            return false;
        }

        const char* keyX = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceX" : "fRightGrabLegacyPalmPivotAHandspaceX";
        const char* keyY = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceY" : "fRightGrabLegacyPalmPivotAHandspaceY";
        const char* keyZ = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceZ" : "fRightGrabLegacyPalmPivotAHandspaceZ";
        bool ok = true;
        ok &= ini.SetDoubleValue(SECTION, keyX, value.x, nullptr, true) >= 0;
        ok &= ini.SetDoubleValue(SECTION, keyY, value.y, nullptr, true) >= 0;
        ok &= ini.SetDoubleValue(SECTION, keyZ, value.z, nullptr, true) >= 0;
        if (!ok) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini {} legacy palm pivot A: set failed", isLeft ? "left" : "right");
            return false;
        }

        return saveRuntimeIni(ini, isLeft ? "left legacy palm pivot A" : "right legacy palm pivot A");
    }

    void RockConfig::processPendingConfigReload()
    {
        if (!_reloadPending.exchange(false, std::memory_order_acq_rel)) {
            return;
        }

        ROCK_LOG_INFO(Config, "ROCK.ini change detected, reloading on frame thread...");
        reload();

        for (const auto& [key, subscriber] : _onConfigChangedSubscribers) {
            ROCK_LOG_DEBUG(Config, "Notify config change subscriber '{}'", key);
            subscriber(key);
        }
    }

    void RockConfig::startFileWatch()
    {
        if (_fileWatch) {
            return;
        }
        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "Cannot start file watch — INI path not resolved");
            return;
        }

        ROCK_LOG_DEBUG(Config, "Starting file watch on '{}'", _iniFilePath);
        _fileWatch = std::make_unique<filewatch::FileWatch<std::string>>(_iniFilePath, [this](const std::string&, const filewatch::Event changeType) {
            if (changeType != filewatch::Event::modified &&
                changeType != filewatch::Event::added &&
                changeType != filewatch::Event::renamed_new) {
                return;
            }

            constexpr auto delay = std::chrono::milliseconds(200);

            auto prevWriteTime = _lastIniFileWriteTime.load();
            std::error_code ec;
            const auto writeTime = std::filesystem::last_write_time(_iniFilePath, ec);
            if (ec || writeTime - prevWriteTime < delay) {
                return;
            }

            const auto selfWriteTime = _lastSelfIniWriteTime.load(std::memory_order_acquire);
            if (_selfIniWriteInProgress.load(std::memory_order_acquire) ||
                (selfWriteTime != std::filesystem::file_time_type{} && writeTime <= selfWriteTime)) {
                _lastIniFileWriteTime.store(writeTime, std::memory_order_release);
                if (!_selfIniWriteInProgress.load(std::memory_order_acquire)) {
                    _lastSelfIniWriteTime.store(std::filesystem::file_time_type{}, std::memory_order_release);
                }
                return;
            }

            if (!_lastIniFileWriteTime.compare_exchange_strong(prevWriteTime, writeTime)) {
                return;
            }

            auto now = std::filesystem::file_time_type::clock::now();
            auto lastEventTime = _lastIniFileWriteTime.load();
            while (now - lastEventTime < delay) {
                std::this_thread::sleep_for(std::max(std::chrono::milliseconds(0), std::chrono::duration_cast<std::chrono::milliseconds>(delay - (now - lastEventTime))));
                now = std::filesystem::file_time_type::clock::now();
                lastEventTime = _lastIniFileWriteTime.load();
            }

            _reloadPending.store(true, std::memory_order_release);
        });
    }

    void RockConfig::stopFileWatch()
    {
        if (_fileWatch) {
            ROCK_LOG_DEBUG(Config, "Stopping file watch on ROCK.ini");
            _fileWatch.reset();
        }
    }

    void RockConfig::subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback) { _onConfigChangedSubscribers[key] = std::move(callback); }

    void RockConfig::unsubscribeFromConfigChanged(const std::string& key) { _onConfigChangedSubscribers.erase(key); }
}
