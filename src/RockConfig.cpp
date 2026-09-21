#include "RockConfig.h"
#include "rock_support/ResourceUtils.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"

#include <chrono>
#include <cstring>

namespace
{
    using rock::configuration_api::Group;

    std::filesystem::path resolveConfigDirectory()
    {
        return rock::resources::getPathInDocuments(
            R"(\My Games\Fallout4VR\Mods_Config\ROCK)");
    }

    std::int64_t eventTicks() noexcept
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
}

namespace rock
{
    void RockConfig::resetToDefaults()
    {
        static_cast<RockConfigValues&>(*this) = RockConfigValues{};
    }

    void RockConfig::load()
    {
        try {
            {
                std::scoped_lock lock(_storeMutex);
                if (!_store) {
                    CSimpleIniA defaults;
                    buildCompiledDefaults(defaults);
                    _store = std::make_unique<config::ConfigurationStore>(resolveConfigDirectory(), defaults);
                    resetToDefaults();
                }
            }
            ROCK_LOG_INFO(Config, "Loading ROCK configuration from: {}", _store->directory().string());
            (void)loadStore(true);
            startFileWatch();
        } catch (const std::exception& error) {
            ROCK_LOG_ERROR(Config, "Cannot initialize ROCK configuration: {}", error.what());
        }
    }

    bool RockConfig::loadStore(bool createConsumer)
    {
        // Do not stall the runtime frame behind an API visitor or INI edit.
        // Keep the existing pending reload until the configuration task finishes.
        std::unique_lock lock(_storeMutex, std::try_to_lock);
        if (!lock.owns_lock()) {
            _reloadPending.store(true, std::memory_order_release);
            return false;
        }
        if (!_store || !_store->load(createConsumer)) {
            ROCK_LOG_WARN(Config, "Configuration reload failed; retaining current values: {}",
                _store ? _store->error() : "configuration is not initialized");
            return false;
        }
        if (_store->revision() == configRevision()) return true;

        CSimpleIniA combined;
        _store->appendLoadedValues(combined);
        static_cast<RockConfigValues&>(*this) = parseValues(combined);
        logger::setLogLevelAndPattern(rockLogLevel, rockLogPattern);
        _configRevision.store(_store->revision(), std::memory_order_release);
        lock.unlock(); // Subscribers may inspect the newly applied catalog.
        ROCK_LOG_INFO(Config, "ROCK configuration applied (revision={}, logLevel={} {})",
            configRevision(), rockLogLevel, logging_policy::logLevelName(rockLogLevel));
        for (const auto& [key, subscriber] : _onConfigChangedSubscribers) subscriber(key);
        return true;
    }

    void RockConfig::reload()
    {
        if (!_store) {
            load();
            return;
        }
        try {
            (void)loadStore(false);
        } catch (const std::exception& error) {
            ROCK_LOG_ERROR(Config, "Cannot reload ROCK configuration: {}", error.what());
        }
    }

    std::filesystem::path RockConfig::getConfigDirectory() const
    {
        std::scoped_lock lock(_storeMutex);
        return _store ? _store->directory() : resolveConfigDirectory();
    }

    bool RockConfig::visitSettings(Group group, configuration_api::VisitorV1 visitor, void* context) const
    {
        std::scoped_lock lock(_storeMutex);
        if (!_store || !visitor || configRevision() == 0) return false;
        if (group != Group::Consumer && group != Group::Developer) return false;
        for (const auto& setting : _store->settings()) {
            if (setting.group != group) continue;
            const configuration_api::SettingV1 record{
                setting.section.c_str(), setting.key.c_str(), setting.value.c_str(),
                setting.defaultValue.c_str(), setting.category.c_str(), setting.description.c_str(),
                setting.type, config::ConfigurationStore::isDefault(setting, setting.value) ? 0u : 1u,
            };
            visitor(&record, context);
        }
        return true;
    }

    bool RockConfig::persistSetting(Group group, const char* section, const char* key,
        const char* value, std::string& error)
    {
        std::scoped_lock lock(_storeMutex);
        if (!_store || !section || !key || !value) {
            error = "ROCK configuration is unavailable";
            return false;
        }
        if (!_store->setValue(group, section, key, value)) {
            error = _store->error();
            ROCK_LOG_WARN(Config, "Cannot persist [{}] {}: {}", section, key, error);
            return false;
        }
        _lastFileEventTicks.store(eventTicks(), std::memory_order_release);
        _reloadPending.store(true, std::memory_order_release);
        return true;
    }

    void RockConfig::processPendingConfigReload()
    {
        // The watcher only publishes a change notification. Editor bursts are
        // coalesced without sleeping in the callback or on the game thread.
        if (!_reloadPending.load(std::memory_order_acquire) ||
            eventTicks() - _lastFileEventTicks.load(std::memory_order_acquire) < 200) return;
        if (_reloadPending.exchange(false, std::memory_order_acq_rel)) reload();
    }

    void RockConfig::startFileWatch()
    {
        if (_fileWatch || !_store) return;
        // Watch the directory so creation, replacement and removal of the
        // optional developer file are observed even when it did not exist.
        _fileWatch = std::make_unique<filewatch::FileWatch<std::string>>(_store->directory().string(),
            [this](const std::string& name, filewatch::Event) {
                if (_stricmp(name.c_str(), "ROCK.ini") != 0 &&
                    _stricmp(name.c_str(), "ROCK_Developer.ini") != 0) return;
                _lastFileEventTicks.store(eventTicks(), std::memory_order_release);
                _reloadPending.store(true, std::memory_order_release);
            });
    }

    void RockConfig::stopFileWatch() { _fileWatch.reset(); }
    void RockConfig::subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback) { _onConfigChangedSubscribers[key] = std::move(callback); }
    void RockConfig::unsubscribeFromConfigChanged(const std::string& key) { _onConfigChangedSubscribers.erase(key); }
}
