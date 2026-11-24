#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"
#include "rock_support/ResourceUtils.h"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace
{
    using rock::config::Group;

    std::filesystem::path resolveConfigDirectory()
    {
        return rock::resources::getPathInDocuments(R"(\My Games\Fallout4VR\Mods_Config\ROCK)");
    }

    std::int64_t eventTicks() noexcept
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
}

namespace rock
{
    void RockConfig::resetToDefaults() { static_cast<RockConfigValues&>(*this) = RockConfigValues{}; }

    void RockConfig::load()
    {
        try {
            {
                std::scoped_lock lock(_storeMutex);
                if (!_store) {
                    _store = config::makeStore(resolveConfigDirectory(), compiledDefaults());
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

    /**
     * Applies the store's values with the preview values on top, reading the files first when readFiles is set; a
     * preview alone reparses the values the last read cached. Returns false when the store is busy (the reload stays
     * pending) or missing; applies nothing when neither the files nor the preview values changed.
     */
    bool RockConfig::loadStore(bool createConsumer, bool readFiles)
    {
        // Do not stall the runtime frame behind an API visitor or file edit.
        // Keep the existing pending reload until the configuration task finishes.
        std::unique_lock lock(_storeMutex, std::try_to_lock);
        if (!lock.owns_lock()) {
            (readFiles ? _reloadPending : _previewPending).store(true, std::memory_order_release);
            return false;
        }
        if (!_store) {
            ROCK_LOG_WARN(Config, "Configuration reload failed; configuration is not initialized");
            return false;
        }
        if (readFiles) {
            _store->load(createConsumer);
            for (const auto& notice : _store->notices())
                ROCK_LOG_INFO(Config, "{}", notice);
            for (const auto& message : _store->messages())
                ROCK_LOG_WARN(Config, "{}", message);
        }
        const auto storeRevision = _store->revision();
        if (configRevision() != 0 && storeRevision == _appliedStoreRevision &&
            _previewRevision == _appliedPreviewRevision)
            return true;

        auto values = _store->loadedValues();
        for (const auto& [path, entries] : _previewValues)
            values.insert_or_assign(path, entries.back().value);
        static_cast<RockConfigValues&>(*this) = parseValues(values);
        logger::setLogLevelAndPattern(rockLogLevel, rockLogPattern);
        logger::setFlushImmediate(rockLogFlushImmediate);
        if (storeRevision != _appliedStoreRevision)
            _configFileRevision.fetch_add(1, std::memory_order_acq_rel);
        _appliedStoreRevision = storeRevision;
        _appliedPreviewRevision = _previewRevision;
        const auto previewCount = _previewValues.size();
        _configRevision.fetch_add(1, std::memory_order_acq_rel);
        lock.unlock(); // Subscribers may inspect the newly applied catalog.
        if (readFiles) {
            ROCK_LOG_INFO(Config, "ROCK configuration applied (revision={}, logLevel={} {}, preview values={})",
                configRevision(), rockLogLevel, logging_policy::logLevelName(rockLogLevel), previewCount);
            ROCK_LOG_INFO(Config, "Weapon recoil mode: {}",
                rockEmergencyDisableRecoil ? "EMERGENCY DISABLED"
                    : rockImmersiveRecoil  ? "Immersive Recoil"
                                           : "Native strength, physical firing hand");
        } else {
            ROCK_LOG_DEBUG(Config, "ROCK preview applied (revision={}, preview values={})", configRevision(),
                previewCount);
        }
        for (const auto& [key, subscriber] : _onConfigChangedSubscribers)
            subscriber(key);
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

    bool RockConfig::visitSettings(Group group, SettingVisitor visitor, void* context) const
    {
        std::scoped_lock lock(_storeMutex);
        if (!_store || !visitor || configRevision() == 0)
            return false;
        for (const auto& setting : _store->settings()) {
            if (setting.group == config::storeGroup(group))
                visitor(setting, context);
        }
        return true;
    }

    bool RockConfig::persistSetting(Group group, const char* section, const char* key, const char* value,
        std::string& error)
    {
        std::scoped_lock lock(_storeMutex);
        if (!_store || !section || !key || !value) {
            error = "ROCK configuration is unavailable";
            return false;
        }
        if (!_store->setValue(config::storeGroup(group), section, key, value)) {
            error = _store->error();
            ROCK_LOG_WARN(Config, "Cannot persist [{}] {}: {}", section, key, error);
            return false;
        }
        _lastFileEventTicks.store(eventTicks(), std::memory_order_release);
        _reloadPending.store(true, std::memory_order_release);
        return true;
    }

    /**
     * Keeps the owner's preview value of an option of either file, in its canonical text, until that owner drops it.
     * Of several owners' values of one option the last set applies. False, and logged, for an option the catalog lacks
     * or a value its type cannot hold.
     */
    bool RockConfig::setPreviewValue(PreviewOwner owner, std::string_view table, std::string_view key,
        std::string_view value)
    {
        {
            std::scoped_lock lock(_storeMutex);
            const auto* setting = _store ? _store->find(table, key) : nullptr;
            const auto canonical = setting ? config::normalizeValue(setting->type, value) : std::nullopt;
            if (canonical) {
                auto& entries = _previewValues[config::settingPath(table, key)];
                const auto own = std::ranges::find(entries, owner, &PreviewEntry::owner);
                if (own != entries.end() && own->value == *canonical)
                    return true;
                if (own != entries.end())
                    entries.erase(own);
                entries.push_back({ owner, *canonical });
                ++_previewRevision;
                _previewPending.store(true, std::memory_order_release);
                return true;
            }
        }
        ROCK_LOG_WARN(Config, "Preview of [{}] {} = '{}' ignored: no such option, or not a value of it", table, key,
            value);
        return false;
    }

    /**
     * Drops the owner's preview value of an option; false when it had none.
     */
    bool RockConfig::discardPreviewValue(PreviewOwner owner, std::string_view table, std::string_view key)
    {
        std::scoped_lock lock(_storeMutex);
        const auto path = _previewValues.find(config::settingPath(table, key));
        if (path == _previewValues.end())
            return false;
        const auto own = std::ranges::find(path->second, owner, &PreviewEntry::owner);
        if (own == path->second.end())
            return false;
        path->second.erase(own);
        if (path->second.empty())
            _previewValues.erase(path);
        ++_previewRevision;
        _previewPending.store(true, std::memory_order_release);
        return true;
    }

    /**
     * Drops every preview value of the owner and returns how many it had.
     */
    std::size_t RockConfig::clearPreviewValues(PreviewOwner owner)
    {
        std::scoped_lock lock(_storeMutex);
        std::size_t removed = 0;
        for (auto path = _previewValues.begin(); path != _previewValues.end();) {
            removed += std::erase_if(path->second, [owner](const PreviewEntry& entry) { return entry.owner == owner; });
            path = path->second.empty() ? _previewValues.erase(path) : std::next(path);
        }
        if (removed == 0)
            return 0;
        ++_previewRevision;
        _previewPending.store(true, std::memory_order_release);
        return removed;
    }

    /**
     * Every owner's preview values, by option path. A path is "table.key", and keys never hold a '.'.
     */
    std::vector<RockConfig::PreviewValue> RockConfig::previewValues() const
    {
        std::scoped_lock lock(_storeMutex);
        std::vector<PreviewValue> values;
        for (const auto& [path, entries] : _previewValues) {
            const auto split = path.rfind('.');
            const auto table = split == std::string::npos ? std::string() : path.substr(0, split);
            const auto key = split == std::string::npos ? path : path.substr(split + 1);
            for (const auto& entry : entries)
                values.push_back({ entry.owner, table, key, entry.value, &entry == &entries.back() });
        }
        return values;
    }

    /**
     * The option's value as the next apply reads it, with the file's value beneath; nullopt for an option the
     * catalog lacks or before the settings load.
     */
    std::optional<RockConfig::SettingValue> RockConfig::settingValue(std::string_view table, std::string_view key) const
    {
        std::scoped_lock lock(_storeMutex);
        const auto* setting = _store ? _store->find(table, key) : nullptr;
        if (!setting)
            return std::nullopt;
        SettingValue result{ setting->value, setting->value,
            setting->group == RPS::Settings::Group::Developer ? Group::Developer : Group::Player, std::nullopt };
        if (const auto path = _previewValues.find(config::settingPath(table, key)); path != _previewValues.end()) {
            result.value = path->second.back().value;
            result.previewOwner = path->second.back().owner;
        }
        return result;
    }

    void RockConfig::requestImmediateReload() noexcept { _immediateReload.store(true, std::memory_order_release); }

    /**
     * Frame-thread apply point for every change of the values: a file reload once the watcher's burst settles (or at
     * once when requested), which carries the preview values too, otherwise a reparse for changed preview values.
     */
    void RockConfig::processPendingConfigReload()
    {
        // The watcher only publishes a change notification. Editor bursts are
        // coalesced without sleeping in the callback or on the game thread.
        const bool immediate = _immediateReload.exchange(false, std::memory_order_acq_rel);
        const bool filesSettled = _reloadPending.load(std::memory_order_acquire) &&
            eventTicks() - _lastFileEventTicks.load(std::memory_order_acquire) >= 200;
        if (immediate || filesSettled) {
            _reloadPending.store(false, std::memory_order_release);
            _previewPending.store(false, std::memory_order_release);
            reload();
            return;
        }
        if (!_previewPending.exchange(false, std::memory_order_acq_rel))
            return;
        try {
            (void)loadStore(false, false);
        } catch (const std::exception& error) {
            ROCK_LOG_ERROR(Config, "Cannot apply ROCK preview values: {}", error.what());
        }
    }

    void RockConfig::startFileWatch()
    {
        if (_fileWatch || !_store)
            return;
        // Watch the directory so creation, replacement and removal of the
        // optional developer file are observed even when it did not exist.
        _fileWatch = std::make_unique<filewatch::FileWatch<std::string>>(_store->directory().string(),
            [this](const std::string& name, filewatch::Event) {
                // Windows file names are case-insensitive.
                if (_stricmp(name.c_str(), config::kPlayerFileName.data()) != 0 &&
                    _stricmp(name.c_str(), config::kDeveloperFileName.data()) != 0)
                    return;
                _lastFileEventTicks.store(eventTicks(), std::memory_order_release);
                _reloadPending.store(true, std::memory_order_release);
            });
    }

    void RockConfig::stopFileWatch() { _fileWatch.reset(); }

    void RockConfig::subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback)
    {
        _onConfigChangedSubscribers[key] = std::move(callback);
    }

    void RockConfig::unsubscribeFromConfigChanged(const std::string& key) { _onConfigChangedSubscribers.erase(key); }
}
