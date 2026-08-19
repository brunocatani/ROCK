#include "physics-interaction/weapon/authored_grip/AuthoredWeaponGripCacheStore.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/grab/saved/SavedGrabOffsetStore.h"
#include "rock_support/ResourceUtils.h"

#include <Windows.h>

#include <algorithm>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rock::authored_weapon_grip_cache
{
    namespace
    {
        constexpr auto kCacheRelativePath = R"(\My Games\Fallout4VR\ROCK_Config\AuthoredWeaponGripCache)";
        constexpr std::size_t kMaximumPendingWrites = 64;
        constexpr std::size_t kMaximumCandidateFiles = kMaximumCachedEntries * 4;

        [[nodiscard]] std::string sanitizeFileComponent(const std::string_view text)
        {
            std::string result;
            result.reserve(text.size());
            for (const char value : text) {
                const bool safe = (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
                                  (value >= '0' && value <= '9') || value == '.' || value == '-' || value == '_';
                result.push_back(safe ? value : '_');
            }
            return result;
        }

        struct PendingWrite
        {
            std::filesystem::path path;
            CacheRecord record{};
            std::filesystem::path evictedPath;
        };

        struct CachedEntry
        {
            CacheRecord record{};
            std::filesystem::path path;
            std::uint64_t accessOrdinal{ 0 };
        };

        struct CandidateFile
        {
            std::filesystem::path path;
            std::filesystem::file_time_type modified{};
        };

        class Store
        {
        public:
            void preload()
            {
                std::filesystem::path directory;
                try {
                    directory = resolveDirectory();
                } catch (const std::exception& error) {
                    std::scoped_lock lock(_mutex);
                    _loaded = true;
                    ROCK_LOG_WARN(Animation, "Authored weapon grip cache path unavailable: {}", error.what());
                    return;
                }

                std::error_code error;
                if (!std::filesystem::exists(directory, error) || !std::filesystem::is_directory(directory, error)) {
                    std::scoped_lock lock(_mutex);
                    _loaded = true;
                    ROCK_LOG_INFO(Animation, "Authored weapon grip cache ready entries=0 path={}", directory.string());
                    return;
                }

                std::vector<CandidateFile> files;
                std::size_t candidateOverflow = 0;
                for (const auto& entry : std::filesystem::directory_iterator(directory, error)) {
                    if (error) {
                        break;
                    }
                    if (!entry.is_regular_file(error) || entry.path().extension() != ".json") {
                        continue;
                    }
                    const auto size = entry.file_size(error);
                    if (error || size == 0 || size > kMaximumRecordBytes) {
                        error.clear();
                        continue;
                    }
                    if (files.size() >= kMaximumCandidateFiles) {
                        ++candidateOverflow;
                        continue;
                    }
                    files.push_back(CandidateFile{
                        .path = entry.path(),
                        .modified = entry.last_write_time(error),
                    });
                    error.clear();
                }
                std::sort(files.begin(), files.end(), [](const CandidateFile& left, const CandidateFile& right) {
                    return left.modified > right.modified;
                });

                std::unordered_map<CacheKey, CachedEntry, CacheKeyHash> loaded;
                std::size_t rejected = 0;
                std::size_t overflow = 0;
                std::uint64_t ordinal = 0;
                for (const auto& file : files) {
                    if (loaded.size() >= kMaximumCachedEntries) {
                        ++overflow;
                        continue;
                    }
                    std::ifstream stream(file.path, std::ios::binary);
                    if (!stream) {
                        ++rejected;
                        continue;
                    }
                    std::ostringstream buffer;
                    buffer << stream.rdbuf();
                    CacheRecord record{};
                    std::string parseError;
                    if (!parse(buffer.str(), record, &parseError)) {
                        ++rejected;
                        continue;
                    }
                    if (loaded.contains(record.key)) {
                        ++rejected;
                        continue;
                    }
                    loaded.emplace(record.key, CachedEntry{
                        .record = std::move(record),
                        .path = file.path,
                        .accessOrdinal = static_cast<std::uint64_t>(files.size()) - ordinal,
                    });
                    ++ordinal;
                }

                const auto loadedCount = loaded.size();
                {
                    std::scoped_lock lock(_mutex);
                    _entries = std::move(loaded);
                    _accessOrdinal = static_cast<std::uint64_t>(_entries.size());
                    _loaded = true;
                }
                ROCK_LOG_INFO(Animation,
                    "Authored weapon grip cache ready entries={} rejected={} overflow={} candidateOverflow={} path={}",
                    loadedCount,
                    rejected,
                    overflow,
                    candidateOverflow,
                    directory.string());
            }

            bool find(const CacheKey& key, CacheRecord& out)
            {
                out = {};
                if (!key.valid()) {
                    return false;
                }
                std::scoped_lock lock(_mutex);
                if (!_loaded) {
                    ++_lookupBeforePreload;
                    return false;
                }
                const auto iterator = _entries.find(key);
                if (iterator == _entries.end()) {
                    ++_misses;
                    return false;
                }
                iterator->second.accessOrdinal = ++_accessOrdinal;
                out = iterator->second.record;
                ++_hits;
                ROCK_LOG_INFO(Animation,
                    "Authored weapon grip disk-cache hit plugin={} localForm={:08X} pGrip={:016X} instance={:016X} graph={:016X} powerArmor={} hits={} misses={}",
                    key.weapon.plugin,
                    key.weapon.localFormId,
                    key.pGripVariantKey,
                    key.instanceContentKey,
                    key.graphProfileKey,
                    key.inPowerArmor ? "yes" : "no",
                    _hits,
                    _misses);
                return true;
            }

            void save(CacheRecord record)
            {
                record.formatVersion = kFormatVersion;
                record.poseAlgorithmVersion = kPoseAlgorithmVersion;
                record.checksum = 0;
                record.checksum = calculateChecksum(record);
                if (!validRecord(record)) {
                    ROCK_LOG_WARN(Animation, "Authored weapon grip cache rejected an invalid persistence record");
                    return;
                }

                PendingWrite write{};
                write.path = pathForRecord(record);
                {
                    std::scoped_lock lock(_mutex);
                    auto iterator = _entries.find(record.key);
                    if (iterator == _entries.end() && _entries.size() >= kMaximumCachedEntries) {
                        auto oldest = _entries.end();
                        for (auto candidate = _entries.begin(); candidate != _entries.end(); ++candidate) {
                            if (oldest == _entries.end() || candidate->second.accessOrdinal < oldest->second.accessOrdinal) {
                                oldest = candidate;
                            }
                        }
                        if (oldest != _entries.end()) {
                            write.evictedPath = oldest->second.path;
                            _entries.erase(oldest);
                            ++_evictions;
                        }
                    }
                    _entries.insert_or_assign(record.key, CachedEntry{
                        .record = record,
                        .path = write.path,
                        .accessOrdinal = ++_accessOrdinal,
                    });
                    ++_publications;
                }
                write.record = std::move(record);
                enqueue(std::move(write));
            }

        private:
            [[nodiscard]] std::filesystem::path resolveDirectory()
            {
                std::scoped_lock lock(_mutex);
                if (_directory.empty()) {
                    _directory = resources::getPathInDocuments(kCacheRelativePath);
                }
                return _directory;
            }

            [[nodiscard]] std::filesystem::path pathForRecord(const CacheRecord& record)
            {
                const auto directory = resolveDirectory();
                const auto keyHash = CacheKeyHash{}(record.key);
                char suffix[96]{};
                std::snprintf(suffix, sizeof(suffix), "_%08X_%016llX_%016llX.grip.json",
                    record.key.weapon.localFormId,
                    static_cast<unsigned long long>(record.key.pGripVariantKey),
                    static_cast<unsigned long long>(keyHash));
                return directory / (sanitizeFileComponent(record.key.weapon.plugin) + suffix);
            }

            void enqueue(PendingWrite write)
            {
                bool replacedPendingWrite = false;
                {
                    std::scoped_lock lock(_mutex);
                    for (auto& pending : _queue) {
                        if (pending.path == write.path) {
                            pending = std::move(write);
                            replacedPendingWrite = true;
                            break;
                        }
                    }
                    if (!replacedPendingWrite && _queue.size() >= kMaximumPendingWrites) {
                        ++_droppedWrites;
                        ROCK_LOG_WARN(Animation,
                            "Authored weapon grip cache write queue full; persistence write dropped count={}",
                            _droppedWrites);
                        return;
                    }
                    if (!replacedPendingWrite) {
                        _queue.push_back(std::move(write));
                    }
                }
                if (!ensureWriterStarted()) {
                    return;
                }
                _wake.notify_one();
            }

            [[nodiscard]] bool ensureWriterStarted()
            {
                std::scoped_lock lock(_mutex);
                if (_writerStarted) {
                    return true;
                }
                try {
                    _writer = std::thread([this]() { writerLoop(); });
                    _writerStarted = true;
                    return true;
                } catch (const std::exception& error) {
                    ROCK_LOG_WARN(Animation, "Authored weapon grip cache writer could not start: {}", error.what());
                    return false;
                }
            }

            void writerLoop()
            {
                for (;;) {
                    PendingWrite write{};
                    {
                        std::unique_lock lock(_mutex);
                        _wake.wait(lock, [this]() { return !_queue.empty(); });
                        write = std::move(_queue.front());
                        _queue.pop_front();
                    }

                    std::error_code error;
                    std::filesystem::create_directories(write.path.parent_path(), error);
                    if (error) {
                        ROCK_LOG_WARN(Animation, "Authored weapon grip cache directory create failed path={} reason={}", write.path.parent_path().string(), error.message());
                        continue;
                    }
                    std::string content;
                    try {
                        content = serialize(write.record);
                    } catch (const std::exception& serializationError) {
                        ROCK_LOG_WARN(Animation,
                            "Authored weapon grip cache serialization failed path={} reason={}",
                            write.path.string(),
                            serializationError.what());
                        continue;
                    }
                    if (content.empty() || content.size() > kMaximumRecordBytes) {
                        ROCK_LOG_WARN(Animation,
                            "Authored weapon grip cache serialization exceeded record limit path={} bytes={}",
                            write.path.string(),
                            content.size());
                        continue;
                    }
                    const auto temporary = write.path.string() + ".tmp";
                    {
                        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
                        if (!stream || !stream.write(content.data(), static_cast<std::streamsize>(content.size()))) {
                            ROCK_LOG_WARN(Animation, "Authored weapon grip cache write failed path={}", temporary);
                            stream.close();
                            std::filesystem::remove(temporary, error);
                            continue;
                        }
                        stream.flush();
                        if (!stream) {
                            ROCK_LOG_WARN(Animation, "Authored weapon grip cache flush failed path={}", temporary);
                            stream.close();
                            std::filesystem::remove(temporary, error);
                            continue;
                        }
                    }

                    const std::filesystem::path temporaryPath{ temporary };
                    if (!MoveFileExW(temporaryPath.c_str(), write.path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
                        ROCK_LOG_WARN(Animation, "Authored weapon grip cache atomic replace failed path={} error={}", write.path.string(), GetLastError());
                        std::filesystem::remove(temporaryPath, error);
                        continue;
                    }
                    if (!write.evictedPath.empty() && write.evictedPath != write.path) {
                        bool pathStillOwned = false;
                        {
                            std::scoped_lock lock(_mutex);
                            pathStillOwned = std::any_of(_entries.begin(), _entries.end(), [&](const auto& entry) {
                                return entry.second.path == write.evictedPath;
                            });
                        }
                        if (!pathStillOwned) {
                            std::filesystem::remove(write.evictedPath, error);
                        }
                    }
                }
            }

            std::filesystem::path _directory;
            std::unordered_map<CacheKey, CachedEntry, CacheKeyHash> _entries;
            std::deque<PendingWrite> _queue;
            std::thread _writer;
            std::mutex _mutex;
            std::condition_variable _wake;
            std::uint64_t _accessOrdinal{ 0 };
            std::uint64_t _hits{ 0 };
            std::uint64_t _misses{ 0 };
            std::uint64_t _lookupBeforePreload{ 0 };
            std::uint64_t _publications{ 0 };
            std::uint64_t _evictions{ 0 };
            std::uint64_t _droppedWrites{ 0 };
            bool _loaded{ false };
            bool _writerStarted{ false };
        };

        [[nodiscard]] Store& store()
        {
            // Process-lifetime service. DLL/static destruction can run under
            // loader teardown, where joining a Windows worker is unsafe; the
            // OS reclaims the bounded queue and writer with the game process.
            static Store* instance = new Store();
            return *instance;
        }
    }

    void preload()
    {
        try {
            store().preload();
        } catch (const std::exception& error) {
            ROCK_LOG_WARN(Animation, "Authored weapon grip cache preload failed closed: {}", error.what());
        }
    }

    bool makeCacheKey(
        const std::uint32_t runtimeWeaponFormId,
        const std::uint64_t pGripVariantKey,
        const std::uint64_t instanceContentKey,
        const std::uint64_t graphProfileKey,
        const bool inPowerArmor,
        CacheKey& out)
    {
        try {
            out = {};
            const auto stable = saved_grab_offset::formRefFromRuntimeId(runtimeWeaponFormId);
            if (stable.empty()) {
                return false;
            }
            out = CacheKey{
                .weapon = StableFormIdentity{
                    .plugin = stable.plugin,
                    .localFormId = stable.localFormId,
                },
                .pGripVariantKey = pGripVariantKey,
                .instanceContentKey = instanceContentKey,
                .graphProfileKey = graphProfileKey,
                .inPowerArmor = inPowerArmor,
            };
            return out.valid();
        } catch (const std::exception& error) {
            out = {};
            ROCK_LOG_WARN(Animation, "Authored weapon grip cache key construction failed closed: {}", error.what());
            return false;
        }
    }

    bool find(const CacheKey& key, CacheRecord& out)
    {
        try {
            return store().find(key, out);
        } catch (const std::exception& error) {
            out = {};
            ROCK_LOG_WARN(Animation, "Authored weapon grip cache lookup failed closed: {}", error.what());
            return false;
        }
    }

    void save(CacheRecord record)
    {
        try {
            store().save(std::move(record));
        } catch (const std::exception& error) {
            ROCK_LOG_WARN(Animation, "Authored weapon grip cache publication failed closed: {}", error.what());
        }
    }
}
