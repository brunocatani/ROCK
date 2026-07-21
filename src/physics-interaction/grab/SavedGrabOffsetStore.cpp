#include "physics-interaction/grab/SavedGrabOffsetStore.h"

#include "physics-interaction/PhysicsLog.h"

#include "rock_support/ResourceUtils.h"

#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESDataHandler.h"

#include <condition_variable>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>

namespace rock::saved_grab_offset
{
    namespace
    {
        constexpr auto kSavedGrabOffsetsRelativePath = R"(\My Games\Fallout4VR\ROCK_Config\SavedGrabOffsets)";

        std::string resolveStoreDirectory()
        {
            return rock::resources::getPathInDocuments(kSavedGrabOffsetsRelativePath);
        }

        // Plugin names become file-name components; keep letters, digits,
        // dots, dashes, underscores and spaces, replace the rest.
        std::string sanitizeForFileName(std::string_view text)
        {
            std::string out;
            out.reserve(text.size());
            for (const char c : text) {
                const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                                (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '_' || c == ' ';
                out.push_back(ok ? c : '_');
            }
            return out;
        }

        struct PendingWrite
        {
            std::string path;
            std::string content;
        };

        /*
         * Process-lifetime store instance (Meyer's singleton: lazy-init on
         * first use, no static-initialization-order dependency). Its
         * destructor drains the write queue and joins the writer thread at
         * static teardown.
         */
        class Store
        {
        public:
            Store() :
                _directory(resolveStoreDirectory())
            {
            }

            ~Store()
            {
                shutdown();
            }

            std::string filePathForObject(const FormRef& object) const
            {
                char idText[16]{};
                std::snprintf(idText, sizeof(idText), "%08X", object.localFormId);
                return _directory + "\\" + sanitizeForFileName(object.plugin) + "_" + idText + ".json";
            }

            void preload()
            {
                std::error_code ec;
                if (!std::filesystem::exists(_directory, ec) || !std::filesystem::is_directory(_directory, ec)) {
                    return;
                }

                std::unordered_map<std::string, SavedGrabOffsetFile> loaded;
                std::size_t failedCount = 0;
                for (const auto& entry : std::filesystem::directory_iterator(_directory, ec)) {
                    if (ec) {
                        break;
                    }
                    if (!entry.is_regular_file(ec) || entry.path().extension() != ".json") {
                        continue;
                    }

                    std::ifstream stream(entry.path(), std::ios::binary);
                    if (!stream) {
                        ++failedCount;
                        continue;
                    }
                    std::ostringstream buffer;
                    buffer << stream.rdbuf();

                    SavedGrabOffsetFile file{};
                    if (!parse(buffer.str(), file, nullptr)) {
                        ++failedCount;
                        continue;
                    }
                    loaded.emplace(entry.path().string(), std::move(file));
                }

                std::lock_guard lock(_mutex);
                _cache = std::move(loaded);
                ROCK_LOG_INFO(Config, "Loaded {} saved grab offset(s) ({} unreadable)", _cache.size(), failedCount);
            }

            bool load(const FormRef& object, SavedGrabOffsetFile& out, std::string* outError) const
            {
                if (outError) {
                    outError->clear();
                }
                if (object.empty()) {
                    return false;
                }
                const auto path = filePathForObject(object);
                std::lock_guard lock(_mutex);
                const auto it = _cache.find(path);
                if (it == _cache.end()) {
                    return false;  // no saved offset for this object; normal, empty error
                }
                out = it->second;
                return true;
            }

            void save(const SavedGrabOffsetFile& file)
            {
                if (file.object.empty()) {
                    return;
                }
                PendingWrite write{ filePathForObject(file.object), serialize(file) };
                {
                    std::lock_guard lock(_mutex);
                    // Cache first, so the very next grab of this object
                    // (even before the writer thread finishes) sees it.
                    _cache[write.path] = file;

                    // Latest-wins per file: replace a still-pending write of
                    // the same object instead of queueing behind it.
                    bool replaced = false;
                    for (auto& pending : _queue) {
                        if (pending.path == write.path) {
                            pending.content = std::move(write.content);
                            replaced = true;
                            break;
                        }
                    }
                    if (!replaced) {
                        _queue.push_back(std::move(write));
                    }
                }
                ensureWriterStarted();
                _wake.notify_one();
            }

        private:
            void ensureWriterStarted()
            {
                std::lock_guard lock(_mutex);
                if (_writerStarted) {
                    return;
                }
                _writerStarted = true;
                _stop = false;
                _writer = std::thread([this]() { writerLoop(); });
            }

            void writerLoop()
            {
                for (;;) {
                    PendingWrite write;
                    {
                        std::unique_lock lock(_mutex);
                        _wake.wait(lock, [this]() { return _stop || !_queue.empty(); });
                        if (_queue.empty()) {
                            if (_stop) {
                                return;
                            }
                            continue;
                        }
                        write = std::move(_queue.front());
                        _queue.pop_front();
                    }

                    std::error_code ec;
                    std::filesystem::create_directories(std::filesystem::path(write.path).parent_path(), ec);
                    const auto tempPath = write.path + ".tmp";
                    {
                        std::ofstream stream(tempPath, std::ios::binary | std::ios::trunc);
                        if (!stream) {
                            ROCK_LOG_WARN(Config, "Saved grab offset: could not open '{}' for writing", tempPath);
                            continue;
                        }
                        stream.write(write.content.data(), static_cast<std::streamsize>(write.content.size()));
                        if (!stream) {
                            ROCK_LOG_WARN(Config, "Saved grab offset: write to '{}' failed", tempPath);
                            continue;
                        }
                    }
                    std::filesystem::rename(tempPath, write.path, ec);
                    if (ec) {
                        ROCK_LOG_WARN(Config, "Saved grab offset: rename to '{}' failed: {}", write.path, ec.message());
                    }
                }
            }

            void shutdown()
            {
                {
                    std::lock_guard lock(_mutex);
                    if (!_writerStarted) {
                        return;
                    }
                    _stop = true;
                }
                _wake.notify_one();
                if (_writer.joinable()) {
                    _writer.join();
                }
                std::lock_guard lock(_mutex);
                _writerStarted = false;
            }

            std::string _directory;
            std::thread _writer;
            mutable std::mutex _mutex;
            std::condition_variable _wake;
            std::deque<PendingWrite> _queue;
            std::unordered_map<std::string, SavedGrabOffsetFile> _cache;
            bool _stop{ false };
            bool _writerStarted{ false };
        };

        Store& instance()
        {
            static Store store;
            return store;
        }
    }

    FormRef formRefFromRuntimeId(std::uint32_t runtimeFormId)
    {
        if (runtimeFormId == 0) {
            return {};
        }
        auto* form = RE::TESForm::GetFormByID(runtimeFormId);
        if (!form) {
            return {};
        }
        auto* file = form->GetFile(0);
        if (!file) {
            return {};
        }
        FormRef ref;
        ref.plugin = std::string(file->GetFilename());
        ref.localFormId = form->GetLocalFormID();
        if (ref.plugin.empty()) {
            return {};
        }
        return ref;
    }

    void preload()
    {
        instance().preload();
    }

    bool load(const FormRef& object, SavedGrabOffsetFile& out, std::string* outError)
    {
        return instance().load(object, out, outError);
    }

    void save(const SavedGrabOffsetFile& file)
    {
        instance().save(file);
    }
}
