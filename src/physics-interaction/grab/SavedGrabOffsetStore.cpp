#include "physics-interaction/grab/SavedGrabOffsetStore.h"

#include "physics-interaction/PhysicsLog.h"

#include "common/CommonUtils.h"

#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESDataHandler.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace rock::saved_grab_offset
{
    namespace
    {
        constexpr auto kSavedGrabOffsetsRelativePath = R"(\My Games\Fallout4VR\ROCK_Config\SavedGrabOffsets)";

        std::string resolveStoreDirectory()
        {
            return common::getRelativePathInDocuments(kSavedGrabOffsetsRelativePath);
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
    }

    SavedGrabOffsetStore::SavedGrabOffsetStore() :
        _directory(resolveStoreDirectory())
    {
    }

    SavedGrabOffsetStore::~SavedGrabOffsetStore()
    {
        shutdown();
    }

    FormRef SavedGrabOffsetStore::formRefFromRuntimeId(std::uint32_t runtimeFormId)
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

    std::string SavedGrabOffsetStore::filePathForObject(const FormRef& object) const
    {
        char idText[16]{};
        std::snprintf(idText, sizeof(idText), "%08X", object.localFormId);
        return _directory + "\\" + sanitizeForFileName(object.plugin) + "_" + idText + ".json";
    }

    bool SavedGrabOffsetStore::load(const FormRef& object, SavedGrabOffsetFile& out, std::string* outError) const
    {
        if (outError) {
            outError->clear();
        }
        if (object.empty()) {
            return false;
        }
        const auto path = filePathForObject(object);
        std::error_code ec;
        if (!std::filesystem::exists(path, ec)) {
            return false;  // absent file: normal, empty error
        }
        std::ifstream stream(path, std::ios::binary);
        if (!stream) {
            if (outError) {
                *outError = "file exists but could not be opened";
            }
            return false;
        }
        std::ostringstream buffer;
        buffer << stream.rdbuf();
        return parse(buffer.str(), out, outError);
    }

    void SavedGrabOffsetStore::save(const SavedGrabOffsetFile& file)
    {
        if (file.object.empty()) {
            return;
        }
        PendingWrite write{ filePathForObject(file.object), serialize(file) };
        {
            std::lock_guard lock(_mutex);
            // Latest-wins per file: replace a still-pending write of the
            // same object instead of queueing behind it.
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

    void SavedGrabOffsetStore::ensureWriterStarted()
    {
        std::lock_guard lock(_mutex);
        if (_writerStarted) {
            return;
        }
        _writerStarted = true;
        _stop = false;
        _writer = std::thread([this]() { writerLoop(); });
    }

    void SavedGrabOffsetStore::writerLoop()
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

    void SavedGrabOffsetStore::shutdown()
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
}
