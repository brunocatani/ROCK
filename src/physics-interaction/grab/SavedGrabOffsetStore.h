#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

namespace rock::saved_grab_offset
{
    /*
     * Disk side of the saved-grab-offset feature: file naming, runtime-formId
     * <-> plugin-local identity conversion, and the write path.
     *
     * Threading model (mirrors PAPER_Redux's MotionLibraryStore):
     *  - Every public method is FRAME-THREAD ONLY.
     *  - save() enqueues a pre-serialized string for a single background
     *    writer thread (latest-wins per file, bounded by natural request
     *    rate) - writes never touch the frame thread. Files are written to a
     *    .tmp sibling and renamed into place so a crash mid-write cannot
     *    corrupt a saved offset.
     *  - load() reads synchronously ON A FORCE-GRAB COMMIT only (never
     *    per frame): a few hundred bytes, the same class as config loads.
     *  - shutdown() drains the queue and joins the writer thread.
     */
    class SavedGrabOffsetStore
    {
    public:
        SavedGrabOffsetStore();
        ~SavedGrabOffsetStore();

        // Runtime formId -> load-order-independent ref (empty on failure /
        // formId 0). Engine lookup; frame thread only.
        [[nodiscard]] static FormRef formRefFromRuntimeId(std::uint32_t runtimeFormId);

        [[nodiscard]] const std::string& directory() const { return _directory; }
        [[nodiscard]] std::string filePathForObject(const FormRef& object) const;

        // Synchronous read+parse; false when the file is absent or unusable
        // (outError says which; absent file sets an empty error).
        bool load(const FormRef& object, SavedGrabOffsetFile& out, std::string* outError) const;

        // Serialize on the calling (frame) thread, write on the writer
        // thread. No-op when the object ref is empty.
        void save(const SavedGrabOffsetFile& file);

        void shutdown();

    private:
        struct PendingWrite
        {
            std::string path;
            std::string content;
        };

        void ensureWriterStarted();
        void writerLoop();

        std::string _directory;
        std::thread _writer;
        std::mutex _mutex;
        std::condition_variable _wake;
        std::deque<PendingWrite> _queue;
        bool _stop{ false };
        bool _writerStarted{ false };
    };
}
