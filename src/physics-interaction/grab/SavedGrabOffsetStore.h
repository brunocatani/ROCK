#pragma once

#include <cstdint>
#include <string>

#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

/*
 * Disk side of the saved-grab-offset feature: file naming, runtime-formId
 * <-> plugin-local identity conversion, and the write path. Free functions
 * backed by a single process-lifetime store, matching the
 * frik_weapon_offset_cache pattern, so both PhysicsInteraction (save
 * gesture) and Hand::grabSelectedObject (apply on pull-catch/far-grab/
 * force-grab commit) can reach it with no cross-module dependency.
 *
 * Threading model (mirrors PAPER_Redux's MotionLibraryStore):
 *  - Every function below is FRAME-THREAD ONLY.
 *  - save() enqueues a pre-serialized string for a single background
 *    writer thread (latest-wins per file) - writes never touch the frame
 *    thread. Files are written to a .tmp sibling and renamed into place so
 *    a crash mid-write cannot corrupt a saved offset.
 *  - load() reads synchronously on a grab-commit or save event only (never
 *    per frame): a few hundred bytes, the same class as config loads.
 */
namespace rock::saved_grab_offset
{
    // Runtime formId -> load-order-independent ref (empty on failure /
    // formId 0). Engine lookup; frame thread only.
    [[nodiscard]] FormRef formRefFromRuntimeId(std::uint32_t runtimeFormId);

    // Synchronous read+parse; false when the file is absent or unusable
    // (outError says which; absent file sets an empty error).
    bool load(const FormRef& object, SavedGrabOffsetFile& out, std::string* outError);

    // Serialize on the calling (frame) thread, write on the writer thread.
    // No-op when the object ref is empty.
    void save(const SavedGrabOffsetFile& file);
}
