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
 * All saved offsets live in an in-memory cache for the whole process
 * lifetime: preload() reads every file once at startup, and load() never
 * touches disk afterward (a grab commit is not the place for blocking file
 * I/O). save() updates the cache synchronously so the very next grab of the
 * same object sees it, in addition to queuing the on-disk write.
 *
 * Threading model (mirrors PAPER_Redux's MotionLibraryStore for the write
 * side):
 *  - Every function below is FRAME-THREAD ONLY.
 *  - save() updates the in-memory cache immediately, then enqueues a
 *    pre-serialized string for a single background writer thread
 *    (latest-wins per file) - the disk write never touches the frame
 *    thread. Files are written to a .tmp sibling and renamed into place so
 *    a crash mid-write cannot corrupt a saved offset.
 *  - preload() reads every existing file once, synchronously, at plugin
 *    startup only (see ROCKMain.cpp), before any grab can occur.
 */
namespace rock::saved_grab_offset
{
    /*
     * Non-throwable weapons never participate in saved grab offsets, on
     * either side (dev-mode save gesture and pull-catch/force-grab apply):
     * guns and melee seat through the FRIK weapon-offset pipeline alone
     * (canonical primary attach / mirrored firing hold), and a saved hold
     * captured outside that pipeline seats the weapon away from the firing
     * grip the grip-zone equip snaps it to (stale left-only Service Rifle
     * save, 2026-07-13). Grenades and mines are hand-thrown refs with no
     * FRIK attach; they keep the saved/calibrated offset path.
     */
    [[nodiscard]] constexpr bool participatesInSavedGrabOffsets(bool isWeaponForm, bool isThrowableWeapon)
    {
        return !isWeaponForm || isThrowableWeapon;
    }

    // Runtime formId -> load-order-independent ref (empty on failure /
    // formId 0). Engine lookup; frame thread only.
    [[nodiscard]] FormRef formRefFromRuntimeId(std::uint32_t runtimeFormId);

    // Populates the in-memory cache from every saved-offset file on disk.
    // Call once at plugin startup, before any grab can occur.
    void preload();

    // Cache-only lookup; false when no offset has ever been saved for this
    // object (outError says why on the rare case a cached file failed to
    // parse at preload time; empty error otherwise).
    bool load(const FormRef& object, SavedGrabOffsetFile& out, std::string* outError);

    // Updates the in-memory cache synchronously, then serializes on the
    // calling (frame) thread and writes on the writer thread. No-op when the
    // object ref is empty.
    void save(const SavedGrabOffsetFile& file);
}
