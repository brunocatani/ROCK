#pragma once

#include <ROCK/GrabV1_2.h>
#include <cstdint>
#include <string>

#include "physics-interaction/grab/SavedGrabCaptureFormat.h"
#include "physics-interaction/grab/SavedGrabOffsetFormat.h"

/*
 * Disk side of the saved-grab-offset feature: file naming, runtime-formId
 * <-> plugin-local identity conversion, and the write path. Free functions
 * backed by a single process-lifetime store, so both PhysicsInteraction (save
 * gesture) and Hand::grabSelectedObject (apply on pull-catch/far-grab/
 * force-grab commit) can reach it with no cross-module dependency.
 *
 * All saved offsets live in an in-memory cache for the whole process
 * lifetime: preload() reads every file once at startup, and load() never
 * touches disk afterward (a grab commit is not the place for blocking file
 * I/O). save() updates the cache synchronously so the very next grab of the
 * same object sees it, in addition to queuing the on-disk write.
 *
 * Threading model (mirrors PAPER_Toolkit's MotionLibraryStore for the write
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

    // Copied addon defaults use the existing cache mutex. Public set/clear and
    // native lookup run on the frame owner; fault cleanup may revoke an owner.
    // No file writes. User-saved offsets are checked before this lookup.
    api::Status setDefaultGrabPoses(api::OwnerToken owner, const api::grab::v1_2::DefaultGrabPoses& request);
    api::Status clearDefaultGrabPoses(api::OwnerToken owner, std::uint32_t baseFormId);
    void clearDefaultGrabPosesForOwner(api::OwnerToken owner);
    bool loadDefaultGrabPose(std::uint32_t baseFormId, bool left, HandOffset& out);

    // Cache-only lookup; false when no offset has ever been saved for this
    // object (outError says why on the rare case a cached file failed to
    // parse at preload time; empty error otherwise).
    bool load(const FormRef& object, SavedGrabOffsetFile& out, std::string* outError);

    // Updates the in-memory cache synchronously, then serializes on the
    // calling (frame) thread and writes on the writer thread. No-op when the
    // object ref is empty.
    void save(const SavedGrabOffsetFile& file);

    /*
     * Ground-truth capture companion for the offset just saved: the mesh, hand
     * geometry, physics, contacts and computed seat that the saved pose was
     * produced against, so the pose can be replayed and scored offline.
     * Written to a 'captures' subdirectory (preload() would otherwise try to
     * parse these as offset files), never cached, never read back by ROCK.
     * Same threading contract as save(): frame thread only, disk write on the
     * shared writer thread.
     */
    void saveCapture(const saved_grab_capture::SavedGrabCaptureFile& capture);
}
