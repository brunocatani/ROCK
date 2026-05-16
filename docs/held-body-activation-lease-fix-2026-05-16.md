# Held Body Activation Lease Fix - 2026-05-16

## Why This Approach

The proxy-constraint motor should remain the production authority, but the spring-era backup had one important side effect that the motor replacement lost: the native held-object action leased an additional body flag while the selected object was held. Reintroducing the native action would split authority again, so this fix ports only the body ownership/activation contract into the current proxy path.

## Local Findings

- Backup path: `E:\fo4dev\Backups\before motor change\PROJECT_ROCK_V2\ROCK`.
- The old native held-object action used `kNativeGrabBodyFlags = 0x08000000u` with body-flag mode `1`.
- That native authority flag was leased only on the selected primary body passed to the native action. ROCK separately leased `0x80` mode `0` across `_heldBodyIds`.
- Current proxy grab already leased `0x80` with mode `0` for the held set, but did not lease the former native held-authority flag.
- Close-grab commit zeroed held-body velocities before the proxy drive existed. A zero velocity write is not a reliable wake/activation event, so already-dynamic or multipart objects could enter the hold without every accepted body being explicitly activated.
- Shared two-hand grabs carried the peer hand's accepted body list in `GrabSharedObjectContext`, but the joining hand still committed `_heldBodyIds` from its own close-selection rescan. On multipart refs that made activation, flag leases, release velocity, and final restore depend on two different body inventories for the same object.

## Implemented

- Added named helpers in `HandGrab.cpp` for held-body activation and body-flag leasing.
- The helpers operate on a primary-first unique body list derived from `_savedObjectState.bodyId` and `_heldBodyIds`.
- Grab commit explicitly activates the accepted held body set after zeroing velocities.
- Grab commit leases held collision participation (`0x80`, mode `0`) across the accepted held body set.
- Grab commit leases held authority support (`0x08000000`, mode `1`) only on the primary body, matching the former native action side effect without turning every multipart child body into a grab-authority body.
- Release restores both lease families with the existing `finalObjectRelease` discipline so peer-held objects keep their leases until the last hand releases.
- Shared grab commit now builds `_heldBodyIds` through `buildCommittedHeldBodyIds()`: a joining hand keeps its selected primary body first for its own drive frame, then adopts the peer hand's committed held-body list for object-wide ownership.
