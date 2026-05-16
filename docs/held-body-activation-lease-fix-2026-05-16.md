# Held Body Activation Lease Fix - 2026-05-16

## Why This Approach

The proxy-constraint motor should remain the production authority, but the spring-era backup had one important side effect that the motor replacement lost: the native held-object action leased an additional body flag while the selected object was held. Reintroducing the native action would split authority again, so this fix ports only the body ownership/activation contract into the current proxy path.

## Local Findings

- Backup path: `E:\fo4dev\Backups\before motor change\PROJECT_ROCK_V2\ROCK`.
- The old native held-object action used `kNativeGrabBodyFlags = 0x08000000u` with body-flag mode `1`.
- Current proxy grab already leased `0x80` with mode `0` for the held set, but did not lease the former native held-authority flag.
- Close-grab commit zeroed held-body velocities before the proxy drive existed. A zero velocity write is not a reliable wake/activation event, so already-dynamic or multipart objects could enter the hold without every accepted body being explicitly activated.

## Implemented

- Added named helpers in `HandGrab.cpp` for held-body activation and body-flag leasing.
- The helpers operate on a primary-first unique body list derived from `_savedObjectState.bodyId` and `_heldBodyIds`.
- Grab commit explicitly activates the accepted held body set after zeroing velocities.
- Grab commit leases both held collision participation (`0x80`, mode `0`) and held authority support (`0x08000000`, mode `1`).
- Release restores both lease families with the existing `finalObjectRelease` discipline so peer-held objects keep their leases until the last hand releases.
