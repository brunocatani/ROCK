# Core (interface 1, major 2)

Header: `include/ROCK/Core.h`; consumer helper `include/ROCK/Client.h`.
Core registers your mod as an owner, binds the other interfaces to it, and
publishes ROCK's lifecycle state.

## Connect

1. Resolve `ROCKAPI_QueryInterfaceV1` (`kQueryExportName`) from `ROCK.dll`.
   If ROCK is not loaded, skip every ROCK feature and keep running.
2. `rock::api::Client::connect(query, "Your mod")` queries Core major 2 and
   calls `registerConsumer`. An older ROCK refuses major 2; a mod built for
   major 1 is refused by this ROCK with `UnsupportedMajor`.
3. `client.acquire(permissions, table)` queries and binds each other interface.
4. `client.close()` on the game thread unregisters the owner and releases
   everything it holds.

## Functions

| Function | Thread | Permission | Result |
| --- | --- | --- | --- |
| `getModVersion(owner, const char**)` | any | Read | ROCK's version string, valid for the process lifetime. |
| `isProviderReady(owner, uint32*)` | any | Read | 1 when ROCK's interaction runtime is ready. |
| `getSnapshot(owner, Snapshot*)` | any | Read | The latest publication's lifecycle state. `NotReady` before the first publication. |
| `registerConsumer(const Registration*, Owner*)` | any, outside ROCK callbacks | none | A new owner. `OwnerConflict` when the mod name is already registered; `CapacityFull` beyond `kMaxOwners`. |
| `unregisterConsumer(owner)` | game | none | Releases every resource and callback of the owner. |
| `registerFrameCallback(owner, callback, userData, token*)` | game | Callbacks | Calls `callback(snapshot, userData)` once per ROCK publication, up to `kMaxFrameCallbacks`. |
| `unregisterFrameCallback(owner, token)` | game | Callbacks | Stops future calls. |
| `bindInterface(owner, id, major, permissions)` | any | none | Binds a discovered interface major with permission bits. `Busy` if already bound at another major or with permissions this call would drop. |
| `getSample(owner, Sample*)` | any | Read | Identity of the current publication. |

Every owner holds Core with `Read` from registration; bind Core again with
`Read | Callbacks` (5) to register frame callbacks.

## Snapshot

`sample` (frame, publication sequence, generations), `lifecycleFlags`
(`LifecycleFlag`), `lastLifecycleReason`, and ROCK's own readiness this frame:
`frikSkeletonReady`, `menuBlocking`, `configBlocking`, `providerReady`,
`stableFrameCount`, `deltaSeconds`. The lifecycle flags come from ROCK's
lifecycle state machine and can lag the readiness fields during
transitions. Frame callbacks receive the same record.

## Callback rules

The snapshot is borrowed for the call. Unregistering does not wait for a call
already in progress; keep `userData` valid through it. A faulting callback
revokes everything its owner holds.

## Example

```cpp
rock::api::core::Snapshot snapshot{};
if (client.core()->getSnapshot(client.owner(), &snapshot) == rock::api::Status::Ok &&
    snapshot.providerReady && !snapshot.menuBlocking) {
    // act this frame
}
```
