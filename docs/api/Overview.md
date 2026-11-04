# ROCK provider API 2.0: overview

ROCK publishes its provider API from `include/ROCK/`. The headers are
self-contained (C++ standard library only) and are the only copy: there is no
SDK, and the release archive does not carry them. Take them from the release
tag you ship against. This page covers what every
interface shares: discovery, the `Client` helper, records, statuses, threads,
generations and weapon slots. Each interface has its own page.

## Interfaces

| Interface | Id | Header | Permissions | Page |
| --- | --- | --- | --- | --- |
| Core | 1 | `Core.h` | Read 1, Callbacks 4 | [Core](Core.md) |
| Hands | 2 | `Hands.h` | Read 1 | [Hands](Hands.md) |
| Collision | 3 | `Collision.h` | Read 1, Write 2 | [Collision](Collision.md) |
| Grab | 4 | `Grab.h` | Read 1, Write 2 | [Grab](Grab.md) |
| Touch | 5 | `Touch.h` | Read 1, Write 2 | [Touch](Touch.md) |
| Weapon | 6 | `Weapon.h` | Read 1, Write 2 | [Weapon](Weapon.md) |
| WeaponParts | 7 | `WeaponParts.h` | Read 1, Write 2 | [WeaponParts](WeaponParts.md) |
| Animation | 8 | `Animation.h` | Read 1, Write 2, Callbacks 4 | [Animation](Animation.md) |
| Input | 9 | `Input.h` | Read 1, Write 2 | [Input](Input.md) |
| PlayerController | 11 | `PlayerController.h` | Read 1, Write 2 | [PlayerController](PlayerController.md) |
| Configuration | 13 | `Configuration.h` | Read 1, Write 2 | [Configuration](Configuration.md) |
| NativeInteraction | 14 | `NativeInteraction.h` | Read 1, Write 2 | [NativeInteraction](NativeInteraction.md) |
| Storage | 15 | `Storage.h` | Read 1, Write 2, Callbacks 4 | [Storage](Storage.md) |

Every interface is major 2, minor 0. Ids 10 (References) and 12 (Diagnostics)
are retired and never reused: References' queries are in Grab, and other mods
draw through RPS UI Framework.

Shared headers: `Abi.h` (common values), `Discovery.h` (the query export),
`Client.h` (consumer helper) and `TransformMath.h` (transform helpers).

## Build

Add ROCK's `include/` directory to your include path (our projects use a
`ROCK_INCLUDE_DIR` CMake cache variable) and include `<ROCK/Client.h>` plus the
interface headers you use. Do not link anything from ROCK: every call goes
through tables ROCK hands out at runtime. Consumers must be 64-bit.

## Discovery

`ROCK.dll` exports `ROCKAPI_QueryInterfaceV1` (`kQueryExportName`):

```cpp
Status ROCKAPI_QueryInterfaceV1(InterfaceId id, std::uint32_t exactMajor,
    std::uint32_t minimumMinor, std::uint32_t minimumTableBytes,
    const InterfaceDescriptorV1** outDescriptor);
```

The descriptor gives `interfaceId`, `major`, `minor`, `tableByteSize`,
`requiredCoreMajor`/`requiredCoreMinor` and the `table`. Pass your own
`sizeof(Api)` as `minimumTableBytes`: a newer ROCK may append slots, never
reorder them.

| Result | Meaning |
| --- | --- |
| `Ok` | `*outDescriptor` points to ROCK's descriptor (valid for the process lifetime). |
| `UnknownInterface` | No such interface, including the retired ids 10 and 12. |
| `UnsupportedMajor` | The interface exists at another major. A mod built against provider API 1 (pre-1.0 ROCK) gets this. |
| `UnsupportedMinor` | ROCK's minor is below `minimumMinor`. |
| `TableTooSmall` | ROCK's table is smaller than `minimumTableBytes`. |
| `InvalidArgument` | Null output, or a zero major or table size. |

A version mismatch stops the game. After `UnknownInterface`,
`UnsupportedMajor`, `UnsupportedMinor` or `TableTooSmall`, ROCK logs the
calling DLL and, when the main menu opens (or at once, if it is already open),
shows an error box naming that DLL and what to update; the game closes when the
box is closed. Request exactly the major, minor and table size your headers
declare, as `Client` does, and read optional additions from the descriptor's
`minor`. Never probe for versions with trial queries.

Your mod should do the same when a loaded ROCK refuses it, so a player with an
older ROCK sees which mod needs which update. RPS Framework's
`RPS/Runtime/RequiredUpdate.h` provides the box. When `ROCK.dll` is not loaded
at all, turn your ROCK features off and keep running.

Discovery grants no permissions. Bind each interface to your owner with the
permissions you need (Core `bindInterface`, or `Client::acquire`).

## Client helper

`rock::api::Client` (in `Client.h`) compiles into your plugin; it is not part
of the DLL ABI.

```cpp
#include <ROCK/Client.h>
#include <ROCK/Hands.h>

rock::api::Client client;
const rock::api::hands::Api* hands{};

void connectRock() {
    auto* module = GetModuleHandleW(L"ROCK.dll");
    if (!module) return;                                   // ROCK absent: run without it
    auto query = reinterpret_cast<rock::api::QueryInterfaceV1>(
        GetProcAddress(module, rock::api::kQueryExportName));
    if (!query || client.connect(query, "My Mod") != rock::api::Status::Ok) return;
    client.acquire(static_cast<std::uint32_t>(rock::api::hands::Permission::Read), hands);
}
```

- `connect(query, name)` queries Core 2.0 and registers one owner. The name is
  your mod's identity: one owner per name (`OwnerConflict` for a second), up to
  `core::kMaxOwners` (64) owners.
- `acquire(permissions, table)` queries an interface by the table type's
  `interfaceId`, `majorVersion` and `minorVersion`, checks the descriptor, binds
  it to your owner and returns the table.
- `close()` (game thread) unregisters the owner and releases everything it
  holds: leases, callbacks, claims, targets. Close before destroying callback
  `userData`.

Pass `client.owner()` as the first argument of every table call.

## Records

Every record that starts with `size` and `version` must have
`size == sizeof(record)` and `version == 1`, whether you pass it in or ROCK
fills it; otherwise the call returns `InvalidSize` or `UnsupportedVersion` and
reads or writes nothing. ROCK clears an output record before filling it. A
record compiled against an older header of a different size is refused, so
rebuild against the headers of the ROCK you ship with.

Copy calls (`copy...`) take a capacity no larger than the interface's
`kMax...` limit (`CapacityFull` otherwise), report the total in `outCount` and
copy `min(capacity, total)` records. Capacity 0 only counts.

## Common values (`Abi.h`)

- `Hand`: `None` 0, `Right` 1, `Left` 2.
- `Transform`: `rotate[9]` (rows of the 3x3 rotation), `translate[3]` in game
  units, `scale`. Game space unless a field name says otherwise.
- `Point3`, `Bounds3` (`min`, `max`, `valid`).
- `Sample`: `frameIndex`, `publicationSequence`, `worldGeneration`,
  `skeletonGeneration`, `providerGeneration`, `collisionGeneration`; every
  interface's `getSample` returns it.
- `ReferenceKey`: `referenceFormId`, `referenceNativeHandle` and the three
  generations; identifies a live reference for requests that act on one.
- `PowerArmorPoint`: `None`, `LeftArmorHand`, `RightArmorHand`.
- `kMaxWeaponSlots` (2): see below.

## Status values

| Value | Meaning |
| --- | --- |
| `Ok` 0 | Done. |
| `NotReady` 1 | ROCK has nothing to report or act on yet (provider, skeleton or first publication missing); retry later. |
| `InvalidArgument` 2 | A bad argument (null pointer, out-of-range value, unknown flag). |
| `InvalidSize` 3, `UnsupportedVersion` 4 | A record's `size` or `version` is wrong. |
| `CapacityFull` 5 | A limit is reached; nothing was evicted. |
| `OwnerNotRegistered` 6 | The owner token is unknown. |
| `OwnerConflict` 7 | Another owner holds that resource. |
| `PermissionDenied` 8 | The interface is not bound with the needed permission. |
| `WorldNotReady` 9 | No live physics world. |
| `TargetInvalid` 10, `TargetUnavailable` 11 | The target does not match, or is not available now. |
| `HandUnavailable` 12, `HandBusy` 13 | The hand cannot take the request, or is doing something else. |
| `ObjectAlreadyOwned` 14 | Another owner or hand already holds the object. |
| `RequestQueued` 15 | Accepted for a later frame; poll for the result. Admission, not success. |
| `RequestNotFound` 16 | No such command or request. |
| `WrongThread` 17 | Called outside the thread or callback phase the function requires. |
| `AlreadyCommitted` 18 | Too late to cancel. |
| `UnknownInterface` 100 to `TableTooSmall` 103 | Discovery results (above). |
| `OwnerRevoked` 104 | ROCK revoked the owner after one of its callbacks faulted. Everything it held is released; reads still work, writes and callbacks are refused. |
| `InternalError` 105 | ROCK failed internally; details are in `ROCK.log`. |
| `Busy` 106 | Try again later (another operation is in progress, or a call from inside a callback that forbids it). |
| `Ambiguous` 107 | A selector matched more than one target. |
| `GenerationMismatch` 108 | The request's generations are stale. |

## Threads and callbacks

Each function's page states its thread: "any" functions read copied state and
are safe from any thread; everything else runs on the game thread, the thread
of FRIK's frame callbacks (calls from elsewhere return `WrongThread`). Before
the first frame, Core frame callbacks and Animation phase callbacks may be
registered from startup. Calling ROCK from inside a callback that forbids it
returns `Busy`. Callback records and strings are borrowed for
the duration of the call. A faulting callback revokes everything its owner
holds (`OwnerRevoked`). Unregistering does not wait for a call already in
progress.

## Generations and samples

ROCK publishes once per frame. The world, skeleton and provider generations
change when the physics world, FRIK's skeleton or ROCK's runtime are rebuilt
(loads, cell changes, power armor). Requests that carry generations are refused
with `GenerationMismatch` when they are stale; copy them from a current
`Sample`. Compare samples before combining two reads: a different
`frameIndex` or `publicationSequence` means ROCK published in between.

Leases (input suppression, authority, targets, displays) last a few frames and
must be renewed every frame; a lost lease shows up as a failed renew. There
are no event streams: follow lasting state through the published records and
their sequence counters.

## Weapon slots

A weapon slot is the native equipment index: 0 is the first equipped weapon, 1
the second (`kMaxWeaponSlots` = 2). A slot never implies a hand; which hand
fires or supports a weapon is data (Hands `getRoles`, Weapon `firingHand`). A
slot at or above `kMaxWeaponSlots` returns `InvalidArgument`.

| Serves both slots | Slot 0 only (slot 1 returns `TargetUnavailable`) |
| --- | --- |
| Weapon `getWeaponState`, `getGrip`, `copyEmitters`, `unequipWeapon`; Hands `getRoles`; Grab hand state flags; Collision `getEnvironment`; NativeInteraction weapon driving | Weapon classification, composition, authored grip and scope sight; every WeaponParts call; Input reload gesture and support trigger; Grab hand reservation and a held object's connected weapon; Collision ejection clearance; Animation `Weapon` authority |

## Transform helpers (`TransformMath.h`)

`rock::transform_math` holds header-only templates over any matrix, vector and
transform type with ROCK's field layout (CommonLib's `NiMatrix3`, `NiPoint3`
and `NiTransform` included): compose and invert transforms, convert points and
vectors between local and world space, orthonormalize rotations, Euler angles,
and the conversions between Ni rows, Havok columns and Havok quaternions. They
follow FO4VR's native compose convention (child-local vectors applied through
the stored basis as `Transpose() * v`) and compute in double precision.
