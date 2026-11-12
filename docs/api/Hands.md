# Hands (interface 2, major 2)

Header: `include/ROCK/Hands.h`. Read-only copies of ROCK's hand, head and
hand-role publications.

## Discover and bind

```cpp
#include <ROCK/Client.h>
#include <ROCK/Hands.h>

rock::api::Client client;              // connect(query, "My mod") first
const rock::api::hands::Api* hands{};
client.acquire(static_cast<std::uint32_t>(rock::api::hands::Permission::Read), hands);
```

`acquire` queries `ROCKAPI_QueryInterfaceV1` for interface `Hands`, major 2,
minor 0, and binds it to the client's owner. A major-1 request is refused with
`UnsupportedMajor`.

Permission: `Read` (1), required by every function.

## Functions

| Function | Thread | Result |
| --- | --- | --- |
| `getHandFrame(owner, hand, HandFrame*)` | any | ROCK's control transform for the hand. `NotReady` until the provider has published that hand. |
| `getPresentedHandFrame(owner, hand, HandFrame*)` | game | The rendered wrist of the last completed presentation. `WrongThread` off the game thread; `NotReady` before the first completed capture. |
| `getPresentedHandPose(owner, hand, PresentedHandPose*)` | game | The rendered wrist and 15 finger joints of the last completed presentation. `TargetUnavailable` when no capture exists for that hand. |
| `getHeadPose(owner, HeadPose*)` | any | HMD transform and forward vector of the control publication. |
| `getRoles(owner, Roles*)` | any | What each physical hand holds of the equipped weapons: the weapon slot and the grip role. |
| `getSample(owner, Sample*)` | any | Identity of the current publication. |

`hand` must be `Hand::Right` or `Hand::Left`; anything else is
`InvalidArgument`.

Every output record must have `size == sizeof(record)` and `version == 1`;
otherwise the call returns `InvalidSize` or `UnsupportedVersion` and writes
nothing. On any other failure the record is cleared.

## Records

- `HandFrame`: `hand`, `flags` (`HandFrameFlag`), `transform` (world), `sample`.
  - `getHandFrame` sets `Valid | RootFlattenedAuthority`, plus `Left` for the
    left hand.
  - `getPresentedHandFrame` sets `Valid | PresentedVisual`, plus `Left`. Its
    sample takes frame and generations from the presentation and the
    publication sequence and collision generation from the control
    publication frozen with that capture.
- `PresentedHandPose`: `hand`, `flags` (`PresentedHandPoseFlag`), `handWorld`,
  and `fingerLocalTransforms[kFingerLocalTransformCount]` (15), each relative to its parent joint, in the
  order thumb, index, middle, ring, pinky (three joints each). Bit N of
  `fingerLocalTransformMask` marks joint N as captured. The pose carries its own
  `frameIndex`, `presentationSequence` and generations.
- `HeadPose`: `sample`, `flags` (`HeadPoseFlag::TransformValid`,
  `ForwardValid`), `transform`, `forwardWorld`.
- `Roles`: `sample`, `right`, `left`, each a `HandWeaponRole`: `weaponSlot`
  (`kNoWeaponSlot` when the hand holds no equipped-weapon grip) and `role`
  (`HandRole`). `Firing` is the weapon's firing grip; `Support` is any other
  grip of that weapon (its support grip or another part grip). A hand holds at
  most one grip of one weapon. Roles follow grip transfers; they are not the
  player's configured default hand.

Transforms are game-space: `rotate` holds rows of the 3x3 matrix, `translate`
is in game units.

## Combining reads

Records from the same publication carry the same `Sample`. Compare
`frameIndex`, `publicationSequence` and the generations before combining two
reads; a mismatch means ROCK published in between.

## Retired values

`HandFrameFlag` bits 2 and 3 (primary/offhand role) and 4 (scene node) are
retired and never reused.

## Example

```cpp
rock::api::hands::HandFrame frame{};
if (hands->getHandFrame(client.owner(), rock::api::Hand::Right, &frame) == rock::api::Status::Ok) {
    const float x = frame.transform.translate[0];
}
```
