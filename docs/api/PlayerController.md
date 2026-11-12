# PlayerController (interface 11, major 2)

Header: `include/ROCK/PlayerController.h`. Reads the player's native
character controller and requests native jumps (wall climbing uses both).

## Discover and bind

Interface `PlayerController`, major 2, minor 0; major 1 is refused with
`UnsupportedMajor`. Permissions: `Read` (1) for the state, `Write` (2) for
jumps.

```cpp
const rock::api::playercontroller::Api* controller{};
client.acquire(3, controller); // Read | Write
```

## Functions

| Function | Thread | Result |
| --- | --- | --- |
| `getPlayerControllerState(owner, PlayerControllerState*)` | game | Controller position, velocity, support and shape. `NotReady` before the provider and controller are ready. |
| `requestPlayerControllerJump(owner, const PlayerControllerJumpRequest*)` | game | Starts a native jump of `heightGameUnits`. `InvalidArgument` for a height outside (0, `kMaxNativeJumpHeightGame` = 256] or a zero generation; `GenerationMismatch` for stale generations; `NotReady` while physics writes are not allowed. |
| `getSample(owner, Sample*)` | any | Identity of the current publication. |

Records must have `size == sizeof(record)` and `version == 1`.

## Records

- `PlayerControllerState`: `sample`; `flags` (`PlayerControllerStateFlag`:
  `Valid`, `PositionValid`, `VelocityValid`, `ShapeValid`,
  `SupportNormalValid`, `Supported`, `Sliding`, `Proxy`, `RigidBody`);
  `implementation` (`PlayerControllerImplementation`: `Unknown`, `Proxy` or
  `RigidBody` controller); `supportState` (`PlayerSupportState`:
  `Unsupported`, `Sliding`, `Supported`); `positionGame`, `velocityGame`,
  `supportNormalGame`; `radiusGame`, `heightGame` (capsule shape). Read a
  vector only when its `...Valid` flag is set.
- `PlayerControllerJumpRequest`: `heightGameUnits` and the world, skeleton and
  provider generations copied from a state's `sample`.

## Retired values

`PlayerControllerStateFlag` bits 7 and 8 (penetration) are retired.

## Example

```cpp
rock::api::playercontroller::PlayerControllerState state{};
if (controller->getPlayerControllerState(client.owner(), &state) == rock::api::Status::Ok) {
    rock::api::playercontroller::PlayerControllerJumpRequest jump{};
    jump.heightGameUnits = 64.0f;
    jump.worldGeneration = state.sample.worldGeneration;
    jump.skeletonGeneration = state.sample.skeletonGeneration;
    jump.providerGeneration = state.sample.providerGeneration;
    controller->requestPlayerControllerJump(client.owner(), &jump);
}
```
