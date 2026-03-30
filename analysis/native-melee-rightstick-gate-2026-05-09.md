# Native Melee RightStick Gate Finding

Date: 2026-05-09

## Why This Exists

The previous native melee suppression build correctly applied the FO4VR `VRInput`
melee velocity settings and installed the `WeaponSwingHandler` and
`HitFrameHandler` animation-event hooks, but gameplay still showed native melee
classification during fast turns and repeated swings. That means the missed path
was not deployment drift or the animation-event handlers.

## Verified FO4VR Path

With user-approved Ghidra inspection of the loaded FO4VR binary:

- `WeaponSwingHandler::vfunction2` is at `0x140FEF820`.
- `HitFrameHandler::vfunction2` is at `0x140FEFFB0`.
- `AttackBlockHandler::ShouldHandleEvent` is at `0x140FCD770`.
- `AttackBlockHandler` vtable starts at `0x142D8A348`.
- Its `ShouldHandleEvent` slot is `0x142D8A350`, relative offset `0x2D8A350`.

The VR branch of `AttackBlockHandler::ShouldHandleEvent` accepts
`PrimaryAttack`, `SecondaryAttack`, and `RightStick`. The missed native melee
path is the `RightStick` branch, which can classify fast rotation or swing-like
motion before the later animation-event hooks run.

## Implementation Boundary

ROCK suppresses only the `RightStick` event in this input gate, and only while
full native melee suppression is active. `PrimaryAttack` and `SecondaryAttack`
must keep calling native so normal weapon/fire/block inputs remain unchanged.
The existing VRInput threshold watchdog and animation-event hooks stay installed
as separate layers.
