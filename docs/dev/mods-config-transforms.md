# Ticket: firing-hand offsets as Mods Config transforms

**Status:** open, not started. **Depends on:** a storage decision (see Options).

## Goal

Show the three `ROCK.toml` offsets that the Mods Config page leaves out today, as `cf.transform` rows with Adjust in world:

| Key | Today | Range (`RockConfigLoad.cpp`) |
| --- | --- | --- |
| `[FiringHand.Left] AimOffsetGameUnits` | `[x, y, z]`, game units | each -15..15 |
| `[FiringHand.Left] GripOffsetGameUnits` | `[along the fingers, into/out of the palm, across the palm]` | each -15..15 |
| `[FiringHand.Right] SupportGripOffsetGameUnits` | same axes | each -15..15 |

`ROCKModsConfigSpecTests` lists them in `kLeftOut`. Move them out of it when they're on the page.

## Why they're left out

- They are RPS Settings `Float3` values, a TOML array `[x, y, z]`. Mods Config's schema says an array is one value with no control.
- `cf.transform` in a TOML file reads an inline table, `{ pos = [x, y, z], rot = [heading, roll, attitude], scale = s }`. Its line form (`x,y,z;h,r,a;s`, all seven numbers) is read only from INI files.
- RPS Settings (`RPS/Settings/Store.h`) has no table type: Boolean, Integer, Float, String, Float3.

## Options

1. **Mods Config reads a TOML array as a transform's position.** An opt-in field on `cf.transform` (position only: `rotation: false`, `scale: false`) that reads and writes `[x, y, z]`. No change in ROCK or RPS. Smallest change.
2. **Mods Config adds a three-number control** for any `[x, y, z]` array: three sliders with one range. No Adjust in world.
3. **RPS Settings gains a transform (inline table) type**, and ROCK migrates these keys to transform tables. That can merge `AimYawDegrees`, `AimPitchDegrees` and `AimOffsetGameUnits` into one left-aim transform (position plus heading and attitude). It changes RPS Framework, ROCK (migration, catalog, examples) and RPS UI / PALM's editor, which shows `Float3` as text today.

Lean: option 1, then decide whether the aim yaw and pitch should join the aim offset.

## To check first

- Axis order and meaning. ROCK's grip offsets are hand-local (along the fingers, palm, across), and the aim offset's frame isn't documented. A transform's pos is "relative to its node". Adjust in world must move the value along the axes the player sees.
- ROCK's `Float3` rotations are `[pitch, yaw, roll]` (`docs/api/Configuration.md`), while the framework's are `[heading, roll, attitude]`. This matters only if option 3 merges the yaw and pitch.
- Whether Adjust in world needs a native client (see `mods-config-native-api.md`) to show the change live, or works with `applies: save`.
- Right-hand values are mirrored from the left; check that the editor's sign matches what the player sees.
