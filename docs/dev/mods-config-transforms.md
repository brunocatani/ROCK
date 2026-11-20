# Ticket: firing-hand offsets as Mods Config transforms

**Status:** done. The three offsets are `cf.transform` rows on the Firing Hand section. Adjust in world waits for the native API (`mods-config-native-api.md`).

## What was done

| Key | Before | Now |
| --- | --- | --- |
| `[FiringHand.Left] AimOffset` | `AimOffsetGameUnits = [x, y, z]` | `{ pos = [x, y, z] }` |
| `[FiringHand.Left] GripOffset` | `GripOffsetGameUnits = [...]` | `{ pos = [along the fingers, into/out of the palm, across the palm] }` |
| `[FiringHand.Right] SupportGripOffset` | `SupportGripOffsetGameUnits = [...]` | same axes |

- RPS Settings (fork `ArthurHub/FO4VR-RPS_Framework`, branch `settings-transform-type`, to go upstream as a PR) gained a `Transform` value type, a renamed-key table, and INI `Float3Component` rows that fill a transform's `pos`. ROCK builds only against it until that merges.
- ROCK renamed the keys (`kRenamedKeys` in `src/config/RockSettings.cpp` carries the old values over) and retargeted the frozen INI rows.
- The Configuration API published each as a `Float3` position (`apiSetting`), so RPS UI's ROCK page kept working. Both go with the Configuration API (`mods-config-palm-removal.md`).
- Spec rows: position only, -15..15 by 0.1, `rotation: false`, `scale: false`.

## Not done

- **Aim yaw and pitch** stay separate keys. Merging them into the aim transform's `rot` means mapping ROCK's `[pitch, yaw, roll]` onto `[heading, roll, attitude]` and checking the mirrored right hand's signs.
- **Adjust in world** needs `applies: live` and a connected client. Its checks (which frame the axes are in, that the editor's signs match what the player sees on the mirrored right hand) are in `mods-config-native-api.md`.
