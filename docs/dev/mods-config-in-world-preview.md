# Ticket: Show in the world what a Mods Config row sets

**Status:** open, not started. Needs the live preview of `mods-config-native-api.md`.

## Goal

While the player points at a row of ROCK's Mods Config page, or has its page open, show in the world what the row sets, and let the firing-hand transforms be adjusted in the world. Reference: Mods Config's `docs/for-mod-authors/native-api.md` ("The Menu's Events", "What the Ray Is On").

## What the client would add

| API | Use in ROCK |
| --- | --- |
| `isPointedAt` / `subscribeToPointing` | Show in the world what a row sets while the player points at it: grip zone markers for Grip Zone Marker Size, the reach zone for Firing Grip Reach. |
| `subscribeToMenuEvents` | Show those markers while the Weapons page is shown (give it an `id`). |

## Work

- [ ] Give the rows and the Weapons page `id`s, and draw their markers through RPS UI Framework while they're pointed at or shown.
- [ ] Adjust in world for the firing-hand transforms (`AimOffset`, `GripOffset`, `SupportGripOffset`): check which frame each position is in (the grip offsets are hand-local: along the fingers, palm, across; the aim offset's frame isn't documented), and that the sticks move the value along the axes the player sees, also on the mirrored right hand.
