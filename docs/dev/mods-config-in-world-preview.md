# Ticket: Show in the world what a Mods Config row sets

**Status:** open, the row markers not started.

## Goal

While the player points at a row of ROCK's Mods Config page, or has its page open, show in the world what the row sets. Reference: Mods Config's `docs/for-mod-authors/native-api.md` ("The Menu's Events", "What the Ray Is On").

## What the client would add

| API | Use in ROCK |
| --- | --- |
| `isPointedAt` / `subscribeToPointing` | Show in the world what a row sets while the player points at it: grip zone markers for Grip Zone Marker Size, the reach zone for Firing Grip Reach. |
| `subscribeToMenuEvents` | Show those markers while the Weapons page is shown (give it an `id`). |

## Adjust in world

Mods Config offers it for any `live` transform whose mod's client is connected, so the firing-hand transforms (`AimOffset`, `GripOffset`, `SupportGripOffset`) got it with the live preview and ROCK adds no code. Seen working in the headset on 2026-11-23. The sticks move the stored X, Y and Z (right stick X and Y, left stick Z), not axes relative to the view. The grip offsets are hand-local (along the fingers, into or out of the palm, across the palm); the aim offset's frame isn't documented.

## Work

- [ ] Give the rows and the Weapons page `id`s, and draw their markers through RPS UI Framework while they're pointed at or shown.
- [ ] Headset check: `GripOffset` and `SupportGripOffset` (help: "Regrab to apply") visibly move while tuning, and the signs make sense on the mirrored right hand.
