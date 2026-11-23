# Ticket: Check Adjust in world on the firing-hand offsets

**Status:** open, the headset check not done.

## What works

Mods Config offers Adjust in world for any `live` transform whose mod's client is connected, so the firing-hand transforms (`AimOffset`, `GripOffset`, `SupportGripOffset`) got it with the live preview and ROCK adds no code. Seen working in the headset on 2026-11-23. The sticks move the stored X, Y and Z (right stick X and Y, left stick Z), not axes relative to the view. The grip offsets are hand-local (along the fingers, into or out of the palm, across the palm); the aim offset's frame isn't documented.

## Work

- [ ] Headset check: `GripOffset` and `SupportGripOffset` (help: "Regrab to apply") visibly move while tuning, and the signs make sense on the mirrored right hand.
- [ ] Find and document the aim offset's frame, in its help if a player needs it.
