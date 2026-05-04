# ROCK Physical Reloads Tracker

This tracker is now a detachment record. ROCK-era physical reload implementation
evidence remains useful as local source-of-truth history, but active physical
reload runtime work belongs in `PAPER`.

## 2026-05-02 PAPER Detachment Status

ROCK physical reload runtime ownership is detached. PAPER owns reload state,
profiles, native ammo gates, visual ownership, reload bodies, Papyrus/config
behavior, diagnostics, and overlay work. ROCK keeps hand physics, generated
weapon collision, support-grip/two-handed-grip behavior, FRIK lifecycle
integration, and the provider API.

PAPER remains inside the ROCK-rooted runtime stack. It is not a standalone
replacement for ROCK; it is the reload owner that depends on ROCK's provider and
FRIK-backed skeleton state.

## Active ROCK Boundary

| Area | Active owner | Evidence |
|---|---|---|
| Native reload/ammo hooks | PAPER | ROCK hook code removed; PAPER installs the accepted ROCK source-of-truth hook |
| Reload native event bridge | PAPER | ROCK event bridge removed; PAPER owns `WeaponReloadEventBridge` |
| Reload profiles and authoring | PAPER | ROCK profile files/scripts/config removed; PAPER uses `PAPER_Config\\WeaponProfiles` |
| Reload visuals and bodies | PAPER | ROCK reload body/runtime files removed; PAPER maps reload body metadata to ROCK provider registrations |
| Weapon collision evidence | ROCK | `WeaponCollisionEvidence.h`, provider evidence descriptors |
| External body contacts | ROCK provider | `ROCKAPI_GetProviderApi`, external body registry, provider boundary tests |
| Offhand reservation | ROCK provider, requested by PAPER | PAPER sets `ReloadReserved`; ROCK support/two-hand logic consumes the generic reservation |
| Public ROCK reload API | Detached compatibility only | `ROCKApi.h` v4 has no reload members; hidden v3 slots return idle/false |
| ROCK INI/package | Reload-free | Repo and active `ROCK_Config\\ROCK.ini` have no reload keys; package has no reload scripts |

## Verification

- `cmake --build build --config Release --target ROCK`
- ROCK focused test executables: provider boundary, transform conventions, body
  frame policy, selection highlight, world-origin diagnostics, input remap, and
  grab-finger local transform.
- `tests/ReloadDetachmentSourceTests.ps1`

New reload implementation notes should be recorded under `PAPER/docs/`.
