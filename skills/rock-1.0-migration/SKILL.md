---
name: rock-1.0-migration
description: Update a Fallout 4 VR F4SE plugin built for ROCK 0.92 (provider API 1, the RPS SDK) to ROCK 1.0 and the providers released with it - ROCK provider API 2, PAPER 2.0, RPS UI Framework 2.0 and RobCo PALM 2.0. Use whenever a mod includes RPS SDK headers (ROCK/*.h from SDK/ROCK/include, PAPERApi.h, RPSUIFrameworkApi.h, RPSUIInputApi.h, RPSUICooperationApi.h, PALMMenuApi.h), calls ROCKAPI_QueryInterfaceV1 with major 1, RPSUI_RequestApi, PAPERAPI_GetProviderApi or GetPALMMenuApi, or when ROCK, PAPER, RPS UI or PALM shows an error box at the main menu saying a mod must be updated - even if the user only says "make my mod work with ROCK 1.0" or "it says my mod is outdated".
---

# Migrating a mod to ROCK 1.0 and its providers

ROCK 1.0 ships together with PAPER 2.0, RPS UI Framework 2.0 and RobCo PALM
2.0. Each publishes a reworked API at major 2 from its own repository's
`include/` directory. None of them serves the old API: a mod built against the
old headers is refused, and the provider stops the game at the main menu with
an error box naming that mod. There is no compatibility layer to keep. The fix
is always a rebuild against the new headers with the code changes each
provider's migration page lists.

| Provider | Old API | New headers | Discovery export | Migration page |
| --- | --- | --- | --- | --- |
| ROCK | provider API 1, RPS SDK `SDK/ROCK/include` | `include/ROCK/` | `ROCKAPI_QueryInterfaceV1` (same name, major 2) | `docs/api/Migration.md` in [ROCK](https://github.com/brunocatani/ROCK) |
| PAPER | `PAPERApi.h`, `PAPERAPI_GetProviderApi` | `include/PAPER/` | `PAPERAPI_QueryInterfaceV1` | `docs/api/Migration.md` in [PAPER](https://github.com/brunocatani/PAPER) |
| RPS UI Framework | `RPSUIFrameworkApi.h`, `RPSUICooperationApi.h`, `RPSUIInputApi.h`, `RPSUI_Request*` | `include/RPS_UI_Framework/` | `RPSUI_QueryInterfaceV1` | `docs/api/Migration.md` in [RPS UI Framework](https://github.com/brunocatani/RPS_UI_Framework) |
| RobCo PALM | `PALMMenuApi.h`, `PALMIcons.h`, `GetPALMMenuApi` | `include/RobCoPALM/` | `PALMAPI_QueryInterfaceV1` | `docs/api/Migration.md` in [RobCo PALM](https://github.com/brunocatani/RobCo_PALM) |

The migration pages are the mapping from every old call, record and flag to
its replacement. This skill is the procedure; read the page of every provider
the mod uses before editing.

## 1. Find what the mod uses

Search the mod's source and build files. Each hit names a provider to migrate:

```text
ROCK:   ROCKAPI_QueryInterfaceV1  rock::api::  <ROCK/  ROCK_SDK|RPS::ROCK  ROCKApi.h  ROCKProviderApi.h  ROCKConfigurationApi.h
PAPER:  PAPERApi.h  PAPERAPI_  paper::
RPS UI: RPSUI  rpsui::sdk  RPS_UI_Framework.dll
PALM:   PALMMenuApi.h  PALMIcons.h  GetPALMMenuApi  palm::api  wheelmenu.dll
SDK:    find_package(RPS_SDK)  RPS_SDK  SDK/ROCK/include  SDK/UI/include  SDK/PAPER/include
```

List, per provider, the interfaces, calls, records and flags the mod touches.
That list is the work.

## 2. Get the headers

Clone each provider the mod uses, at the release the mod will ship against,
and point the build at its `include/` directory through a CMake cache
variable (`ROCK_INCLUDE_DIR`, `PAPER_INCLUDE_DIR`, `RPSUI_INCLUDE_DIR`,
`PALM_INCLUDE_DIR`). Never copy the headers into the mod and never link a
provider: everything binds at runtime. Remove every RPS SDK reference from the
build. The headers need C++20 and a 64-bit build.

## 3. Migrate, provider by provider

Work from the migration page's sections in order. For each provider:

1. Replace discovery with the provider's `Client.h` (`rock::api::Client`,
   `paper::api::Client`, `rpsui::api::acquire`, `palm::api::acquire`). It
   requests exactly the major, minor and table size the headers declare.
2. Rename: `V1` is gone from every call, record and enum; the old SDK
   namespaces (`rpsui::sdk`) and prefixes (`Paper...`) are gone.
3. Fix records: `structSize`/`apiVersion` became `size`/`version`, reserved
   fields are gone, and ROCK's loose frame and generation fields became one
   `Sample sample`. Value-initialize every record.
4. Replace removed calls with what the page names. Where the page says "no
   replacement", remove the feature from the mod and tell the user.
5. Compile, and fix each error from the page's tables. Repeat until it builds.

Do not keep the old code path beside the new one, and do not query an old
major as a fallback: every refused query stops the game. One build of the mod
targets one set of provider majors.

## 4. Changes that compile but behave differently

Check the mod's logic against these even when the code compiles:

- **ROCK weapon slots.** Two weapons can be equipped. Every weapon call takes
  a slot (0 is the first equipped weapon). A slot never implies a hand: read
  `WeaponState.firingHand`/`supportHand` or Hands `getRoles`. `Roles.primary`
  and `offhand` are now per-hand `right`/`left` roles.
- **ROCK event streams are gone.** Code that drained `copyEvents` must follow
  published state and its sequence counters instead (table on the ROCK page).
- **ROCK animation phase callbacks** moved from Core to Animation, need the
  Animation `Callbacks` permission and a `phaseMask`; ROCK calls only the
  phases in the mask.
- **ROCK stick clicks are per hand.** Held placement and the bipod reserve one
  hand's click (`ReserveRightClick`/`ReserveLeftClick`,
  `RightInputReserved`/`LeftInputReserved`).
- **ROCK settings** are TOML tables and keys without type letters
  (`Grabbing` / `FarGrabMode`), matched exactly. Read the catalog through
  Configuration `visit` instead of hard-coding old INI names.
- **ROCK draws nothing for other mods.** Debug drawing moves to RPS UI's
  WorldDraw, and the mod must keep working without RPS UI.
- **RPS UI owns Dear ImGui.** Panel code stops creating contexts, backends and
  frames; it builds Dear ImGui at the host's revision with `ImGuiBuild.h`,
  fills `RPSUI_IMGUI_BUILD()`, starts each callback with `RPSUI_IMGUI_BIND`,
  and draws in layout units with the player's theme.
- **PAPER's observation, weapon-motion and capture calls are removed** with no
  replacement.

## 5. Absent and mismatched providers

- Provider DLL not loaded: turn that provider's features off and keep running.
- Provider loaded but refusing the mod's request: show an error box naming the
  mod and the provider to update, and stop at the main menu, as the providers
  do. RPS Framework's header-only `RPS/Runtime/RequiredUpdate.h`
  ([RPS Framework](https://github.com/brunocatani/RPS_Framework)) provides it.
- Never ask for several versions in turn to find one that works.

## 6. Verify

1. The mod builds against the providers' `include/` directories with no RPS SDK
   path left.
2. Search for leftovers: `V1` outside the discovery names
   (`QueryInterfaceV1`, `InterfaceDescriptorV1`), `reserved`, `structSize`,
   `apiVersion`, `copyEvents`, `rpsui::sdk`, `RPSUI_Request`,
   `PAPERAPI_GetProviderApi`, `GetPALMMenuApi`, literal majors or table sizes.
3. Ask the user to start the game with the new providers: no error box at the
   main menu, the provider logs (`ROCK.log`, `PAPER.log`,
   `RPS_UI_Framework.log` in `Documents/My Games/Fallout4VR/F4SE/`) name no
   refusal for the mod's DLL, and the mod's own log shows each connection
   returning `Ok`. A mod that touches weapons is tested with a weapon in each
   hand.

Report to the user what changed per provider, every feature removed because
its API has no replacement, and what still needs an in-game check.
