# Configuration (interface 13, major 2)

Header: `include/ROCK/Configuration.h`. Reads and changes ROCK's TOML settings
(`ROCK.toml` and `ROCK_Developer.toml`) through ROCK's own catalog, the same
way PALM's ROCK pages do.

## Discover and bind

```cpp
const rock::api::configuration::Api* config{};
client.acquire(static_cast<std::uint32_t>(rock::api::configuration::Permission::Read) |
               static_cast<std::uint32_t>(rock::api::configuration::Permission::Write), config);
```

Interface `Configuration`, major 2, minor 0. Major 1 is refused with
`UnsupportedMajor`.

Permissions: `Read` (1) for `revision` and `visit`; `Write` (2) for
`setValue` and `openSetupWizard`.

## Functions

| Function | Thread | Result |
| --- | --- | --- |
| `revision(owner, uint64* out)` | any | A number that increases whenever any setting changes, including hot reloads. |
| `visit(owner, group, visitor, user)` | any | Calls `visitor(setting, user)` once per setting of `Group::Consumer` (`ROCK.toml`) or `Group::Developer` (`ROCK_Developer.toml`), in catalog order. `NotReady` before the catalog loads. |
| `openSetupWizard(owner)` | any | Opens ROCK's setup wizard (controllers and main options) in front of the player on the next frame. `NotReady` when RPS UI Framework is unavailable or the game is not in play; `Ok` when the wizard is open or opening. |
| `setValue(owner, group, section, key, value, error, errorCapacity)` | any | Validates, applies and saves one setting to its owning file. `InvalidArgument` when rejected; `error` then holds the reason (truncated to `errorCapacity`). A file with a syntax error is not overwritten: the write is rejected until the file is fixed. |

## Records

`Setting` (borrowed for the duration of one visitor call only): `section`,
`key`, `value`, `defaultValue`, `category`, `description` (UTF-8 strings),
`type` (`ValueType`), and `overridden` (nonzero when the value differs from
ROCK's compiled default).

`section` is the TOML table path that holds the key, for example `Grabbing` or
`ControllerInput.Left`. `section` and `key` are case-sensitive and identify the
setting exactly; pass them to `setValue` as `visit` reports them.

| `ValueType` | `value` text | `setValue` accepts |
| --- | --- | --- |
| `Boolean` | `true` or `false` | also `1`/`0`, `yes`/`no`, `on`/`off` |
| `Integer` | decimal whole number | a whole number |
| `Float` | decimal number, always with `.` or an exponent (`4.0`) | any finite number |
| `String` | the text itself, unquoted | one line of text |
| `Float3` | `[x, y, z]`, for example `[0.5, -1.0, 0.0]` | three finite numbers, with or without brackets |

Rotations stored as `Float3` are `[pitch, yaw, roll]`; the setting's
`description` says which form it uses.

## Callback rules

The visitor runs synchronously inside `visit`. It must not call back into
ROCK. A visitor that faults revokes the owner (`OwnerRevoked`). Copy any
string you need before returning.

## Example

```cpp
std::uint64_t seen{};
if (config->revision(client.owner(), &seen) == rock::api::Status::Ok) {
    config->visit(client.owner(), rock::api::configuration::Group::Consumer,
        [](const rock::api::configuration::Setting* setting, void*) { /* copy what you need */ },
        nullptr);
}
char error[256]{};
config->setValue(client.owner(), rock::api::configuration::Group::Consumer, "Grabbing", "QuickLootPanelPlacement", "2",
    error, sizeof(error));
```
