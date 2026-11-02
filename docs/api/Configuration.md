# Configuration (interface 13, major 2)

Header: `include/ROCK/Configuration.h`. Reads and changes ROCK's INI settings
through ROCK's own catalog, the same way PALM's ROCK pages do.

## Discover and bind

```cpp
const rock::api::configuration::Api* config{};
client.acquire(static_cast<std::uint32_t>(rock::api::configuration::Permission::Read) |
               static_cast<std::uint32_t>(rock::api::configuration::Permission::Write), config);
```

Interface `Configuration`, major 2, minor 0. Major 1 is refused with
`UnsupportedMajor`.

Permissions: `Read` (1) for `revision` and `visit`; `Write` (2) for
`setValue`.

## Functions

| Function | Thread | Result |
| --- | --- | --- |
| `revision(owner, uint64* out)` | any | A number that increases whenever any setting changes, including hot reloads. |
| `visit(owner, group, visitor, user)` | any | Calls `visitor(setting, user)` once per setting of `Group::Consumer` (`ROCK.ini`) or `Group::Developer` (`ROCK_Developer.ini`). `NotReady` before the catalog loads. |
| `setValue(owner, group, section, key, value, error, errorCapacity)` | any | Validates, applies and saves one setting to its owning INI. `InvalidArgument` when rejected; `error` then holds the reason (truncated to `errorCapacity`). |

## Records

`Setting` (borrowed for the duration of one visitor call only): `section`,
`key`, `value`, `defaultValue`, `category`, `description` (UTF-8 strings),
`type` (`ValueType`: `Boolean`, `Integer`, `Float`, `String`), and `overridden` (nonzero
when the value differs from ROCK's compiled default).

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
```
